#include <SPI.h>
#include <BluetoothSerial.h>

// ===== Pins (VSPI) =====
static const int PIN_SCK   = 18;
static const int PIN_MISO  = 19;
static const int PIN_MOSI  = 23;
static const int PIN_CS    = 5;
static const int PIN_READY = 13;   // <- SLAVE GPIO4

static const uint32_t SPI_HZ = 4000000; // 4 MHz (можна 8+ при коротких дротах)

// -------- UART до root-ноди --------
static const int UART_ROOT_RX = 16;   // MASTER RX  <- TX root
static const int UART_ROOT_TX = 17;   // MASTER TX  -> RX root
HardwareSerial RootSerial(2);         // Serial2

BluetoothSerial SerialBT;

// ===== 40B frame =====
static const uint8_t FRAME_SIZE = 40;
static const uint8_t MAGIC      = 0xA5;
enum : uint8_t { T_NOP=0, T_TEXT=1, T_DOUBLE=2 };

// Розклад 40B: [0]MAGIC [1]TYPE [2..3]SEQ [4]LEN(0..32) [5..36]DATA [37]CHK(XOR0..36) [38..39]ACK(lo,hi)
uint8_t  tx_frame[FRAME_SIZE];
uint8_t  rx_frame[FRAME_SIZE];

// ---- Ввід із BT/USB по рядках ----
static char    bt_buf[32];  static uint8_t bt_len  = 0;
static char    usb_buf[32]; static uint8_t usb_len = 0;
static char    root_buf[32];
static uint8_t root_len = 0;

char   pending_text[33] = {0};  // 32 + '\0'
bool   have_pending = false;
bool   doubleMode   = false;
double pending_double = 0.0;

volatile bool bt_connected = false;
bool hello_sent = false;

// READY interrupt
volatile bool readyFlag = false;
void IRAM_ATTR readyISR(){ readyFlag = true; }

// ===== ACK/SEQ (stop-and-wait) =====
uint16_t tx_seq = 0;                 // MASTER->SLAVE
uint16_t inflight_seq = 0;
bool     awaiting_ack = false;
uint8_t  inflight_frame[FRAME_SIZE];

uint16_t rx_last_ok = 0;             // останній коректно прийнятий SEQ від SLAVE (ACK назад)
uint16_t rx_last_delivered = 0;      // антидубль SLAVE->MASTER

// ===== utils =====
static inline uint8_t checksum37(const uint8_t* f){ uint8_t c=0; for(int i=0;i<=36;i++) c^=f[i]; return c; }
static inline uint8_t safeLen32(const char* s){ uint8_t n=0; while (n<32 && s[n]) n++; return n; }

void rootPump() {
  while (RootSerial.available()) {
    char c = (char)RootSerial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      if (root_len == 0) break;
      root_buf[root_len] = 0;

      // Від root -> в BT і в USB-лог
      if (bt_connected) {
        SerialBT.write((const uint8_t*)root_buf, root_len);
        SerialBT.write('\n');
        SerialBT.flush();
      }
      Serial.print("[ROOT->PC] ");
      Serial.println(root_buf);

      root_len = 0;
      break;
    }

    if (root_len < sizeof(root_buf) - 1) {
      root_buf[root_len++] = c;
    } else {
      // overflow: просто відправляємо те, що є
      root_buf[root_len] = 0;
      if (bt_connected) {
        SerialBT.write((const uint8_t*)root_buf, root_len);
        SerialBT.write('\n');
        SerialBT.flush();
      }
      Serial.print("[ROOT->PC OVF] ");
      Serial.println(root_buf);
      root_len = 0;
      break;
    }
  }
}


void buildFrame(uint8_t type, uint16_t seq, const uint8_t* data, uint8_t len, uint16_t ackseq){
  if (len>32) len=32;
  memset(tx_frame,0,FRAME_SIZE);
  tx_frame[0]=MAGIC; tx_frame[1]=type;
  tx_frame[2]=(uint8_t)(seq & 0xFF);
  tx_frame[3]=(uint8_t)(seq >> 8);
  tx_frame[4]=len;
  if (len) memcpy(&tx_frame[5], data, len);
  tx_frame[37]=checksum37(tx_frame);
  tx_frame[38]=(uint8_t)(ackseq & 0xFF);
  tx_frame[39]=(uint8_t)(ackseq >> 8);
}
bool parseFrame(const uint8_t* f, uint8_t& type, uint16_t& seq, uint8_t* data, uint8_t& len, uint16_t& ackseq){
  if (f[0]!=MAGIC) return false;
  if (checksum37(f)!=f[37]) return false;
  type=f[1];
  seq   = (uint16_t)f[2] | ((uint16_t)f[3]<<8);
  len   = f[4]; if (len>32) len=32;
  if (data && len) memcpy(data,&f[5],len);
  ackseq= (uint16_t)f[38] | ((uint16_t)f[39]<<8);
  return true;
}
void makeNOP(uint16_t ackseq)                { buildFrame(T_NOP,    tx_seq, nullptr, 0, ackseq); }
void makeText(const char* s, uint16_t ackseq){ buildFrame(T_TEXT,   ++tx_seq, (const uint8_t*)s, safeLen32(s), ackseq); }
void makeDouble(double v, uint16_t ackseq)   { uint8_t b[8]; memcpy(b,&v,8); buildFrame(T_DOUBLE, ++tx_seq, b, 8, ackseq); }

// BT callback
void bt_cb(esp_spp_cb_event_t e, esp_spp_cb_param_t*){
  if (e==ESP_SPP_SRV_OPEN_EVT){ bt_connected = true;  hello_sent = false; }
  if (e==ESP_SPP_CLOSE_EVT)   { bt_connected = false; hello_sent = false; }
}

// --- byte pumps (newline framed) ---
void btPump() {
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c=='\r') continue;
    if (c=='\n') {
      if (bt_len==0) break;
      bt_buf[bt_len]=0;
      if (bt_len>=2 && bt_buf[0]=='d' && bt_buf[1]==' ') { pending_double=atof(bt_buf+2); doubleMode=true; }
      else { memcpy(pending_text, bt_buf, min((int)sizeof(pending_text),(int)bt_len+1)); doubleMode=false; }
      bt_len=0; have_pending=true; break;
    }
    if (bt_len < sizeof(bt_buf)-1) bt_buf[bt_len++]=c;
    else { 
      bt_buf[bt_len]=0;
      memcpy(pending_text, bt_buf, sizeof(pending_text));
              // Додатково відправити рядок в root по UART
      RootSerial.write((const uint8_t*)bt_buf, bt_len);
      RootSerial.write('\n');
      RootSerial.flush();

      bt_len=0; 
      doubleMode=false;
      have_pending=true;
      break; }
  }
}
void usbPump() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c=='\r') continue;
    if (c=='\n') {
      if (usb_len==0) break;
      usb_buf[usb_len]=0;
      if (usb_len>=2 && usb_buf[0]=='d' && usb_buf[1]==' ') { pending_double=atof(usb_buf+2); doubleMode=true; }
      else { memcpy(pending_text, usb_buf, min((int)sizeof(pending_text),(int)usb_len+1)); doubleMode=false; }
      usb_len=0; have_pending=true; break;
    }
    if (usb_len < sizeof(usb_buf)-1) usb_len++;
    else { 
      usb_buf[usb_len]=0; 
      memcpy(pending_text, usb_buf, sizeof(pending_text)); 

      RootSerial.write((const uint8_t*)usb_buf, usb_len);
      RootSerial.write('\n');
      RootSerial.flush();

      usb_len=0; 
      doubleMode=false; 
      have_pending=true; 
      break; }
  }
}

// --- ACK-aware TX frame prep ---
void prepareTxFrameWithAck(){
  const uint16_t ack_to_send = rx_last_ok;

  if (awaiting_ack){
    // повтор: inflight + оновити ACK
    memcpy(tx_frame, inflight_frame, FRAME_SIZE);
    tx_frame[38]=(uint8_t)(ack_to_send & 0xFF);
    tx_frame[39]=(uint8_t)(ack_to_send >> 8);
  } else {
    // новий кадр або NOP
    if (have_pending){
      if (doubleMode) makeDouble(pending_double, ack_to_send);
      else            makeText(pending_text,    ack_to_send);
      memcpy(inflight_frame, tx_frame, FRAME_SIZE);
      inflight_seq  = (uint16_t)tx_frame[2] | ((uint16_t)tx_frame[3]<<8);
      awaiting_ack  = (tx_frame[1]!=T_NOP || tx_frame[4]!=0);
      have_pending  = false;
    } else {
      makeNOP(ack_to_send);
      awaiting_ack = false; // NOP не вимагає ACK
    }
  }
}

void spiExchange40(){
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  for (uint8_t i=0;i<FRAME_SIZE;i++) rx_frame[i]=SPI.transfer(tx_frame[i]);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

void handleRx(){
  uint8_t type,len,data[32]; uint16_t rseq,rack;
  if (!parseFrame(rx_frame,type,rseq,data,len,rack)) return;

  // підтвердження нашого кадра
  if (awaiting_ack && rack == inflight_seq){
    awaiting_ack = false;
  }

  // антидубль прийому
  if (rseq == rx_last_delivered) return;
  rx_last_delivered = rseq;
  rx_last_ok        = rseq;

  if (type==T_TEXT && len){
    String s; s.reserve(len);
    for(uint8_t i=0;i<len;i++) s+=(char)data[i];
    // обрізати краєві пробіли/CR/LF — на всяк випадок
    while (s.length() && (s[s.length()-1]=='\n'||s[s.length()-1]=='\r'||s[s.length()-1]==' ')) s.remove(s.length()-1);
    if (s.length()){
    // в BT
      if (bt_connected) {
        SerialBT.write((const uint8_t*)s.c_str(), s.length());
        SerialBT.write('\n');
        SerialBT.flush();
      }
      // в root по UART
      RootSerial.write((const uint8_t*)s.c_str(), s.length());
      RootSerial.write('\n');
      RootSerial.flush();

      // дебаг в USB
      Serial.print("[SPI->] ");
      Serial.println(s);
    }

  } else if (type==T_DOUBLE && len==8){
    double dv; memcpy(&dv,data,8);
    // при потребі: SerialBT.printf("d %.10f\n", dv);
  }
}

void setup(){
  Serial.begin(115200);
  Serial.printf("ROLE: MASTER, MAC:%llX\n", ESP.getEfuseMac());

  pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  pinMode(PIN_READY, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PIN_READY), readyISR, RISING);

  RootSerial.begin(115200, SERIAL_8N1, UART_ROOT_RX, UART_ROOT_TX);
  Serial.println("Root UART started 115200 on RX=16, TX=17");

  SerialBT.begin("Kennet'MASH'");
  SerialBT.register_callback(bt_cb);

  Serial.println("MASTER: BT+SPI (ACK, 40B) готовий.");
}

void loop(){
  if (SerialBT.hasClient() && bt_connected && !hello_sent){
    SerialBT.print("hello\n");
    hello_sent = true;
  }

  btPump();
  usbPump();
  rootPump();     

  // Працюємо, коли є дані до відправки або SLAVE підняв READY
  if (have_pending || readyFlag || digitalRead(PIN_READY)==HIGH) {
    do {
      readyFlag=false;
      prepareTxFrameWithAck();
      spiExchange40();
      handleRx();
      delayMicroseconds(100); // дати SLAVE поставити наступну DMA чергу
    } while (digitalRead(PIN_READY)==HIGH);
    // не лишаємо сміття у tx_frame між циклами
    memset(tx_frame,0,FRAME_SIZE);
  }
}
