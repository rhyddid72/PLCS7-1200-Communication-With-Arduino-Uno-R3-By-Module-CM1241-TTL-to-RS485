// UNO + MAX485 + 28BYJ-48 (ULN2003)
// RS485: RO->D8, DI->D9, RE&DE->D4
// Motor: IN1->D10, IN2->D11, IN3->D12, IN4->D13
// Modbus RTU: 9600 8N1, SlaveID=1
// FC06: value 0=STOP, 1=FWD, 2=REV  (addr 0 or 1)

#include <SoftwareSerial.h>
SoftwareSerial RS485(8, 9);
const uint8_t PIN_RE_DE = 4;

// ---- Stepper 28BYJ-48 (half-step) ----
const uint8_t M1 = 10, M2 = 11, M3 = 12, M4 = 13;
const uint8_t stepSeq[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};
volatile int8_t stepIndex = 0;
volatile uint8_t motorMode = 0;          // 0=stop, 1=fwd, 2=rev
unsigned long lastStepMs = 0;
unsigned int  stepIntervalMs = 3;        // chỉnh tốc độ (2..10ms tuỳ nguồn)

void coilsOff() {
  digitalWrite(M1, LOW); digitalWrite(M2, LOW);
  digitalWrite(M3, LOW); digitalWrite(M4, LOW);
}
void applyStep(uint8_t idx) {
  digitalWrite(M1, stepSeq[idx][0]);
  digitalWrite(M2, stepSeq[idx][1]);
  digitalWrite(M3, stepSeq[idx][2]);
  digitalWrite(M4, stepSeq[idx][3]);
}
void motorTask() {
  if (motorMode == 0) { coilsOff(); return; }
  unsigned long now = millis();
  if (now - lastStepMs < stepIntervalMs) return;
  lastStepMs = now;

  if (motorMode == 1) {           // forward
    stepIndex++; if (stepIndex >= 8) stepIndex = 0;
  } else if (motorMode == 2) {    // reverse
    stepIndex--; if (stepIndex < 0) stepIndex = 7;
  }
  applyStep(stepIndex);
}

// ---- Modbus helpers (từ code gốc) ----
uint16_t modbusCRC(const uint8_t *buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i=0;i<len;i++) {
    crc ^= buf[i];
    for (uint8_t b=0;b<8;b++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}
void printHex(const uint8_t *buf, uint8_t n) {
  for (uint8_t i=0;i<n;i++) {
    Serial.print("0x"); if (buf[i] < 16) Serial.print('0');
    Serial.print(buf[i], HEX); Serial.print(i==n-1?"":" ");
  }
  Serial.println();
}
bool readFrame(uint8_t *buf, uint8_t &len, uint16_t gap_ms = 10) {
  uint32_t t0 = millis(); len = 0;
  while (millis() - t0 < gap_ms) {
    while (RS485.available()) {
      if (len < 64) buf[len++] = RS485.read();
      t0 = millis();
    }
  }
  return len >= 4;
}
inline void preTX(){ digitalWrite(PIN_RE_DE, HIGH); }
inline void postTX(){ digitalWrite(PIN_RE_DE, LOW); }

void setup() {
  // RS485
  RS485.begin(9600);   // 8N1
  pinMode(PIN_RE_DE, OUTPUT);
  postTX();

  // Motor pins
  pinMode(M1, OUTPUT); pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT); pinMode(M4, OUTPUT);
  coilsOff();

  Serial.begin(9600);
  Serial.println("UNO+MAX485 Modbus Slave FC06 -> 28BYJ48 (0=stop,1=fwd,2=rev)");
}

void loop() {
  // chạy motor không chặn
  motorTask();

  // nhận Modbus
  uint8_t f[64]; uint8_t n=0;
  if (!readFrame(f, n)) return;
  if (n < 8) return;

  // CRC
  uint16_t crcRecv = f[n-2] | (f[n-1] << 8);
  uint16_t crcCalc = modbusCRC(f, n-2);
  if (crcCalc != crcRecv) return;

  uint8_t slave = f[0], fc = f[1];
  if (slave != 1) return;               // chỉ nhận ID=1

  // ---- FC06 Write Single Register ----
  if (fc == 0x06 && n >= 8) {
    uint16_t addr = (f[2] << 8) | f[3];   // big-endian
    uint16_t val  = (f[4] << 8) | f[5];

    // chấp nhận addr 0 hoặc 1 (tuỳ cấu hình Siemens)
    if (addr == 0 || addr == 1) {
      if (val == 0) { motorMode = 0; coilsOff(); }
      else if (val == 1) { motorMode = 1; }
      else if (val == 2) { motorMode = 2; }
      // giá trị khác: bỏ qua, vẫn giữ trạng thái trước
      Serial.print("CMD: "); Serial.println(val);
    }

    // *** Trả lời echo để PLC DONE=1 ***
    uint8_t resp[8];
    memcpy(resp, f, 6);
    uint16_t crc = modbusCRC(resp, 6);
    resp[6] = crc & 0xFF; resp[7] = crc >> 8;
    preTX(); RS485.write(resp, 8); RS485.flush(); postTX();
  }

  // (nếu muốn, bạn có thể bổ sung FC03 để PLC đọc trạng thái hiện tại)
}
