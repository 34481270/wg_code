#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include "MS5837.h"

// 腳位定義
// #define RS485_DE     7
// #define RS485_RE     6
#define PROP_PIN     7
#define CS_PIN       2

# define motorResolution  3000
# define reductionRatio   50

// 安全停機時間 (ms)
const unsigned long TIMEOUT_MS = 3000;

// 全域物件與變數
MS5837 sensor;
Servo propeller;
char buf[128];
uint8_t idx = 0;
unsigned long lastCmd = 0;
float offset = 0;
bool rudderAlarm = 0;
long preRudder = 0;
int prePropeller = 0;

uint8_t SPI_T (uint8_t msg)    //Repetive SPI transmit sequence
{
   uint8_t msg_temp = 0;  //vairable to hold recieved data
   digitalWrite(CS_PIN,LOW);     //select spi device
   msg_temp = SPI.transfer(msg);    //send and recieve
   digitalWrite(CS_PIN,HIGH);    //deselect spi device
   return(msg_temp);      //return recieved byte
}

float readEncoderDeg() {
  uint8_t recieved = 0xA5;  // temp variable
  uint16_t ABSposition = 0;
  uint8_t temp[2];

  digitalWrite(CS_PIN, LOW);  // 啟動晶片選擇

  SPI_T(0x10);                 // 發送讀取指令
  recieved = SPI_T(0x00);      // 發送 NOP，檢查編碼器是否準備好

  // 等待編碼器準備好（加入逾時避免 blocking）
  // 每次 2ms，最多 25 次 ≈ 50ms
  uint8_t tries = 0;
  while (recieved != 0x10 && tries < 25) {   // FIX: 加入逾時
    recieved = SPI_T(0x00);
    delay(2);
    tries++;
  }

  if (recieved != 0x10) { // 逾時保護：收尾並回傳 NAN
    digitalWrite(CS_PIN, HIGH);
    return NAN;
  }

  temp[0] = SPI_T(0x00);  // 接收 MSB
  temp[1] = SPI_T(0x00);  // 接收 LSB

  digitalWrite(CS_PIN, HIGH);  // 停止晶片選擇

  temp[0] &= ~0xF0;  // 清除前4位無效資料
  ABSposition = (temp[0] << 8) + temp[1];  // 合併成 12-bit 資料

  float deg = ABSposition * 0.08789;  // 轉換成角度（360 / 4096 ≈ 0.08789）
  return deg;
}

void motorCmd(float turningAngle){
  long cmd = (long)reductionRatio * (long)motorResolution * turningAngle / 360.0;
  float tmp = readEncoderDeg();
  Serial.println(tmp);
  Serial.print("p=");
  Serial.println(cmd);

  Serial1.print("p=");
  Serial1.println(cmd);
  Serial1.println("^");
}

// 讀壓力並回覆
void replyPressure() {
  sensor.read();
  float d = sensor.depth();  // m

  Serial3.print("pressure@");
  Serial3.println(d, 2);
  Serial3.flush();
  delayMicroseconds(200);
}

bool validateChecksum(String packet){
  if(!packet.startsWith("$")){
    return false;
  }
  int starIndex = packet.indexOf('*');
  if (starIndex == -1 || starIndex < 2 ){
    return false;
  }
  String payload = packet.substring(1, starIndex);
  String checksumStr = packet.substring(starIndex + 1);
  checksumStr.trim();  // 移除前後空白與 \r \n

  byte checksum = 0;
  for(int i = 0; i < payload.length(); i++){
    checksum ^= payload[i];
  }
  byte receivedChecksum = strtoul(checksumStr.c_str(), NULL, 16);

  return checksum == receivedChecksum;
}

// 解析 "cmd@val" 並執行
void handleCommand(char* s) {
  // 1) 如果是自己剛剛回的 pressure，就跳過
  if (strncmp(s, "pressure@", 9) == 0) {
    return;
  }

  // 3) 拆命令 & 參數
  char* at = strchr(s, '@');
  if (!at) return;
  *at = '\0';
  char* cmd = s;
  char* arg = at + 1;

  // 先處理 rudderAlarm（避免被 if(rudderAlarm==0) 擋住）
  if (strcmp(cmd, "rudderAlarm") == 0) {        // FIX: 移到最前面
    // 支援 rudderAlarm@1 開啟、rudderAlarm@0 關閉
    rudderAlarm = atoi(arg) != 0;
    Serial.print("rudderAlarm = ");
    Serial.println(rudderAlarm ? 1 : 0);
    lastCmd = millis();
    replyPressure();
    return;
  }

  if(rudderAlarm == 0)
  {
    if (strcmp(cmd, "propeller") == 0) {
      float p = constrain(atof(arg), 0, 100);
      int pwm = map((int)p, 0, 100, 1500, 1800);

      if (abs(pwm - prePropeller) >= 5){
        Serial.print("送出的PWM訊號 : ");
        Serial.println(pwm);
        prePropeller = pwm;
        propeller.writeMicroseconds(pwm);
      }

    }
    else if (strcmp(cmd, "rudder") == 0) {
      long deg = atol(arg);
      if (abs(deg - preRudder) >= 1){
        motorCmd(-deg);
        Serial.print("馬達角度：");
        Serial.println(-deg);
        preRudder = deg;
      }
    }
  }

  // 5) 更新計時 & 回壓力
  lastCmd = millis();
  replyPressure();
}


void setup() {
  delay(2000);
  Serial.begin(9600);
  Serial3.begin(9600);
  Serial1.begin(38400);

  pinMode(CS_PIN, OUTPUT);

  Wire.begin();
  int counter = 0;
  while (!sensor.init()) {
    Serial.println("Sensor init failed");
    if(counter >= 5)
    {
      break;
    }
    counter = counter + 1;
    delay(2000);
  }
  sensor.setFluidDensity(1029);

  propeller.attach(PROP_PIN, 1500, 1800);
  propeller.writeMicroseconds(1500);

  digitalWrite(CS_PIN,HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV32);

  // 馬達歸0
  // SPI_T(0x70); // Encoder歸零
  float deg = readEncoderDeg();
  Serial.println(deg);

  if (deg >= 0  && deg <= 60) {
    deg = -deg; // 返回負值
  } else if(deg >= 300) {
    deg = 360 - deg;
  }

  Serial.print("解碼器角度： ");
  Serial.println(-deg);

  motorCmd(deg);
  Serial.println("--------------");
  delay(5000);
  Serial1.println("|2");

  deg = readEncoderDeg();
  Serial.println(deg);
  Serial.print("轉動後解碼器角度： ");
  Serial.println(deg);

  lastCmd = millis();
}

void loop() {
  // 接收 RS-485
  while (Serial3.available()) {
    char c = Serial3.read();
    if (c == '\r')      continue;
    if (c == '\n') {
      buf[idx] = '\0';
      if (idx > 0) {
        Serial.println(String(buf));
        if(validateChecksum(String(buf))){
          handleCommand(buf + 1);
        }
      }
      idx = 0;
    } else if (idx + 1 < sizeof(buf)) {
      buf[idx++] = c;
    }
  }

  // 安全停機
  if (millis() - lastCmd > TIMEOUT_MS) {
    propeller.writeMicroseconds(1500);
    lastCmd = millis();
    Serial.println("[TIMEOUT] No command, propeller stopped");
  }

  // 原本用 while + 單一 & 會卡住主迴圈；改成一次判斷的 if，且用 &&
  float enc = readEncoderDeg();                                  // FIX: 非阻塞檢查
  if (!isnan(enc) && enc >= 90 && enc <= 270) {                  // FIX: && 且去除 while
    if (!rudderAlarm) {
      rudderAlarm = 1;
      Serial1.println("]");
      Serial3.print("rudderAlarm@");
      Serial3.println("1");
      Serial3.flush();
      Serial.println("rudderAlarm triggered");
    }
  }
}
