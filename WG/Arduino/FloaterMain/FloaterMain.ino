/*
  整合性感測器讀取程式 (風速計 + 電池電壓) - 校準版

  功能：
  此程式同時讀取並校準兩個類比感測器：
  1. 風速計：(0-10V 輸出) 透過一個 5:1 的電壓感測模組連接。
  2. 電池電壓：(0-25V) 直接透過一個 5:1 的電壓感測模組連接。

  硬體連接：
  - 風速計 0-10V 訊號 -> [5:1 分壓模組 IN] -> [模組 OUT] -> Arduino A0
  - 電池正極 (+)       -> [5:1 分壓模組 IN] -> [模組 OUT] -> Arduino A1
  - 兩個模組的 GND (-) 都需要連接到 Arduino 的 GND。
*/


//c:\Users\david ho\Desktop\WG\WG\Arduino\FloaterMain\FloaterMain.ino
// =================================================================================
// ### 校準係數設定 (請務必依照您的硬體進行一次性校準) ###
// =================================================================================

// --- 1. 風速計校準 (A0腳位, 0-10V 透過 5:1 分壓模組) ---
//    由於我們無法輕易產生已知風速，所以我們用「已知電壓」來校準。
// a. [建議] 用穩定電源代替風速計，輸入一個【已知電壓】到 A0 的分壓模組 (例如: 9.0V)。
// b. 計算此電壓對應的【理論風速】: (已知電壓 / 10V) * 30m/s (例如: (9.0 / 10) * 30 = 27.0 m/s)。
// c. 上傳程式後，從序列埠讀取 A0 的【ADC 原始值】 (例如，此時讀到: 370)。
// d. 計算係數: 理論風速 / ADC原始值 (27.0 / 370 = 0.07297)。
// e. 將算出的值填入下方。
const float windSpeedCorrectionFactor = 0.07297; // 請替換成【您風速計】的校準值

// --- 2. 電池電壓校準 (A1腳位, 0-25V) ---
// a. 用精準的電錶量測電池的【真實電壓】 (例如: 24.7V)。
// b. 上傳程式後，從序列埠讀取 A1 的【ADC 原始值】 (例如，此時讀到: 1015)。
// c. 計算係數: 真實電壓 / ADC原始值 (24.7 / 1015 = 0.024335)。
// d. 將算出的值填入下方。
const float batteryVoltageCorrectionFactor = 0.0249497; // 請替換成【您電池電壓】的校準值


// ------------------------- 感測器腳位定義 -------------------------
const int windSensorPin = A0;
const int batteryVoltagePin = A1;


void setup() {
  // 初始化序列埠通訊，鮑率設定為 9600
  Serial.begin(9600);
  // 等待序列埠穩定
  delay(100); 
}

void loop() {
  // 1. 分別讀取兩個感測器的原始 ADC 值 (0 - 1023)
  int windAdcValue = analogRead(windSensorPin);
  int batteryAdcValue = analogRead(batteryVoltagePin);

  // 2. 使用各自的校準係數計算出真實的物理量
  float windSpeed_ms = windAdcValue * windSpeedCorrectionFactor;
  float batteryVoltage = batteryAdcValue * batteryVoltageCorrectionFactor;

  // 3. 依照您的 Python 腳本格式，從序列埠輸出結果
  Serial.print("WindSpeed@");
  Serial.println(windSpeed_ms, 2); // 風速印到小數點後兩位

  Serial.print("BatteryVoltage@");
  Serial.println(batteryVoltage, 2); // 電壓印到小數點後兩位

  // 可選：印出原始值以供除錯
  // Serial.print("Raw ADC (Wind, Batt): ");
  // Serial.print(windAdcValue);
  // Serial.print(", ");
  // Serial.println(batteryAdcValue);
  // Serial.println("--------------------");

  delay(1000); 
}