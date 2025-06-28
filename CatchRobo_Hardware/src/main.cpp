#include <M5Stack.h>
#include <math.h>

// サーボ設定
const byte servoID = 1;
const int centerPosition = 2048;
const float maxAmplitude = 1600.0;
const float radiansIncrement = 0.06;
float radiansVal = 0.0;

// UART2 設定
HardwareSerial SerialSTS(2); // UART2

void setup() {
  M5.begin();
  Serial.begin(115200);
  SerialSTS.begin(1000000, SERIAL_8N1, 36, 26); // TX=26, RX=36

  delay(200);
  Serial.println("サーボスイング制御開始（UART2, 1Mbps）");

  // 画面初期化
  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setCursor(40, 20);
  M5.Lcd.print("Servo Swing Monitor");
}

void sts_moveToPos(byte id, int position) {
  byte packet[13];

  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  packet[3] = 9;
  packet[4] = 3;
  packet[5] = 42;
  packet[6] = position & 0xFF;
  packet[7] = (position >> 8) & 0xFF;
  packet[8] = 0x00;
  packet[9] = 0x00;
  packet[10] = 0x00;
  packet[11] = 0x00;

  byte checksum = 0;
  for (int i = 2; i < 12; i++) checksum += packet[i];
  packet[12] = ~checksum;

  for (int i = 0; i < 13; i++) SerialSTS.write(packet[i]);
}

void loop() {
  radiansVal += radiansIncrement;
  if (radiansVal > TWO_PI) radiansVal -= TWO_PI;

  int targetPos = centerPosition + int(sin(radiansVal) * maxAmplitude);
  sts_moveToPos(servoID, targetPos);

  // デバッグ出力
  Serial.printf("Target Pos: %d\n", targetPos);

  // ディスプレイに表示
  M5.Lcd.fillRect(40, 60, 240, 40, BLACK);  // 前の値を消去
  M5.Lcd.setCursor(40, 60);
  M5.Lcd.printf("Pos: %d", targetPos);

  delay(20);
}