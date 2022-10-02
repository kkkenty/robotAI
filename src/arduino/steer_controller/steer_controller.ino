// Steer用プログラム
#include "motor_control.h"
const float FRIQUENCY = 100.0;
encoder enc_two(PIN_A_TWO, PIN_B_TWO);
encoder enc_six(PIN_A_SIX, PIN_B_SIX);
encoder enc_ten(PIN_A_TEN, PIN_B_TEN);

void setup() {
  Serial.begin(57600);
  attachInterrupt(5, counter0, CHANGE);
  attachInterrupt(4, counter0, CHANGE);
  attachInterrupt(3, counter0, CHANGE);
  attachInterrupt(2, counter0, CHANGE);
  attachInterrupt(0, counter1, CHANGE);
  attachInterrupt(1, counter1, CHANGE);
  PERIOD = 1000.0/FRIQUENCY;  
}

void loop() {
  // 現在の速度の取得 //
  enc_two.getSPEED(OBJECT_TWO);
  enc_six.getSPEED(OBJECT_SIX);
  enc_ten.getSPEED(OBJECT_TEN);
  serial.print(SEPPD_NOW[OBJECT_TWO]);
  serial.print("  :  ");
  serial.print(SEPPD_NOW[OBJECT_SIX]);
  serial.print("  :  ");
  serial.println(SEPPD_NOW[OBJECT_TEN]);
  
  delay(PERIOD);
}
