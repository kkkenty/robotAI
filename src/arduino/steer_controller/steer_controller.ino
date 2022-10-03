// Steer用プログラム
// value = 55 / 22 * ステアの回転 * 2048 * 4 = 20480 * ステアの回転
#include "motor_control.h"
#include <ros.h>
#include "msgs/SteerSensor.h"
const float FRIQUENCY = 50.0;
float PERIOD;
msgs::SteerSensor SPEED;
encoder enc_two(PIN_A_TWO, PIN_B_TWO);
encoder enc_six(PIN_A_SIX, PIN_B_SIX);
encoder enc_ten(PIN_A_TEN, PIN_B_TEN);

void setup() {
  Serial.begin(57600);
  attachInterrupt(5, counterTWO, CHANGE);
  attachInterrupt(4, counterTWO, CHANGE);
  attachInterrupt(3, counterTEN, CHANGE);
  attachInterrupt(2, counterTEN, CHANGE);
  attachInterrupt(0, counterSIX, CHANGE);
  attachInterrupt(1, counterSIX, CHANGE);
  PERIOD = 1000.0/FRIQUENCY;  
}

void loop() {
  // 現在の速度の取得 //
  enc_two.getSPEED(OBJECT_TWO);
  enc_six.getSPEED(OBJECT_SIX);
  enc_ten.getSPEED(OBJECT_TEN);
  /* 
  Serial.print(PULSE_NOW[OBJECT_TWO]); // debug
  Serial.print("  :  ");
  Serial.print(PULSE_NOW[OBJECT_SIX]);
  Serial.print("  :  ");
  Serial.println(PULSE_NOW[OBJECT_TEN]);
  */
  delay(PERIOD);
}
