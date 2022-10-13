// Steer用プログラム
// value = 55 / 22 * ステアの回転 * 2048 * 4 = 20480 * ステアの回転
#include "motor_control.h"
#include <ros.h>
#include "msgs/SteerSensor.h"
#include "msgs/SteerPower.h"
#include "CytronMotorDriver.h"
const float FRIQUENCY = 100.0;
float PERIOD; // delayの引数[ms]
int PWM[] = {0, 0, 0}, i;
msgs::SteerSensor Sensor;
encoder enc_two(PIN_A_TWO, PIN_B_TWO);
encoder enc_six(PIN_A_SIX, PIN_B_SIX);
encoder enc_ten(PIN_A_TEN, PIN_B_TEN);
CytronMD motorTWO(PWM_PWM, PIN_1A, PIN_1B);
CytronMD motorSIX(PWM_PWM, PIN_2A, PIN_2B); 
CytronMD motorTEN(PWM_PWM, PIN_3A, PIN_3B); 

ros::NodeHandle nh;
ros::Publisher pub("StrEncoder", &Sensor);

void powerCb(const msgs::SteerPower& get_msg){
  PWM[OBJECT_TWO] = (int)get_msg.SteerTwo;
  PWM[OBJECT_SIX] = (int)get_msg.SteerSix;
  PWM[OBJECT_TEN] = (int)get_msg.SteerTen;
}

ros::Subscriber<msgs::SteerPower> sub("StrPower", &powerCb);

void setup() {
  nh.getHardware()->setBaud(57600);
  attachInterrupt(5, counterTWO, CHANGE);
  attachInterrupt(4, counterTWO, CHANGE);
  attachInterrupt(3, counterTEN, CHANGE);
  attachInterrupt(2, counterTEN, CHANGE);
  attachInterrupt(0, counterSIX, CHANGE);
  attachInterrupt(1, counterSIX, CHANGE);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
  PERIOD = 1000.0/FRIQUENCY;  
  PWM[OBJECT_TWO] = 0; PWM[OBJECT_SIX] = 0; PWM[OBJECT_TEN] = 0;
  Sensor.PulseTwo = 0; Sensor.PulseSix = 0; Sensor.PulseTen = 0;
}

void loop() {
  // 現在の回転数の取得 //
  enc_two.getPULSE(OBJECT_TWO);
  enc_six.getPULSE(OBJECT_SIX);
  enc_ten.getPULSE(OBJECT_TEN);
  
  Sensor.PulseTwo = PULSE_NOW[OBJECT_TWO];
  Sensor.PulseSix = PULSE_NOW[OBJECT_SIX];
  Sensor.PulseTen = PULSE_NOW[OBJECT_TEN];
  /*
  Serial.print(PULSE_NOW[OBJECT_TWO]); // debug
  Serial.print("  :  ");
  Serial.print(PULSE_NOW[OBJECT_SIX]);
  Serial.print("  :  ");
  Serial.println(PULSE_NOW[OBJECT_TEN]);
  */
  // 速度の出力 //
  motorTWO.setSpeed(PWM[OBJECT_TWO]);
  motorSIX.setSpeed(PWM[OBJECT_SIX]);
  motorTEN.setSpeed(PWM[OBJECT_TEN]);
  // ROS処理 //
  pub.publish(&Sensor);
  nh.spinOnce();
  delay(PERIOD);
}
