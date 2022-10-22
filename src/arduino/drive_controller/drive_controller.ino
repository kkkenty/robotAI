// Drive用プログラム
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
CytronMD motorTWO(PWM_DIR, PIN_PWM1, PIN_DIR1);
CytronMD motorSIX(PWM_DIR, PIN_PWM2, PIN_DIR2);
CytronMD motorTEN(PWM_DIR, PIN_PWM3, PIN_DIR3); 

ros::NodeHandle nh;
ros::Publisher pub("DrvEncoder", &Sensor);

void powerCb(const msgs::SteerPower& get_msg){
  PWM[OBJECT_TWO] = (int)get_msg.DriveTwo;
  PWM[OBJECT_SIX] = (int)get_msg.DriveSix;
  PWM[OBJECT_TEN] = (int)get_msg.DriveTen;
}

ros::Subscriber<msgs::SteerPower> sub("DrvPower", &powerCb);

void setup() {
  nh.getHardware()->setBaud(115200);
  attachInterrupt(4, counterTWO, CHANGE);
  //attachInterrupt(5, counterTWO, CHANGE);
  attachInterrupt(2, counterSIX, CHANGE);
  //attachInterrupt(3, counterSIX, CHANGE);
  attachInterrupt(1, counterTEN, CHANGE);
  //attachInterrupt(0, counterTEN, CHANGE);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
  PERIOD = 1000.0/FRIQUENCY;  
  PWM[OBJECT_TWO] = 0; PWM[OBJECT_SIX] = 0; PWM[OBJECT_TEN] = 0;
  Sensor.SpeedTwo = 0; Sensor.SpeedSix = 0; Sensor.SpeedTen = 0;
}

void loop() {
  // 現在の速度の取得 //
  enc_two.getSPEED(OBJECT_TWO);
  enc_six.getSPEED(OBJECT_SIX);
  enc_ten.getSPEED(OBJECT_TEN);
  
  Sensor.SpeedTwo = SPEED_NOW[OBJECT_TWO];
  Sensor.SpeedSix = SPEED_NOW[OBJECT_SIX];
  Sensor.SpeedTen = SPEED_NOW[OBJECT_TEN];
  /*
  Serial.print(SPEED_NOW[OBJECT_TWO]); // debug
  Serial.print("  :  ");
  Serial.print(SPEED_NOW[OBJECT_SIX]);
  Serial.print("  :  ");
  Serial.println(SPEED_NOW[OBJECT_TEN]);
  */
  
  Serial.print(value[OBJECT_TWO]); // debug
  Serial.print("  :  ");
  Serial.print(value[OBJECT_SIX]);
  Serial.print("  :  ");
  Serial.println(value[OBJECT_TEN]);
  
  
  // 速度の出力 //
  motorTWO.setSpeed(PWM[OBJECT_TWO]);
  motorSIX.setSpeed(PWM[OBJECT_SIX]);
  motorTEN.setSpeed(PWM[OBJECT_TEN]);
  // ROS処理 //
  pub.publish(&Sensor);
  nh.spinOnce();
  delay(PERIOD);
}
