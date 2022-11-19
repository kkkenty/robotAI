// joystickとcmd_velから正確な目標速度、/vel_FBから現在速度を受け取り、ノード内で速度制御する
// LEDのトピックも行う
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
//#include <msgs/Motor.h>
#include <msgs/MotorLED.h>
#include <msgs/PID.h>
#include <geometry_msgs/Twist.h>

// global変数 //
msgs::MotorLED control;
msgs::PID param;
float kp = 25.0, ki = 2.0, kd = 0.1, LIMIT = 0.15, ACC = 1.0; 
double v_goal = 0.0, w_goal = 0.0, kv = 0.458, kw = 1.478, d = 0.155;
int state = 0, ledout = 0, FRIQUENCE = 100;

// クラスの定義 //
class feedback
{
  private:
    float SPEED_SUM = 0.0, SPEED_ACC = 0.0, SPEED_ERROR = 0.0, SPEED_ERROR_PRE = 0.0, SPEED_PRE = 0.0;
  public:
    float PWM = 0.0, SPEED_GOAL = 0.0, SPEED_NOW = 0.0;
    float PID(); // PID制御でPWM計算
    float daikei(); // 台形制御でPWM計算
};
float feedback::PID(){
  SPEED_ERROR = SPEED_GOAL - SPEED_NOW; // P項 or 偏差
  SPEED_SUM += (SPEED_ERROR + SPEED_ERROR_PRE) / 2.0; // I項
  SPEED_ACC = (float)(SPEED_NOW - SPEED_PRE) * (float)FRIQUENCE; // D項
  //ROS_INFO("%lf", SPEED_ACC);
  param.p = kp * SPEED_ERROR;
  param.i = ki * ki * SPEED_SUM;
  param.d = kd * SPEED_ACC;
  PWM = param.p + param.i - param.d;
  SPEED_ERROR_PRE = SPEED_ERROR;
  SPEED_PRE = SPEED_NOW;
  return PWM;
}
float feedback::daikei(){
  SPEED_ERROR = SPEED_GOAL - SPEED_NOW; // P項 or 偏差
  if(SPEED_ERROR > LIMIT){
    PWM += ACC;
  }
  else if(SPEED_ERROR < -LIMIT){
    PWM -= ACC;
  }
  else{ // -LIMIT <= SPEED_ERROR[i] <= LIMIT
  }
  return PWM;
}

// クラスの宣言 //
feedback LEFT, RIGHT;

// CallBack関数 //
void joyCb(const sensor_msgs::Joy &joy_msg)
{
  v_goal = kv * joy_msg.axes[1]; w_goal = kw * joy_msg.axes[3];
  LEFT.SPEED_GOAL  = v_goal - (w_goal * d);
  RIGHT.SPEED_GOAL = v_goal + (w_goal * d);
  if(joy_msg.buttons[0]){
    if(state == 0){
      ROS_INFO("STOPPING!");
      state = 1;
    }
    else if(state == 1){
      ROS_INFO("RESTARTING!");
      state = 0;
    }
  }
}
void navCb(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
  v_goal = cmd_vel->linear.x; w_goal = cmd_vel->angular.z;
  ledout = (int)cmd_vel->linear.z;
  LEFT.SPEED_GOAL  = v_goal - (w_goal * d);
  RIGHT.SPEED_GOAL = v_goal + (w_goal * d);
}
void FBCb(const msgs::MotorLED &vel)
{
  LEFT.SPEED_NOW  = vel.left;
  RIGHT.SPEED_NOW = vel.right;
}

// main関数 //
int main(int argc, char **argv)
{
  ros::init(argc, argv, "controllerLED");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("kv", kv);
  pnh.getParam("kw", kw);
  pnh.getParam("d", d);
  pnh.getParam("kp", kp);
  pnh.getParam("ki", ki);
  pnh.getParam("kd", kd);
  pnh.getParam("LIMIT", LIMIT);
  pnh.getParam("ACC", ACC);
  pnh.getParam("FRIQUENCE", FRIQUENCE);
  ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCb);
  ros::Subscriber nav_sub = nh.subscribe("cmd_vel", 10, navCb);
  ros::Subscriber FB_sub = nh.subscribe("vel_FB", 1, FBCb);
  ros::Publisher ard_pub = nh.advertise<msgs::MotorLED>("power", 1);
  //ros::Publisher scr_pub = nh.advertise<msgs::PID>("param", 1);
  ros::Rate loop_rate(FRIQUENCE);
  
  while (ros::ok())
  {
    control.left  = LEFT.PID();
    control.right = RIGHT.PID();
    // 緊急停止 //
    if(state){
      control.left  = 0.0;
      control.right = 0.0;
      control.led = 0;
    }
    // LEDの出力 //
    if(ledout){
      control.led = 1;
    }
    else{
      control.led = 0;
    }
    // publish //
    ard_pub.publish(control);
    //scr_pub.publish(param);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
