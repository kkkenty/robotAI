// joystickを使って、PWM値を配信する
// 左右のstickで1ユニットのモータを動かす
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <msgs/Steer.h>

msgs::Steer PWM;
float MAX_Drive = 255.0, MAX_Steer = 255.0;
int FRIQUENCE = 100;
int unit = 2;
int i;

void joy_callback(const sensor_msgs::Joy &joy_msg)
{
  // 初期化
  PWM.DriveTen = 0; PWM.SteerTen = 0;
  PWM.DriveSix = 0; PWM.SteerSix = 0;
  PWM.DriveTwo = 0; PWM.SteerTwo = 0;
  for(i=1;i<5;i++){
      if(joy_msg.buttons[i-1] == 1){
          unit = i; // ここでunitの値が変わらない限り、PWMは変わらない
          break;
      }
  }
  switch(unit){
    case 1:
      PWM.DriveTen = MAX_Drive * joy_msg.axes[1];
      PWM.SteerTen = MAX_Steer * joy_msg.axes[2];
      break;
    case 3:
      PWM.DriveSix = MAX_Drive * joy_msg.axes[1];
      PWM.SteerSix = MAX_Steer * joy_msg.axes[2];
      break;
    case 4:
      PWM.DriveTwo = MAX_Drive * joy_msg.axes[1];
      PWM.SteerTwo = MAX_Steer * joy_msg.axes[2];
      break;
    default:
      break;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("MAX_Drive", MAX_Drive);
  pnh.getParam("MAX_Steer", MAX_Steer);
  pnh.getParam("FRIQUENCE", FRIQUENCE);
  ros::Subscriber sub = nh.subscribe("joy", 10, joy_callback);
  ros::Publisher pub = nh.advertise<msgs::Steer>("pwm", 10);
  ros::Rate loop_rate(FRIQUENCE);

  while (ros::ok())
  {
    pub.publish(PWM);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}