// joystickを使って、PWM値を配信する
// 左右のstickで1ユニットのモータを動かす
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <msgs/3Steer.h>

msgs::3Steer PWM;
const float MAX_L = 255.0, MAX_R = 255.0;
int FRIQUENCE = 100;
int unit = 0;
int i;

void joy_callback(const sensor_msgs::Joy &joy_msg)
{
  for(i=1;i<5;i++){
      if(joy_msg.button[i] == 1){
          unit = i;
          break;
      }
  }
  PWM.left  = MAX_L * joy_msg.axes[1];
  PWM.right = MAX_R * joy_msg.axes[2];
  if(joy_msg.buttons[0]){
    PWM.left = 0.0; PWM.right = 0.0;
  }
  //ROS_INFO("PWM.L:%lf", (int)PWM.L);
  //ROS_INFO("PWM.R:%lf", (int)PWM.R);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("MAX_L", MAX_L);
  pnh.getParam("MAX_R", MAX_R);
  pnh.getParam("FRIQUENCE", FRIQUENCE);
  ros::Subscriber sub = nh.subscribe("joy", 10, joy_callback);
  ros::Publisher pub = nh.advertise<msgs::Motor>("power", 1);
  ros::Rate loop_rate(FRIQUENCE);

  while (ros::ok())
  {
    pub.publish(PWM);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}