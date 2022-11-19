// Arduinoからエンコーダ情報を得て、計算したオドメータ情報をtf,/odom,/vel_FBに配信
// LED改良版
#include <ros/ros.h>
#include <msgs/MotorLED.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <math.h>

// global変数 //
int FRIQUENCE = 100;
const double pi = 3.14159265;
double d = 0.155;
msgs::MotorLED vel, acc;
nav_msgs::Odometry odom;

// 関数、Cb関数の定義 //
geometry_msgs::Pose setPose(float x, float y, float yaw){
  geometry_msgs::Pose output;
  output.position.x = x;
  output.position.y = y;
  output.position.z = 0.0;
  output.orientation.x = 0.0;
  output.orientation.y = 0.0;
  output.orientation.z = sin(yaw / 2.0);
  output.orientation.w = cos(yaw / 2.0);
  return output;
}
void getvel(msgs::MotorLED fake_vel){ // 正確なv,wを取得
  vel.left = 0.12 * pi * 1000 / 3292 * fake_vel.left;
  vel.right = 0.12 * pi * 1000 / 3292 * fake_vel.right;
}

// クラスの定義 //
class tf_odom
{
  private:
    double x = 0.0, y = 0.0, th = 0.0, dt = 0.0, V = 0.0, W = 0.0, preV = 0.0, preW = 0.0;
    tf::TransformBroadcaster br;
    tf::Transform tf_base, tf_laser;
    tf::Quaternion q_base, q_laser;
    ros::Time ros_pre = ros::Time::now();
  public:
    void get_odom(); // odom情報を計算、配信
};
void tf_odom::get_odom()
{  
  ros::Time ros_now = ros::Time::now();
  dt = ros_now.toSec() - ros_pre.toSec();
  //ROS_INFO("ROS::Time: %lf", dt);
  
  V = (vel.left + vel.right) / 2.0;
  W = (vel.right - vel.left) / 2.0 / d;
  x += V * dt * cos(th);
  y += V * dt * sin(th);
  th += W * dt;
  acc.left = (V - preV) / dt;
  acc.right = (W - preW) / dt;
  //ROS_INFO("x:%lf, y:%lf, th:%lf", x, y, th);
  
  // tf message
  tf_base.setOrigin(tf::Vector3(x, y, 0.0));
  q_base.setRPY(0, 0, th);
  tf_base.setRotation(q_base);
  tf_laser.setOrigin(tf::Vector3(-0.07, -0.07, 0.085));
  q_laser.setRPY(0, 0, pi);
  tf_laser.setRotation(q_laser);
  
  br.sendTransform(tf::StampedTransform(tf_base, ros_now, "odom", "base_link"));
  br.sendTransform(tf::StampedTransform(tf_laser, ros_now, "base_link", "base_laser"));
  
  // odometry message
  odom.header.stamp = ros_now;
  odom.header.frame_id = "odom";
  odom.pose.pose = setPose(x, y, th);
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = V;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = W;
  
  ros_pre = ros_now;
  preV = V;
  preW = W;
}

// main関数 //
int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometryLED");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("d", d);
  pnh.getParam("FRIQUENCE", FRIQUENCE);
  tf_odom robot;
  vel.left = 0.0; vel.right = 0.0; acc.left = 0.0; acc.right = 0.0;// inisialize
  ros::Subscriber vel_sub = nh.subscribe("encoder", 1, getvel);
  ros::Publisher odm_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Publisher vel_pub = nh.advertise<msgs::MotorLED>("vel_FB", 1);
  //ros::Publisher acc_pub = nh.advertise<msgs::MotorLED>("acc", 1);
  ros::Rate loop_rate(FRIQUENCE);

  while (ros::ok())
  {
    robot.get_odom();
    odm_pub.publish(odom);
    vel_pub.publish(vel);
    //acc_pub.publish(acc);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
