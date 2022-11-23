// Arduinoからエンコーダ情報を得て、計算したオドメータ情報をtf,/odom,/angvelに配信
#include <ros/ros.h>
#include <msgs/SteerSensor.h>
#include <msgs/SteerPower.h>
#include <msgs/SteerOdometry.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#define deg_to_rad(deg) ((deg)/180*M_PI)
#define rad_to_deg(rad) ((rad)/M_PI*180)

int FRIQUENCY = 100;
int STRRESOLUTION = 10240, DRVRESOLUTION = 480, DRVFRIQ = 40;
double DIAMETER = 0.133414;
msgs::SteerOdometry Now;
nav_msgs::Odometry odom;

geometry_msgs::Pose setPose(float x, float y, float yaw)
{
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
void StrArdCb(const msgs::SteerSensor &Ardmsg)
{
    Now.AngleTwo = -(float) Ardmsg.PulseTwo / STRRESOLUTION * M_PI;
    Now.AngleSix = -(float) Ardmsg.PulseSix / STRRESOLUTION * M_PI;
    Now.AngleTen = -(float) Ardmsg.PulseTen / STRRESOLUTION * M_PI;
}
void DrvArdCb(const msgs::SteerSensor &Ardmsg)
{
    Now.SpeedTwo = Ardmsg.SpeedTwo / (float)DRVRESOLUTION * M_PI * DIAMETER * (float)DRVFRIQ;
    Now.SpeedSix = Ardmsg.SpeedSix / (float)DRVRESOLUTION * M_PI * DIAMETER * (float)DRVFRIQ;
    Now.SpeedTen = Ardmsg.SpeedTen / (float)DRVRESOLUTION * M_PI * DIAMETER * (float)DRVFRIQ;
}
class tf_odom
{
  private:
    double x = 0.0, y = 0.0, yaw = 0.0, dt = 0.0, vx = 0.0, vy = 0.0, vw = 0.0;
    double P = 0.0, Q = 0.0, PP = 0.0, QQ = 0.0, PQ = 0.0;
    double A[3][6], b[6], vel[3];
    int i,j;
    tf::TransformBroadcaster br;
    tf::Transform tf_base, tf_laser;
    tf::Quaternion q_base, q_laser;
    ros::Time ros_pre = ros::Time::now();
  public:
    tf_odom();
    void get_odom(); // odom情報を計算、配信
};
tf_odom::tf_odom(){
  for(i=0;i<6;i++){
    for(j=0;j<3;j++){
      A[j][i] = 0.0;
      vel[j] = 0.0;
    }
    b[i] = 0.0;
  }
}
void tf_odom::get_odom()
{
  ros::Time ros_now = ros::Time::now();
  dt = ros_now.toSec() - ros_pre.toSec();
  //ROS_INFO("ROS::Time: %lf", dt);
  // 独立3輪ステア 運動学
  P = ;
  Q = ;
  PP = ;
  PQ = ;
  QQ = ;
  A[][] = ;
  if(Now.SppedTen < )
  b[][] = ;
  for(){
    x[]=A[][]*b[];
  }
  vx = x[];

  /*
  V = (vel.left + vel.right) / 2.0;
  W = (vel.right - vel.left) / 2.0 / d;
  x += V * dt * cos(yaw);
  y += V * dt * sin(yaw);
  yaw += W * dt;
  //ROS_INFO("x:%lf, y:%lf, yaw:%lf", x, y, yaw);
  */
  
  // tf message
  tf_base.setOrigin(tf::Vector3(x, y, 0.0));
  q_base.setRPY(0, 0, yaw);
  tf_base.setRotation(q_base);
  tf_laser.setOrigin(tf::Vector3(-0.07, -0.07, 0.085));
  q_laser.setRPY(0, 0, pi);
  tf_laser.setRotation(q_laser);
  
  br.sendTransform(tf::StampedTransform(tf_base, ros_now, "odom", "base_link"));
  br.sendTransform(tf::StampedTransform(tf_laser, ros_now, "base_link", "base_laser"));
  
  // odometry message
  odom.header.stamp = ros_now;
  odom.header.frame_id = "odom";
  odom.pose.pose = setPose(x, y, yaw);
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = V;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = W;
  
  ros_pre = ros_now;
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
