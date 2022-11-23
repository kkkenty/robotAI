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

int STRRESOLUTION = 2560, DRVRESOLUTION = 384, FRIQUENCY = 100;
double r = 0.133414; // 回転半径
double DIAMETER = 0.064;// 駆動輪直径
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
    Now.SpeedTwo = Ardmsg.SpeedTwo / (float)DRVRESOLUTION * M_PI * DIAMETER * 1000.0; // Ardからの速度単位は[pulse/ms]
    Now.SpeedSix = Ardmsg.SpeedSix / (float)DRVRESOLUTION * M_PI * DIAMETER * 1000.0;
    Now.SpeedTen = Ardmsg.SpeedTen / (float)DRVRESOLUTION * M_PI * DIAMETER * 1000.0;
}
class tf_odom
{
  private:
    double x = 0.0, y = 0.0, yaw = 0.0, dt = 0.0, vx = 0.0, vy = 0.0, vw = 0.0;
    double P = 0.0, Q = 0.0, PP = 0.0, QQ = 0.0, PQ = 0.0, rr = 0.0, nolA = 0.0, c[3], s[3];
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
    for(i=0;i<3;i++){
        for(j=0;j<6;j++){
            A[i][j] = 0.0;
            b[j] = 0.0;
        }
        vel[i] = 0.0;
        c[i] = 0.0; 
        s[i] = 0.0;
    }
}
void tf_odom::get_odom()
{
    ros::Time ros_now = ros::Time::now();
    dt = ros_now.toSec() - ros_pre.toSec();
    //ROS_INFO("ROS::Time: %lf", dt);
    // 運動学(独立3輪ステア)
    c[0] = cos(yaw); c[1] = cos(yaw-M_PI/3.0); c[2] = cos(yaw-2.0*M_PI/3.0);
    s[0] = sin(yaw); s[1] = sin(yaw-M_PI/3.0); s[2] = sin(yaw-2.0*M_PI/3.0);  
    P = r*(c[1]-c[0]-c[2]);
    Q = r*(-s[1]+s[0]+s[2]);
    PP = P*P;  
    PQ = P*Q;  
    QQ = Q*Q;
    rr = r*r;
    nolA = 27.0*rr;
    A[0][0] = 9.0*rr-PP+3.0*r*s[1]*Q; A[0][1] = PQ-3.0*r*c[1]*Q;        A[0][2] = 9.0*rr-PP-3.0*r*s[0]*Q; A[0][3] = PQ+3.0*r*c[0]*Q;        A[0][4] = 9.0*rr-PP-3.0*r*s[2]*Q; A[0][5] = PQ+3.0*r*c[2]*Q;
    A[1][0] = PQ+3.0*r*s[1]*P;        A[1][1] = 9.0*rr-QQ-3.0*r*c[1]*P; A[1][2] = PQ-3.0*r*s[0]*P;        A[1][3] = 9.0*rr-QQ+3.0*r*c[0]*P; A[1][4] = PQ-3.0*r*s[2]*P;        A[1][5] = 9.0*rr-QQ+3.0*r*c[2]*P;
    A[2][0] = -3.0*Q-9.0*r*s[1];      A[2][1] = -3.0*P+9.0*r*c[1];      A[2][2] = -3.0*Q+9.0*r*s[0];      A[2][3] = -3.0*P-9.0*r*c[0];      A[2][4] = -3.0*Q+9.0*r*s[2];      A[2][5] = -3.0*P-9.0*r*c[2];
    for(i=0;i<3;i++){
        for(j=0;j<6;j++){
            A[i][j] /= nolA;
        }
    } 
    b[0] = Now.SpeedTwo*cos(yaw+Now.AngleTwo);
    b[1] = Now.SpeedTwo*sin(yaw+Now.AngleTwo);
    b[2] = Now.SpeedSix*cos(yaw+Now.AngleSix);
    b[3] = Now.SpeedSix*sin(yaw+Now.AngleSix);
    b[4] = Now.SpeedTen*cos(yaw+Now.AngleTen);
    b[5] = Now.SpeedTen*sin(yaw+Now.AngleTen);
    for(i=0;i<3;i++){
        vel[i] = 0.0;
        for(j=0;j<6;j++){
            vel[i] += A[i][j]*b[j]; // vel = A*b
        }
    }
    vx = vel[0];
    vy = vel[1];
    vw = vel[2];
    x += vx*dt;
    y += vy*dt;
    yaw += vw*dt;
    ROS_INFO("x:%lf, y:%lf, yaw:%lf", x, y, yaw);
    
    // tf message
    tf_base.setOrigin(tf::Vector3(x, y, 0.0));
    q_base.setRPY(0, 0, yaw);
    tf_base.setRotation(q_base);
    tf_laser.setOrigin(tf::Vector3(-0.0665, -0.115, 0.0875));
    q_laser.setRPY(0, 0, -2.0*M_PI/3.0);
    tf_laser.setRotation(q_laser);
    br.sendTransform(tf::StampedTransform(tf_base, ros_now, "odom", "base_link"));
    br.sendTransform(tf::StampedTransform(tf_laser, ros_now, "base_link", "base_laser"));
  
    // odometry message
    odom.header.stamp = ros_now;
    odom.header.frame_id = "odom";
    odom.pose.pose = setPose(x, y, yaw);
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vw;
  
    ros_pre = ros_now;
}
// main関数 //
int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;
    nh.getParamCached("odometry/FRIQUENCY", FRIQUENCY);
    nh.getParamCached("controller/RADIUS", r);
    nh.getParamCached("controller/DIAMETER", DIAMETER);
    nh.getParamCached("controller/STRRESOLUTION", STRRESOLUTION);
    nh.getParamCached("controller/DRVRESOLUTION", DRVRESOLUTION);

    tf_odom robot;
    ros::Subscriber str_ard_sub = nh.subscribe("StrEncoder", 10, StrArdCb);
    ros::Subscriber drv_ard_sub = nh.subscribe("DrvEncoder", 10, DrvArdCb);
    ros::Publisher odm_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Publisher ctr_pub = nh.advertise<msgs::SteerOdometry>("angvel", 10);
    ros::Rate loop_rate(FRIQUENCY);

    while(ros::ok())
    {
        robot.get_odom();
        odm_pub.publish(odom);
        ctr_pub.publish(Now);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
