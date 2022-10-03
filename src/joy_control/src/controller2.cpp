// joystickを使って、ステアの位置制御を行いPWM値を配信する
#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <msgs/SteerSensor.h>
#include <msgs/SteerPower.h>
#include <msgs/PID.h>
#define OBJECT_TWO 4
#define OBJECT_SIX 3
#define OBJECT_TEN 1

msgs::SteerPower PWM;
msgs::PID param;
int MAX_Drive_PWM = 255, MAX_Steer_PWM = 255, FRIQUENCY = 100;
int unit = 2, i;
float yaw_kp = 0.0, yaw_ki = 0.0, yaw_kd = 0.0;

class feedback{
    private:
        float AngleNow = 0.0, AngleError = 0.0, AngleSum = 0.0, AngleErrorPre = 0.0;
        float AngleSpd = 0.0, AnglePre = 0.0;
        int Power = 0;
    public:
        int GetPulse = 0;
        double AngleGoal = 0.0;
        void cal_angle();
        int PID();
};
void feedback::cal_angle(){
    AngleNow = (double) GetPulse / 10240 * M_PI;
    //ROS_INFO("  %lf  ", AngleNow / M_PI * 180.0);
    //ROS_INFO("  %lf  ", AngleGoal / M_PI * 180.0);
}
int feedback::PID(){
    AngleError = AngleGoal - AngleNow; // P項
    AngleSum += (AngleError + AngleErrorPre) / 2.0; // I項
    AngleSpd = (float)(AngleNow - AnglePre) * (float)FRIQUENCY; // D項
    param.p = yaw_kp * AngleError;
    param.i = yaw_ki * AngleSum;
    param.d = yaw_kd * AngleSpd;
    Power = (int)(param.p + param.i - param.d);
    ROS_INFO("  %lf   ", AngleError);
    if(Power > MAX_Steer_PWM){
        Power = MAX_Steer_PWM;
    }
    if(Power < -MAX_Steer_PWM){
        Power = -MAX_Steer_PWM;
    }
    AngleErrorPre = AngleError;
    AnglePre = AngleNow;
    return Power;
}

feedback TWO, SIX, TEN;

void joyCb(const sensor_msgs::Joy &joy_msg)
{
  for(i=1;i<5;i++){
      if(joy_msg.buttons[i-1] == 1){
          unit = i; // ここでunitの値が変わらない限り、PWMは変わらない
          ROS_INFO("the mode is %d\n", unit);
          break;
      }
  }
  switch(unit){
    case OBJECT_TEN:
        TEN.AngleGoal = atan2(joy_msg.axes[1],-joy_msg.axes[0]);
        break;
    case OBJECT_SIX:
        SIX.AngleGoal = atan2(joy_msg.axes[1],-joy_msg.axes[0]);
        break;
    case OBJECT_TWO:
        TWO.AngleGoal = atan2(joy_msg.axes[1],-joy_msg.axes[0]);
        break;
    default:
        break;
  }
}
void StrArdCb(const msgs::SteerSensor &Ardmsg)
{
    TWO.GetPulse = Ardmsg.PulseTwo; 
    SIX.GetPulse = Ardmsg.PulseSix;
    TEN.GetPulse = Ardmsg.PulseTen;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller2");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.getParam("MAX_Drive_PWM", MAX_Drive_PWM);
    pnh.getParam("MAX_Steer_PWM", MAX_Steer_PWM);
    pnh.getParam("FRIQUENCY", FRIQUENCY);
    pnh.getParam("yaw_kp", yaw_kp);
    pnh.getParam("yaw_ki", yaw_ki);
    pnh.getParam("yaw_kd", yaw_kd);
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCb);
    ros::Subscriber str_ard_sub = nh.subscribe("encoder", 10, StrArdCb);
    ros::Publisher str_ard_pub = nh.advertise<msgs::SteerPower>("power", 10);
    ros::Publisher dbg_pub = nh.advertise<msgs::PID>("param", 10);
    ros::Rate loop_rate(FRIQUENCY);

    while (ros::ok())
    {
        TWO.cal_angle();
        SIX.cal_angle();
        TEN.cal_angle();
        ROS_INFO("\n");
        PWM.SteerTwo = TWO.PID();
        PWM.SteerSix = SIX.PID();
        PWM.SteerTen = TEN.PID();
        if(unit == 2){
            PWM.SteerTwo = 0;
            PWM.SteerSix = 0;
            PWM.SteerTen = 0;
        }
        str_ard_pub.publish(PWM);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}