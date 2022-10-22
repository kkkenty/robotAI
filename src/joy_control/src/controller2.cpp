// joystickを使って、ステアの位置制御、駆動輪の速度制御を行いPWM値を配信する
// 各モータのデバッグ用
#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <msgs/SteerSensor.h>
#include <msgs/SteerPower.h>
#include <msgs/PID.h>
#define OBJECT_TWO 4
#define OBJECT_SIX 3
#define OBJECT_TEN 1

msgs::SteerPower StrPwm, DrvPwm;
int MAX_Drive_PWM = 255, MAX_Steer_PWM = 255, FRIQUENCY = 100, STRRESOLUTION = 10240, DRVRESOLUTION = 480, STROFFSET = 10, DRVOFFSET = 10;
int state = 0, i, ACC = 5;
float STRKP = 0.0, STRKI = 0.0, STRKD = 0.0, DRVKP = 0.0, DRVKI = 0.0, DRVKD = 0.0, KV = 1.0;
float DIAMETER = 133.414, LIMIT = 1.0;

class feedback{
    private:
        float Sum = 0.0, ErrorPre = 0.0, Dif = 0.0, Pre = 0.0;
        int Power = 0;
    public:
        int GetPulse = 0, offset = 0;
        float Now = 0.0, Goal = 0.0, Error = 0.0, kp = 0.0, ki = 0.0, kd = 0.0;
        msgs::PID param;
        int PID();
        int FORWARD();
        int daikei();
};
int feedback::PID(){
    Error = Goal - Now; // P項
    Sum += (Error + ErrorPre) / 2.0; // I項
    Dif = (float)(Now - Pre) * (float)FRIQUENCY; // D項
    param.p = kp * Error;
    param.i = ki * Sum;
    param.d = kd * Dif;
    Power = (int)(param.p + param.i - param.d);
    if(Power > 0){
        Power += offset;
    }
    else if(Power < 0){
        Power -= offset;
    }
    ErrorPre = Error;
    Pre = Now;
    return Power;
}
int feedback::daikei(){
  Error = Goal - Now; // P項 or 偏差
  if(Error > LIMIT){
    Power += ACC;
  }
  else if(Error < -LIMIT){
    Power -= ACC;
  }
  else{ // -LIMIT <= SPEED_ERROR[i] <= LIMIT
  }
  return Power;
}
int feedback::FORWARD(){
    Power = (int)Goal;
    if(Power > 0){
        Power += offset;
    }
    else if(Power < 0){
        Power -= offset;
    }
    return Power;
}

feedback StrTwo, StrSix, StrTen, DrvTwo, DrvSix, DrvTen;

void joyCb(const sensor_msgs::Joy &joy_msg)
{
    // 2のボタンを押せば緊急停止する
    if(joy_msg.buttons[1]){
        state++;
        if(state % 2 == 1){
            /*DrvPwm.DriveTwo = 0; DrvPwm.DriveSix = 0; DrvPwm.DriveTen = 0;
            StrPwm.SteerTwo = 0; StrPwm.SteerSix = 0; StrPwm.SteerTen = 0; */
            ROS_INFO("STOPPING!");
        }
        else if(state % 2 == 0){
            ROS_INFO("RESTARTING!");
        }
    }
    // ステアの目標角度の算出
    if(!(joy_msg.axes[0] == 0 && joy_msg.axes[1] == 0)){
        StrTwo.Goal = -atan2(joy_msg.axes[0], joy_msg.axes[1]);
        StrSix.Goal = -atan2(joy_msg.axes[0], joy_msg.axes[1]);
        StrTen.Goal = -atan2(joy_msg.axes[0], joy_msg.axes[1]);
        //ROS_INFO("%lf, %lf, %lf", StrTwo.Goal / M_PI * 180.0, StrSix.Goal / M_PI * 180.0, StrTen.Goal / M_PI * 180.0);
    }
    // 駆動輪の目標速度の算出
    DrvTwo.Goal = KV * joy_msg.axes[2];
    DrvSix.Goal = KV * joy_msg.axes[2];
    DrvTen.Goal = KV * joy_msg.axes[2];
}
void StrArdCb(const msgs::SteerSensor &Ardmsg)
{
    StrTwo.Now = (float) Ardmsg.PulseTwo / STRRESOLUTION * M_PI;
    StrSix.Now = (float) Ardmsg.PulseSix / STRRESOLUTION * M_PI;
    StrTen.Now = (float) Ardmsg.PulseTen / STRRESOLUTION * M_PI;
}
void DrvArdCb(const msgs::SteerSensor &Ardmsg)
{
    DrvTwo.Now = Ardmsg.SpeedTwo / (float)DRVRESOLUTION * M_PI * DIAMETER;
    DrvSix.Now = Ardmsg.SpeedSix / (float)DRVRESOLUTION * M_PI * DIAMETER;
    DrvTen.Now = Ardmsg.SpeedTen / (float)DRVRESOLUTION * M_PI * DIAMETER;
}
void ParamSet(){
    StrTwo.kp = STRKP; StrSix.kp = STRKP; StrTen.kp = STRKP;
    StrTwo.ki = STRKI; StrSix.ki = STRKI; StrTen.ki = STRKI;
    StrTwo.kd = STRKD; StrSix.kd = STRKD; StrTen.kd = STRKD;
    DrvTwo.kp = DRVKP; DrvSix.kp = DRVKP; DrvTen.kp = DRVKP;
    DrvTwo.ki = DRVKI; DrvSix.ki = DRVKI; DrvTen.ki = DRVKI;
    DrvTwo.kd = DRVKD; DrvSix.kd = DRVKD; DrvTen.kd = DRVKD;
    StrTwo.offset = STROFFSET; StrSix.offset = STROFFSET; StrTen.offset = STROFFSET;
    DrvTwo.offset = DRVOFFSET; DrvSix.offset = DRVOFFSET; DrvTen.offset = DRVOFFSET;
}
void LimitPwm(){
    if(StrPwm.SteerTwo > MAX_Steer_PWM) StrPwm.SteerTwo = MAX_Steer_PWM;
    if(StrPwm.SteerSix > MAX_Steer_PWM) StrPwm.SteerSix = MAX_Steer_PWM;
    if(StrPwm.SteerTen > MAX_Steer_PWM) StrPwm.SteerTen = MAX_Steer_PWM;
    if(StrPwm.SteerTwo < -MAX_Steer_PWM) StrPwm.SteerTwo = -MAX_Steer_PWM;
    if(StrPwm.SteerSix < -MAX_Steer_PWM) StrPwm.SteerSix = -MAX_Steer_PWM;
    if(StrPwm.SteerTen < -MAX_Steer_PWM) StrPwm.SteerTen = -MAX_Steer_PWM;
    if(DrvPwm.DriveTwo > MAX_Drive_PWM) DrvPwm.DriveTwo = MAX_Drive_PWM;
    if(DrvPwm.DriveSix > MAX_Drive_PWM) DrvPwm.DriveSix = MAX_Drive_PWM;
    if(DrvPwm.DriveTen > MAX_Drive_PWM) DrvPwm.DriveTen = MAX_Drive_PWM;
    if(DrvPwm.DriveTwo < -MAX_Drive_PWM) DrvPwm.DriveTwo = -MAX_Drive_PWM;
    if(DrvPwm.DriveSix < -MAX_Drive_PWM) DrvPwm.DriveSix = -MAX_Drive_PWM;
    if(DrvPwm.DriveTen < -MAX_Drive_PWM) DrvPwm.DriveTen = -MAX_Drive_PWM;
    if(state % 2 == 1){
        DrvPwm.DriveTwo = 0; DrvPwm.DriveSix = 0; DrvPwm.DriveTen = 0;
        StrPwm.SteerTwo = 0; StrPwm.SteerSix = 0; StrPwm.SteerTen = 0; 
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller2");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.getParamCached("MAX_Drive_PWM", MAX_Drive_PWM);
    pnh.getParamCached("MAX_Steer_PWM", MAX_Steer_PWM);
    pnh.getParamCached("FRIQUENCY", FRIQUENCY);
    pnh.getParamCached("STRKP", STRKP);
    pnh.getParamCached("STRKI", STRKI);
    pnh.getParamCached("STRKD", STRKD);
    pnh.getParamCached("DRVKP", DRVKP);
    pnh.getParamCached("DRVKI", DRVKI);
    pnh.getParamCached("DRVKD", DRVKD);
    pnh.getParamCached("STROFFSET", STROFFSET);
    pnh.getParamCached("DRVOFFSET", DRVOFFSET);
    pnh.getParamCached("DIAMETER", DIAMETER);
    pnh.getParamCached("KV", KV);
    pnh.getParamCached("STRRESOLUTION", STRRESOLUTION);
    pnh.getParamCached("DRVRESOLUTION", DRVRESOLUTION);
    pnh.getParamCached("LIMIT", LIMIT);
    pnh.getParamCached("ACC", ACC);
    ParamSet();
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCb);
    ros::Subscriber str_ard_sub = nh.subscribe("StrEncoder", 10, StrArdCb);
    ros::Subscriber drv_ard_sub = nh.subscribe("DrvEncoder", 10, DrvArdCb);
    ros::Publisher str_ard_pub = nh.advertise<msgs::SteerPower>("StrPower", 10);
    ros::Publisher drv_ard_pub = nh.advertise<msgs::SteerPower>("DrvPower", 10);
    ros::Publisher dbg_pub = nh.advertise<msgs::PID>("param", 10);
    ros::Rate loop_rate(FRIQUENCY);

    while (ros::ok())
    {
        StrPwm.SteerTwo = -StrTwo.PID();
        StrPwm.SteerSix = -StrSix.PID();
        StrPwm.SteerTen = -StrTen.PID();
        DrvPwm.DriveTwo = DrvTwo.PID();
        DrvPwm.DriveSix = DrvSix.PID();
        DrvPwm.DriveTen = DrvTen.PID();
        LimitPwm();
        /*
        ROS_INFO("Goal  %lf, %lf, %lf", StrTwo.Goal / M_PI * 180.0, StrSix.Goal / M_PI * 180.0, StrTen.Goal / M_PI * 180.0);
        ROS_INFO("Error %lf, %lf, %lf", StrTwo.Error / M_PI * 180.0, StrSix.Error / M_PI * 180.0, StrTen.Error / M_PI * 180.0);
        ROS_INFO("PWM   %d, %d, %d\n", StrPwm.SteerTwo, StrPwm.SteerSix, StrPwm.SteerTen);
        */
        ROS_INFO("Goal  %lf, %lf, %lf", DrvTwo.Goal, DrvSix.Goal, DrvTen.Goal);
        ROS_INFO("Now   %lf, %lf, %lf", DrvTwo.Now, DrvSix.Now, DrvTen.Now);
        ROS_INFO("Error %lf, %lf, %lf", DrvTwo.Error, DrvSix.Error, DrvTen.Error);
        ROS_INFO("PWM   %d, %d, %d\n", DrvPwm.DriveTwo, DrvPwm.DriveSix, DrvPwm.DriveTen);
        
        str_ard_pub.publish(StrPwm);
        drv_ard_pub.publish(DrvPwm);
        dbg_pub.publish(DrvSix.param);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}