// joystickを使って、ステアの位置制御を逆運動学を用いて行う (より近い角度にステア角を移動させる)
// 左スティックで平行移動 / 右スティックの左右で角速度入力(回転移動)
// まだ実機テスト試していない
#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <msgs/SteerSensor.h>
#include <msgs/SteerPower.h>
#include <msgs/PID.h>
#define OBJECT_TWO 4
#define OBJECT_SIX 3
#define OBJECT_TEN 1
#define deg_to_rad(deg) ((deg)/180*M_PI)
#define rad_to_deg(rad) ((rad)/M_PI*180)

ando

msgs::SteerPower StrPwm, DrvPwm;
int MAX_Drive_PWM = 255, MAX_Steer_PWM = 255, FRIQUENCY = 100; 
int STRRESOLUTION = 10240, DRVRESOLUTION = 480, STROFFSET = 10, DRVOFFSET = 10;
int state = 0, i, ACC = 5;
float STRKP = 0.0, STRKI = 0.0, STRKD = 0.0, DRVKP = 0.0, DRVKI = 0.0, DRVKD = 0.0;
float RADIUS = 133.414, KV = 5.0, KW = 10.0, DIAMETER = 133.414, LIMIT = 1.0;
float vx = 0.0, vy = 0.0, vw = 0.0, vrw = 0.0;
const float ROOT3 = 1.7320508;

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
    if(Power > 0) Power += offset;
    else if(Power < 0) Power -= offset;
    ErrorPre = Error;
    Pre = Now;
    return Power;
}
int feedback::daikei(){
  Error = Goal - Now; // P項 or 偏差
  if(Error > LIMIT) Power += ACC;
  else if(Error < -LIMIT) Power -= ACC;
  // -LIMIT <= SPEED_ERROR[i] <= LIMIT は何もなし
  return Power;
}
int feedback::FORWARD(){
    Power = (int)Goal;
    if(Power > 0) Power += offset;
    else if(Power < 0) Power -= offset;
    return Power;
}

feedback StrTwo, StrSix, StrTen, DrvTwo, DrvSix, DrvTen;

void joyCb(const sensor_msgs::Joy &joy_msg)
{
    // 2のボタンを押せば緊急停止する
    if(joy_msg.buttons[1]){
        state++;
        if(state % 2 == 1) ROS_INFO("STOPPING!");
        else if(state % 2 == 0) ROS_INFO("RESTARTING!");
    }
    // ロボットの目標並進速度vx,vy、目標角速度vwを算出
    vx = KV * joy_msg.axes[1];
    vy = KV * joy_msg.axes[0];
    vw = KW * joy_msg.axes[3];
    // 各ステアユニットの目標速度を算出
    /* vx,vy,vwが全て0のときは角度を維持するようにする(目標角度を更新しない)
        もしうまく行かない場合は、安藤さんの内積を使う方式で試す */
    if(!(vx == 0 && vy == 0 && vw == 0)){
        vrw = RADIUS*vw;
        StrTwo.Goal = -atan2(vy+vrw/2.0, vx+ROOT3/2.0*vrw);
        StrSix.Goal = -atan2(vy-vrw, vx);
        StrTen.Goal = -atan2(vy+vrw/2.0, vx-ROOT3/2.0*vrw);
        ROS_INFO("%lf, %lf, %lf", StrTwo.Goal / M_PI * 180.0, StrSix.Goal / M_PI * 180.0, StrTen.Goal / M_PI * 180.0);
    }
    // 駆動輪の目標速度の算出(ユークリッド距離)
    DrvTwo.Goal = hypotf(vy+vrw/2.0, vx+ROOT3/2.0*vrw);
    DrvSix.Goal = hypotf(vy-vrw, vx);
    DrvTen.Goal = hypotf(vy+vrw/2.0, vx-ROOT3/2.0*vrw);

    


}
void StrArdCb(const msgs::SteerSensor &Ardmsg)
{
    StrTwo.Now = (float) Ardmsg.PulseTwo / STRRESOLUTION * M_PI;
    StrSix.Now = (float) Ardmsg.PulseSix / STRRESOLUTION * M_PI;
    StrTen.Now = (float) Ardmsg.PulseTen / STRRESOLUTION * M_PI;
}
void DrvArdCb(const msgs::SteerSensor &Ardmsg)
{
    DrvTwo.Now = Ardmsg.SpeedTwo / (float)STRRESOLUTION * M_PI * DIAMETER;
    DrvSix.Now = Ardmsg.SpeedSix / (float)STRRESOLUTION * M_PI * DIAMETER;
    DrvTen.Now = Ardmsg.SpeedTen / (float)STRRESOLUTION * M_PI * DIAMETER;
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
    ros::init(argc, argv, "controller4");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.getParamCached("MAX_Drive_PWM", MAX_Drive_PWM);
    pnh.getParamCached("MAX_Steer_PWM", MAX_Steer_PWM);
    pnh.getParamCached("FRIQUENCY", FRIQUENCY);
    pnh.getParamCached("STRKP", STRKP);
    pnh.getParamCached("STRKI", STRKI);
    pnh.getParamCached("STRKD", STRKD);
    pnh.getParamCached("STROFFSET", STROFFSET);
    pnh.getParamCached("DRVKP", DRVKP);
    pnh.getParamCached("DRVKI", DRVKI);
    pnh.getParamCached("DRVKD", DRVKD);
    pnh.getParamCached("DRVOFFSET", DRVOFFSET);
    pnh.getParamCached("DIAMETER", DIAMETER);
    pnh.getParamCached("STRRESOLUTION", STRRESOLUTION);
    pnh.getParamCached("DRVRESOLUTION", DRVRESOLUTION);
    pnh.getParamCached("LIMIT", LIMIT);
    pnh.getParamCached("ACC", ACC);
    pnh.getParamCached("RADIUS", RADIUS);
    pnh.getParamCached("KV", KV);
    pnh.getParamCached("KW", KW);
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
        ROS_INFO("%lf, %lf, %lf", StrTwo.Goal / M_PI * 180.0, StrSix.Goal / M_PI * 180.0, StrTen.Goal / M_PI * 180.0);
        ROS_INFO("%lf, %lf, %lf", StrTwo.Error / M_PI * 180.0, StrSix.Error / M_PI * 180.0, StrTen.Error / M_PI * 180.0);
        ROS_INFO("%d, %d, %d\n", StrPwm.SteerTwo, StrPwm.SteerSix, StrPwm.SteerTen);
        */
        str_ard_pub.publish(StrPwm);
        drv_ard_pub.publish(DrvPwm);
        //dbg_pub.publish(StrSix.param);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}