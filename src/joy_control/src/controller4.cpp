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

msgs::SteerPower StrPwm, DrvPwm;
int MAX_Drive_PWM = 255, MAX_Steer_PWM = 255, FRIQUENCY = 100; 
int STRRESOLUTION = 10240, DRVRESOLUTION = 480, STROFFSET = 10, DRVOFFSET = 10;
int state = 0, i, ACC = 5, DRVFRIQ = 40;
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
        float gx = 0.0, gy = 0.0, nx = 0.0, ny = 0.0;
        msgs::PID param;
        int PID();
        int FORWARD();
        int daikei();
        int calc_dir();
        float calc_deg();
        int calc_vel();
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
int feedback::calc_dir(){
    if((nx*gy - gx*ny) >= 0) return 1; // 外積のz成分で回転方向を判明する
    else return -1;
}
float feedback::calc_deg(){
    if(abs(hypotf(gx,gy)*hypotf(nx,ny)) > 1e-5){ // 0で除算させない工夫
        float cos_theta = ( (gx*nx)+(gy*ny) ) / ( hypotf(gx,gy)*hypotf(nx,ny) );
        if(cos_theta >= 0 && cos_theta <= 1){ // 角度偏差が -90 ~ 90 deg
            return (Now + (float)calc_dir() * acos(cos_theta)); // 角度偏差を加える（角度の正負を考慮）
        }
        else if(cos_theta < 0 && cos_theta >= -1){ // 角度偏差が-180 ~ -90 もしくは 90 ~ 180 deg
            return (Now - (float)calc_dir() * (M_PI - acos(cos_theta))); // 角度を逆方向に加える
        }
    }
}
int feedback::calc_vel(){
    if(hypotf(gx,gy)*hypotf(nx,ny) != 0){ // 角速度がともに0でない場合
        float cos_theta = (gx*nx)+(gy*ny);
        if(cos_theta < 0) return -1; // 回転角度が -90 ~ 90 degをはみ出す場合
        else return 1; // 速度を反転させる
    }
    else return 1;
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
    // ステアの目標角度を算出（参考：chjk_node）
    vrw = RADIUS*vw;
    if(!(vx == 0 && vy == 0 && vw == 0)){
        StrTwo.gx = vx+ROOT3/2.0*vrw; 
        StrTwo.gy = vy+vrw/2.0;
        StrTwo.Goal = StrTwo.calc_deg();
        StrSix.gx = vx; 
        StrSix.gy = vy-vrw;
        StrSix.Goal = StrSix.calc_deg();
        /*
        ROS_INFO("joy %lf", atan2(vy,vx)/M_PI*180.0);
        ROS_INFO("Goal %lf", StrSix.Goal/M_PI*180.0);
        ROS_INFO("Now %lf\n", StrSix.Now/M_PI*180.0);
        */
        StrTen.gx = vx-ROOT3/2.0*vrw; 
        StrTen.gy = vy+vrw/2.0;
        StrTen.Goal = StrTen.calc_deg();
        //ROS_INFO("%lf, %lf, %lf", StrTwo.Goal / M_PI * 180.0, StrSix.Goal / M_PI * 180.0, StrTen.Goal / M_PI * 180.0);
    }
    if(joy_msg.buttons[4] && joy_msg.buttons[5]){
        StrTwo.Goal = 0.0;
        StrSix.Goal = 0.0;
        StrTen.Goal = 0.0;
    }
    // 駆動輪の目標速度の算出(ユークリッド距離)
    DrvTwo.Goal = (float)StrTwo.calc_vel() * hypotf(vy+vrw/2.0, vx+ROOT3/2.0*vrw);
    DrvSix.Goal = (float)StrSix.calc_vel() * hypotf(vy-vrw, vx);
    DrvTen.Goal = (float)StrTen.calc_vel() * hypotf(vy+vrw/2.0, vx-ROOT3/2.0*vrw);
}
void StrArdCb(const msgs::SteerSensor &Ardmsg)
{
    StrTwo.Now = -(float) Ardmsg.PulseTwo / STRRESOLUTION * M_PI;
    StrTwo.nx = cos(StrTwo.Now); 
    StrTwo.ny = sin(StrTwo.Now);
    StrSix.Now = -(float) Ardmsg.PulseSix / STRRESOLUTION * M_PI;
    StrSix.nx = cos(StrSix.Now); 
    StrSix.ny = sin(StrSix.Now);
    StrTen.Now = -(float) Ardmsg.PulseTen / STRRESOLUTION * M_PI;
    StrTen.nx = cos(StrTen.Now); 
    StrTen.ny = sin(StrTen.Now);
}
void DrvArdCb(const msgs::SteerSensor &Ardmsg)
{
    DrvTwo.Now = Ardmsg.SpeedTwo / (float)DRVRESOLUTION * M_PI * DIAMETER * (float)DRVFRIQ;
    DrvSix.Now = Ardmsg.SpeedSix / (float)DRVRESOLUTION * M_PI * DIAMETER * (float)DRVFRIQ;
    DrvTen.Now = Ardmsg.SpeedTen / (float)DRVRESOLUTION * M_PI * DIAMETER * (float)DRVFRIQ;
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
    pnh.getParamCached("DRVFRIQ", DRVFRIQ);
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
        StrPwm.SteerTwo = StrTwo.PID();
        StrPwm.SteerSix = StrSix.PID();
        StrPwm.SteerTen = StrTen.PID();
        DrvPwm.DriveTwo = DrvTwo.PID();
        DrvPwm.DriveSix = DrvSix.PID();
        DrvPwm.DriveTen = DrvTen.PID();
        LimitPwm();
        
        ROS_INFO("Str.Goal %lf, %lf, %lf", StrTwo.Goal / M_PI * 180.0, StrSix.Goal / M_PI * 180.0, StrTen.Goal / M_PI * 180.0);
        ROS_INFO("Str.Now %lf, %lf, %lf", StrTwo.Now / M_PI * 180.0, StrSix.Now / M_PI * 180.0, StrTen.Now / M_PI * 180.0);
        ROS_INFO("Str.Error %lf, %lf, %lf", StrTwo.Error / M_PI * 180.0, StrSix.Error / M_PI * 180.0, StrTen.Error / M_PI * 180.0);
        ROS_INFO("Str.Pwm %d, %d, %d\n", StrPwm.SteerTwo, StrPwm.SteerSix, StrPwm.SteerTen);
        /*
        ROS_INFO("Goal  %lf, %lf, %lf", DrvTwo.Goal, DrvSix.Goal, DrvTen.Goal);
        ROS_INFO("Now   %lf, %lf, %lf", DrvTwo.Now, DrvSix.Now, DrvTen.Now);
        ROS_INFO("Error %lf, %lf, %lf", DrvTwo.Error, DrvSix.Error, DrvTen.Error);
        ROS_INFO("PWM   %d, %d, %d\n", DrvPwm.DriveTwo, DrvPwm.DriveSix, DrvPwm.DriveTen);
        */
        str_ard_pub.publish(StrPwm);
        drv_ard_pub.publish(DrvPwm);
        //dbg_pub.publish(StrSix.param);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}