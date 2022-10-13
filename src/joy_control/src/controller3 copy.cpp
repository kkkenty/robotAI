// joystickを使って、ステアの位置制御を逆運動学を用いて行う
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
int unit = 2, state = 0, i;
float YAWKP = 0.0, YAWKI = 0.0, YAWKD = 0.0;
float vx = 0.0, vy = 0.0, vw = 0.0, vrw = 0.0;
float KV = 5.0, KW = 10.0, RADIUS = 133.414;
const float ROOT3 = 1.7320508;

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
    param.p = YAWKP * AngleError;
    param.i = YAWKI * AngleSum;
    param.d = YAWKD * AngleSpd;
    Power = (int)(param.p + param.i - param.d);
    //ROS_INFO("  %lf   ", AngleError);
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
    // ロボットの目標並進速度vx,vy、目標角速度vwを算出
    vx = KV * (-joy_msg.axes[0]);
    vy = KV * joy_msg.axes[1];
    vw = KW * joy_msg.axes[3];
    // 各ステアユニットの目標速度を算出
    /* vx,vy,vwが全て0のときは角度を維持するようにする(目標角度を更新しない)
        もしうまく行かない場合は、安藤さんの内積を使う方式で試す */
    if(!(vx == 0 && vy == 0 && vw == 0)){
        vrw = RADIUS*vw;
        TWO.AngleGoal = -atan2(2.0*vy+vrw, 2.0*vx+ROOT3*vrw);
        SIX.AngleGoal = -atan2(vy-vrw, vx);
        TEN.AngleGoal = -atan2(2.0*vy+vrw, 2.0*vx-ROOT3*vrw);
        ROS_INFO("%lf, %lf, %lf", TWO.AngleGoal / M_PI * 180.0, SIX.AngleGoal / M_PI * 180.0, TEN.AngleGoal / M_PI * 180.0);
    }
    // 2のボタンを押せば緊急停止する
    if(joy_msg.buttons[1]){
        state++;
        if(state % 2 == 1){
            ROS_INFO("STOPPING!");
        }
        else if(state % 2 == 0){
            ROS_INFO("RESTARTING!");
        }
    }
}
void StrArdCb(const msgs::SteerSensor &Ardmsg)
{
    // SteerArduinoからパルス情報を受け取る
    TWO.GetPulse = Ardmsg.PulseTwo; 
    SIX.GetPulse = Ardmsg.PulseSix;
    TEN.GetPulse = Ardmsg.PulseTen;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller3 copy");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.getParam("MAX_Drive_PWM", MAX_Drive_PWM);
    pnh.getParam("MAX_Steer_PWM", MAX_Steer_PWM);
    pnh.getParam("FRIQUENCY", FRIQUENCY);
    pnh.getParam("YAWKP", YAWKP);
    pnh.getParam("YAWKI", YAWKI);
    pnh.getParam("YAWKD", YAWKD);
    pnh.getParam("KV", KV);
    pnh.getParam("KW", KW);
    pnh.getParam("RADIUS", RADIUS);
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCb);
    ros::Subscriber str_ard_sub = nh.subscribe("encoder", 10, StrArdCb);
    ros::Publisher str_ard_pub = nh.advertise<msgs::SteerPower>("power", 10);
    ros::Publisher dbg_pub = nh.advertise<msgs::PID>("param", 10);
    ros::Rate loop_rate(FRIQUENCY);

    PWM.SteerSix = 0; PWM.SteerTen = 0; PWM.SteerTwo = 0;

    while (ros::ok())
    {
        // 現在のSteerの回転角度を算出
        TWO.cal_angle();
        SIX.cal_angle();
        TEN.cal_angle();
        // PID制御
        PWM.SteerTwo = -TWO.PID();
        PWM.SteerSix = -SIX.PID();
        PWM.SteerTen = -TEN.PID();
        if(state % 2 == 1){
            PWM.SteerTwo = 0;
            PWM.SteerSix = 0;
            PWM.SteerTen = 0;
        }
        str_ard_pub.publish(PWM);
        dbg_pub.publish(param);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}