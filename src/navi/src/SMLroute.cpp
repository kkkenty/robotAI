// 大ボールを移動できるように直線+旋回を追加
//      きれいに旋回できるようにする　予めステア角を移動
//      処理を軽くする
//      最初から壁に突っ込む問題を解決する、処理性能の問題かなあ...
// [ERROR] [1670058293.159778]: Mismatched protocol version in packet ('\xff'): lost sync or rosserial_python is from different ros release than the rosserial client
// [ERROR] [1670058946.703050]: Lost sync with device, restarting...

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>
#define deg_to_rad(deg) ((deg)/180.0*M_PI)
#define rad_to_deg(rad) ((rad)/M_PI*180.0)

const int pt = 20; //目標地点の個数
// double goal[pt][2] = {{0.5, -0.5}, {3.9, -0.5}, {3.9, -2.2}, {0.5, -2.2}, {0.5, -0.5}};  // 時計回り
// double goal[pt][2] = {{0.3, -2.1}, {4.0, -2.1}, {4.0, -0.6}, {0.3, -0.6}, {0.3, -2.1}};  // 反時計周り
const double goal[pt][2] = {{1.4, -0.4}, {3.1, -0.4}, {3.6, -0.9}, {3.6, -1.8}, {3.1, -2.3}, {1.25, -2.3},  // 1個目
                            {0.75, -1.8}, {0.75, -0.9}, {1.25, -0.4}, {3.1, -0.4}, {3.6, -0.9}, {3.6, -1.8}, {3.1, -2.3}, {1.4, -2.3},  // 2個目
                            {0.65, -2.2}, {0.65, -0.35}, {3.9, -0.5}, {3.9, -2.2}, {0.65, -2.2}, {0.65, -0.4}};  // 3個目
const int FirstTurnPoint = 14; // 最初に旋回する点の番号

int stop = 1; // 停止変数
double VEL = 1.0; // ロボットの速度
int FRIQUENCY = 20, den = 100, ahed = 5; // 経路分割数、lookaheddistance
double MAX_VEL = 2.0, MIN_VEL = 0.1, VEL_STP = 0.1, UPVEL = 1.5, STDVEL = 1.0;
double BallSetBorderX = 0.0, BallSetBorderY = 0.0, UP_RANGE = 20.0, BallToStr =  1.0;
double BallSetGoalX = 0.0, BallSetGoalY = 0.0, BallSetGoalR = 0.0, StrBackGoalX = 0.0, StrBackGoalY = 0.0, StrBackGoalR = 0.0;
int TurnAngleError = 5, STAYTIME = 100;
double TURNVEL = 1.0, TurnRadius = 0.5, BACKVEL = 0.5;

// 第1,2引数と第3,4引数の点間距離を算出 //
inline double dis(double x, double y, double ax, double ay){
    return hypot(ax-x, ay-y);
}

// 最も近い点を選択(局所解は無いと仮定) //
inline int dismin(const double &x, const double &y, int &sum, double dotpath[][2]){
    static int nowpose = 0, lastpose = 0;
    int i, mode = 0, count = 0; // 計算状態変数
    double mindis, nowdis; // 最小経路値、現在経路値
    if(lastpose-ahed<0) lastpose = sum + (lastpose - ahed); // 最後の場所より少し前から探索開始
    else                lastpose -= ahed;

    while(count <= sum){ // 全探索しない限り周回
        for(i=lastpose;i<sum;++i){ // 目標経路の更新
            nowdis = dis(x, y, dotpath[i][0], dotpath[i][1]);
            ++count;
            if(mode == 0){ // 初期化
                mindis = nowdis;
                nowpose = i;
                mode = 1;
                continue;
            }
            if(mindis > nowdis){ // 最短距離の更新
                mindis = nowdis;
                nowpose = i;
                mode = 2;
            }
            else if(mode == 2){ // 最短経路の更新が途絶えたら終了
                lastpose = nowpose; // 現在の位置を保持
                return nowpose;
            }
            if(count > sum){ // 初期位置が最短の場合
                lastpose = nowpose; // 現在の位置を保持
                return nowpose;
            }
        }
        lastpose = 0;
    }
}

// joyのcallback関数 //
void joyCb(const sensor_msgs::Joy &joy_msg)
{
    if(joy_msg.buttons[3]){
        if(stop == 0){
            ROS_INFO("NAVI STOPPING!");
            stop = 1;
        }
        else if(stop == 1){
            ROS_INFO("NAVI RESTARTING!");
            stop = 0;
        }
    }
    if(joy_msg.buttons[7]){
        STDVEL += VEL_STP;
        if(STDVEL > MAX_VEL)  STDVEL = MAX_VEL;
        ROS_INFO("Now, VELocity is [%lf]", STDVEL);
    }
    if(joy_msg.buttons[6]){
        STDVEL -= VEL_STP;
        if(STDVEL < MIN_VEL)  STDVEL = MIN_VEL;
        ROS_INFO("Now, VELocity is [%lf]", STDVEL);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "SMLroute");
    ros::NodeHandle nh;
  
    tf::StampedTransform tf;
    geometry_msgs::Twist cmd;
    
    nh.getParamCached("SMLroute/FRIQUENCY", FRIQUENCY);
    nh.getParamCached("SMLroute/den", den);
    nh.getParamCached("SMLroute/ahed", ahed);
    nh.getParamCached("SMLroute/STDVEL", STDVEL);
    nh.getParamCached("SMLroute/UPVEL", UPVEL);
    nh.getParamCached("SMLroute/MAX_VEL", MAX_VEL);
    nh.getParamCached("SMLroute/MIN_VEL", MIN_VEL);
    nh.getParamCached("SMLroute/VEL_STP", VEL_STP);
    nh.getParamCached("SMLroute/BallSetBorderX", BallSetBorderX);
    nh.getParamCached("SMLroute/BallSetBorderY", BallSetBorderY);
    nh.getParamCached("SMLroute/BallSetGoalX", BallSetGoalX);
    nh.getParamCached("SMLroute/BallSetGoalY", BallSetGoalY);
    nh.getParamCached("SMLroute/BallSetGoalR", BallSetGoalR);
    nh.getParamCached("SMLroute/StrBackGoalX", StrBackGoalX);
    nh.getParamCached("SMLroute/StrBackGoalY", StrBackGoalY);
    nh.getParamCached("SMLroute/StrBackGoalR", StrBackGoalR);
    nh.getParamCached("SMLroute/UP_RANGE", UP_RANGE);
    nh.getParamCached("SMLroute/TurnRadius", TurnRadius);
    nh.getParamCached("SMLroute/TURNVEL", TURNVEL);
    nh.getParamCached("SMLroute/BACKVEL", BACKVEL);
    nh.getParamCached("SMLroute/TurnAngleError", TurnAngleError);
    nh.getParamCached("SMLroute/STAYTIME", STAYTIME);
  
    int i, j, k;
    double x = 0.0, y = 0.0, yaw = 0.0; // robot's pose
    double alpha = 0.0, L = 0.0; // 方位誤差、距離
  
    // Markerの定義 //
    visualization_msgs::Marker points;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    points.ns = "line_and_points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = points.scale.y = 0.03;
    points.color.r = points.color.g = points.color.b = points.color.a = 1.0;

    // 点線経路の作成 //
    double path[pt-1][2], lpath[pt-1], sumpath = 0.0; // 各経路、総経路距離
    int npath[pt-1], sum = 0; // 全体に対する経路への整数変換、全体の分割数
    for(i= 0;i<pt-1;++i){
        path[i][0] = goal[i+1][0] - goal[i][0]; // x
        path[i][1] = goal[i+1][1] - goal[i][1]; // y
        lpath[i] = hypot(path[i][0], path[i][1]);
        sumpath += lpath[i];
    }
    for(i=0;i<pt-1;++i){
        npath[i] = round((int)(lpath[i] / sumpath * (double)den));
        sum += npath[i];
    }
    double dotpath[sum][2]; // 点線の座標
    geometry_msgs::Point p;
    for(i=0,k=0;i<pt-1;++i){
        for(j=0;j<npath[i];++j){
            p.x = dotpath[k][0] = goal[i][0] + (path[i][0] * (double)j / (double)npath[i]);
            p.y = dotpath[k][1] = goal[i][1] + (path[i][1] * (double)j / (double)npath[i]);
            points.points.push_back(p);
            ++k;
        }
    }
    int MODE = 1, turnpoint = 0;
    turnpoint = FirstTurnPoint;

    // pubsub宣言 //
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 10);
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCb);
    tf::TransformListener listener;
    ros::Rate rate(FRIQUENCY);
  
    while(nh.ok()){
        rate.sleep();
        ros::spinOnce();
    
        // gpoint, npointの初期化(もっと良い初期化の方法は無いかな...) //
        visualization_msgs::Marker npoint, gpoint;
        npoint.header.frame_id = gpoint.header.frame_id = "map";
        npoint.header.stamp = gpoint.header.stamp = ros::Time::now();
        npoint.ns = gpoint.ns = "line_and_points";
        npoint.action = gpoint.action = visualization_msgs::Marker::ADD;
        npoint.pose.orientation.w = gpoint.pose.orientation.w = 1.0;
        npoint.id = 1;  gpoint.id = 2;
        npoint.type = gpoint.type = visualization_msgs::Marker::POINTS;
        npoint.scale.x = npoint.scale.y = gpoint.scale.x = gpoint.scale.y = 0.1; 
        npoint.color.g = 1.0;  npoint.color.a = 1.0;
        gpoint.color.r = 1.0;  gpoint.color.a = 1.0;
    
        // ロボット座標の取得 //
        try{
            listener.lookupTransform("/map", "/base_link", ros::Time(0), tf);
            x = tf.getOrigin().x();
            y = tf.getOrigin().y();
            yaw = tf::getYaw(tf.getRotation());
        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // 動作変更 //
        static int SETBALL = 0;
        if((x < BallSetBorderX && y < BallSetBorderY) && (SETBALL%6 == 0 || SETBALL%6 == 3)) MODE = 2; 
        else if(SETBALL%6 == 1 || SETBALL%6 == 4)                                            MODE = 3;
        else if(SETBALL%6 == 2 || SETBALL%6 == 5)                                            MODE = 1;

        static int interval = 0, interval2 = 0;
        // ROS_INFO("interval: %d, interval2: %d", interval, interval2);
        if(MODE == 1){
            // 最も近い点を選択 //
            static int pose = 0, inisial = 0;
            pose = dismin(x, y, sum, dotpath);
            p.x = dotpath[pose][0];
            p.y = dotpath[pose][1]; 
            npoint.points.push_back(p);
            if(p.x >= goal[0][0] && p.x <= goal[1][0]) inisial = 1; // 初期の目標点は最初の経路上に設定
            if(p.x >= goal[0][0] && p.x <= goal[1][0] && p.y >= (goal[2][1]+goal[3][1])/2.0 && SETBALL%6 == 2) ++SETBALL; // 地図の左半分に位置すれば、ボール置き換え可能

            static int turning = 0;
            if((dis(x, y, goal[turnpoint][0], goal[turnpoint][1]) <= TurnRadius && SETBALL%6 == 5) || (turning >= 2)){ // 一度turnpointに入るとずっと行われる
                // 旋回モード //
                static int turncount = 0;
                bool turnlimit = false;
                ++interval2;
                ++turning;
                     if(turncount % 6 == 0) turnlimit = (yaw <= deg_to_rad(90+TurnAngleError));                                            // 右下
                else if(turncount % 6 == 1) turnlimit = (yaw <= deg_to_rad(TurnAngleError));                                               // 左下
                else if(turncount % 6 == 2) turnlimit = (yaw <= deg_to_rad(-90+TurnAngleError));                                           // 左上
                else if(turncount % 6 == 3) turnlimit = (yaw <= deg_to_rad(-180+TurnAngleError) || yaw >= deg_to_rad(180-TurnAngleError)); // 右上
                else if(turncount % 6 == 4) turnlimit = (yaw <= deg_to_rad(90+TurnAngleError));                                            // 右下
                else if(turncount % 6 == 5){
                    turnlimit = (yaw <= deg_to_rad(TurnAngleError));  // 左下
                    if(turnlimit)  ++SETBALL;
                }
                p.x = goal[turnpoint][0];
                p.y = goal[turnpoint][1]; 
                gpoint.points.push_back(p);
                if(turnlimit && interval2 >= STAYTIME){ 
                    ++turncount;
                    turning = 0;
                    if(turncount % 6 == 0)  turnpoint = FirstTurnPoint;
                    else                    ++turnpoint;
                    interval = 0; interval2 = 0;
                    cmd.linear.x = 0; 
                    cmd.linear.y = -TURNVEL;
                    cmd.angular.z = TURNVEL / TurnRadius;
                }
                else if(interval2 >= STAYTIME){
                    cmd.linear.x = 0;
                    cmd.linear.y = TURNVEL; 
                    cmd.angular.z = - TURNVEL / TurnRadius;
                }
                else{
                    cmd.linear.x = - BACKVEL;
                    cmd.linear.y = 0;
                    cmd.angular.z = 0;
                }
                // ROS_INFO("x, y, z: %lf %lf %lf", cmd.linear.x, cmd.linear.y, rad_to_deg(cmd.angular.z));
                ROS_INFO("turncount: %d", turncount);
            }
            else if(interval >= STAYTIME){
                // Pure Pursuit //
                // 目標点の設定 //
                pose += ahed;
                if(pose >= sum)   pose -= sum;       // poseの繰り上げ
                if(inisial == 0)  pose = npath[0]*2/3; // 初期の目標点は最初の経路上に設定
                p.x = dotpath[pose][0];
                p.y = dotpath[pose][1];
                gpoint.points.push_back(p);
    
                // 目標点との相対的な角度、距離の算出 //
                alpha = atan2(dotpath[pose][1] - y, dotpath[pose][0] - x) - yaw;
                L = dis(x, y, dotpath[pose][0], dotpath[pose][1]);
                //ROS_INFO("alpha: %lf", rad_to_deg(alpha));

                // 直線走行時は速度を上げる //
                if(fabs(alpha) < deg_to_rad(UP_RANGE) && x >= goal[5][0] && x <= goal[4][0])  VEL = UPVEL;
                else                                                                          VEL = STDVEL;
                // 大ボールのときはターン速度で移動 //
                if(SETBALL%6 == 5) VEL = TURNVEL;
                // 車速と角速度の算出 //
                if(VEL > MAX_VEL)  VEL = MAX_VEL;
                if(VEL < MIN_VEL)  VEL = MIN_VEL;
                cmd.linear.x = VEL;
                cmd.angular.z = 2.0 * VEL * sin(alpha) / L;
            }
            else{
                ++interval;
                cmd.linear.x  = 0.0; 
                cmd.linear.y  = 0.0;
                cmd.angular.z = 0.0;
            }
        }
        else if(MODE == 2){
            // ボール置き動作
            BallToStr = dis(x,y,BallSetGoalX, BallSetGoalY);
            p.x = BallSetGoalX;
            p.y = BallSetGoalY; 
            gpoint.points.push_back(p);
            if(BallToStr > BallSetGoalR){
                cmd.linear.x = VEL/BallToStr*(x-BallSetGoalX);
                cmd.linear.y = VEL/BallToStr*(y-BallSetGoalY);
                cmd.angular.z = 0.0;
            }
            else{
                cmd.linear.x = 0.0;
                cmd.linear.y = 0.0;
                cmd.angular.z = 0.0;
                MODE = 3;   
                ++SETBALL;                
            }
        }
        else if(MODE == 3){
            // バック動作
            BallToStr = dis(x,y,StrBackGoalX, StrBackGoalY);
            p.x = StrBackGoalX;
            p.y = StrBackGoalY; 
            gpoint.points.push_back(p);                
            if(BallToStr > StrBackGoalR){ // 0.3
                cmd.linear.x = VEL/BallToStr*(x-StrBackGoalX);
                cmd.linear.y = VEL/BallToStr*(y-StrBackGoalY);
                cmd.angular.z = 0.0;
            }
            else{
                cmd.linear.x = 0.0;
                cmd.linear.y = 0.0;
                cmd.angular.z = 0.0;
                MODE = 1;
                ++SETBALL;
            }
        }
        ROS_INFO("mode: %d",MODE);

        // naviの停止コマンド //
        if(stop){ 
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.z = 0.0;
        }
    
        // pub配信 //
        cmd_pub.publish(cmd);
        marker_pub.publish(points);
        marker_pub.publish(npoint);
        marker_pub.publish(gpoint);
    }
    return 0;
}
