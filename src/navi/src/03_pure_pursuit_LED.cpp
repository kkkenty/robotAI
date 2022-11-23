// セットアップ完了時にLEDも光らせる（NAVIが停止中に光らせる）
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

// global変数 // 
const int pt = 5; //目標地点の個数
double goal[pt][2] = {{0.3, -0.6}, {4.0, -0.6}, {4.0, -2.1}, {0.3, -2.1}, {0.3, -0.6}}; // robosa
//double goal[pt][2] = {{-0.5, -0.2}, {1.0, -0.2}, {1.0, -1.2}, {-0.5, -1.2}, {-0.5, -0.2}}; // sister's room
int stap = 1; // 停止変数
double vel = 0.35; // ロボットの速度

// 第1,2引数と第3,4引数の点間距離を算出 //
double dis(double x, double y, double ax, double ay);

// 最も近い点を選択(局所解は無いと仮定) //
int dismin(const double &x, const double &y, int &sum, double dotpath[][2]);

// joyのcallback関数 //
void joyCb(const sensor_msgs::Joy &joy_msg);

int main(int argc, char** argv){
  ros::init(argc, argv, "pure_pursuit_LED");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  tf::StampedTransform tf;
  geometry_msgs::Twist cmd;
  int FRIQUENCE = 20, den = 100, ahed = 5; // 経路分割数、lookaheddistance
  pnh.getParam("FRIQUENCE", FRIQUENCE);
  pnh.getParam("den", den);
  pnh.getParam("ahed", ahed);
  pnh.getParam("vel", vel);
  
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
  for(i= 0;i<pt-1;i++){
    path[i][0] = goal[i+1][0] - goal[i][0]; // x
    path[i][1] = goal[i+1][1] - goal[i][1]; // y
    lpath[i] = sqrt(pow(path[i][0], 2) + pow(path[i][1], 2));
    sumpath += lpath[i];
  }
  for(i=0;i<pt-1;i++){
    npath[i] = round((int)(lpath[i] / sumpath * (double)den));
    sum += npath[i];
  }
  double dotpath[sum][2]; // 点線の座標
  geometry_msgs::Point p;
  for(i=0,k=0;i<pt-1;i++){
      for(j=0;j<npath[i];j++){
          p.x = dotpath[k][0] = goal[i][0] + (path[i][0] * (double)j / (double)npath[i]);
          p.y = dotpath[k][1] = goal[i][1] + (path[i][1] * (double)j / (double)npath[i]);
          points.points.push_back(p);
          k++;
      }
  }
  
  // pubsub宣言 //
  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 10);
  ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCb);
  tf::TransformListener listener;
  ros::Rate rate(FRIQUENCE);
  
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
    // 正常時にLEDは消灯する
    cmd.linear.z = 0;
    
    // 最も近い点を選択 //
    static int pose = 0;
    pose = dismin(x, y, sum, dotpath);
    p.x = dotpath[pose][0];
    p.y = dotpath[pose][1]; 
    npoint.points.push_back(p);
    
    // 目標点の設定 //
    pose += ahed;
    if(pose >= sum){ // poseの繰り上げ
      pose -= sum;
    }
    p.x = dotpath[pose][0];
    p.y = dotpath[pose][1];
    gpoint.points.push_back(p);
    
    // 目標点との相対的な角度、距離の算出 //
    alpha = atan2(dotpath[pose][1] - y, dotpath[pose][0] - x) - yaw;
    L = dis(x, y, dotpath[pose][0], dotpath[pose][1]);
    
    // 車速と角速度の算出 //
    cmd.linear.x = vel;
    cmd.angular.z = 2.0 * vel * sin(alpha) / L;
    // naviの停止コマンド //
    if(stap){ 
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd.linear.z = 1;
    }
    
    // pub配信 //
    cmd_pub.publish(cmd);
    marker_pub.publish(points);
    marker_pub.publish(npoint);
    marker_pub.publish(gpoint);
  }
  return 0;
}

// 第1,2引数と第3,4引数の点間距離を算出 //
double dis(double x, double y, double ax, double ay){
  return sqrt(pow(ax-x, 2) + pow(ay-y, 2));
}
// 最も近い点を選択(局所解は無いと仮定) //
int dismin(const double &x, const double &y, int &sum, double dotpath[][2]){
  static int nowpose = 0, lastpose = 0;
  int i, mode = 0, count = 0; // 計算状態変数
  double mindis, nowdis; // 最小経路値、現在経路値

  while(count <= sum){ // 全探索しない限り周回
    for(i=lastpose;i<sum;i++){ // 目標経路の更新
      nowdis = dis(x, y, dotpath[i][0], dotpath[i][1]);
      count++;
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
    if(stap == 0){
      ROS_INFO("NAVI STOPPING!");
      stap = 1;
    }
    else if(stap == 1){
      ROS_INFO("NAVI RESTARTING!");
      stap = 0;
    }
  }
  if(joy_msg.buttons[4]){
    vel += 0.05;
    if(vel > 0.45){
      vel = 0.45;
    }
    ROS_INFO("Now, velocity is [%lf]", vel);
  }
  if(joy_msg.buttons[5]){
    vel -= 0.05;
    if(vel < 0.05){
      vel = 0.05;
    }
    ROS_INFO("Now, velocity is [%lf]", vel);
  }
}
