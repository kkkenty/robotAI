#include <ros/ros.h>
#include <chjk/unit_cmd.h>
#include <chjk/vw_cmd.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <math.h>
#include <sstream>

#define MIN_DEG 0.001

double alpha,r;
double now_theta;//現在の姿勢
double v_dir;
double target_v[2],target_w;//目標速度
double local_v[2];//local座標系での指令ベクトル
double now_rf[2] = {1,0};
double now_rr[2] = {1,0};
double now_lf[2] = {1,0};
double now_lr[2] = {1,0};

double cmd_rf[2],cmd_rr[2],cmd_lf[2],cmd_lr[2];
double rf_sign = 1;
double rr_sign = 1;
double lf_sign = 1;
double lr_sign = 1;

int standby = 1;
int flag = 1;

ros::Subscriber cmd_recv,pose_recv,standby_recv;

ros::Publisher	left_front_pub,
		left_rear_pub,
		right_front_pub,
		right_rear_pub;

chjk::unit_cmd left_front,left_rear,right_front,right_rear;

void rotation(double *new_v,double *pre_v,double theta);
double InnerProduct(double *vec1, double *vec2, int n);
double Norm(double *vec);
double calc_dir(double *now_point,double *cmd_point);
double calc_deg(double *now_point,double *cmd_point,double *now_deg);
void calc_vel(double *now_point,double *cmd_point,double *sign);

void get_pose(const geometry_msgs::Twist::ConstPtr& pose){
	now_theta = pose->angular.z;
}
void get_standby(const std_msgs::Int64::ConstPtr& msg){
	standby = msg->data;
}
void get_vw(const geometry_msgs::Twist::ConstPtr& msg){
	if (standby == 1)
	{
		right_front.deg = 0;
		right_rear.deg = 0;
		left_front.deg = 0;
		left_rear.deg = 0;

		right_front.vel = 0;
		right_rear.vel = 0;
		left_front.vel = 0;
		left_rear.vel = 0;

		now_rf[0] = 1;
		now_rf[0] = 0;

		now_rr[0] = 1;
		now_rr[0] = 0;

		now_lf[0] = 1;
		now_lf[0] = 0;

		now_lr[0] = 1;
		now_lr[0] = 0;

		rf_sign = 1;
		rr_sign = 1;
		lf_sign = 1;
		lr_sign = 1;
	}else{
		target_v[0] = msg->linear.x;
		target_v[1] = msg->linear.y;
		target_w = msg->angular.z;
		rotation(local_v,target_v,-now_theta);//速度指令をlocal座標変換

		if(!(target_v[0] == 0 && target_v[1] == 0 && target_w == 0) ){

			cmd_rf[0] = local_v[0] - target_w*r*sin(alpha);
			cmd_rf[1] = local_v[1] + target_w*r*cos(alpha);

			cmd_lf[0] = local_v[0] - target_w*r*sin(alpha);
			cmd_lf[1] = local_v[1] - target_w*r*cos(alpha);

			cmd_lr[0] = local_v[0] + target_w*r*sin(alpha);
			cmd_lr[1] = local_v[1] - target_w*r*cos(alpha);

			cmd_rr[0] = local_v[0] + target_w*r*sin(alpha);
			cmd_rr[1] = local_v[1] + target_w*r*cos(alpha);
			
			//InnerProduct(now_point, cmd_point,2)/(Norm(now_point,2)*Norm(cmd_point,2));

			//ROS_FATAL("%lf",Norm(cmd_rr,2));
			//ROS_FATAL("%lf",InnerProduct(now_lr,cmd_lr,2));

			// ステア角度の計算
			calc_deg(now_lf,cmd_lf,&left_front.deg);
			calc_deg(now_lr,cmd_lr,&left_rear.deg);
			calc_deg(now_rr,cmd_rr,&right_rear.deg);
			calc_deg(now_rf,cmd_rf,&right_front.deg);
			//ROS_FATAL("%lf %lf %lf %lf %lf %lf",now_lf[0],now_lf[1],cmd_lf[0],cmd_lf[1],InnerProduct(now_lf, cmd_lf,2) / (Norm(now_lf) * Norm(cmd_lf)),calc_dir(now_lf,cmd_lf));
		}
		if(target_v[0]==0 && target_v[1]==0 && target_w==0){
			// 入力がなければ速度を0にする
			right_front.vel = 0;
			right_rear.vel = 0;
			left_front.vel = 0;
			left_rear.vel = 0;
		}else{
			// 駆動輪の速度計算
			// ここで、車輪の回転方向を見定めている
			calc_vel(now_rf,cmd_rf,&rf_sign);
			calc_vel(now_rr,cmd_rr,&rr_sign);
			calc_vel(now_lf,cmd_lf,&lf_sign);
			calc_vel(now_lr,cmd_lr,&lr_sign);
			// 速度のノルムとその符号を計算して、代入
			right_front.vel = rf_sign * Norm(cmd_rf);
			left_front.vel = lf_sign *  Norm(cmd_lf);
			left_rear.vel = lr_sign *  Norm(cmd_lr);
			right_rear.vel = rr_sign *  Norm(cmd_rr);

			now_rf[0] = cmd_rf[0];
			now_rf[1] = cmd_rf[1];

			now_rr[0] = cmd_rr[0];
			now_rr[1] = cmd_rr[1];

			now_lf[0] = cmd_lf[0];
			now_lf[1] = cmd_lf[1];

			now_lr[0] = cmd_lr[0];
			now_lr[1] = cmd_lr[1];
		}
	}
	left_front_pub.publish(left_front);
	left_rear_pub.publish(left_rear);
	right_rear_pub.publish(right_rear);
	right_front_pub.publish(right_front);

	if(standby == 1 && flag == 1){
		ROS_FATAL("during standby");
		flag = -1;
	}
	else if(standby == -1 && flag == -1){

		ROS_FATAL("Not standby");
		flag = 1;
	}
}

int main(int argc,char **argv){
	ros::init(argc,argv,"chojyuku");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);

	nh.getParam("chjk_node/alpha", alpha);
	alpha = alpha /180 *M_PI;
	nh.getParam("chjk_node/r", r);
	pose_recv = nh.subscribe("spe",1,get_pose);
	cmd_recv = nh.subscribe("cmd",1,get_vw);
	standby_recv = nh.subscribe("standby",1,get_standby);
	left_front_pub = nh.advertise<chjk::unit_cmd>("left_front",1);
	left_rear_pub = nh.advertise<chjk::unit_cmd>("left_rear",1);
	right_front_pub = nh.advertise<chjk::unit_cmd>("right_front",1);
	right_rear_pub = nh.advertise<chjk::unit_cmd>("right_rear",1);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

double calc_dir(double *now_point,double *cmd_point){
	int i = 1;
	if(now_point[0]*cmd_point[1] - now_point[1]*cmd_point[0]>=0){ // 外積のz成分で回転方向を判明する
		 return i;
	}
	else {
		return -i;
	}
}
double calc_deg(double *now_point,double *cmd_point,double *now_deg){
	if(abs(Norm(now_point)*Norm(cmd_point))> 1e-5  ){ // 0で除算させない工夫
		double cos_theta = InnerProduct(now_point, cmd_point,2) / (Norm(now_point) * Norm(cmd_point));
		if(cos_theta>=0 && cos_theta<=1){ // 角度偏差が -90 ~ 90 deg
			*now_deg = *now_deg + calc_dir(now_point,cmd_point) * acos(cos_theta) / M_PI * 180.0; // 角度偏差を加える（角度の正負を考慮）
		}else if(cos_theta<0 && cos_theta>=-1){ // 角度偏差が-180 ~ -90 もしくは 90 ~ 180 deg
			*now_deg = *now_deg - calc_dir(now_point,cmd_point) * (M_PI - acos(cos_theta)) / M_PI * 180.0; // 角度を逆方向に加える
		}
	}
}
void calc_vel(double *now_point,double *cmd_point,double *sign){
	if(!(Norm(now_point)==0 || Norm(cmd_point)==0)){ // 角速度がともに0でない場合
		double cos_theta = InnerProduct(now_point, cmd_point,2); 
		if(cos_theta<0){ // 回転角度が -90 ~ 90 degをはみ出す場合
			*sign = -(*sign); // 速度を反転させる
		}
	}
}
void rotation(double *new_v,double *pre_v,double theta){
	new_v[0] = pre_v[0]*cos(theta) -pre_v[1]*sin(theta); 
	new_v[1] = pre_v[0]*sin(theta) +pre_v[1]*cos(theta);
}
double InnerProduct(double *vec1, double *vec2, int n) {
    int i;
    double s = 0.0;
    for ( i = 0; i < n; i++ ) {
        s += vec1[i] * vec2[i];
    }
    return s;
}
double Norm(double *vec) {
    double s = 0.0;
	s = hypotf(vec[0],vec[1]);
    return s;
}