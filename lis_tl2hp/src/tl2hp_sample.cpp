// tl2hp_sample.cpp
// Copyright 2017 Naohiro Hayashi <kaminuno@kaminuno-ThinkPad-T440p> 
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <lis_msgs/EndPoses.h>
#include <lis_msgs/End_PosesArray.h>
#include "tl2hp.h"
using namespace std;

class Pipe_path{
  private:
    ros::NodeHandle n_;
	ros::Publisher pub_otlnav_topic_;
	ros::Publisher pub_ctlnav_topic_;
    ros::Publisher pub_lgpnav_topic_;
    ros::Publisher pub_rgpnav_topic_;
    ros::Publisher pub_lhpnav_topic_;
    ros::Publisher pub_rhpnav_topic_;
    string kFrameName_;
	double kDistance_;
	int kNum_;

  public:
    Pipe_path(){
		pub_otlnav_topic_ = n_.advertise<nav_msgs::Path>("display_otl_topic", 10);//物体上目標線のパス
		pub_ctlnav_topic_ = n_.advertise<nav_msgs::Path>("display_ctl_topic", 10);//布上目標線のパス
		pub_lgpnav_topic_ = n_.advertise<geometry_msgs::PoseStamped>("display_lgp_topic", 10);//左手把持点
		pub_rgpnav_topic_ = n_.advertise<geometry_msgs::PoseStamped>("display_rgp_topic", 10);//右手把持点
   		pub_lhpnav_topic_ = n_.advertise<nav_msgs::Path>("display_lhp_topic", 10);//左手先経路のパス
   		pub_rhpnav_topic_ = n_.advertise<nav_msgs::Path>("display_rhp_topic", 10);//右手先経路のパス
		kFrameName_ = "/desk";//目標線や初期grapsingpointは何座標系から見たものか？
		kDistance_ = 0.05;//目標線の間隔
		kNum_ = 2;//箱一辺の経由辺数 
    }

    //z方向目標線
    geometry_msgs::Pose cal_otl_z(int i){
       geometry_msgs::Pose pose;
       pose.position.x = 0.0;
       pose.position.y = 0.0;
       pose.position.z = kDistance_ * i;       

       tf::Quaternion q;
       q.setRPY(0.0, -90.0 * M_PI / 180.0, 0.0);
       pose.orientation.x = q.x();
       pose.orientation.y = q.y();
       pose.orientation.z = q.z();
       pose.orientation.w = q.w();

       return pose;
    }

    //x方向目標線
    geometry_msgs::Pose cal_otl_x(int i){
       geometry_msgs::Pose pose;
       pose.position.x = kDistance_ * i;
       pose.position.y = 0.0;
       pose.position.z = (double)kNum_ * kDistance_;       

       tf::Quaternion q;
       q.setRPY(0.0, 00.0, 0.0);
       pose.orientation.x = q.x();
       pose.orientation.y = q.y();
       pose.orientation.z = q.z();
       pose.orientation.w = q.w();

       return pose;
    }

    geometry_msgs::Pose cal_ctl(int i){
       geometry_msgs::Pose pose;
       pose.position.x = -kDistance_ * i;
       pose.position.y = 0.0;
       pose.position.z = 0.0;       

       tf::Quaternion q;
       q.setRPY(0.0, 180.0 * M_PI / 180.0, 0.0);
       pose.orientation.x = q.x();
       pose.orientation.y = q.y();
       pose.orientation.z = q.z();
       pose.orientation.w = q.w();

       return pose;
    }

    int gen_path(){
    	//把持点の作成
        lis_msgs::EndPoses gp_msg;
        gp_msg.header.frame_id = kFrameName_;
        gp_msg.left.position.x = -0.3;
        gp_msg.left.position.y = 0.0;
        gp_msg.left.position.z = 0.0;
		tf::Quaternion q;
		q.setRPY(0.0, 180.0 * M_PI / 180.0, 0.0);
		gp_msg.left.orientation.x = q.x();
		gp_msg.left.orientation.y = q.y();
		gp_msg.left.orientation.z = q.z();
		gp_msg.left.orientation.w = q.w();
        gp_msg.right = gp_msg.left;

        //布上目標線の作成
        geometry_msgs::PoseArray ctl_msg;
        ctl_msg.header.frame_id = kFrameName_;
        for(int i = 0; i < kNum_*2 + 1; i++) ctl_msg.poses.push_back(cal_ctl(i));

        //物体上目標線の作成
        geometry_msgs::PoseArray otl_msg;
        otl_msg.header.frame_id = kFrameName_;
        for(int i = 0; i < kNum_; i++) otl_msg.poses.push_back(cal_otl_z(i));
        for(int i = 0; i < kNum_ + 1; i++) otl_msg.poses.push_back(cal_otl_x(i));


        //-------------------------目標線と把持点から手先経路の生成------------------------//
	    Tl2Hp tl2hp;
        /*
        Tl2Hp.GenHandPath関数の引数
        lis_msgs::EndPoses gp;//把持点
        geometry_msgs::PoseArray ctl;//布上目標線
        geometry_msgs::PoseArray otl;//物体上目標線
	    double kIntHpMinDeg;//角度が何度以上なら円弧補間を行うか？
	    double kIntHpDevideDeg;//何度刻みで円弧補間を行うか？
	    bool kLeftArmPermit;//左手の手先経路を生成を禁止するか？
	    bool kRightArmPermit;//右手の手先経路を禁止するか？
	    bool kLeftArmIniPose;//左手先経路の姿勢は初期姿勢と同じ姿勢にするか？
	    bool kRightArmIniPose;//右手先経路の姿勢は初期姿勢と同じ姿勢にするか？
	    string kFrameName;//目標線や初期は自位置は何座標系から見たものか？
	    return lis_msg::End_PosesArray //hand path
		*/
		const double kIntHpMinDeg = 45.0;
		const double kIntHpDevideDeg= 10.0;
		const bool   kLeftArmPermit = false;
		const bool   kRightArmPermit = true;
		const bool   kLeftArmIniPose = true;
		const bool   kRightArmIniPose = true;
	    lis_msgs::End_PosesArray hp_msg;

	    if(tl2hp.GenHandPath(gp_msg, ctl_msg, otl_msg, 
	    				     kIntHpMinDeg, kIntHpDevideDeg,
	    				     kLeftArmPermit, kRightArmPermit,
	    				     kLeftArmIniPose, kRightArmIniPose, 
	    				     kFrameName_, hp_msg)) {
        	ROS_INFO("SUCCESS to get hp");
	    }
	    else {
	        ROS_ERROR("FAILED to get hp");
	        return -1;
	    }
	    
        //---------------------------------rviz用-------------------------------//
        //grasping point
        geometry_msgs::PoseStamped lgpnav_msg, rgpnav_msg;
		lgpnav_msg.header.stamp = rgpnav_msg.header.stamp = ros::Time::now();
		lgpnav_msg.header.frame_id = rgpnav_msg.header.frame_id = kFrameName_;
		lgpnav_msg.pose = gp_msg.left;
		rgpnav_msg.pose = gp_msg.right;

		//target line
        nav_msgs::Path pub_otlnav_msg, pub_ctlnav_msg;
		pub_otlnav_msg.header.stamp = pub_ctlnav_msg.header.stamp = ros::Time::now();
		pub_otlnav_msg.header.frame_id = pub_ctlnav_msg.header.frame_id = kFrameName_;
		pub_ctlnav_msg.poses.resize(ctl_msg.poses.size());
		pub_otlnav_msg.poses.resize(otl_msg.poses.size());
		for(int i = 0; i < ctl_msg.poses.size(); i++) pub_ctlnav_msg.poses[i].pose = ctl_msg.poses[i];
		for(int i = 0; i < otl_msg.poses.size(); i++) pub_otlnav_msg.poses[i].pose = otl_msg.poses[i];

		//hand path
        nav_msgs::Path  pub_lhpnav_msg, pub_rhpnav_msg;
	    pub_lhpnav_msg.header.stamp = pub_rhpnav_msg.header.stamp = ros::Time::now();
	    pub_lhpnav_msg.header.frame_id = pub_rhpnav_msg.header.frame_id = hp_msg.header.frame_id;
	    pub_lhpnav_msg.poses.resize(hp_msg.left.size());
	    pub_rhpnav_msg.poses.resize(hp_msg.right.size());
	    for(int i = 0; i < hp_msg.left.size(); i++) pub_lhpnav_msg.poses[i].pose = hp_msg.left[i];
	    for(int i = 0; i < hp_msg.right.size(); i++) pub_rhpnav_msg.poses[i].pose = hp_msg.right[i];


		pub_lgpnav_topic_.publish(lgpnav_msg);
		pub_rgpnav_topic_.publish(rgpnav_msg);
		pub_otlnav_topic_.publish(pub_otlnav_msg);
		pub_ctlnav_topic_.publish(pub_ctlnav_msg);
	    pub_lhpnav_topic_.publish(pub_lhpnav_msg);
	    pub_rhpnav_topic_.publish(pub_rhpnav_msg);
		ros::spinOnce();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "box_path_node");

    cout << "Initializing node... " << endl;
    Pipe_path pipe_path;
    pipe_path.gen_path();

    ros::spinOnce();
    
    return 0;
}