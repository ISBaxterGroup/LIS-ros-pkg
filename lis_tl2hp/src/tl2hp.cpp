// sep hparray_node.cpp 目標線を手先経路に変換する
// Changing useing arm from flags of left_arm_permit and right_arm_permit
// 角度の急なところは、球面補間してつなぐ
// 布上目標線と物体上目標線と把持点から手先経路を計算する
// Copyright 2017 Naohiro Hayashi <kaminuno@kaminuno-ThinkPad-T440p>
#include "tl2hp.h"

Tl2Hp::Tl2Hp(){}
Tl2Hp::~Tl2Hp(){}

int Tl2Hp::GenHandPath(const  lis_msgs::EndPoses& gp, 
                                       const  geometry_msgs::PoseArray& ctl, 
                                       const  geometry_msgs::PoseArray& otl,
                                       const double kIntHpMinDeg,
                                       const double kIntHpDevideDeg,
                                       const bool   kLeftArmPermit,
                                       const bool   kRightArmPermit,
                                       const bool   kLeftArmIniPose,
                                       const bool   kRightArmIniPose,
                                       const string kFrameName,
                                       lis_msgs::End_PosesArray& hp) 
{
  if (otl.header.frame_id != kFrameName){
     ROS_ERROR("otl is nothing, is otl frame name correct?");
     return -1;
  }
  if (ctl.header.frame_id != kFrameName){
     ROS_ERROR("ctl is nothing, is ctl frame name correct?");
     return -1;
  }
  if (gp.header.frame_id != kFrameName){
     ROS_ERROR("gp is nothing, is gp frame name correct?");
     return -1;
  }

  tf::Transform tf_transform;
  tf::StampedTransform rep;
  tf::StampedTransform lep;
  geometry_msgs::PoseStamped stamped_in;
  geometry_msgs::PoseStamped stamped_out;
  geometry_msgs::Quaternion comp_q;
  ros::Time now;
  double xdeg = 0.0;
  ros::Rate rate(10);
  ros::Rate rate_debug(0.5);

  //------------------------/base座標系から見た初期把持点の格納-------------------------//
  if (kLeftArmPermit == false) {
    stamped_in = createPoseStampedMsg (gp.left, kFrameName, ros::Time::now());

    try{
      listener_.waitForTransform("/base", ros::Time(0), kFrameName, stamped_in.header.stamp, kFrameName, ros::Duration(1.0));
      listener_.transformPose ("/base", ros::Time(0), stamped_in, kFrameName, stamped_out);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }

    hp.left.push_back(stamped_out.pose);
  }

  if (kRightArmPermit == false) {
    stamped_in = createPoseStampedMsg (gp.right, kFrameName, ros::Time::now());

    try{
      listener_.waitForTransform("/base", ros::Time(0), kFrameName, stamped_in.header.stamp, kFrameName, ros::Duration(1.0));
      listener_.transformPose ("/base", ros::Time(0), stamped_in, kFrameName, stamped_out);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }

    hp.right.push_back(stamped_out.pose);
  }
  
  //-----------------------------calcuration of hand path----------------------------------------//
  for (int i=0; i<otl.poses.size(); i++) {
    //円弧補間判定前処理
    if (i == 0) {//初回(i=0)だけは布上目標線始点の姿勢と物体上目標線の姿勢の差を監視する
      xdeg = cal_x2x_orientation(ctl.poses[0].orientation, otl.poses[i].orientation);
    }
    else {
      xdeg = cal_x2x_orientation(otl.poses[i-1].orientation, otl.poses[i].orientation);
    }

    //円弧補間処理
    for (int j = 0; j <= (int)(xdeg/kIntHpDevideDeg); j++) {
      if (xdeg <= kIntHpMinDeg) comp_q = otl.poses[i].orientation;
      else if (i == 0) comp_q = Spherical_Linear_Interpolation(ctl.poses[0].orientation, otl.poses[i].orientation, (double)j*(1.0/(xdeg/kIntHpDevideDeg)));//補間姿勢を計算
      else comp_q = Spherical_Linear_Interpolation(otl.poses[i-1].orientation, otl.poses[i].orientation, (double)j*(1.0/(xdeg/kIntHpDevideDeg)));//補間姿勢を計算

      /*-------------------------Calucuration left--------------------------*/
      if (kLeftArmPermit == false) {
        now = ros::Time::now();
        
        //把持座標系-布上目標線座標系の同次変換行列を計算
        tf::transformMsgToTF(createTransformMsg (otl.poses[i].position, comp_q), tf_transform);
        br_.sendTransform(tf::StampedTransform(tf_transform, now, kFrameName, "/current_object_target_line"));
        
        tf::transformMsgToTF(createTransformMsg (ctl.poses[i].position, ctl.poses[i].orientation), tf_transform);
        br_.sendTransform(tf::StampedTransform(tf_transform, now, kFrameName, "/current_cloth_target_line"));
        
        tf::transformMsgToTF(createTransformMsg (gp.left.position, gp.left.orientation), tf_transform);
        br_.sendTransform(tf::StampedTransform(tf_transform, now, kFrameName, "/left_grasp_pose"));

        rate.sleep();

        try{
          listener_.waitForTransform("/current_cloth_target_line", "/left_grasp_pose", now,  ros::Duration(1.0));
          listener_.lookupTransform("/current_cloth_target_line", "/left_grasp_pose", now, lep);
        }
        catch (tf::TransformException ex) {
          ROS_ERROR("%s",ex.what());
        }

        stamped_in = createPoseStampedMsg (pose_tf2gm(&lep), "/current_object_target_line", otl.header.stamp);
        if (kLeftArmIniPose == false) {//ハンドの姿勢は初期姿勢と同じ姿勢にする
          stamped_in.pose.orientation = gp.left.orientation;
        }

        try{
          listener_.waitForTransform("/base", "/current_object_target_line", now, ros::Duration(1.0));
          listener_.transformPose ("/base", now, stamped_in, "/current_object_target_line", stamped_out);
        }
        catch (tf::TransformException ex) {
          ROS_ERROR("%s",ex.what());
        }
        
        hp.left.push_back(stamped_out.pose);

      }

      /*-------------------------Calucuration right--------------------------*/
      if (kRightArmPermit == false) {
        now = ros::Time::now();
        
        //把持座標系-布上目標線座標系の同次変換行列を計算
        tf::transformMsgToTF(createTransformMsg (otl.poses[i].position, comp_q), tf_transform);
        br_.sendTransform(tf::StampedTransform(tf_transform, now, kFrameName, "/current_object_target_line"));
        
        tf::transformMsgToTF(createTransformMsg (ctl.poses[i].position, ctl.poses[i].orientation), tf_transform);
        br_.sendTransform(tf::StampedTransform(tf_transform, now, kFrameName, "/current_cloth_target_line"));
        
        tf::transformMsgToTF(createTransformMsg (gp.right.position, gp.right.orientation), tf_transform);
        br_.sendTransform(tf::StampedTransform(tf_transform, now, kFrameName, "/right_grasp_pose"));

        rate.sleep();

        try{
          listener_.waitForTransform("/current_cloth_target_line", "/right_grasp_pose", now,  ros::Duration(1.0));
          listener_.lookupTransform("/current_cloth_target_line", "/right_grasp_pose", now, rep);
        }
        catch (tf::TransformException ex) {
          ROS_ERROR("%s",ex.what());
        }

        stamped_in = createPoseStampedMsg (pose_tf2gm(&rep), "/current_object_target_line", otl.header.stamp);
        stamped_in.pose.orientation = gp.right.orientation;

        try{
          listener_.waitForTransform("/base", "/current_object_target_line", now, ros::Duration(1.0));
          listener_.transformPose ("/base", now, stamped_in, "/current_object_target_line", stamped_out);
        }
        catch (tf::TransformException ex) {
          ROS_ERROR("%s",ex.what());
        }

        hp.right.push_back(stamped_out.pose);
      }
      /*--------------------------------------------------------------- -----*/
      if (xdeg <= kIntHpMinDeg) break;
    }//j-roop Interpolation
  }//i-roop

  hp.header = stamped_out.header;

  return 1;
}