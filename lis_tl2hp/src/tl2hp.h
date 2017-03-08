// tl2hp class 目標線を手先経路に変換する
// Changing useing arm from flags of left_arm_permit and right_arm_permit
// 角度の急なところは、球面補間してつなぐ
// 布上目標線と物体上目標線と把持点から手先経路を計算する
// Copyright 2017 Naohiro Hayashi <kaminuno@kaminuno-ThinkPad-T440p>
// #ifndef LIS_GRAPH_PLANNING_DIJKSTRA_H_
#ifndef TL2HP_H_
#define TL2HP_H_
#include <iostream>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <lis_msgs/End_PosesArray.h>
#include <lis_msgs/EndPoses.h>
#include <lis_msgs/lis.h>
#include <msg_helpers.h>
#include <std_msgs/Int16.h>
using namespace std;
using namespace object_manipulator;
using namespace msg;

class Tl2Hp{
 private:
  //TF関係
  tf::TransformListener listener_;
  tf::TransformBroadcaster br_;
  
 public:
  Tl2Hp();
  ~Tl2Hp();

  /* ------ Tl2Hp.GenHandPath関数の引数 ----------
  const lis_msgs::EndPoses gp;//把持点
  const geometry_msgs::PoseArray ctl;//布上目標線
  const geometry_msgs::PoseArray otl;//物体上目標線
  const double kIntHpMinDeg;//角度が何度以上なら円弧補間を行うか？
  const double kIntHpDevideDeg;//何度刻みで円弧補間を行うか？
  const bool kLeftArmPermit;//左手の手先経路を生成を禁止するか？
  const bool kRightArmPermit;//右手の手先経路を禁止するか？
  const bool kLeftArmIniPose;//左手先経路の姿勢は初期姿勢と同じ姿勢にするか？
  const bool kRightArmIniPose;//右手先経路の姿勢は初期姿勢と同じ姿勢にするか？
  const string kFrameName;//目標線や初期は自位置は何座標系から見たものか？
  lis_msg::End_PosesArray //hand path
  return success 0  error -1 
  */
  int GenHandPath(const  lis_msgs::EndPoses& gp, 
                                       const  geometry_msgs::PoseArray& ctl, 
                                       const  geometry_msgs::PoseArray& otl,
                                       const double kIntHpMinDeg,
                                       const double kIntHpDevideDeg,
                                       const bool   kLeftArmPermit,
                                       const bool   kRightArmPermit,
                                       const bool   kLeftArmIniPose,
                                       const bool   kRightArmIniPose,
                                       const string kFrameName,
                                       lis_msgs::End_PosesArray& hp);
};
#endif  // TL2HP_H_