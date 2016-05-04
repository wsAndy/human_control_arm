#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include  "ros/ros.h"
#include  "tf/tf.h"
#include "robot_state_publisher/joint_state_listener.h"
#include "robot_state_publisher/robot_state_publisher.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"

#include "ros/console.h"
#include "iostream"
#include "time.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include <map>

#include "explorer_msgs/explorer_arm.h"   
#include "skeleton_tracker/user_Joints.h"


using namespace std;

class HumanControlArm
{
public:
  HumanControlArm();
  ~HumanControlArm();

  ros::Subscriber joint_sub_camera; // get from openni2
  ros::Subscriber joint_sub_robot;  
  ros::Publisher  joint_pub_;
  ros::Publisher  arm_pub;

  vector<string> joint_save;
  map<string,double> joint_data;

  void jointGet(const sensor_msgs::JointStateConstPtr& robot_joint);
  void getFromCamera(const skeleton_tracker::user_JointsConstPtr& human_joint);

  double det_theta_below;
  double det_theta_abrove;

  double theta_below_init;
  double theta_abrove_init;
  
  double theta_below_;
  double theta_abrove_;

  string joint_name;
  string camera_topic;

  void init_data(geometry_msgs::Point & ,geometry_msgs::Point &,geometry_msgs::Point &);
  void init_joint_data();

  int count ;
  int cycle;
  int INIT ;
  int N ;

  int threshold_start;
  int threshold_rec;

  geometry_msgs::Point A0,B0,C0,A1,B1,C1;

};


#endif
