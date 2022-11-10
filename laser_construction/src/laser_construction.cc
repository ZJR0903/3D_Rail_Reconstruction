/**
* @file calculate_initial_pose.cc
* @author Zhang Jingrao (jingrao_zhang@roboticplus.com)
* @brief Laser guideway reconstruction.
* @version 1.0.0
* @date 2021-05-19
*
* @copyright Copyright (c) roboticplus 2021
*
*/
#ifndef LIB_LASER_CONSTRUCTION_H
#define LIB_LASER_CONSTRUCTION_H 

#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <string>
#include <fstream>
#include <ctime>
#include <bits/stdc++.h> 
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <map> 

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

class LaserConstruction 
{
public:
  LaserConstruction();
  ~LaserConstruction();
  void Execute();
private:
  /** Const **/
  const int ROS_RATE_HZ    = 20;
  const int ADD_POINTCLOUD = 2;
  const double ARM_LENGTH  = 0.94;
  const int REBUILD_TIME   = 120;

  /** Node Handle **/
  ros::NodeHandle n_;
  
  /** Subscribers **/
  ros::Subscriber scan_sub_;
  ros::Subscriber pcd_sub_;
  
  /** Publishers **/
  ros::Publisher pointcloud_pub_;

  /** Tf **/
  tf::TransformBroadcaster tf_odom_;
  tf::TransformBroadcaster tf_map_;
  
  /** Variables **/
  geometry_msgs::Twist conveyor_speed_;
  sensor_msgs::PointCloud scan_points_;
  string pcd_name_;
  
  /** Functions **/
  /**
  *@brief Broadcast TF, calculate pointcloud poses according to conveyor speed and publish the final result to Rviz by freeze-frame footage, then save the picutre in .pcd format.
  */
  void broadcastTf();
  
  /** Callbacks **/
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& Input)
  {
    scan_points_.points.clear();
    geometry_msgs::Point32 temp_point;
    for (int i = 0; i < Input->ranges.size(); ++i) 
    {
      double current_angle = Input->angle_min + i * Input->angle_increment;
      if(Input->ranges[i] <= Input->range_min || Input->ranges[i] >= Input->range_max) 
      {
        continue;
      } 
      temp_point.x = Input->ranges[i] * cos(current_angle);
      temp_point.y = Input->ranges[i] * sin(current_angle);
      scan_points_.points.push_back(temp_point);
    }
    scan_points_.header.frame_id = "laser_link";
    scan_points_.header.stamp = ros::Time::now();
  }

  //Define the file name that it is going to be stored.
  void pcdCallback(const std_msgs::String::ConstPtr& msg) 
  {
    pcd_name_ = msg->data;
  }
};

#endif

LaserConstruction::LaserConstruction() 
{
  scan_sub_       = n_.subscribe("/scan",1, &LaserConstruction::scanCallback,this);
  pcd_sub_        = n_.subscribe("/scan/pcd",1, &LaserConstruction::pcdCallback,this);
  pointcloud_pub_ = n_.advertise<sensor_msgs::PointCloud>("scan_points", 1);
}

LaserConstruction::~LaserConstruction() 
{}

void LaserConstruction::broadcastTf()
{
  //The time of overlie pointcloud.
  double check_time_interval = 1/ADD_POINTCLOUD;
  static bool is_time_update_tf = true;
  static ros::Time last_frame_tf_time = ros::Time::now();
  static ros::Time begin_play_bag_time = ros::Time::now();
  double delta_time = (ros::Time::now() - begin_play_bag_time).toSec();
  static pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b;

  if ((ros::Time::now() - last_frame_tf_time).toSec() > check_time_interval) 
  {
    ROS_INFO("IT IS TIME");
    //Judge if record pointcloud in this frame upon user stated frequency.
    is_time_update_tf = true;
    if (scan_points_.points.size() > 0) 
    { 
      cloud_a.width = scan_points_.points.size();
      cloud_a.height = 1;
      cloud_a.points.resize (cloud_a.width * cloud_a.height);
      for ( size_t i = 0; i < cloud_a.size(); ++i)
      {
        cloud_a.points[i].x = scan_points_.points.at(i).x;
        cloud_a.points[i].y = scan_points_.points.at(i).y - ARM_LENGTH;
        cloud_a.points[i].z = conveyor_speed_.linear.z * delta_time;
      }
      cloud_b += cloud_a;
    }

    //Whenever a new file name is subscribed, save a .pcd format file.
    if (pcd_name_ != "EMPTY") 
    {
      ROS_INFO("recored");
      string pcd_name = pcd_name_ + ".pcd";
      pcl::io::savePCDFileASCII (pcd_name, cloud_b);
      pcd_name_ = "EMPTY";
    }
  }

  //After overlie pointcloud, reset is_time_update_tf for next overlying.
  if (is_time_update_tf) 
  {
    is_time_update_tf = false;  
    last_frame_tf_time = ros::Time::now();
  }

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "map";
  odom_trans.child_frame_id = "odom";
  odom_trans.transform.translation.x = 0;
  odom_trans.transform.translation.y = 0;
  //Overlie scaned pointcloud by freeze-frame foodage in calculated z distance.
  odom_trans.transform.translation.z = conveyor_speed_.linear.z * delta_time;
  geometry_msgs::Quaternion q;
  q = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
  odom_trans.transform.rotation = q;
  tf_odom_.sendTransform(odom_trans);

  if (cloud_b.size() > 0) 
  {
    sensor_msgs::PointCloud material;
    for (int i=0;i<=cloud_b.size()-1;++i) 
    {
      geometry_msgs::Point32 temp_point;
      temp_point.x = cloud_b.points[i].x;
      temp_point.y = cloud_b.points[i].y;
      temp_point.z = cloud_b.points[i].z;
      material.points.push_back(temp_point);
    }
    material.header.frame_id = "map";
    material.header.stamp = ros::Time::now();
    pointcloud_pub_.publish(material);
  }
}

void LaserConstruction::Execute() 
{
  conveyor_speed_.linear.z = 0.025;
	ros::Rate loop_rate(ROS_RATE_HZ);
  pcd_name_ = "EMPTY";
  while (ros::ok()) 
  {
    broadcastTf();

    loop_rate.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char ** argv) 
{
  ros::init(argc, argv, "laser_construction_node");
  LaserConstruction MyVisual;
  MyVisual.Execute();
  return 0;
}
