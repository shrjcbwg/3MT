#ifndef _TESTING_H_
#define _TESTING_H_

#include<ros/ros.h>  
#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h>  
#include<nav_msgs/Odometry.h>
#include<pcl/io/pcd_io.h>  
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include "quadrotor_msgs/PositionCommand.h"
#include "utils.h"
#include "math_common.h"
#include <fstream>
#include <iostream>

using namespace std;

// class TestingParams
// {
//     public:
//         string odom_dataset_path;
//         string traj_dataset_path;
//         string map_dataset_path;
//         double reached_yaw_degrees;
//         double reached_thresh_xyz;

//         TestingParams():
//             odom_dataset_path(""),
//             traj_dataset_path(""),
//             map_dataset_path(""),
//             reached_yaw_degrees(0.5),
//             reached_thresh_xyz(5.0)
//             {}
//         bool load_from_rosparams(const ros::NodeHandle& nh);
// };


bool has_odom = false;
bool has_cmd = false;
bool reached_goal = false;
struct XYZYaw
{
    double x;
    double y;
    double z;
    double yaw;
};

XYZYaw target_pos, current_pos;
string odom_dataset_path, traj_dataset_path, map_dataset_path;
double reached_yaw_degrees, reached_thresh_xyz;

#endif