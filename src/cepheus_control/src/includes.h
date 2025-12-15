#ifndef MY_HEADER1_H
#define MY_HEADER1_H
#include <eigen3/Eigen/Dense>
#//include <Eigen/Dense>
#include "gazebo_msgs/LinkStates.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <tf/transform_listener.h>
// #include "define.h"
#include "geometry_msgs/Wrench.h"
#include <algorithm>
#include <geometry_msgs/WrenchStamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <signal.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <string>
#include <vector>

#define DESIRED_VEL 40 // RW_qdot_des [rad/s]
#define NUM_OF_MEASUREMENTS 1000
#define POS_FILTER 0.005
#define VEL_FILTER 0.05
#define TORQUE_LIMIT 0.00000001
#define MAX_TORQUE 5 // mexri 5Nm na boroun na askithoun

// FT sensor zero offset values
#include <rokubimini_msgs/ResetWrench.h>
#include <ros/service_client.h>

#endif