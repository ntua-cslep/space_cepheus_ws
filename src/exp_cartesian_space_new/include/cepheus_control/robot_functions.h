#pragma once

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Float64.h>

// -------------------------------------------------------
// GLOBAL STATE ACCESS (declared in robot_globals.cpp/h)
// -------------------------------------------------------

// Joint positions (from robot_globals.cpp)
extern double a1;
extern double a2;
extern double a3;

// Joint velocities (if needed)
extern double a1_dot;
extern double a2_dot;
extern double a3_dot;

// -------------------------------------------------------
// FUNCTION DECLARATIONS FROM robot_functions.cpp
// -------------------------------------------------------

void log_bag_data(rosbag::Bag& bag);

void initialise_parameters();

// Callback declarations (as seen in your code)
void le_vel_callback(const std_msgs::Float64::ConstPtr& msg);
void ls_vel_callback(const std_msgs::Float64::ConstPtr& msg);
void le_pos_callback(const std_msgs::Float64::ConstPtr& msg);
void ls_pos_callback(const std_msgs::Float64::ConstPtr& msg);

void lw_vel_callback(const std_msgs::Float64::ConstPtr& msg);
void lw_pos_callback(const std_msgs::Float64::ConstPtr& msg);


