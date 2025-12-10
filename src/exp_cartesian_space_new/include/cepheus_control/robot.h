#ifndef CEPHEUS_CONTROL_ROBOT_H
#define CEPHEUS_CONTROL_ROBOT_H

// Core includes
#include <deque>
#include <signal.h>

// ROS core
#include <ros/ros.h>

// ROS messages
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

// TF
#include <tf/transform_listener.h>

// Rosbag (logging support)
#include <rosbag/bag.h>

// Utility functions
double moving_average(double new_value, std::deque<double>& window, int size, double& running_sum);
double moving_median(double new_value, std::deque<double>& window, int window_size, double alpha, double smoothed_value);

// Callbacks
void ee_pos_callback(const geometry_msgs::TransformStamped::ConstPtr& msg);
void target_pos_callback(const geometry_msgs::TransformStamped::ConstPtr& msg);
void base_pos_callback(const geometry_msgs::TransformStamped::ConstPtr& msg);
void ls_pos_callback(const std_msgs::Float64::ConstPtr& cmd);
void le_pos_callback(const std_msgs::Float64::ConstPtr& cmd);
void re_pos_callback(const std_msgs::Float64::ConstPtr& cmd);
void ls_vel_vallback(const std_msgs::Float64::ConstPtr& cmd);
void le_vel_callback(const std_msgs::Float64::ConstPtr& cmd);
void re_vel_callback(const std_msgs::Float64::ConstPtr& cmd);
void force_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
void arduino_callback_test(const std_msgs::String::ConstPtr& msg);

// Control and trajectory
void initialise_parameters();
void calculate_trajectory_polynomials(double tf);
void final_trajectories(double t, double tf);
void update_vel(double dt, double t, double tf);
void controller(int count, double tf, double t);

// Logging and shutdown
void log_bag_data(rosbag::Bag& bag);
void maybe_log_bag_data(bool record, rosbag::Bag& bag);
void sigintHandler(int sig);

#endif // CEPHEUS_CONTROL_ROBOT_H
