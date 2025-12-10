#ifndef CEPHEUS_CONTROL_ROBOT_GLOBALS_H
#define CEPHEUS_CONTROL_ROBOT_GLOBALS_H

#include <deque>
#include <signal.h>

#include <eigen3/Eigen/Dense>

#include <geometry_msgs/Wrench.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

// Torque commands
extern Eigen::Vector4d cmd_torque;
extern Eigen::Vector4d prev_cmd_torque;

extern std_msgs::Float64 msg_base_rw;
extern std_msgs::Float64 msg_arm_shoulder;
extern std_msgs::Float64 msg_arm_elbow;
extern std_msgs::Float64 msg_arm_wrist;

// Base state
extern double base_mocap_x;
extern double base_mocap_y;
extern double base_mocap_theta;
extern double prev_base_mocap_x;
extern double prev_base_mocap_y;
extern double prev_base_mocap_theta;
extern double base_calc_x_dot;
extern double base_calc_y_dot;
extern double base_calc_theta_dot;
extern double base_init_theta;
extern double base_des_theta;
extern double base_des_theta_dot;
extern double base_des_theta_ddot;

// Arm state
extern double arm_enc_q1;
extern double arm_enc_q2;
extern double arm_enc_q3;
extern double arm_calc_q1_dot;
extern double arm_calc_q2_dot;
extern double arm_calc_q3_dot;
extern double offset_q1;
extern double offset_q2;
extern double offset_q3;
extern double q1_known;
extern double q2_known;
extern double q3_known;

// End-effector state
extern double ee_mocap_x;
extern double ee_mocap_y;
extern double ee_mocap_theta;
extern double prev_ee_mocap_x;
extern double prev_ee_mocap_y;
extern double prev_ee_mocap_theta;
extern Eigen::Vector3d ee_calc_dot;
extern double ee_init_x;
extern double ee_init_y;
extern double ee_init_theta;
extern double ee_des_x;
extern double ee_des_y;
extern double ee_des_theta;
extern double ee_des_x_dot;
extern double ee_des_y_dot;
extern double ee_des_theta_dot;
extern double ee_des_x_ddot;
extern double ee_des_y_ddot;
extern double ee_des_theta_ddot;

extern double target_mocap_x;
extern double target_mocap_y;
extern double target_mocap_theta;
extern double prev_target_mocap_x;
extern double prev_target_mocap_y;
extern double prev_target_mocap_theta;
extern double target_calc_x_dot;
extern double target_calc_y_dot;
extern double target_calc_theta_dot;
extern double target_init_x;
extern double target_init_y;
extern double target_init_theta;

// Force sensor
extern double force_x;
extern double raw_force_x;
extern double force_sum;

// Control and state flags
extern bool ee_check;
extern bool base_check;
extern bool target_check;
extern bool reached_target;
extern bool in_contact;
extern bool grab_started;
extern bool first_time;
extern bool stop_motors;
extern bool offsets_done;
extern bool start_movement;

// Trajectory coefficients
extern double a0, a1, a2, a3, a4, a5;

// Filtering sums
extern double sum_q1;
extern double sum_q2;
extern double sum_q3;

extern double sum_q1_dot;
extern double sum_q2_dot;
extern double sum_q3_dot;

// Sliding windows
extern std::deque<double> q1_window;
extern std::deque<double> q2_window;
extern std::deque<double> q3_window;

extern std::deque<double> q1_dot_window;
extern std::deque<double> q2_dot_window;
extern std::deque<double> q3_dot_window;

extern std::deque<double> force_window;

// ROS message storage
extern geometry_msgs::Wrench base_wrench;
extern std_msgs::String  arduino_msg;
extern std_msgs::Bool    start_moving;
extern std_msgs::Bool    start_grab_msg;
extern std_msgs::Float64 temp_msg;

extern volatile sig_atomic_t shutdown_requested;

#endif // CEPHEUS_CONTROL_ROBOT_GLOBALS_H
