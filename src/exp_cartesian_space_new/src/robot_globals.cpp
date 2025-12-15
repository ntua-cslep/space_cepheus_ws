#include <cepheus_control/robot_globals.h>

// Torque commands
Eigen::Vector4d cmd_torque(0.0, 0.0, 0.0, 0.0);
Eigen::Vector4d prev_cmd_torque(0.0, 0.0, 0.0, 0.0);

std_msgs::Float64 msg_base_rw;
std_msgs::Float64 msg_arm_shoulder;
std_msgs::Float64 msg_arm_elbow;
std_msgs::Float64 msg_arm_wrist;

// Base State
double base_mocap_x             = 0.0;
double base_mocap_y             = 0.0;
double base_mocap_theta         = 0.0;

double prev_base_mocap_x        = 0.0;
double prev_base_mocap_y        = 0.0;
double prev_base_mocap_theta    = 0.0;

double base_calc_x_dot          = 0.0;
double base_calc_y_dot          = 0.0;
double base_calc_theta_dot      = 0.0;

double base_init_theta          = 0.0;

double base_des_theta           = 0.0;
double base_des_theta_dot       = 0.0;
double base_des_theta_ddot      = 0.0;

// Arm state
double arm_enc_q1               = 0.0;
double arm_enc_q2               = 0.0;
double arm_enc_q3               = 0.0;

double arm_calc_q1_dot          = 0.0;
double arm_calc_q2_dot          = 0.0;
double arm_calc_q3_dot          = 0.0;

double offset_q1                 = 0.0;
double offset_q2                 = 0.0;
double offset_q3                 = 0.0;

double q1_known                  = 1.84447;
double q2_known                  = -0.9542;
double q3_known                  = -0.8393;

// End-effector state
double ee_mocap_x               = 0.0;
double ee_mocap_y               = 0.0;
double ee_mocap_theta           = 0.0;

double prev_ee_mocap_x          = 0.0;
double prev_ee_mocap_y          = 0.0;
double prev_ee_mocap_theta      = 0.0;

Eigen::Vector3d ee_calc_dot(0.0, 0.0, 0.0);

double ee_init_x                = 0.0;
double ee_init_y                = 0.0;
double ee_init_theta            = 0.0;

double ee_des_x                 = 0.0;
double ee_des_y                 = 0.0;
double ee_des_theta             = 0.0;

double ee_des_x_dot             = 0.0;
double ee_des_y_dot             = 0.0;
double ee_des_theta_dot         = 0.0;

double ee_des_x_ddot            = 0.0;
double ee_des_y_ddot            = 0.0;
double ee_des_theta_ddot        = 0.0;

// Target state
double target_mocap_x           = 0.0;
double target_mocap_y           = 0.0;
double target_mocap_theta       = 0.0;

double prev_target_mocap_x      = 0.0;
double prev_target_mocap_y      = 0.0;
double prev_target_mocap_theta  = 0.0;

double target_calc_x_dot        = 0.0;
double target_calc_y_dot        = 0.0;
double target_calc_theta_dot    = 0.0;

double target_init_x            = 0.0;
double target_init_y            = 0.0;
double target_init_theta        = 0.0;

// Force sensor
double fts_force_z                  = 0.0;
double raw_force_x              = 0.0;
double force_sum                = 0.0;

// Control and state flags
bool ee_check                   = false;
bool base_check                 = false;
bool target_check               = false;

bool reached_target             = false;
bool in_contact                 = false;
bool grab_started               = false;

bool first_time                 = true;
bool offsets_done               = false;
bool start_movement             = false;

// Trajectory coefficients
double a0, a1, a2, a3, a4, a5;

// Filter stuff
double sum_q1 = 0.0;
double sum_q2 = 0.0;
double sum_q3 = 0.0;

double sum_q1_dot = 0.0;
double sum_q2_dot = 0.0;
double sum_q3_dot = 0.0;

std::deque<double> q1_window;
std::deque<double> q2_window;
std::deque<double> q3_window;

std::deque<double> q1_dot_window;
std::deque<double> q2_dot_window;
std::deque<double> q3_dot_window;

std::deque<double> force_window;

// ROS message storage
geometry_msgs::Wrench   base_wrench;
std_msgs::Float64       temp_msg;

std_msgs::String        arduino_msg;

std_msgs::Bool          start_moving;
std_msgs::Bool          start_grab_msg;

volatile sig_atomic_t shutdown_requested = 0;
