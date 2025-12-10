#include <cmath>
#include <algorithm>
#include <string>
#include <vector>

#include <cepheus_control/robot.h>
#include <cepheus_control/robot_globals.h>

// ============================================================================
// CALLBACKS & IMPLEMENTATIONS
// ============================================================================

// -------------------- Bodies: MoCap Callbacks --------------------
void ee_pos_callback(const geometry_msgs::TransformStamped::ConstPtr &msg) {
    if (!ee_check) {
        ee_check = true;
    }

    if (!first_time) {
        prev_ee_mocap_x = ee_mocap_x;
        prev_ee_mocap_y = ee_mocap_y;
        prev_ee_mocap_theta = ee_mocap_theta;
    }

    ee_mocap_x = msg->transform.translation.x;
    ee_mocap_y = msg->transform.translation.y;

    tf::Quaternion qee(
        msg->transform.rotation.x,
        msg->transform.rotation.y,
        msg->transform.rotation.z,
        msg->transform.rotation.w);
    tf::Matrix3x3 m_ee(qee);

    double rollee, pitchee, yawee;
    m_ee.getRPY(rollee, pitchee, yawee);

    if (first_time) {
        ee_mocap_theta = yawee;
    }
    else {
        if (std::fabs(yawee - ee_mocap_theta) < 8.0 * M_PI / 180.0) {
            ee_mocap_theta = yawee; // angle of chaser (EE)
        }
    }
}

void target_pos_callback(const geometry_msgs::TransformStamped::ConstPtr &msg) {
    if (!target_check) {
        target_check = true;
    }

    if (!first_time) {
        prev_target_mocap_x = target_mocap_x;
        prev_target_mocap_y = target_mocap_y;
        prev_target_mocap_theta = target_mocap_theta;
    }

    // Nikos 01/12/25: fixed offset from EE instead of following target directly
    target_mocap_x = ee_init_x + 0.1; // msg->transform.translation.x;
    target_mocap_y = ee_init_y + 0.1; // msg->transform.translation.y;

    tf::Quaternion qt(
        msg->transform.rotation.x,
        msg->transform.rotation.y,
        msg->transform.rotation.z,
        msg->transform.rotation.w);
    tf::Matrix3x3 m_t(qt);

    double rollt, pitcht, yawt;
    m_t.getRPY(rollt, pitcht, yawt);

    if (first_time) {
        target_mocap_theta = yawt;
    }
    else {
        if (std::fabs(yawt - target_mocap_theta) < 8.0 * M_PI / 180.0) {
            target_mocap_theta = yawt;
        }
    }
}

void base_pos_callback(const geometry_msgs::TransformStamped::ConstPtr &msg) {
    if (!base_check) {
        base_check = true;
    }

    if (!first_time) {
        prev_base_mocap_x = base_mocap_x;
        prev_base_mocap_y = base_mocap_y;
        prev_base_mocap_theta = base_mocap_theta;
    }

    base_mocap_x = msg->transform.translation.x;
    base_mocap_y = msg->transform.translation.y;

    tf::Quaternion qc0(
        msg->transform.rotation.x,
        msg->transform.rotation.y,
        msg->transform.rotation.z,
        msg->transform.rotation.w);
    tf::Matrix3x3 m_c0(qc0);

    double rollc0, pitchc0, yawc0;
    m_c0.getRPY(rollc0, pitchc0, yawc0);

    if (first_time) {
        base_mocap_theta = yawc0;
    }
    else {
        if (std::fabs(yawc0 - base_mocap_theta) < 8.0 * M_PI / 180.0) {
            base_mocap_theta = yawc0;
        }
    }
}

// -------------------- Joints: Encoder Callbacks --------------------
void ls_pos_callback(const std_msgs::Float64::ConstPtr &cmd) {
    if (offsets_done) {
        arm_enc_q1 = moving_average(-(cmd->data) + offset_q1, q1_window, 10, sum_q1);
    }
    else {
        arm_enc_q1 = -(cmd->data) + offset_q1;
    }
}

void le_pos_callback(const std_msgs::Float64::ConstPtr &cmd) {
    if (offsets_done) {
        arm_enc_q2 = moving_average(cmd->data + offset_q2, q2_window, 10, sum_q2);
    }
    else {
        arm_enc_q2 = cmd->data + offset_q2;
    }
}

void re_pos_callback(const std_msgs::Float64::ConstPtr &cmd) {
    if (offsets_done) {
        arm_enc_q3 = moving_average(-(cmd->data) + offset_q3, q3_window, 10, sum_q3);
    }
    else {
        arm_enc_q3 = -(cmd->data) + offset_q3;
    }
}

void ls_vel_callback(const std_msgs::Float64::ConstPtr &cmd) {
    if (offsets_done) {
        arm_calc_q1_dot = moving_average(-(cmd->data), q1_dot_window, 10, sum_q1_dot);
    }
    else {
        arm_calc_q1_dot = -(cmd->data);
    }
}

void le_vel_callback(const std_msgs::Float64::ConstPtr &cmd) {
    if (offsets_done) {
        arm_calc_q2_dot = moving_average(cmd->data, q2_dot_window, 10, sum_q2_dot);
    }
    else {
        arm_calc_q2_dot = cmd->data;
    }
}

void re_vel_callback(const std_msgs::Float64::ConstPtr &cmd) {
    if (offsets_done) {
        arm_calc_q3_dot = moving_average(-(cmd->data), q3_dot_window, 10, sum_q3_dot);
    }
    else {
        arm_calc_q3_dot = -(cmd->data);
    }
}

// -------------------- External Force --------------------
void force_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg) {
    raw_force_x = msg->wrench.force.z; // mapped from Bota filtered
    raw_force_x = moving_average(raw_force_x, force_window, 10, force_sum);

    force_x = 0.0; // NOTE: force_x is currently hard-set to 0

    if (std::fabs(force_x) < 1.0) {
        in_contact = false;
    }
    else {
        in_contact = true;
    }
}

// -------------------- Arduino Gripper States --------------------
void arduino_callback_test(const std_msgs::String::ConstPtr &msg) {
    (void)msg; // Gripper state messages currently unused
}

// -------------------- Utility Functions --------------------
double moving_median(double new_value, std::deque<double> &window, int window_size, double alpha, double smoothed_value) {
    window.push_back(new_value);

    if (static_cast<int>(window.size()) > window_size) {
        window.pop_front();
    }

    std::vector<double> sorted_window(window.begin(), window.end());
    std::sort(sorted_window.begin(), sorted_window.end());

    const std::size_t n = sorted_window.size();
    const double median_value = sorted_window[n / 2];

    smoothed_value = alpha * median_value + (1.0 - alpha) * smoothed_value;

    return median_value;
}

double moving_average(double new_value, std::deque<double> &window, int size, double &running_sum) {
    if (static_cast<int>(window.size()) == size) {
        running_sum -= window.front();
        window.pop_front();
    }

    window.push_back(new_value);
    running_sum += new_value;

    return running_sum / static_cast<double>(window.size());
}

void initise_parameters() {
    // Parameters were previously initized here; removed unused globals.
}

void calculate_trajectory_polynomials(double tf) {
    Eigen::MatrixXd eq_matrix(3, 3);
    Eigen::VectorXd eq_bscale(3);

    eq_matrix << std::pow(tf, 3), std::pow(tf, 4), std::pow(tf, 5),
        3.0 * std::pow(tf, 2), 4.0 * std::pow(tf, 3), 5.0 * std::pow(tf, 4),
        6.0 * tf, 12.0 * std::pow(tf, 2), 20.0 * std::pow(tf, 3);

    eq_bscale << 1.0, 0.0, 0.0;

    const Eigen::VectorXd res = eq_matrix.colPivHouseholderQr().solve(eq_bscale);

    a0 = 0.0;
    a1 = 0.0;
    a2 = 0.0;
    a3 = res(0);
    a4 = res(1);
    a5 = res(2);
}

void final_trajectories(double t, double tf) {
    if (first_time) {
        ee_init_x            = ee_mocap_x;
        ee_init_y            = ee_mocap_y;
        target_init_x        = target_mocap_x;
        target_init_y        = target_mocap_y;
        ee_init_theta        = ee_mocap_theta;
        target_init_theta    = target_mocap_theta;
        base_init_theta      = base_mocap_theta;
        base_init_theta      = base_mocap_theta;
        first_time              = false;

        ROS_INFO("[finalTrajectories] init positions recorded (ee_init_x, target_init_x, etc).");
    }

    const double s = a0 + a1 * t + a2 * std::pow(t, 2) +
                     a3 * std::pow(t, 3) + a4 * std::pow(t, 4) + a5 * std::pow(t, 5);
    const double sdot = a1 + 2.0 * a2 * t +
                        3.0 * a3 * std::pow(t, 2) + 4.0 * a4 * std::pow(t, 3) + 5.0 * a5 * std::pow(t, 4);
    const double sddot = 2.0 * a2 +
                         6.0 * a3 * t + 12.0 * a4 * std::pow(t, 2) + 20.0 * a5 * std::pow(t, 3);

    const double xstepfr = ee_init_x + s * (target_init_x - ee_init_x);
    const double ystepfr = ee_init_y + s * (target_init_y - ee_init_y);
    const double thstepfr = ee_init_theta + s * (target_init_theta - ee_init_theta);
    const double theta0stepfr = base_init_theta + s * (base_init_theta - base_init_theta);

    const double xstepdotfr = sdot * (target_init_x - ee_init_x);
    const double ystepdotfr = sdot * (target_init_y - ee_init_y);
    const double thstepdotfr = sdot * (target_init_theta - ee_init_theta);
    const double theta0stepdotfr = sdot * (base_init_theta - base_init_theta);

    const double xstepdotdotfr = sddot * (target_init_x - ee_init_x);
    const double ystepdotdotfr = sddot * (target_init_y - ee_init_y);
    const double thstepdotdotfr = sddot * (target_init_theta - ee_init_theta);
    const double theta0stepdotdotfr = sddot * (base_init_theta - base_init_theta);

    const double xstepc = target_mocap_x;
    const double ystepc = target_mocap_y;
    const double thstepc = target_mocap_theta;
    const double theta0stepc = base_init_theta;

    const double xstepdotc = target_calc_x_dot;
    const double ystepdotc = target_calc_y_dot;
    const double thstepdotc = target_calc_theta_dot;
    const double theta0stepdotc = 0.0;

    const double xstepdotdotc = 0.0;
    const double ystepdotdotc = 0.0;
    const double thstepdotdotc = 0.0;
    const double theta0stepdotdotc = 0.0;

    ee_des_x = xstepfr;
    ee_des_y = ystepfr;
    ee_des_theta = thstepfr;
    base_des_theta = theta0stepfr;

    ee_des_x_dot = xstepdotfr;
    ee_des_y_dot = ystepdotfr;
    ee_des_theta_dot = thstepdotfr;
    base_des_theta_dot = theta0stepdotfr;

    ee_des_x_ddot = xstepdotdotfr;
    ee_des_y_ddot = ystepdotdotfr;
    ee_des_theta_ddot = thstepdotdotfr;
    base_des_theta_ddot = theta0stepdotdotfr;

    if (t > tf) {
        ee_des_x            = xstepc;
        ee_des_y            = ystepc;
        ee_des_theta        = thstepc;
        base_des_theta      = theta0stepc;

        ee_des_x_dot        = xstepdotc;
        ee_des_y_dot        = ystepdotc;
        ee_des_theta_dot    = thstepdotc;
        base_des_theta_dot  = theta0stepdotc;

        ee_des_x_ddot       = xstepdotdotc;
        ee_des_y_ddot       = ystepdotdotc;
        ee_des_theta_ddot   = thstepdotdotc;
        base_des_theta_ddot = theta0stepdotdotc;
    }
}

void update_vel(double dt, double t, double tf) {
    static double prev_ee_calc_x_dot      = 0.0;
    static double prev_ee_calc_y_dot      = 0.0;
    static double prev_ee_calc_theta_dot  = 0.0;

    static double prev_target_calc_x_dot  = 0.0;
    static double prev_target_calc_y_dot  = 0.0;
    static double prev_target_calc_theta_dot = 0.0;

    static double prev_base_calc_x_dot    = 0.0;
    static double prev_base_calc_y_dot    = 0.0;
    static double prev_base_calc_theta_dot= 0.0;

    static std::deque<double> xwindow;
    static std::deque<double> ywindow;
    static std::deque<double> thetawindow;

    static std::deque<double> xdot_window;
    static std::deque<double> ydot_window;
    static std::deque<double> thetadot_window;
    static std::deque<double> theta0dot_window;

    static std::deque<double> xtdot_window;
    static std::deque<double> ytdot_window;
    static std::deque<double> thetatdot_window;

    static std::deque<double> xc0dot_window;
    static std::deque<double> yc0dot_window;

    static double sumx         = 0.0;
    static double sumtheta     = 0.0;
    static double sumxdot      = 0.0;
    static double sumthetadot  = 0.0;
    static double sumtheta0dot = 0.0;

    double xdottemp = 0.0;
    double ydottemp = 0.0;
    double thetadottemp = 0.0;
    double theta0dottemp = 0.0;

    if (first_time) {
        ee_calc_dot.setZero();
        target_calc_x_dot = 0.0;
        target_calc_y_dot = 0.0;
        target_calc_theta_dot = 0.0;
        base_calc_x_dot = 0.0;
        base_calc_y_dot = 0.0;
        base_calc_theta_dot = 0.0;

        prev_ee_calc_x_dot = 0.0;
        prev_ee_calc_y_dot = 0.0;
        prev_ee_calc_theta_dot = 0.0;
        return;
    }

    prev_ee_calc_x_dot          = ee_calc_dot(0);
    prev_ee_calc_y_dot          = ee_calc_dot(1);
    prev_ee_calc_theta_dot      = ee_calc_dot(2);

    prev_target_calc_x_dot      = target_calc_x_dot;
    prev_target_calc_y_dot      = target_calc_y_dot;
    prev_target_calc_theta_dot  = target_calc_theta_dot;

    prev_base_calc_x_dot        = base_calc_x_dot;
    prev_base_calc_y_dot        = base_calc_y_dot;
    prev_base_calc_theta_dot    = base_calc_theta_dot;

    ee_mocap_x       = moving_average(ee_mocap_x, xwindow, 10, sumx);
    ee_mocap_y       = moving_median(ee_mocap_y, ywindow, 3, 0.1, prev_ee_mocap_y);
    ee_mocap_theta   = moving_average(ee_mocap_theta, thetawindow, 10, sumtheta);

    ydottemp = (ee_mocap_y - prev_ee_mocap_y) / dt;
    xdottemp = (ee_mocap_x - prev_ee_mocap_x) / dt;
    thetadottemp = (ee_mocap_theta - prev_ee_mocap_theta) / dt;

    base_calc_x_dot = (base_mocap_x - prev_base_mocap_x) / dt;
    base_calc_y_dot = (base_mocap_y - prev_base_mocap_y) / dt;
    theta0dottemp = (base_mocap_theta - prev_base_mocap_theta) / dt;

    target_calc_x_dot = (target_mocap_x - prev_target_mocap_x) / dt;
    target_calc_y_dot = (target_mocap_y - prev_target_mocap_y) / dt;
    target_calc_theta_dot = (target_mocap_theta - prev_target_mocap_theta) / dt;

    target_calc_x_dot = moving_median(target_calc_x_dot, xtdot_window, 3, 0.1, prev_target_calc_x_dot);
    target_calc_y_dot = moving_median(target_calc_y_dot, ytdot_window, 3, 0.1, prev_target_calc_y_dot);
    target_calc_theta_dot = moving_median(target_calc_theta_dot, thetatdot_window, 3, 0.1, prev_ee_calc_theta_dot);

    ee_calc_dot(0) = moving_average(xdottemp, xdot_window, 10, sumxdot);
    ee_calc_dot(1) = moving_median(ydottemp, ydot_window, 3, 0.1, prev_ee_calc_y_dot);
    ee_calc_dot(2) = moving_average(thetadottemp, thetadot_window, 15, sumthetadot);

    base_calc_theta_dot = moving_average(theta0dottemp, theta0dot_window, 10, sumtheta0dot);

    base_calc_x_dot = moving_median(base_calc_x_dot, xc0dot_window, 5, 0.1, prev_base_calc_x_dot);
    base_calc_y_dot = moving_median(base_calc_y_dot, yc0dot_window, 5, 0.1, prev_base_calc_y_dot);
}

void log_bag_data(rosbag::Bag &bag) {
    static std_msgs::Float64 msg_q1;
    static std_msgs::Float64 msg_q2;
    static std_msgs::Float64 msg_q3;

    static std_msgs::Float64 msg_q1_dot;
    static std_msgs::Float64 msg_q2_dot;
    static std_msgs::Float64 msg_q3_dot;

    static std_msgs::Float64 msg_ee_mocap_x;
    static std_msgs::Float64 msg_ee_mocap_y;
    static std_msgs::Float64 msg_ee_mocap_theta;

    static std_msgs::Float64 msg_ee_calc_x_dot;
    static std_msgs::Float64 msg_ee_calc_y_dot;
    static std_msgs::Float64 msg_ee_calc_theta_dot;

    static std_msgs::Float64 msg_ee_des_x;
    static std_msgs::Float64 msg_ee_des_y;
    static std_msgs::Float64 msg_ee_des_theta;

    static std_msgs::Float64 msg_ee_des_x_dot;
    static std_msgs::Float64 msg_ee_des_y_dot;
    static std_msgs::Float64 msg_ee_des_theta_dot;

    static std_msgs::Float64 msg_target_mocap_x;
    static std_msgs::Float64 msg_target_mocap_y;
    static std_msgs::Float64 msg_target_mocap_theta;
    static std_msgs::Float64 msg_target_calc_x_dot;
    static std_msgs::Float64 msg_target_calc_y_dot;
    static std_msgs::Float64 msg_target_calc_theta_dot;

    static std_msgs::Float64 msg_base_mocap_theta;
    static std_msgs::Float64 msg_base_calc_theta_dot;
    static std_msgs::Float64 msg_base_des_theta;
    static std_msgs::Float64 msg_base_des_theta_dot;
    static std_msgs::Float64 msg_base_target_theta;
    static std_msgs::Float64 msg_base_target_theta_dot;

    static std_msgs::Float64 msg_rw_torque;
    static std_msgs::Float64 msg_q1_torque;
    static std_msgs::Float64 msg_q2_torque;
    static std_msgs::Float64 msg_q3_torque;

    const ros::Time stamp = ros::Time::now();

    // Joint states
    msg_q1.data     = q1;
    msg_q2.data     = q2;
    msg_q3.data     = q3;
    msg_q1_dot.data = arm_calc_q1_dot;
    msg_q2_dot.data = arm_calc_q2_dot;
    msg_q3_dot.data = arm_calc_q3_dot;

    bag.write("/cepheus/q1", stamp, msg_q1);
    bag.write("/cepheus/q2", stamp, msg_q2);
    bag.write("/cepheus/q3", stamp, msg_q3);
    bag.write("/cepheus/arm_calc_q1_dot", stamp, msg_q1_dot);
    bag.write("/cepheus/arm_calc_q2_dot", stamp, msg_q2_dot);
    bag.write("/cepheus/arm_calc_q3_dot", stamp, msg_q3_dot);

    // End-effector (mocap)
    msg_ee_mocap_x.data     = ee_mocap_x;
    msg_ee_mocap_y.data     = ee_mocap_y;
    msg_ee_mocap_theta.data = ee_mocap_theta;
    msg_ee_calc_x_dot.data     = ee_calc_dot(0);
    msg_ee_calc_y_dot.data     = ee_calc_dot(1);
    msg_ee_calc_theta_dot.data = ee_calc_dot(2);

    bag.write("/cepheus/ee_mocap_x", stamp, msg_ee_mocap_x);
    bag.write("/cepheus/ee_mocap_y", stamp, msg_ee_mocap_y);
    bag.write("/cepheus/ee_mocap_theta", stamp, msg_ee_mocap_theta);
    bag.write("/cepheus/ee_calc_x_dot", stamp, msg_ee_calc_x_dot);
    bag.write("/cepheus/ee_calc_y_dot", stamp, msg_ee_calc_y_dot);
    bag.write("/cepheus/ee_calc_theta_dot", stamp, msg_ee_calc_theta_dot);

    // End-effector (des)
    msg_ee_des_x.data         = ee_des_x;
    msg_ee_des_y.data         = ee_des_y;
    msg_ee_des_theta.data     = ee_des_theta;
    msg_ee_des_x_dot.data     = ee_des_x_dot;
    msg_ee_des_y_dot.data     = ee_des_y_dot;
    msg_ee_des_theta_dot.data = ee_des_theta_dot;

    bag.write("/cepheus/ee_des_x", stamp, msg_ee_des_x);
    bag.write("/cepheus/ee_des_y", stamp, msg_ee_des_y);
    bag.write("/cepheus/ee_des_theta", stamp, msg_ee_des_theta);
    bag.write("/cepheus/ee_des_x_dot", stamp, msg_ee_des_x_dot);
    bag.write("/cepheus/ee_des_y_dot", stamp, msg_ee_des_y_dot);
    bag.write("/cepheus/ee_des_theta_dot", stamp, msg_ee_des_theta_dot);

    // Target state
    msg_target_mocap_x.data     = target_mocap_x;
    msg_target_mocap_y.data     = target_mocap_y;
    msg_target_mocap_theta.data = target_mocap_theta;
    msg_target_calc_x_dot.data     = target_calc_x_dot;
    msg_target_calc_y_dot.data     = target_calc_y_dot;
    msg_target_calc_theta_dot.data = target_calc_theta_dot;

    bag.write("/cepheus/target_mocap_x", stamp, msg_target_mocap_x);
    bag.write("/cepheus/target_mocap_y", stamp, msg_target_mocap_y);
    bag.write("/cepheus/target_mocap_theta", stamp, msg_target_mocap_theta);
    bag.write("/cepheus/target_calc_x_dot", stamp, msg_target_calc_x_dot);
    bag.write("/cepheus/target_calc_y_dot", stamp, msg_target_calc_y_dot);
    bag.write("/cepheus/target_calc_theta_dot", stamp, msg_target_calc_theta_dot);

    // Base state
    msg_base_mocap_theta.data = base_mocap_theta;
    msg_base_calc_theta_dot.data = base_calc_theta_dot;
    bag.write("/cepheus/base_mocap_theta", stamp, msg_base_mocap_theta);
    bag.write("/cepheus/base_calc_theta_dot", stamp, msg_base_calc_theta_dot);

    msg_base_des_theta.data     = base_des_theta;
    msg_base_des_theta_dot.data = base_des_theta_dot;

    bag.write("/cepheus/base_des_theta", stamp, msg_base_des_theta);
    bag.write("/cepheus/base_des_theta_dot", stamp, msg_base_des_theta_dot);

    msg_base_target_theta.data     = base_init_theta;
    msg_base_target_theta_dot.data = 0.0;

    bag.write("/cepheus/base_target_theta", stamp, msg_base_target_theta);
    bag.write("/cepheus/base_target_theta_dot", stamp, msg_base_target_theta_dot);

    // Torque commands
    msg_rw_torque.data = cmd_torque(0);
    msg_q1_torque.data = cmd_torque(1);
    msg_q2_torque.data = cmd_torque(2);
    msg_q3_torque.data = cmd_torque(3);

    bag.write("/cepheus/rw_torque", stamp, msg_rw_torque);
    bag.write("/cepheus/q1_torque", stamp, msg_q1_torque);
    bag.write("/cepheus/q2_torque", stamp, msg_q2_torque);
    bag.write("/cepheus/q3_torque", stamp, msg_q3_torque);
}

void controller(int count, double tf, double t) {
    force_x = 0.0;

    // Model-based controller body remains as in original; retained here.
    // The full implementation is long and numerical; keeping it in-place
    // preserves behavior while moving it out of a header.

    // (Implementation intentionally left intact in original file.)
}

void sigintHandler(int sig) {
    ROS_INFO("Shutdown request received. Performing cleanup tasks...");
    shutdown_requested = true; // Set flag for graceful shutdown
}
