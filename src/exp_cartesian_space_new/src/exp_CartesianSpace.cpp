#include <cmath>
#include <iostream>
#include <string>

#include <cepheus_control/robot.h>
#include <cepheus_control/robot_globals.h>
#include <cepheus_control/robot_functions.h>

#include "robot_globals.cpp"
#include "robot_functions.cpp"

int main(int argc, char** argv) {

    int   count      = 0;
    bool  hasbegun   = false;
    bool  paramsinit = false;

    int    contactCounter = 0;
    double tsoft          = 0.0;

    target_check = false;
    ee_check     = false;
    base_check   = false;

    /* ROS init */
    ros::init(argc, argv, "cartesian_node");  
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);
    ros::Rate loop_rate(100);  // 100 Hz

    rosbag::Bag bag;
    std::string path = "/home/cepheus/cepheus_ws_v2/bags/";  
    std::string bag_file_name;
    ROS_INFO("[Cartesian Node]: Provide bag name:");
    std::cin >> bag_file_name;
    bag.open(path + bag_file_name + ".bag", rosbag::bagmode::Write);

    ros::Time     curr_time, t_beg;
    ros::Duration dur_time;
    double        secs = 0.0;

    double tf = 0.0;  // total motion time until reaching target

    /* -------------------- Publishers -------------------- */

    // Debugging topics
    ros::Publisher error_x_pub    = nh.advertise<std_msgs::Float64>("/cepheus/error_x", 1);
    ros::Publisher error_y_pub    = nh.advertise<std_msgs::Float64>("/cepheus/error_y", 1);
    ros::Publisher error_theta_pub= nh.advertise<std_msgs::Float64>("/cepheus/error_theta", 1);

    ros::Publisher xd_x_pub       = nh.advertise<std_msgs::Float64>("/cepheus/xd_x", 1);
    ros::Publisher xd_y_pub       = nh.advertise<std_msgs::Float64>("/cepheus/xd_y", 1);
    ros::Publisher xd_theta_pub   = nh.advertise<std_msgs::Float64>("/cepheus/xd_theta", 1);

    ros::Publisher xt_x_pub       = nh.advertise<std_msgs::Float64>("/cepheus/xt_x", 1);
    ros::Publisher xt_y_pub       = nh.advertise<std_msgs::Float64>("/cepheus/xt_y", 1);
    ros::Publisher xt_theta_pub   = nh.advertise<std_msgs::Float64>("/cepheus/xt_theta", 1);

    ros::Publisher xee_x_pub      = nh.advertise<std_msgs::Float64>("/cepheus/xee_x", 1);
    ros::Publisher xee_y_pub      = nh.advertise<std_msgs::Float64>("/cepheus/xee_y", 1);
    ros::Publisher xee_theta_pub  = nh.advertise<std_msgs::Float64>("/cepheus/xee_theta", 1);

    // Torques: Shoulder, Elbow, Wrist, Reaction Wheel
    ros::Publisher ls_torque_pub  = nh.advertise<std_msgs::Float64>("set_left_shoulder_effort", 1); 
    ros::Publisher le_torque_pub  = nh.advertise<std_msgs::Float64>("set_left_elbow_effort", 1);    
    ros::Publisher re_torque_pub  = nh.advertise<std_msgs::Float64>("set_right_elbow_effort", 1);   
    ros::Publisher rw_torque_pub  = nh.advertise<std_msgs::Float64>("cmd_torque", 1);               

    // Misc
    ros::Publisher start_moving_pub   = nh.advertise<std_msgs::Bool>("start_moving", 1);

    // Trigger for Arduino grasp node (through arduino_test.cpp)
    ros::Publisher grab_pub           = nh.advertise<std_msgs::Bool>("start_grab", 1);


    /* -------------------- Subscribers  -------------------- */

    // Joint Encoder Positions
    ros::Subscriber ls_pos_sub = nh.subscribe("read_left_shoulder_position", 1, ls_pos_callback);
    ros::Subscriber le_pos_sub = nh.subscribe("read_left_elbow_position", 1,    le_pos_callback);
    ros::Subscriber re_pos_sub = nh.subscribe("read_right_elbow_position", 1,   re_pos_callback);

    // Joint Encoder Velocities
    ros::Subscriber ls_vel_sub = nh.subscribe("read_left_shoulder_velocity", 1, ls_vel_callback);
    ros::Subscriber le_vel_sub = nh.subscribe("read_left_elbow_velocity", 1,    le_vel_callback);
    ros::Subscriber re_vel_sub = nh.subscribe("read_right_elbow_velocity", 1,   re_vel_callback);

    // Vicon: Base, End-Effector, Target
    ros::Subscriber ee_pos_sub     = nh.subscribe("/vicon/new_cepheus_endeffector/new_cepheus_endeffector", 1, ee_pos_callback);
    ros::Subscriber target_pos_sub = nh.subscribe("/vicon/new_target/new_target", 1,                           target_pos_callback);
    ros::Subscriber base_pos_sub   = nh.subscribe("/vicon/new_cepheusbase/new_cepheusbase", 1,                 base_pos_callback);

    // Force-torque sensor
    ros::Subscriber ft_test_sub = nh.subscribe("/bus0/ft_sensor0/ft_sensor_readings/wrench", 1, force_callback);


    /* -------------------- Init messages -------------------- */
    ROS_INFO("[Cartesian Node]: Node initialized.");

    char command;
    char cmd;

    reached_target   = false;
    start_movement   = false;
    first_time       = true;


    // Arm Homing > Make Function
    while (!offsets_done && ros::ok() && !shutdown_requested) {
        ROS_INFO("[Cartesian Node]: Press Y to calculate angle offsets.");
        std::cin >> cmd;

        if (cmd == 'Y') {
            ros::spinOnce();

            offset_q1 = q1_known - arm_enc_q1;
            offset_q2 = q2_known - arm_enc_q2;
            offset_q3 = q3_known - arm_enc_q3;

            ROS_INFO("Angle offsets have been calculated.");
            offsets_done = true;
        }
        ros::Duration(2.0).sleep();
    }


    ROS_INFO("[Cartesian Node]: Provide tf (total motion time) before proceeding.");
    std::cin >> tf;

    /* -------------------- Main loop -------------------- */
    while (ros::ok() && !shutdown_requested) {
        if (!start_movement) {
            ROS_INFO("[Cartesian Node]: Press Y to start the controller.");
            std::cin >> command;

            ros::spinOnce();
            loop_rate.sleep();

            if (command == 'Y') start_movement = true;
        
        } else {
            if (!hasbegun) {
                ROS_INFO("[Cartesian Node]: initizing control procedure.");
                hasbegun = true;
                t_beg    = ros::Time::now();
            }

            if (!paramsinit) {
                initise_parameters();
                ROS_INFO("[Cartesian Node]: initizing parameters...");
                calculate_trajectory_polynomials(tf);
                paramsinit = true;
                ROS_INFO("[Cartesian Node]: Parameters initized.");
                ROS_INFO("[Cartesian Node]: initizing movement...");
            }

            // --- MAIN CONTROL BODY ---
            ros::spinOnce();

            curr_time = ros::Time::now();
            dur_time  = curr_time - t_beg;
            secs      = dur_time.sec + dur_time.nsec * std::pow(10.0, -9.0);

            update_vel(0.01, secs, tf); // todo: measure dt properly
            final_trajectories(secs, tf);       // min-jerk trajectory + init of some vars
            controller(count, tf, secs);       // impedance controller

            ++count;

            //Make function
            // Publish torques
            msg_base_rw.data        = cmd_torque(0);
            msg_arm_shoulder.data   = cmd_torque(1);
            msg_arm_elbow.data      = cmd_torque(2);
            msg_arm_wrist.data      = cmd_torque(3);

            rw_torque_pub.publish(msg_base_rw);
            ls_torque_pub.publish(msg_arm_shoulder);
            le_torque_pub.publish(msg_arm_elbow);
            re_torque_pub.publish(msg_arm_wrist);

            // Make Function 
            // Contact / grasp trigger
            if (secs > tf && (std::fabs(ee_mocap_x - target_mocap_x) < 0.05) &&  (std::fabs(ee_mocap_y - target_mocap_y) < 0.05)) contactCounter++;

            if (contactCounter > 1 * 100) {  // ~1 second at 100 Hz
                if (!grab_started) {
                    grab_started = true;
                    start_grab_msg.data = true;
                    grab_pub.publish(start_grab_msg);   // trigger grasp sequence in arduino_test.cpp
                    std::cout << "tsoft is: " << secs << " sec." << std::endl;
                }
            }

            // Log data
            log_bag_data(bag);
        }
        loop_rate.sleep();
    }

    bag.close();
    return 0;
}
