#include <exp_JointSpace/exp_JointSpace.h>

//------------------------------------- GLOBAL DEFINITIONS & INITIALIZATIONS ---------------------------------------------//
bool shutdown_requested = false;

// Base pose (planar position + yaw) and their derivatives
double x_base      = 0.0;
double y_base      = 0.0;
double xdot_base   = 0.0;
double ydot_base   = 0.0;

double theta0      = 0.0;
double theta0prev  = 0.0;
double theta0dot   = 0.0;
//starst
double x_prev      = 0.0;
double y_prev      = 0.0;

ros::Time base_time_prev;
bool firstTime = true;
double Kcont;

rosbag::Bag bag;
std::string path = "/home/cepheus/cepheus_ws_v2/bags/" ;
std::string bag_file_name;

std_msgs::Float64 msg_rw, msg_ls, msg_le, msg_re;


ros::Publisher base_pose_pub;
ros::Publisher base_twist_pub;
ros::Publisher ls_torque_pub;
ros::Publisher le_torque_pub;
ros::Publisher re_torque_pub;
ros::Publisher rw_torque_pub;
ros::Subscriber base_pos_sub;
ros::Subscriber ls_pos_sub;
ros::Subscriber le_pos_sub;
ros::Subscriber re_pos_sub;
ros::Subscriber ls_vel_sub;
ros::Subscriber le_vel_sub;
ros::Subscriber re_vel_sub;

double q1, q2, q3, th0;
double th0dot,q1dot, q2dot, q3dot;
double q1dot_prev = 0.0;
double q2dot_prev = 0.0;
double q3dot_prev = 0.0;
double offsetq1 = 0.0;
double offsetq2 = 0.0;
double offsetq3 = 0.0;
double rw_max_torque = 0.2;
double a0,a1,a2,a3,a4,a5;
double q1step,q2step,q3step,theta0step;
double q1stepdot,q2stepdot,q3stepdot,theta0stepdot;
double q1stepdotdot,q2stepdotdot,q3stepdotdot,theta0stepdotdot;
double q1in,q2in,q3in,theta0in;
double tfree;
double theta0des,q1des, q2des, q3des;
double maxtorq;
bool firstTimeq1 = true;
bool firstTimeq2 = true;
bool firstTimeq3 = true;
bool offsetsdone = false;
double rawq1, rawq2, rawq3; //angles before the offset addition
const double qfilter = 5*M_PI/180; //5 moires

std::deque<double> q1_window;  // Stores the last N values
std::deque<double> q2_window;  // Stores the last N values
std::deque<double> q3_window;  // Stores the last N values
std::deque<double> q1dot_window;  // Stores the last N values
std::deque<double> q2dot_window;  // Stores the last N values
std::deque<double> q3dot_window;  // Stores the last N values
const int window_size = 10;     // Size of the sliding window
double sumq1 = 0, sumq2 = 0, sumq3 =0;
double sumq1dot = 0, sumq2dot = 0, sumq3dot =  0;
ros::Time theta0_time_prev;

//------------------------------------- HELPER FUNCTIONS---------------------------------------------//
void sigintHandler(int sig) {
    ROS_INFO("Shutdown request received. Performing cleanup tasks...");
    shutdown_requested = true;
    if (bag.isOpen()) bag.close();
    ros::shutdown(); // tells ros::ok() to go false, stops callbacks
}
double clamp (double value, double limit) {
    if (value >  limit) return limit;
    if (value < -limit) return -limit;
    return value;
}
// POSSIBLE PAGKAKIA
double filter_torque(double torq, double prev) { 
	if (torq == 0.0){
		// torq = 0.00001;
		torq = 0.00001;
		if (prev < 0.0)
			torq = torq * -1;
		// printf("CHANGED ZERO TORQUE\n");
	}
	return torq;
}

// in this Bilinear transform filter. It reduces noise, by acknowledging previous vel
double filter_qdot(double qdot_raw, double qdot_prev) { 
	return 0.95 * qdot_prev + (1 - 0.95) * qdot_raw; // 0.95 is coeff alpha
}

double moving_average(double new_value, std::deque<double>& window, int size) {
    window.push_back(new_value);               // Add the new value
    if (window.size() > size) window.pop_front();  // Remove oldest value if window exceeds size
    double sum = 0;
    for (double val : window) sum += val;      // Sum all values in the window
    return sum / window.size();                // Return the average
}


//------------------------------------- CALLBACK FUNCTIONS ---------------------------------------------//
void base_posCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
    // Timestamp from Vicon (or fallback)
    ros::Time t_now = msg->header.stamp;
    if (t_now.isZero()) {
        t_now = ros::Time::now();   // fallback if stamps are missing
    }

    if (!msg) {
        ROS_WARN("Vicon null pointer, ignoring message...");
        return;
    }

    // Position from Vicon (world frame)
    double x_meas = msg->transform.translation.x;
    double y_meas = msg->transform.translation.y;
    // double z_meas = msg->transform.translation.z; // not used for planar

    // Orientation: quaternion -> roll, pitch, yaw
    tf::Quaternion qBase(
        msg->transform.rotation.x,
        msg->transform.rotation.y,
        msg->transform.rotation.z,
        msg->transform.rotation.w);
    tf::Matrix3x3 mBase(qBase);	
    double roll, pitch, yaw;
    mBase.getRPY(roll, pitch, yaw);   // yaw = theta0 (rotation around Z)

    // First valid sample: initialise state, no velocity yet
    if (firstTime) {
        x_base        = x_meas;
        y_base        = y_meas;
        x_prev        = x_meas;
        y_prev        = y_meas;

        theta0        = yaw;
        theta0prev    = yaw;
        theta0dot     = 0.0;
        xdot_base     = 0.0;
        ydot_base     = 0.0;

        base_time_prev = t_now;
        firstTime      = false;
        return;
    }

    // Reject crazy yaw jumps > 30 deg (wrap / marker glitch protection)
    if (std::abs(theta0 - yaw) > 30.0 * M_PI / 180.0) {
        // Ignore this measurement but update time so dt doesn't blow up
        base_time_prev = t_now;
        return;
    }

    // Compute dt from last valid sample
    double dt = (t_now - base_time_prev).toSec();

    if (dt <= 0.0001 || dt >= 0.1) {
        // Bad dt: update pose only, keep old velocities
        x_prev        = x_base;
        y_prev        = y_base;
        x_base        = x_meas;
        y_base        = y_meas;

        theta0prev    = theta0;
        theta0        = yaw;

        base_time_prev = t_now;
        return;
    }

    // Proper finite-difference using previous pose/orientation
    double x_new     = x_meas;
    double y_new     = y_meas;
    double theta_new = yaw;

    double vx  = (x_new     - x_base) / dt;
    double vy  = (y_new     - y_base) / dt;
    double omg = (theta_new - theta0) / dt;

    // Commit update
    x_prev        = x_base;
    y_prev        = y_base;
    x_base        = x_new;
    y_base        = y_new;

    theta0prev    = theta0;
    theta0        = theta_new;

    xdot_base     = vx;
    ydot_base     = vy;
    theta0dot     = omg;

    base_time_prev = t_now;

     // --- PUBLISH BASE STATE ---
    geometry_msgs::Pose2D pose_msg;
    pose_msg.x     = x_base;
    pose_msg.y     = y_base;
    pose_msg.theta = theta0;
    base_pose_pub.publish(pose_msg);

    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x  = xdot_base;
    twist_msg.linear.y  = ydot_base;
    twist_msg.linear.z  = 0.0;          // planar, unused
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = theta0dot;    // yaw rate
    base_twist_pub.publish(twist_msg);
}

void lsPosCallback(const std_msgs::Float64::ConstPtr& cmd) {
    q1 =  -(cmd->data) +  offsetq1;
}
void lePosCallback(const std_msgs::Float64::ConstPtr& cmd) {
    q2 = cmd->data + offsetq2;
}
void rePosCallback(const std_msgs::Float64::ConstPtr& cmd) {
    q3 = -(cmd->data) + offsetq3;
}
void lsVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	q1dot = -(cmd->data);
}
void leVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	q2dot = cmd->data;
}
void reVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	q3dot = -(cmd->data);
}

//------------------------------------- CALCULATIONS ---------------------------------------------// To CHECK
void calculateTrajecotryPolynomials(double tfree){
    Eigen::MatrixXd eq_matrix(3,3);
    Eigen::VectorXd eq_bscale(3);
    // double ts = 0.1*tfree;
    // double wn = 6/ts;
    // kprop_mb = wn*wn;
    // kder_mb = 2*wn; //ta vgala tha ta vro monos mou
    
    
    eq_matrix << pow(tfree,3), pow(tfree,4), pow(tfree,5),
                3*pow(tfree,2), 4*pow(tfree,3), 5*pow(tfree,4),
                6*tfree, 12*pow(tfree,2), 20*pow(tfree,3); 
    
    eq_bscale << 1 , 0, 0;

    Eigen::VectorXd res = eq_matrix.colPivHouseholderQr().solve(eq_bscale);

    a0 = a1 = a2 = 0; //from paper calculations(kostas nanos paper), for t0 = 0
    a3 = res(0);
    a4 = res(1);
    a5 = res(2);
}
void finaltrajectories(double t,double tfree){
  	// if(firstTime){   //initialize the postiion of chaser and target for the first time ONLY , einai se sxolio ta kano initialize stin main
    //     theta0in = theta0;
    //     q1in = q1;
    //     q2in = q2;
    //     q3in = q3;
	// 	ROS_INFO("[Motors test]: First angles have been recorded . \n");
    //     firstTime = false;
	//   }
    double s,sdot, sdotdot;

    s = a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4) + a5*pow(t,5);
    sdot = a1 + 2*a2*t + 3*a3*pow(t,2) + 4*a4*pow(t,3) + 5*a5*pow(t,4);
    sdotdot = 2*a2 + 6*a3*t + 12*a4*pow(t,2) + 20*a5*pow(t,3);


    if(t<=tfree){
       q1step = q1in + s*(q1des - q1in);
       q2step = q2in + s*(q2des - q2in);
       q3step = q3in + s*(q3des - q3in);
       theta0step = theta0in + s*(theta0des - theta0in);
      
       q1stepdot =  sdot*(q1des - q1in);
       q2stepdot =  sdot*(q2des - q2in);
       q3stepdot =  sdot*(q3des - q3in);
       theta0stepdot =  sdot*(theta0des - theta0in);

       q1stepdotdot =  sdotdot*(q1des - q1in);
       q2stepdotdot =  sdotdot*(q2des - q2in);
       q3stepdotdot =  sdotdot*(q3des - q3in);
       theta0stepdotdot =  sdotdot*(theta0des - theta0in);

    // anti gia polyonimo kano step input
    //  q1step = q1des;
    //  q2step = q2des;
    //  q3step = q3des;
    //  theta0step = theta0des;
      
    //  q1stepdot =  0;
    //  q2stepdot =  0;
    //  q3stepdot =  0;
    //  theta0stepdot =  0;

    //  q1stepdotdot =  0;
    //  q2stepdotdot =  0;
    //  q3stepdotdot =  0;
    //  theta0stepdotdot =  0;
      
    }
    else{
      q1step = q1des;
      q2step = q2des;
      q3step = q3des;
      theta0step = theta0des;
      
      q1stepdot =  0;
      q2stepdot =  0;
      q3stepdot =  0;
      theta0stepdot =  0;

      q1stepdotdot =  0;
      q2stepdotdot =  0;
      q3stepdotdot =  0;
      theta0stepdotdot =  0;
    }
}

int main(int argc, char **argv) {

    //------------------------------------- ROS INITIALIZATION ---------------------------------------------//
    ros::init(argc, argv, "motors_test_node");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);

    ls_torque_pub = nh.advertise<std_msgs::Float64>("set_left_shoulder_effort", 1); // Q1
	le_torque_pub = nh.advertise<std_msgs::Float64>("set_left_elbow_effort", 1);  // Q2
  	re_torque_pub = nh.advertise<std_msgs::Float64>("set_right_elbow_effort", 1); // Q3
    rw_torque_pub = nh.advertise<std_msgs::Float64>("cmd_torque", 1);  // REACTION WHEEL

    base_pose_pub  = nh.advertise<geometry_msgs::Pose2D>("base/pose2d", 10);
    base_twist_pub = nh.advertise<geometry_msgs::Twist>("base/twist", 10);

    int count = 0;
    ros::Rate loop_rate(100);
    std_msgs::Float64 torque;
    
    base_pos_sub = nh.subscribe("/vicon/new_cepheusbase/new_cepheusbase", 10, base_posCallback); 
    ls_pos_sub = nh.subscribe("read_left_shoulder_position", 10, lsPosCallback);
	le_pos_sub = nh.subscribe("read_left_elbow_position", 10, lePosCallback);
	re_pos_sub = nh.subscribe("read_right_elbow_position", 10, rePosCallback);
	ls_vel_sub = nh.subscribe("read_left_shoulder_velocity", 10, lsVelCallback);
	le_vel_sub = nh.subscribe("read_left_elbow_velocity", 10, leVelCallback);
	re_vel_sub = nh.subscribe("read_right_elbow_velocity", 10, reVelCallback);

    std_msgs::Float64 msg_torqueq1;
    std_msgs::Float64 msg_torqueq2;
    std_msgs::Float64 msg_torqueq3;
    std_msgs::Float64 msg_torquerw;

    std_msgs::Float64 msg_torqueMBq1;
    std_msgs::Float64 msg_torqueMBq2;
    std_msgs::Float64 msg_torqueMBq3;
    std_msgs::Float64 msg_torqueMBrw;

    std_msgs::Float64 msg_u0;
    std_msgs::Float64 msg_u1;
    std_msgs::Float64 msg_u2;
    std_msgs::Float64 msg_u3;

    std_msgs::Float64 msg_cbar0;
    std_msgs::Float64 msg_cbar1;
    std_msgs::Float64 msg_cbar2;
    std_msgs::Float64 msg_cbar3;

    std_msgs::Float64 msg_Qtest0;
    std_msgs::Float64 msg_Qtest1;
    std_msgs::Float64 msg_Qtest2;
    std_msgs::Float64 msg_Qtest3;

    std_msgs::Float64 msg_q1;
    std_msgs::Float64 msg_q2;
    std_msgs::Float64 msg_q3;
    std_msgs::Float64 msg_theta0;

    std_msgs::Float64 msg_q1dot;
    std_msgs::Float64 msg_q2dot;
    std_msgs::Float64 msg_q3dot;
    std_msgs::Float64 msg_theta0dot;

    std_msgs::Float64 msg_q1d;
    std_msgs::Float64 msg_q2d;
    std_msgs::Float64 msg_q3d;
    std_msgs::Float64 msg_theta0d;

    std_msgs::Float64 msg_q1ddot;
    std_msgs::Float64 msg_q2ddot;
    std_msgs::Float64 msg_q3ddot;
    std_msgs::Float64 msg_theta0ddot;

    std_msgs::Float64 msg_q1ddotdot;
    std_msgs::Float64 msg_q2ddotdot;
    std_msgs::Float64 msg_q3ddotdot;
    std_msgs::Float64 msg_theta0ddotdot;

    std_msgs::Float64 msg_error1;
    std_msgs::Float64 msg_error2;
    std_msgs::Float64 msg_error3;
    std_msgs::Float64 msg_error0;

    std_msgs::Float64 msg_errord1;
    std_msgs::Float64 msg_errord2;
    std_msgs::Float64 msg_errord3;
    std_msgs::Float64 msg_errord0;
    
    char cmd;
    bool initialiseArm = false;

	Eigen::VectorXd errorq(4);
	Eigen::VectorXd errorqdot(4);
    double torq[4];
	double prev_torq[4];
	Eigen::VectorXd torq_MBPD(4);
	Eigen::VectorXd prev_torq_rekl(4);
	double qd[4];
	double qd_dot[4];

    // /gonies analoga me tin gnosti diataksi, ego ekana shoulder thetiko akro, agonas karpos arnitika akra
    double q1known = 1.84447; // ~105deg
    double q2known = -0.9542; // ~-54.7deg      
    double q3known = -0.8393; // ~48.1deg  Palia itan -0.2856 omos allakse otan lysame tin triti arthrosi kai tin ksanadesame

    bool legitChar = false;
    ros::Time curr_time, t_beg;
    ros::Duration dur_time;
    double secs;
    bool record = false;


    
    //---------------------------------------- EXPERIMENT INTITIALIZATION ------------------------------------------//
	for (int i = 0; i < 4; i++) {
		errorq[i] = 0.0;
		errorqdot[i] = 0.0;
		torq[i] = 0.0001;
		prev_torq[i] = 0.0001;
		qd[i] = 0.0;
		qd_dot[i] = 0.0;
	}

    // Safe motors
    torque.data = 0;
    rw_torque_pub.publish(torque);
    ls_torque_pub.publish(torque);
    le_torque_pub.publish(torque);
    re_torque_pub.publish(torque);

    ros::spinOnce();
    ROS_WARN("2025_12_02_18: \n");
    ROS_INFO("[Motors test]: You want to record to a bag? Press Y for yes, anything else for no. \n");
    std::cin>>cmd;
    if(cmd == 'Y'){
        record = true;
        ROS_INFO("[Motors test]: Please provide the name of the bag (dont put .bag). \n");
        std::cin >>  bag_file_name;
        bag.open(path + bag_file_name + ".bag", rosbag::bagmode::Write);
    }
     while(!offsetsdone && ros::ok() && !shutdown_requested){
        ROS_INFO("[Motors test]: Press Y to initialise arm (calculate angle offsets).");
        std::cin>>cmd;
        if(cmd == 'Y'){
            ros::spinOnce();
            loop_rate.sleep();
            offsetq1 = q1known - q1;
            offsetq2 = q2known - q2;
            offsetq3 = q3known - q3;
            ROS_INFO("Arm Initialised.");
            offsetsdone = true;

        }
        ros::Duration(2.0).sleep();
        if (!ros::ok() || shutdown_requested) {
            ROS_WARN("Shutdown before experiment start (offsets).");
            if (bag.isOpen()) bag.close();
            return 0;
        }
    }

    std::cout<<"theta0 is: "<<(theta0*180/M_PI)<<" degrees."<<std::endl; 
    std::cout<<"q1 is: "<<(q1*180/M_PI)<<" degrees."<<std::endl;
    std::cout<<"q2 is: "<<(q2*180/M_PI)<<" degrees."<<std::endl;
    std::cout<<"q3 is: "<<(q3*180/M_PI)<<" degrees."<<std::endl;
    std::cout<<"q1off is: "<<(offsetq1*180/M_PI)<<" degrees."<<std::endl;
    std::cout<<"q2 is: "<<(offsetq2*180/M_PI)<<" degrees."<<std::endl;
    std::cout<<"q3 is: "<<(offsetq3*180/M_PI)<<" degrees."<<std::endl;

    std::cout <<"Give me q1des (degrees): "<<std::endl;
    std::cin >>q1des;
    std::cout <<"Give me q2des (degrees): "<<std::endl;
    std::cin >>q2des;    
    std::cout <<"Give me q3des (degrees): "<<std::endl;
    std::cin >>q3des;

    // -------------------------------------- EXPERIMIENT SETUP --------------------------------------------//
    while(!legitChar && ros::ok() && !shutdown_requested){
        ros::spinOnce();
        std::cout<<"theta0 is: "<<theta0<<std::endl;
        ROS_INFO("[Motors test]:Press Y to start experiment or N not to.");
        std::cin >>cmd;
        if(cmd == 'Y'){
            ROS_INFO("[Motors test]: Please provide tfree. ");
            std::cin>>tfree;
            calculateTrajecotryPolynomials(tfree);
            legitChar = true;
            initialiseArm = true;
            ROS_WARN("Starting Exriment...");
            ros::spinOnce();
            theta0in = theta0;  //to arxiko theta0, thelo na meinei statheri gonia
            q1des = q1des*M_PI/180; //from deg to rad
            q2des = q2des*M_PI/180; //from deg to rad
            q3des = q3des*M_PI/180; //from deg to rad
            std::cout<<"Initial theta0 is: "<<theta0<<std::endl;
            q1in = q1;
            q2in = q2;
            q3in = q3;
            t_beg  = ros::Time::now();
        }
        if(cmd == 'N'){
            legitChar = true;
            initialiseArm = false;
            ROS_WARN("Not running experiment.");
            }
        if (!ros::ok() || shutdown_requested) {
            ROS_WARN("Shutdown before experiment start (offsets).");
            if (bag.isOpen()) bag.close();
            return 0;
        }
    }

    // -------------------------------------- EXPERIMENT LOOP --------------------------------------------//
    while(ros::ok() && !shutdown_requested){
        // Get new data (100hz)
        ros::spinOnce();

        // Run Experiment
        if(initialiseArm){

            // PATCH to fix theta0 and theta0des not being passed to the model based controller
            th0 = theta0;
            theta0des = theta0in;

            curr_time = ros::Time::now();
		    dur_time = curr_time - t_beg;
            secs = dur_time.sec + dur_time.nsec * pow(10, -9);
            finaltrajectories(secs,tfree);

            // velocity filtering, from raw encoder differentiation, before error computation and graphs

            double q1dot_fltr, q2dot_fltr, q3dot_fltr, theta0dot_fltr, q1dot_fltr_prev, q2dot_fltr_prev, q3dot_fltr_prev, theta0dot_fltr_prev;
            q1dot_fltr = filter_qdot(q1dot, q1dot_fltr_prev);
            q2dot_fltr = filter_qdot(q2dot, q2dot_fltr_prev);
            q3dot_fltr = filter_qdot(q3dot, q3dot_fltr_prev);
            // theta0dot_fltr = filter_qdot(theta0dot, theta0dot_fltr_prev);
            q1dot_fltr_prev = q1dot_fltr;
            q2dot_fltr_prev = q2dot_fltr;
            q3dot_fltr_prev = q3dot_fltr;
            // theta0dot_fltr_prev = theta0dot_fltr;

            bool vel_filter = true;
            if (vel_filter) {
                q1dot = q1dot_fltr;
                q2dot = q2dot_fltr;
                q3dot = q3dot_fltr;
                // theta0dot = theta0dot_fltr;
            }

            // errorq[0] = theta0step - theta0;
            errorq[0] = theta0des - theta0;
            errorq[1] = q1step - q1;
            errorq[2] = q2step - q2;
            errorq[3] = q3step - q3;
            
            // errorqdot[0] = theta0stepdot - theta0dot;
            errorqdot[0] = 0 - theta0dot;
            errorqdot[1] = q1stepdot - q1dot;
            errorqdot[2] = q2stepdot - q2dot;
            errorqdot[3] = q3stepdot - q3dot;

            if((secs > tfree+10) && abs(errorq[0])<0.001 && abs(errorq[1])<0.001 && abs(errorq[2])<0.001
                && abs(errorqdot[0])<0.001 && abs(errorqdot[1])<0.001 && abs(errorqdot[2])<0.001 ){
                    ROS_INFO("Initial arm pose established.");
                    initialiseArm = false;
                    torque.data = 0;
                    ls_torque_pub.publish(torque);
                    le_torque_pub.publish(torque);
                    re_torque_pub.publish(torque);
                    break;
                }

            prev_torq[0] = torq[0];
            prev_torq[1] = torq[1];
            prev_torq[2] = torq[2];
            prev_torq[3] = torq[3];


            //Parameters
            double m1 = 0.22983;
            double m2 = 0.38104;
            double m3 = 1.14661;
            double m0 = 19.7;

            double M = m0+m1+m2+m3;

            double q01 = 0;
            double q0 = q01;

            double l1 = 0.185;
            double r1 = 0.37 - l1;

            double l2 = 0.143;
            double r2 = 0.286 - l2;

            double l3 = 0.1436;
            double r3 = 0.075 + 0.03595 + 0.03 + 0.08895 - l3;

            double r0x = 0.1682;
            double r0y = 0;

            double I0z = 0.616;
            double I1z = 0.00284;
            double I2z = 0.002758;
            double I3z = 0.006989;

            double L1 = 0.37;
            double L2 = 0.286;

            // //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            // //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            // //                       Simple Joint-space PD Control
            // //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            // //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            // // torq[0] = 25*errorq[0] + 5*errorqdot[0]; //apo emena afta
            // // torq[1] = -(6*errorq[1] + 0.6*errorqdot[1])/186; //- giati ta mesa einai pros ta arnhtika , to /186 to vlepo pantou ston vraxiona
            // // torq[2] = (6*errorq[2] + 0.6*errorqdot[2])/186;
            // // torq[3] = -(6*errorq[3] + 0.6*errorqdot[3])/186;
        
            double J1 = I1z + I2z + I3z + m1*l1*l1 + m2*(L1+l2)*(L1+l2) + m3*(L1+L2+l3)*(L1+L2+l3);
            double J2 = I2z + I3z + m2*l2*l2 + m3*(L2+l3)*(L2+l3);
            double J3 = I3z + m3*l3*l3;

            // SOS
            double zeta = 0.707; //was 1, but it increases Kd, so maybe causes the jerk
            double ts = 0.8;  //0.2*tfree;
            double wn = 4/(zeta*ts); // 6/ts at zeta = 1;
            //

            double Kp1 = 4;//wn*wn*J1*0.7; //4
            double Kp2 = 4.5;//wn*wn*J2*0.7; //4.5
            double Kp3 = 3.5;//wn*wn*J3*0.7; //3.5

            double Kd1 = 2*sqrt(0.7*J1*Kp1);
            double Kd2 = 2*sqrt(0.7*J2*Kp2);
            double Kd3 = 2*sqrt(0.7*J3*Kp3);

            torq[0] = 0.5*errorq[0] + 2*errorqdot[0];  //0.5 kai 2      //Reference:0.5 kai 2             //15.10.25 ,1.5, 2
            torq[1] = Kp1*errorq[1] + Kd1*errorqdot[1] ;//1.8 KAI 0.6    //Reference:1.8 kai 0.6           //15.10.25 ,6.5, 1.5   //14.11.25, 5.3
            torq[2] = Kp2*errorq[2] + Kd2*errorqdot[2] ;//2.5 kai 0.4    //Reference:1.7 kai 0.4           //15.10.25 ,9, 1.2     //14.11.25, 5.6
            torq[3] = Kp3*errorq[3] + Kd3*errorqdot[3] ;//2.2 kai 1.2    //Reference:2 KAI 1.2 POLY KALA   //15.10.25 ,8, 1.2     //14.11.25, 6.2


            //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            //                       Joint-space Model-based PD Control
            //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

            // double zeta = 1; PIO PANW STO PD
            // double ts = 4; //0.2*tfree;
            // double wn = 6/ts;

            double kp = pow(wn,2); //wn*wn
            double kd = 2*zeta*wn;
            Eigen::MatrixXd Kp(4,4);
            Kp(0,0)= 1* kp;
            Kp(1,1)= 1*kp; //1.1
            Kp(2,2)= 1*kp; //1.4
            Kp(3,3)= 1*kp; //1.8
            Kp(0,1)=0;
            Kp(0,2)=0;
            Kp(0,3)=0;
            Kp(1,0)=0;
            Kp(1,2)=0;
            Kp(1,3)=0;
            Kp(2,0)=0;
            Kp(2,1)=0;
            Kp(2,3)=0;
            Kp(3,0)=0;
            Kp(3,1)=0;
            Kp(3,2)=0;
            Eigen::MatrixXd Kd(4,4);
            Kd(0,0)= 1*kd;
            Kd(1,1)= 1*kd;//1.2*kd;
            Kd(2,2)= 1*kd;//1.3*kd;
            Kd(3,3)= 1*kd;//1.4*kd;
            Kd(0,1)=0;
            Kd(0,2)=0;
            Kd(0,3)=0;
            Kd(1,0)=0;
            Kd(1,2)=0;
            Kd(1,3)=0;
            Kd(2,0)=0;
            Kd(2,1)=0;
            Kd(2,3)=0;
            Kd(3,0)=0;
            Kd(3,1)=0;
            Kd(3,2)=0;

            //*****************************************//

            double B_load=0.02*0.00002;      //0.00002 Nanos // *0.02 //2025_10_14_JointSpace_04: *0 (q2) *0.9(q3)

            double B_motor=0.02*0.000282;   //0.0000282 Nanos //0.02*0.000282

            double J_motor=0.000000885; //2025_10_14_JointSpace_04: *0 (q2) *0.9(q3)

            double n=186;

            // Non-Backdrivability Compensation (NBC)
            double NBCh =  1*n*n*J_motor;// Joint motors' non-backdrivability IS taken into account in the model-based part of the controller, in matrix H
            // double NBCh = 0;// Joint motors' non-backdrivability IS NOT taken into account in the model-based part of the controller, in matrix H
            double NBCc =  0*(B_load+n*n*B_motor);// Joint motors' non-backdrivability IS taken into account in the model-based part of the controller, in vector c
            // double NBCc = 0;// Joint motors' non-backdrivability IS NOT taken into account in the model-based part of the controller, in vector c
            
            //c coefficients
            double c1, c2, c3, c4, c5, c6;

            //h coefficients/0.0000282 Nanos
            double H11, H12, H13, H14, H15, H16,
                H21, H22, H23, H24, H25, H26,
                H31, H32, H33, H34, H35, H36,
                H41, H42, H43, H44, H45, H46,
                H51, H52, H53, H54, H55, H56,
                H61, H62, H63, H64, H65, H66;

            Eigen::VectorXd c(6);
            Eigen::MatrixXd h(6,6);
            
            Eigen::MatrixXd H11star(2,2);
            Eigen::MatrixXd H12star(2,4);
            Eigen::MatrixXd H21star(4,2);
            Eigen::MatrixXd H22star(4,4);

            Eigen::MatrixXd Hbar(4,4);

            Eigen::VectorXd c1star(2);
            Eigen::VectorXd c2star(4);
            Eigen::VectorXd cbar(4);                  // Nikos, 14/11/25, prin itan, Eigen::MatrixXd cbar(4,4);  

            Eigen::VectorXd xdotdot_des(4);
            
            Eigen::VectorXd u(4);
            Eigen::VectorXd Q(4);


            // Basic vectors' dfinition

            xdotdot_des(0) = 0;
            xdotdot_des(1) = q1stepdotdot;
            xdotdot_des(2) = q2stepdotdot;
            xdotdot_des(3) = q3stepdotdot;

            th0dot=theta0dot;

            // System parameter clusters definition

            double p1=M;
            double p2=(m1+m2+m3)*r0x;
            double p3=(m1+m2+m3)*r0y;
            double p4=(m1+m2+m3)*l1+(m2+m3)*r1;
            double p5=(m2+m3)*l2+m3*r2;
            double p6=I0z+(m1+m2+m3)*(r0x*r0x+r0y*r0y);
            double p7=I1z+(m1+m2+m3)*l1*l1+2*(m2+m3)*l1*r1+(m2+m3)*r1*r1;
            double p8=I2z+(m2+m3)*l2*l2+2*m3*l2*r2+m3*r2*r2;
            double p9=I3z+m3*l3*l3;
            double p10=((m1+m2+m3)*l1+(m2+m3)*r1)*r0x;
            double p11=((m1+m2+m3)*l1+(m2+m3)*r1)*r0y;
            double p12=(l1+r1)*((m2+m3)*l2+m3*r2);
            double p13=((m2+m3)*l2+m3*r2)*r0x;
            double p14=((m2+m3)*l2+m3*r2)*r0y;
            double p15=m3*l3;
            double p16=m3*l3*r0x;
            double p17=m3*l3*r0y;
            double p18=(l1+r1)*m3*l3;
            double p19=(l2+r2)*m3*l3;

            //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            //                           Inertia Matrix
            //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            
            H11 = p1;
            H12 = 0;
            H13 = (-1)*p3*cos(th0)+(-1)*p2*sin(th0)+(-1)*p4*sin(q01+q1+th0)+(-1)*p5*sin(q01+q1+q2+th0)+(-1)*p15*sin(q01+q1+q2+q3+th0);
            H14 = (-1)*p4*sin(q01+q1+th0)+(-1)*p5*sin(q01+q1+q2+th0)+(-1)*p15*sin(q01+q1+q2+q3+th0);
            H15 = (-1)*p5*sin(q01+q1+q2+th0)+(-1)*p15*sin(q01+q1+q2+q3+th0);
            H16 = (-1)*p15*sin(q01+q1+q2+q3+th0);
  
            H21 = 0;
            H22 = p1;
            H23 = p2*cos(th0)+p4*cos(q01+q1+th0)+p5*cos(q01+q1+q2+th0)+p15*cos(q01+q1+q2+q3+th0)+(-1)*p3*sin(th0);
            H24 = p4*cos(q1+q0 + th0) + p5*cos(q1+q0 + q2 + th0) + p15*cos(q1+q0 + q2 + q3 + th0);
            H25 = p5*cos(q1+q0 + q2 + th0) + p15*cos(q1+q0 + q2 + q3 + th0);
            H26 = p15*cos(q1+q0 + q2 + q3 + th0);
  
            H31 = (-1)*p3*cos(th0) + (-1)*p2*sin(th0) + (-1)*p4*sin(q1+q0 + th0) + (-1)*p5*sin(q1+q0 + q2 + th0) + (-1)*p15*sin(q1+q0 + q2 + q3 + th0);
            H32 = p2*cos(th0) + p4*cos(q1+q0 + th0) + p5*cos(q1+q0 + q2 + th0) + p15*cos(q1+q0 + q2 + q3 + th0) + (-1)*p3*sin(th0);
            H33 = p6+p7+p8+p9+2*p10*cos(q01+q1)+2*p12*cos(q2)+2*p13*cos(q01+q1+q2)+2*p19*cos(q3)+2*p18*cos(q2+q3)+2*p16*cos(q01+q1+q2+q3)+2*p11*sin(q01+q1)+2*p14*sin(q01+q1+q2)+2*p17*sin(q01+q1+q2+q3);
            H34 = p7+p8+p9+p10*cos(q01+q1)+2*p12*cos(q2)+p13*cos(q01+q1+q2)+2*p19*cos(q3)+2*p18*cos(q2+q3)+p16*cos(q01+q1+q2+q3)+p11*sin(q01+q1)+p14*sin(q01+q1+q2)+p17*sin(q01+q1+q2+q3);
            H35 = p8+p9+p12*cos(q2)+p13*cos(q01+q1+q2)+2*p19*cos(q3)+p18*cos(q2+q3)+p16*cos(q01+q1+q2+q3)+p14*sin(q01+q1+q2)+p17*sin(q01+q1+q2+q3);
            H36 = p9+p19*cos(q3)+p18*cos(q2+q3)+p16*cos(q01+q1+q2+q3)+p17*sin(q01+q1+q2+q3);
  
            H41 = (-1)*p4*sin(q01+q1+th0)+(-1)*p5*sin(q01+q1+q2+th0)+(-1)*p15*sin(q01+q1+q2+q3+th0);
            H42 = p4*cos(q01+q1+th0)+p5*cos(q01+q1+q2+th0)+p15*cos(q01+q1+q2+q3+th0);
            H43 = p7+p8+p9+p10*cos(q01+q1)+2*p12*cos(q2)+p13*cos(q01+q1+q2)+2*p19*cos(q3)+2*p18*cos(q2+q3)+p16*cos(q01+q1+q2+q3)+p11*sin(q01+q1)+p14*sin(q01+q1+q2)+p17*sin(q01+q1+q2+q3);
            H44 = p7+p8+p9+2*p12*cos(q2)+2*p19*cos(q3)+2*p18*cos(q2+q3) + NBCh;   // q1, before 1
            H45 = p8+p9+p12*cos(q2)+2*p19*cos(q3)+p18*cos(q2+q3);
            H46 = p9+p19*cos(q3)+p18*cos(q2+q3);
  
            H51 = (-1)*p5*sin(q01+q1+q2+th0)+(-1)*p15*sin(q01+q1+q2+q3+th0);
            H52 = p5*cos(q01+q1+q2+th0)+p15*cos(q01+q1+q2+q3+th0);
            H53 = p8+p9+p12*cos(q2)+p13*cos(q01+q1+q2)+2*p19*cos(q3)+p18*cos(q2+q3)+p16*cos(q01+q1+q2+q3)+p14*sin(q01+q1+q2)+p17*sin(q01+q1+q2+q3);
            H54 = p8+p9+p12*cos(q2)+2*p19*cos(q3)+p18*cos(q2+q3);
            H55 = p8+p9+2*p19*cos(q3) + NBCh;       //q2, before 0
            H56 = p9+p19*cos(q3);
  
            H61 = (-1)*p15*sin(q01+q1+q2+q3+th0);
            H62 = p15*cos(q01+q1+q2+q3+th0);
            H63 = p9+p19*cos(q3)+p18*cos(q2+q3)+p16*cos(q01+q1+q2+q3)+p17*sin(q01+q1+q2+q3);
            H64 = p9+p19*cos(q3)+p18*cos(q2+q3);
            H65 = p9+p19*cos(q3);
            H66 = p9 + 1*NBCh;        // q3, before 0.85

            //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            //                       Nonlinear velocities vector
            //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -


            c1 = (1/2)*((-2)*p15*q3dot*(q1dot+q2dot+q3dot+th0dot)*cos(q01+q1+ 
                q2+q3+th0)+q2dot*((-2)*p5*(q1dot+q2dot+th0dot)*cos(q01+q1+q2+ 
                th0)+(-2)*p15*(q1dot+q2dot+q3dot+th0dot)*cos(q01+q1+q2+q3+th0)) 
                +q1dot*((-2)*p4*(q1dot+th0dot)*cos(q01+q1+th0)+(-2)*p5*( 
                q1dot+q2dot+th0dot)*cos(q01+q1+q2+th0)+(-2)*p15*(q1dot+q2dot+ 
                q3dot+th0dot)*cos(q01+q1+q2+q3+th0))+th0dot*((-2)*p2*th0dot* 
                cos(th0)+(-2)*p4*(q1dot+th0dot)*cos(q01+q1+th0)+(-2)*p5*( 
                q1dot+q2dot+th0dot)*cos(q01+q1+q2+th0)+(-2)*p15*(q1dot+q2dot+ 
                q3dot+th0dot)*cos(q01+q1+q2+q3+th0)+2*p3*th0dot*sin(th0)));

            c2 = (1/2)*((-2)*p15*q3dot*(q1dot+q2dot+q3dot+th0dot)*sin(q01+q1+ 
                q2+q3+th0)+q2dot*((-2)*p5*(q1dot+q2dot+th0dot)*sin(q01+q1+q2+ 
                th0)+(-2)*p15*(q1dot+q2dot+q3dot+th0dot)*sin(q01+q1+q2+q3+th0)) 
                +q1dot*((-2)*p4*(q1dot+th0dot)*sin(q01+q1+th0)+(-2)*p5*( 
                q1dot+q2dot+th0dot)*sin(q01+q1+q2+th0)+(-2)*p15*(q1dot+q2dot+ 
                q3dot+th0dot)*sin(q01+q1+q2+q3+th0))+th0dot*((-2)*p3*th0dot* 
                cos(th0)+(-2)*p2*th0dot*sin(th0)+(-2)*p4*(q1dot+th0dot)*sin( 
                q01+q1+th0)+(-2)*p5*(q1dot+q2dot+th0dot)*sin(q01+q1+q2+th0)+( 
                -2)*p15*(q1dot+q2dot+q3dot+th0dot)*sin(q01+q1+q2+q3+th0)));

            c3 = p11*q1dot*(q1dot+2*th0dot)*cos(q01+q1)+p14*(q1dot+q2dot)*( 
                q1dot+q2dot+2*th0dot)*cos(q01+q1+q2)+p17*q1dot*q1dot*cos(q01+q1+ 
                q2+q3)+2*p17*q1dot*q2dot*cos(q01+q1+q2+q3)+p17*q2dot*q2dot*cos( 
                q01+q1+q2+q3)+2*p17*q1dot*q3dot*cos(q01+q1+q2+q3)+2*p17* 
                q2dot*q3dot*cos(q01+q1+q2+q3)+p17*q3dot*q3dot*cos(q01+q1+q2+q3)+ 
                2*p17*q1dot*th0dot*cos(q01+q1+q2+q3)+2*p17*q2dot*th0dot* 
                cos(q01+q1+q2+q3)+2*p17*q3dot*th0dot*cos(q01+q1+q2+q3)+(-1)* 
                p10*q1dot*q1dot*sin(q01+q1)+(-2)*p10*q1dot*th0dot*sin(q01+q1)+( 
                -2)*p12*q1dot*q2dot*sin(q2)+(-1)*p12*q2dot*q2dot*sin(q2)+(-2) 
                *p12*q2dot*th0dot*sin(q2)+(-1)*p13*q1dot*q1dot*sin(q01+q1+q2)+ 
                (-2)*p13*q1dot*q2dot*sin(q01+q1+q2)+(-1)*p13*q2dot*q2dot*sin( 
                q01+q1+q2)+(-2)*p13*q1dot*th0dot*sin(q01+q1+q2)+(-2)*p13* 
                q2dot*th0dot*sin(q01+q1+q2)+(-2)*p19*q1dot*q3dot*sin(q3)+( 
                -2)*p19*q2dot*q3dot*sin(q3)+(-1)*p19*q3dot*q3dot*sin(q3)+(-2) 
                *p19*q3dot*th0dot*sin(q3)+(-2)*p18*q1dot*q2dot*sin(q2+q3)+ 
                (-1)*p18*q2dot*q2dot*sin(q2+q3)+(-2)*p18*q1dot*q3dot*sin(q2+ 
                q3)+(-2)*p18*q2dot*q3dot*sin(q2+q3)+(-1)*p18*q3dot*q3dot*sin( 
                q2+q3)+(-2)*p18*q2dot*th0dot*sin(q2+q3)+(-2)*p18*q3dot* 
                th0dot*sin(q2+q3)+(-1)*p16*q1dot*q1dot*sin(q01+q1+q2+q3)+(-2)* 
                p16*q1dot*q2dot*sin(q01+q1+q2+q3)+(-1)*p16*q2dot*q2dot*sin(q01+ 
                q1+q2+q3)+(-2)*p16*q1dot*q3dot*sin(q01+q1+q2+q3)+(-2)*p16* 
                q2dot*q3dot*sin(q01+q1+q2+q3)+(-1)*p16*q3dot*q3dot*sin(q01+q1+ 
                q2+q3)+(-2)*p16*q1dot*th0dot*sin(q01+q1+q2+q3)+(-2)*p16* 
                q2dot*th0dot*sin(q01+q1+q2+q3)+(-2)*p16*q3dot*th0dot*sin( 
                q01+q1+q2+q3);

            c4 = (-1)*p11*th0dot*th0dot*cos(q01+q1)+(-1)*p14*th0dot*th0dot*cos(q01+ 
                q1+q2)+(-1)*p17*th0dot*th0dot*cos(q01+q1+q2+q3)+p10*th0dot*th0dot* 
                sin(q01+q1)+(-2)*p12*q1dot*q2dot*sin(q2)+(-1)*p12*q2dot*q2dot* 
                sin(q2)+(-2)*p12*q2dot*th0dot*sin(q2)+p13*th0dot*th0dot*sin(q01+ 
                q1+q2)+(-2)*p19*q1dot*q3dot*sin(q3)+(-2)*p19*q2dot*q3dot* 
                sin(q3)+(-1)*p19*q3dot*q3dot*sin(q3)+(-2)*p19*q3dot*th0dot* 
                sin(q3)+(-2)*p18*q1dot*q2dot*sin(q2+q3)+(-1)*p18*q2dot*q2dot* 
                sin(q2+q3)+(-2)*p18*q1dot*q3dot*sin(q2+q3)+(-2)*p18*q2dot* 
                q3dot*sin(q2+q3)+(-1)*p18*q3dot*q3dot*sin(q2+q3)+(-2)*p18* 
                q2dot*th0dot*sin(q2+q3)+(-2)*p18*q3dot*th0dot*sin(q2+q3)+ 
                p16*th0dot*th0dot*sin(q01+q1+q2+q3) + q1dot*NBCc;                         // prin sketo NBCc, 21/11/25 Kostas

            c5 = (-1)*p14*th0dot*th0dot*cos(q01+q1+q2)+(-1)*p17*th0dot*th0dot*cos( 
                q01+q1+q2+q3)+p12*q1dot*q1dot*sin(q2)+2*p12*q1dot*th0dot*sin( 
                q2)+p12*th0dot*th0dot*sin(q2)+p13*th0dot*th0dot*sin(q01+q1+q2)+(-2)* 
                p19*q1dot*q3dot*sin(q3)+(-2)*p19*q2dot*q3dot*sin(q3)+(-1)* 
                p19*q3dot*q3dot*sin(q3)+(-2)*p19*q3dot*th0dot*sin(q3)+p18* 
                q1dot*q1dot*sin(q2+q3)+2*p18*q1dot*th0dot*sin(q2+q3)+p18* 
                th0dot*th0dot*sin(q2+q3)+p16*th0dot*th0dot*sin(q01+q1+q2+q3) + q2dot*NBCc;                         // prin sketo NBCc, 21/11/25 Kostas

            c6 = (-1)*p17*th0dot*th0dot*cos(q01+q1+q2+q3)+p19*(q1dot+q2dot+th0dot) 
                *(q1dot+q2dot+th0dot)*sin(q3)+p18*q1dot*q1dot*sin(q2+q3)+2*p18*q1dot*th0dot*sin( 
                q2+q3)+p18*th0dot*th0dot*sin(q2+q3)+p16*th0dot*th0dot*sin(q01+q1+q2+ 
                q3) + 1*q3dot*NBCc;                         // prin sketo NBCc, 21/11/25 Kostas , twra *3 se ola kai meta +-20%


            //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            //                      System matrices formulation
            //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

            h(0,0) = H11;
            h(0,1) = H12;
            h(0,2) = H13;
            h(0,3) = H14;
            h(0,4) = H15;
            h(0,5) = H16;
            //////////////
            h(1,0) = H21;
            h(1,1) = H22;
            h(1,2) = H23;
            h(1,3) = H24;
            h(1,4) = H25;
            h(1,5) = H26;
            /////////////
            h(2,0) = H31;
            h(2,1) = H32;
            h(2,2) = H33;
            h(2,3) = H34;
            h(2,4) = H35;
            h(2,5) = H36;
            /////////////
            h(3,0) = H41;
            h(3,1) = H42;
            h(3,2) = H43;
            h(3,3) = H44;
            h(3,4) = H45;
            h(3,5) = H46;
            ////////////
            h(4,0) = H51;
            h(4,1) = H52;
            h(4,2) = H53;
            h(4,3) = H54;
            h(4,4) = H55;
            h(4,5) = H56;
            /////////////
            h(5,0) = H61;
            h(5,1) = H62;
            h(5,2) = H63;
            h(5,3) = H64;
            h(5,4) = H65;
            h(5,5) = H66;

            c(0) = c1;
            c(1) = c2;
            c(2) = c3;
            c(3) = c4;
            c(4) = c5;
            c(5) = c6;


            H11star << H11, H12,
                        H21, H22;
            H12star << H13, H14, H15, H16,
                        H23, H24, H25, H26;
            H21star << H31, H32,
                        H41, H42,
                        H51, H52,
                        H61, H62;
            H22star << H33, H34, H35, H36,
                        H43, H44, H45, H46,
                        H53, H54, H55, H56,
                        H63, H64, H65, H66;

            c1star << c1, c2;
            c2star << c3, c4, c5, c6;

            Hbar = H22star-H21star*(H11star.inverse())*H12star;

            cbar = c2star-H21star*(H11star.inverse())*c1star;

            //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            //                              PD Controller
            //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

            u = xdotdot_des+Kp*errorq+Kd*errorqdot;

            //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            //                           Model-based PD control
            //  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

            Q = Hbar*u+cbar;


            Eigen::VectorXd Qtest(4);
            Qtest = Hbar*u;// For debugging

            torq_MBPD[0] = Q[0]; 
            torq_MBPD[1] = Q[1]+0.05;    //+0.065;   //+0.1; // Tf=0.1 (static friction compensation)
            torq_MBPD[2] = Q[2]-0.05;    //-0.05;   //-0.1; // Tf=0.1 (static friction compensation)
            torq_MBPD[3] = Q[3] +0.14;    //+0.14;   //+0.1; // Tf=0.025 (static friction compensation)
 
            // -------------------------------------- PUBLISH RESULTS --------------------------------------------//
            bool MBPD_switch = true;
            if(MBPD_switch){
                torq[0] = torq_MBPD[0];
                torq[1] = torq_MBPD[1];
                torq[2] = torq_MBPD[2];
                torq[3] = torq_MBPD[3];
            }

            // Clamp torques to max values to 1 Nm
            torq[1] = clamp(-torq[1], 1);
            torq[2] = clamp(torq[2], 1);
            torq[3] = clamp(-torq[3], 1);
            
            // Publish torques
            msg_torquerw.data = torq[0];
            msg_torqueq1.data = torq[1];
            msg_torqueq2.data = torq[2];
            msg_torqueq3.data = torq[3];

            rw_torque_pub.publish(msg_torquerw);
            ls_torque_pub.publish(msg_torqueq1);
            le_torque_pub.publish(msg_torqueq2);
            re_torque_pub.publish(msg_torqueq3);


            if(count%100 == 0){
                std::cout<<"///////////////////////////////////"<<std::endl;
                std::cout<<"time is: "<<secs<<" seconds."<<std::endl;
                std::cout<<"error q1  is: "<<errorq[1]*180/M_PI<< " deg"<<std::endl;
                std::cout<<"error q2  is: "<<errorq[2]*180/M_PI<< " deg"<<std::endl;
                std::cout<<"error q3  is: "<<errorq[3]*180/M_PI<< " deg"<<std::endl;
            }
            count++;

            // -------------------------------------- DATA RECORDER --------------------------------------------//
            if(record){
                /*edo kaneis record o,ti thes se rosbag, ypo kapoio topic, meta sto .py arxeio sou ta anoigeis kai ta typoneis
                gia to motors_test xrisimopoio to angles_plotter.py*/
                    msg_q1d.data = q1step;
                    msg_q2d.data = q2step;
                    msg_q3d.data = q3step;
                    msg_theta0.data = theta0;
                    msg_theta0d.data = theta0in;

                    msg_q1ddot.data = q1stepdot;
                    msg_q2ddot.data = q2stepdot;
                    msg_q3ddot.data = q3stepdot;
                    msg_theta0dot.data = theta0dot;  
                    msg_theta0ddot.data = theta0stepdot; 

                    msg_q1ddotdot.data = q1stepdotdot;
                    msg_q2ddotdot.data = q2stepdotdot;
                    msg_q3ddotdot.data = q3stepdotdot;
                    msg_theta0ddotdot.data = theta0stepdotdot;  

                    msg_q1.data = q1;
                    msg_q2.data = q2;
                    msg_q3.data = q3;
                    msg_theta0.data = theta0;

                    msg_q1dot.data = q1dot;
                    msg_q2dot.data = q2dot;
                    msg_q3dot.data = q3dot;
                    msg_theta0dot.data = theta0dot;   


                    msg_torqueMBrw.data = torq_MBPD[0];
                    msg_torqueMBq1.data = torq_MBPD[1];
                    msg_torqueMBq2.data = torq_MBPD[2];
                    msg_torqueMBq3.data = torq_MBPD[3];

                    msg_u0.data = u[0];
                    msg_u1.data = u[1];
                    msg_u2.data = u[2];
                    msg_u3.data = u[3];

                    msg_cbar0.data = cbar[0];
                    msg_cbar1.data = cbar[1];
                    msg_cbar2.data = cbar[2];
                    msg_cbar3.data = cbar[3];

                    msg_Qtest0.data = Qtest[0];
                    msg_Qtest1.data = Qtest[1];
                    msg_Qtest2.data = Qtest[2];
                    msg_Qtest3.data = Qtest[3];

                    msg_error0.data = errorq[0];
                    msg_error1.data = errorq[1];
                    msg_error2.data = errorq[2];
                    msg_error3.data = errorq[3];

                    msg_errord0.data = errorqdot[0];
                    msg_errord1.data = errorqdot[1];
                    msg_errord2.data = errorqdot[2];
                    msg_errord3.data = errorqdot[3];

                    bag.write("/cepheus/q1d", ros::Time::now(), msg_q1d);
                    bag.write("/cepheus/q2d", ros::Time::now(), msg_q2d);
                    bag.write("/cepheus/q3d", ros::Time::now(), msg_q3d);
                    bag.write("/cepheus/theta0d", ros::Time::now(), msg_theta0d);

                    bag.write("/cepheus/q1ddot", ros::Time::now(), msg_q1ddot);
                    bag.write("/cepheus/q2ddot", ros::Time::now(), msg_q2ddot);
                    bag.write("/cepheus/q3ddot", ros::Time::now(), msg_q3ddot);
                    bag.write("/cepheus/theta0ddot", ros::Time::now(), msg_theta0ddot);  

                    bag.write("/cepheus/q1ddotdot", ros::Time::now(), msg_q1ddotdot);
                    bag.write("/cepheus/q2ddotdot", ros::Time::now(), msg_q2ddotdot);
                    bag.write("/cepheus/q3ddotdot", ros::Time::now(), msg_q3ddotdot);
                    bag.write("/cepheus/theta0ddotdot", ros::Time::now(), msg_theta0ddotdot);  

                    bag.write("/cepheus/q1", ros::Time::now(), msg_q1);
                    bag.write("/cepheus/q2", ros::Time::now(), msg_q2);
                    bag.write("/cepheus/q3", ros::Time::now(), msg_q3);
                    bag.write("/cepheus/theta0", ros::Time::now(), msg_theta0);

                    bag.write("/cepheus/q1dot", ros::Time::now(), msg_q1dot);
                    bag.write("/cepheus/q2dot", ros::Time::now(), msg_q2dot);
                    bag.write("/cepheus/q3dot", ros::Time::now(), msg_q3dot);
                    bag.write("/cepheus/theta0dot", ros::Time::now(), msg_theta0dot);  

                    bag.write("/cepheus/torquerw", ros::Time::now(), msg_torquerw);
                    bag.write("/cepheus/torqueq1", ros::Time::now(), msg_torqueq1);
                    bag.write("/cepheus/torqueq2", ros::Time::now(), msg_torqueq2);
                    bag.write("/cepheus/torqueq3", ros::Time::now(), msg_torqueq3);
                    
                    bag.write("/cepheus/torqueMBrw", ros::Time::now(), msg_torqueMBrw);
                    bag.write("/cepheus/torqueMBq1", ros::Time::now(), msg_torqueMBq1);
                    bag.write("/cepheus/torqueMBq2", ros::Time::now(), msg_torqueMBq2);
                    bag.write("/cepheus/torqueMBq3", ros::Time::now(), msg_torqueMBq3);   

                    bag.write("/cepheus/u0", ros::Time::now(), msg_u0);
                    bag.write("/cepheus/u1", ros::Time::now(), msg_u1);
                    bag.write("/cepheus/u2", ros::Time::now(), msg_u2);
                    bag.write("/cepheus/u3", ros::Time::now(), msg_u3);   

                    bag.write("/cepheus/cbar0", ros::Time::now(), msg_cbar0);
                    bag.write("/cepheus/cbar1", ros::Time::now(), msg_cbar1);
                    bag.write("/cepheus/cbar2", ros::Time::now(), msg_cbar2);
                    bag.write("/cepheus/cbar3", ros::Time::now(), msg_cbar3);
                    
                    bag.write("/cepheus/Qtest0", ros::Time::now(), msg_Qtest0);
                    bag.write("/cepheus/Qtest1", ros::Time::now(), msg_Qtest1);
                    bag.write("/cepheus/Qtest2", ros::Time::now(), msg_Qtest2);
                    bag.write("/cepheus/Qtest3", ros::Time::now(), msg_Qtest3);

                    // katagrafo  error q kai qdot
                    bag.write("/cepheus/error0", ros::Time::now(), msg_error0);     
                    bag.write("/cepheus/error1", ros::Time::now(), msg_error1);
                    bag.write("/cepheus/error2", ros::Time::now(), msg_error2);
                    bag.write("/cepheus/error3", ros::Time::now(), msg_error3);  

                    bag.write("/cepheus/errord0", ros::Time::now(), msg_errord0);
                    bag.write("/cepheus/errord1", ros::Time::now(), msg_errord1);
                    bag.write("/cepheus/errord2", ros::Time::now(), msg_errord2);
                    bag.write("/cepheus/errord3", ros::Time::now(), msg_errord3);  
            }
        }
        
        // Speep N time to complete 0.1s (100hz)
        loop_rate.sleep();
    }

    if (bag.isOpen()) bag.close();
    return 0;
}
