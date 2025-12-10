#include <cepheus_hw.h>
#include <cepheus_hw_functions.cpp>

FILE *latency_fp;
#define RT_PRIORITY 95

CepheusHW robot;

double rw_torque = 0.0;
double rw_cur_vel = 0.0;
double rw_last_vel = 0.0;

ros::Publisher  left_shoulder_pub, left_elbow_pub, right_elbow_pub;

// Helper functions
int readErr()
{
	FILE *f;
	int i;
	f = fopen("/home/mrrobot/nerr.txt","r");
	if (f == NULL) return 0;

	fscanf(f,"%d",&i);
	ROS_INFO("Errors number = %d",i);

	fclose(f);
	return i;
}
sig_atomic_t volatile g_request_shutdown = 0;
void ctrl_C_Handler(int sig)
{
	g_request_shutdown = 1;
}

// Callback functions
void thrusterCallback(const geometry_msgs::Vector3Stamped::ConstPtr& cmd)
{
	double thrust[4];
	thrust[0] = (double)cmd->vector.x;
	thrust[1] = (double)cmd->vector.y;
	thrust[2] = (double)cmd->vector.z;
	thrust[3] = (double)0;
	//ROS_WARN("%lf %lf %lf", thrust[0], thrust[1], thrust[2]);
	robot.setThrustPwm(thrust, 0.001, 0.9);
	return;
}
void torqueCallback(const std_msgs::Float64::ConstPtr& cmd)
{
	rw_torque = (double)cmd->data;
}
void setLeftShoulderEffort(const std_msgs::Float64::ConstPtr& cmd)
{
	robot.setCmd(LEFT_SHOULDER, cmd->data);
	//  ROS_INFO("GOT EFFORT FOR LEFT SHOULDER: %f", cmd->data); //gia debugging
	robot.writeMotors(); 
}
void setLeftElbowEffort(const std_msgs::Float64::ConstPtr& cmd)
{
	robot.setCmd(LEFT_ELBOW, cmd->data);
	// ROS_INFO("GOT EFFORT FOR LEFT ELBOW: %f", cmd->data);
	robot.writeMotors();
}
void setRightElbowEffort(const std_msgs::Float64::ConstPtr& cmd)
{
	robot.setCmd(RIGHT_ELBOW, cmd->data);
	// ROS_INFO("GOT EFFORT FOR RIGHT ELBOW: %f", cmd->data);
	robot.writeMotors();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cepheus_interface_node", ros::init_options::NoSigintHandler);
	signal(SIGINT, ctrl_C_Handler);
	ros::NodeHandle nh;

	setpriority(PRIO_PROCESS, 0, 19);

	double rate;
	ros::param::param<double>("~loop_rate", rate, 200);
	ros::Rate loop_rate(rate);

	double max_thrust;
	double rw_max_torque, rw_max_speed, rw_max_power, rw_total_inertia;


	ros::param::param<double>("~thruster_force", max_thrust, 0.6); 
	ros::param::param<double>("~rw_max_torque", rw_max_torque, 0.5);
	ros::param::param<double>("~rw_max_speed",  rw_max_speed, 100);
	ros::param::param<double>("~rw_max_power",  rw_max_power, 60);
	ros::param::param<double>("~rw_total_inertia", rw_total_inertia, 0.00197265);

	struct sched_param schedParam;
	schedParam.sched_priority = RT_PRIORITY;
	int sched_policy = SCHED_RR;
	sched_setscheduler(0, sched_policy, &schedParam);

	double max_cur[8];
	max_cur[0] = 1.72;
	max_cur[1] = 0;
	max_cur[2] = 0;
	max_cur[3] = 0;
	max_cur[4] = 0.8;
	max_cur[5] = 0.8;
	max_cur[6] = 0.8;
	max_cur[7] = 0;;
	robot.set(max_cur, max_thrust);

	controller_manager::ControllerManager cm(&robot);
	ros::Time prev_time = ros::Time::now();

	ros::Subscriber thrust_sub =  nh.subscribe("cmd_thrust", 1, thrusterCallback);
	ros::Subscriber torque_sub =  nh.subscribe("cmd_torque", 1, torqueCallback);
	ros::Subscriber set_left_shoulder_effort = nh.subscribe("set_left_shoulder_effort", 1, setLeftShoulderEffort);
	ros::Subscriber set_left_elbow_effort = nh.subscribe("set_left_elbow_effort", 1, setLeftElbowEffort);
	ros::Subscriber set_right_elbow_effort =  nh.subscribe("set_right_elbow_effort", 1, setRightElbowEffort);

	ros::Publisher rw_pos_pub = nh.advertise<std_msgs::Float64>("read_reaction_wheel_position", 1);
	ros::Publisher ls_pos_pub = nh.advertise<std_msgs::Float64>("read_left_shoulder_position", 1);
	ros::Publisher le_pos_pub = nh.advertise<std_msgs::Float64>("read_left_elbow_position", 1);
	ros::Publisher re_pos_pub = nh.advertise<std_msgs::Float64>("read_right_elbow_position", 1);
		
	ros::Publisher rw_vel_pub = nh.advertise<std_msgs::Float64>("read_reaction_wheel_velocity", 1);
	ros::Publisher ls_vel_pub = nh.advertise<std_msgs::Float64>("read_left_shoulder_velocity", 1);
	ros::Publisher le_vel_pub = nh.advertise<std_msgs::Float64>("read_left_elbow_velocity", 1);
	ros::Publisher re_vel_pub = nh.advertise<std_msgs::Float64>("read_right_elbow_velocity", 1);

	ROS_WARN("About to enter normal spinning...");
	ros::AsyncSpinner spinner(3);
	spinner.start();

	ros::Time curr_time;
	ros::Duration time_step;

	bool first_time = true;

	int a = 120;
	std_msgs::Float64 robot_info_msg;
	std_msgs::UInt8 limit_msg;

	while(!g_request_shutdown) {
		curr_time = ros::Time::now();
		if (first_time) {
			prev_time = curr_time;
			first_time = false;
		}

		time_step = curr_time - prev_time;
		prev_time = curr_time;

		robot.readEncoders(time_step);
		cm.update(curr_time, time_step);

		robot_info_msg.data = robot.getPos(LEFT_SHOULDER);
		ls_pos_pub.publish(robot_info_msg);
		robot_info_msg.data = robot.getPos(LEFT_ELBOW);
		le_pos_pub.publish(robot_info_msg);
		robot_info_msg.data = robot.getPos(RIGHT_ELBOW);
		re_pos_pub.publish(robot_info_msg);
		robot_info_msg.data = robot.getPos(REACTION_WHEEL);
		rw_pos_pub.publish(robot_info_msg);
		robot_info_msg.data = robot.getVel(LEFT_SHOULDER);
		ls_vel_pub.publish(robot_info_msg);
		robot_info_msg.data = robot.getVel(LEFT_ELBOW);
		le_vel_pub.publish(robot_info_msg);
		robot_info_msg.data = robot.getVel(RIGHT_ELBOW);
		re_vel_pub.publish(robot_info_msg);
		robot_info_msg.data = robot.getVel(REACTION_WHEEL);
		rw_vel_pub.publish(robot_info_msg);

		robot.writeMotors();
		robot.heartbeat();

		if (rw_torque!=0.0) {
			std_msgs::Float64 cmd;
			cmd.data = rw_torque;
			torque_pub.publish(cmd);
			rw_last_vel = robot.getVel(0);
			rw_cur_vel = rw_last_vel;
		}
		else {
			rw_cur_vel = robot.getVel(0);
			double error = rw_last_vel - rw_cur_vel;
			std_msgs::Float64 cmd;
			cmd.data = -10*error;
			torque_pub.publish(cmd);
		}

		loop_rate.sleep();
	}
	robot.safeClose();
	return 0;
}