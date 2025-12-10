#include <dm7820_library.h>
#include <ros/ros.h>
#include <errno.h>
#include <error.h>
#include <signal.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <stdio.h>
#include <math.h>
#include <std_srvs/Empty.h>
#include <controller_manager/controller_manager.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_listener.h>
#include <sys/resource.h>
#include <boost/bind.hpp>

#define PWM_MOTOR_FREQ 			5000
#define ADC_PWM_FREQ 			25000
#define PWM_THRUSTER_FREQ 		10
#define CLOCK_FREQ 				25000000
#define PWM_MOTOR_PERIOD_COUNTS (CLOCK_FREQ / PWM_MOTOR_FREQ)
#define PWM_MOTOR_MIN_DT 		(PWM_MOTOR_PERIOD_COUNTS * 0.1)
#define PWM_MOTOR_MAX_DT 		(PWM_MOTOR_PERIOD_COUNTS * 0.9)
#define PWM_MOTOR_RANGE 		(PWM_MOTOR_MAX_DT - PWM_MOTOR_MIN_DT)
#define ADC_PWM_PERIOD_COUNTS 	(CLOCK_FREQ / ADC_PWM_FREQ)

enum ManipulatorMapping{
		
	REACTION_WHEEL = 0,	//0 Active
	unknown1,			//1
	unknown2,			//2
	unknown3,			//3
	LEFT_SHOULDER,		//4 Active
	LEFT_ELBOW,			//5 Active
	RIGHT_SHOULDER,		//6	
	RIGHT_ELBOW,		//7	Active
	LEFT_WRIST,			//8
	RIGHT_WRIST,		//9
	LEFT_GRIPPER,		//10
	RIGHT_GRIPPER		//11
};

class CepheusHW : public hardware_interface::RobotHW
{
	public:
		void heartbeat();
		void enable();
		void disable();
		void setThrustPwm(double*, double, double);
		void writeMotors();
		void readEncoders(ros::Duration);
		void setParam(double*, double);
		void setCmd(int,double);
		double getVel(int);
		void safeClose();
		CepheusHW();

		void set_manipulator_width(int manipulator, uint16_t width_val){
			if(manipulator >=0 && manipulator <=12)
				width[manipulator] = width_val;
			else
				ROS_WARN("Cannot set width!. Manipulator %d does not exist!", manipulator);
		}
	
		double getCmd(int i){
			return cmd[i];
		}
	
		double getPos(int i){
			return pos[i];
		}
	
		void setOffset(int i, double offset){
			offset_pos[i] = offset;
		}

	private:
		bool homing(int, float);
		/***controller manager interface***/
		hardware_interface::JointStateInterface jnt_state_interface;
		hardware_interface::EffortJointInterface jnt_eff_interface;
		hardware_interface::PositionJointInterface jnt_pos_interface;
		

		enum motors{ rw, l1, l2, r1, r2}; 

		/*****Motors parameters*********/
		double max_current[8];
		double max_thrust;
		/*****SERVOS parameters*********/
		double force[4];

		/******motors********/
		int limit[8];
		double cmd[12];
		uint16_t width[12];
		uint16_t dir[10];
		double current[8];
		double home_pos[8];
		double pos[8], prev_pos[8], offset_pos[8];
		double vel[8];
		double eff[8];
		double vel_new[8];
		//encoder values from card
		uint16_t encoder_1_val;
		uint16_t encoder_2_val;
		uint16_t encoder_3_val;
		uint16_t encoder_4_val;
		uint16_t encoder_5_val;
		uint16_t encoder_6_val;
		uint16_t encoder_7_val;
		uint16_t encoder_8_val;
		//encoder overflow counters
		int encoder_1_ovf;
		int encoder_2_ovf;
		int encoder_3_ovf;
		int encoder_4_ovf;
		int encoder_5_ovf;
		int encoder_6_ovf;
		int encoder_7_ovf;
		int encoder_8_ovf;
		//encoder values to transmit
		int encoder_1;
		int encoder_2;
		int encoder_3;
		int encoder_4;
		int encoder_5;
		int encoder_6;
		int encoder_7;
		int encoder_8;
		/******thrusters********/
		uint32_t period;

		/*******IO board********/
		DM7820_Board_Descriptor *board;
		DM7820_Board_Descriptor *manipulator_board;
		DM7820_Error dm7820_status;
		uint8_t encoder_status;
		dm7820_incenc_phase_filter phase_filter;
		int status;

		int count;
		uint16_t output_value;
		bool strobe;
};
