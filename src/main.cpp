//Group C: Florian Koehler, Rasmus Larsen, Adam Zylka

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/JointState.h"
#include "message_filters/subscriber.h"
#include <string.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/TactileTouch.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedAction.h>
#include <std_srvs/Empty.h>
#include <eigen3/Eigen/Eigen>
#include <tf/transform_listener.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedActionGoal.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;

bool stop_thread=false;

void spinThread()
{
	while(!stop_thread)
	{
		ros::spinOnce();
	}
}


class Nao_control
{
public:
	// ros handler
	ros::NodeHandle nh_;

	// subscriber to joint states
	ros::Subscriber sensor_data_sub;
	ros::Subscriber joint_action_status_sub;

	// subscriber to bumpers states
	ros::Subscriber bumper_sub;

	// subscriber to head tactile states
	ros::Subscriber tactile_sub;

	//publisher of joint angles
	ros::Publisher nao_joints_command_pub;

    // publish states of the left hand
	ros::Publisher nao_left_hand_pub;

     // publish states of the right hand
	ros::Publisher nao_right_hand_pub;

     // publish states of the head and legs hand
	ros::Publisher nao_head_legs_pub;

    // service for stiffeness disable command
	ros::ServiceClient stiffness_disable_srv;

    // variables which store current states of the joints
	sensor_msgs::JointState current_left_arm_state;
	sensor_msgs::JointState current_right_arm_state;
	sensor_msgs::JointState current_head_legs_state;

    // variable contains current status of the joint action execution
	int current_joint_action_status;

	boost::thread *spin_thread;

	tf::TransformListener *listener;
	tf::TransformBroadcaster br;

	Nao_control(tf::TransformListener *listener_init)
	{
		// subscribe to topic joint_states and specify that all data will be processed by function sensorCallback
		sensor_data_sub=nh_.subscribe("/nao/joint_states",1, &Nao_control::sensorCallback, this);

		// subscribe to topic bumper and specify that all data will be processed by function bumperCallback
		bumper_sub=nh_.subscribe("/nao/bumper",1, &Nao_control::bumperCallback, this);

		// subscribe to topic tactile_touch and specify that all data will be processed by function tactileCallback
		tactile_sub=nh_.subscribe("/nao/tactile_touch",1, &Nao_control::tactileCallback, this);

        // initialize service for stiffeness
		stiffness_disable_srv  =nh_.serviceClient<std_srvs::Empty>("/nao/body_stiffness/disable");

		//setup publisher for joint angles, the message type is JointAnglesWithSpeedActionGoal
		nao_joints_command_pub = nh_.advertise<naoqi_bridge_msgs::JointAnglesWithSpeedActionGoal>("/nao/joint_angles_action/goal", 1);

        // subscribe for joint action status updates
		joint_action_status_sub=nh_.subscribe("/nao/joint_angles_action/status",1, &Nao_control::jointActionStatusCB, this);

        // publishers of current nao joint states
		nao_left_hand_pub=nh_.advertise<sensor_msgs::JointState>("/left_arm_joint_states",1);
		nao_right_hand_pub=nh_.advertise<sensor_msgs::JointState>("/right_arm_joint_states",1);
		nao_head_legs_pub=nh_.advertise<sensor_msgs::JointState>("/head_legs_joint_states",1);

		stop_thread=false;
		spin_thread=new boost::thread(&spinThread);
		listener=listener_init;
	}
	~Nao_control()
	{
		stop_thread=true;
		sleep(1);
		spin_thread->join();
	}

// this callback provides information about current action status
	void jointActionStatusCB(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
	{
		if(!msg->status_list.empty())
		{
			current_joint_action_status=(int)msg->status_list.at(0).status;
		}
	}

// this callback function provides information about nao feet bumpers
	void bumperCallback(const naoqi_bridge_msgs::Bumper::ConstPtr& bumperState)
	{

	}

    // this callback provides information about current head tactile buttons. Currently middle button will trigger call to disable joint stiffeness service.
	void tactileCallback(const naoqi_bridge_msgs::TactileTouch::ConstPtr& tactileState)
	{

		if(tactileState->button==naoqi_bridge_msgs::TactileTouch::buttonMiddle)
		{
			if(tactileState->state==naoqi_bridge_msgs::TactileTouch::statePressed)
			{
                // generate empty request
				std_srvs::Empty srv;
				stiffness_disable_srv.call(srv);
			}
		}
	}

    // this function checks joint limits of the left arm. You need to provide JointState vector of 5 elements
	bool check_joint_limits_left_arm(sensor_msgs::JointState joints)
	{
		bool check=true;

		//LShoulderPitch -2.0857 to 2.0857
		if((joints.position.at(0) < -1.9)||(joints.position.at(0) >1.9))
		{
			check=false;
		}
		//LShoulderRoll -0.3142 to 1.3265
		if((joints.position.at(1) < -0.3)||(joints.position.at(1) >1.3))
		{
			check=false;
		}
		//LElbowYaw -2.0857 to 2.0857
		if((joints.position.at(2) < -1.9)||(joints.position.at(2) >1.9))
		{
			check=false;
		}
		//LElbowRoll -1.5446 to -0.0349
		if((joints.position.at(3) < -1.5)||(joints.position.at(3) >-0.03))
		{
			check=false;
		}
		//LWristYaw -1.8238 to 1.8238
		if((joints.position.at(4) < -1.7)||(joints.position.at(4) >1.7))
		{
			check=false;
		}
		return check;
	}

    // this function checks joint limits of the right arm. You need to provide JointState vector of 5 elements
	bool check_joint_limits_right_arm(sensor_msgs::JointState joints)
	{
		bool check=true;
		//RShoulderPitch -2.0857 to 2.0857
		if((joints.position.at(0) < -1.9)||(joints.position.at(0) >1.9))
		{
			check=false;
		}
		//RShoulderRoll -1.3265 to 0.3142
		if((joints.position.at(1) < -1.3)||(joints.position.at(1) >0.3))
		{
			check=false;
		}
		//RElbowYaw -2.0857 to 2.0857
		if((joints.position.at(2) < -1.9)||(joints.position.at(2) >1.99))
		{
			check=false;
		}
		//RElbowRoll 0.0349 to 1.5446
		if((joints.position.at(3) < 0.03)||(joints.position.at(3) >1.5))
		{
			check=false;
		}
		//RWristYaw -1.8238 to 1.8238
		if((joints.position.at(4) < -1.7)||(joints.position.at(4) >1.7))
		{
			check=false;
		}
		return check;
	}

    // this callback recives info about current joint states
	void sensorCallback(const sensor_msgs::JointState::ConstPtr& jointState)
	{
		current_left_arm_state.name.clear();
		current_left_arm_state.position.clear();
		current_right_arm_state.name.clear();
		current_right_arm_state.position.clear();
		current_head_legs_state.name.clear();
		current_head_legs_state.position.clear();

		current_left_arm_state.header.stamp=ros::Time::now();

		current_left_arm_state.name.push_back(jointState->name.at(2));
		current_left_arm_state.position.push_back(jointState->position.at(2));
		current_left_arm_state.name.push_back(jointState->name.at(3));
		current_left_arm_state.position.push_back(jointState->position.at(3));
		current_left_arm_state.name.push_back(jointState->name.at(4));
		current_left_arm_state.position.push_back(jointState->position.at(4));
		current_left_arm_state.name.push_back(jointState->name.at(5));
		current_left_arm_state.position.push_back(jointState->position.at(5));
		current_left_arm_state.name.push_back(jointState->name.at(6));
		current_left_arm_state.position.push_back(jointState->position.at(6));

		current_right_arm_state.name.push_back(jointState->name.at(20));
		current_right_arm_state.position.push_back(jointState->position.at(20));
		current_right_arm_state.name.push_back(jointState->name.at(21));
		current_right_arm_state.position.push_back(jointState->position.at(21));
		current_right_arm_state.name.push_back(jointState->name.at(22));
		current_right_arm_state.position.push_back(jointState->position.at(22));
		current_right_arm_state.name.push_back(jointState->name.at(23));
		current_right_arm_state.position.push_back(jointState->position.at(23));
		current_right_arm_state.name.push_back(jointState->name.at(24));
		current_right_arm_state.position.push_back(jointState->position.at(24));

		current_head_legs_state.name.push_back(jointState->name.at(0));
		current_head_legs_state.position.push_back(jointState->position.at(0));
		current_head_legs_state.name.push_back(jointState->name.at(1));
		current_head_legs_state.position.push_back(jointState->position.at(1));
		for(int i=7; i<20;i++)
		{
			current_head_legs_state.name.push_back(jointState->name.at(i));
			current_head_legs_state.position.push_back(jointState->position.at(i));
		}
		current_head_legs_state.name.push_back(jointState->name.at(25));
		current_head_legs_state.position.push_back(jointState->position.at(25));
	}


    // this function computes Jacobian for selected parameters
	Eigen::MatrixXd jacobian(double q1,double q2,double q3,double q4,double q5,double L1,double L2,double L3,double L4)
	{
		Eigen::MatrixXd J(6,5);
		J(0,0)=L4*(sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - cos(q2)*cos(q4)*sin(q1)) - sin(q1)*(L3*cos(q2) - L2*sin(q2));
		J(0,1)=-cos(q1)*(L4*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) + L2*cos(q2) + L3*sin(q2));
		J(0,2)=L4*sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3));
		J(0,3)=-L4*(cos(q1)*cos(q2)*sin(q4) - cos(q4)*sin(q1)*sin(q3) + cos(q1)*cos(q3)*cos(q4)*sin(q2));
		J(0,4)=0;
		J(1,0)=L4*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*cos(q4)) + cos(q1)*(L3*cos(q2) - L2*sin(q2));
		J(1,1)=-sin(q1)*(L4*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) + L2*cos(q2) + L3*sin(q2));
		J(1,2)=-L4*sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3));
		J(1,3)=-L4*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4))*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - L4*cos(q2)*sin(q3)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*cos(q4));
		J(1,4)=0;
		J(2,0)=0;
		J(2,1)=L3*cos(q2) - L2*sin(q2) + L4*cos(q2)*cos(q4) - L4*cos(q3)*sin(q2)*sin(q4);
		J(2,2)=-L4*cos(q2)*sin(q3)*sin(q4);
		J(2,3)=L4*cos(q2)*cos(q3)*cos(q4) - L4*sin(q2)*sin(q4);
		J(2,4)=0;
		J(3,0)=0;
		J(3,1)=sin(q1);
		J(3,2)=cos(q1)*cos(q2);
		J(3,3)=cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3);
		J(3,4)=sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*cos(q4);
		J(4,0)=0;
		J(4,1)=-cos(q1);
		J(4,2)=cos(q2)*sin(q1);
		J(4,3)=sin(q1)*sin(q2)*sin(q3) - cos(q1)*cos(q3);
		J(4,4)=cos(q2)*cos(q4)*sin(q1) - sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2));
		J(5,0)=1;
		J(5,1)=0;
		J(5,2)=sin(q2);
		J(5,3)=-cos(q2)*sin(q3);
		J(5,4)=cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4);
		return J;
	}

    // this function computes homogeneous tranformation from the end effector to the base frame
	Eigen::Matrix4d transformation_EF_Torso(double q1,double q2,double q3,double q4,double q5,double L1,double L2,double L3,double L4)
	{
		Eigen::Matrix4d T_shoulder_torso;
		T_shoulder_torso(0,0)=1;
		T_shoulder_torso(0,1)=0;
		T_shoulder_torso(0,2)=0;
		T_shoulder_torso(0,3)=0;
		T_shoulder_torso(1,0)=0;
		T_shoulder_torso(1,1)=0;
		T_shoulder_torso(1,2)=1;
		T_shoulder_torso(1,3)=0;
		T_shoulder_torso(2,0)=0;
		T_shoulder_torso(2,1)=-1;
		T_shoulder_torso(2,2)=0;
		T_shoulder_torso(2,3)=0.1;
		T_shoulder_torso(3,0)=0;
		T_shoulder_torso(3,1)=0;
		T_shoulder_torso(3,2)=0;
		T_shoulder_torso(3,3)=1;

		Eigen::Matrix4d T_ef_shoulder;
		T_ef_shoulder(0,0)=cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) - cos(q1)*cos(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3));
		T_ef_shoulder(0,1)=cos(q5)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) - cos(q1)*cos(q2)*sin(q4));
		T_ef_shoulder(0,2)=sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*cos(q4);
		T_ef_shoulder(0,3)=L4*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*cos(q4)) + cos(q1)*(L3*cos(q2) - L2*sin(q2));
		T_ef_shoulder(1,0)=- cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + cos(q2)*sin(q1)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3));
		T_ef_shoulder(1,1)=sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + cos(q2)*sin(q1)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3));
		T_ef_shoulder(1,2)=cos(q2)*cos(q4)*sin(q1) - sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2));
		T_ef_shoulder(1,3)=sin(q1)*(L3*cos(q2) - L2*sin(q2)) - L4*(sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - cos(q2)*cos(q4)*sin(q1));
		T_ef_shoulder(2,0)=- cos(q5)*(sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4)) - cos(q2)*sin(q3)*sin(q5);
		T_ef_shoulder(2,1)=sin(q5)*(sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4)) - cos(q2)*cos(q5)*sin(q3);
		T_ef_shoulder(2,2)=cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4);
		T_ef_shoulder(2,3)=L1 + L4*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) + L2*cos(q2) + L3*sin(q2);
		T_ef_shoulder(3,0)=0;
		T_ef_shoulder(3,1)=0;
		T_ef_shoulder(3,2)=0;
		T_ef_shoulder(3,3)=1;

		Eigen::Matrix4d T_ef_torso;
		T_ef_torso=T_shoulder_torso*T_ef_shoulder;
		return T_ef_torso;
	}

    // this function computes forward kinematics for selected parameters
	Eigen::VectorXd forward_Kinematics(double q1,double q2,double q3,double q4,double q5,double L1,double L2,double L3,double L4)
	{
		Eigen::Matrix4d T_ef_torso=transformation_EF_Torso(q1,q2,q3,q4,q5,L1,L2,L3,L4);
		Eigen::Quaterniond qe;
		qe=T_ef_torso.block<3,3>(0,0);

		tf::Transform transform;

		transform.setOrigin( tf::Vector3(T_ef_torso(0,3), T_ef_torso(1,3),T_ef_torso(2,3)) );
		transform.setRotation(tf::Quaternion(qe.x(),qe.y(),qe.z(),qe.w()));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "end_effector"));
		Eigen::VectorXd x_current(6);

		Eigen::Matrix3d R=T_ef_torso.block<3,3>(0,0);
		double roll=atan2(R(2,1),R(2,2));
		double pitch=atan2(-R(2,0), sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2)));
		double yaw=atan2(R(1,0),R(0,0));
		//x_current<<T_ef_torso(0,3),T_ef_torso(1,3),T_ef_torso(2,3),roll,pitch,yaw;
		x_current<<T_ef_torso(0,3),T_ef_torso(1,3),T_ef_torso(2,3),0,0,0;
		return x_current;
	}

    // this function computes inverse kinematics for both arms.
	Eigen::VectorXd inverse_Kinematics(Eigen::VectorXd q_current,Eigen::VectorXd x_desired, double threshold, double k_gain, int arm)
	{
		double L1,L2,L3,L4;
		//LEFT ARM
		if(arm==0)
		{
			L4=0.1137;
			L3=0.105;
			L2=0.015;
			L1=0.098;
		}
		//RIGHT ARM
		if(arm==1)
		{
			L4=0.1137;
			L3=0.105;
			L2=-0.015;
			L1=-0.098;

		}
		Eigen::VectorXd q_desired = q_current;
		
		Eigen::VectorXd x_current = forward_Kinematics(q_current(0), q_current(1), q_current(2), q_current(3), q_current(4), L1, L2, L3, L4);
		Eigen::VectorXd dx = x_desired - x_current;
		while(dx.norm() > threshold)
		{

			dx = x_desired - x_current;
			Eigen::MatrixXd jac = jacobian(q_desired(0), q_desired(1), q_desired(2), q_desired(3), q_desired(4), L1, L2, L3, L4);
			Eigen::VectorXd dq = k_gain * jac.transpose() * dx;
			q_desired += dq;
			x_current = forward_Kinematics(q_desired(0), q_desired(1), q_desired(2), q_desired(3), q_desired(4), L1, L2, L3, L4);

		}
		

		return q_desired;
	}

// this is main loop which should send commands to the nao arms.
	void publish_joint_states()
	{
		int mArm = 0;
		Eigen::VectorXd mx_desired(6);
		mx_desired << 0.213, 0.143, 0.117, 0.0, 0.0, 0.0;
		double threshold = 0.01;
		double k_gain = 0.5;
		ros::Rate rate_sleep(10);
		
		while(nh_.ok())
		{
			Eigen::VectorXd q_current(5);
			if(current_left_arm_state.position.size() == 0 || current_right_arm_state.position.size() == 0) continue;
			if(mArm == 0)
			{
				q_current(0) = current_left_arm_state.position[0];
				q_current(1) = current_left_arm_state.position[1];
				q_current(2) = current_left_arm_state.position[2];
				q_current(3) = current_left_arm_state.position[3];
				q_current(4) = current_left_arm_state.position[4];
				
			}
			else if(mArm == 1)
			{
				q_current(0) = current_right_arm_state.position[0];
				q_current(1) = current_right_arm_state.position[1];
				q_current(2) = current_right_arm_state.position[2];
				q_current(3) = current_right_arm_state.position[3];
				q_current(4) = current_right_arm_state.position[4];
			}
			Eigen::VectorXd q_desired = inverse_Kinematics(q_current, mx_desired, threshold, k_gain, mArm);
			moveRobot(q_desired, mArm);
			rate_sleep.sleep();


			while (current_joint_action_status != 3)
			{		
				rate_sleep.sleep();
			}
			mArm = 1;
			mx_desired << 0.213, -0.143, 0.117, 0.0, 0.0, 0.0;
		}
	}

// this function executes joint motion for the desired arm on the robot
	void moveRobot(Eigen::VectorXd q_desired, int arm)
	{

		// LEFT ARM
		if(arm==0)
		{
			sensor_msgs::JointState left_joints;
			for(int i=0;i<5;i++)
			{
				left_joints.name.push_back(current_left_arm_state.name.at(i));
				left_joints.position.push_back(q_desired[i]);
			}
			
			bool check_limits=check_joint_limits_left_arm(left_joints);
			if(check_limits)
			{
				naoqi_bridge_msgs::JointAnglesWithSpeedActionGoal action_execute;
				stringstream ss;
				ss<<ros::Time::now().sec;
				action_execute.goal_id.id="move_"+ss.str();
				action_execute.goal.joint_angles.speed=0.05;
				action_execute.goal.joint_angles.relative=0;
				action_execute.goal.joint_angles.joint_names=left_joints.name;
				for(int i=0;i<5;i++)
					action_execute.goal.joint_angles.joint_angles.push_back((float)left_joints.position.at(i));
				action_execute.header.stamp=ros::Time::now();
				nao_joints_command_pub.publish(action_execute);
				current_joint_action_status=0;
				ros::Rate rate_sleep(10);
				while(current_joint_action_status!=3)
				{
					rate_sleep.sleep();
					nao_left_hand_pub.publish(current_left_arm_state);
					nao_right_hand_pub.publish(current_right_arm_state);
					nao_head_legs_pub.publish(current_head_legs_state);
				}
			}
		}
		//RIGHT ARM
		if(arm==1)
		{
			sensor_msgs::JointState right_joints;
			for(int i=0;i<5;i++)
			{
				right_joints.name.push_back(current_right_arm_state.name.at(i));
				right_joints.position.push_back(q_desired[i]);
			}
			bool check_limits=check_joint_limits_right_arm(right_joints);

			if(check_limits)
			{
				naoqi_bridge_msgs::JointAnglesWithSpeedActionGoal action_execute;
				stringstream ss;
				ss<<ros::Time::now().sec;
				action_execute.goal_id.id="move_"+ss.str();
				action_execute.goal.joint_angles.speed=0.05;
				action_execute.goal.joint_angles.relative=0;
				action_execute.goal.joint_angles.joint_names=right_joints.name;
				for(int i=0;i<5;i++)
					action_execute.goal.joint_angles.joint_angles.push_back((float)right_joints.position.at(i));
				action_execute.header.stamp=ros::Time::now();
				nao_joints_command_pub.publish(action_execute);
				current_joint_action_status=0;
				ros::Rate rate_sleep(10);
				while(current_joint_action_status!=3)
				{
					rate_sleep.sleep();
					nao_left_hand_pub.publish(current_left_arm_state);
					nao_right_hand_pub.publish(current_right_arm_state);
					nao_head_legs_pub.publish(current_head_legs_state);
				}
			}
		}

	}

    // this function executes joint motion for the desired arm on the simulator
	void moveSimulator(Eigen::VectorXd q_desired,int arm)
	{
		// LEFT ARM
		if(arm==0)
		{
			Eigen::VectorXd q_current(5);
			q_current<<current_left_arm_state.position.at(0),current_left_arm_state.position.at(1),current_left_arm_state.position.at(2),current_left_arm_state.position.at(3),current_left_arm_state.position.at(4);
			Eigen::VectorXd q_delta(5);
			q_delta=q_desired-q_current;
			q_delta=q_delta/10.0;
			ros::Rate rate_sleep(10);
			for(int i=0;i<10;i++)
			{
				q_current=q_current+q_delta;
				current_left_arm_state.position.at(0)=q_current[0];
				current_left_arm_state.position.at(1)=q_current[1];
				current_left_arm_state.position.at(2)=q_current[2];
				current_left_arm_state.position.at(3)=q_current[3];
				current_left_arm_state.position.at(4)=q_current[4];
				nao_left_hand_pub.publish(current_left_arm_state);
				nao_right_hand_pub.publish(current_right_arm_state);
				nao_head_legs_pub.publish(current_head_legs_state);
				rate_sleep.sleep();
			}
		}
		//RIGHT ARM
		if(arm==1)
		{
			Eigen::VectorXd q_current(5);
			q_current<<current_right_arm_state.position.at(0),current_right_arm_state.position.at(1),current_right_arm_state.position.at(2),current_right_arm_state.position.at(3),current_right_arm_state.position.at(4);
			Eigen::VectorXd q_delta(5);
			q_delta=q_desired-q_current;
			q_delta=q_delta/10.0;
			ros::Rate rate_sleep(10);
			for(int i=0;i<10;i++)
			{
				q_current+=q_delta;
				current_right_arm_state.position.at(0)=q_current[0];
				current_right_arm_state.position.at(1)=q_current[1];
				current_right_arm_state.position.at(2)=q_current[2];
				current_right_arm_state.position.at(3)=q_current[3];
				current_right_arm_state.position.at(4)=q_current[4];
				nao_left_hand_pub.publish(current_left_arm_state);
				nao_right_hand_pub.publish(current_right_arm_state);
				nao_head_legs_pub.publish(current_head_legs_state);
				rate_sleep.sleep();
			}
		}
	}

};
int main(int argc, char** argv)
{
	ros::init(argc, argv, "imitation");

	ros::NodeHandle n;
	ros::Rate rate_sleep(20);
	tf::TransformListener *listener=new tf::TransformListener();
	Nao_control ic(listener);
	ic.publish_joint_states();

	return 0;

}
