/*-------------------------------------Joint Space Controller------------------------------*/
/* 10.12.15 Florian Köhler */


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
#include <naoqi_bridge_msgs/JointAnglesWithSpeedActionGoal.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>
#include <actionlib_msgs/GoalStatusArray.h>
#include "optim.hpp"


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
    // Nodehandle
    ros::NodeHandle nodeh;

    // Subscriber
    // Joint States
    ros::Subscriber sensor_data_sub;
    // Action Status
    ros::Subscriber joint_action_status_sub;
    // Head Tactile States
    ros::Subscriber tactile_sub;

    // Publisher
    //Joint angles
    ros::Publisher joints_move_pub;
    //joint states
    ros::Publisher current_joint_state_pub;

    // Services
    ros::ServiceClient stiffness_disable_srv;
    ros::ServiceClient stiffness_enable_srv;

    // variables
    //state of joints
    sensor_msgs::JointState current_state;
    //goal state of joints
    sensor_msgs::JointState desired_states;
    //status of the joint action execution
    int current_joint_action_status;

    boost::thread *spin_thread;

    Nao_control(sensor_msgs::JointState des_states)
    {
        // subscribe to topic joint_states and specify that all data will be processed by function sensorCallback
        sensor_data_sub = nodeh.subscribe("/nao/joint_states",1, &Nao_control::sensorCB, this);

        // subscribe to topic tactile_touch and specify that all data will be processed by function tactileCallback
        tactile_sub = nodeh.subscribe("/nao/tactile_touch",1, &Nao_control::tactileCB, this);

        // initialize service for stiffeness
        stiffness_disable_srv  = nodeh.serviceClient<std_srvs::Empty>("/nao/body_stiffness/disable");
        stiffness_enable_srv = nodeh.serviceClient<std_srvs::Empty>("/nao/body_stiffness/enable");

        //setup publisher for joint angles, the message type is JointAnglesWithSpeedActionGoal
        joints_move_pub = nodeh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeedActionGoal>("/nao/joint_angles_action/goal", 1);

        // subscribe for joint action status updates
        joint_action_status_sub = nodeh.subscribe("/nao/joint_angles_action/status",1, &Nao_control::jointActionStatusCB, this);

        // publishers of current nao joint states
        current_joint_state_pub = nodeh.advertise<sensor_msgs::JointState>("/joint_states", 1);

        for(int m = 0; m<des_states.name.size(); m++)
        {
            desired_states.name.push_back(des_states.name[m]);
            desired_states.position.push_back(des_states.position[m]);
        }

        stop_thread=false;
        spin_thread=new boost::thread(&spinThread);

    }
    ~Nao_control()
    {
        stop_thread=true;
        sleep(1);
        spin_thread->join();
    }

// Callback
    // current action status
    void jointActionStatusCB(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
    {
        if(!msg->status_list.empty())
        {
            current_joint_action_status=(int)msg->status_list.at(0).status;
        }
    }
    // head tactile buttons
    void tactileCB(const naoqi_bridge_msgs::TactileTouch::ConstPtr& tactileState)
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

        if(tactileState->button==naoqi_bridge_msgs::TactileTouch::buttonFront)
        {
            if(tactileState->state==naoqi_bridge_msgs::TactileTouch::statePressed)
            {
                // generate empty request
                std_srvs::Empty srv;
                stiffness_enable_srv.call(srv);
            }
        }
    }
    // current joint states
    void sensorCB(const sensor_msgs::JointState::ConstPtr& jointState)
    {
        current_state.name.clear();
        current_state.position.clear();
        current_state.header.stamp = ros::Time::now();

        for(int i=0; i<jointState->name.size();i++)
        {
            current_state.name.push_back(jointState->name.at(i));
            current_state.position.push_back(jointState->position.at(i));
        }
     }


// Joint Limits
    bool check_joint_limits(sensor_msgs::JointState joints)
    {
        bool check=true;
        //Head
        //HeadYaw
        if((joints.position.at(0) < -1.9)||(joints.position.at(0) >1.9))
        {
            check=false;
            cout << "0" << endl;
        }
        //HeadPitch
        if((joints.position.at(1) < -0.3)||(joints.position.at(1) >0.2))
        {
            check=false;
            cout << "1" << endl;
        }

        //LArm
        //LShoulderPitch
        if((joints.position.at(2) < -0.3)||(joints.position.at(2) >1.3))
        {
            check=false;
            cout << "2" << endl;
        }
        //LShoulderRoll
        if((joints.position.at(3) < -1.9)||(joints.position.at(3) >1.9))
        {
            check=false;
            cout << "3" << endl;
        }
        //LElbowYaw -2.0857 to 2.0857
        if((joints.position.at(4) < -1.9)||(joints.position.at(4) >1.9))
        {
            check=false;
            cout << "4" << endl;
        }
        //LElbowRoll -1.5446 to -0.0349
        if((joints.position.at(5) < -1.5)||(joints.position.at(5) >-0.03))
        {
            check=false;
            cout << "5" << endl;
        }
        //LWristYaw -1.8238 to 1.8238
        if((joints.position.at(6) < -1.7)||(joints.position.at(6) >1.7))
        {
            check=false;
            cout << "6" << endl;
        }
        //LHand
        if((joints.position.at(7) != 1)&&(joints.position.at(7) != 0))
        {
            check=false;
            cout << "7" << endl;
        }

        //LLeg
        //LHipYawPitch
        if((joints.position.at(8) < -1.1)||(joints.position.at(8) >0.7))
        {
            check=false;
            cout << "8" << endl;
        }
        //LHipRoll
        if((joints.position.at(9) < -0.3)||(joints.position.at(9) >0.7))
        {
            check=false;
            cout << "9" << endl;
        }
        //LHipPitch
        if((joints.position.at(10) < -1.5)||(joints.position.at(10) >0.4))
        {
            check=false;
            cout << "10" << endl;
        }
        //LKneePitch
        if((joints.position.at(11) < 0.0)||(joints.position.at(11) >2.0))
        {
            check=false;
            cout << "11" << endl;
        }
        //LAnklePitch
        if((joints.position.at(12) < -1.1)||(joints.position.at(12) >0.9))
        {
            check=false;
            cout << "12" << endl;
        }
        //LAnkleRoll
        if((joints.position.at(13) < -0.3)||(joints.position.at(13) >0.7))
        {
            check=false;
            cout << "13" << endl;
        }

        //RLeg
        //RHipYawPitch
        if((joints.position.at(14) < -1.1)||(joints.position.at(14) >0.7))
        {
            check=false;
            cout << "14" << endl;
        }
        //RHipRoll
        if((joints.position.at(15) < -0.7)||(joints.position.at(15) >0.3))
        {
            check=false;
            cout << "15" << endl;
        }
        //RHipPitch
        if((joints.position.at(16) < -1.5)||(joints.position.at(16) >0.4))
        {
            check=false;
            cout << "16" << endl;
        }
        //RKneePitch
        if((joints.position.at(17) < 0.0)||(joints.position.at(17) >2.1))
        {
            check=false;
            cout << "17" << endl;
        }
        //RAnklePitch
        if((joints.position.at(18) < -1.1)||(joints.position.at(18) >0.9))
        {
            check=false;
            cout << "18" << endl;
        }
        //RAnkleRoll
        if((joints.position.at(19) < -0.7)||(joints.position.at(19) >0.3))
        {
            check=false;
            cout << "19" << endl;
        }

        //RArm
        //RShoulderPitch -2.0857 to 2.0857
        if((joints.position.at(20) < -1.9)||(joints.position.at(20) >1.9))
        {
            check=false;
            cout << "20" << endl;
        }
        //RShoulderRoll -1.3265 to 0.3142
        if((joints.position.at(21) < -1.3)||(joints.position.at(21) >0.3))
        {
            check=false;
            cout << "21" << endl;
        }
        //RElbowYaw -2.0857 to 2.0857
        if((joints.position.at(22) < -1.9)||(joints.position.at(22) >1.9))
        {
            check=false;
            cout << "22" << endl;
        }
        //RElbowRoll 0.0349 to 1.5446
        if((joints.position.at(23) < 0.03)||(joints.position.at(23) >1.5))
        {
            check=false;
            cout << "23" << endl;
        }
        //RWristYaw -1.8238 to 1.8238
        if((joints.position.at(24) < -1.7)||(joints.position.at(24) >1.7))
        {
            check=false;
            cout << "24" << endl;
        }
        //RHand
        if((joints.position.at(25) != 1)&&(joints.position.at(25) != 0))
        {
            check=false;
            cout << "25" << endl;
        }

        return check;
    }
// move Robot
    void moveRobot()
    {
        while(nodeh.ok())
        {

            if(current_state.name.size() != 0)
            {

                if (check_joint_limits(desired_states))
                {
                    naoqi_bridge_msgs::JointAnglesWithSpeedActionGoal action;
                    stringstream ss;
                    ss << ros::Time::now().sec;
                    action.goal_id.id = "move_" + ss.str();
                    action.goal.joint_angles.speed = 0.2;
                    action.goal.joint_angles.relative = 0;

                    for(int i = 0; i < desired_states.name.size(); i++)
                    {
                        action.goal.joint_angles.joint_names.push_back(desired_states.name[i]);
                        action.goal.joint_angles.joint_angles.push_back((float)desired_states.position[i]);
                    }

                    action.header.stamp = ros::Time::now();
                    joints_move_pub.publish(action);
                    current_joint_action_status = 0;
                    ros::Rate rate_sleep(50);

                    /*while(current_joint_action_status != 3)
                    {
                        rate_sleep.sleep();
                        current_joint_state_pub.publish(current_state);

                    }*/
                }
                else
                {
                    cout << "joint limits out of range" << endl;
                }
            }
        }
    }
};




int main(int argc, char** argv)
{
    //ros::init(argc, argv, "control");

    //ros::NodeHandle n;
    //ros::Rate rate_sleep(50);

    vector<Vector3d> target;
    Vector3d q1; q1 << 0, 0, 0; target.push_back(q1);
    Vector3d q2; q2 << 0.2, 0.8, -0.6; target.push_back(q2);
    Vector3d q3; q3 << 1.0, 2, 0; target.push_back(q3);

    cout << linterp(0, target) << endl;

    /*vector<double> angles = left_angles(target);

    sensor_msgs::JointState states;
    states.name.push_back("LShoulderPitch");
    states.position.push_back(angles[0]);
    states.name.push_back("LShoulderRoll");
    states.position.push_back(angles[1]);
    states.name.push_back("LElbowYaw");
    states.position.push_back(angles[2]);
    states.name.push_back("LElbowRoll");
    states.position.push_back(angles[3]);
    states.name.push_back("LWristYaw");
    states.position.push_back(angles[4]);
    states.name.push_back("LHand");
    states.position.push_back(angles[5]);

    states.name.push_back("RShoulderPitch");
    states.position.push_back(0.0);
    states.name.push_back("RShoulderRoll");
    states.position.push_back(0.0);
    states.name.push_back("RElbowYaw");
    states.position.push_back(0.0);
    states.name.push_back("RElbowRoll");
    states.position.push_back(0.7);
    states.name.push_back("RWristYaw");
    states.position.push_back(0.0);
    states.name.push_back("RHand");
    states.position.push_back(1.0);

    Nao_control control(states);
    control.moveRobot();*/

    return 0;
}
