/*-------------------------------------Joint Space Controller------------------------------*/
/* 10.12.15 Florian Köhler */

#include "load.hpp"
#include <vector>
#include "Kinematics/NAOKinematics.h"
#include "Kinematics/KMat.hpp"

using namespace std;
using namespace KMath::KMat;
using namespace KDeviceLists;


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

    Nao_control()
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
        if(tactileState->button==naoqi_bridge_msgs::TactileTouch::buttonRear)
        {
            if(tactileState->state==naoqi_bridge_msgs::TactileTouch::statePressed)
            {
              target_sequence left_targets, right_targets;
              load_msr_skeleton("src/imitation/a01_s01_e01_skeleton.txt", left_targets, right_targets);
              do_sequence(left_targets, right_targets);
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
            //cout << current_state.position[i] << endl;
        }
     }

// move Robot
    void moveRobot()
    {
        if(current_state.name.size() != 0)
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
          ros::spinOnce();
        }
        desired_states.name.clear();
        desired_states.position.clear();
    }

// check Center of mass
    bool chkcom()
        {
            NAOKinematics kin;

            std::vector<float> jointangles;

            while (current_state.position.size() == 0)
            {

            }
            cout << current_state.position.size()<< endl;


            cout << M_PI_2 << endl;
            while (current_state.position.size() != 0)
            {
            for(int a = 0; a < current_state.position.size(); a++)
            {
                jointangles.push_back(current_state.position[a]);
                cout << jointangles[a] << endl;

            }

            /*kin.setJoints(jointangles);
            KVecDouble3 sumcom = kin.calculateCenterOfMass();
            cout << "Center of mass x: " << sumcom(0,0) << " y: " << sumcom(1,0) << " z: " << sumcom(2,0) << endl;

            NAOKinematics::kmatTable output = kin.getForwardEffector((NAOKinematics::Effectors)CHAIN_L_ARM);
            cout << "LArm x: " << output(0,3) << " y: " << output(1,3) << " z: " << output(2,3) << endl;
            */
            cout << current_state.position.size() << endl << endl;
            sleep(4);
            }
            /*

            if(sumcom(0,0)>output(0,3)+70 || sumcom(0,0)<output(0,3)-30)
            {
                return false;
            }

            else if(sumcom(0,1)>output(1,3)+20 || sumcom(0,1)<output(1,3)-30)
            {
                return false;
            }
            else
            {
                return true;
            }
            */
            return true;
        }

    void imitate_left(const vector<Vector3d>& target)
    {
      try
      {
        dlib_vector starting_point(5); starting_point = 0,0,0,-0.1,0;
        dlib_vector lower_bound(5); lower_bound = -1.9, -0.3, -1.9, -1.5, -1.7;
        dlib_vector upper_bound(5); upper_bound = 1.9, 1.3, 1.9, -0.03, 1.7;
        dlib::find_min_bobyqa(objective_function(left_arm_normalized, target, 30),
    			  starting_point,
    			  10,
    			  lower_bound,
    			  upper_bound,
    			  0.5,
    			  0.005,
    			  2000);

        desired_states.name.push_back("LShoulderPitch");
        desired_states.position.push_back(starting_point(0));
        desired_states.name.push_back("LShoulderRoll");
        desired_states.position.push_back(starting_point(1));
        desired_states.name.push_back("LElbowYaw");
        desired_states.position.push_back(starting_point(2));
        desired_states.name.push_back("LElbowRoll");
        desired_states.position.push_back(starting_point(3));
        desired_states.name.push_back("LWristYaw");
        desired_states.position.push_back(starting_point(4));
      }
      catch(exception& e)
      {
        cout << e.what() << endl;
      }
    }

    void imitate_right(const vector<Vector3d>& target)
    {
      try
      {
        dlib_vector starting_point(5); starting_point = 0,0,0,0.1,0;
        dlib_vector lower_bound(5); lower_bound = -1.9, -1.3, -1.9, 0.04, -1.7;
        dlib_vector upper_bound(5); upper_bound = 1.9, 0.3, 1.9, 1.5, 1.7;
        dlib::find_min_bobyqa(objective_function(right_arm_normalized, target, 30),
                  starting_point,
                  10,
                  lower_bound,
                  upper_bound,
                  0.5,
                  0.005,
                  2000);

        desired_states.name.push_back("RShoulderPitch");
        desired_states.position.push_back(starting_point(0));
        desired_states.name.push_back("RShoulderRoll");
        desired_states.position.push_back(starting_point(1));
        desired_states.name.push_back("RElbowYaw");
        desired_states.position.push_back(starting_point(2));
        desired_states.name.push_back("RElbowRoll");
        desired_states.position.push_back(starting_point(3));
        desired_states.name.push_back("RWristYaw");
        desired_states.position.push_back(starting_point(4));
      }
      catch(exception& e)
      {
        cout << e.what() << endl;
      }
    }

    void do_sequence(const target_sequence& left_seq, const target_sequence& right_seq)
    {
      if(left_seq.size() != right_seq.size())
      {
        cerr << "Sequences must be the same length!" << endl;
        return;
      }

      for(int i = 0; i < left_seq.size(); i++)
      {
        imitate_left(left_seq[i]);
        imitate_right(right_seq[i]);
        moveRobot();
        //sleep(0.05);
      }
    }
  };