/*-------------------------------------Joint Space Controller------------------------------*/
/* 10.12.15 Florian Köhler */

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
            cout << "Joint status CB: " << current_joint_action_status << endl;
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

                vector<Vector3d> left_target;
                Vector3d q1; q1 << 0, 0, 0; left_target.push_back(q1);
                Vector3d q2; q2 << 0.1, 0.3, 0.0; left_target.push_back(q2);
                Vector3d q3; q3 << 0.3, 0.5, -0.1; left_target.push_back(q3);

                vector<Vector3d> right_target;
                q1 << 0, 0, 0; right_target.push_back(q1);
                q2 << 0.1, -0.3, 0.0; right_target.push_back(q2);
                q3 << 0.3, -0.5, -0.1; right_target.push_back(q3);

                imitate_left(left_target);
                imitate_right(right_target);
                moveRobot();
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
        cout << "Done with movement" << endl;
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

        cout << starting_point << endl;

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
        cout << starting_point << endl;

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
};
