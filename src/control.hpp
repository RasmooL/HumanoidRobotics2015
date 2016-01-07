/*-------------------------------------Joint Space Controller------------------------------*/
/* 10.12.15 Florian Köhler */

#include "load.hpp"
#include <vector>
#include "Kinematics/NAOKinematics.h"
#include "Kinematics/KMat.hpp"
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>

#include "incVision/vision.h"


using namespace std;
using namespace cv;
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
    // Recognition
    ros::Subscriber recog_sub;
	// Vision
	ros::Subscriber vision_sub;
	// subscriber to bumpers states
	ros::Subscriber bumper_sub;

    // Publisher
    //Joint angles
    ros::Publisher joints_move_pub;
    //joint states
    ros::Publisher current_joint_state_pub;
    //Speech
    ros::Publisher speech_pub;
    //Vocabulary
    ros::Publisher voc_params_pub;


    // Services
    ros::ServiceClient stiffness_disable_srv;
    ros::ServiceClient stiffness_enable_srv;
	ros::ServiceClient recog_start_srv;
    ros::ServiceClient recog_stop_srv;

    // Variables
    //state of joints
    sensor_msgs::JointState current_state;
    //goal state of joints
    sensor_msgs::JointState desired_states;
    //
    sensor_msgs::JointState balance_state;
    //status of the joint action execution
    int current_joint_action_status;
    //status of balance
    bool bal_RLeg;

    boost::thread *spin_thread;

	// vision
	int visionState;

    Arm arm_left;
    Arm arm_right;

    Nao_control() : visionState(INIT_COLOR_LEFT), arm_left("arm_left"), arm_right("arm_right")
    {
        //Subscriber initialization
        // subscribe to topic joint_states and specify that all data will be processed by function sensorCallback
        sensor_data_sub = nodeh.subscribe("/nao/joint_states",1, &Nao_control::sensorCB, this);
        // subscribe to topic tactile_touch and specify that all data will be processed by function tactileCallback
        tactile_sub = nodeh.subscribe("/nao/tactile_touch",1, &Nao_control::tactileCB, this);
        // subscribe for joint action status updates
        joint_action_status_sub = nodeh.subscribe("/nao/joint_angles_action/status",1, &Nao_control::jointActionStatusCB, this);
        // subscribe for recognition
        //recog_sub=nodeh.subscribe("/nao/word_recognized",1, &Nao_control::speechRecognitionCB, this);
		// subscribe images from top camera
		vision_sub = nodeh.subscribe("/nao/nao_robot/camera/top/camera/image_raw", 1, &Nao_control::visionCB, this);
		// subscribe bumper
		bumper_sub= nodeh.subscribe("/nao/bumper",1, &Nao_control::bumperCallback, this);

        //Publisher initialization
        //setup publisher for joint angles, the message type is JointAnglesWithSpeedActionGoal
        joints_move_pub = nodeh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeedActionGoal>("/nao/joint_angles_action/goal", 1);
        // publishers of current nao joint states
        current_joint_state_pub = nodeh.advertise<sensor_msgs::JointState>("/joint_states", 1);
        // publisher for speech
        speech_pub = nodeh.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/nao/speech_action/goal", 1);
        // publisher for vocab used
        voc_params_pub= nodeh.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>("/nao/speech_vocabulary_action/goal", 1);

        // initialize service
        //for stiffeness
        stiffness_disable_srv  = nodeh.serviceClient<std_srvs::Empty>("/nao/body_stiffness/disable");
        stiffness_enable_srv = nodeh.serviceClient<std_srvs::Empty>("/nao/body_stiffness/enable");
        //for speech
        recog_start_srv=nodeh.serviceClient<std_srvs::Empty>("/nao/start_recognition");
		    recog_stop_srv=nodeh.serviceClient<std_srvs::Empty>("/nao/stop_recognition");

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
              //target_sequence left_targets, right_targets;
              //load_msr_skeleton("src/imitation/drink.txt", left_targets, right_targets);
              //do_sequence(left_targets, right_targets);
              for(vector<string>::iterator it = desired_states.name.begin(); it != desired_states.name.end(); it++) cout << *it << endl;
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
            if( i > 7 && i < 20)
            {
//              cout << jointState->name.at(i) << ":   " << jointState->position.at(i) << endl;
            }
        }
     }
    /*void speechRecognitionCB(const naoqi_bridge_msgs::word_recognized::ConstPtr& msg)
    {

    }*/
	
	// vision
	void visionCB(const sensor_msgs::Image::ConstPtr& Img)
	{
		//receive image and convert to BGR8
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(Img, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		if(SHOW_ORIGINAL_IMG == ON)
			imshow("img", cv_ptr->image);
		
		switch(visionState)
		{
		case INIT_COLOR_LEFT:
		    stateInitColor(cv_ptr->image, arm_left);
		    break;
		
		case DESTROY_WINDOW_INIT_COLOR_LEFT:
			visionState = INIT_COLOR_RIGHT;
			destroyAllWindows();
			break;

		case INIT_COLOR_RIGHT:
		    stateInitColor(cv_ptr->image, arm_right);
		    break;

		case DESTROY_WINDOW_INIT_COLOR_RIGHT:
			visionState = INIT_ARM_LEFT;
			destroyAllWindows();
			break;

		case INIT_ARM_LEFT:
		    visionState = stateInitArm(cv_ptr->image, &arm_left);
			break;

		case INIT_ARM_RIGHT:
		    visionState = stateInitArm(cv_ptr->image, &arm_right);
			break;

		case RUN_BODY_TRACKING:
		    if(runArmTracking(cv_ptr->image, &arm_left, &arm_right) == SUCCESS)
			{
/*				// left arm
				cout << arm_left->getArmName() << endl;

        		Vector3d jointLeft;

        		jointLeft = arm_left->getJ1Coord();
        		cout << "j1   " << jointLeft(0) << "  " << jointLeft(1) << "  " << jointLeft(2) << endl;

        		jointLeft = arm_left->getJ2Coord();
        		cout << "j2   " << jointLeft(0) << "  " << jointLeft(1) << "  " << jointLeft(2) << endl;

        		jointLeft = arm_left.getJ3Coord();
        		cout << "j3   " << jointLeft(0) << "  " << jointLeft(1) << "  " << jointLeft(2) << endl;

				cout << arm_right->getArmName() << endl;


				// right arm
        		Vector3d jointRight;

        		jointRight = arm_right->getJ1Coord();
        		cout << "j1   " << jointRight(0) << "  " << jointRight(1) << "  " << jointRight(2) << endl;

        		jointRight = arm_right->getJ2Coord();
        		cout << "j2   " << jointRight(0) << "  " << jointRight(1) << "  " << jointRight(2) << endl;

        		jointRight = arm_right->getJ3Coord();
        		cout << "j3   " << jointRight(0) << "  " << jointRight(1) << "  " << jointRight(2) << endl << endl << endl;
*/
			}
/*			else 
				cout << "not all joints were found\n";	
*/
		    break;

		default:
		    cout << "wrong state: " << visionState << endl;
		}

		waitKey(5);
	}

	void bumperCallback(const naoqi_bridge_msgs::Bumper::ConstPtr& bumperState)
	{
        // Left/right bumper pressed
        if (bumperState->state == bumperState->statePressed)
        {
            if (bumperState->bumper == bumperState->left)
            {
				visionState = getNextState(visionState);
			}
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
              cout << desired_states.position[i] << endl;

          }

          action.header.stamp = ros::Time::now();
          joints_move_pub.publish(action);
          ros::spinOnce();
        }
        desired_states.name.clear();
        desired_states.position.clear();
    }

    void balance()
    {
      //standup

      //balance
      desired_states.name.clear();
      desired_states.position.clear();

      desired_states.name.push_back("LHipYawPitch");
      desired_states.position.push_back(-0.29602);
      desired_states.name.push_back("LHipRoll");
      desired_states.position.push_back(-0.079726);
      desired_states.name.push_back("LHipPitch");
      desired_states.position.push_back(-0.539926);
      desired_states.name.push_back("LKneePitch");
      desired_states.position.push_back(2.11074);
      desired_states.name.push_back("LAnklePitch");
      desired_states.position.push_back(-1.18944);
      desired_states.name.push_back("LAnkleRoll");
      desired_states.position.push_back(-0.0183661);
      desired_states.name.push_back("RHipYawPitch");
      desired_states.position.push_back(-0.00302601);
      desired_states.name.push_back("RHipRoll");
      desired_states.position.push_back(0.147306);
      desired_states.name.push_back("RHipPitch");
      desired_states.position.push_back(0.239262);
      desired_states.name.push_back("RKneePitch");
      desired_states.position.push_back(-0.0923279);
      desired_states.name.push_back("RAnklePitch");
      desired_states.position.push_back(0.0061779);
      desired_states.name.push_back("RAnkleRoll");
      desired_states.position.push_back(-0.170232);
    }



// check Center of mass
    bool chkCoM_RLeg()
        {
            NAOKinematics kin;
            std::vector<float> jointangles(26, 0);
            // two while loops probably not necessary in action
            while (current_state.position.size() == 0)
            {

            }

            while(current_state.position.size() != 0)
            {
              for(int a = 0; a < current_state.position.size(); a++)
              {
                jointangles[a]=current_state.position[a];
              }

              kin.setJoints(jointangles);
              KVecDouble3 sumcom = kin.calculateCenterOfMass();
              //cout << "Center of mass x: " << sumcom(0,0) << " y: " << sumcom(1,0) << " z: " << sumcom(2,0) << endl;
              NAOKinematics::kmatTable rlegpos = kin.getForwardEffector((NAOKinematics::Effectors)CHAIN_R_LEG);
              //cout << "RLeg x: " << rlegpos(0,3) << " y: " << rlegpos(1,3) << " z: " << rlegpos(2,3) << endl;

              if(sumcom(0,0)>rlegpos(0,3)+70 || sumcom(0,0)<rlegpos(0,3)-30)
              {
                  return false;
              }

              else if(sumcom(1,0)>rlegpos(1,3)+20 || sumcom(1,0)<rlegpos(1,3)-30)
              {
                  return false;
              }
              else
              {
                  return true;
              }
            }
        }



    void imitate_left(const vector<Vector3d>& target)
    {
      try
      {
        const int interp_pts = 20;
        const int bobyqa_pts = 10;
        const float start_trustregion = 0.4;
        const float end_trustregion = 0.01;
        const float max_iter = 500;
        dlib_vector starting_point(5); starting_point = 0,0,0,-0.1,0;
        dlib_vector lower_bound(5); lower_bound = -1.9, -0.3, -1.9, -1.5, -1.7;
        dlib_vector upper_bound(5); upper_bound = 1.9, 1.3, 1.9, -0.03, 1.7;
        dlib::find_min_bobyqa(objective_function(left_arm_normalized, target, interp_pts),
    			  starting_point,
    			  bobyqa_pts,
    			  lower_bound,
    			  upper_bound,
    			  start_trustregion,
    			  end_trustregion,
    			  max_iter);

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
        const int interp_pts = 20;
        const int bobyqa_pts = 10;
        const float start_trustregion = 0.4;
        const float end_trustregion = 0.01;
        const float max_iter = 500;
        dlib_vector starting_point(5); starting_point = 0, 0, 0, 0.1, 0;
        dlib_vector lower_bound(5); lower_bound = -1.9, -1.3, -1.9, 0.04, -1.7;
        dlib_vector upper_bound(5); upper_bound = 1.9, 0.3, 1.9, 1.5, 1.7;
        dlib::find_min_bobyqa(objective_function(right_arm_normalized, target, interp_pts),
    			  starting_point,
    			  bobyqa_pts,
    			  lower_bound,
    			  upper_bound,
    			  start_trustregion,
    			  end_trustregion,
    			  max_iter);

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
        /*if (bal_RLeg)
        {
          if(!chkCoM_RLeg())
          {
            break;
          }
        }*/
      }
    }
  };
