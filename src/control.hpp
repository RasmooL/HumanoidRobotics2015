/*-------------------------------------Joint Space
 * Controller------------------------------*/
/* 10.12.15 Florian Köhler */

#include "load.hpp"
#include <vector>
#include "Kinematics/NAOKinematics.h"
#include "Kinematics/KMat.hpp"
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "incVision/vision.h"

using namespace std;
using namespace cv;
using namespace KMath::KMat;
using namespace KDeviceLists;

bool stop_thread = false;

void spinThread() {
  while (!stop_thread) {
    ros::spinOnce();
  }
}

class Nao_control {
public:
  // NODEHANDLE
  ros::NodeHandle nodeh;

  // SUBSCRIBER
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

  // PUBLISHER
  // Joint angles
  ros::Publisher joints_move_pub;
  // joint states
  ros::Publisher current_joint_state_pub;
  // Speech
  ros::Publisher speech_pub;
  // Vocabulary
  ros::Publisher voc_params_pub;

  // SERVICES
  ros::ServiceClient stiffness_disable_srv;
  ros::ServiceClient stiffness_enable_srv;
  ros::ServiceClient recog_start_srv;
  ros::ServiceClient recog_stop_srv;

  // VARIABLES
  // state of joints
  sensor_msgs::JointState current_state;
  // goal state of joints
  sensor_msgs::JointState desired_states;
  // status of the joint action execution
  int current_joint_action_status;
  // status of balance
  bool bal_RLeg;
  // Kinematics object
  NAOKinematics kin;
  // vision
  int visionState;

  Arm arm_left;
  Arm arm_right;
  tf::TransformListener *listener;

  // initial optimization solutions
  dlib_vector leftsol, rightsol;

  boost::thread *spin_thread;
  // CONSTRUCTOR
  Nao_control(tf::TransformListener *listener_m)
      : visionState(INIT_COLOR_LEFT), arm_left("arm_left"),
        arm_right("arm_right"),
        leftsol(5), rightsol(5) {
    listener = listener_m;
    // Subscriber initialization
    // subscribe for recognition
    // recog_sub=nodeh.subscribe("/nao/word_recognized",1,
    // &Nao_control::speechRecognitionCB, this);
    // subscribe to topic joint_states and specify that all data will be
    // processed by function sensorCallback
    sensor_data_sub =
        nodeh.subscribe("/nao/joint_states", 1, &Nao_control::sensorCB, this);
    // subscribe to topic tactile_touch and specify that all data will be
    // processed by function tactileCallback
    tactile_sub =
        nodeh.subscribe("/nao/tactile_touch", 1, &Nao_control::tactileCB, this);
    // subscribe for joint action status updates
    joint_action_status_sub =
        nodeh.subscribe("/nao/joint_angles_action/status", 1,
                        &Nao_control::jointActionStatusCB, this);
    // subscribe images from top camera
    // vision_sub =
    // nodeh.subscribe("/nao/nao_robot/camera/top/camera/image_raw", 1,
    // &Nao_control::visionCB, this);
    // subscribe bumper
    bumper_sub =
        nodeh.subscribe("/nao/bumper", 1, &Nao_control::bumperCallback, this);

    // Publisher initialization
    // setup publisher for joint angles, the message type is
    // JointAnglesWithSpeedActionGoal
    joints_move_pub =
        nodeh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeedActionGoal>(
            "/nao/joint_angles_action/goal", 1);
    // publishers of current nao joint states
    current_joint_state_pub =
        nodeh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    // publisher for speech
    speech_pub =
        nodeh.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>(
            "/nao/speech_action/goal", 1);
    // publisher for vocab used
    voc_params_pub =
        nodeh.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>(
            "/nao/speech_vocabulary_action/goal", 1);

    // initialize service
    // for stiffeness
    stiffness_disable_srv =
        nodeh.serviceClient<std_srvs::Empty>("/nao/body_stiffness/disable");
    stiffness_enable_srv =
        nodeh.serviceClient<std_srvs::Empty>("/nao/body_stiffness/enable");
    // for speech
    recog_start_srv =
        nodeh.serviceClient<std_srvs::Empty>("/nao/start_recognition");
    recog_stop_srv =
        nodeh.serviceClient<std_srvs::Empty>("/nao/stop_recognition");

    // set variables
    bal_RLeg = 0;

    // initial left and right arm solutions for optimization (the newest solution is always kept in these)
    leftsol = 0, 0, 0, -0.1, 0;
    rightsol = 0, 0, 0, 0.1, 0;

    stop_thread = false;
    spin_thread = new boost::thread(&spinThread);
  }
  // DESTRUCTOR
  ~Nao_control() {
    stop_thread = true;
    sleep(1);
    spin_thread->join();
  }

  // CALLBACKs
  /*void speechRecognitionCB(const naoqi_bridge_msgs::word_recognized::ConstPtr&
  msg)
  {}*/
  // current action status
  void
  jointActionStatusCB(const actionlib_msgs::GoalStatusArray::ConstPtr &msg) {
    if (!msg->status_list.empty()) {
      current_joint_action_status = (int)msg->status_list.at(0).status;
    }
  }
  // head tactile buttons
  void
  tactileCB(const naoqi_bridge_msgs::TactileTouch::ConstPtr &tactileState) {

    if (tactileState->button == naoqi_bridge_msgs::TactileTouch::buttonMiddle) {
      if (tactileState->state ==
          naoqi_bridge_msgs::TactileTouch::statePressed) {
        // generate empty request
        std_srvs::Empty srv;
        stiffness_disable_srv.call(srv);
      }
    }

    if (tactileState->button == naoqi_bridge_msgs::TactileTouch::buttonFront) {
      if (tactileState->state ==
          naoqi_bridge_msgs::TactileTouch::statePressed) {
        std_srvs::Empty srv;
        stiffness_enable_srv.call(srv);
      }
    }
    if (tactileState->button == naoqi_bridge_msgs::TactileTouch::buttonRear) {
      if (tactileState->state ==
          naoqi_bridge_msgs::TactileTouch::statePressed) {
        // target_sequence left_targets, right_targets;
        // load_msr_skeleton("src/imitation/drink.txt", left_targets,
        // right_targets);
        // do_sequence(left_targets, right_targets);
      }
    }

    if (tactileState->button == naoqi_bridge_msgs::TactileTouch::buttonFront) {
      if (tactileState->state ==
          naoqi_bridge_msgs::TactileTouch::statePressed) {
        std_srvs::Empty srv;
        stiffness_enable_srv.call(srv);
      }
    }
  }
  // foot bumper buttons
  void bumperCallback(const naoqi_bridge_msgs::Bumper::ConstPtr &bumperState) {
    // Left/right bumper pressed
    if (bumperState->bumper == bumperState->left) {
      if (bumperState->state == bumperState->statePressed) {
        visionState = getNextState(visionState);
      }
    }
    if (bumperState->bumper == bumperState->right) {
      if (bumperState->state == bumperState->statePressed && bal_RLeg == 0) {
        balance();
      } else if (bumperState->state == bumperState->statePressed &&
                 bal_RLeg == 1) {
        stand();
      }
    }
  }
  // current joint states
  void sensorCB(const sensor_msgs::JointState::ConstPtr &jointState) {
    current_state.name.clear();
    current_state.position.clear();
    current_state.header.stamp = ros::Time::now();
    std::vector<float> jointangles;

    for (int i = 0; i < jointState->name.size(); i++) {
      current_state.name.push_back(jointState->name.at(i));
      current_state.position.push_back(jointState->position.at(i));
      jointangles.push_back(jointState->position.at(i));
      // if( i > 7 && i < 20) cout << jointState->name.at(i) << ":   " <<
      // jointState->position.at(i) << endl;
    }
    current_joint_state_pub.publish(current_state);
    kin.setJoints(jointangles);
  }
  // camera
  void visionCB(const sensor_msgs::Image::ConstPtr &Img) {
    // receive image and convert to BGR8
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(Img, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (SHOW_ORIGINAL_IMG == ON)
      imshow("img", cv_ptr->image);

    switch (visionState) {
    case INIT_COLOR_LEFT:
      stateInitColor(cv_ptr->image, arm_left);
      break;

    case DESTROY_WINDOW_INIT_COLOR_LEFT:
      visionState = INIT_COLOR_RIGHT;
      destroyAllWindows();
      break;

    case INIT_COLOR_RIGHT:
      visionState = INIT_ARM_LEFT;
      // stateInitColor(cv_ptr->image, arm_right);
      break;

    case DESTROY_WINDOW_INIT_COLOR_RIGHT:
      visionState = INIT_ARM_LEFT;
      destroyAllWindows();
      break;

    case INIT_ARM_LEFT:
      visionState = stateInitArm(cv_ptr->image, &arm_left);
      break;

    case INIT_ARM_RIGHT:
      // visionState = stateInitArm(cv_ptr->image, &arm_right);
      visionState = RUN_BODY_TRACKING;
      break;

    case RUN_BODY_TRACKING:
      if (runArmTracking(cv_ptr->image, &arm_left) == SUCCESS) {
        /*// left arm
        cout << arm_left->getArmName() << endl;

Vector3d jointLeft;

jointLeft = arm_left->getJ1Coord();
cout << "j1   " << jointLeft(0) << "  " << jointLeft(1) << "  " << jointLeft(2)
<< endl;

jointLeft = arm_left->getJ2Coord();
cout << "j2   " << jointLeft(0) << "  " << jointLeft(1) << "  " << jointLeft(2)
<< endl;

jointLeft = arm_left.getJ3Coord();
cout << "j3   " << jointLeft(0) << "  " << jointLeft(1) << "  " << jointLeft(2)
<< endl;

        cout << arm_right->getArmName() << endl;


        // right arm
Vector3d jointRight;

jointRight = arm_right->getJ1Coord();
cout << "j1   " << jointRight(0) << "  " << jointRight(1) << "  " <<
jointRight(2) << endl;

jointRight = arm_right->getJ2Coord();
cout << "j2   " << jointRight(0) << "  " << jointRight(1) << "  " <<
jointRight(2) << endl;

jointRight = arm_right->getJ3Coord();
cout << "j3   " << jointRight(0) << "  " << jointRight(1) << "  " <<
jointRight(2) << endl << endl << endl;*/

        vector<Vector3d> left_target;
        left_target.push_back(arm_left.getJ1Coord());
        left_target.push_back(arm_left.getJ2Coord());
        left_target.push_back(arm_left.getJ3Coord());
        imitate_left(left_target, leftsol);
        moveRobot(0.2);
      }
      /*else
              cout << "not all joints were found\n";*/

      break;

    default:
      visionState = INIT_COLOR_LEFT;
      destroyAllWindows();
    }

    waitKey(5);
  }

  // STANCEs
  void balance() {
    desired_states.name.clear();
    desired_states.position.clear();

    // prepbalance
    desired_states.name.push_back("LHipYawPitch");
    desired_states.position.push_back(0.0276539);
    desired_states.name.push_back("LHipRoll");
    desired_states.position.push_back(0.368202);
    desired_states.name.push_back("LHipPitch");
    desired_states.position.push_back(0.0153821);
    desired_states.name.push_back("LKneePitch");
    desired_states.position.push_back(-0.0923279);
    desired_states.name.push_back("LAnklePitch");
    desired_states.position.push_back(0.141086);
    desired_states.name.push_back("LAnkleRoll");
    desired_states.position.push_back(-0.35738);
    desired_states.name.push_back("RHipYawPitch");
    desired_states.position.push_back(0.0276539);
    desired_states.name.push_back("RHipRoll");
    desired_states.position.push_back(0.374338);
    desired_states.name.push_back("RHipPitch");
    desired_states.position.push_back(0.159494);
    desired_states.name.push_back("RKneePitch");
    desired_states.position.push_back(-0.0923279);
    desired_states.name.push_back("RAnklePitch");
    desired_states.position.push_back(0.0061779);
    desired_states.name.push_back("RAnkleRoll");
    desired_states.position.push_back(-0.368118);

    moveRobot(0.05);
    sleep(2);

    // finbalance
    desired_states.name.push_back("LHipYawPitch");
    desired_states.position.push_back(0.021518);
    desired_states.name.push_back("LHipRoll");
    desired_states.position.push_back(0.372804);
    desired_states.name.push_back("LHipPitch");
    desired_states.position.push_back(-0.412604);
    desired_states.name.push_back("LKneePitch");
    desired_states.position.push_back(1.18421);
    desired_states.name.push_back("LAnklePitch");
    desired_states.position.push_back(-0.546146);
    desired_states.name.push_back("LAnkleRoll");
    desired_states.position.push_back(-0.335904);
    desired_states.name.push_back("RHipYawPitch");
    desired_states.position.push_back(0.0276539);
    desired_states.name.push_back("RHipRoll");
    desired_states.position.push_back(0.374338);
    desired_states.name.push_back("RHipPitch");
    desired_states.position.push_back(0.159494);
    desired_states.name.push_back("RKneePitch");
    desired_states.position.push_back(-0.0923279);
    desired_states.name.push_back("RAnklePitch");
    desired_states.position.push_back(0.0061779);
    desired_states.name.push_back("RAnkleRoll");
    desired_states.position.push_back(-0.368118);

    moveRobot(0.1);
    sleep(2);
    bal_RLeg = 1;
  }
  void stand() {
    desired_states.name.clear();
    desired_states.position.clear();

    // prepbalance
    desired_states.name.push_back("LHipYawPitch");
    desired_states.position.push_back(0.0276539);
    desired_states.name.push_back("LHipRoll");
    desired_states.position.push_back(0.368202);
    desired_states.name.push_back("LHipPitch");
    desired_states.position.push_back(0.0153821);
    desired_states.name.push_back("LKneePitch");
    desired_states.position.push_back(-0.0923279);
    desired_states.name.push_back("LAnklePitch");
    desired_states.position.push_back(0.141086);
    desired_states.name.push_back("LAnkleRoll");
    desired_states.position.push_back(-0.35738);
    desired_states.name.push_back("RHipYawPitch");
    desired_states.position.push_back(0.0276539);
    desired_states.name.push_back("RHipRoll");
    desired_states.position.push_back(0.374338);
    desired_states.name.push_back("RHipPitch");
    desired_states.position.push_back(0.159494);
    desired_states.name.push_back("RKneePitch");
    desired_states.position.push_back(-0.0923279);
    desired_states.name.push_back("RAnklePitch");
    desired_states.position.push_back(0.0061779);
    desired_states.name.push_back("RAnkleRoll");
    desired_states.position.push_back(-0.368118);

    moveRobot(0.1);
    sleep(2);
    bal_RLeg = 0;

    // stand
    desired_states.name.push_back("LHipYawPitch");
    desired_states.position.push_back(0.0859461);
    desired_states.name.push_back("LHipRoll");
    desired_states.position.push_back(0.0521979);
    desired_states.name.push_back("LHipPitch");
    desired_states.position.push_back(0.107422);
    desired_states.name.push_back("LKneePitch");
    desired_states.position.push_back(-0.0923279);
    desired_states.name.push_back("LAnklePitch");
    desired_states.position.push_back(0.0413761);
    desired_states.name.push_back("LAnkleRoll");
    desired_states.position.push_back(-0.0613179);
    desired_states.name.push_back("RHipYawPitch");
    desired_states.position.push_back(0.0859461);
    desired_states.name.push_back("RHipRoll");
    desired_states.position.push_back(-0.110406);
    desired_states.name.push_back("RHipPitch");
    desired_states.position.push_back(0.0613179);
    desired_states.name.push_back("RKneePitch");
    desired_states.position.push_back(-0.0720561);
    desired_states.name.push_back("RAnklePitch");
    desired_states.position.push_back(0.0123138);
    desired_states.name.push_back("RAnkleRoll");
    desired_states.position.push_back(0.10282);

    moveRobot(0.05);
    sleep(2);
  }

  /*// BALANCING
      bool CoM_chk_and_adjust()
      {
        if (bal_RLeg)
        {
          if (chk_CoM_RLeg()[0]) return true;
          else if(comp_movement(chk_CoM_RLeg())) return true;
          else return false;
        }
      }

      std::vector<bool> chk_CoM_RLeg()
      {
                    std::vector<bool> commatch(5) = 1;
                    Eigen::Vector4d sumcom;
                    sumcom =  Tf_torso_rfoot();
                    //cout << "CoM   x: " << sumcom(0) << endl; //<< " y: " <<
     sumcom(1) << " z: " << sumcom(2) << endl;

                    if(-30.0 > sumcom(0))
                    {
                                                //cout << "CoM is not over right
     foot in negative x-direction: " << sumcom(0) << endl;
                                                commatch[0] = false;
                      commatch[1] = false;
                    }
                    if (sumcom(0) > 40)
                    {
                      //cout << "CoM is not over right foot in positive
     x-direction: " << sumcom(0) << endl;
                      commatch[0] = false;
                      commatch[2] = false;
                    }
                    if(-30.0 > sumcom(1))
                    {
                                                //cout << "CoM is not over right
     foot in negative y-direction: " << sumcom(1) << endl;
                                                commatch[0] = false;
                      commatch[3] = false;
                    }
                    if (sumcom(1) > 23.0)
                    {
                      //cout << "CoM is not over right foot in positive
     y-direction: " << sumcom(1) << endl;
                                                commatch[0] = false;
                      commatch[4] = false;
                    }
                    if (sumcom(1) < 23.0 && -30.0 < sumcom(1) && sumcom(0) < 40
     && -30.0 < sumcom(0))
                    {
                                                    commatch[0] = true;
                          commatch[1] = true;
                          commatch[2] = true;
                          commatch[3] = true;
                          commatch[4] = true;
                    }
                    return commatch;
      }
      Eigen::Vector4d Tf_torso_rfoot()
      {
        tf::StampedTransform transform;
        try
        {
          listener->lookupTransform("/r_sole", "/torso", ros::Time(0),
     transform);
        }
        catch(tf::TransformException ex)
        {
              ROS_ERROR("%s",ex.what());
        }
        Eigen::Matrix4d T_torso_rfoot;
        T_torso_rfoot(0,0)=transform.getBasis()[0][0];
        T_torso_rfoot(0,1)=transform.getBasis()[0][1];
        T_torso_rfoot(0,2)=transform.getBasis()[0][2];
        T_torso_rfoot(0,3)=transform.getOrigin().getX();
        T_torso_rfoot(1,0)=transform.getBasis()[1][0];
        T_torso_rfoot(1,1)=transform.getBasis()[1][1];
        T_torso_rfoot(1,2)=transform.getBasis()[1][2];
        T_torso_rfoot(1,3)=transform.getOrigin().getY();
        T_torso_rfoot(2,0)=transform.getBasis()[2][0];
        T_torso_rfoot(2,1)=transform.getBasis()[2][1];
        T_torso_rfoot(2,2)=transform.getBasis()[2][2];
        T_torso_rfoot(2,3)=transform.getOrigin().getZ();
        T_torso_rfoot(3,0)=0;
        T_torso_rfoot(3,1)=0;
        T_torso_rfoot(3,2)=0;
        T_torso_rfoot(3,3)=1;
        //cout << T_torso_rfoot << endl;
        KVecDouble3 Sum_CoM_Torso = kin.calculateCenterOfMass();
        Eigen::Vector4d Sum_CoM_Torso_v;
        Sum_CoM_Torso_v(0) = Sum_CoM_Torso(0,0)/1000;
        Sum_CoM_Torso_v(1) = Sum_CoM_Torso(1,0)/1000;
        Sum_CoM_Torso_v(2) = Sum_CoM_Torso(2,0)/1000;
        Sum_CoM_Torso_v(3) = 1;


        Eigen::Vector4d Sum_CoM_RFoot = T_torso_rfoot * Sum_CoM_Torso_v;
        Sum_CoM_RFoot = Sum_CoM_RFoot * 1000; //in mm
        return Sum_CoM_RFoot;
      }

      bool comp_movement(std::vector<bool> com_match(5))
      {
          //build vector with desired states to modify for compensational
     movement
          sensor_msgs::JointState chk_state;
          chk_state = current_state;
          for (int v = 0; v < desired_states.position.size(); v++)
          {
            for(int u = 0; u < chk_state.position.size(); u++)
            {
              if(chk_state.name[u] == desired_states.name[v])
              {
                chk_state.position[u] = desired_states.position[v];
                break;
              }
            }

          }

          std::vector<float> jointangles(26, 0);
          for (int t = 0; t < chk_state.position.size(); t++)
          {
            jointangles[t]= chk_state.position[t];
          }
          kin.setJoints(jointangles);

        bool move;
        if (bal_RLeg)
        {
          if(current_state.name.size() != 0 && chk_CoM_RLeg())
          {move = 1;}
          else
          {move = 0;}
        }
        else
        {
          if (current_state.name.size() != 0)
          {move = 1;}
          else
          {move = 0;}
        }

        if (bal_RLeg)
        {
          if(!pos_y_lim)
          {
            //move something in neg x-direction
          }
          if(!pos_x_lim)
          {

          }
          if(!neg_y_lim)
          {

          }
          if(!neg_y_lim)
          {

          }
        }
      }
  */

  // MAPPING
  void imitate_left(const vector<Vector3d> &target, dlib_vector &solution) {
    try {
      const int interp_pts = 20;
      const int bobyqa_pts = 10;
      const float start_trustregion = 0.4;
      const float end_trustregion = 0.01;
      const float max_iter = 500;
      dlib_vector lower_bound(5);
      lower_bound = -1.9, -0.3, -1.9, -1.5, -1.7;
      dlib_vector upper_bound(5);
      upper_bound = 1.9, 1.3, 1.9, -0.03, 1.7;
      dlib::find_min_bobyqa(
          objective_function(left_arm_normalized, target, interp_pts), solution,
          bobyqa_pts, lower_bound, upper_bound, start_trustregion,
          end_trustregion, max_iter);

      desired_states.name.push_back("LShoulderPitch");
      desired_states.position.push_back(solution(0));
      desired_states.name.push_back("LShoulderRoll");
      desired_states.position.push_back(solution(1));
      desired_states.name.push_back("LElbowYaw");
      desired_states.position.push_back(solution(2));
      desired_states.name.push_back("LElbowRoll");
      desired_states.position.push_back(solution(3));
      desired_states.name.push_back("LWristYaw");
      desired_states.position.push_back(solution(4));

    } catch (exception &e) {
      cout << e.what() << endl;
    }
  }

  void imitate_right(const vector<Vector3d> &target, dlib_vector &solution) {
    try {
      const int interp_pts = 20;
      const int bobyqa_pts = 10;
      const float start_trustregion = 0.4;
      const float end_trustregion = 0.01;
      const float max_iter = 500;
      dlib_vector lower_bound(5);
      lower_bound = -1.9, -1.3, -1.9, 0.04, -1.7;
      dlib_vector upper_bound(5);
      upper_bound = 1.9, 0.3, 1.9, 1.5, 1.7;
      dlib::find_min_bobyqa(
          objective_function(right_arm_normalized, target, interp_pts),
          solution, bobyqa_pts, lower_bound, upper_bound, start_trustregion,
          end_trustregion, max_iter);

      desired_states.name.push_back("RShoulderPitch");
      desired_states.position.push_back(solution(0));
      desired_states.name.push_back("RShoulderRoll");
      desired_states.position.push_back(solution(1));
      desired_states.name.push_back("RElbowYaw");
      desired_states.position.push_back(solution(2));
      desired_states.name.push_back("RElbowRoll");
      desired_states.position.push_back(solution(3));
      desired_states.name.push_back("RWristYaw");
      desired_states.position.push_back(solution(4));

    } catch (exception &e) {
      cout << e.what() << endl;
    }
  }

  // MOVEMENT
  void moveRobot(double speed) {
    if (true) // CoM_chk_and_adjust())
    {
      naoqi_bridge_msgs::JointAnglesWithSpeedActionGoal action;
      stringstream ss;
      ss << ros::Time::now().sec;
      action.goal_id.id = "move_" + ss.str();
      action.goal.joint_angles.speed = speed;
      action.goal.joint_angles.relative = 0;

      for (int i = 0; i < desired_states.name.size(); i++) {
        action.goal.joint_angles.joint_names.push_back(desired_states.name[i]);
        action.goal.joint_angles.joint_angles.push_back(
            (float)desired_states.position[i]);
        // cout << action.goal.joint_angles.joint_names[i] << endl;
      }
      action.header.stamp = ros::Time::now();
      joints_move_pub.publish(action);

      // ros::spinOnce();

    } else {
      cout << "Goal joint states move CoM out of support polygon. No "
              "adjustment possible."
           << endl;
    }
    desired_states.name.clear();
    desired_states.position.clear();

    current_joint_state_pub.publish(current_state);
  }

  void do_sequence(const target_sequence &left_seq,
                   const target_sequence &right_seq) {
    if (left_seq.size() != right_seq.size()) {
      cerr << "Sequences must be the same length!" << endl;
      return;
    }

    for (int i = 0; i < left_seq.size(); i++) {
      imitate_left(left_seq[i], leftsol);
      imitate_right(right_seq[i], rightsol);

      moveRobot(0.2);
    }
  }
};
