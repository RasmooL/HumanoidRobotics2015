// Florian

#ifndef _CONTROL_HPP_
#define _CONTROL_HPP_

#include <vector>
#include "Kinematics/NAOKinematics.h"
#include "Kinematics/KMat.hpp"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Image.h"
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
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <numeric>
#include <std_msgs/Bool.h>

#include "incVision/vision.h"
#include "load.hpp"
#include "optim.hpp"

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace KMath::KMat;
using namespace KDeviceLists;

extern bool stop_thread;
void spinThread();

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
  // Vision
  ros::Subscriber vision_sub;
  // subscriber to bumpers states
  ros::Subscriber bumper_sub;

  // PUBLISHER
  // Non-blocking joint angles
  ros::Publisher joint_angles_pub;

  // SERVICES
  ros::ServiceClient stiffness_disable_srv;
  ros::ServiceClient stiffness_enable_srv;


  // VARIABLES
  // state of joints
  sensor_msgs::JointState current_state;
  // goal state of joints
  sensor_msgs::JointState desired_states;
  // status of the joint action execution
  int current_joint_action_status;
  // status of balance
  bool bal_RLeg;
  bool bal_2Leg;
  // Kinematics object
  NAOKinematics kin;

  Arm arm_left;
  Arm arm_right;
  Chest chest;

  // initial optimization solutions
  dlib_vector leftsol, rightsol;

  boost::thread *spin_thread;
  // CONSTRUCTOR
  Nao_control(tf::TransformListener *listener_m)
      : arm_left("arm_left"),
        arm_right("arm_right"),
        leftsol(5), rightsol(5) {

    // Subscriber initialization
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
    vision_sub =
    nodeh.subscribe("/nao/nao_robot/camera/top/camera/image_raw", 1,
                    &Nao_control::visionCB, this);
    // subscribe bumper
    bumper_sub =
        nodeh.subscribe("/nao/bumper", 1, &Nao_control::bumperCallback, this);

    // Publisher initialization
    // publisher for non-blocking joint movement
    joint_angles_pub =
        nodeh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>(
          "/nao/joint_angles", 1);

    // initialize service
    // for stiffeness
    stiffness_disable_srv =
        nodeh.serviceClient<std_srvs::Empty>("/nao/body_stiffness/disable");
    stiffness_enable_srv =
        nodeh.serviceClient<std_srvs::Empty>("/nao/body_stiffness/enable");

    // set variables
    bal_RLeg = 0;
    bal_2Leg = 1;

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
        target_sequence left_targets, right_targets;
        load_msr_skeleton("src/imitation/guitar.txt", left_targets, right_targets);

        do_sequence(left_targets, right_targets);
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
    //std::vector<float> jointa;

    for (int i = 0; i < jointState->name.size(); i++) {
      current_state.name.push_back(jointState->name.at(i));
      current_state.position.push_back(jointState->position.at(i));
      //if (i > 7 && i < 20) cout << jointState->name.at(i) << ": " << jointState->position.at(i) << endl;
      //cout << jointState->name.at(i) << ": " << jointState->position.at(i) << endl;
      //jointa.push_back(jointState->position.at(i));
    }
    //kin.setJoints(jointa);
    //cout << chk_CoM_2legs() << endl;
  }
  // camera
  void visionCB(const sensor_msgs::Image::ConstPtr &Img);

  // STANCEs
  void balance() {
    desired_states.name.clear();
    desired_states.position.clear();

    // prepbalance
    desired_states.name.push_back("LHipYawPitch");
    desired_states.position.push_back(-0.0214341);
    desired_states.name.push_back("LHipRoll");
    desired_states.position.push_back(0.412688);
    desired_states.name.push_back("LHipPitch");
    desired_states.position.push_back(0.135034);
    desired_states.name.push_back("LKneePitch");
    desired_states.position.push_back(-0.0923279);
    desired_states.name.push_back("LAnklePitch");
    desired_states.position.push_back(0.0475121);
    desired_states.name.push_back("LAnkleRoll");
    desired_states.position.push_back(-0.384992);
    desired_states.name.push_back("RHipYawPitch");
    desired_states.position.push_back(-0.0214341);
    desired_states.name.push_back("RHipRoll");
    desired_states.position.push_back(0.366668);
    desired_states.name.push_back("RHipPitch");
    desired_states.position.push_back(0.161028);
    desired_states.name.push_back("RKneePitch");
    desired_states.position.push_back(-0.0923279);
    desired_states.name.push_back("RAnklePitch");
    desired_states.position.push_back(-0.026036);
    desired_states.name.push_back("RAnkleRoll");
    desired_states.position.push_back(-0.36505);

    moveRobot(0.02);
    sleep(2);
    bal_RLeg = 1;
    bal_2Leg = 0;
    //cout << "prepped" << endl;

    // finbalance
    desired_states.name.push_back("LHipYawPitch");
    desired_states.position.push_back(0.098218);
    desired_states.name.push_back("LHipRoll");
    desired_states.position.push_back(0.590632);
    desired_states.name.push_back("LHipPitch");
    desired_states.position.push_back(-0.358914);
    desired_states.name.push_back("LKneePitch");
    desired_states.position.push_back(1.65514);
    desired_states.name.push_back("LAnklePitch");
    desired_states.position.push_back(-1.18944);
    desired_states.name.push_back("LAnkleRoll");
    desired_states.position.push_back(-1.18944);
    desired_states.name.push_back("RHipYawPitch");
    desired_states.position.push_back(0.098218);//0.0276539);
    desired_states.name.push_back("RHipRoll");
    desired_states.position.push_back(0.368202);//0.374338);
    desired_states.name.push_back("RHipPitch");
    desired_states.position.push_back(0.0889301);//0.159494);
    desired_states.name.push_back("RKneePitch");
    desired_states.position.push_back(-0.0923279);//-0.0923279);
    desired_states.name.push_back("RAnklePitch");
    desired_states.position.push_back(-0.00916195);//0.0061779);
    desired_states.name.push_back("RAnkleRoll");
    desired_states.position.push_back(-0.381924);//-0.368118);


    moveRobot(0.02);
    sleep(2);
    //cout << "balancing" << endl;
  }
  void stand() {
    desired_states.name.clear();
    desired_states.position.clear();

    // prepbalance
    desired_states.name.push_back("LHipYawPitch");
    desired_states.position.push_back(-0.0214341);
    desired_states.name.push_back("LHipRoll");
    desired_states.position.push_back(0.412688);
    desired_states.name.push_back("LHipPitch");
    desired_states.position.push_back(0.135034);
    desired_states.name.push_back("LKneePitch");
    desired_states.position.push_back(-0.0923279);
    desired_states.name.push_back("LAnklePitch");
    desired_states.position.push_back(0.0475121);
    desired_states.name.push_back("LAnkleRoll");
    desired_states.position.push_back(-0.384992);
    desired_states.name.push_back("RHipYawPitch");
    desired_states.position.push_back(-0.0214341);
    desired_states.name.push_back("RHipRoll");
    desired_states.position.push_back(0.366668);
    desired_states.name.push_back("RHipPitch");
    desired_states.position.push_back(0.161028);
    desired_states.name.push_back("RKneePitch");
    desired_states.position.push_back(-0.0923279);
    desired_states.name.push_back("RAnklePitch");
    desired_states.position.push_back(-0.026036);
    desired_states.name.push_back("RAnkleRoll");
    desired_states.position.push_back(-0.36505);

    moveRobot(0.02);
    sleep(2);
    bal_RLeg = 0;
    bal_2Leg = 1;
    //cout << "prepped" << endl;

    // stand
    desired_states.name.push_back("LHipYawPitch");
    desired_states.position.push_back(-0.0199001);
    desired_states.name.push_back("LHipRoll");
    desired_states.position.push_back(-0.0383081);
    desired_states.name.push_back("LHipPitch");
    desired_states.position.push_back(0.257754);
    desired_states.name.push_back("LKneePitch");
    desired_states.position.push_back(-0.0923279);
    desired_states.name.push_back("LAnklePitch");
    desired_states.position.push_back(-0.0138481);
    desired_states.name.push_back("LAnkleRoll");
    desired_states.position.push_back(0.0307219);
    desired_states.name.push_back("RHipYawPitch");
    desired_states.position.push_back(-0.0199001);
    desired_states.name.push_back("RHipRoll");
    desired_states.position.push_back(-0.0106959);
    desired_states.name.push_back("RHipPitch");
    desired_states.position.push_back(0.25);
    desired_states.name.push_back("RKneePitch");
    desired_states.position.push_back(-0.0923279);
    desired_states.name.push_back("RAnklePitch");
    desired_states.position.push_back(-0.0383081);
    desired_states.name.push_back("RAnkleRoll");
    desired_states.position.push_back(0.00924587);

    moveRobot(0.02);
    sleep(2);
    //cout << "standing" << endl;
  }

  // BALANCING
      bool CoM_chk(){
        while (current_state.position.size() == 0){}

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

          std::vector<float> jointanglesdes(chk_state.position.begin(), chk_state.position.end());
          kin.setJoints(jointanglesdes);

          if (bal_RLeg){
            //cout << "1leg stance balance: "<< chk_CoM_RLeg()[0] << endl;
            if (chk_CoM_RLeg()[0]) return true;
            else return false;
          }
          else if (bal_2Leg){
            //cout << "2leg stance balance: " << chk_CoM_2legs() << endl;
            if(chk_CoM_2legs()) return true;
            else return false;
          }
          else return true;
      }

      //RLeg CoM chk
      std::vector<bool> chk_CoM_RLeg(){
        std::vector<bool> commatch(5, true);
        Eigen::Vector4d sumcom;
        sumcom =  Tf_torso_rfoot();
        //cout << "CoM   x: " << sumcom(0) << endl << " y: " << sumcom(1);// << " z: " << sumcom(2) << endl;

        // check limits
        if(-30.0 > sumcom(0))
                    {
                      cout << "CoM is not over right foot in negative x-direction: " << sumcom(0) << endl;
                      commatch[0] = false;
                      commatch[1] = false;
                    }
        if (sumcom(0) > 20)
                    {
                      cout << "CoM is not over right foot in positive x-direction: " << sumcom(0) << endl;
                      commatch[0] = false;
                      commatch[2] = false;
                    }
        if(-30.0 > sumcom(1))
                    {
                      cout << "CoM is not over right foot in negative y-direction: " << sumcom(1) << endl;
                      commatch[0] = false;
                      commatch[3] = false;
                    }
        if (sumcom(1) > 20.0)
                    {
                      cout << "CoM is not over right foot in positive y-direction: " << sumcom(1) << endl;
                      commatch[0] = false;
                      commatch[4] = false;
                    }
        if (sumcom(1) < 23.0 && -30.0 < sumcom(1) && sumcom(0) < 40 && -30.0 < sumcom(0))
                    {
                        commatch[0] = true;
                        commatch[1] = true;
                        commatch[2] = true;
                        commatch[3] = true;
                        commatch[4] = true;
                    }
                    return commatch;
      }
      //Transformation torso right foot
      Eigen::Vector4d Tf_torso_rfoot(){
        // get transformation from right foot to torso
        NAOKinematics::kmatTable footcoord = kin.getForwardEffector((NAOKinematics::Effectors)CHAIN_R_LEG);
        Eigen::Matrix4d T_torso_rfoot;
        T_torso_rfoot(0,0) =  footcoord.getRotation()(0,0);
        T_torso_rfoot(0,1) =  footcoord.getRotation()(0,1);
        T_torso_rfoot(0,2) =  footcoord.getRotation()(0,2);
        T_torso_rfoot(0,3) =  footcoord.getTranslation()(0);
        T_torso_rfoot(1,0) =  footcoord.getRotation()(1,0);
        T_torso_rfoot(1,1) =  footcoord.getRotation()(1,1);
        T_torso_rfoot(1,2) =  footcoord.getRotation()(1,2);
        T_torso_rfoot(1,3) =  footcoord.getTranslation()(1);
        T_torso_rfoot(2,0) =  footcoord.getRotation()(2,0);
        T_torso_rfoot(2,1) =  footcoord.getRotation()(2,1);
        T_torso_rfoot(2,2) =  footcoord.getRotation()(2,2);
        T_torso_rfoot(2,3) =  footcoord.getTranslation()(2);
        T_torso_rfoot(3,0) =  0;
        T_torso_rfoot(3,1) =  0;
        T_torso_rfoot(3,2) =  0;
        T_torso_rfoot(3,3) =  1;
        // gget CoM in torso coordinates
        KVecDouble3 Sum_CoM_Torso = kin.calculateCenterOfMass();
        Eigen::Vector4d Sum_CoM_Torso_v;
        Sum_CoM_Torso_v(0) = Sum_CoM_Torso(0,0);
        Sum_CoM_Torso_v(1) = Sum_CoM_Torso(1,0);
        Sum_CoM_Torso_v(2) = Sum_CoM_Torso(2,0);
        Sum_CoM_Torso_v(3) = 1;

        //calculate CoM in foot coordinates
        Eigen::Vector4d Sum_CoM_RFoot = T_torso_rfoot.inverse() * Sum_CoM_Torso_v;
        //cout << "Sum_CoM_RFoot x: "<< Sum_CoM_RFoot(0) << " y: " << Sum_CoM_RFoot(1)<< endl;// << " z: " << Sum_CoM_RFoot(2) << endl;
        return Sum_CoM_RFoot;
      }

      // 2 Legs
      bool chk_CoM_2legs(){
        Eigen::Matrix4d sumcom;
        bool commatch_2l;
        sumcom =  Tf_torso_2feet();

        // distance between feet in y and x direction
        double x_dist = sumcom(0,3);
        double y_dist = sumcom(1,3);

        //calculate connection line between left and right foo
        double x_betw_feet_at_yCoM = x_dist/y_dist*sumcom(1,0);

        //check the limits
        if(x_betw_feet_at_yCoM-30.0 > sumcom(0,0))
                    {
                      cout << "CoM is not over feet in negative x-direction"  << endl;
                      commatch_2l = false;
                    }
        else if (sumcom(0,0) > x_betw_feet_at_yCoM+30)
                    {
                      cout << "CoM is not over feet in positive x-direction" << endl;
                      commatch_2l = false;
                    }
        else if(-30.0 > sumcom(1,0))
                    {
                      cout << "CoM is not over feet in negative y-direction" << endl;
                      commatch_2l = false;
                    }
        else if (sumcom(1,1) > 20.0)
                    {
                      cout << "CoM is not over feet in positive y-direction" << endl;
                      commatch_2l = false;
                    }
        else
                    {
                        commatch_2l = true;
                    }
                    return commatch_2l;
      }
      // Transformation for 2leg balance
      Eigen::Matrix4d Tf_torso_2feet(){
        // TF Rleg-Torso
        NAOKinematics::kmatTable footcoord_R = kin.getForwardEffector((NAOKinematics::Effectors)CHAIN_R_LEG);
        Eigen::Matrix4d T_torso_rfoot;
        T_torso_rfoot(0,0) =  footcoord_R.getRotation()(0,0);
        T_torso_rfoot(0,1) =  footcoord_R.getRotation()(0,1);
        T_torso_rfoot(0,2) =  footcoord_R.getRotation()(0,2);
        T_torso_rfoot(0,3) =  footcoord_R.getTranslation()(0);
        T_torso_rfoot(1,0) =  footcoord_R.getRotation()(1,0);
        T_torso_rfoot(1,1) =  footcoord_R.getRotation()(1,1);
        T_torso_rfoot(1,2) =  footcoord_R.getRotation()(1,2);
        T_torso_rfoot(1,3) =  footcoord_R.getTranslation()(1);
        T_torso_rfoot(2,0) =  footcoord_R.getRotation()(2,0);
        T_torso_rfoot(2,1) =  footcoord_R.getRotation()(2,1);
        T_torso_rfoot(2,2) =  footcoord_R.getRotation()(2,2);
        T_torso_rfoot(2,3) =  footcoord_R.getTranslation()(2);
        T_torso_rfoot(3,0) =  0;
        T_torso_rfoot(3,1) =  0;
        T_torso_rfoot(3,2) =  0;
        T_torso_rfoot(3,3) =  1;

        // TF Lleg-Torso
        NAOKinematics::kmatTable footcoord_L = kin.getForwardEffector((NAOKinematics::Effectors)CHAIN_L_LEG);
        Eigen::Matrix4d T_torso_lfoot;
        T_torso_lfoot(0,0) =  footcoord_L.getRotation()(0,0);
        T_torso_lfoot(0,1) =  footcoord_L.getRotation()(0,1);
        T_torso_lfoot(0,2) =  footcoord_L.getRotation()(0,2);
        T_torso_lfoot(0,3) =  footcoord_L.getTranslation()(0);
        T_torso_lfoot(1,0) =  footcoord_L.getRotation()(1,0);
        T_torso_lfoot(1,1) =  footcoord_L.getRotation()(1,1);
        T_torso_lfoot(1,2) =  footcoord_L.getRotation()(1,2);
        T_torso_lfoot(1,3) =  footcoord_L.getTranslation()(1);
        T_torso_lfoot(2,0) =  footcoord_L.getRotation()(2,0);
        T_torso_lfoot(2,1) =  footcoord_L.getRotation()(2,1);
        T_torso_lfoot(2,2) =  footcoord_L.getRotation()(2,2);
        T_torso_lfoot(2,3) =  footcoord_L.getTranslation()(2);
        T_torso_lfoot(3,0) =  0;
        T_torso_lfoot(3,1) =  0;
        T_torso_lfoot(3,2) =  0;
        T_torso_lfoot(3,3) =  1;

        //CoM in torso frame
        KVecDouble3 Sum_CoM_Torso = kin.calculateCenterOfMass();
        Eigen::Vector4d Sum_CoM_Torso_v;
        Sum_CoM_Torso_v(0) = Sum_CoM_Torso(0,0);
        Sum_CoM_Torso_v(1) = Sum_CoM_Torso(1,0);
        Sum_CoM_Torso_v(2) = Sum_CoM_Torso(2,0);
        Sum_CoM_Torso_v(3) = 1;

        //calculate CoM in R and L foot frame
        Eigen::Vector4d Sum_CoM_RFoot = T_torso_rfoot.inverse() * Sum_CoM_Torso_v;
        Eigen::Vector4d Sum_CoM_LFoot = T_torso_lfoot.inverse() * Sum_CoM_Torso_v;

        //calculate position of left foot in right foot coordinates
        Eigen::Vector4d O_LFoot;
        O_LFoot << 0, 0, 0, 1;
        Eigen::Matrix4d Tf_lf_rf = T_torso_lfoot * T_torso_rfoot.inverse();
        Eigen::Vector4d O_LFoot_in_RFoot = Tf_lf_rf * O_LFoot ;
        Eigen::Vector4d O_RFoot;
        O_RFoot << 0, 0, 0, 1;

        Eigen::Matrix4d Sum_CoM_2Foot;
        Sum_CoM_2Foot << Sum_CoM_RFoot, Sum_CoM_LFoot, O_RFoot, O_LFoot_in_RFoot;
        //return CoM in left and right foot frame and coordinates of feet in right foot frame
        return Sum_CoM_2Foot;
      }

  // MAPPING
  void imitate_left(const vector<Vector3d> &target, dlib_vector &solution);
  void imitate_right(const vector<Vector3d> &target, dlib_vector &solution);

  // MOVEMENT
  void moveRobot(double speed);

  // Perform recorded sequence
  void do_sequence(const target_sequence &left_seq,
                   const target_sequence &right_seq);
};

#endif
