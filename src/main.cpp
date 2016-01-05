// Group C

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
#include "control.hpp"
#include "load.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imitation");

    ros::NodeHandle n;
    ros::Rate rate_sleep(50);

    Nao_control control;

    target_sequence left_targets, right_targets;
    load_msr_skeleton("../../MSRDaily/a01_s01_e01_skeleton.txt", left_targets, right_targets);

    while(ros::ok()) {}

    return 0;
}
