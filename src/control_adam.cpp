// Adam
#include "control.hpp"

void Nao_control::visionCB(const sensor_msgs::Image::ConstPtr &Img)
{
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

    // get all informations from the picture
    getJointPositions(cv_ptr->image, &arm_left, &arm_right, &chest);

    // move left arm when all markers for the left arm are found
    if(arm_left.getArmFound())
    {
        vector<Vector3d> left_target;
        left_target.push_back(arm_left.getJ1Coord());
        left_target.push_back(arm_left.getJ2Coord());
        left_target.push_back(arm_left.getJ3Coord());
        imitate_left(left_target, leftsol);
    }

    // move right arm when all markers for the right arm are found
    if(arm_right.getArmFound())
    {
        vector<Vector3d> right_target;
        right_target.push_back(arm_right.getJ1Coord());
        right_target.push_back(arm_right.getJ2Coord());
        right_target.push_back(arm_right.getJ3Coord());
        imitate_right(right_target, rightsol);
    }

    // move torso, when initial position has been detected and the user leans
    /*if(chest.getTorsoMoved())
    {
        // correct every angle with a unique adaptation value
        desired_states.name.push_back("LHipYawPitch");
        desired_states.position.push_back(chest.getAngle()-0.138018);
        desired_states.name.push_back("RHipYawPitch");
        desired_states.position.push_back(chest.getAngle()-0.138018);
        desired_states.name.push_back("HeadPitch");
        desired_states.position.push_back(chest.getAngle()-0.147306);
    }*/
    moveRobot(0.1);

    waitKey(5);
}
