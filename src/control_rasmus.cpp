// Rasmus
#include "control.hpp"

void Nao_control::imitate_left(const vector<Vector3d> &target, dlib_vector &solution)
{
  try {
    const int interp_pts = 30;
    const int bobyqa_pts = 10;
    const float start_trustregion = 0.09;
    const float end_trustregion = 0.02;
    const float max_iter = 500;
    dlib_vector lower_bound(5);
    lower_bound = -1.9, -0.3, -1.9, -1.5, -0.1; // wrist can be -1.7 to 1.7 max
    dlib_vector upper_bound(5);
    upper_bound = 1.9, 1.3, 1.9, -0.03, 0.1;
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

void Nao_control::imitate_right(const vector<Vector3d> &target, dlib_vector &solution)
{
  try {
    const int interp_pts = 30;
    const int bobyqa_pts = 10;
    const float start_trustregion = 0.09;
    const float end_trustregion = 0.02;
    const float max_iter = 500;
    dlib_vector lower_bound(5);
    lower_bound = -1.9, -1.3, -1.9, 0.04, 0.0;
    dlib_vector upper_bound(5);
    upper_bound = 1.9, 0.3, 1.9, 1.5, 0.2;
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
void Nao_control::moveRobot(double speed)
{
  // check if desired position is balanced
  if (CoM_chk())
  {
    //move robot by publishing to joint_angles
    naoqi_bridge_msgs::JointAnglesWithSpeed action;
    for (int i = 0; i < desired_states.name.size(); i++) {
      action.joint_names.push_back(desired_states.name[i]);
      action.joint_angles.push_back(
          (float)desired_states.position[i]);
    }
    action.header.stamp = ros::Time::now();
    action.speed = speed;

    joint_angles_pub.publish(action);
  }
  else
  {
    cout << "Goal joint states move CoM out of support polygon. No movement possible." << endl;
  }
  //clear desired states attribute
  desired_states.name.clear();
  desired_states.position.clear();
}

void Nao_control::do_sequence(const target_sequence &left_seq,
                 const target_sequence &right_seq)
{
  if (left_seq.size() != right_seq.size()) {
    cerr << "Sequences must be the same length!" << endl;
    return;
  }

  for (int i = 0; i < left_seq.size(); i++) {
    imitate_left(left_seq[i], leftsol);
    imitate_right(right_seq[i], rightsol);

    moveRobot(0.1);
  }
}
