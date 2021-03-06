// Rasmus

#include "optim.hpp"

// Get homogenous transformation using DH parameters
Matrix4d dh_to_homog(double a, double alpha, double d, double theta) {
  Matrix4d T;
  T << cos(theta), -sin(theta), 0, a, sin(theta) * cos(alpha),
      cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha),
      sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha),
      d * cos(alpha), 0, 0, 0, 1;
  return T;
}

// Returns the task space coordinates of the normalized kinematic chain using
// the generalized coordinate s (0.0 - 1.0)
Vector3d left_arm_normalized(double s, const vector<double> &q) {
  if (q.size() != 5)
    return Vector3d();

  double UpperArmLength = 0.105;
  double LowerArmLength = 0.056;
  double HandOffsetX = 0.058;
  double L = UpperArmLength + LowerArmLength + HandOffsetX;
  UpperArmLength = UpperArmLength / L;
  LowerArmLength = LowerArmLength / L;
  HandOffsetX = HandOffsetX / L;

  Matrix4d Base, ShoulderPitch, ShoulderRoll, ElbowYaw, ElbowRoll, WristRoll;
  Vector3d pos;

  Base = Matrix4d::Identity();
  ShoulderPitch = dh_to_homog(0, -pi / 2, 0, q[0]);
  ShoulderRoll = dh_to_homog(0, pi / 2, 0, q[1] - pi / 2);
  ElbowYaw = dh_to_homog(0, -pi / 2, UpperArmLength, q[2]);
  ElbowRoll = dh_to_homog(0, pi / 2, 0, q[3]);
  WristRoll = dh_to_homog(0, -pi / 2, LowerArmLength + HandOffsetX,
                          q[4]); // Combined wrist into lower arm

  if (s < UpperArmLength) {
    s = s / UpperArmLength;
    Matrix4d A = ShoulderPitch * ShoulderRoll * ElbowYaw;
    Vector3d elbow_pos = A.block<3, 1>(0, 3);
    pos = elbow_pos * s;
  } else // Wrist is 'combined' into lower arm, since it only has a roll dof
  {
    s = (s - UpperArmLength) / (LowerArmLength + HandOffsetX);
    Matrix4d A = ShoulderPitch * ShoulderRoll * ElbowYaw;
    Vector3d elbow_pos = A.block<3, 1>(0, 3);
    A = A * ElbowRoll * WristRoll;
    Vector3d hand_pos = A.block<3, 1>(0, 3);
    pos = hand_pos * s + (1 - s) * elbow_pos;
  }

  return pos;
}

// Returns the task space coordinates of the normalized kinematic chain using
// the generalized coordinate s (0.0 - 1.0)
Vector3d right_arm_normalized(double s, const vector<double> &q) {
  if (q.size() != 5)
    return Vector3d();

  double UpperArmLength = 0.105;
  double LowerArmLength = 0.056;
  double HandOffsetX = 0.058;
  double L = UpperArmLength + LowerArmLength + HandOffsetX;
  UpperArmLength = UpperArmLength / L;
  LowerArmLength = LowerArmLength / L;
  HandOffsetX = HandOffsetX / L;

  Matrix4d Base, ShoulderPitch, ShoulderRoll, ElbowYaw, ElbowRoll, WristRoll;
  Vector3d pos;

  Base = Matrix4d::Identity();
  ShoulderPitch = dh_to_homog(0, -pi / 2, 0, q[0]);
  ShoulderRoll = dh_to_homog(0, pi / 2, 0, q[1] + pi / 2);
  ElbowYaw = dh_to_homog(0, -pi / 2, -UpperArmLength, -q[2]);
  ElbowRoll = dh_to_homog(0, pi / 2, 0, q[3]);
  WristRoll = dh_to_homog(0, -pi / 2, -LowerArmLength - HandOffsetX,
                          q[4]); // Combined wrist into lower arm

  if (s < UpperArmLength) {
    s = s / UpperArmLength;
    Matrix4d A = ShoulderPitch * ShoulderRoll * ElbowYaw;
    Vector3d elbow_pos = A.block<3, 1>(0, 3);
    pos = elbow_pos * s;
  } else // Wrist is 'combined' into lower arm, since it only has a roll dof
  {
    s = (s - UpperArmLength) / (LowerArmLength + HandOffsetX);
    Matrix4d A = ShoulderPitch * ShoulderRoll * ElbowYaw;
    Vector3d elbow_pos = A.block<3, 1>(0, 3);
    A = A * ElbowRoll * WristRoll;
    Vector3d hand_pos = A.block<3, 1>(0, 3);
    pos = hand_pos * s + (1 - s) * elbow_pos;
  }

  return pos;
}

// Sum of vector<double> elements from start to end
double sum_vec(const std::vector<double> &vec, const int start, const int end) {
  double res = 0;
  for (int i = start; i <= end; i++) {
    res += vec[i];
  }
  return res;
}

// Sum of vector<Vector3d> elements from start to end, can't use templates since
// Vector3d constructor sucks :(
Vector3d sum_vec(const std::vector<Vector3d> &vec, const int start,
                 const int end) {
  Vector3d res;
  res << 0, 0, 0;
  for (int i = start; i <= end; i++) {
    res += vec[i];
  }
  return res;
}

// Take a sequence of world-base 3d vectors and interpolate as a chain
Vector3d linterp(double s, const vector<Vector3d> &q) {
  int n = q.size();

  // Construct normalized, relative chain
  vector<double> len;
  vector<Vector3d> chain;
  for (int i = 0; i < n - 1; i++) {
    chain.push_back(q[i + 1] - q[i]);
    len.push_back(chain[i].norm());
  }
  // chain = chain / sum(len)
  double sum_len = sum_vec(len, 0, len.size() - 1);
  for (vector<Vector3d>::iterator it = chain.begin(); it != chain.end(); it++)
    *it = *it / sum_len;
  // len = len / sum(len)
  for (vector<double>::iterator it = len.begin(); it != len.end(); it++)
    *it = *it / sum_len;

  // Where in the chain are we (parameter s = 0 to 1)
  for (int i = 0; i < n - 1; i++) {
    if (i == 0) {
      if (s <= len[0]) // Between start and first point
      {
        return s * chain[0];
      }
    } else {
      if (s > sum_vec(len, 0, i - 1) && s <= sum_vec(len, 0, i)) {
        s = (s - sum_vec(len, 0, i - 1)) / len[i];
        return (1 - s) * sum_vec(chain, 0, i - 1) + s * sum_vec(chain, 0, i);
      }
    }
  }

  return Vector3d(0, 0, 0);
}
