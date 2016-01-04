// Group C: Rasmus Larsen

#include <eigen3/Eigen/Eigen>
#include <vector>
#include "dlib/optimization.h"
using namespace Eigen;
using namespace std;

const long double pi = 3.141592653589793238462643383279502884L;
typedef dlib::matrix<double, 0, 1> dlib_vector;

// Get homogenous transformation using DH parameters
Matrix4d dh_to_homog(double a, double alpha, double d, double theta)
{
  Matrix4d T;
  T << cos(theta), -sin(theta), 0, a,
       sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha),
       sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha),
       0, 0, 0, 1;
      return T;
}

// Returns the task space coordinates of the normalized kinematic chain using the generalized coordinate s (0.0 - 1.0)
Vector3d left_arm_normalized(double s, double q1, double q2, double q3, double q4, double q5)
{
  double UpperArmLength = 0.105;
  double LowerArmLength = 0.056;
  double HandOffsetX = 0.058;
  double L = UpperArmLength + LowerArmLength + HandOffsetX;
  UpperArmLength = UpperArmLength / L; // 0.479452
  LowerArmLength = LowerArmLength / L; // + = 0.73516
  HandOffsetX = HandOffsetX / L;


  Matrix4d Base, ShoulderPitch, ShoulderRoll, ElbowYaw, ElbowRoll, WristRoll;
  Vector3d pos;

  Base = Matrix4d::Identity();
  ShoulderPitch = dh_to_homog(0, -pi/2, 0, q1);
  ShoulderRoll = dh_to_homog(0, pi/2, 0, q2 - pi/2);
  ElbowYaw = dh_to_homog(0, -pi/2, UpperArmLength, q3);
  ElbowRoll = dh_to_homog(0, pi/2, 0, q4);
  WristRoll = dh_to_homog(0, -pi/2, LowerArmLength + HandOffsetX, q5); // Combined wrist into lower arm

  if(s < UpperArmLength)
  {
    s = s / UpperArmLength;
    Matrix4d A = ShoulderPitch * ShoulderRoll * ElbowYaw;
    Vector3d elbow_pos = A.block<3,1>(0, 3);
    pos = elbow_pos * s;
  }
  else // Wrist is 'combined' into lower arm, since it only has a roll dof
  {
    s = (s - UpperArmLength) / (LowerArmLength + HandOffsetX);
    Matrix4d A = ShoulderPitch * ShoulderRoll * ElbowYaw;
    Vector3d elbow_pos = A.block<3,1>(0,3);
    A = A * ElbowRoll * WristRoll;
    Vector3d hand_pos = A.block<3,1>(0,3);
    pos = hand_pos * s + (1-s) * elbow_pos;
  }

  return pos;
}

// Sum of vector<double> elements from start to end
double sum_vec(const std::vector<double>& vec, const int start, const int end)
{
  double res = 0;
  for(int i = start; i <= end; i++)
  {
    res += vec[i];
  }
  return res;
}

// Sum of vector<Vector3d> elements from start to end
Vector3d sum_vec(const std::vector<Vector3d>& vec, const int start, const int end)
{
  Vector3d res; res << 0,0,0;
  for(int i = start; i <= end; i++)
  {
    res += vec[i];
  }
  return res;
}

// Take a sequence of world-base 3d vectors and interpolate as a chain
Vector3d linterp(double s, const vector<Vector3d>& q)
{
  int n = q.size();

  // Construct normalized, relative chain
  vector<double> len;
  vector<Vector3d> chain;
  for(int i = 0; i < n-1; i++)
  {
    chain.push_back(q[i+1] - q[i]);
    len.push_back(chain[i].norm());
  }
  // chain = chain / sum(len)
  double sum_len = sum_vec(len, 0, len.size()-1);
  for(vector<Vector3d>::iterator it = chain.begin(); it != chain.end(); it++) *it = *it / sum_len;
  // len = len / sum(len)
  for(vector<double>::iterator it = len.begin(); it != len.end(); it++) *it = *it / sum_len;

  // Where in the chain are we (parameter s = 0 to 1)
  for(int i = 0; i < n-1; i++)
  {
    if(i == 0)
    {
      if(s <= len[0]) // Between start and first point
      {
        return s * chain[0];
      }
    }
    else
    {
      if(s > sum_vec(len, 0, i-1) && s <= sum_vec(len, 0, i))
      {
        s = (s - sum_vec(len, 0, i-1)) / len[i];
        return (1-s) * sum_vec(chain, 0, i-1) + s * sum_vec(chain, 0, i);
      }
    }
  }

  return Vector3d(0,0,0);

}

// This object is a function object that can hold information used in calculating the cost with operator()
class objective_function
{
  Vector3d (*chain)(double, double, double, double, double, double);
  vector<Vector3d> target;
  const int npts;
public:
  objective_function(Vector3d (*chain)(double,double,double,double,double,double), const vector<Vector3d>& target, const int npts) : chain(chain), target(target), npts(npts) {}

  double operator()(const dlib_vector& input) const
  {
    double cost = 0;
    for(int i = 1; i <= npts; i++)
    {
      cost += (chain((double)i/npts, input(0), input(1), input(2), input(3), input(4)) - linterp((double)i/npts, target)).squaredNorm();
    }

    return cost;
  }
};

