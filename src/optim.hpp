// Group C: Rasmus Larsen

#ifndef _OPTIM_HPP_
#define _OPTIM_HPP_

#include <eigen3/Eigen/Eigen>
#include <vector>
#include "dlib/optimization.h"
using namespace Eigen;
using namespace std;

const long double pi = 3.141592653589793238462643383279502884L;
typedef dlib::matrix<double, 0, 1> dlib_vector;

// Get homogenous transformation using DH parameters
Matrix4d dh_to_homog(double a, double alpha, double d, double theta);

// Returns the task space coordinates of the normalized kinematic chain using
// the generalized coordinate s (0.0 - 1.0)
Vector3d left_arm_normalized(double s, const vector<double> &q);

// Returns the task space coordinates of the normalized kinematic chain using
// the generalized coordinate s (0.0 - 1.0)
Vector3d right_arm_normalized(double s, const vector<double> &q);

// Sum of vector<double> elements from start to end
double sum_vec(const std::vector<double> &vec, const int start, const int end);

// Sum of vector<Vector3d> elements from start to end, can't use templates since
// Vector3d constructor sucks :(
Vector3d sum_vec(const std::vector<Vector3d> &vec, const int start,
                 const int end);

// Take a sequence of world-base 3d vectors and interpolate as a chain
Vector3d linterp(double s, const vector<Vector3d> &q);

// This object is a function object that can hold information used in
// calculating the cost with operator()
class objective_function {
  Vector3d (*chain)(double, const vector<double> &);
  vector<Vector3d> target;
  const int npts;

public:
  objective_function(Vector3d (*chain)(double, const vector<double> &),
                     const vector<Vector3d> &target, const int npts)
      : chain(chain), target(target), npts(npts) {}

  vector<double> dlib_to_std(const dlib_vector &vec) const {
    vector<double> res;
    for (dlib_vector::const_iterator it = vec.begin(); it != vec.end(); it++)
      res.push_back(*it);
    return res;
  }

  double operator()(const dlib_vector &input) const {
    double cost = 0;
    vector<double> stdinput = dlib_to_std(input);
    for (int i = 1; i <= npts; i++) {
      cost += (chain((double)i / npts, stdinput) -
               linterp((double)i / npts, target))
                  .squaredNorm();
    }

    return cost;
  }
};

#endif
