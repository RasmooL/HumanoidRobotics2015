// Group C: Rasmus

#ifndef _LOAD_HPP_
#define _LOAD_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <eigen3/Eigen/Eigen>
using namespace std;
using namespace Eigen;

typedef vector<vector<Vector3d> > target_sequence;

void load_msr_skeleton(string fname, target_sequence &left_arm, target_sequence &right_arm); 

#endif
