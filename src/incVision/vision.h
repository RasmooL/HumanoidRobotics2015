#ifndef VISION_H
#define VISION_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "constants.h"
#include "arm.h"
#include "settings.h"


using namespace cv;
using namespace std;
using namespace Eigen;


// aruco_vision.cpp
void getJointPositions(Mat imgOrg, Arm *arm_left, Arm *arm_right);

#endif /* VISION_H */
