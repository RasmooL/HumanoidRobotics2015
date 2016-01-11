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
int getJointPositions(Mat imgOrg, Arm * arm);

#endif /* VISION_H */
