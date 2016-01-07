#ifndef VISION_H
#define VISION_H

#include "calculatePosition.h"
#include "constants.h"
#include "arm.h"
#include "settings.h"


using namespace cv;
using namespace std;
using namespace Eigen;



int stateInitColor(Mat imgOrg, Arm arm);

int stateInitArm(Mat imgOrg, Arm *arm);

int runArmTracking(Mat imgOrg, Arm *arm_left, Arm *arm_right);

#endif /* VISION_H */
