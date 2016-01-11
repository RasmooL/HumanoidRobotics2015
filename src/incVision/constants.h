#ifndef COLOR_SETTINGS_H
#define COLOR_SETTINGS_H

using namespace cv;

// return values
#define SUCCESS     0
#define FAILURE     -5

enum
{
  SHOULDER_LEFT   = 1,
  ELBOW_LEFT      = 2,
  WRIST_LEFT      = 3,
  SHOULDER_RIGHT  = 4,
  ELBOW_RIGHT     = 5,
  WRIST_RIGHT     = 6
};

// min dist
const double MIN_DIST  = 0.001;

// arm lengths
const int BONE1HUMAN            = 300;
const int BONE2HUMAN            = 330;
const int BONE1ROBOT            = 105;
const int BONE2ROBOT            = 114;


// camera size
const Size CAMERA_RESOLUTION      = Size(640, 480);


#endif
