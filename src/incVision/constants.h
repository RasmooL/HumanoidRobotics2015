#ifndef COLOR_SETTINGS_H
#define COLOR_SETTINGS_H

// return values
#define SUCCESS     0
#define FAILURE     -5


// final state automat
enum
{
    INIT_COLOR_LEFT,
    INIT_COLOR_RIGHT,
	DESTROY_WINDOW_INIT_COLOR_LEFT,
	DESTROY_WINDOW_INIT_COLOR_RIGHT,
    INIT_ARM_LEFT,
    INIT_ARM_RIGHT,
    RUN,
    ERR
};

// arm lengths
const int BONE1HUMAN            = 300;
const int BONE2HUMAN            = 330;
const int BONE1ROBOT            = 105;
const int BONE2ROBOT            = 114;



using namespace cv;

// threshold
const int BIN_THRESHOLD         = 100;
const int MAX_THRESH_VAL        = 255;

// contour
// length
const int MIN_CONTOUR_LENGTH    = 20;
const int MAX_CONTOUR_LENGTH    = 100;
// color
const Scalar contourColor(0, 0, 255);

// rectangles
// the same objects
const int SAME_OBJECT_BORDER    = 15;
const int SAME_OBJECT_SIZE      = 15;

// model
// line color
const Scalar lineColor(255, 255, 255);
// joint color
const Scalar jointColor(0, 255, 255);
// radius of the joint
const int jointRadius           = 5;


// keyboard shortcuts
#define PRESS_ESC     27
#define PRESS_ENTER   13


#endif
