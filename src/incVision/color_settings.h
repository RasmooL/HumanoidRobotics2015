#ifndef COLOR_SETTINGS_H
#define COLOR_SETTINGS_H

using namespace cv;

// bin img threshold
const int BIN_THRESHOLD         = 100;

// contour length
const int MIN_CONTOUR_LENGTH    = 60;
const int MAX_CONTOUR_LENGTH    = 500;
// contour color
const Scalar contourColor(0, 0, 255);

// the same objects
const int SAME_OBJECT_BORDER    = 15;
const int SAME_OBJECT_SIZE      = 15;

// red
const int iRHLow    = 0;
const int iRHHigh   = 179;
const int iRSLow    = 176;
const int iRSHigh   = 255;
const int iRVLow    = 120;
const int iRVHigh   = 200;

// blue


const Scalar sRedLow(iRHLow, iRSLow, iRVLow);
const Scalar sRedHigh(iRHHigh, iRSHigh, iRVHigh);

#endif // COLOR_SETTINGS_H
