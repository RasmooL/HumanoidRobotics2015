#ifndef CALCULATEPOSITION_H
#define CALCULATEPOSITION_H

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;


// functions
Point getJCenter(Mat imgHSV, Mat imgOriginal, Mat withoutBG, String color);

#endif // CALCULATEPOSITION_H
