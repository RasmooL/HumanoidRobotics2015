#ifndef JOINT_H
#define JOINT_H

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "eigen3/Eigen/Eigen"

using namespace std;
using namespace cv;
using namespace Eigen;


class Joint
{
public:
    Joint();

    void setJName(string name)          { m_Name = name;                }
    const string getJName()             { return m_Name;                }

    void setJColor(string color)        { m_Color = color;              }
    const string getJColor()            { return m_Color;               }

    void setJCenter(Point center)       { m_CenterInPicture = center;   }
    Point getJCenter()                  { return m_CenterInPicture;     }

    void setJCoord(Vector3d coord)      { m_Coord = coord;              }
    Vector3d getJCoord()                { return m_Coord;               }

private:
    string m_Name;
    string m_Color;
    Point m_CenterInPicture;
    Vector3d m_Coord;
};


// global variables
// left_arm
extern int iRHLow;
extern int iRHHigh;
extern int iRSLow;
extern int iRSHigh;
extern int iRVLow;
extern int iRVHigh;
extern Scalar sRedLow;
extern Scalar sRedHigh;

extern int iYHLow;
extern int iYHHigh;
extern int iYSLow;
extern int iYSHigh;
extern int iYVLow;
extern int iYVHigh;
extern Scalar sYellowLow;
extern Scalar sYellowHigh;

extern int iBHLow;
extern int iBHHigh;
extern int iBSLow;
extern int iBSHigh;
extern int iBVLow;
extern int iBVHigh;
extern Scalar sBlueLow;
extern Scalar sBlueHigh;

// right_arm
extern int iGHLow;
extern int iGHHigh;
extern int iGSLow;
extern int iGSHigh;
extern int iGVLow;
extern int iGVHigh;
extern Scalar sGreenLow;
extern Scalar sGreenHigh;

extern int iDBHLow;
extern int iDBHHigh;
extern int iDBSLow;
extern int iDBSHigh;
extern int iDBVLow;
extern int iDBVHigh;
extern Scalar sDarkBlueLow;
extern Scalar sDarkBlueHigh;

extern int iBRHLow;
extern int iBRHHigh;
extern int iBRSLow;
extern int iBRSHigh;
extern int iBRVLow;
extern int iBRVHigh;
extern Scalar sBrownLow;
extern Scalar sBrownHigh;

#endif // JOINT_H
