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

    void setJCoord(Vector3d coord)      { m_Coord = coord;              }
    Vector3d getJCoord()                { return m_Coord;               }

private:
    string m_Name;
    Vector3d m_Coord;
};

#endif // JOINT_H
