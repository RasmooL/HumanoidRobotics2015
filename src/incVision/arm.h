#ifndef ARM_H
#define ARM_H

#include "joint.h"

class Arm
{
public:
    Arm(string name);

    const string getArmName()       { return m_name;            }

    Joint getJ1()                   { return m_j1;              }
    Joint getJ2()                   { return m_j2;              }
    Joint getJ3()                   { return m_j3;              }


    void setJ1Center(Point center)  { m_j1.setJCenter(center);  }
    void setJ2Center(Point center)  { m_j2.setJCenter(center);  }
    void setJ3Center(Point center)  { m_j3.setJCenter(center);  }

    Point getJ1Center()             { return m_j1.getJCenter(); }
    Point getJ2Center()             { return m_j2.getJCenter(); }
    Point getJ3Center()             { return m_j3.getJCenter(); }


    const string getJ1Color()       { return m_j1.getJColor();  }
    const string getJ2Color()       { return m_j2.getJColor();  }
    const string getJ3Color()       { return m_j3.getJColor();  }


    void setJ1Coord(Vector3d coord) { m_j1.setJCoord(coord);    }
    void setJ2Coord(Vector3d coord) { m_j2.setJCoord(coord);    }
    void setJ3Coord(Vector3d coord) { m_j3.setJCoord(coord);    }

    Vector3d getJ1Coord()           { return m_j1.getJCoord();  }
    Vector3d getJ2Coord()           { return m_j2.getJCoord();  }
    Vector3d getJ3Coord()           { return m_j3.getJCoord();  }


    void setBone1(double distance)  { m_bone1 = distance;       }
    void setBone2(double distance)  { m_bone2 = distance;       }

    int getBone1()                  { return m_bone1;           }
    int getBone2()                  { return m_bone2;           }


private:
    string m_name;

    Joint m_j1;
    Joint m_j2;
    Joint m_j3;

    double m_bone1;
    double m_bone2;
};
#endif // ARM_H
