#ifndef ARM_H
#define ARM_H

#include "joint.h"

class Arm
{
public:
    Arm();
    Arm(string name);

    const string getArmName()       { return m_name;            }

    Joint getJ1()                   { return m_j1;              }
    Joint getJ2()                   { return m_j2;              }
    Joint getJ3()                   { return m_j3;              }

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


    void setJ1Found()               { m_j1Found = true;         }
    void setJ2Found()               { m_j2Found = true;         }
    void setJ3Found()               { m_j3Found = true;         }

    void resetJ1Found()             { m_j1Found = false;        }
    void resetJ2Found()             { m_j2Found = false;        }
    void resetJ3Found()             { m_j3Found = false;        }

    bool getJ1Found()               { return m_j1Found;         }
    bool getJ2Found()               { return m_j2Found;         }
    bool getJ3Found()               { return m_j3Found;         }

private:
    string m_name;

    Joint m_j1;
    Joint m_j2;
    Joint m_j3;

    bool m_j1Found;
    bool m_j2Found;
    bool m_j3Found;

    double m_bone1;
    double m_bone2;
};
#endif // ARM_H
