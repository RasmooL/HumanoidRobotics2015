// Zylka, Adam

#ifndef CHEST_H
#define CHEST_H

class Chest
{
public:
    Chest()                             { m_init = false;               }

    void setInit()                      { m_init = true;                }
    bool getInit()                      { return m_init;                }

    void resetTorsoMoved()              { m_torsoMoved = false;         }
    void setTorsoMoved()                { m_torsoMoved = true;          }
    bool getTorsoMoved()                { return m_torsoMoved;          }

    void setInitPos(double pos)         { m_initPos = pos;              }
    double getInitPos()                 { return m_initPos;             }

    void setAngle(double angle)         { m_angle = angle;              }
    double getAngle()                   { return m_angle;               }

private:
    bool m_init;
    bool m_torsoMoved;

    double m_initPos;
    double m_angle;
};

#endif
