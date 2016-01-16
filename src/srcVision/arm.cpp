// Zylka, Adam

#include "../incVision/arm.h"

Arm::Arm(string name)
{
    m_name = name;
    m_wholeArmfound = false;

    if(m_name == "arm_left")
    {
        m_j1.setJName("SHOULDER_LEFT");
        m_j2.setJName("ELBOW_LEFT");
        m_j3.setJName("WRIST_LEFT");
    }
    else if(m_name == "arm_right")
    {
        m_j1.setJName("SHOULDER_RIGHT");
        m_j2.setJName("ELBOW_RIGHT");
        m_j3.setJName("arm_right_3_joint");
    }
    else
        cout << "wrong arm name\n";
}
