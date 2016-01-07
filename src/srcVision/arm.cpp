#include "../incVision/arm.h"

Arm::Arm(string name)
{
    m_name = name;

    if(m_name == "arm_left")
    {
        m_j1.setJName("arm_left_1_joint");
        m_j1.setJColor("red");
        m_j2.setJName("arm_left_2_joint");
        m_j2.setJColor("yellow");
        m_j3.setJName("arm_left_3_joint");
        m_j3.setJColor("blue");
    }
    else if(m_name == "arm_right")
    {
        m_j1.setJName("arm_right_1_joint");
        m_j1.setJColor("green");
        m_j2.setJName("arm_right_2_joint");
        m_j2.setJColor("dark_blue");
        m_j3.setJName("arm_right_3_joint");
        m_j3.setJColor("brown");
    }
    else
        cout << "wrong arm name\n";
}
