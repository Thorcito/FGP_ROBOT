#include<tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>


namespace tum_ics_ur_robot_lli{
namespace RobotControllers{


ControlEffort::ControlEffort(const QString name, ControlType type, ControlSpace space, double weight):
    Controller(name,EFFORT_INTF,type,space,weight),
    m_tauExt(VectorDOFd::Zero())
{
    //    std::cout<<"ControlEffort"<<std::endl;
}

ControlEffort::~ControlEffort()
{

}

VectorDOFd ControlEffort::tauExt() const
{
    return m_tauExt;
}

void ControlEffort::setTauExt(const VectorDOFd &tExt)
{
    m_tauExt=tExt;
}

}
}
