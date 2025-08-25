#include<tum_ics_ur_robot_lli/RobotControllers/ControlPosition.h>


namespace tum_ics_ur_robot_lli{
namespace RobotControllers{


ControlPosition::ControlPosition(const QString& name, ControlType type, ControlSpace space) :
    Controller(name,POSITION_INTF,type,space)
{
//    std::cout<<"ControlPosition: "<<m_ctrlIntf<<std::endl;
}

ControlPosition::~ControlPosition()
{

}


}
}
