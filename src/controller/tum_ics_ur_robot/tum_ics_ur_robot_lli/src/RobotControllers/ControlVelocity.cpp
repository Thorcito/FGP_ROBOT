#include<tum_ics_ur_robot_lli/RobotControllers/ControlVelocity.h>


namespace tum_ics_ur_robot_lli{
namespace RobotControllers{


ControlVelocity::ControlVelocity(const QString& name, ControlType type, ControlSpace space) :
    Controller(name,VELOCITY_INTF,type,space)
{

}

ControlVelocity::~ControlVelocity()
{

}


}
}
