#ifndef CONTROLVELOCITY_H
#define CONTROLVELOCITY_H

#include<tum_ics_ur_robot_lli/RobotControllers/Controller.h>



namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

class ControlVelocity: public Controller
{
    // Member variables
private:



    // Member Methods
public:
    ControlVelocity(const QString& name, ControlType type, ControlSpace space);
    virtual ~ControlVelocity();



    // Memeber Methods
private:



};

}
}

#endif // CONTROLVELOCITY_H
