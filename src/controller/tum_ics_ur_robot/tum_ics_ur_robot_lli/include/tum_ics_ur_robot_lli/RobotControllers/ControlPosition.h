#ifndef CONTROLPOSITION_H
#define CONTROLPOSITION_H

#include<tum_ics_ur_robot_lli/RobotControllers/Controller.h>



namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

class ControlPosition: public Controller
{
    // Member variables
private:



    // Member Methods
public:
    ControlPosition(const QString& name, ControlType type, ControlSpace space);
    virtual ~ControlPosition();



    // Memeber Methods
private:


};

}
}

#endif // CONTROLPOSITION_H
