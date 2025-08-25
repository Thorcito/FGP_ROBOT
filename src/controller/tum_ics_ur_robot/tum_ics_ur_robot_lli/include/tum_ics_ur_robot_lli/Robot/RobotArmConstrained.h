#ifndef ROBOTARMCONSTRAINED_H
#define ROBOTARMCONSTRAINED_H

#include <tum_ics_ur_robot_lli/RobotControllers/Controller.h>

namespace tum_ics_ur_robot_lli{
namespace Robot{

class RobotArmConstrained
{
private:
    void* m_id;

public:
    RobotArmConstrained(const QString &configFilePath);
    ~RobotArmConstrained();

    bool add(RobotControllers::Controller *u);
    bool init();
    void start();
    bool stop();


    JointState qHome() const;
    JointState qPark() const;



};

}


}

#endif // ROBOTARMCONSTRAINED_H
