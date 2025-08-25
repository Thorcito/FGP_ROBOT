#ifndef ROBOTARMCLOSED_H
#define ROBOTARMCLOSED_H
#include<tum_ics_ur_robot_lli/Robot/RobotArm.h>
#include <tum_ics_ur_robot_manager/RobotScriptManager.h>

namespace tum_ics_ur_robot_manager{

typedef tum_ics_ur_robot_lli::Robot::RobotArm RobotArm_;

class RobotArmClosed: public RobotArm_
{
private:
    RobotScriptManager *m_scriptManager;
    bool m_realRobotArmClosed;
    //RobotArm_ *robot;

public:
    RobotArmClosed(const QString &configFilePath, QString &pcIppAdd);
    ~RobotArmClosed();

    bool connect();
    bool send(const QString& name, bool enable);

};

}


#endif // ROBOTARMCLOSED_H
