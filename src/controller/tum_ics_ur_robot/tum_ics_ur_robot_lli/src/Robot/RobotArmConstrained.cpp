#include <tum_ics_ur_robot_lli/Robot/RobotArmConstrained.h>
#include <tum_ics_ur_robot_lli/Robot/RobotArm.h>
#include <QList>

#define ROBOT_ARM_PTR ((RobotArm*)m_id)

namespace tum_ics_ur_robot_lli{
namespace Robot{

QList<RobotArm*> robots;

RobotArmConstrained::RobotArmConstrained(const QString &configFilePath)
{
    RobotArm* r = new RobotArm(configFilePath);
    robots.append(r);
    m_id = (void*) r;

}

RobotArmConstrained::~RobotArmConstrained()
{
    robots.removeAll((RobotArm*)m_id);
}

bool RobotArmConstrained::add(RobotControllers::Controller *u)
{
    return ROBOT_ARM_PTR->add(u);
}

bool RobotArmConstrained::init()
{
    return ROBOT_ARM_PTR->init();
}

void RobotArmConstrained::start()
{
    ROBOT_ARM_PTR->start();
}

bool RobotArmConstrained::stop()
{
    return ROBOT_ARM_PTR->stop();
}

JointState RobotArmConstrained::qHome() const
{
    return ROBOT_ARM_PTR->qHome();
}

JointState RobotArmConstrained::qPark() const
{
    return ROBOT_ARM_PTR->qPark();
}

}
}
