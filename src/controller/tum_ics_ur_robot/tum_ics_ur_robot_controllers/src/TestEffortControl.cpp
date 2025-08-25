#include <tum_ics_ur_robot_controllers/TestEffortControl.h>

#include <tum_ics_ur_robot_msgs/ControlData.h>

namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

TestEffortControl::TestEffortControl(
        Robot::KinematicModel *kModel,
        Robot::DynamicModel *dModel,
        double weight,
        const QString& name):
    ControlEffort(name,SPLINE_TYPE,JOINT_SPACE,weight),
    m_kinematicModel(kModel),
    m_dynamicModel(dModel),
    m_startFlag(false)
{
}

TestEffortControl::~TestEffortControl()
{

}

void TestEffortControl::setQInit(const JointState& qinit)
{
    m_qInit=qinit;
}
void TestEffortControl::setQHome(const JointState& qhome)
{
    m_qHome=qhome;
}
void TestEffortControl::setQPark(const JointState& qpark)
{
    m_qPark=qpark;
}


bool TestEffortControl::init()
{
    return true;
}

bool TestEffortControl::start()
{
    return true;
}

VectorDOFd TestEffortControl::update(const RobotTime& time, const JointState &current)
{
    if(!m_startFlag)
    {
        m_startFlag=true;
    }


    VectorDOFd tau = VectorDOFd::Zero();

    double t=time.tD();

    ROS_INFO("time: %f",t);

    return tau;
}

bool TestEffortControl::stop()
{
    return true;
}




}
}
