#ifndef TUM_ICS_ROBOT_CONTROLLERS_TEST_EFFORT_CONTROL_H
#define TUM_ICS_ROBOT_CONTROLLERS_TEST_EFFORT_CONTROL_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <tum_ics_ur_robot_lli/Robot/KinematicModel.h>
#include <tum_ics_ur_robot_lli/Robot/DynamicModel.h>


namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

// Flo: for now: SimpleEffortCtrl

/*!
 * \brief The TestEffortControl class
 *
 * This is a simple dummy control class for effort
 * control to test the ros time issue.
 */
class TestEffortControl : public ControlEffort
{
    // Member variables
private:

    Robot::KinematicModel *m_kinematicModel;
    Robot::DynamicModel *m_dynamicModel;

    bool m_startFlag;

    JointState m_qInit;
    JointState m_qHome;
    JointState m_qPark;

    ros::NodeHandle m_n;

    // Member Methods
public:
    TestEffortControl(
            Robot::KinematicModel *kModel,
            Robot::DynamicModel *dModel=NULL,
            double weight=1.0,
            const QString& name="TestEffortCtrl");
    ~TestEffortControl();

    void setQInit(const JointState& qinit);
    void setQHome(const JointState& qhome);
    void setQPark(const JointState& qpark);

    // Memeber Methods
private:
    bool init();
    bool start();

    VectorDOFd update(const RobotTime& time, const JointState &current);
    bool stop();

};

}
}



#endif // TUM_ICS_ROBOT_CONTROLLERS_TEST_EFFORT_CONTROL_H
