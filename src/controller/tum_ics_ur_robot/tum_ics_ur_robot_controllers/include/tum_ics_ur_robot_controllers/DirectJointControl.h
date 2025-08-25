#ifndef UR_ROBOT_LLI_DIRECTJOINTCONTROL_H
#define UR_ROBOT_LLI_DIRECTJOINTCONTROL_H

#include<tum_ics_ur_robot_lli/RobotControllers/ControlPosition.h>
#include<tum_ics_ur_robot_lli/Robot/KinematicModel.h>



namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

// Flo: ??? for now: DirectJointCtrl
class DirectJointControl: public ControlPosition
{
    // Member variables
private:

    Robot::KinematicModel *m_kinematicModel;
    bool m_startFlag;

    VectorDOFd m_qStart;
    JointState m_qInit;
    JointState m_qHome;
    JointState m_qPark;

    // Member Methods
public:
    DirectJointControl(Robot::KinematicModel *kModel,
                       const QString& name = "DirectJointCtrl");
    ~DirectJointControl();

    void setQInit(const JointState& qinit);
    void setQHome(const JointState& qhome);
    void setQPark(const JointState& qpark);


    // Memeber Methods
private:
    bool init();
    bool start();
//    Vector6d update(double time, const JointState &current);
    VectorDOFd update(const RobotTime& time, const JointState &current);
    bool stop();

};

}
}



#endif // UR_ROBOT_LLI_DIRECTJOINTCONTROL_H
