#ifndef UR_ROBOT_LLI_PIDCONTROL_H
#define UR_ROBOT_LLI_PIDCONTROL_H

#include<tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include<tum_ics_ur_robot_lli/Robot/KinematicModel.h>
#include<tum_ics_ur_robot_lli/Robot/DynamicModel.h>


namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

// Flo: ??? seems to be JointSplineCtrl, for now: PIDCtrl
class PIDControl: public ControlEffort
{
    // Member variables
private:

    bool m_startFlag;

    VectorDOFd m_qStart;
    JointState m_qInit;
    JointState m_qHome;
    JointState m_qPark;

    ros::NodeHandle n;
    ros::Publisher pubCtrlData;

    MatrixDOFd m_Kp;
    MatrixDOFd m_Kd;
    MatrixDOFd m_Ki;
    VectorDOFd m_goal;
    double m_totalTime;

    VectorDOFd m_DeltaQ;
    VectorDOFd m_DeltaQ_1;

    VectorDOFd m_DeltaQp;
    VectorDOFd m_DeltaQp_1;

    VectorDOFd m_iDeltaQ;
    VectorDOFd m_iDeltaQ_1;


    double m_controlPeriod;     //[s]
    double m_controlPeriod_2;   //[s]


    // Member Methods
public:
    PIDControl(double controlPeriod, double weight=1.0, const QString& name="PIDCtrl");
    ~PIDControl();

    void setQInit(const JointState& qinit);
    void setQHome(const JointState& qhome);
    void setQPark(const JointState& qpark);



    // Memeber Methods
private:
    bool init();
    bool start();

    VectorDOFd update(const RobotTime& time, const JointState &current);
    bool stop();
    VectorDOFd Gc(const JointState& js);

};

}
}



#endif // UR_ROBOT_LLI_PIDCONTROL_H
