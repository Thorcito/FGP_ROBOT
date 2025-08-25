#ifndef UR_ROBOT_LLI_SOSMCONTROL_H
#define UR_ROBOT_LLI_SOSMCONTROL_H

#include<tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include<tum_ics_ur_robot_lli/Robot/KinematicModel.h>
#include<tum_ics_ur_robot_lli/Robot/DynamicModel.h>


namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

// Flo: ??? for now: SOSMCtrl
class SOSMControl: public ControlEffort
{
    // Member variables
private:

    Robot::KinematicModel *m_kinematicModel;
    Robot::DynamicModel *m_dynamicModel;

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

    VectorDOFd m_S_dt0;
    bool m_initErrorS;
    VectorDOFd m_TanhSq_1,m_iTanhSq_1;

    double m_kappa;//parameter for the exponential function of the Bias
    double m_lambda;//tanh(lambda*s)


    double m_controlPeriod;     //[s]
    double m_controlPeriod_2;   //[s]


    double L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,L12;
    double m1,m2,m3,m4,m5,m6;

    // Member Methods
public:
    SOSMControl(Robot::KinematicModel *kModel,
                Robot::DynamicModel *dModel=NULL,
                double weight=1.0,
                const QString& name="SOMSCtrl");
    ~SOSMControl();

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



#endif // UR_ROBOT_LLI_SOSMCONTROL_H
