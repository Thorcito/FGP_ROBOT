#ifndef UR_ROBOT_LLI_X_IMPEDANCE_CONTROL_H
#define UR_ROBOT_LLI_X_IMPEDANCE_CONTROL_H

#include<tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include<tum_ics_ur_robot_lli/Robot/KinematicModel.h>
#include<tum_ics_ur_robot_lli/Robot/DynamicModel.h>


namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

// Flo: ImpCartesianCtrl
class XImpedanceControl: public ControlEffort
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

    Matrix3d m_Kpx;
    Matrix3d m_Kdx;
    MatrixDOFd m_Kd;
    MatrixDOFd m_Kint;
    MatrixDOFd m_stopKd;


    Vector3d m_xGoal;
    Vector3d m_xini;

    double m_totalTime;

    Vector6d m_DeltaX;
    Vector6d m_DeltaX_1;

    Vector6d m_DeltaXp;
    Vector6d m_DeltaXp_1;

    Vector6d m_iDeltaX;
    Vector6d m_iDeltaX_1;


    Vector6d m_Xef;
    Vector6d m_Xd;

    double m_controlPeriod;     //[s]
    double m_controlPeriod_2;   //[s]

    Robot::KinematicModel *m_kinematicModel;



    // Member Methods
public:
    XImpedanceControl(Robot::KinematicModel *kModel,
                      double controlPeriod=0.002,
                      double weight=1.0,
                      const QString& name="ImpCartesianCtrl");
    ~XImpedanceControl();

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



#endif // UR_ROBOT_LLI_X_IMPEDANCE_CONTROL_H
