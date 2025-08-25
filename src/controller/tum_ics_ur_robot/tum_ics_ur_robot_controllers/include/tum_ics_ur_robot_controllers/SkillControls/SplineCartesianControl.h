#ifndef UR_ROBOT_LLI_SPLINE_CARTESIAN_CONTROL_H
#define UR_ROBOT_LLI_SPLINE_CARTESIAN_CONTROL_H

#include<tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include<tum_ics_ur_robot_lli/Robot/KinematicModel.h>
#include<tum_ics_ur_robot_lli/Robot/DynamicModel.h>

#include <tf/transform_broadcaster.h>


namespace tum_ics_ur_robot_lli{
namespace RobotControllers{


class SplineCartesianControl: public ControlEffort
{
    // Member variables
private:

    Robot::KinematicModel *m_kinematicModel;
    Robot::DynamicModel *m_dynamicModel;
    RobotControllers::ControlEffort *m_extCtrl;

    bool m_startFlag;
    double m_localTime;

    VectorDOFd m_qStart;
    JointState m_qInit;
    JointState m_qHome;
    JointState m_qPark;

    // Operational Space
    Vector3d m_xefStart_0;
    Vector3d m_aefStart_0;

    Vector3d m_xefGoal_0;
    Matrix3d m_RefGoal_0;
    Vector3d m_xefGoal_w;
    Matrix3d m_RefGoal_w;



    Vector3d m_eAnglesGoal_0;
    Vector3d m_aefGoal_0;

    Vector3d m_xef_0,m_xefd_0;
    Vector3d m_xefp_0,m_xefpd_0,m_xefppd_0;
    Vector3d m_wefp_0,m_wefpd_0;

    Matrix3d m_RefStart_0;
    Matrix3d m_Ref_0,m_Refd_0;



    double m_ikSign;
    Vector3d m_eAngles_0,m_eAnglesd_0;
    Vector3d m_eAngles_0_1,m_eAnglesd_0_1;
    Vector3d m_aef_0,m_aefd_0;
    Vector3d m_aefp_0,m_aefpd_0;
    Vector3d m_aefpp_0,m_aefppd_0;

    Vector3d m_DeltaXef_0;
    Vector3d m_DeltaXefp_0;
    Vector3d m_iDeltaXef_0;

    Vector3d m_DeltaXef_0_1;
    Vector3d m_DeltaXefp_0_1;
    Vector3d m_iDeltaXef_0_1;

    Vector3d m_DeltaAlpha_0;
    Vector3d m_DeltaAlphap_0;

    double m_q3_limit;//to avoid singularitites
    double m_minDetJ;
    bool m_q3LimitReached;
    bool m_setDefaultXef;


    ros::NodeHandle n;
    ros::Publisher pubCtrlData;
    tf::TransformBroadcaster m_br;


    MatrixDOFd m_Kp;
    MatrixDOFd m_Kd;
    MatrixDOFd m_stopKd;
    MatrixDOFd m_Ki;
    VectorDOFd m_goal;
    double m_totalTime;

    double m_deltaX;
    double m_deltaA;
    double m_deltaQ;


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

    bool m_useTauExtFlag;
    double m_nExtTauThresh;
    bool m_startRecording;
    bool m_updatedDTotalTime;
    double m_eventTime_i;
    double m_elapsedTime;
    double m_DeltaTotalTime;


    double L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,L12;
    double m1,m2,m3,m4,m5,m6;

    // Member Methods
public:
    SplineCartesianControl(Robot::KinematicModel *kModel,
                  Robot::DynamicModel *dModel=NULL, ControlEffort *extCtrl=NULL,
                  double weight=1.0,
                  const QString& name="SplineCartesianCtrl");
    ~SplineCartesianControl();

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



#endif // UR_ROBOT_LLI_SPLINE_CARTESIAN_CONTROL_H
