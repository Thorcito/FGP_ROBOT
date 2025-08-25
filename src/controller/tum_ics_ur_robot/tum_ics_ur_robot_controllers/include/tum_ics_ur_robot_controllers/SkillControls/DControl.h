#ifndef UR_ROBOT_LLI_DCONTROL_H
#define UR_ROBOT_LLI_DCONTROL_H

#include<tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include<tum_ics_ur_robot_lli/Robot/DynamicModel.h>


namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

// Flo: this is GCtrl
class DControl: public ControlEffort
{
    // Member variables
private:

    VectorDOFd m_qStart;
    JointState m_qInit;
    JointState m_qHome;
    JointState m_qPark;

    ros::NodeHandle m_n;
    ros::Publisher m_pubCtrlData;

    MatrixDOFd m_Kd;


    // Member Methods
public:
    DControl(double weight=1.0,
             const QString& name = "DCtrl");
    ~DControl();

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



#endif // UR_ROBOT_LLI_DCONTROL_H
