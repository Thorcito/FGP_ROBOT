#ifndef UR_ROBOT_LLI_GCONTROL_H
#define UR_ROBOT_LLI_GCONTROL_H

#include<tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include<tum_ics_ur_robot_lli/Robot/DynamicModel.h>


namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

// Flo: this is GCtrl
class GControl: public ControlEffort
{
    // Member variables
private:

    Robot::DynamicModel *m_dynamicModel;

    VectorDOFd m_qStart;
    JointState m_qInit;
    JointState m_qHome;
    JointState m_qPark;

    ros::NodeHandle n;
    ros::Publisher pubCtrlData;



    double L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,L12;
    double m1,m2,m3,m4,m5,m6;

    // Member Methods
public:
    GControl(Robot::DynamicModel *dModel=NULL,
             double weight=1.0,
             const QString& name = "GCtrl");
    ~GControl();

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



#endif // UR_ROBOT_LLI_GCONTROL_H
