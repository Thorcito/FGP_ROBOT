#include<tum_ics_ur_robot_controllers/DirectJointControl.h>


namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

DirectJointControl::DirectJointControl(Robot::KinematicModel *kModel,
                                       const QString& name) :
    ControlPosition(name,STANDARD_TYPE,JOINT_SPACE),
    m_kinematicModel(kModel),
    m_startFlag(false)
{

}

DirectJointControl::~DirectJointControl()
{

}

void DirectJointControl::setQInit(const JointState& qinit)
{
    m_qInit=qinit;
}
void DirectJointControl::setQHome(const JointState& qhome)
{
    m_qHome=qhome;
}
void DirectJointControl::setQPark(const JointState& qpark)
{
    m_qPark=qpark;
}


bool DirectJointControl::init()
{
    return true;

}
bool DirectJointControl::start()
{
    return true;

}
//Vector6d DirectJointControl::update(double time, const JointState &current)
//{
//    if(!m_startFlag)
//    {
//        m_qStart=current.q;
//        m_startFlag=true;
//    }


//    VVector6d qD;

//    double t1=10.0;
//    double t2=20.0;

////    ROS_INFO_STREAM("time: "<<time);


//    if(time<=t1)
//    {

//        qD=getJointPVT5(m_qStart,m_qPark.q,time,t1);
//    }
//    else if((time>t1)&&(time<=t2))
//    {

//        qD=getJointPVT5(m_qPark.q,m_qHome.q,time-t1,t1);

//    }
//    else
//    {
//        qD=getJointPVT5(m_qHome.q,m_qPark.q,time-t2,t1);
//    }


//    return qD.at(0);

//}
VectorDOFd DirectJointControl::update(const RobotTime& time, const JointState &current)
{
    if(!m_startFlag)
    {
        m_qStart=current.q;

        VectorDOFd q,qp,qpp;
        q=current.q;
        qp=current.qp;
        qpp=current.qpp;

        ROS_WARN_STREAM("inital q: "<<q.transpose());
        ROS_WARN_STREAM("inital qp: "<<qp.transpose());
        ROS_WARN_STREAM("inital qpp: "<<qpp.transpose());

        m_qStart<<0.0,-90.0,0.0,-90.0,0.0,0.0;
        m_qStart=DEG2RAD(m_qStart);

        ROS_WARN_STREAM("m_qStart: "<<m_qStart.transpose());

        m_startFlag=true;
    }


    VVectorDOFd qD;

    double t1=10.0;
    double t2=20.0;

//    ROS_INFO_STREAM("time: "<<time);


    if(time.tD()<=t1)
    {

        qD=getSpline5<VVectorDOFd,VectorDOFd>(m_qStart,m_qPark.q,time.tD(),t1);
    }
    else if((time.tD()>t1)&&(time.tD()<=t2))
    {

        qD=getSpline5<VVectorDOFd,VectorDOFd>(m_qPark.q,m_qHome.q,time.tD()-t1,t1);

    }
    else
    {
        qD=getSpline5<VVectorDOFd,VectorDOFd>(m_qHome.q,m_qPark.q,time.tD()-t2,t1);
    }


    return qD.at(0);

}
bool DirectJointControl::stop()
{ 
    return true;
}



}
}
