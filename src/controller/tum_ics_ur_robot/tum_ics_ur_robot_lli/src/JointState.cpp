#include<tum_ics_ur_robot_lli/JointState.h>

#include<iostream>
#include<QTextStream>


namespace tum_ics_ur_robot_lli{


JointState::JointState() :
    q(VectorDOFd::Zero()),
    qp(VectorDOFd::Zero()),
    qpp(VectorDOFd::Zero()),
    tau(VectorDOFd::Zero()),
    qReal(VectorDOFd::Zero()),
    qpReal(VectorDOFd::Zero()),
    qppReal(VectorDOFd::Zero()),
    tauReal(VectorDOFd::Zero())
{

}
JointState::JointState(const JointState &s):
    q(s.q), qp(s.qp), qpp(s.qpp), tau(s.tau), qReal(s.qReal), qpReal(s.qpReal), qppReal(s.qppReal), tauReal(s.tauReal)
{

}
JointState::~JointState()
{

}
JointState& JointState::operator -=(const JointState& other)
{
    q -=other.q;
    qp -=other.qp;
    qpp -=other.qpp;
    tau -=other.tau;
    return *this;
}
JointState& JointState::operator +=(const JointState& other)
{
    q +=other.q;
    qp +=other.qp;
    qpp +=other.qpp;
    tau +=other.tau;
    return *this;
}

QString JointState::toString() const
{

    std::stringstream s;

    s << q.transpose()<<", "<<qp.transpose()<<", "<<qpp.transpose()<<", "<<tau.transpose()<<", "<<
         qReal.transpose()<<", "<<qpReal.transpose()<<", "<<qppReal.transpose()<<", "<<tauReal.transpose();


    return QString(s.str().c_str());
}

sensor_msgs::JointState JointState::toJointStateMsg(const VQString& jointNames, const ros::Time& t) const
{
    sensor_msgs::JointState jsMsg;

    int dof=jointNames.size();

    jsMsg.header.stamp=t;
    jsMsg.name.resize(dof);
    jsMsg.position.resize(dof);
    jsMsg.velocity.resize(dof);
    jsMsg.effort.resize(dof);

    for(int i=0;i<dof;i++)
    {
        jsMsg.name[i]=jointNames.at(i).toStdString();
        jsMsg.position[i]=q(i);
        jsMsg.velocity[i]=qp(i);
        jsMsg.effort[i]=tau(i);
    }


    return jsMsg;
}

sensor_msgs::JointState& JointState::updateMsg(sensor_msgs::JointState& msg, const ros::Time &t) const
{
    int dof=msg.position.size();

    msg.header.stamp=t;


    for(int i=0;i<dof;i++)
    {
        msg.position[i]=q(i);
        msg.velocity[i]=qp(i);
        msg.effort[i]=tau(i);
    }


    return msg;
}

void JointState::initQ(const JointState& other)
{
    q=other.q;
    qp.setZero();
    qpp.setZero();
    tau.setZero();

    qReal=other.q;
    qpReal.setZero();
    qppReal.setZero();
    tauReal.setZero();

}

void JointState::Zero()
{
    q.setZero();
    qp.setZero();
    qpp.setZero();
    tau.setZero();

    qReal.setZero();
    qpReal.setZero();
    qppReal.setZero();
    tauReal.setZero();
}

}
