#include<tum_ics_ur_robot_lli/RobotInterface/RobotStatePub.h>


namespace tum_ics_ur_robot_lli{
namespace RobotInterface{


RobotStatePub::RobotStatePub(const QString& topicName, const VQString& jointNames, double updateRate):
    m_topicName(topicName),
    m_jointNames(jointNames),
    m_stop(false),
    m_updateRate(updateRate)
{
    m_jointStatePub = m_n.advertise<sensor_msgs::JointState>(m_topicName.toStdString(),1000);
}

RobotStatePub::~RobotStatePub()
{
    stop();
    if(!QThread::wait(100))
    {
        qWarning("Couldn't terminate RobotStatePub thread.");
        QThread::terminate();
        QThread::wait();
    }
}

void RobotStatePub::start()
{
    QThread::start();
}

void RobotStatePub::stop()
{
    m_stop=true;
}

void RobotStatePub::setJointState(const JointState &in)
{
    m_mutex.lock();

    m_jointState=in;

    m_mutex.unlock();
}


void RobotStatePub::run()
{

    sensor_msgs::JointState msg = m_jointState.toJointStateMsg(m_jointNames);

//    ros::Time ti=ros::Time::now();
//    ros::Time tc;



    while(!m_stop)
    {
        m_mutex.lock();

        m_jointStatePub.publish(m_jointState.updateMsg(msg));

        m_mutex.unlock();

        usleep((unsigned long)(1.0/m_updateRate*1E6));

//        tc=ros::Time::now();

//        double d=tc.toSec()-ti.toSec();

//        ti=tc;

//        ROS_INFO_STREAM("Ts: "<< ((unsigned long)(1.0/m_updateRate*1E6)));

//        ROS_INFO_STREAM("J time: "<<d);

    }

}






}
}


