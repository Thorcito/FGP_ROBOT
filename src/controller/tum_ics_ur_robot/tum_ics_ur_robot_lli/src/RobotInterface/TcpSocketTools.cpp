#include<tum_ics_ur_robot_lli/RobotInterface/TcpSocketTools.h>
#include<tum_ics_ur_robot_lli/RobotInterface/DataUnpacker.h>

#include<tum_ics_ur_robot_msgs/JointPoseInt.h>
#include<tum_ics_ur_robot_msgs/JointPoseFloat.h>

#include <QTextStream>
#include <QFile>
#include <QDataStream>



namespace tum_ics_ur_robot_lli{
namespace RobotInterface{

#define DEBUG_DATA

TcpSocketTools::TcpSocketTools(QTcpSocket *socket):
    m_socket(socket)
{
    qInt32Pub=n.advertise<tum_ics_ur_robot_msgs::JointPoseInt>("JointPoseInt32",100);
    qFloat64Pub=n.advertise<tum_ics_ur_robot_msgs::JointPoseFloat>("JointPoseFloat64",100);
}
TcpSocketTools::~TcpSocketTools(){}


QByteArray TcpSocketTools::readByteArrayAll()
{
    m_socket->waitForReadyRead(0);
    int bytesAvail = m_socket->bytesAvailable();
    if (bytesAvail > 0)
    {
        return  m_socket->readAll();
    }
    else
    {
        QByteArray a;
        return a;
    }
}

void TcpSocketTools::writeString( const QString &line )
{
    if (line.length() > 0)
    {
        m_socket->write(line.toLatin1());
        if (! m_socket->waitForBytesWritten())
        {

            qDebug()<<"Write to the socket failed";

        }
    }
}

void TcpSocketTools::writeByteArray(const QByteArray &data)
{
    if(data.length() > 0)
    {
        m_socket->write(data);
        if (! m_socket->waitForBytesWritten())
        {

            qDebug()<<"Server - write to the socket failed";

        }
    }
}

void TcpSocketTools::writeCommand(MsgHeader msg)
{
    QByteArray cmdData;
    QDataStream s(&cmdData, QIODevice::ReadWrite);
    s << qint32(msg);

    writeByteArray(cmdData);
}

void TcpSocketTools::writeTraj(int waypointId, const VectorDOFd &qd, double acc, double vel, double totalTime, double round)
{
    QByteArray cmdData;
    QDataStream s(&cmdData, QIODevice::ReadWrite);


    s <<qint32(waypointId);
    for (int i = 0;i <= STD_DOF-1; i++)
    {
        s << qint32(qd[i]*DataUnpacker::MULT_jointstate);
    }

    s << qint32(acc*DataUnpacker::MULT_jointstate)<< qint32(vel*DataUnpacker::MULT_jointstate);
    s << qint32(totalTime*DataUnpacker::MULT_time)<< qint32(round*DataUnpacker::MULT_blend);

    writeByteArray(cmdData);

#ifdef DEBUG_DATA
    //to test
    tum_ics_ur_robot_msgs::JointPoseInt msg;
    tum_ics_ur_robot_msgs::JointPoseFloat msgF;

    msg.header.stamp=ros::Time::now();
    msgF.header.stamp=msg.header.stamp;

    msg.waypoint=qint32(waypointId);
    msgF.waypoint=qint64(waypointId);

    for (int i = 0;i <= STD_DOF-1; i++)
    {
        msg.q[i]=qint32(qd[i]*DataUnpacker::MULT_jointstate);
        msgF.q[i]=qd[i];
    }

    msg.acc=qint32(acc*DataUnpacker::MULT_jointstate);
    msg.vel=qint32(vel*DataUnpacker::MULT_jointstate);
    msg.totaltime=qint32(totalTime*DataUnpacker::MULT_time);
    msg.round=qint32(round*DataUnpacker::MULT_blend);

    msgF.acc=acc;
    msgF.vel=vel;
    msgF.totaltime=totalTime;
    msgF.round=round;

    qInt32Pub.publish(msg);
    qFloat64Pub.publish(msgF);
#endif

}


QByteArray TcpSocketTools::genMsgPacket(MsgHeader msg)
{
    QByteArray cmdData;
    QDataStream s(&cmdData, QIODevice::ReadWrite);
    s << qint32(msg);
    return cmdData;
}

}
}
