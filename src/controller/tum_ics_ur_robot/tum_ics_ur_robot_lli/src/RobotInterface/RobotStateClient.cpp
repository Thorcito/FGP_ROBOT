#include <tum_ics_ur_robot_lli/RobotInterface/RobotStateClient.h>
#include <QFile>
#include <QElapsedTimer>
#include <unistd.h>
#include <QSettings>

#include <QTextStream>
#include <QFile>
#include <QDataStream>

namespace tum_ics_ur_robot_lli{
namespace RobotInterface{

RobotStateClient::RobotStateClient(
        const QString &robotIpAddr,
        Version vers,
        quint16 robotStatePort) :
    QTcpSocket(),
    TcpSocketTools(this),
    m_vers(vers),
    m_error(false),
    m_ready(false)
{
    QElapsedTimer timer;

    bool ok = false;
    switch(m_vers)
    {
    case VERSION_1V8:
        ROS_INFO_STREAM("RobotStateClient(): Using robot version 1.8.");
        ok = true;
        break;

    case VERSION_3V1:
        ROS_INFO_STREAM("RobotStateClient(): Using robot version 3.1.");
        ok = true;
        break;

    case VERSION_3V4:
        ROS_INFO_STREAM("RobotStateClient(): Using robot version 3.4.");
        ok = true;
        break;

    case VERSION_3V5:
        ROS_INFO_STREAM("RobotStateClient(): Using robot version 3.5.");
        ok = true;
        break;

    default:
        break;
    }

    if(!ok)
    {
        m_error = true;
        m_errorString.sprintf("Invalid version '%d'",m_vers);
        qCritical("%s",m_errorString.toLatin1().data());
        return;
    }

    bool connected = false;

    timer.start();
    while(!connected)
    {

        if(timer.elapsed()>TOTAL_CONNECT_TIME_OUT)
        {
            m_error=true;
            m_errorString="RobotStateClient(): Total connection time reached! sorry :(, did you turn on the robot?";
            qDebug()<<m_errorString;
            QTcpSocket::close();
            return;
        }
        QTcpSocket::connectToHost(robotIpAddr, robotStatePort);
        qDebug("RobotStateClient(): trying to connect to server");

        if (QTcpSocket::waitForConnected(CONNECT_TIME_OUT))
        {
            connected = true;
            QHostAddress hostAddr = QTcpSocket::localAddress();
            if (hostAddr != QHostAddress::Null)
            {
                qDebug()<<QString("RobotStateClient(): Client connected on address %1:%2")\
                          .arg(hostAddr.toString()).arg(QTcpSocket::localPort());
            }

        }
        else
        {
            sleep(1);
            qDebug()<<"RobotStateClient(): Client socket failed to connect."+QTcpSocket::errorString();
            qDebug()<<"Trying again ...";
        }
    }


    qDebug()<<"RobotStateClient(): Client succefully connected.";
    return;

}
RobotStateClient::~RobotStateClient()
{
    //Close the client connection
    if(QTcpSocket::isOpen())
    {
        QTcpSocket::close();
    }
}

bool RobotStateClient::update()
{
    // READ
    QByteArray d=TcpSocketTools::readByteArrayAll();
    m_buffer.append(d);
    //    QTcpSocket::write(d,d.size());

//        qDebug()<<"Size of data: "<<d.size()<<" StructSize: "<<sizeof(RobotStateMsg);
//    ROS_WARN_STREAM("Size of data: "<< d.size());

    // unpack
    switch(m_vers)
    {
    case VERSION_1V8:
        unpackDataV1V8();
        break;

    case VERSION_3V1:
        unpackDataV3V1();
        break;

    case VERSION_3V4:
        unpackDataV3V4();
        break;

    case VERSION_3V5:
        unpackDataV3V5();
        break;

    default:
        break;
    }
    return true;
}

JointState RobotStateClient::getJointState() const
{
    JointState js;

    switch(m_vers)
    {
    case VERSION_1V8:
        js.q=m_robotStateMsgV1V8.qAct;
        js.qp=m_robotStateMsgV1V8.qpAct;
        js.tau=m_robotStateMsgV1V8.iAct;
        break;

    case VERSION_3V1:
        js.q=m_robotStateMsgV3V1.qAct;
        js.qp=m_robotStateMsgV3V1.qpAct;
        js.tau=m_robotStateMsgV3V1.iAct;
        break;

    case VERSION_3V4:
        js.q=m_robotStateMsgV3V4.qAct;
        js.qp=m_robotStateMsgV3V4.qpAct;
        js.tau=m_robotStateMsgV3V4.iAct;
        break;

    case VERSION_3V5:
        js.q=m_robotStateMsgV3V5.qAct;
        js.qp=m_robotStateMsgV3V5.qpAct;
        js.tau=m_robotStateMsgV3V5.iAct;
        break;

    default:
        break;
    }


    return js;
}

RobotStateClient::RobotStateMsgV1V8 RobotStateClient::getRobotStateMsgV1V8() const
{
    return m_robotStateMsgV1V8;
}

RobotStateClient::RobotStateMsgV3V1 RobotStateClient::getRobotStateMsgV3V1() const
{
    return m_robotStateMsgV3V1;
}

RobotStateClient::RobotStateMsgV3V4 RobotStateClient::getRobotStateMsgV3V4() const
{
    return m_robotStateMsgV3V4;
}

RobotStateClient::RobotStateMsgV3V5 RobotStateClient::getRobotStateMsgV3V5() const
{
    return m_robotStateMsgV3V5;
}

bool RobotStateClient::isReady() const
{
    //This flag will be changed to true with the first valid packet (JointState packet)
    return m_ready;
}

bool RobotStateClient::error()
{
    return m_error;
}
const QString& RobotStateClient::errorString()
{
    return m_errorString;
}

void RobotStateClient::unpackDataV1V8()
{
    QDataStream dataStream(&m_buffer,QIODevice::ReadOnly);
    dataStream.setByteOrder(QDataStream::BigEndian);
    dataStream.setFloatingPointPrecision(QDataStream::DoublePrecision);

    RobotStateMsgV1V8& msg = m_robotStateMsgV1V8;
    const int size = PACKET_SIZE_V1V8;

    while(!(m_buffer.size()<size))
    {

        //        int        msgSize;     //        Message Size          integer	1		4
        //        double     time;        //        Time                  double	1		8
        //        Vector6d   qTar;        //        q target              double	6		48
        //        Vector6d   qpTar;       //        qd target             double	6		48
        //        Vector6d   qppTar;      //        qdd target            double	6		48
        //        Vector6d   iTar;        //        I target              double	6		48
        //        Vector6d   mTar;        //        M target              double	6		48
        //        Vector6d   qAct;        //        q actual              double	6		48
        //        Vector6d   qpAct;       //        qd actual             double	6		48
        //        Vector6d   iAct;        //        I actual              double	6		48
        //        Vector6d   iControl;    //        I control             double	6		48
        //        Vector6d   vtoolAct;    //        Tool vector actual	double	6		48
        //        Vector6d   tcpSpeedAct; //        TCP speed actual      double	6		48
        //        Vector6d   tcpForce;    //        TCP force             double	6		48
        //        Vector6d   vtoolTar;    //        Tool vector target	double	6		48
        //        Vector6d   tcpSpeedTar; //        TCP speed target      double	6		48
        //        double     dio;         //        Digital input bits	double	1		8
        //        Vector6d   motorTemp;   //        Motor temperatures	double	6		48
        //        double     ctrlTimer;   //        Controller Timer      double	1		8
        //        double     testVal;     //        Test value            double	1		8
        //        double     robotMode;   //        Robot Mode            double	1		8
        //        Vector6d   jointModes;  //        Joint Modes           double	6		48

        // DECODE
        dataStream >> msg.msgSize;
        if(msg.msgSize != size)
        {
            m_buffer.remove(0, size);
            break;
        }
        dataStream >> msg.time;

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qpTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qppTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.iTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.mTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qpAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.iAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.iControl(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.vtoolAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.tcpSpeedAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.tcpForce(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.vtoolTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.tcpSpeedTar(i);

        dataStream >> msg.dio;

        for(int i = 0; i < 6; i++)
            dataStream >> msg.motorTemp(i);

        dataStream >> msg.ctrlTimer;
        dataStream >> msg.testVal;
        dataStream >> msg.robotMode;

        for(int i = 0; i < 6; i++)
            dataStream >> msg.jointModes(i);


        // filtering zero positions sent by the robot at startup
        for(int i=0;i<STD_DOF;i++)
        {

            if(abs(msg.qAct(i))>1e-3)
            {
                m_ready=true;
                break;
            }
        }

        m_buffer.remove(0,size);
    }
}


void RobotStateClient::unpackDataV3V1()
{
    QDataStream dataStream(&m_buffer,QIODevice::ReadOnly);
    dataStream.setByteOrder(QDataStream::BigEndian);
    dataStream.setFloatingPointPrecision(QDataStream::DoublePrecision);

    RobotStateMsgV3V1& msg = m_robotStateMsgV3V1;
    const int size = PACKET_SIZE_V3V1;

    while(!(m_buffer.size()<size))
    {
        // DECODE
        dataStream >> msg.msgSize;
        if(msg.msgSize != size)
        {
            m_buffer.remove(0, size);
            break;
        }
        dataStream >> msg.time;
        for(int i = 0; i < 6; i++)
            dataStream >> msg.qTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qpTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qppTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.iTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.mTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qpAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.iAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.iControl(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.vtoolAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.tcpSpeedAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.tcpForce(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.vtoolTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.tcpSpeedTar(i);

        dataStream >> msg.dio;

        for(int i = 0; i < 6; i++)
            dataStream >> msg.motorTemp(i);

        dataStream >> msg.ctrlTimer;
        dataStream >> msg.testVal;
        dataStream >> msg.robotMode;

        for(int i = 0; i < 6; i++)
            dataStream >> msg.jointModes(i);


        dataStream >> msg.safetyMode;

        for(int i = 0; i < 6; i++)
            dataStream >> msg.undefined(i);

        for(int i = 0; i < 3; i++)
            dataStream >> msg.toolAcc(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.undefined1(i);

        dataStream >> msg.speedScaling;
        dataStream >> msg.linMomNorm;
        dataStream >> msg.undefined2;
        dataStream >> msg.undefined3;
        dataStream >> msg.voltMain;
        dataStream >> msg.voltRobot;
        dataStream >> msg.currentRobot;

        for(int i = 0; i < 6; i++)
            dataStream >> msg.voltActual(i);

        // filtering zero positions sent by the robot at startup
        for(int i=0;i<STD_DOF;i++)
        {

            if(abs(msg.qAct(i))>1e-3)
            {
                m_ready=true;
                break;
            }
        }

//        std::stringstream ss;
//        QString str;
//        ss << RAD2DEG(msg.qAct.transpose());
//        str = QString(ss.str().c_str());
//        ss.str("");

//        qDebug("q: [%s]",str.toLatin1().data());

        m_buffer.remove(0,size);
    }
}

void RobotStateClient::unpackDataV3V4()
{
    QDataStream dataStream(&m_buffer,QIODevice::ReadOnly);
    dataStream.setByteOrder(QDataStream::BigEndian);
    dataStream.setFloatingPointPrecision(QDataStream::DoublePrecision);

    RobotStateMsgV3V4& msg = m_robotStateMsgV3V4;
    const int size = PACKET_SIZE_V3V4;

//    ROS_WARN_STREAM("------------>DataV3V4 real size: "<<m_buffer.size()<<" PACKET_SIZE_V3V4: "<<PACKET_SIZE_V3V4);

    while(!(m_buffer.size()<size))
    {
        // DECODE
        dataStream >> msg.msgSize;
        if(msg.msgSize != size)
        {
            m_buffer.remove(0, size);
            break;
        }
//        ROS_INFO_STREAM("msgSize: "<<msg.msgSize);
        dataStream >> msg.time;

//        ROS_INFO_STREAM("time: "<<msg.time);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qpTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qppTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.iTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.mTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qpAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.iAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.iControl(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.vtoolAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.tcpSpeedAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.tcpForce(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.vtoolTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.tcpSpeedTar(i);

        dataStream >> msg.dio;

        for(int i = 0; i < 6; i++)
            dataStream >> msg.motorTemp(i);

        dataStream >> msg.ctrlTimer;
        dataStream >> msg.testVal;
        dataStream >> msg.robotMode;

        for(int i = 0; i < 6; i++)
            dataStream >> msg.jointModes(i);


        dataStream >> msg.safetyMode;

        for(int i = 0; i < 6; i++)
            dataStream >> msg.undefined(i);

        for(int i = 0; i < 3; i++)
            dataStream >> msg.toolAcc(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.undefined1(i);

        dataStream >> msg.speedScaling;
        dataStream >> msg.linMomNorm;
        dataStream >> msg.undefined2;
        dataStream >> msg.undefined3;
        dataStream >> msg.voltMain;
        dataStream >> msg.voltRobot;
        dataStream >> msg.currentRobot;

        for(int i = 0; i < 6; i++)
            dataStream >> msg.voltActual(i);

        // New in version 3.4
        dataStream >> msg.dioO;
        dataStream >> msg.stateProg;

        // filtering zero positions sent by the robot at startup
        for(int i=0;i<STD_DOF;i++)
        {

            if(abs(msg.qAct(i))>1e-3)
            {
                m_ready=true;
                break;
            }
        }

//        std::stringstream ss;
//        QString str;
//        ss << RAD2DEG(msg.qAct.transpose());
//        str = QString(ss.str().c_str());
//        ss.str("");

//        qDebug("q: [%s]",str.toLatin1().data());

        m_buffer.remove(0,size);
    }
}

void RobotStateClient::unpackDataV3V5()
{
    QDataStream dataStream(&m_buffer,QIODevice::ReadOnly);
    dataStream.setByteOrder(QDataStream::BigEndian);
    dataStream.setFloatingPointPrecision(QDataStream::DoublePrecision);

    RobotStateMsgV3V5& msg = m_robotStateMsgV3V5;
    const int size = PACKET_SIZE_V3V5;

//    ROS_WARN_STREAM("------------>DataV3V5 real size: "<<m_buffer.size()<<" PACKET_SIZE_V3V5: "<<PACKET_SIZE_V3V5);

    while(!(m_buffer.size()<size))
    {
        // DECODE
        dataStream >> msg.msgSize;
        if(msg.msgSize != size)
        {
            m_buffer.remove(0, size);
            break;
        }
//        ROS_INFO_STREAM("msgSize: "<<msg.msgSize);
        dataStream >> msg.time;

//        ROS_INFO_STREAM("time: "<<msg.time);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qpTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qppTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.iTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.mTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.qpAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.iAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.iControl(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.vtoolAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.tcpSpeedAct(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.tcpForce(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.vtoolTar(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.tcpSpeedTar(i);

        dataStream >> msg.dio;

        for(int i = 0; i < 6; i++)
            dataStream >> msg.motorTemp(i);

        dataStream >> msg.ctrlTimer;
        dataStream >> msg.testVal;
        dataStream >> msg.robotMode;

        for(int i = 0; i < 6; i++)
            dataStream >> msg.jointModes(i);


        dataStream >> msg.safetyMode;

        for(int i = 0; i < 6; i++)
            dataStream >> msg.undefined(i);

        for(int i = 0; i < 3; i++)
            dataStream >> msg.toolAcc(i);

        for(int i = 0; i < 6; i++)
            dataStream >> msg.undefined1(i);

        dataStream >> msg.speedScaling;
        dataStream >> msg.linMomNorm;
        dataStream >> msg.undefined2;
        dataStream >> msg.undefined3;
        dataStream >> msg.voltMain;
        dataStream >> msg.voltRobot;
        dataStream >> msg.currentRobot;

        for(int i = 0; i < 6; i++)
            dataStream >> msg.voltActual(i);

        // New in version 3.5
        dataStream >> msg.dioO;
        dataStream >> msg.stateProg;

        for(int i = 0; i < 3; i++)
            dataStream >> msg.elbowPos(i);

        for(int i = 0; i < 3; i++)
            dataStream >> msg.elbowVel(i);

        // filtering zero positions sent by the robot at startup
        for(int i=0;i<STD_DOF;i++)
        {

            if(abs(msg.qAct(i))>1e-3)
            {
                m_ready=true;
                break;
            }
        }

//        std::stringstream ss;
//        QString str;
//        ss << RAD2DEG(msg.qAct.transpose());
//        str = QString(ss.str().c_str());
//        ss.str("");

//        qDebug("q: [%s]",str.toLatin1().data());

        m_buffer.remove(0,size);
    }
}

}
}


