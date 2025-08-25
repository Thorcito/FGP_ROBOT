#include<ros/ros.h>

#include <tum_ics_ur_robot_lli/RobotInterface/CommInterface.h>

#include<QElapsedTimer>
#include<QApplication>
#include<QFileInfo>
#include<QSettings>
#include<unistd.h>

#include <QTextStream>
#include <QFile>
#include <QDataStream>

#include <tumtools/Math/EigenDefs.h>

#define COMM_INTF_BEFORE_TRAJ_FOLLOW_WAIT_CYCLES    250     /* 2 sec @ 8 ms cycle time */




// #define COMM_PERIOD  0.008
// #define COMM_PERIOD_2  0.004
// #define MULT_jointstate  1.0E8 //10000.0;
// #define MULT_time  1000000.0
// #define MULT_blend  1000.0



namespace tum_ics_ur_robot_lli{
namespace RobotInterface{


const double CommInterface::COMM_PERIOD = 0.008;
const double CommInterface::COMM_PERIOD_2 = 0.004;
const double CommInterface::MULT_jointstate = 1.0E8;//10000.0;
const double CommInterface::MULT_time = 1000000.0;
const double CommInterface::MULT_blend = 1000.0;


CommInterface::CommInterface(
        const QString &pcIpAddr,
        const quint16 trajPort,
        const QString &robotIpAddr,
        const quint16 robotStatePort,
        QObject *parent):
    TThread(RtThreads::Thread::RtThreadMode,"commInterface"),
    m_pcIp(pcIpAddr),
    m_trajPort(trajPort),
    m_robotIp(robotIpAddr),
    m_robotStatePort(robotStatePort),
    m_robotVersion(RobotStateClient::VERSION_1V8),
    m_serverTraj(0),
    m_socketTraj(0),
    m_error(false),
    m_stopThread(false),
    m_stoppedThread(true),
    m_mstate(STATE_INIT),
    m_qStart(VectorDOFd::Zero()),
    m_qd(VectorDOFd::Zero()),
    m_qpd(VectorDOFd::Zero()),
    m_currentCycle(0),
    m_waypointId(0),
    m_robotStateClient(NULL),
    m_ready(false),
    m_motionCommand(POSITION_INTF),
    m_pend_mstate(STATE_IDLE),
    m_mstatePendFlag(false),
    m_mWaitCnt(0),
    m_cnt(0)
{
    TThread::start();

    if(TThread::isRtThread())
    {
        qDebug("CommInterface: Using RtThread");
        TThread::rtThread()->setPriority(7);
        TThread::rtThread()->setStackSize(16*1024*1024);
        TThread::rtThread()->setStackPreFaultSize(64*1024*1024);
    }
    else
    {
        qDebug("CommInterface: Using QThread");
    }

    m_setScriptState = m_node.serviceClient
            <tum_ics_ur_robot_msgs::setScriptManagerState>
            ("setScriptManagerState");
}

CommInterface::CommInterface(const QString &configFilePath, RobotStateClient *extRobotStateClient):
    TThread(RtThreads::Thread::RtThreadMode,"commInterface"),
    m_robotVersion(RobotStateClient::VERSION_1V8),
    m_serverTraj(0),
    m_socketTraj(0),
    m_error(false),
    m_stopThread(false),
    m_stoppedThread(true),
    m_mstate(STATE_INIT),
    m_qStart(VectorDOFd::Zero()),
    m_qd(VectorDOFd::Zero()),
    m_qpd(VectorDOFd::Zero()),
    m_currentCycle(0),
    m_waypointId(0),
    m_robotStateClient(extRobotStateClient),
    m_ready(false),
    m_motionCommand(POSITION_INTF),
    m_pend_mstate(STATE_IDLE),
    m_mstatePendFlag(false),
    m_mWaitCnt(0),
    m_cnt(0)
{

    if(!parseConfig(configFilePath))
    {
        return;
    }

    TThread::start();

    if(TThread::isRtThread())
    {
        qDebug("CommInterface: Using RtThread");
    }
    else
    {
        qDebug("CommInterface: Using QThread");
    }


    m_setScriptState = m_node.serviceClient
            <tum_ics_ur_robot_msgs::setScriptManagerState>
            ("setScriptManagerState");

    //We need to control if the robotStateClient (rsc) is internally created
    //or externally provided
    //this flag will be used in the destructor, if the rsc is internally created
    //then we need to delete it, otherwise it should be deleted externally.
    if(extRobotStateClient!=NULL)
    {
        m_extRobotSateClientFlag=true;
    }
    else
    {
        m_extRobotSateClientFlag=false;
    }
}



CommInterface::~CommInterface()
{
    setFinishMState();

    QElapsedTimer timer;
    timer.start();

    //Wait until stoppedThread changes or the limit time has been reached (5secs)
    while((!m_stoppedThread) && (timer.elapsed()<5000))
    {
        ::usleep(5*1E5);
    }

    if(m_stoppedThread)
    {
        cleanup();
        qDebug("CommInterface class destroyed.");
        return;
    }

    qWarning("WARNING: Waiting for thread of CommInterface class to finish ...");

    //if the thread has not been stopped
    if(!TThread::wait(500))
    {
        TThread::terminate();
        TThread::wait();
        qDebug()<<"CommInterface::~CommInterface: Thread forced to end!!!";
    }

    cleanup();
    qDebug("CommInterface class destroyed.");
}

bool CommInterface::error()
{
    return m_error;
}
const QString& CommInterface::errorString()
{
    return m_errorString;
}



JointState CommInterface::getCurrentJointState()
{
    JointState jS;
    m_mutexJointState.lock();
    jS=m_jointState;
    m_mutexJointState.unlock();

    return jS;
}
void CommInterface::setQd(const VectorDOFd &qTarget, const VectorDOFd &qpTarget)
{
    m_mutex.lock();
    m_qd=qTarget;
    m_qpd=qpTarget;
    m_mutex.unlock();


}

void CommInterface::setIdleMState()
{
    m_mutex.lock();
    if(m_mstate == STATE_TRAJ_FOLLOW)
    {
        m_pend_mstate = STATE_IDLE;
        m_mstatePendFlag = true;
    }
    m_mutex.unlock();
}

void CommInterface::setFinishMState()
{
    m_mutex.lock();

    if(m_mstate == STATE_TRAJ_FOLLOW)
    {
        m_pend_mstate = STATE_FINISH;
        m_mstatePendFlag = true;
    }
    m_mutex.unlock();

}
CommInterface::State CommInterface::getMState()
{
    State s;
    m_mutex.lock();
    s = m_mstate;
    m_mutex.unlock();
    return s;
}

bool CommInterface::robotStateIsReady() const
{
    return m_robotStateClient->isReady();
}

bool CommInterface::isReady() const
{
    return m_ready;
}

const CommInterface::MotionCommand CommInterface::getMotionIntf() const
{
    return m_motionCommand;
}
void CommInterface::setMotionIntf(const MotionCommand intf)
{
    m_motionCommand=intf;
}


bool CommInterface::init()
{
    // stop all traj script threads
    bool ret = enableTrajScript(POSITION_INTF,false);
    TThread::usleep(500*1000);
    ret = enableTrajScript(VELOCITY_INTF,false) && ret;
    TThread::usleep(500*1000);

    if(!ret)
    {
        m_error = true;
        m_errorString = "CommInterface::init(): Failed to contact script manager.";
        qDebug("%s",m_errorString.toLatin1().data());
        return false;
    }

    //Start the client to get the robot state from port 30003

//    ROS_ERROR_STREAM(__FILE__<<" -- m_robotVersion:"<<m_robotVersion);

    if(!m_extRobotSateClientFlag)
    {
        //If the RSC is not externally, we need to creat it.
        m_robotStateClient=new RobotStateClient(m_robotIp,
                                                m_robotVersion,
                                                m_robotStatePort);
        if(m_robotStateClient->error())
        {
            m_error=true;
            m_errorString="CommInterface::init(): RobotStateClient: " + m_robotStateClient->errorString();
            qDebug()<<m_errorString;
            return false;
        }
    }

    return true;
}

void CommInterface::run()
{

    m_stoppedThread=false;

    if(!init())
    {
        m_stoppedThread=true;
        qDebug()<<"CommInterface::run(): !init-->Thread exited";
        return;
    }



    while(!m_stopThread)
    {
        //        m_cnt++;

        //        if(m_cnt % 100 == 0)
        //        {
        //            qDebug("CommInterface: thread alive");
        //            qDebug("CommInterface: state = %d", m_mstate);
        //        }


        QElapsedTimer tic;

        tic.start();

        //        QApplication::processEvents();

        ///TODO: CHECK ALL THE TCPSOCKETS AND DO AS FOLLOWS!!!

        //http://www.qtforum.org/article/27867/qtcpsocket-without-event-loop.html

        //        qDebug("CommInterface: Before update state");
        m_robotStateClient->update();

        //Try to read the current JointState from UR controller
        m_mutexJointState.lock();
        m_jointState=m_robotStateClient->getJointState();
        m_mutexJointState.unlock();

        //        qDebug("CommInterface: After update state");

        //        qDebug()<<"Joints: "<<m_jointState.toString();

        //calculate the next state of the state machine
        updateStateMachine();

        //executes the correspondign functions to the current state of the state machine
        executeMState();


        double elapsedTime=tic.nsecsElapsed()*1E-9;
        double delay = (COMM_PERIOD-elapsedTime)*1E6;

        //        ROS_INFO_STREAM("E time: "<<elapsedTime);

        if(m_mstate == STATE_TRAJ_FOLLOW && elapsedTime > COMM_PERIOD)
        {
            ROS_WARN_STREAM("Communication loop period exceeds the limit: "<<elapsedTime);
            continue;
        }


        if(TThread::isRtThread())
        {
            //            qDebug("RtThread sleep");
            TThread::usleep(COMM_PERIOD*1e6);
        }
        else
        {
            //            qDebug("Normal sleep");
            if(delay <= 0)
            {
                TThread::usleep(10*1000);
                continue;
            }
            TThread::usleep((unsigned long)delay);
        }
    }

    m_stoppedThread = true;
    qDebug("CommInterface::run(): Thread exited");


}

void CommInterface::stop()
{
    if(m_stoppedThread)
    {
        return;
    }

    m_stopThread = true;
    qDebug()<<"CommInterface::stop(): m_stopThread="<<m_stopThread;
}

void CommInterface::cleanup()
{
    if(!m_extRobotSateClientFlag)
    {
        //If the ext RSC then it has been created internally, therfore it needs to be
        //deleted!
        if(m_robotStateClient!=NULL)
        {
            delete m_robotStateClient;
            m_robotStateClient=NULL;
        }
    }
}

void CommInterface::updateStateMachine()
{
    //Get the JointState from the controller (port 50001)

    if(m_mstate == STATE_INIT)
    {
        //        ROS_INFO_STREAM("STATE_INIT up");

        if(m_robotStateClient!=NULL)
        {
            if(m_robotStateClient->isReady())
            {
                qDebug("STATE_INIT: Robot state client is ready.");
                m_mutex.lock();
                m_mstate = STATE_IDLE;
                m_mutex.unlock();
            }
        }
    }
    else if(m_mstate == STATE_IDLE)
    {
        ROS_INFO_STREAM("STATE_IDLE up");
        m_mutex.lock();
        m_mstate = STATE_BEFORE_TRAJ_FOLLOW_WAIT;
        m_mutex.unlock();
    }
    else if(m_mstate == STATE_BEFORE_TRAJ_FOLLOW_WAIT)
    {
        if(m_mWaitCnt > COMM_INTF_BEFORE_TRAJ_FOLLOW_WAIT_CYCLES)
        {
            m_mutex.lock();
            m_mstate = STATE_BEFORE_TRAJ_FOLLOW;
            m_mutex.unlock();
        }
    }
    else if(m_mstate == STATE_BEFORE_TRAJ_FOLLOW)
    {
        if(m_motionCommand==POSITION_INTF)
        {
            ROS_INFO_STREAM("STATE_BEFORE_TRAJ_FOLLOW: POSITION_INTERFACE");
        }
        else
        {
            ROS_WARN_STREAM("STATE_BEFORE_TRAJ_FOLLOW: VELOCITY_INTERFACE");
        }

        // next state
        m_mutex.lock();
        m_mstate = STATE_TRAJ_FOLLOW;
        m_mutex.unlock();
    }
    else if(m_mstate == STATE_TRAJ_FOLLOW) // exit on finish or idle
    {
//        ROS_INFO_STREAM("STATE_TRAJ_FOLLOW");
    }
    else if(m_mstate == STATE_FINISH)
    {
        ROS_INFO_STREAM("STATE_FINISH");
    }

    // external set state has precedence over interal new state
    m_mutex.lock();
    if(m_mstatePendFlag)
    {
        m_mstate = m_pend_mstate;
        m_mstatePendFlag = false;
    }
    m_mutex.unlock();
}

void CommInterface::executeMState()
{
    if(m_mstate != STATE_TRAJ_FOLLOW)
    {
        m_ready = false;
    }

    if(m_mstate==STATE_INIT)
    {
    }
    else if(m_mstate==STATE_IDLE)
    {
        qDebug("CommInterface: STATE_IDLE -> stop all script traj threads");
        setMotionCommand(m_motionCommand,false);
        // usleep(2000*1000); // this does not always work, it block the update function too long
        m_mWaitCnt = 0;
    }
    else if(m_mstate==STATE_BEFORE_TRAJ_FOLLOW_WAIT)
    {
        m_mWaitCnt++;
    }
    else if(m_mstate==STATE_BEFORE_TRAJ_FOLLOW)
    {
        m_mutexJointState.lock();
        m_qStart=m_jointState.q;
        m_mutexJointState.unlock();

        m_mutex.lock();
        m_qd=m_qStart;
        m_qpd=VectorDOFd::Zero();
        m_mutex.unlock();

        m_currentCycle=0;

        qDebug("CommInterface: STATE_BEFORE_TRAJ_FOLLOW -> start script traj thread...");

        if(!setMotionCommand(m_motionCommand,true))
        {
            // exit with error;
            setFinishMState();
            m_error=true;
            m_errorString="Setting motion interface failed";
        }
    }
    else if(m_mstate == STATE_TRAJ_FOLLOW)
    {
        m_ready = true;

        if(m_motionCommand==POSITION_INTF)
        {
            m_mutex.lock();
            writeTraj(m_waypointId++,m_qd,19,3.1,COMM_PERIOD_2);
            m_mutex.unlock();
        }
        else if (m_motionCommand==VELOCITY_INTF)
        {
            //            ROS_INFO_STREAM("m_qpd: "<<m_qpd.transpose());


            m_mutex.lock();
            writeTraj(m_waypointId++,m_qpd,19,3.1,COMM_PERIOD_2);
            m_qpd.setZero();
            m_mutex.unlock();
        }
        else
        {
            m_error=true;
            m_errorString = "Invalid motion command. Not sending to robot.";
            ROS_ERROR_STREAM(m_errorString.toStdString().c_str());
        }
    }
    else if(m_mstate == STATE_FINISH)
    {
        qDebug("STATE_FINISH");
        setMotionCommand(m_motionCommand,false);
        stop();
    }
}

bool CommInterface::parseConfig(const QString &configFilePath)
{
    QFileInfo config(configFilePath);
    if (!config.exists())
    {

        m_error=false;
        m_errorString="CommInterface (): Error reading the config file. File: " + configFilePath +" does not exist!!!";
        ROS_ERROR_STREAM(m_errorString.toStdString());

        return false;
    }

    QSettings iniFile(configFilePath, QSettings::IniFormat);

    iniFile.beginGroup("MOTION_SERVER_SETTINGS");


    /* TODO: Remove PcAddress from ini file and load it from ros param server. This parameter is used in both the
     * robotArm and in the ScriptManager. It should be defiened only once and used in all the different nodes
     */


    m_pcIp=iniFile.value("PC_ADDRESS", "192.168.1.3").toString();
    m_robotIp=iniFile.value("ROBOT_ADDRESS", "192.168.1.10").toString();
    m_trajPort=iniFile.value("PC_TRAJ_PORT", 50002).toUInt();
    m_robotStatePort=iniFile.value("ROBOT_STATE_PORT", 30003).toUInt();

    QString mComm = iniFile.value("MOTION_COMMAND", "VELOCITY").toString();

    if((!mComm.compare("VELOCITY"))||(!mComm.compare("velocity"))||(!mComm.compare("vel"))||(!mComm.compare("Velocity")))
    {
        //If velocity interface is not explicitly called then use position interface (safer!)
        m_motionCommand=VELOCITY_INTF;
        ROS_WARN_STREAM("VELOCITY_INTF: "<<m_motionCommand);
    }
    else
    {
        m_motionCommand=POSITION_INTF;
    }

    //    m_motionCommand=POSITION_INTF;



    // get the robot version from the robot config; always fall back to version 1.8
    // as this is the version for tomm's arms
    QString robotVersionStr=iniFile.value("ROBOT_VERSION","1.8").toString();
    if(robotVersionStr == "1.8")
    {
        m_robotVersion = RobotStateClient::VERSION_1V8;
    }
    else if(robotVersionStr == "3.1")
    {
        m_robotVersion = RobotStateClient::VERSION_3V1;
    }
    else if(robotVersionStr == "3.4")
    {
        m_robotVersion = RobotStateClient::VERSION_3V4;
    }
    else if(robotVersionStr == "3.5")
    {
        m_robotVersion = RobotStateClient::VERSION_3V5;
    }
    else
    {
        ROS_ERROR("Unsupported robot version '%s'",robotVersionStr.toLatin1().data());
        ROS_ERROR("Fall back to default robot version 1.8");
        m_robotVersion = RobotStateClient::VERSION_1V8;
        robotVersionStr = "1.8";
    }


    ROS_INFO_STREAM("m_pcIp: "<<m_pcIp.toStdString());
    ROS_INFO_STREAM("m_robotIp: "<<m_robotIp.toStdString());
    ROS_INFO_STREAM("m_trajPort: "<<m_trajPort);
    ROS_INFO_STREAM("m_robotStatePort: "<<m_robotStatePort);
    ROS_INFO_STREAM("m_robotVersion: "<<robotVersionStr.toStdString());

    iniFile.endGroup();

    return true;
}

bool CommInterface::enableTrajScript(MotionCommand cmd, bool enable)
{
    tum_ics_ur_robot_msgs::setScriptManagerState srv;

    switch(cmd)
    {
    case POSITION_INTF:
        srv.request.name="traj_pos";
        break;

    case VELOCITY_INTF:
        srv.request.name="traj_vel";
        break;

    default:
        srv.request.name="traj_pos";
        break;
    }

    srv.request.enable = enable;

    if(!m_setScriptState.call(srv))
    {
        ROS_ERROR("CommInterface: Failed to call service 'setScriptManagerState'");

        return false;
    }

    if(!srv.response.ok)
    {
        ROS_ERROR("CommInterface: Calling service 'setScriptManagerState' failed.");
        return false;
    }

    return true;
}

bool CommInterface::setMotionCommand(MotionCommand cmd, bool enable)
{
    if(enable)
    {
        // start the server
        qDebug("Start server ...");
        m_serverTraj = new QTcpServer();
        if(!m_serverTraj->listen(QHostAddress(m_pcIp), m_trajPort))
        {
            ROS_ERROR("CommInterface: Server listen failed with error '%s'",
                      m_serverTraj->errorString().toLatin1().data());
            return false;
        }

        qDebug("Enable script thread traj ...");
        if(!enableTrajScript(cmd,true))
        {
            if(m_serverTraj->isListening())
            {
                m_serverTraj->close();
            }
            delete m_serverTraj;
            m_serverTraj = 0;

            return false;
        }

        qDebug("CommInterface: Waiting for incomming socket client connection ...");
        // the connection should now already be on the server
        if(!m_serverTraj->waitForNewConnection(100))
        {
            ROS_ERROR("CommInterface: Server has no client connection.");

            if(m_serverTraj->isListening())
            {
                m_serverTraj->close();
            }
            delete m_serverTraj;
            m_serverTraj = 0;
            return false;
        }

        qDebug("CommInterface: Get connected client socket.");
        m_socketTraj = m_serverTraj->nextPendingConnection();
        if(m_socketTraj == 0)
        {
            ROS_ERROR("CommInterface: Getting client socket failed");

            if(m_serverTraj->isListening())
            {
                m_serverTraj->close();
            }
            delete m_serverTraj;
            m_serverTraj = 0;
            return false;
        }

        qDebug("CommInterface: Traj socket successfully created.");
    }
    else
    {
        if(!enableTrajScript(cmd,false))
        {
            ROS_ERROR("CommInterface: Stopping script thread traj failed");
        }

        if(m_socketTraj != 0)
        {
            m_socketTraj->close();
            m_socketTraj = 0;
        }

        if(m_serverTraj != 0)
        {
            if(m_serverTraj->isListening())
            {
                m_serverTraj->close();
            }
            delete m_serverTraj;
            m_serverTraj = 0;
        }
    }

    return true;
}

bool CommInterface::writeTraj(
        int waypointId,
        const Tum::VectorDOFd &qd,
        double acc,
        double vel,
        double totalTime,
        double round)
{
    QByteArray data;
    QDataStream s(&data, QIODevice::ReadWrite);

    Tum::VectorDOFi qi;

    s << qint32(waypointId);
    for (int i = 0; i <= STD_DOF-1; i++)
    {
        s << qint32(qd[i]*MULT_jointstate);
        qi(i)=qint32(qd[i]*MULT_jointstate);
    }

    s << qint32(acc*MULT_jointstate)<< qint32(vel*MULT_jointstate);
    s << qint32(totalTime*MULT_time)<< qint32(round*MULT_blend);

    if(data.length() > 0)
    {
        m_socketTraj->write(data);
        if (!m_socketTraj->waitForBytesWritten())
        {
            qDebug("CommInterface: Write to the socket failed.");
            return false;
        }
    }

    return true;
}
\

}}
