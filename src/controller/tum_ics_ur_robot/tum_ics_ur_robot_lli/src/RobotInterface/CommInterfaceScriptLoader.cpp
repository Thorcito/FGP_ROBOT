#include<ros/ros.h>

#include<ur_robot_lli/RobotInterface/CommInterfaceScriptLoader.h>

#include<QElapsedTimer>
#include<QApplication>
#include<QFileInfo>
#include<QSettings>
#include<unistd.h>



namespace ur_robot_lli{
namespace RobotInterface{

CommInterfaceScriptLoader::CommInterfaceScriptLoader(const QString &pcIpAddr, const quint16 cmdPort, const quint16 trajPort, const QString &robotIpAddr, const QString &scriptFilePath, const quint16 robotScriptPort, const quint16 robotStatePort, QObject *parent):
    TThread(RtThreads::Thread::RtThreadMode,"CommInterfaceScriptLoader"),
    //m_server(this),
    m_pcIp(pcIpAddr),
    m_cmdPort(cmdPort),
    m_trajPort(trajPort),
    m_robotIp(robotIpAddr),
    m_scriptFilePath(scriptFilePath),
    m_robotScriptPort(robotScriptPort),
    m_robotStatePort(robotStatePort),
    m_serverTraj(NULL),
    m_serverCmd(NULL),
    m_socketCmd(NULL),
    m_socketTraj(NULL),
    m_error(false),
    m_stopThread(false),
    m_stoppedThread(true),
    m_mstate(STATE_INIT),
    m_socketCmdTools(NULL),
    m_socketTrajTools(NULL),
    m_qStart(VectorDOFd::Zero()),
    m_qd(VectorDOFd::Zero()),
    m_qpd(VectorDOFd::Zero()),
    m_currentCycle(0),
    m_waypointId(0),
    m_robotStateClient(NULL),
    m_ready(false),
    m_scriptLoader(NULL),
    m_motionCommand(POSITION_INTF)
{
    //    QThread::start();
    TThread::start();
}

CommInterfaceScriptLoader::CommInterfaceScriptLoader(const QString &configFilePath):
    m_serverTraj(NULL),
    m_serverCmd(NULL),
    m_socketCmd(NULL),
    m_socketTraj(NULL),
    m_error(false),
    m_stopThread(false),
    m_stoppedThread(true),
    m_mstate(STATE_INIT),
    m_socketCmdTools(NULL),
    m_socketTrajTools(NULL),
    m_qStart(VectorDOFd::Zero()),
    m_qd(VectorDOFd::Zero()),
    m_qpd(VectorDOFd::Zero()),
    m_currentCycle(0),
    m_waypointId(0),
    m_robotStateClient(NULL),
    m_ready(false),
    m_scriptLoader(NULL),
    m_motionCommand(POSITION_INTF)
{

    if(!parseConfig(configFilePath))
    {
        return;
    }

    //    QThread::start();
    TThread::start();

}



CommInterfaceScriptLoader::~CommInterfaceScriptLoader()
{
    //stop();



    if(m_scriptLoader!=NULL)
    {

        m_scriptLoader->stop();
        ::usleep(2*1E6);

    }


    setFinishMState();


    QElapsedTimer timer;

    timer.start();

    //Wait until stoppedThread changes or the limit time has been reached (5secs)
    while((!m_stoppedThread)&&(timer.elapsed()<5000))
    {
        ::usleep(5*1E5);
    }

    if(m_stoppedThread)
    {

        cleanup();
        return;
    }

    //if the thread has not been stopped
    if(!TThread::wait(500))
    {
        TThread::terminate();
        TThread::wait();
        qDebug()<<"CommInterfaceScriptLoader::~CommInterfaceScriptLoader: Thread forced to end!!!";
    }

    cleanup();

}

bool CommInterfaceScriptLoader::error()
{
    return m_error;
}
const QString& CommInterfaceScriptLoader::errorString()
{
    return m_errorString;
}



JointState CommInterfaceScriptLoader::getCurrentJointState()
{
    JointState jS;
    m_mutexJointState.lock();

    jS=m_jointState;
    m_mutexJointState.unlock();

    return jS;
}
void CommInterfaceScriptLoader::setQd(const VectorDOFd &qTarget, const VectorDOFd &qpTarget)
{
    m_mutex.lock();
    m_qd=qTarget;
    m_qpd=qpTarget;
    m_mutex.unlock();
}

void CommInterfaceScriptLoader::setIdleMState()
{
    m_mutex.lock();
    if(m_mstate==STATE_TRAJ_FOLLOW)
        m_mstate=STATE_IDLE;
    m_mutex.unlock();

}
void CommInterfaceScriptLoader::setFinishMState()
{
    m_mutex.lock();
    m_mstate=STATE_FINISH;
    //qDebug()<<"setFinish(): m_mstate: "<<m_mstate;
    m_mutex.unlock();

}
CommInterfaceScriptLoader::State CommInterfaceScriptLoader::getMState()
{
    State s;
    m_mutex.lock();

    s=m_mstate;

    m_mutex.unlock();
    return s;
}

bool CommInterfaceScriptLoader::robotStateIsReady() const
{
    return m_robotStateClient->isReady();
}

bool CommInterfaceScriptLoader::isReady() const
{
    return m_ready;
}

const CommInterfaceScriptLoader::MotionCommand CommInterfaceScriptLoader::getMotionIntf() const
{
    return m_motionCommand;
}
void CommInterfaceScriptLoader::setMotionIntf(const MotionCommand intf)
{
    m_motionCommand=intf;
}


bool CommInterfaceScriptLoader::init()
{
    // This function requires that the ur script is already loaded in the controller
    // This class connects to the ur controller server and loads the script
    m_scriptLoader = new ScriptLoader(m_robotIp,m_scriptFilePath,m_robotScriptPort);

    if(m_scriptLoader->error())
    {
        m_error=true;
        m_errorString="CommInterfaceScriptLoader::init(): Scriptloader: " + m_scriptLoader->errorString();
        qDebug()<<m_errorString;

        return false;
    }
    delete m_scriptLoader;
    m_scriptLoader=NULL;

    //Start the client to get the robot state from port 30003
    m_robotStateClient=new RobotStateClient(m_robotIp,m_robotStatePort);
    if(m_robotStateClient->error())
    {
        m_error=true;
        m_errorString="CommInterfaceScriptLoader::init(): RobotStateClient: " + m_robotStateClient->errorString();
        qDebug()<<m_errorString;
        return false;
    }


    m_serverCmd=new QTcpServer();

    //open ports and waits for connection (blocking function)
    m_socketCmd = buildSocketConnection(m_serverCmd, m_cmdPort);
    if(m_socketCmd == NULL)
    {
        m_error=true;
        m_errorString="CommInterfaceScriptLoader::init(): falied to build socket connection with command port";
        qDebug()<<m_errorString;

        if(m_serverCmd->isListening())
        {
            m_serverCmd->close();
        }

        return false;
    }

    m_serverTraj=new QTcpServer();
    m_socketTraj = buildSocketConnection(m_serverTraj, m_trajPort);
    if(m_socketTraj == NULL)
    {
        m_error=true;
        m_errorString="CommInterfaceScriptLoader::init(): falied to build socket connection with traj port";
        qDebug()<<m_errorString;

        //Close the previously opened socket (cmd port)
        m_socketCmd->close();

        if(m_serverCmd->isListening())
        {
            m_serverCmd->close();
        }


        if(m_serverTraj->isListening())
        {
            m_serverTraj->close();
        }

        return false;
    }

    //Add the socket information to the tool class
    m_socketCmdTools=new TcpSocketTools(m_socketCmd);
    m_socketTrajTools=new TcpSocketTools(m_socketTraj);

    return true;
}

void CommInterfaceScriptLoader::run()
{

    m_stoppedThread=false;

    if(!init())
    {
        m_stoppedThread=true;
        qDebug()<<"CommInterfaceScriptLoader::run(): !init-->Thread exited";
        return;
    }



    while(!m_stopThread)
    {
        QElapsedTimer tic;

        tic.start();

        //        QApplication::processEvents();

        ///TODO: CHECK ALL THE TCPSOCKETS AND DO AS FOLLOWS!!!

        //http://www.qtforum.org/article/27867/qtcpsocket-without-event-loop.html

        m_robotStateClient->update();


        //Try to read the current JointState from UR controller
        m_mutexJointState.lock();
        //        m_jointState=processRxRobotData();
        m_jointState=m_robotStateClient->getJointState();
        m_mutexJointState.unlock();

        //qDebug()<<"Joints: "<<m_jointState.toString();

        //calculate the next state of the state machine
        updateStateMachine();

        //executes the correspondign functions to the current state of the state machine
        executeMState();



        double elapsedTime=tic.nsecsElapsed()*1E-9;
        double delay = (DataUnpacker::COMM_PERIOD-elapsedTime)*1E6;

        //        ROS_INFO_STREAM("E time: "<<elapsedTime);

        if(elapsedTime>DataUnpacker::COMM_PERIOD)
        {
            ROS_WARN_STREAM("Communication loop period exceeds the limit: "<<elapsedTime);
            continue;
        }

        if(TThread::isRtThread())
        {
            TThread::usleep(DataUnpacker::COMM_PERIOD*1e6);
        }
        else
        {
            TThread::usleep((unsigned long)delay);
        }

    }
    m_socketCmd->close();
    m_socketTraj->close();

    if(m_serverCmd->isListening())
    {
        m_serverCmd->close();
    }
    if(m_serverTraj->isListening())
    {
        m_serverTraj->close();
    }

    m_stoppedThread=true;
    qDebug()<<"CommInterfaceScriptLoader::run(): Thread exited";


}

void CommInterfaceScriptLoader::stop()
{

    if(m_stoppedThread)
    {
        return;
    }
    m_stopThread=true;

    qDebug()<<"CommInterfaceScriptLoader::stop(): m_stopThread="<<m_stopThread;

}

void CommInterfaceScriptLoader::cleanup()
{


    //delete memory
    if(m_scriptLoader!=NULL)
    {

        delete m_scriptLoader;
        m_scriptLoader=NULL;
    }

    if(m_socketCmdTools!=NULL)
    {

        delete m_socketCmdTools;
        m_socketCmdTools=NULL;
    }

    if(m_socketTrajTools!=NULL)
    {

        delete m_socketTrajTools;
        m_socketTrajTools=NULL;
    }


    if(m_socketCmd!=NULL)
    {

        delete m_socketCmd;
        m_socketCmd=NULL;
    }

    if(m_socketTraj!=NULL)
    {

        delete m_socketTraj;
        m_socketTraj=NULL;
    }

    if(m_serverCmd!=NULL)
    {

        delete m_serverCmd;
        m_serverCmd=NULL;
    }

    if(m_serverTraj!=NULL)
    {

        delete m_serverTraj;
        m_serverTraj=NULL;
    }

    if(m_robotStateClient!=NULL)
    {

        delete m_robotStateClient;
        m_robotStateClient=NULL;
    }


}

QTcpSocket* CommInterfaceScriptLoader::buildSocketConnection(QTcpServer* srv, const quint16 port)
{

    if (srv->listen(QHostAddress(m_pcIp), port))
    {
        qDebug()<<"ur_robot_lli::buildSocketConnection: listening on port: " << port;
        //keep listening untill get connection
        QElapsedTimer timer;
        timer.start();
        while ((srv->isListening())&&(timer.elapsed()<3000))
        {

            if (srv->waitForNewConnection(100))
            {
                qDebug()<<"ur_robot_lli::buildSocketConnection: got a TCP connection on port:"<<port;
                return srv->nextPendingConnection();
            }
        }
        qDebug()<<"ur_robot_lli::CommInterfaceScriptLoader::buildSocketConnection(): Client connection timed out";
    }
    else
    {
        qDebug()<<"ur_robot_lli::buildSocketConnection: listen operation failed\n";
        qDebug()<< srv->errorString();
    }

    return NULL;
}

JointState CommInterfaceScriptLoader::processRxRobotData()
{
    //qDebug()<<"N Bytes socketCmd: "<<m_socketCmd->bytesAvailable();
    QByteArray newData = m_socketCmdTools->readByteArrayAll();

    //qDebug()<<"newData: "<<newData.size();


    m_dataUnpacker.appendNewData(newData); //Copy packet to a local variable
    m_dataUnpacker.unpackData();    //process packet, update q,qp and tau if the packet is complete

    // to avoid increasing the buffer when incomplete packets arrrive
    m_dataUnpacker.unpackData();

    //Read data from server (lock the variable while saving)
    //QMutexLocker ml(&mutex_q);
    //    m_dataUnpacker.getJointStates(q, qdot, tau);

    return m_dataUnpacker.getJointState(); //uses q, qp, tau to generate JointState

    //latestFinishedWayPointId = m_dataUnpacker.getWayPointId();
}

void CommInterfaceScriptLoader::updateStateMachine()
{
    //Get the JointState from the controller (port 50001)

    if(m_mstate == STATE_INIT)
    {
        qDebug() << "STATE_INIT";
        ROS_INFO_STREAM(__FILE__<<": "<<__LINE__<<": ready: "<<m_robotStateClient->isReady());
        //        if(m_dataUnpacker.isReady())
        if(m_robotStateClient->isReady())
        {
            ROS_INFO_STREAM(__FILE__<<": "<<__LINE__);
            m_mutex.lock();
            m_mstate=STATE_IDLE;
            m_mutex.unlock();
        }
    }
    else if(m_mstate == STATE_IDLE)
    {
        qDebug() << "STATE_IDLE";
        m_mutex.lock();
        m_mstate = STATE_BEFORE_TRAJ_FOLLOW;
        m_mutex.unlock();
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


        m_mutex.lock();
        m_mstate = STATE_TRAJ_FOLLOW;
        m_mutex.unlock();
    }
    else if(m_mstate == STATE_TRAJ_FOLLOW)
    {
        //t_final is the totalTime for the trajectory (defined by the user)
        // The robot instance change the state to STATE_FINISH (with UR_Robot->stopThread())

        //                if(t_current >= t_final)
        //                {
        //                    state = STATE_FINISH;

        //                }

        //Wait for task finished flag and change to EXIT either P or V
    }
    else if(m_mstate == STATE_FINISH)
    {

        qDebug()<<QString("STATE_FINISH");

        //set the flag to terminate robot class
        //        this->serverEnded=true;
    }
}

void CommInterfaceScriptLoader::executeMState()
{
    if(m_state!=STATE_TRAJ_FOLLOW)
    {
        m_ready=false;
    }
    if(m_mstate==STATE_INIT)
    {

    }
    else if(m_mstate==STATE_IDLE)
    {
        //call exit both pos and vel

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

        //Change the state in the controller to initialize the thread to read the desired joint position
        if(m_motionCommand==POSITION_INTF)
        {
            m_socketCmdTools->writeCommand(TcpSocketTools::MSG_ENTER_TRAJ_FOLLOW);
        }
        else if (m_motionCommand==VELOCITY_INTF)
        {
            m_socketCmdTools->writeCommand(TcpSocketTools::MSG_ENTER_TRAJ_FOLLOW_V);
        }
        else
        {
            std::stringstream s;

            s<<"CommInterfaceScriptLoader::executeSM() SBTF: Motion Interface not defined --"<<m_motionCommand<<"--";

            m_error=true;
            m_errorString=s.str().c_str();
            //qDebug()<<m_errorString;
            ROS_ERROR_STREAM(m_errorString.toStdString().c_str());
        }
    }
    else if(m_mstate == STATE_TRAJ_FOLLOW)
    {
        m_ready=true;

        if(m_motionCommand==POSITION_INTF)
        {
            m_mutex.lock();
            //        m_socketTrajTools->writeTraj(m_waypointId++,m_qd,3,0.75,DataUnpacker::COMM_PERIOD);
            //ROS_ERROR_STREAM(__FILE__<<": "<<m_qd.transpose());
            m_socketTrajTools->writeTraj(m_waypointId++,m_qd,19,3.1,DataUnpacker::COMM_PERIOD_2);
            m_mutex.unlock();
        }
        else if (m_motionCommand==VELOCITY_INTF)
        {
            m_mutex.lock();
            //        m_socketTrajTools->writeTraj(m_waypointId++,m_qd,3,0.75,DataUnpacker::COMM_PERIOD);
            m_socketTrajTools->writeTraj(m_waypointId++,m_qpd,19,3.1,DataUnpacker::COMM_PERIOD_2);
            m_mutex.unlock();
        }
        else
        {
            std::stringstream s;

            s<<"CommInterfaceScriptLoader::executeSM() STF: Motion Interface not defined --"<<m_motionCommand<<"--";

            m_error=true;
            m_errorString=s.str().c_str();
            //qDebug()<<m_errorString;
            ROS_ERROR_STREAM(m_errorString.toStdString().c_str());
        }


    }
    else if(m_mstate == STATE_FINISH)
    {
        m_socketCmdTools->writeCommand(TcpSocketTools::MSG_QUIT);

        //Stop thread and SM (closes sockets and clean shutdown)
        stop();
    }
    //    else if(m_mstate == EXITP)
    //    {
    //        m_socketCmdTools->writeCommand(TcpSocketTools::MSG_EXIT_TRAJ_FOLLOW);

    //        m_mstate=
    //        //Stop thread and SM (closes sockets and clean shutdown)
    //        stop();
    //    }
}

bool CommInterfaceScriptLoader::parseConfig(const QString &configFilePath)
{
    QFileInfo config(configFilePath);
    if (!config.exists())
    {

        m_error=false;
        m_errorString="CommInterfaceScriptLoader (): Error reading the config file. File: " + configFilePath +" does not exist!!!";
        ROS_ERROR_STREAM(m_errorString.toStdString());

        return false;
    }

    QSettings iniFile(configFilePath, QSettings::IniFormat);

    iniFile.beginGroup("MOTION_SERVER_SETTINGS");


    m_pcIp=iniFile.value("PC_ADDRESS", "192.168.1.10").toString();
    m_robotIp=iniFile.value("ROBOT_ADDRESS", "192.168.1.5").toString();
    m_scriptFilePath=iniFile.value("UR_SCRIPT", "none").toString();

    m_cmdPort=iniFile.value("PC_CMD_PORT", 50001).toUInt();
    m_trajPort=iniFile.value("PC_TRAJ_PORT", 50002).toUInt();
    m_robotScriptPort=iniFile.value("ROBOT_SCRIPT_PORT", 30001).toUInt();
    m_robotStatePort=iniFile.value("ROBOT_STATE_PORT", 30003).toUInt();

    QString mComm = iniFile.value("MOTION_COMMAND", "POSITION").toString();

    if((!mComm.compare("VELOCITY"))||(!mComm.compare("velocity"))||(!mComm.compare("vel"))||(!mComm.compare("Velocity")))
    {
        //If velocity interface is not explicitly called then use position interface (safer!)
        m_motionCommand=VELOCITY_INTF;
    }
    else
    {
        m_motionCommand=POSITION_INTF;
    }


    ROS_INFO_STREAM("m_pcIp: "<<m_pcIp.toStdString());
    ROS_INFO_STREAM("m_robotIp: "<<m_robotIp.toStdString());
    ROS_INFO_STREAM("m_scriptFilePath: "<<m_scriptFilePath.toStdString());

    ROS_INFO_STREAM("m_cmdPort: "<<m_cmdPort);
    ROS_INFO_STREAM("m_trajPort: "<<m_trajPort);
    ROS_INFO_STREAM("m_robotScriptPort: "<<m_robotScriptPort);
    ROS_INFO_STREAM("m_robotStatePort: "<<m_robotStatePort);

    iniFile.endGroup();

    return true;
}



}
}
