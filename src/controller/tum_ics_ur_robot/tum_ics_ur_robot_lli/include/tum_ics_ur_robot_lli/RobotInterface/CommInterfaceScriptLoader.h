#ifndef COMMINTERFACESCRIPTLOADER_H
#define COMMINTERFACESCRIPTLOADER_H

#include<QThread>
#include<QTcpServer>
#include<QMutex>

#include<tum_ics_ur_robot_lli/RobotInterface/TcpSocketTools.h>
#include<tum_ics_ur_robot_lli/RobotInterface/DataUnpacker.h>
#include<tum_ics_ur_robot_lli/RobotInterface/RobotStateClient.h>
#include<tum_ics_ur_robot_lli/RobotInterface/ScriptLoader.h>
#include <RtThreads/Thread.h>

namespace tum_ics_ur_robot_lli {
namespace RobotInterface{

typedef RtThreads::Thread TThread;
//typedef QThread TThread;

class CommInterfaceScriptLoader: TThread/*QThread*/
{
    // Member variables
public:
    enum State
    {
        STATE_INIT,
        STATE_IDLE,
        STATE_BEFORE_TRAJ_FOLLOW,
        STATE_TRAJ_FOLLOW,
        STATE_FINISH
    };

    enum MotionCommand
    {
        POSITION_INTF=1,
        VELOCITY_INTF
    };

    // Member variables
private:
    QTcpServer* m_serverCmd;
    QTcpServer* m_serverTraj;

    QTcpSocket* m_socketCmd;
    QTcpSocket* m_socketTraj;

    TcpSocketTools* m_socketCmdTools;
    TcpSocketTools* m_socketTrajTools;

    DataUnpacker m_dataUnpacker;

    ScriptLoader *m_scriptLoader;


    State m_state;
    //IP Addresses
    QString m_pcIp;
    QString m_robotIp;

    //Ports
    quint16 m_cmdPort; //   e.g. 50001 To send commands and get robot states
    quint16 m_trajPort;//   e.g. 50002 To send joint desired state (these two ports are defined inside the script file)
    quint16 m_robotScriptPort; //30001 to load the script into the robot control unit (this port is fixed in the robot control unit)
    quint16 m_robotStatePort;

    //Script
    QString m_scriptFilePath;
    JointState m_jointState;
    VectorDOFd m_qStart;
    VectorDOFd m_qd; //Desired joint position for the UR controller
    VectorDOFd m_qpd; //Desired joint velocity for the UR controller
    int m_currentCycle;
    int m_waypointId;

    //Error handlers
    QString m_errorString;
    bool m_error;
    bool m_stopThread;
    bool m_stoppedThread;
    bool m_ready;

    //State MachineState
    State m_mstate;

    QMutex m_mutex;

    QMutex m_mutexJointState;

    RobotStateClient *m_robotStateClient;

    MotionCommand m_motionCommand;





    // Member functions
public:
    CommInterfaceScriptLoader(const QString &pcIpAddr,
                  const quint16 cmdPort,
                  const quint16 trajPort,
                  const QString &robotIpAddr,
                  const QString &scriptFilePath,
                  const quint16 robotScriptPort=30001,
                  const quint16 robotStatePort=30003,
                  QObject *parent=0);

    CommInterfaceScriptLoader(const QString &configFilePath);
    ~CommInterfaceScriptLoader();

    bool error();
    const QString& errorString();

    JointState getCurrentJointState();
    void setQd(const VectorDOFd &qTarget, const VectorDOFd &qpTarget=VectorDOFd::Zero());

    void setIdleMState();
    void setFinishMState();
    State getMState();

    bool robotStateIsReady() const;
    bool isReady() const;

    const MotionCommand getMotionIntf() const;
    void setMotionIntf(const MotionCommand intf);





    // Member functions
private:
    bool init();
    void run();
    void stop(); //Stop thread and close sockets and ports (on PC side only)
    void cleanup();

    QTcpSocket* buildSocketConnection(QTcpServer *srv, const quint16 port);
    JointState processRxRobotData();

    void updateStateMachine();
    void executeMState();

    void sendJointStateDesired();
    bool parseConfig(const QString &configFilePath);





};

}
}

#endif // COMMINTERFACESCRIPTLOADER_H
