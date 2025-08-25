#ifndef COMMINTERFACE_H
#define COMMINTERFACE_H

#include <QThread>
#include <QTcpServer>
#include <QMutex>

#include <tum_ics_ur_robot_lli/RobotInterface/TcpSocketTools.h>
#include <tum_ics_ur_robot_lli/RobotInterface/RobotStateClient.h>

#include <tum_ics_ur_robot_msgs/setScriptManagerState.h>
#include <tum_ics_ur_robot_msgs/getScriptManagerStates.h>

#include <RtThreads/Thread.h>

namespace tum_ics_ur_robot_lli {
namespace RobotInterface{

typedef RtThreads::Thread TThread;
//typedef QThread TThread;

class CommInterface: TThread
{
    // Member variables
public:
    enum State
    {
        STATE_INIT=0,
        STATE_IDLE,
        STATE_BEFORE_TRAJ_FOLLOW_WAIT,
        STATE_BEFORE_TRAJ_FOLLOW,
        STATE_TRAJ_FOLLOW,
        STATE_FINISH
    };

    enum MotionCommand
    {
        POSITION_INTF=1,
        VELOCITY_INTF
    };

    // static constexpr double COMM_PERIOD = 0.008;
    // static constexpr double COMM_PERIOD_2 = 0.004;
    // static constexpr double MULT_jointstate = 1.0E8;//10000.0;
    // static constexpr double MULT_time = 1000000.0;
    // static constexpr double MULT_blend = 1000.0;

    static const double COMM_PERIOD;
    static const double COMM_PERIOD_2;
    static const double MULT_jointstate;
    static const double MULT_time;
    static const double MULT_blend;

    // Member variables
private:
    ros::NodeHandle     m_node;
    ros::ServiceClient  m_setScriptState;

    QTcpServer* m_serverTraj;
    QTcpSocket* m_socketTraj;

    //IP Addresses
    QString m_pcIp;
    QString m_robotIp;

    //Ports
    quint16 m_trajPort;//   e.g. 50002 To send joint desired state (these two ports are defined inside the script file)
    quint16 m_robotStatePort; // 30003 get robot state from real time module on robot

    // robot version for robot state client
    RobotStateClient::Version m_robotVersion;

    JointState m_jointState;
    Tum::VectorDOFd m_qStart;
    Tum::VectorDOFd m_qd; //Desired joint position for the UR controller
    Tum::VectorDOFd m_qpd; //Desired joint velocity for the UR controller
    int m_currentCycle;
    int m_waypointId;

    //Error handlers
    QString m_errorString;
    bool m_error;
    bool m_stopThread;
    bool m_stoppedThread;
    bool m_ready;

    int m_cnt;

    //State MachineState
    State m_mstate;
    State m_pend_mstate;
    bool m_mstatePendFlag;

    int m_mWaitCnt;

    QMutex m_mutex;

    QMutex m_mutexJointState;

    RobotStateClient *m_robotStateClient;
    bool m_extRobotSateClientFlag;

    MotionCommand m_motionCommand;

    // Member functions
public:
    CommInterface(const QString &pcIpAddr,
                  const quint16 trajPort,
                  const QString &robotIpAddr,
                  const quint16 robotStatePort=30003,
                  QObject *parent=0);

    // Added option to link an external robotStateClient
    CommInterface(const QString &configFilePath,
                  RobotStateClient *extRobotStateClient=NULL);


    ~CommInterface();

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

private:
    bool init();
    void run();
    void stop(); //Stop thread and close sockets and ports (on PC side only)
    void cleanup();

    void updateStateMachine();
    void executeMState();

    bool parseConfig(const QString &configFilePath);

    // start and stop the thread in the robot script
    bool enableTrajScript(MotionCommand cmd, bool enable);

    // starts / stops the motion interface and the server
    // never start both and never start before stop
    bool setMotionCommand(MotionCommand cmd, bool enable);

    // send pos / vel to robot
    bool writeTraj(
            int waypointId,
            const Tum::VectorDOFd &qd,
            double acc=3.0,
            double vel=0.75,
            double totalTime=0.0,
            double round=0.0);
};

}}

#endif // COMMINTERFACE_H
