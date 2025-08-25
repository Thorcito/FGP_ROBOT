#ifndef DATAUNPACKER_H
#define DATAUNPACKER_H

#include<QString>
#include<tum_ics_ur_robot_lli/JointState.h>



namespace tum_ics_ur_robot_lli {
namespace RobotInterface{

class DataUnpacker
{

public:
    enum MsgHeader
    {
        MSG_OUT = 1,				//robot side: string message
        MSG_QUIT = 2, 				//robot side: quit flag
        MSG_JOINT_STATES = 3, 		//robot side: info of joint states
        MSG_MOVEJ = 4,              //computer side: command of movej
        MSG_WAYPOINT_FINISHED = 5, 	//robot side
        MSG_STOPJ = 6,              //computer side: stop
        MSG_SERVOJ = 7, 			//TODO to check the ros package driver.py
        MSG_PARK_ROBOT = 8,
        MSG_GO_HOME = 9,
        MSG_ENTER_TRAJ_FOLLOW = 10,
        MSG_EXIT_TRAJ_FOLLOW = 11
    };
    // static constexpr double MULT_jointstate = 1.0E8;//10000.0;
    // static constexpr double MULT_time = 1000000.0;
    // static constexpr double MULT_blend = 1000.0;
    // static constexpr double COMM_PERIOD = 0.008;
    // static constexpr double COMM_PERIOD_2 = 0.004;

    static const double COMM_PERIOD;
    static const double COMM_PERIOD_2;
    static const double MULT_jointstate;
    static const double MULT_time;
    static const double MULT_blend;

private:

    QByteArray pendingData;
    int wayPointId;
    double q[6];
    double qdot[6];
    double tau[6];
    int q_raw[6];
    int qdot_raw[6];
    int tau_raw[6];
    QString msg;

    bool m_ready;
    //QMutex mutex_config;
    //QMutex mutex_msg;

public:
    DataUnpacker();
    void appendNewData(QByteArray& newData);
    void unpackData();
    void getJointStates(double q[6], double qdot[6], double tau[6]);
    JointState getJointState() const;
    bool hasJointStatesInfo();
    QString getMsg();
    int getWayPointId();
    bool isReady() const;


};
}
}

#endif // DATAUNPACKER_H
