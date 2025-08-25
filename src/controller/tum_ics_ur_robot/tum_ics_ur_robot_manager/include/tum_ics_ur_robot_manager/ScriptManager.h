#ifndef UR_ROBOT_MANAGER_SCRIPTMANAGER_H
#define UR_ROBOT_MANAGER_SCRIPTMANAGER_H

#include <QVector>
#include <QString>

#include <QTcpSocket>
#include <QTcpServer>

namespace tum_ics_ur_robot_manager{

class ScriptManager
{
public:
    enum Command
    {
        START_GRIPPER_CMD   = 0,
        STOP_GRIPPER_CMD    = 1,
        START_TRAJ_POS_CMD  = 2,
        STOP_TRAJ_POS_CMD   = 3,
        START_TRAJ_VEL_CMD  = 4,
        STOP_TRAJ_VEL_CMD   = 5,
        QUIT_CMD            = 6
    };

    enum Response
    {
        NO_RESP      = 0,
        OK_RESP      = 1,
        ERROR_RESP   = 2
    };

    static QString convResponse(Response r);
    static QString convCommand(Command cmd);

private:
    // timeout in ms
    static QTcpSocket* waitForSocketConnection(
            QTcpServer* server,
            const QString pcIpAddr,
            const quint16 port,
            int timeout = 5000);


private:
    QTcpServer* m_server;
    QTcpSocket* m_socket;

    QByteArray m_rxBuf;
    Response m_response;

    QString m_pcIp;
    QString m_robotIp;
    quint16 m_managerPort;

protected:
    bool m_started;

public:
    ScriptManager(
            const QString &pcIpAddr,
            const QString &robotIpAddr,
            quint16 managerPort = 50001);

    ~ScriptManager();

    bool start();
    void stop();

    // timeout = -1: don't wait for response
    bool send(Command cmd, Response* resp=0, int timeout=-1); //ms

private:
    bool waitForResponse(Response* resp, int timeout); // ms
};

}

#endif // UR_ROBOT_MANAGER_SCRIPTMANAGER_H
