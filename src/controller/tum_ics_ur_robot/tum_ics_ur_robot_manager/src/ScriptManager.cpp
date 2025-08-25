#include <tum_ics_ur_robot_manager/ScriptManager.h>

#include <unistd.h>

#include <QFile>
#include <QElapsedTimer>
#include <QHostAddress>
#include <QDataStream>

namespace tum_ics_ur_robot_manager{

QString ScriptManager::convResponse(Response r)
{
    switch (r)
    {
    case OK_RESP:
        return "OK_RESP";

    case ERROR_RESP:
        return "ERROR_RESP";

    case NO_RESP:
        return "NO_RESP";

    default:
        return "Unknown Response";
    }
}

QString ScriptManager::convCommand(Command cmd)
{
    switch (cmd)
    {
    case START_GRIPPER_CMD:
        return "START_GRIPPER_CMD";

    case STOP_GRIPPER_CMD:
        return "STOP_GRIPPER_CMD";

    case START_TRAJ_POS_CMD:
        return "START_TRAJ_POS_CMD";

    case STOP_TRAJ_POS_CMD:
        return "STOP_TRAJ_POS_CMD";

    case START_TRAJ_VEL_CMD:
        return "START_TRAJ_VEL_CMD";

    case STOP_TRAJ_VEL_CMD:
        return "STOP_TRAJ_VEL_CMD";

    default:
        return "Unknown Command";
    }
}

QTcpSocket* ScriptManager::waitForSocketConnection(
        QTcpServer* server,
        const QString pcIpAddr,
        const quint16 port,
        int timeout)
{
    if (server->listen(QHostAddress(pcIpAddr), port))
    {
        qDebug("ScriptManager::waitForSocketConnection: listening on port: %d", port);        //keep listening untill get connection
        QElapsedTimer timer;
        timer.start();
        while ((server->isListening()) && (timer.elapsed() < timeout))
        {
            if (server->waitForNewConnection(100))
            {
                qDebug("ScriptManager::buildSocketConnection: Got a TCP connection on port: %d", port);
                return server->nextPendingConnection();
            }
        }
        qDebug("ScriptManager::waitForSocketConnection(): Client connection timeout, retry ...");
    }
    else
    {
        qDebug("ScriptManager::waitForSocketConnection(): listen failed with error '%s'",
               server->errorString().toLatin1().data());
    }

    return 0;
}

ScriptManager::ScriptManager(
        const QString &pcIpAddr,
        const QString &robotIpAddr,
        quint16 managerPort) :
    m_pcIp(pcIpAddr),
    m_robotIp(robotIpAddr),
    m_managerPort(managerPort),
    m_server(0),
    m_socket(0),
    m_started(false)
{

}

ScriptManager::~ScriptManager()
{
    stop();
}

bool ScriptManager::start()
{
    if(m_started)
    {
        return false;
    }

    m_server = new QTcpServer();
    m_socket = waitForSocketConnection(m_server,m_pcIp,m_managerPort);
    if(m_socket == 0)
    {
        if(m_server->isListening())
        {
            m_server->close();
        }
        delete m_server;
        m_server = 0;
        return false;
    }

    m_started = true;
    return true;
}

void ScriptManager::stop()
{
    if(!m_started)
    {
        return;
    }

    send(QUIT_CMD);

    m_socket->close();

    if(m_server->isListening())
    {
        m_server->close();
    }

    delete m_server;
    m_started = false;
    m_server = 0;
}

bool ScriptManager::send(Command cmd, Response* resp, int timeout)
{
    QByteArray data;
    QDataStream s(&data, QIODevice::ReadWrite);
    s << qint32(cmd);

    // clear rx buffer
    m_socket->waitForReadyRead(0);
    m_socket->readAll();

    m_socket->write(data);
    if(!m_socket->waitForBytesWritten())
    {
        qDebug() << "Server - write to the socket failed";
        return false;
    }

    if(resp != 0)
    {
        *resp = NO_RESP;
    }

    if(timeout > 0)
    {
        waitForResponse(resp,timeout);
    }

    return true;
}

bool ScriptManager::waitForResponse(Response* resp, int timeout)
{
    QElapsedTimer timer;
    QByteArray buf;
    timer.start();

    while(timer.elapsed() < timeout)
    {
        m_socket->waitForReadyRead(0);
        if(m_socket->bytesAvailable() <= 0)
        {
            continue;
        }
        buf.append(m_socket->readAll());

        if(buf.size() >= sizeof(qint32))
        {
            qint32 response;
            QDataStream ds(&buf,QIODevice::ReadOnly);
            ds.setByteOrder(QDataStream::BigEndian);
            ds >> response;
            if(resp != 0)
            {
                *resp = (Response)response;
            }
            return true;
        }
    }

    if(resp != 0)
    {
        *resp = NO_RESP;
    }

    qDebug("ScriptManager: Response timeout.");
    return false;
}

}
