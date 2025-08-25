#ifndef UR_ROBOT_MANAGER_SCRIPTLOADER_H
#define UR_ROBOT_MANAGER_SCRIPTLOADER_H

#include <QVector>
#include <QString>

#include <QTcpSocket>

namespace tum_ics_ur_robot_manager{

class ScriptLoader :
        QTcpSocket
{
public:

private:
    static const unsigned int CONNECT_TIME_OUT = 1*1000; // 1 second
    static const unsigned int TOTAL_CONNECT_TIME_OUT = 3*1000; // 5 seconds

private:
    QString m_robotIpAddr;
    quint16 m_port;

    QString m_errorString;
    bool m_error;

public:
    ScriptLoader(
            const QString &robotIpAddr,
            quint16 scriptLoaderPort=30001);
    ~ScriptLoader();

    // blocks until timeout or success
    bool load(const QString& scriptFilePath);

    //over load for fixed script
    bool load();

    bool error();
    const QString& errorString();

private:
    QString readFromFile(const QString& filePath);
    void writeString(const QString &s);
};

}

#endif // UR_ROBOT_MANAGER_SCRIPTLOADER_H
