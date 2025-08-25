#ifndef SCRIPTLOADER_H
#define SCRIPTLOADER_H

#include<QString>
#include<QTcpSocket>

#include<tum_ics_ur_robot_lli/RobotInterface/TcpSocketTools.h>

namespace tum_ics_ur_robot_lli{
namespace RobotInterface{

class ScriptLoader: TcpSocketTools, QTcpSocket
{

private:
    QString m_errorString;
    bool m_error;
    static const unsigned int CONNECT_TIME_OUT = 1*1000; // 5 seconds
    static const unsigned int TOTAL_CONNECT_TIME_OUT = 5 * 1000;
    bool m_stop;

public:
    ScriptLoader(const QString &robotIpAddr, const QString &scriptFileName="../scripts/prog", quint16 robotPort=30001);
    ~ScriptLoader();
    bool error();
    const QString& errorString();
    void stop();

private:
    QString readFromFile(const QString& path);



};



}
}



#endif // SCRIPTLOADER_H
