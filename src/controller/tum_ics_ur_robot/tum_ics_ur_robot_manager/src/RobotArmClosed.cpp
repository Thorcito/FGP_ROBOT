#include<tum_ics_ur_robot_manager/RobotArmClosed.h>
#include<QSettings>
#include<QFileInfo>

namespace tum_ics_ur_robot_manager{


RobotArmClosed::RobotArmClosed(const QString &configFilePath, QString &pcIppAdd):
    RobotArm_(configFilePath),
    m_scriptManager(new RobotScriptManager(pcIppAdd,"192.168.1.10")),
    m_realRobotArmClosed(false)
{
    QFileInfo config(configFilePath);
    if (!config.exists())
    {


        ROS_ERROR_STREAM("RobotArmClosed(): Error reading the config file. File: "<<configFilePath.toStdString()<<" does not exist!!!");

        exit (-1);
    }

    QSettings iniFile(configFilePath, QSettings::IniFormat);

    iniFile.beginGroup("ROBOT_EXECUTION_PARAMETERS");



    QString robotType=iniFile.value("ROBOT_TYPE", "sim").toString();
    if(!(robotType.compare("real")) || !(robotType.compare("REAL")) )
    {
        m_realRobotArmClosed=true;
    }
    else
    {
        m_realRobotArmClosed=false;
    }


}

RobotArmClosed::~RobotArmClosed()
{

}

bool RobotArmClosed::connect()
{
    if(m_realRobotArmClosed)
    {
        return m_scriptManager->start();
    }
    else
    {
        ROS_INFO_STREAM("RobotArmClosed(): Simulated ROBOT");
        return true;
    }

}

bool RobotArmClosed::send(const QString &name, bool enable)
{
    return m_scriptManager->send(name,enable);
}



}



