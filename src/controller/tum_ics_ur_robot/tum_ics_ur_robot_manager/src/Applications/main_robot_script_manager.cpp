#include <QApplication>
#include <QDebug>
#include <QDir>

#include <ros/ros.h>
#include <tum_ics_ur_robot_manager/RobotScriptManager.h>

#include "../ConsoleReader.h"

using namespace tum_ics_ur_robot_manager;

int main(int argc, char **argv)
{
    ros::init(argc, argv,"script_loader",ros::init_options::AnonymousName);
    QApplication app(argc, argv);

    ros::NodeHandle n;
    ros::Rate r(30);

    ConsoleReader console;

    if(argc < 5)
    {
        qCritical("Invalid number of arguments: argc = %d",argc);
        return -1;
    }

    QString scriptFilePath = QString(argv[1]);
    QFileInfo fi(scriptFilePath);
    if(!fi.exists())
    {
        qCritical("Invalid script file path '%s'",scriptFilePath.toLatin1().data());
        return -1;
    }

    //    QString robotIpAddr = "192.168.1.5";
    //    QString pcIpAddr = "192.168.1.3";

    QString pcIpAddr = QString(argv[2]);
    QString robotIpAddr = QString(argv[3]);

    QHostAddress ip;
    if(!ip.setAddress(pcIpAddr))
    {
        qCritical("Invalid pc ip address '%s'",pcIpAddr.toLatin1().data());
        return -1;
    }

    if(!ip.setAddress(robotIpAddr))
    {
        qCritical("Invalid robot ip address '%s'",robotIpAddr.toLatin1().data());
        return -1;
    }

    qDebug("script: '%s'", scriptFilePath.toLatin1().data());
    qDebug("pc:     %s", pcIpAddr.toLatin1().data());
    qDebug("robot:  %s", robotIpAddr.toLatin1().data());

    bool ok;
    int managerPort = QString(argv[4]).toUShort(&ok);
    if(!ok)
    {
        qCritical("Invalid manager port '%s'",argv[4]);
        return -1;
    }

    int scriptPort = QString(argv[5]).toUShort(&ok);

    if(scriptPort==0)       // If 0, set default value 30001
        scriptPort = 30001;

    qDebug("Manager Port:  %d", managerPort);
    qDebug("Script  Port:  %d", scriptPort);

    RobotScriptManager rsm(pcIpAddr,robotIpAddr,managerPort,scriptPort);

    if(!rsm.start(scriptFilePath))
    {
        return -1;
    }

    int cnt=0;
    bool flag = true;
    QString line;
    bool hasLine;

    while(ros::ok())
    {
        line = console.getLine(&hasLine);
        if(hasLine && (line == "q"))
        {
            ros::shutdown();
        }

        if(hasLine && (line == "start gripper"))
        {
            rsm.send("gripper",true);
        }

        if(hasLine && (line == "stop gripper"))
        {
            rsm.send("gripper",false);
        }

        if(hasLine && (line == "start traj pos"))
        {
            rsm.send("traj_pos",true);
        }

        if(hasLine && (line == "stop traj pos"))
        {
            rsm.send("traj_pos",false);
        }

        if(hasLine && (line == "start traj vel"))
        {
            rsm.send("traj_vel",true);
        }

        if(hasLine && (line == "stop traj vel"))
        {
            rsm.send("traj_vel",false);
        }

        if(hasLine && (line == "h"))
        {
            qDebug("==================================================\n");
            qDebug("Help - Supported commands\n");
            qDebug("h");
            qDebug("q");
            qDebug("start gripper");
            qDebug("stop gripper");
            qDebug("start traj pos");
            qDebug("stop traj pos");
            qDebug("start traj vel");
            qDebug("stop traj vel");
            qDebug("\n==================================================");
        }

        QApplication::processEvents();
        ros::spinOnce();
        r.sleep();
        cnt++;
    }   

    qDebug("Clean exit");
    return 0;

}

