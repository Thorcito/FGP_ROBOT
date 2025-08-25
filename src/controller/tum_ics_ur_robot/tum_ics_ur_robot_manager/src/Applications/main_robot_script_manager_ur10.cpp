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


    QString pcIpAddr = "192.168.1.3";//QString(argv[1]);//
    QString robotIpAddr = "192.168.1.10";

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


    int managerPort = 50001;

    ROS_INFO_STREAM("PC IP: "<<pcIpAddr.toStdString().c_str());
    ROS_INFO_STREAM("Robot IP: "<<robotIpAddr.toStdString().c_str());
    ROS_INFO_STREAM("Port: "<<managerPort);

    RobotScriptManager rsm(pcIpAddr,robotIpAddr,managerPort);

    if(!rsm.start())
    {
        ROS_ERROR_STREAM("Could not load the script in "<<robotIpAddr.toStdString().c_str()<<" from host: "<<pcIpAddr.toStdString().c_str());
        ROS_ERROR_STREAM("Is the robot on?? or have you already upload the script into the robot???");
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


        if(hasLine && (line == "h"))
        {
            qDebug("==================================================\n");
            qDebug("Help - Supported commands\n");
            qDebug("h");
            qDebug("q");
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

