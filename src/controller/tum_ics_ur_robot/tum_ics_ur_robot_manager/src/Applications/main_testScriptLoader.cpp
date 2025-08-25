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


    if(argc < 5)
    {
        ROS_ERROR_STREAM("Invalid number of arguments: argc = "<<argc);
        return -1;
    }

    QString scriptFilePath = QString(argv[1]);
    QFileInfo fi(scriptFilePath);
    if(!fi.exists())
    {
        ROS_ERROR_STREAM("Invalid script file path: "<<scriptFilePath.toStdString());
        return -1;
    }

    //    QString robotIpAddr = "192.168.1.5";
    //    QString pcIpAddr = "192.168.1.3";

    QString pcIpAddr = QString(argv[2]);
    QString robotIpAddr = QString(argv[3]);

    QHostAddress ip;
    if(!ip.setAddress(pcIpAddr))
    {
        ROS_ERROR_STREAM("Invalid pc ip address: "<<pcIpAddr.toStdString());
        return -1;
    }

    if(!ip.setAddress(robotIpAddr))
    {
        ROS_ERROR_STREAM("Invalid robot ip address: "<<robotIpAddr.toStdString());
        return -1;
    }

    ROS_WARN_STREAM("script: "<< scriptFilePath.toStdString());
    ROS_WARN_STREAM("pc:     "<< pcIpAddr.toStdString());
    ROS_WARN_STREAM("robot:  "<< robotIpAddr.toStdString());

    bool ok;
    int managerPort = QString(argv[4]).toUShort(&ok);
    if(!ok)
    {
        ROS_ERROR_STREAM("Invalid manager port: "<<argv[4]);
        return -1;
    }

    int scriptPort = QString(argv[5]).toUShort(&ok);

    if(scriptPort==0)       // If 0, set default value 30001
        scriptPort = 30001;

    ROS_WARN_STREAM("Manager Port:  "<<managerPort);
    ROS_WARN_STREAM("Script  Port:  "<<scriptPort);

    ScriptLoader m_loader(robotIpAddr,scriptPort);

    if(!m_loader.load(scriptFilePath))
    {
        ROS_ERROR_STREAM("ERROR loading script: "<<scriptFilePath.toStdString());
        return -1;
    }

    sleep(5);

//    RobotScriptManager rsm(pcIpAddr,robotIpAddr,managerPort,scriptPort);

//    if(!rsm.start(scriptFilePath))
//    {
//        return -1;
//    }

//    int cnt=0;
//    bool flag = true;
//    QString line;
//    bool hasLine;

//    while(ros::ok())
//    {
//        line = console.getLine(&hasLine);
//        if(hasLine && (line == "q"))
//        {
//            ros::shutdown();
//        }

//        if(hasLine && (line == "start gripper"))
//        {
//            rsm.send("gripper",true);
//        }

//        if(hasLine && (line == "stop gripper"))
//        {
//            rsm.send("gripper",false);
//        }

//        if(hasLine && (line == "start traj pos"))
//        {
//            rsm.send("traj_pos",true);
//        }

//        if(hasLine && (line == "stop traj pos"))
//        {
//            rsm.send("traj_pos",false);
//        }

//        if(hasLine && (line == "start traj vel"))
//        {
//            rsm.send("traj_vel",true);
//        }

//        if(hasLine && (line == "stop traj vel"))
//        {
//            rsm.send("traj_vel",false);
//        }

//        if(hasLine && (line == "h"))
//        {
//            ROS_WARN_STREAM("==================================================\n");
//            ROS_WARN_STREAM("Help - Supported commands\n");
//            ROS_WARN_STREAM("h");
//            ROS_WARN_STREAM("q");
//            ROS_WARN_STREAM("start gripper");
//            ROS_WARN_STREAM("stop gripper");
//            ROS_WARN_STREAM("start traj pos");
//            ROS_WARN_STREAM("stop traj pos");
//            ROS_WARN_STREAM("start traj vel");
//            ROS_WARN_STREAM("stop traj vel");
//            ROS_WARN_STREAM("\n==================================================");
//        }

//        QApplication::processEvents();
//        ros::spinOnce();
//        r.sleep();
//        cnt++;
//    }

    ROS_WARN_STREAM("Clean exit");
    return 0;

}

