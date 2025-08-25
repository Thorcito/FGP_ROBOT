#include <QApplication>
#include <QDebug>
#include <QDir>

//#include <ros/ros.h>
//#include <tum_ics_ur_robot_manager/RobotScriptManager.h>
#include <tum_ics_ur_robot_manager/RobotArmClosed.h>
#include<tum_ics_ur_robot_controllers/SkillControls/GControl.h>
#include<tum_ics_ur_robot_controllers/SkillControls/JointControl.h>

#include "../ConsoleReader.h"

using namespace tum_ics_ur_robot_manager;

int main(int argc, char **argv)
{
    ros::init(argc, argv,"script_loader",ros::init_options::AnonymousName);
    QApplication app(argc, argv);

    ros::NodeHandle n;
    ros::Rate r(125);

    ConsoleReader console;

//    if(argc != 5)
//    {
//        qCritical("Invalid number of arguments: argc = %d",argc);
//        return -1;
//    }


    //QString robotIpAddr = "192.168.1.5";
    QString pcIpAddr = "192.168.1.3";

    //    QString pcIpAddr = QString(argv[2]);
    //    QString robotIpAddr = QString(argv[3]);

    QHostAddress ip;
    if(!ip.setAddress(pcIpAddr))
    {
        qCritical("Invalid pc ip address '%s'",pcIpAddr.toLatin1().data());
        return -1;
    }

    //    if(!ip.setAddress(robotIpAddr))
    //    {
    //        qCritical("Invalid robot ip address '%s'",robotIpAddr.toLatin1().data());
    //        return -1;
    //    }

    //    bool ok;
    //    int managerPort = QString(argv[4]).toUShort(&ok);
    //    if(!ok)
    //    {
    //        qCritical("Invalid manager port '%s'",argv[4]);
    //        return -1;
    //    }

    //    qDebug("script: '%s'", scriptFilePath.toLatin1().data());
    qDebug("pc:     %s", pcIpAddr.toLatin1().data());
    // qDebug("robot:  %s", robotIpAddr.toLatin1().data());

    QString configFilePath=argv[1];

    ROS_INFO_STREAM("Config File: "<<configFilePath.toStdString().c_str());

    RobotArmClosed robot(configFilePath,pcIpAddr);
    if(!robot.connect())
    {
        return -1;
    }

    //Wait a few seconds to stablish the communication
    usleep(5E6);


    if(!robot.init())
    {
        return -1;
    }
    //Setting g vector
    Tum::Vector3d gw;
    gw<<0,0,-1;
    robot.getDynamicModel()->setG_w(gw);


    tum_ics_ur_robot_lli::RobotControllers::GControl gControl(robot.getDynamicModel());
    tum_ics_ur_robot_lli::RobotControllers::JointControl JointCtrl(robot.getKinematicModel(),robot.getDynamicModel());


    //To enable the topic subscriber
    //JointCtrl.enableQdTopic();


    tum_ics_ur_robot_lli::Robot::RobotArm::LController listControls;



    listControls.push_back(&gControl);
    listControls.push_back(&JointCtrl);


    for(int i=0;i<listControls.size();i++)
    {
        listControls[i]->setQHome(robot.qHome());
        listControls[i]->setQPark(robot.qPark());
        //Define the goal (just needed for the PID controller)
        //listControls[i]->setGoalJoints(robot.qPark().q);
        //listControls[i]->setGoalTime(10.0);
    }


    ROS_INFO_STREAM("Connecting control");
    for(int i=0;i<listControls.size();i++)
    {
        if(!robot.add(listControls[i]))
        {
            return -1;
        }
    }


    tum_ics_ur_robot_lli::Robot::VQString_ names=robot.jointNames();

    robot.start();


    tum_ics_ur_robot_lli::Robot::VQString_ m_jointNames=tum_ics_ur_robot_lli::RobotInterface::RobotStatePub::VQString()
            << "shouldepan_joint"
            << "shouldelift_joint"
            << "elbow_joint"
            << "wrist_1_joint"
            << "wrist_2_joint"
            << "wrist_3_joint";

    sensor_msgs::JointState msg;

    msg.name.resize(STD_DOF);
    msg.position.resize(STD_DOF);
    msg.velocity.resize(STD_DOF);
    msg.effort.resize(STD_DOF);
    for(int i=0;i<STD_DOF;i++)
    {
        msg.name[i]=m_jointNames.at(i).toStdString();
    }


    int cnt=0;
//    bool flag = true;
//    QString line;
//    bool hasLine;
    double w=2*M_PI/80.0;


    for(int i=0;i<listControls.size();i++)
    {
        listControls[i]->start();
    }

    ROS_INFO_STREAM("Starting Ctrl");


    ros::Time tc;
    ros::Time ti=ros::Time::now();

    while(ros::ok())
    {

        tc=ros::Time::now();
        double t=tc.toSec()-ti.toSec();

        //JOINT CTRL
        Tum::VectorDOFd qd;

        for(unsigned int i=0;i<STD_DOF;i++)
        {
            qd(i)=M_PI_4*sin(w*t)+robot.qHome().q(i);
        }

        listControls[1]->setGoalJoints(qd);



        //            msg.header.seq=count;
        //            msg.header.stamp=tc;


        //            for(int i=0;i<STD_DOF;i++)
        //            {
        //                msg.position[i]=M_PI_4*sin(w*t)+robot.qHome().q(i);
        //                msg.velocity[i]=0.0;
        //                msg.effort[i]=0.0;
        //            }


        //            pubQd.publish(msg);


        QApplication::processEvents();
        ros::spinOnce();
        r.sleep();
        cnt++;
    }


    for(int i=0;i<listControls.size();i++)
    {
        listControls[i]->stop();
        //ROS_INFO_STREAM("finished: "<<listControls[i]->isFinished());
    }
    ROS_INFO_STREAM("Stoping Ctrl");

    ROS_INFO_STREAM("main: Stoping RobotArm()");
    robot.stop(); //stops robotArm thread

    while(robot.isRunning())
    {
        usleep(50*1000);
    }

    qDebug("Clean exit");
    return 0;

}




//        line = console.getLine(&hasLine);
//        if(hasLine && (line == "q"))
//        {
//            ros::shutdown();
//        }

//        //        if(hasLine && (line == "start gripper"))
//        //        {
//        //            rsm.send("gripper",true);
//        //        }

//        //        if(hasLine && (line == "stop gripper"))
//        //        {
//        //            rsm.send("gripper",false);
//        //        }

//        //        if(hasLine && (line == "start traj pos"))
//        //        {
//        //            rsm.send("traj_pos",true);
//        //        }

//        //        if(hasLine && (line == "stop traj pos"))
//        //        {
//        //            rsm.send("traj_pos",false);
//        //        }

//        //        if(hasLine && (line == "start traj vel"))
//        //        {
//        //            rsm.send("traj_vel",true);
//        //        }

//        //        if(hasLine && (line == "stop traj vel"))
//        //        {
//        //            rsm.send("traj_vel",false);
//        //        }

//        if(hasLine && (line == "h"))
//        {
//            qDebug("==================================================\n");
//            qDebug("Help - Supported commands\n");
//            qDebug("h");
//            qDebug("q");
////            qDebug("start gripper");
////            qDebug("stop gripper");
////            qDebug("start traj pos");
////            qDebug("stop traj pos");
////            qDebug("start traj vel");
////            qDebug("stop traj vel");
//            qDebug("\n==================================================");
//        }
