#include <tum_ics_ur_robot_lli/Robot/RobotArm.h>
#include <tum_ics_ur_robot_controllers/SkillControls/SplineJointControl.h>
#include <tum_ics_ur_robot_controllers/SkillControls/GControl.h>
#include <visualization_msgs/Marker.h>

#include <tum_ics_ur_robot_manager/SkillList.h>
#include <tum_ics_ur_robot_manager/SkillManager.h>

#include <QApplication>

#include "tum_ics_ur_robot_manager/ConsoleReader.h"

using namespace Eigen;
using namespace tum_ics_ur_robot_manager;

typedef tum_ics_ur_robot_lli::RobotControllers::Controller Controller;

int main(int argc, char **argv)
{
    QApplication a(argc,argv);
    ros::init(argc,argv,"skill_manager_test",ros::init_options::AnonymousName);

    ros::NodeHandle n;
    ros::Rate r(125);

    ConsoleReader console;

    const Tum::VectorDOFd qWorkRight =
            DEG2RAD((Tum::VectorDOFd() << -50.0, -90.0, 90.0, 0.0, 90.0, -90.0).finished());

    const Tum::VectorDOFd qWork2Right =
            DEG2RAD((Tum::VectorDOFd() << -45.0, -90.0, 90.0, 0.0, 90.0, 90.0).finished());

    const Tum::VectorDOFd qHomeRight =
            DEG2RAD((Tum::VectorDOFd() << 129.13, -97.37, 1.43, -188.08, -100.52, 0.01).finished());


    QString configFilePath=argv[1];

    ROS_INFO_STREAM("Config File: "<<configFilePath.toStdString().c_str());

    tum_ics_ur_robot_lli::Robot::RobotArm robot(configFilePath);


    //starts robotArm communication and the thread
    //inits Kinematic Model, Dynamic Model
    if(!robot.init())
    {
        return -1;
    }

    //Setting g vector
    Vector3d gw;
    gw<<0,0,-1;
    robot.getDynamicModel()->setG_w(gw);


    tum_ics_ur_robot_lli::RobotControllers::GControl gControl(robot.getDynamicModel());
    tum_ics_ur_robot_lli::RobotControllers::SplineJointControl splineJointCtrl(robot.getKinematicModel(),robot.getDynamicModel());

    tum_ics_ur_robot_lli::Robot::RobotArm::LController controllers;

    controllers.push_back(&gControl);
    controllers.push_back(&splineJointCtrl);

    ROS_ERROR_STREAM("Goal: "<<RAD2DEG(robot.qPark().q.transpose()));

    // init controllers
    for(int i=0;i<controllers.size();i++)
    {
        controllers[i]->setQHome(robot.qHome());
        controllers[i]->setQPark(robot.qHome());
        //Define the goal (just needed for the PID controller)
        controllers[i]->setGoalJoints(robot.qPark().q);
        controllers[i]->setGoalTime(10.0);
    }

    SkillList skillList;
    tum_ics_ur_robot_lli::RobotControllers::ControllerList ctrlList(controllers);

    if(!skillList.load())
    {
        ROS_ERROR("Loading skill list from parameter server failed.");
        return -1;
    }

//    qDebug("%s",skillList.toString().toLatin1().data());

    SkillManager skillManager(
                ctrlList,
                skillList,
                &robot);

    qDebug("%s",skillManager.skillList().toString().toLatin1().data());

    robot.start();
    skillManager.start();

    int cnt=0;
    QString line;
    bool hasLine;

    while((ros::ok()))
    {
        line = console.getLine(&hasLine);
        if(hasLine && (line == "q"))
        {
            ros::shutdown();
        }

        if(hasLine && (line == "reach qgoal home"))
        {
            QString skill = "REACH_JOINT_GOAL";
            Tum::VectorDOFd qGoal = qHomeRight;
            double duration = 10.0;
            if(!skillManager.setNextSkill(skill,qGoal,duration))
            {
                ROS_ERROR("Setting next skill '%s' failed.",skill.toLatin1().data());
            }
            skillManager.requestSkillTransition();
        }

        if(hasLine && (line == "reach qgoal work"))
        {
            QString skill = "REACH_JOINT_GOAL";
            Tum::VectorDOFd qGoal = qWorkRight;
            double duration = 10.0;
            if(!skillManager.setNextSkill(skill,qGoal,duration))
            {
                ROS_ERROR("Setting next skill '%s' failed.",skill.toLatin1().data());
            }
            skillManager.requestSkillTransition();
        }

        if(hasLine && (line == "reach qgoal work 2"))
        {
            QString skill = "REACH_JOINT_GOAL";
            Tum::VectorDOFd qGoal = qWork2Right;
            double duration = 10.0;
            if(!skillManager.setNextSkill(skill,qGoal,duration))
            {
                ROS_ERROR("Setting next skill '%s' failed.",skill.toLatin1().data());
            }
            skillManager.requestSkillTransition();
        }

        if(hasLine && (line == "idle"))
        {
            QString skill = "IDLE";
            if(!skillManager.setNextSkill(skill))
            {
                ROS_ERROR("Setting next skill '%s' failed.",skill.toLatin1().data());
            }
            skillManager.requestSkillTransition();
        }

//        if(cnt%125==0)
//        {
//            if(skillManager.isRunning())
//            {
//                qDebug("SkillManager is running.");
//            }

//            if(skillManager.isReadyToStart())
//            {
//                qDebug("SkillManager is ready to start.");
//            }
//        }

        QApplication::processEvents();
        ros::spinOnce();
        r.sleep();
        cnt++;
    }

    skillManager.stop();

    // wait till thread finished
    while(skillManager.isRunning())
    {
        usleep(50*1000);
    }

    qDebug("main: Stop RobotArm() ...");
    robot.stop(); //stops robotArm thread

    // wait till thread finished
    while(robot.isRunning())
    {
        usleep(50*1000);
    }

    qDebug()<<"main: Stopped RobotArm().";

}

