//ROS
#include <ros/ros.h>


#include<ur_robot_lli/Robot/RobotArm.h>
#include<ur_robot_lli/RobotControllers/DirectJointControl.h>
#include<ur_robot_lli/RobotControllers/SimpleEffortControl.h>

#include<QApplication>

using namespace Eigen;

int main(int argc, char **argv)
{

    QApplication a(argc,argv);



    ros::init(argc,argv,"testRobotArmClass",ros::init_options::AnonymousName);


    QString configFilePath=argv[1];

    ROS_INFO_STREAM("Config File: "<<configFilePath.toStdString().c_str());


    ur_robot_lli::Robot::RobotArm robot(configFilePath);


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

    //    ur_robot_lli::RobotControllers::DirectJointControl simpleControl(robot.getKinematicModel());
    ur_robot_lli::RobotControllers::SimpleEffortControl simpleEffortControl(robot.getKinematicModel(),robot.getDynamicModel());



    //The control must be connected to the robot after the init()-->The dynamic model needs to
    //be initialized!
    //also calls control.init(), e.g. load ctrl gains
    if(!robot.connectControl(&simpleEffortControl))
    {
        return -1;
    }


    simpleEffortControl.setQHome(robot.qHome());
    simpleEffortControl.setQPark(robot.qPark());


    robot.start();

    ROS_INFO_STREAM("Start main thread");

//    ros::Rate r(30);

//    while((ros::ok())&&(robot.isRunning()))
//    {
//        ros::spinOnce();
//        r.sleep();
//    }

    ros::spin();

    qDebug()<<"main: Stoping RobotArm()";
    robot.stop(); //stops robotArm thread




    qDebug()<<"main: Stopped!!!";

}








//    QString scriptFilePath="/home/dean/indigo_ws_iros_flo/src/tom_robot/tom_configs/Arms/parameters/Scripts/progTOM_right";

//    ur_robot_lli::CommInterface commInterface(computerIp, commandPort, trajPort,robotIp, scriptFilePath, robotScriptPort);


//    usleep(20*1E6);

//    qDebug()<<"main: Stoping commInterface()";
//    //commInterface.stop();

//    commInterface.setFinishMState();


//    ur_robot_lli::ScriptLoader scriptLoader(robotIp,
//    "/home/dean/indigo_ws_iros_flo/src/tom_robot/tom_configs/Arms/parameters/Scripts/progParkTOM_RA");

