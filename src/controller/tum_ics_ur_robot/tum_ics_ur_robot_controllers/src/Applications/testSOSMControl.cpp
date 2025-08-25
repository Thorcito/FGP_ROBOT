#include<tum_ics_ur_robot_lli/Robot/RobotArm.h>
#include<tum_ics_ur_robot_controllers/SOSMControl.h>
#include<tum_ics_ur_robot_controllers/PIDControl.h>
#include<tum_ics_ur_robot_controllers/SkillControls/SplineJointControl.h>


#include<QApplication>

#include<visualization_msgs/Marker.h>

using namespace Eigen;

int main(int argc, char **argv)
{

    QApplication a(argc,argv);

    ros::init(argc,argv,"testRobotArmClass",ros::init_options::AnonymousName);

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

    tum_ics_ur_robot_lli::RobotControllers::SplineJointControl splineJCtrl(robot.getKinematicModel(),robot.getDynamicModel());
    //    tum_ics_ur_robot_lli::RobotControllers::SOSMControl sosmControl(robot.getKinematicModel(),robot.getDynamicModel());
    //    tum_ics_ur_robot_lli::RobotControllers::PIDControl sosmControl(0.008);//robot.getKinematicModel(),robot.getDynamicModel());



    //The control must be connected to the robot after the init()-->The dynamic model needs to
    //be initialized!
    //also calls control.init(), e.g. load ctrl gains

    tum_ics_ur_robot_lli::Robot::RobotArm::LController listControls;


    listControls.push_back(&splineJCtrl);

    for(int i=0;i<listControls.size();i++)
    {
        listControls[i]->setQHome(robot.qHome());
        listControls[i]->setQPark(robot.qHome());
    }

    //Define the goal (just needed for the PID controller)
    listControls[0]->setGoalJoints(robot.qPark().q);
    listControls[0]->setGoalTime(15.0);


    //        splineJCtrl.setQHome(robot.qHome());
    //        splineJCtrl.setQPark(robot.qHome());


    //Define the goal (just needed for the PID controller)
    //        splineJCtrl.setGoalJoints(robot.qPark().q);
    //        splineJCtrl.setGoalTime(10.0);




    ROS_INFO_STREAM("Connecting control");
    for(int i=0;i<listControls.size();i++)
    {
        if(!robot.add(listControls[i]))
        {
            return -1;
        }
    }

    //        if(!robot.add(&splineJCtrl))
    //        {
    //            return -1;
    //        }

    robot.start();

    ros::Rate r(125);

    int count=0;
    int startC=125*2;
    int stopC=startC+(125*5);

    bool stoping=true;
    bool starting=true;
    while((ros::ok())&&(!listControls[0]->isFinished()))
    {

//        if((count>=stopC)&&(stoping))
//        {

//            listControls[0]->stop();
//            //                        splineJCtrl.stop();
//            ROS_INFO_STREAM("Stoping Ctrl");
//            stoping=false;
//        }
        if((count>=startC)&&(starting))
        {

            listControls[0]->start();
            //                        splineJCtrl.start();
            ROS_INFO_STREAM("Starting Ctrl");
            starting=false;

        }


        ros::spinOnce();
        r.sleep();

        count++;
    }

    //    ROS_INFO_STREAM("Finished? :"<<listControls[0]->isFinished());

    qDebug()<<"main: Stoping RobotArm()";
    robot.stop(); //stops robotArm thread

    while(robot.isRunning())
    {
        usleep(50*1000);
    }

    qDebug()<<"main: Stopped!!!";

}
