#include <tum_ics_ur_robot_lli/Robot/RobotArm.h>

#include <tum_ics_ur_robot_controllers/TestEffortControl.h>

#include <tum_ics_ur_robot_controllers/SkillControls/JointControl.h>
#include <tum_ics_ur_robot_controllers/SkillControls/GControl.h>

#include<QApplication>


using namespace Eigen;

int main(int argc, char **argv)
{

    QApplication a(argc,argv);

    ros::init(argc,argv,"testRobotTime",ros::init_options::AnonymousName);

    QString configFilePath=argv[1];

    ROS_INFO_STREAM("Config File: "<<configFilePath.toStdString().c_str());


    tum_ics_ur_robot_lli::Robot::RobotArm robot(configFilePath);

//    test


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



    tum_ics_ur_robot_lli::RobotControllers::JointControl jointCtrl(robot.getKinematicModel(),robot.getDynamicModel());
    tum_ics_ur_robot_lli::RobotControllers::GControl gControl(robot.getDynamicModel());
    tum_ics_ur_robot_lli::RobotControllers::TestEffortControl tCtrl(robot.getKinematicModel(),robot.getDynamicModel());

    //The control must be connected to the robot after the init()-->The dynamic model needs to
    //be initialized!
    //also calls control.init(), e.g. load ctrl gains

    tum_ics_ur_robot_lli::Robot::RobotArm::LController listControls;


    //listControls.push_back(&jointCtrl);
    //listControls.push_back(&gControl);
    listControls.push_back(&tCtrl);

    for(int i=0;i<listControls.size();i++)
    {
        listControls[i]->setQHome(robot.qHome());
        listControls[i]->setQPark(robot.qHome());
    }




    ROS_INFO_STREAM("Connecting control");
    for(int i=0;i<listControls.size();i++)
    {
        if(!robot.add(listControls[i]))
        {
            return -1;
        }
    }



    robot.start();

    ros::Rate r(125);


    //Get the control weights from the INI file and assign them to the controllers
    robot.setCtrlWeights();


    ROS_INFO_STREAM("Starting Ctrl");
    for(int i=0;i<listControls.size();i++)
    {
        listControls[i]->start();
    }

    jointCtrl.enableQdTopic();




    while((ros::ok())&&(!listControls[0]->isFinished()))
    {

        ros::spinOnce();
        r.sleep();
    }

    for(int i=0;i<listControls.size();i++)
    {
        listControls[i]->stop();
    }


    qDebug()<<"main: Stoping RobotArm()";
    robot.stop(); //stops robotArm thread

    while(robot.isRunning())
    {
        usleep(50*1000);
    }

    qDebug()<<"main: Stopped!!!";

}
