#include<tum_ics_ur_robot_lli/Robot/RobotArm.h>
#include<tum_ics_ur_robot_controllers/PIDControl.h>
#include<tum_ics_ur_robot_controllers/SkillControls/GControl.h>
#include<tum_ics_ur_robot_controllers/SOSMOpControl.h>


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

    //    tum_ics_ur_robot_lli::RobotControllers::DirectJointControl simpleJointControl(robot.getKinematicModel());
    tum_ics_ur_robot_lli::RobotControllers::PIDControl pidControl(robot.getDynamicModel()->getCtrlPeriod());
    tum_ics_ur_robot_lli::RobotControllers::SOSMOpControl sosmOpControl(robot.getKinematicModel(),robot.getDynamicModel());
    tum_ics_ur_robot_lli::RobotControllers::GControl gControl(robot.getDynamicModel());



    //The control must be connected to the robot after the init()-->The dynamic model needs to
    //be initialized!
    //also calls control.init(), e.g. load ctrl gains
    ROS_INFO_STREAM("Connecting control");
    if(!robot.add(&gControl))
    {
        return -1;
    }

    if(!robot.add(&pidControl))
    {
        return -1;
    }

    pidControl.setQHome(robot.qHome());
    pidControl.setQPark(robot.qPark());

    sosmOpControl.setQHome(robot.qHome());
    sosmOpControl.setQPark(robot.qPark());

    Vector3d xGoal;
    xGoal<<0.381, -0.360, 0.810;

    Vector3d xtorso_base;

    xtorso_base<<0.093, 0.132, -0.157;

    Quaterniond qtorso_base(0.321, 0.863, -0.321, -0.220);

    Matrix3d Rtorso_base=qtorso_base.toRotationMatrix();

    Affine3d Ttorso_base;

    Matrix3d R=Rtorso_base*Tum::Tools::MathTools::Rxd(180);


    Matrix3d RGoal;
    RGoal.setIdentity();

    RGoal=R;

//    RGoal=Tum::Tools::MathTools::Rxd(90.0);


    sosmOpControl.setGoalPose(xGoal,RGoal);
    sosmOpControl.setGoalTime(10.0);

    //Get the control weights from the INI file and assign them to the controllers
    robot.setCtrlWeights();

    robot.start();

    ROS_INFO_STREAM("Start main thread");

    ros::spin();

    ROS_INFO_STREAM("main: Stoping RobotArm()");
    robot.stop(); //stops robotArm thread

    while(robot.isRunning())
    {
        usleep(50*1000);
    }

    ROS_INFO_STREAM("main: Stopped!!!");

}

