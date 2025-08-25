#include<tum_ics_ur_robot_lli/Robot/RobotArm.h>


#include<tum_ics_ur_robot_controllers/SkillControls/GControl.h>
#include<tum_ics_ur_robot_controllers/SkillControls/SplineJointControl.h>
#include<tum_ics_ur_robot_controllers/SkillControls/JointControl.h>
#include<tum_ics_ur_robot_controllers/SkillControls/SplineCartesianControl.h>
#include<tum_ics_ur_robot_controllers/SkillControls/CartesianControl.h>


#include<QApplication>

#include<visualization_msgs/Marker.h>

using namespace Eigen;

int main(int argc, char **argv)
{

    QApplication a(argc,argv);

    ros::init(argc,argv,"testRobotArmClass",ros::init_options::AnonymousName);

    ros::NodeHandle n;

    ros::Publisher pubQd=n.advertise<sensor_msgs::JointState>("joint_desired_cmd",100);

    ros::Publisher pubXd_w=n.advertise<geometry_msgs::Pose>("x_desired_cmd",100);



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
    tum_ics_ur_robot_lli::RobotControllers::SplineJointControl splineJCtrl(robot.getKinematicModel(),robot.getDynamicModel());
    tum_ics_ur_robot_lli::RobotControllers::JointControl JointCtrl(robot.getKinematicModel(),robot.getDynamicModel());
    tum_ics_ur_robot_lli::RobotControllers::SplineCartesianControl splineCarCtrl(robot.getKinematicModel(),robot.getDynamicModel());
    tum_ics_ur_robot_lli::RobotControllers::CartesianControl carCtrl(robot.getKinematicModel(),robot.getDynamicModel());

    //To enable the topic subscriber
    //    JointCtrl.enableQdTopic();
    carCtrl.enableXdTopic();


    Vector3d xGoal;
    xGoal<<0.5, 0.0, -0.300;

    Vector3d xtorso_base;

    xtorso_base<<0.093, 0.132, -0.157;

    Quaterniond qtorso_base(0.321, 0.863, -0.321, -0.220);

    Matrix3d Rtorso_base=qtorso_base.toRotationMatrix();

    Affine3d Ttorso_base=Translation3d(xtorso_base)*AngleAxisd(Rtorso_base);

    Matrix3d R=Tum::Tools::MathTools::Rxd(180);



    Matrix3d RGoal;
    RGoal=R;



    //Goal wrt 0
    splineCarCtrl.setGoalPose(xGoal,RGoal);
    splineCarCtrl.setGoalTime(10.0);


    Quaterniond q;
    q=R;

    ROS_ERROR_STREAM("q: "<<q.x()<<", "<<q.y()<<", "<<q.z()<<", "<<q.w());

    Quaterniond q2(q.w(),q.x(),q.y(),q.z());

    ROS_ERROR_STREAM("q2: "<<q2.x()<<", "<<q2.y()<<", "<<q2.z()<<", "<<q2.w());

    tum_ics_ur_robot_lli::Robot::RobotArm::LController listControls;



    listControls.push_back(&gControl);
    listControls.push_back(&carCtrl);
    //    listControls.push_back(&splineCarCtrl);
//                listControls.push_back(&splineJCtrl);
    //    listControls.push_back(&JointCtrl);


    for(int i=0;i<listControls.size();i++)
    {
        listControls[i]->setQHome(robot.qHome());
        listControls[i]->setQPark(robot.qPark());
        //Define the goal (just needed for the PID controller)
        listControls[i]->setGoalJoints(robot.qPark().q);
        listControls[i]->setGoalTime(10.0);
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

    int count=0;
    int startC=125*2;
    int stopC=startC+(125*15);

    bool stoping=true;
    bool starting=true;



    //    double w=2*M_PI/5.0;

    //    tum_ics_ur_robot_lli::Robot::VQString_ m_jointNames=tum_ics_ur_robot_lli::RobotInterface::RobotStatePub::VQString()
    //            << "r_shoulder_pan_joint"
    //            << "r_shoulder_lift_joint"
    //            << "r_elbow_joint"
    //            << "r_wrist_1_joint"
    //            << "r_wrist_2_joint"
    //            << "r_wrist_3_joint";

    //    sensor_msgs::JointState msg;

    //    msg.name.resize(STD_DOF);
    //    msg.position.resize(STD_DOF);
    //    msg.velocity.resize(STD_DOF);
    //    msg.effort.resize(STD_DOF);
    //    for(int i=0;i<STD_DOF;i++)
    //    {
    //        msg.name[i]=m_jointNames.at(i).toStdString();
    //    }



    geometry_msgs::Pose msg_x;



    ros::Time tc;
    ros::Time ti;//=ros::Time::now();
    while((ros::ok())&&((!listControls[0]->isFinished())||(!listControls[1]->isFinished())))
    {

        if((count>=stopC)&&(stoping))
        {

            for(int i=0;i<listControls.size();i++)
            {
                listControls[i]->stop();
                //ROS_INFO_STREAM("finished: "<<listControls[i]->isFinished());
            }
            ROS_INFO_STREAM("Stoping Ctrl");
            stoping=false;
        }
        else if(count>=startC)
        {

            if(starting)
            {


                for(int i=0;i<listControls.size();i++)
                {
                    listControls[i]->start();
                }

                ROS_INFO_STREAM("Starting Ctrl");
                starting=false;
                ti=ros::Time::now();
            }


            tc=ros::Time::now();
            double t=tc.toSec()-ti.toSec();

            //JOINT CTRL
            //            //            Tum::VectorDOFd qd;

            //            //            for(unsigned int i=0;i<STD_DOF;i++)
            //            //            {
            //            //                qd(i)=M_PI_4*sin(w*t)+robot.qHome().q(i);
            //            //            }

            //            //            listControls[1]->setGoalJoints(qd);



            //            msg.header.seq=count;
            //            msg.header.stamp=tc;


            //            for(int i=0;i<STD_DOF;i++)
            //            {
            //                msg.position[i]=M_PI_4*sin(w*t)+robot.qHome().q(i);
            //                msg.velocity[i]=0.0;
            //                msg.effort[i]=0.0;
            //            }


            //            pubQd.publish(msg);


            //            //CARTESIAN CTRL
            Vector3d xGoal_0;
            xGoal_0<< -0.546228, 0.391894, 0.378792;
            Matrix3d RGoal_0;
            RGoal_0<<-0.274452,   0.73803, -0.616432,
                    -0.581269,  0.383343,  0.717757,
                    0.76603,  0.555302,  0.323785;

            Vector3d xGoal_w=robot.getKinematicModel()->T0_w()*xGoal_0.homogeneous();
            Matrix3d RGoal_w=robot.getKinematicModel()->T0_w().rotation()*RGoal_0;


            xGoal_w(1)+=0.04*t;
            xGoal_w(2)-=0.02*t;


            Quaterniond qGoal_w;
            qGoal_w=RGoal_w;

            msg_x.position.x=xGoal_w(0);
            msg_x.position.y=xGoal_w(1);
            msg_x.position.z=xGoal_w(2);

            msg_x.orientation.x=qGoal_w.x();
            msg_x.orientation.y=qGoal_w.y();
            msg_x.orientation.z=qGoal_w.z();
            msg_x.orientation.w=qGoal_w.w();

            pubXd_w.publish(msg_x);


            //            listControls[1]->setGoalPose(xGoal_w,RGoal_w);


        }


        ros::spinOnce();
        r.sleep();

        count++;
    }


    qDebug()<<"main: Stoping RobotArm()";
    robot.stop(); //stops robotArm thread


    while(robot.isRunning())
    {
        usleep(50*1000);
    }

    qDebug()<<"main: Stopped!!!";

}





//    QString scriptFilePath="/home/dean/indigo_ws_iros_flo/src/tom_robot/tom_configs/Arms/parameters/Scripts/progTOM_right";

//    tum_ics_ur_robot_lli::CommInterface commInterface(computerIp, commandPort, trajPort,robotIp, scriptFilePath, robotScriptPort);


//    usleep(20*1E6);

//    qDebug()<<"main: Stoping commInterface()";
//    //commInterface.stop();

//    commInterface.setFinishMState();


//    tum_ics_ur_robot_lli::ScriptLoader scriptLoader(robotIp,
//    "/home/dean/indigo_ws_iros_flo/src/tom_robot/tom_configs/Arms/parameters/Scripts/progParkTOM_RA");

