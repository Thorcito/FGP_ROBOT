#include <QApplication>
#include <QFileInfo>

#include <tum_ics_ur_robot_lli/RobotInterface/RobotStateClient.h>
#include <tum_ics_ur_robot_lli/RobotInterface/RobotStatePub.h>

#include "ConsoleReader.h"

using namespace tum_ics_ur_robot_lli;
using namespace tum_ics_ur_robot_lli::RobotInterface;

int main(int argc, char **argv)
{
    ros::init(argc, argv,"robotStateClientTest",ros::init_options::AnonymousName);
    QApplication app(argc, argv);

    ros::NodeHandle n;
    ros::Rate r(30);

    ConsoleReader console;

    if(argc != 2)
    {
        qCritical("Invalid number of arguments: argc = %d",argc);
        return -1;
    }

    QString robotIpAddr = QString(argv[1]);

    QHostAddress ip;
    if(!ip.setAddress(robotIpAddr))
    {
        qCritical("Invalid robot ip address '%s'",robotIpAddr.toLatin1().data());
        return -1;
    }

    //Start the client to get the robot state from port 30003
    RobotStateClient rsc(robotIpAddr,RobotStateClient::VERSION_3V1);

    if(rsc.error())
    {
        return false;
    }


    //For now to test
    QVector<QString> jointNames = QVector<QString> ()
            << "ursa_shoulder_pan_joint"
            << "ursa_shoulder_lift_joint"
            << "ursa_elbow_joint"
            << "ursa_wrist_1_joint"
            << "ursa_wrist_2_joint"
            << "ursa_wrist_3_joint";

    QString topicName="/ursa_joint_states";

    RobotStatePub rsp(topicName, jointNames);

    while(ros::ok() && !rsc.isReady())
    {
        rsc.update();
        usleep(100*1000);
    }

    rsp.start();

    QString line;
    bool hasLine;

    std::stringstream ss;
    QString str;

    while(ros::ok())
    {
        rsc.update();
        rsp.setJointState(rsc.getJointState());


        ss << RAD2DEG(rsc.getJointState().q.transpose());
        str = QString(ss.str().c_str());
        ss.str("");

//        qDebug("q: [%s]",str.toLatin1().data());

        line = console.getLine(&hasLine);
        if(hasLine && (line == "q"))
        {
            break;
        }

        QApplication::processEvents();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
