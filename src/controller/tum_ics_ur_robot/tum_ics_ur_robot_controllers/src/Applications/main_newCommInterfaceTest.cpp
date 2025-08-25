//#include<tum_ics_ur_robot_lli/Robot/RobotArm.h>
//#include<tum_ics_ur_robot_controllers/DirectJointControl.h>
//#include<tum_ics_ur_robot_controllers/SimpleEffortControl.h>
//#include<tum_ics_ur_robot_controllers/SOSMControl.h>
//#include<tum_ics_ur_robot_controllers/SOSMOpControl.h>

#include <QApplication>
#include <QFileInfo>

#include <tum_ics_ur_robot_lli/RobotInterface/CommInterface.h>

#include "ConsoleReader.h"

using namespace tum_ics_ur_robot_lli;

int main(int argc, char **argv)
{
    ros::init(argc, argv,"newCommInterfaceTest",ros::init_options::AnonymousName);
    QApplication app(argc, argv);

    ros::NodeHandle n;
    ros::Rate r(30);

    ConsoleReader console;


    QString configFilePath = QString(argv[1]);
    QFileInfo fi(configFilePath);
    if(!fi.exists())
    {
        qCritical("Invalid config file path '%s'",configFilePath.toLatin1().data());
        return -1;
    }

    RobotInterface::CommInterface commIntf(configFilePath);

    QString line;
    bool hasLine;

    while(ros::ok())
    {
        if(commIntf.error())
        {
            ros::shutdown();
        }

        line = console.getLine(&hasLine);
        if(hasLine && (line == "q"))
        {
            commIntf.setFinishMState();
            ros::shutdown();
        }

        QApplication::processEvents();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
