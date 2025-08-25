#include <QApplication>
#include <QFileInfo>
#include <QSettings>

#include <tum_ics_ur_robot_lli/RobotInterface/CommInterface.h>
#include <tum_ics_ur_robot_lli/RobotTime.h>

#include "../ConsoleReader.h"


using namespace tum_ics_ur_robot_lli;
using namespace tum_ics_ur_robot_lli::RobotInterface;

int main(int argc, char **argv)
{
    ros::init(argc, argv,"comm_intf_test",ros::init_options::AnonymousName);
    QApplication app(argc, argv);

    ConsoleReader console;

    ros::NodeHandle n;
    ros::Rate r(125);

    if(argc != 2)
    {
        ROS_ERROR("Invalid number of arguments: argc = %d",argc);
        return -1;
    }

    QString robotConfigFilePath = QString(argv[1]);
    QFileInfo fi(robotConfigFilePath);
    if(!fi.exists())
    {
        ROS_ERROR("Invalid robot config path '%s'",robotConfigFilePath.toLatin1().data());
        return -1;
    }

    CommInterface commIntf(robotConfigFilePath);
    RobotTime time;

    int cnt=0;
    bool once = true;

    QString line;
    bool hasLine;

    while(ros::ok())
    {
        line = console.getLine(&hasLine);
        if(hasLine && (line == "q"))
        {
            commIntf.setFinishMState();
            usleep(2000);
            ros::shutdown();
        }

        time.update();

        VectorDOFd q = VectorDOFd::Zero();
        VectorDOFd qp = VectorDOFd::Zero();

        if(cnt%125 == 0)
        {
            if(commIntf.error())
            {
                return -1;
            }

            if(commIntf.isReady() && once)
            {
                ROS_INFO_STREAM("Comm inteface is ready!");
                JointState qO=commIntf.getCurrentJointState();
                ROS_WARN_STREAM("Pos: "<< qO.q.transpose());
                ROS_WARN_STREAM("Vel: "<< qO.qp.transpose());
                once = false;
            }
        }

//        qp(0) = 0.001;
        commIntf.setQd(q,qp);

        QApplication::processEvents();
        ros::spinOnce();
        r.sleep();
        cnt++;
    }

    return 0;
}
