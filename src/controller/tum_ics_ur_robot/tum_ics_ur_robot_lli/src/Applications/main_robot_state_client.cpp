#include <QApplication>
#include <QFileInfo>
#include <QSettings>

#include <tum_ics_ur_robot_lli/RobotInterface/RobotStateClient.h>
#include <tum_ics_ur_robot_lli/RobotInterface/RobotStatePub.h>
#include <tum_ics_ur_robot_lli/Robot/KinematicModel.h>

using namespace tum_ics_ur_robot_lli;
using namespace tum_ics_ur_robot_lli::RobotInterface;
using namespace tum_ics_ur_robot_lli::Robot;

bool parseConfig(QString& topicName,
                 QVector<QString>& jointNames,
                 QString& robotIpAddr,
                 RobotStateClient::Version& robotVersion,
                 const QString& configFilePath);

int main(int argc, char **argv)
{
    ros::init(argc, argv,"robot_state_client",ros::init_options::AnonymousName);
    QApplication app(argc, argv);

    ros::NodeHandle n;
    ros::Rate r(30);

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

    QVector<QString> jointNames;
    QString topicName;
    QString robotIpAddr;
    RobotStateClient::Version robotVersion;

    if(!parseConfig(topicName,jointNames,robotIpAddr,robotVersion,robotConfigFilePath))
    {
        return -1;
    }



    //    Start the client to get the robot state from port 30003
    RobotStateClient robotStateClient(robotIpAddr,robotVersion);
    if(robotStateClient.error())
    {
        return -1;
    }

    KinematicModel kinModel(robotConfigFilePath);
    if(kinModel.error())
    {
        return -1;
    }

    RobotStatePub robotStatePublisher(topicName, jointNames);

    while(ros::ok() && !robotStateClient.isReady())
    {
        robotStateClient.update();
        usleep(100*1000);
    }

    robotStatePublisher.start();

    RobotTime time;

    while(ros::ok())
    {
        time.update();

        robotStateClient.update();
        robotStatePublisher.setJointState(robotStateClient.getJointState());

        kinModel.update(time,robotStateClient.getJointState());
        kinModel.publish_FK();

        QApplication::processEvents();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

bool parseConfig(QString& topicName,
                 QVector<QString>& jointNames,
                 QString& robotIpAddr,
                 RobotStateClient::Version& robotVersion,
                 const QString& configFilePath)
{
    QSettings iniFile(configFilePath, QSettings::IniFormat);

    iniFile.beginGroup("ROBOT_EXECUTION_PARAMETERS");

    for(int i=0;i<STD_DOF;i++)
    {
        std::stringstream s;
        s<<"JOINT_NAME_"<<i+1;
        QString aux = iniFile.value(s.str().c_str(), "none").toString();
        jointNames.push_back(aux);

        ROS_INFO_STREAM(s.str().c_str()<<": "<<jointNames.at(i).toStdString());
    }

    topicName="/" + iniFile.value("JOINT_TOPIC", "right_arm_joint_states").toString();

    ROS_INFO_STREAM("JOINT_TOPIC: "<<topicName.toStdString());

    iniFile.endGroup();


    iniFile.beginGroup("MOTION_SERVER_SETTINGS");

    robotIpAddr=iniFile.value("ROBOT_ADDRESS", "none").toString();

    ROS_INFO_STREAM("Robot IP: "<<robotIpAddr.toStdString());

    QHostAddress ip;
    if(!ip.setAddress(robotIpAddr))
    {
        ROS_ERROR("Invalid robot ip address '%s'",robotIpAddr.toLatin1().data());
        return false;
    }

    QString robotVersionStr=iniFile.value("ROBOT_VERSION","1.8").toString();

    ROS_INFO_STREAM("Robot Version: "<<robotVersionStr.toStdString());

    if(robotVersionStr == "1.8")
    {
        robotVersion = RobotStateClient::VERSION_1V8;
    }
    else if(robotVersionStr == "3.1")
    {
        robotVersion = RobotStateClient::VERSION_3V1;
    }
    else if(robotVersionStr == "3.4")
    {
        robotVersion = RobotStateClient::VERSION_3V4;
    }
    else if(robotVersionStr == "3.5")
    {
        robotVersion = RobotStateClient::VERSION_3V5;
    }
    else
    {
        ROS_ERROR("Unsupported robot version '%s'",robotVersionStr.toLatin1().data());
        return false;
    }

    iniFile.endGroup();

    return true;
}
