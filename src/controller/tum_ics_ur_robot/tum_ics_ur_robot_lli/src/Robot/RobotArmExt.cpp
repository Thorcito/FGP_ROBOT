#include <tum_ics_ur_robot_lli/Robot/RobotArmExt.h>
//#include <QList>
#include <QSettings>

namespace tum_ics_ur_robot_lli{
namespace Robot{


RobotArmExt::RobotArmExt(const QString &configFilePath):
    m_configFilePath(configFilePath),
    m_error(false),
    m_realRobot(false)
{
    ROS_INFO_STREAM("Creating Robot with config file: "<<m_configFilePath.toStdString());

    std::stringstream s;
    if(!parseConfig())
    {

        s<<"Couldn't load RobotConfigFilePath: "<<m_configFilePath.toStdString();
        m_error=true;
        m_errorString=s.str().c_str();
        ROS_ERROR_STREAM(m_errorString.toStdString());
        s<<"";
    }


    if(m_realRobot)
    {
        m_robotStateClient = new RobotInterface::RobotStateClient(m_robotIpAddr,m_robotVersion);
        if(m_robotStateClient->error())
        {
            std::stringstream s;
            s<<"Error Starting the Robot State client: "<<m_robotStateClient->errorString().toStdString();
            m_error=true;
            m_errorString=s.str().c_str();
            ROS_ERROR_STREAM(m_errorString.toStdString());
            s<<"";
        }
        //Wait until the robotStateClient is connected and the link to the robot arm
        while(ros::ok() && !m_robotStateClient->isReady())
        {
            m_robotStateClient->update();
            usleep(100*1000);
        }
    }

    m_robot = new RobotArm(m_configFilePath,m_robotStateClient);
}

RobotArmExt::~RobotArmExt()
{
    if(m_robot!=NULL)
    {
        delete m_robot;
    }
    if(m_robotStateClient!=NULL)
    {
        delete m_robotStateClient;
    }
}

bool RobotArmExt::add(RobotControllers::Controller *u)
{
    return m_robot->add(u);
}

bool RobotArmExt::init()
{
    return m_robot->init();
}

void RobotArmExt::start()
{
    m_robot->start();
}

bool RobotArmExt::stop()
{
    return m_robot->stop();
}

JointState RobotArmExt::qHome() const
{
    return m_robot->qHome();
}

JointState RobotArmExt::qPark() const
{
    return m_robot->qPark();
}

bool RobotArmExt::parseConfig()
{
    QSettings iniFile(m_configFilePath, QSettings::IniFormat);

    iniFile.beginGroup("ROBOT_EXECUTION_PARAMETERS");

    for(int i=0;i<STD_DOF;i++)
    {
        std::stringstream s;
        s<<"JOINT_NAME_"<<i+1;
        QString aux = iniFile.value(s.str().c_str(), "none").toString();
        m_jointNames.push_back(aux);

        ROS_INFO_STREAM(s.str().c_str()<<": "<<m_jointNames.at(i).toStdString());
    }

    m_topicName="/" + iniFile.value("JOINT_TOPIC", "right_arm_joint_states").toString();

    ROS_INFO_STREAM("JOINT_TOPIC: "<<m_topicName.toStdString());

    iniFile.endGroup();


    iniFile.beginGroup("MOTION_SERVER_SETTINGS");

    m_robotIpAddr=iniFile.value("ROBOT_ADDRESS", "none").toString();

    ROS_INFO_STREAM("Robot IP: "<<m_robotIpAddr.toStdString());

    QHostAddress ip;
    if(!ip.setAddress(m_robotIpAddr))
    {
        ROS_ERROR("Invalid robot ip address '%s'",m_robotIpAddr.toLatin1().data());
        return false;
    }

    QString m_robotVersionStr=iniFile.value("ROBOT_VERSION","1.8").toString();

    ROS_INFO_STREAM("Robot Version: "<<m_robotVersionStr.toStdString());

    if(m_robotVersionStr == "1.8")
    {
        m_robotVersion = RobotInterface::RobotStateClient::VERSION_1V8;
    }
    else if(m_robotVersionStr == "3.1")
    {
        m_robotVersion = RobotInterface::RobotStateClient::VERSION_3V1;
    }
    else if(m_robotVersionStr == "3.4")
    {
        m_robotVersion = RobotInterface::RobotStateClient::VERSION_3V4;
    }
    else if(m_robotVersionStr == "3.5")
    {
        m_robotVersion = RobotInterface::RobotStateClient::VERSION_3V5;
    }
    else
    {
        ROS_ERROR("Unsupported robot version '%s'",m_robotVersionStr.toLatin1().data());
        return false;
    }

    iniFile.endGroup();

    iniFile.beginGroup("ROBOT_EXECUTION_PARAMETERS");
    QString robotType=iniFile.value("ROBOT_TYPE", "sim").toString();
    if(!(robotType.compare("real")) || !(robotType.compare("REAL")) )
    {
        m_realRobot=true;
    }
    else
    {
        m_realRobot=false;
    }
    iniFile.endGroup();


    return true;
}

bool RobotArmExt::error() const
{
    return m_error;
}
const QString& RobotArmExt::errorString() const
{
    return m_errorString;
}

const bool RobotArmExt::isRunning() const
{
    return m_robot->isRunning();
}

}
}
