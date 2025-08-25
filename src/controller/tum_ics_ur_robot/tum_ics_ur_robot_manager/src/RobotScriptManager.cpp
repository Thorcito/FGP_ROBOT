#include <tum_ics_ur_robot_manager/RobotScriptManager.h>

#include <unistd.h>

namespace tum_ics_ur_robot_manager{

QVector<QString> RobotScriptManager::progNames()
{
    const QVector<QString> names = QVector<QString>()
            << "gripper"
            << "traj_pos"
            << "traj_vel";

    return names;
}

RobotScriptManager::Command RobotScriptManager::getCommand(
            const QString& name,
            bool enable)
{
    const QVector<Command> listEnable = QVector<Command>()
            << START_GRIPPER_CMD
            << START_TRAJ_POS_CMD
            << START_TRAJ_VEL_CMD;

    const QVector<Command> listDisable = QVector<Command>()
            << STOP_GRIPPER_CMD
            << STOP_TRAJ_POS_CMD
            << STOP_TRAJ_VEL_CMD;

    int ind = progNames().indexOf(name);

    if(enable)
    {
        return listEnable.at(ind);
    }

    return listDisable.at(ind);
}

RobotScriptManager::RobotScriptManager(
        const QString &pcIpAddr,
        const QString &robotIpAddr,
        quint16 managerPort,
        quint16 scriptLoaderPort) :
    m_loader(robotIpAddr,scriptLoaderPort),
    ScriptManager(pcIpAddr,robotIpAddr,managerPort)
{
    m_states = QVector<bool>() << false << false << false;

    m_setState = m_node.advertiseService(
                "setScriptManagerState",
                &RobotScriptManager::setStateCallback,
                this);

    m_getStates = m_node.advertiseService(
                "getScriptManagerStates",
                &RobotScriptManager::getStatesCallback,
                this);

}

RobotScriptManager::~RobotScriptManager()
{
    ScriptManager::stop();
}


bool RobotScriptManager::start(const QString& scriptFilePath)
{
    if(m_started)
    {
        return false;
    }

    if(!m_loader.load(scriptFilePath))
    {
        return false;
    }

    m_states = QVector<bool>() << false << false << false;
    return ScriptManager::start();
}

bool RobotScriptManager::start()
{
    if(m_started)
    {
        return false;
    }

    if(!m_loader.load())
    {
        return false;
    }

    m_states = QVector<bool>() << false << false << false;
    return ScriptManager::start();
}

bool RobotScriptManager::send(const QString& name, bool enable)
{
    if(!m_started)
    {
        qCritical("Error: Script not started!");
        return false;
    }

    if(!progNames().contains(name))
    {
        qCritical("Error: Unknown script programm name '%s'",name.toLatin1().data());
        return false;
    }

    Command cmd = getCommand(name,enable);
    Response resp;

    if(enable && getState(name))
    {
        qCritical("Error: Send cmd '%s' ignored. Already started.",
                  convCommand(cmd).toLatin1().data());
        return false;
    }


    if(!ScriptManager::send(cmd,&resp,5000))
    {
        qCritical("Error: Send cmd '%s' failed",
                  convCommand(cmd).toLatin1().data());
        return false;
    }

    if(enable && resp != OK_RESP)
    {
        qCritical("Error: Send cmd '%s' failed.",
                  convCommand(cmd).toLatin1().data());
        qCritical("Script couldn't connect to server.");
        return false;
    }

    qDebug("Send cmd '%s' was successful.",
             convCommand(cmd).toLatin1().data());

    setState(name,enable);
    return true;
}

void RobotScriptManager::setState(const QString& name, bool enable)
{
    int ind = progNames().indexOf(name);
    m_states[ind] = enable;
}

bool RobotScriptManager::getState(const QString& name)
{
    int ind = progNames().indexOf(name);
    return m_states.at(ind);
}

bool RobotScriptManager::setStateCallback(
        tum_ics_ur_robot_msgs::setScriptManagerState::Request &req,
        tum_ics_ur_robot_msgs::setScriptManagerState::Response &res)
{
    QString name = QString(req.name.c_str());
    bool enable = req.enable;

    if(!send(name,enable))
    {
        ROS_ERROR("setScriptManagerState: set %s, %d failed.",
                  name.toLatin1().data(),
                  enable);
        res.ok = false;
        return true;
    }
    res.ok = true;
    return true;
}

bool RobotScriptManager::getStatesCallback(
        tum_ics_ur_robot_msgs::getScriptManagerStates::Request &req,
        tum_ics_ur_robot_msgs::getScriptManagerStates::Response &res)
{
    int size = progNames().size();
    res.names.resize(size);
    res.enabled.resize(size);

    QVector<QString> names = progNames();

    for(int i=0; i<size; i++)
    {
        res.names[i] = names.at(i).toStdString();
        res.enabled[i] = m_states.at(i);
    }

    return true;
}

}
