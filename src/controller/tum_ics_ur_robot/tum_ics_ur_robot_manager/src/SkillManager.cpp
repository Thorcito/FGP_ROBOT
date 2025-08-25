#include <tum_ics_ur_robot_manager/SkillManager.h>

#include <unistd.h>
#include <QElapsedTimer>
#include <math.h>

namespace tum_ics_ur_robot_manager{

SkillManager::SkillManager(
        const ControllerList& ctrls,
        const SkillList& skills,
        Robot *robot,
        QObject* parent) :
    QThread(parent),
    m_ctrls(ctrls),
    m_skills(skills),
    m_robot(robot),
    m_transMgr(ctrls,skills,robot),
    m_started(false),
    m_state(NOT_RUNNING_STATE),
    m_tauThresh(100.0),
    m_updateCtrlWeights(false)
{
    create();
}

SkillManager::SkillManager(
        const QList<Controller*> ctrls,
        const SkillList& skills,
        Robot* robot,
        QObject* parent) :
    QThread(parent),
    m_ctrls(ControllerList(ctrls)),
    m_skills(skills),
    m_robot(robot),
    m_transMgr(ControllerList(ctrls),skills,robot),
    m_started(false),
    m_state(NOT_RUNNING_STATE),
    m_tauThresh(100.0),
    m_updateCtrlWeights(false)
{
    create();
}

SkillManager::~SkillManager()
{
    stop();
    QElapsedTimer timer;
    timer.start();

    while(m_started && timer.elapsed() < 5000)
    {
        ::usleep(50*1000);
    }

    if(!m_started)
    {
        return;
    }

    if(!QThread::wait(500))
    {
        QThread::terminate();
        QThread::wait();
        qWarning("SkillManager::~SkillManager(): Terminated thread.");
    }
}


bool SkillManager::setRobot(Robot* robot)
{
    if(m_started)
    {
        return false;
    }

    m_transMgr.setRobot(robot);
    m_robot = robot;
    return true;
}

bool SkillManager::setControllers(const ControllerList& ctrls)
{
    if(m_started)
    {
        return false;
    }

    m_transMgr.setControllers(ctrls);

    m_ctrls = ctrls;
    m_skills.clear();
    return true;
}

bool SkillManager::setControllers(const QVector<Controller*> ctrls)
{
    return setControllers(ControllerList(ctrls));
}

bool SkillManager::setControllers(const QList<Controller*> ctrls)
{
    return setControllers(ControllerList(ctrls));
}

bool SkillManager::setSkills(const SkillList& skills)
{
    if(m_started)
    {
        return false;
    }

    if(m_ctrls.isEmpty())
    {
        return false;
    }

    m_transMgr.setSkills(skills);

    m_skills = skills;
    m_skills.update(m_ctrls.names());
    return true;
}

bool SkillManager::setSkills(const QString& paramRoot)
{
    return setSkills(SkillList(paramRoot));
}

bool SkillManager::requestSkillTransition()
{
    return requestSkillTransition(m_nextSkill);
}

bool SkillManager::requestSkillTransition(const QString& nextSkillName)
{
    return requestSkillTransition(m_skills.at(nextSkillName));
}

bool SkillManager::requestSkillTransition(const Skill& nextSkill)
{
    if(!m_started)
    {
        return false;
    }

    if(!m_skills.hasSkill(nextSkill))
    {
        qDebug("SkillManager::requestSkillTransition(): Refused: Invalid next skill.");
        return false;
    }

    m_mutex.lock();
    if(m_state != RUNNING_STATE)
    {
        qDebug("SkillManager::requestSkillTransition(): Refused: Skin manager is busy.");
        m_mutex.unlock();
        return false;
    }

    m_nextSkill = nextSkill;
    m_state = START_SKILL_TRANSITION_STATE;
    m_mutex.unlock();

    qDebug("SkillManager::requestSkillTransition(): Skill transition accepted.");
    return true;
}

bool SkillManager::setNextSkill(const QString& nextSkillName)
{
    return setNextSkill(m_skills.at(nextSkillName));
}

bool SkillManager::setNextSkill(const Skill& nextSkill)
{
    if(!m_started)
    {
        return false;
    }

    if(!m_skills.hasSkill(nextSkill))
    {
        qDebug("SkillManager::setNextSkill(): Refused: Invalid next skill.");
        return false;
    }

    m_mutex.lock();
    if(m_state != RUNNING_STATE)
    {
        qDebug("SkillManager::setNextSkill(): Refused: Skin manager is busy.");
        m_mutex.unlock();
        return false;
    }

    m_nextSkill = nextSkill;
    m_mutex.unlock();

    qDebug("SkillManager::setNextSkill(): Next skill accepted.");
    return true;
}

bool SkillManager::setNextSkill(const QString& nextSkillName,
                                const Tum::VectorDOFd& goal,
                                double duration)
{
    QVector<double> temp;
    for(int i=0; i<STD_DOF; i++)
    {
        temp.append(goal(i));
    }

    return setNextSkill(nextSkillName,temp,duration);
}

bool SkillManager::setNextSkill(const Skill& nextSkill,
                                const Tum::VectorDOFd& goal,
                                double duration)
{
    QVector<double> temp;
    for(int i=0; i<STD_DOF; i++)
    {
        temp.append(goal(i));
    }

    return setNextSkill(nextSkill,temp,duration);
}

bool SkillManager::setNextSkill(const QString& nextSkillName,
                  const QVector<double>& goal, double duration)
{
    return setNextSkill(m_skills.at(nextSkillName),goal,duration);
}

bool SkillManager::setNextSkill(const Skill& nextSkill,
                  const QVector<double>& goal,
                  double duration)
{
    if(!m_started)
    {
        return false;
    }

    if(goal.size() != 6)
    {
        qDebug("SkillManager::setNextSkill(): Refused: Invalid goal dimension: %d",goal.size());
        return false;
    }


    if(!m_skills.hasSkill(nextSkill))
    {
        qDebug("SkillManager::setNextSkill(): Refused: Invalid skill id.");
        return false;
    }

    m_mutex.lock();
    if(m_state != RUNNING_STATE)
    {
        qDebug("SkillManager::setNextSkill(): Refused: Skin manager is busy.");
        m_mutex.unlock();
        return false;
    }

//    const QVector<QString>& load = nextSkill.loadCtrls();

//    for(int i=0; i<load.size();i++)
//    {
//        Controller* c = m_ctrls.at(load.at(i));

//        if(c->getCtrlSpace() == tum_ics_ur_robot_lli::RobotControllers::JOINT_SPACE &&
//                c->getCtrlType() == tum_ics_ur_robot_lli::RobotControllers::SPLINE_TYPE)
//        {
//            Tum::VectorDOFd qGoal;
//            for(int i=0; i<STD_DOF; i++)
//            {
//                qGoal(i) = DEG2RAD(goal.at(i));
//            }

//            c->setGoalJoints(qGoal);
//        }

//        if(c->getCtrlSpace() == tum_ics_ur_robot_lli::RobotControllers::CARTESIAN_SPACE &&
//                c->getCtrlType() == tum_ics_ur_robot_lli::RobotControllers::SPLINE_TYPE)
//        {
//            Tum::Vector6d XGoal;
//            for(int i=0; i<STD_DOF; i++)
//            {
//                XGoal(i) = goal.at(i);
//            }

//            c->setGoalPose(XGoal);
//        }

//        c->setGoalTime(duration);
//    }

    const QVector<QString>& load = nextSkill.loadCtrls();

    for(int i=0; i<load.size();i++)
    {
        Controller* c = m_ctrls.at(load.at(i));

        if(c->getCtrlType() == tum_ics_ur_robot_lli::RobotControllers::SPLINE_TYPE)
        {

            qDebug("Skill manager: skill %s, goal = [%f, %f, %f, %f, %f, %f]",
                   nextSkill.name().toLatin1().data(),
                   goal.at(0),
                   goal.at(1),
                   goal.at(2),
                   goal.at(3),
                   goal.at(4),
                   goal.at(5));
            break;
        }
    }

    m_goal = goal;
    m_duration = duration;
    m_nextSkill = nextSkill;
    m_mutex.unlock();

    qDebug("SkillManager::setNextSkill(): Skill accepted.");
    return true;
}

bool SkillManager::start()
{
    if(m_started)
    {
        return false;
    }

    if(!isReadyToStart())
    {
        return false;
    }

    m_state = NOT_RUNNING_STATE;
    m_started = true;

    QThread::start();

    return true;
}

void SkillManager::stop()
{
    if(!m_started)
    {
        return;
    }

    m_started = false;
}

bool SkillManager::isReadyToStart() const
{
    if(m_started)
    {
        return false;
    }

    if(m_ctrls.isEmpty())
    {
        return false;
    }

    if(m_skills.isUndefined())
    {
        return false;
    }

    if(!checkSkills())
    {
        return false;
    }

    if(m_robot == 0)
    {
        return false;
    }

    return true;
}

bool SkillManager::isRunning() const
{
    return (m_state != NOT_RUNNING_STATE);
}

bool SkillManager::isFinished() const
{
    return (m_state == NOT_RUNNING_STATE);
}

const SkillList& SkillManager::skillList() const
{
    return m_skills;
}

Skill SkillManager::currentSkill()
{
    m_mutex.lock();
    Skill s = m_currSkill;
    m_mutex.unlock();

    return s;
}

Skill SkillManager::nextSkill()
{
    m_mutex.lock();
    Skill s = m_nextSkill;
    m_mutex.unlock();

    return s;
}

bool SkillManager::skillFinished(const QString& skill) const
{
    if(!m_skills.hasSkill(skill))
    {
        return false;
    }

    return skillFinished(m_skills.at(skill));
}

bool SkillManager::skillFinished(const Skill& skill) const
{
    const QVector<QString>& ctrls = skill.loadCtrls();
    for(int i=0; i<ctrls.size(); i++)
    {
        Controller* c = m_ctrls.at(ctrls.at(i));
        if(c->isFinished())
        {
            return true;
        }
    }

    return false;
}

bool SkillManager::skillRunning(const QString& skill) const
{
    if(!m_skills.hasSkill(skill))
    {
        return false;
    }

    return skillRunning(m_skills.at(skill));
}

bool SkillManager::skillRunning(const Skill& skill) const
{
    const QVector<QString>& ctrls = skill.loadCtrls();
    for(int i=0; i<ctrls.size(); i++)
    {
        Controller* c = m_ctrls.at(ctrls.at(i));
        if(!c->isRunning())
        {
            return false;
        }
    }

    return true;
}

bool SkillManager::skillFailed(const QString& skill) const
{
    if(!m_skills.hasSkill(skill))
    {
        return false;
    }

    return skillFailed(m_skills.at(skill));
}

bool SkillManager::skillFailed(const Skill& skill) const
{
    const QVector<QString>& ctrls = skill.loadCtrls();
    for(int i=0; i<ctrls.size(); i++)
    {
        Controller* c = m_ctrls.at(ctrls.at(i));
        if(c->hasFailed())
        {
            return true;
        }
    }
    return false;
}

bool SkillManager::skillStarted(const QString& skill) const
{
    if(!m_skills.hasSkill(skill))
    {
        return false;
    }

    return skillStarted(m_skills.at(skill));
}

bool SkillManager::skillStarted(const Skill& skill) const
{
    const QVector<QString>& ctrls = skill.loadCtrls();
    for(int i=0; i<ctrls.size(); i++)
    {
        Controller* c = m_ctrls.at(ctrls.at(i));
        if(!c->isStarted())
        {
            return false;
        }
    }
    return true;
}

void SkillManager::create()
{
    m_skills.update(m_ctrls.names());

    QString param = "~skin_spline_skills/tau_thresh";
    if(hasParam(param))
    {
        getParamDouble(m_tauThresh,param);
        ROS_WARN("SkillManager: skin-spline skill weight adj. thresh: %f", m_tauThresh);
    }
}

void SkillManager::run()
{   
    // wait until robot is ready
    while(!isRobotReady())
    {
        if(!m_started)
        {
            return;
        }
        usleep(100*1000);
    }

    // load the idle skill to the robot
    if(!loadSkillToRobot("IDLE"))
    {
        qCritical("SkillManager::run(): Exit: Couldn't load IDLE skill to robot.");
        m_started = false;
        return;
    }

    qDebug("SkillManager: '%s' skill loaded.", m_currSkill.name().toLatin1().data());

    // idle skill is running
    updateControllerTypeLists();
    m_state = RUNNING_STATE;

    while(m_started)
    {
        m_mutex.lock();
        if(m_state == START_SKILL_TRANSITION_STATE)
        {
            m_transMgr.start(SkillTransition(m_currSkill,m_nextSkill),m_goal,m_duration);
            qDebug("Started skill transition: '%s'",
                   m_transMgr.skillTransition().name().toLatin1().data());
            m_state = IN_SKILL_TRANSITION_STATE;
        }
        else if(m_state == IN_SKILL_TRANSITION_STATE)
        {
            m_transMgr.update();
            if(m_transMgr.isFinished())
            {
                qDebug("Finished skill transition: '%s'",
                       m_transMgr.skillTransition().name().toLatin1().data());

                qDebug("New skill running: '%s'",
                       m_nextSkill.name().toLatin1().data());

                m_currSkill = m_nextSkill;
                m_nextSkill = m_skills.at("IDLE");
                updateControllerTypeLists();
                m_state = RUNNING_STATE;
            }
        }
        else if(m_state == RUNNING_STATE)
        {
            updateControllerWeights();

            if(skillFinished(m_currSkill))
            {
                m_state = START_SKILL_TRANSITION_STATE;
            }

            if(skillFailed(m_currSkill))
            {
                m_nextSkill = m_skills.at("IDLE");
                m_state = START_SKILL_TRANSITION_STATE;
            }
        }
        m_mutex.unlock();

        usleep(50*1000);
    }

    // wait if transition is still in progress
    if(m_state == IN_SKILL_TRANSITION_STATE)
    {
        while(!m_transMgr.isFinished())
        {
            m_transMgr.update();
            usleep(10*1000);
        }
        qDebug("Finished skill transition: '%s'",
               m_transMgr.skillTransition().name().toLatin1().data());
        m_currSkill = m_nextSkill;
        m_nextSkill = m_skills.at("IDLE");
    }

    // transition to idle skill, then exit
    m_nextSkill = m_skills.at("IDLE");

    m_transMgr.start(SkillTransition(m_currSkill,m_nextSkill),m_goal,m_duration);
    while(!m_transMgr.isFinished())
    {
        m_transMgr.update();
        usleep(10*1000);
    }

    usleep(100*1000);
    if(unloadSkillFromRobot(m_currSkill))
    {
        qDebug("SkillManager: Unloaded IDLE skill from robot.");
    }
    else
    {
        qWarning("SkillManager: Couldn't unload IDLE skill from robot.");
    }

    if(m_robot->isIdle())
    {
        qDebug("SkillManager: Robot is idle.");
    }

    m_state = NOT_RUNNING_STATE;
    qDebug("SkillManager: Thread exited.");
}

bool SkillManager::loadSkillToRobot(const QString& skillName)
{
    return loadSkillToRobot(m_skills.at(skillName));
}

bool SkillManager::loadSkillToRobot(const Skill& skill)
{
    if(!m_skills.hasSkill(skill))
    {
        qWarning("SkillManager::loadSkillToRobot(): Failed: Invalid skill %s",
                 skill.name().toLatin1().data());

        return false;
    }

    const QVector<QString>& load = skill.loadCtrls();

    for(int i=0; i<load.size(); i++)
    {
        Controller* c = m_ctrls.at(load.at(i));
        m_robot->addWhileRunning(c);

        if(!waitForInitialized(c))
        {
            m_robot->removeWhileRunning(c);
            qWarning("SkillManager::loadSkillToRobot(): Starting skill failed.");
            unloadSkillFromRobot(skill);
            return false;
        }

        c->start();
        while(!c->isRunning())
        {
            ::usleep(10*1000);
        }
    }

    m_currSkill = skill;
    m_nextSkill = m_skills.at("IDLE");
    return true;
}

bool SkillManager::unloadSkillFromRobot(const QString& skillName)
{
    return unloadSkillFromRobot(m_skills.at(skillName));
}

bool SkillManager::unloadSkillFromRobot(const Skill& skill)
{
    if(!m_skills.hasSkill(skill))
    {
        qWarning("SkillManager::unloadSkillFromRobot(): Failed: Invalid skill %s",
                 skill.name().toLatin1().data());

        return false;
    }

    const QVector<QString>& unload = skill.unloadCtrls();

    for(int i=0; i<unload.size(); i++)
    {
        Controller* c = m_ctrls.at(unload.at(i));
        if(!c->isRunning())
        {
            continue;
        }

        c->stop();

        while(!c->isFinished())
        {
            ::usleep(10*1000);
        }
        m_robot->removeWhileRunning(c);
    }

    m_currSkill = Skill::Undefined();
    m_nextSkill = Skill::Undefined();
    return true;
}

bool SkillManager::isRobotReady() const
{
    if(m_robot == 0)
    {
        return false;
    }

    if(!m_robot->isRunning())
    {
        return false;
    }

    if(!m_robot->isIdle())
    {
        return false;
    }

    return true;
}

bool SkillManager::checkSkills() const
{
    const QVector<Skill>& skills = m_skills.skills();

    // go through skills
    for(int i=0; i<skills.size(); i++)
    {
        const Skill& s = skills.at(i);
        const QVector<QString>& load = s.loadCtrls();

        // ctrl intf of first ctrl in list
        ControlInterface intf = m_ctrls.at(load.at(0))->getCtrlIntf();

        // check other ctrl intfs
        for(int i=1; i<load.size(); i++)
        {
            Controller* c = m_ctrls.at(load.at(i));
            if(intf != c->getCtrlIntf())
            {
                qWarning("SkillManager::checkSkills(): Invalid controller set in skill %s.",
                         s.name().toLatin1().data());
                return false;
            }
        }
    }

    return true;
}

bool SkillManager::waitForInitialized(Controller* c, int timeout)
{
    QElapsedTimer timer;
    timer.start();

    // wait with timeout till initialized
    while(!c->isInitialized())
    {
        if(timer.elapsed() > timeout)
        {
            return false;
        }
    }

    return true;
}

void SkillManager::updateControllerTypeLists()
{
    m_updateCtrlWeights = false;
    m_skinCtrls.clear();
    m_splineCtrls.clear();

    const QVector<QString>& ctrls = m_currSkill.loadCtrls();

    for(int i=0; i<ctrls.size();i++)
    {
        Controller* c = m_ctrls.at(ctrls.at(i));

        // init controller weight
        c->setWeight(1.0);

        if(c->getCtrlType() == tum_ics_ur_robot_lli::RobotControllers::SPLINE_TYPE)
        {
            m_splineCtrls.append(c);
        }

        if(c->getCtrlType() == tum_ics_ur_robot_lli::RobotControllers::SKIN_TYPE)
        {
            m_skinCtrls.append(c);
        }
    }

    if(!m_splineCtrls.isEmpty() && !m_skinCtrls.isEmpty())
    {
        m_updateCtrlWeights = true;
        for(int i=0; i<m_skinCtrls.size(); i++)
        {
            Controller* c = m_skinCtrls.at(i);
            ROS_WARN_STREAM("changed controller weight: " << c->name().toLatin1().data());
            c->setWeight(2.0);
        }

    }
}

void SkillManager::updateControllerWeights()
{
    if(!m_updateCtrlWeights)
    {
        return;
    }

    Tum::VectorDOFd tau = Tum::VectorDOFd::Zero();

//    for(int i=0; i<m_skinCtrls.size(); i++)
//    {
//        Controller* c = m_skinCtrls.at(i);
//        c->setWeight(10.0);
////        tau += c->getTau();
//    }

//    double weight = calcControllerWeight(tau);

//    for(int i=0; i<m_splineCtrls.size(); i++)
//    {
//        Controller* c = m_splineCtrls.at(i);
//        c->setWeight(weight);
//    }
}

double SkillManager::calcControllerWeight(const Tum::VectorDOFd& tau)
{
    double weight = 1.0;
    double tauNorm = tau.norm();
    double tauThreshAbs = abs(m_tauThresh);

    if(tauNorm > tauThreshAbs)
    {
        ROS_WARN_STREAM("Skill: SkinTau: "<<tau.transpose());
        ROS_WARN_STREAM("Spline ctrl " << m_splineCtrls.size());
        weight = 1.0 / (1 + abs(tauNorm - tauThreshAbs));
        ROS_WARN_STREAM("Skill: Spline weight: "<<weight);
    }


    return weight;
}

bool SkillManager::hasParam(const QString& param)
{
    std::string p = param.toStdString();
    return ros::param::has(p);
}

bool SkillManager::getParamDouble(double& val, const QString& param)
{
    val = 0.0;
    std::string p = param.toStdString();

    if(!ros::param::has(p))
    {
        ROS_ERROR("Parameter '%s' is missing.", p.c_str());
        return false;
    }

    if(!ros::param::get(p,val))
    {
        ROS_ERROR("Couldn't get parameter '%s'.", p.c_str());
        return false;
    }
    return true;
}

}
