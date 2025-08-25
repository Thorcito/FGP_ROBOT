#include <tum_ics_ur_robot_manager/SkillTransitionManager.h>

namespace tum_ics_ur_robot_manager{

SkillTransitionManager::SkillTransitionManager(
        const ControllerList& ctrls,
        const SkillList& skills,
        Robot *robot) :
    m_ctrls(ctrls),
    m_skills(skills),
    m_robot(robot),
    m_started(false)
{
    m_skills.update(m_ctrls.names());
}


SkillTransitionManager::~SkillTransitionManager()
{

}

bool SkillTransitionManager::setRobot(Robot* robot)
{
    if(m_started)
    {
        return false;
    }

    m_robot = robot;
    return true;
}

bool SkillTransitionManager::setControllers(const ControllerList& ctrls)
{
    if(m_started)
    {
        return false;
    }

    m_ctrls = ctrls;
    m_skills.clear();
    return true;
}

bool SkillTransitionManager::setSkills(const SkillList& skills)
{
    if(m_started)
    {
        return false;
    }

    if(m_ctrls.isEmpty())
    {
        return false;
    }

    m_skills = skills;
    m_skills.update(m_ctrls.names());
    return true;
}

bool SkillTransitionManager::start(const SkillTransition& st, const QVector<double> &goal, double duration)
{
    m_trans = st;

    if(m_started)
    {
        return false;
    }

    if(!checkIfReady())
    {
        return false;
    }

    if(!m_trans.isValid(m_skills))
    {
        qDebug("SkillTransitionManager::start(): Error invalid skills in requested skill transition: '%s'",
               m_trans.name().toLatin1().data());
        return false;
    }

    m_goal = goal;
    m_duration = duration;

    // check if controllers of next skill are all of the same type
    const QVector<QString>& ln = m_trans.next().loadCtrls();
    tum_ics_ur_robot_lli::RobotControllers::ControlInterface intf = m_ctrls.at(ln.at(0))->getCtrlIntf();

    for(int i=1; i<ln.size(); i++)
    {
        Controller* c = m_ctrls.at(ln.at(i));
        if(intf != c->getCtrlIntf())
        {
            qWarning("SkillTransitionManager::start(): Invalid controller set in next skill.");
            return false;
        }
    }

    if(m_trans.isIdentityTansition())
    {
        return true;
    }

    m_state = STOP_CTRL_STATE;
    m_ctrlInd = 0;
    m_started = true;
    return true;
}

bool SkillTransitionManager::update()
{
    if(!m_started)
    {
        return false;
    }

    const QVector<QString> stop = m_trans.stopCtrls();
    const QVector<QString> start = m_trans.startCtrls();
    const QVector<QString> keep = m_trans.keepCtrls();

    if(m_state == STOP_CTRL_STATE)
    {
        if(stop.isEmpty())
        {
            m_ctrlInd = 0;
            m_state = START_CTRL_STATE;
        }
        else
        {
            m_currCtrl = m_ctrls.at(stop.at(m_ctrlInd));
            m_currCtrl->stop();
            m_state = WAIT_FOR_FINISHING_CTRL_STATE;
        }
    }
    else if(m_state == WAIT_FOR_FINISHING_CTRL_STATE)
    {
        if(m_currCtrl->isFinished())
        {
            // unload the controller from the robot if not kept for next skill
            if(!keep.contains(stop.at(m_ctrlInd)))
            {
                m_robot->removeWhileRunning(m_currCtrl);
            }

            // next controller to stop
            m_ctrlInd++;
            if(m_ctrlInd < stop.size())
            {
                m_state = STOP_CTRL_STATE;
            }
            else
            {
                m_timer.start();
                m_ctrlInd = 0;
                m_state = START_CTRL_STATE;
            }
        }
    }
    else if(m_state == START_CTRL_STATE)
    {
        m_currCtrl = m_ctrls.at(start.at(m_ctrlInd));

        // reinitialize the virtual robot to avoid drifts
        m_robot->reinit();


        if(m_currCtrl->getCtrlType() == tum_ics_ur_robot_lli::RobotControllers::SPLINE_TYPE)
        {
            for(int i=0; i<start.size(); i++)
            {
                Controller* c = m_ctrls.at(start.at(i));
                if(c->getCtrlType() == tum_ics_ur_robot_lli::RobotControllers::SKIN_TYPE)
                {
                    m_currCtrl->enableTauExtFlag();
                    break;
                }
            }
        }


        // load controller if not already loaded
        if(!keep.contains(start.at(m_ctrlInd)))
        {
            m_robot->addWhileRunning(m_currCtrl);
        }


        if(m_currCtrl->getCtrlSpace() == tum_ics_ur_robot_lli::RobotControllers::JOINT_SPACE &&
                m_currCtrl->getCtrlType() == tum_ics_ur_robot_lli::RobotControllers::SPLINE_TYPE)
        {
            Tum::VectorDOFd qGoal;
            for(int i=0; i<STD_DOF; i++)
            {
                qGoal(i) = DEG2RAD(m_goal.at(i));
            }

            m_currCtrl->setGoalJoints(qGoal);
        }

        if(m_currCtrl->getCtrlSpace() == tum_ics_ur_robot_lli::RobotControllers::CARTESIAN_SPACE &&
                m_currCtrl->getCtrlType() == tum_ics_ur_robot_lli::RobotControllers::SPLINE_TYPE)
        {
            Tum::Vector6d XGoal;
            for(int i=0; i<STD_DOF; i++)
            {
                XGoal(i) = m_goal.at(i);
            }

            m_currCtrl->setGoalPose(XGoal);
        }

        m_currCtrl->setGoalTime(m_duration);


        m_currCtrl->start();
        m_state = WAIT_FOR_RUNNING_CTRL_STATE;
    }
    // TODO: is trapped here when we couldn't start one controller
    else if(m_state == WAIT_FOR_RUNNING_CTRL_STATE)
    {
        if(m_currCtrl->isRunning())
        {
            if(m_ctrlInd ==0)
            {
                qDebug("Time without any running controller: %llu ms",m_timer.elapsed());
            }
            m_ctrlInd++;
            if(m_ctrlInd < start.size())
            {
                m_state = START_CTRL_STATE;
            }
            else
            {
                m_state = TRANS_FINISHED_STATE;
            }
        }
    }
    else if(m_state == TRANS_FINISHED_STATE)
    {
        m_started = false;
    }

    return true;
}

bool SkillTransitionManager::isReady() const
{
    return checkIfReady();
}

bool SkillTransitionManager::isRunning() const
{
    return m_started;
}

bool SkillTransitionManager::isFinished() const
{
    return !m_started;
}

const SkillTransition& SkillTransitionManager::skillTransition() const
{
    return m_trans;
}

bool SkillTransitionManager::checkIfReady() const
{
    if(m_ctrls.isEmpty())
    {
        return false;
    }

    if(m_skills.isUndefined())
    {
        return false;
    }

    if(m_robot == 0)
    {
        return false;
    }

    if(!m_robot->isRunning())
    {
        return false;
    }

    return true;
}


}
