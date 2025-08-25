#ifndef UR_ROBOT_MANAGER_SKILLTRANSITIONMANAGER_H
#define UR_ROBOT_MANAGER_SKILLTRANSITIONMANAGER_H

#include <QVector>
#include <QString>
#include <QElapsedTimer>

#include <tum_ics_ur_robot_manager/Skill.h>
#include <tum_ics_ur_robot_manager/SkillList.h>
#include <tum_ics_ur_robot_manager/SkillTransition.h>

#include <tum_ics_ur_robot_lli/RobotControllers/Controller.h>
#include <tum_ics_ur_robot_lli/RobotControllers/ControllerList.h>
#include <tum_ics_ur_robot_lli/Robot/RobotArm.h>

namespace tum_ics_ur_robot_manager{

class SkillTransitionManager
{

private:
    typedef tum_ics_ur_robot_lli::RobotControllers::ControllerList ControllerList;
    typedef tum_ics_ur_robot_lli::Robot::RobotArm Robot;
    typedef tum_ics_ur_robot_lli::RobotControllers::Controller Controller;

    enum State
    {
        STOP_CTRL_STATE = 0,
        WAIT_FOR_FINISHING_CTRL_STATE,
        START_CTRL_STATE,
        WAIT_FOR_RUNNING_CTRL_STATE,
        TRANS_FINISHED_STATE
    };

    ControllerList m_ctrls;
    SkillList m_skills;
    Robot* m_robot;

    bool m_started;

    Controller* m_currCtrl;
    SkillTransition m_trans;
    State m_state;
    int m_ctrlInd;

    QVector<double> m_goal;
    double m_duration;

    QElapsedTimer m_timer;


public:
    explicit SkillTransitionManager(
            const ControllerList& ctrls = ControllerList(),
            const SkillList& skills = SkillList(),
            Robot* robot = 0);
    ~SkillTransitionManager();

    bool setRobot(Robot* robot);

    // if you set the controllers, the current skill list will be deleted
    bool setControllers(const ControllerList& ctrls);

    // fails if no controllers are set yet
    // updates skill list according to controllers
    bool setSkills(const SkillList& skills);

    // doesn't start if not properly configured
    // doesn't start if skill transition is not possible
    bool start(const SkillTransition& st,
               const QVector<double>& goal,
               double duration);

    // this needs to be called repeatedly to handle the skill transition
    //  returns true until finished
    bool update();

    bool isReady() const;
    bool isRunning() const;
    bool isFinished() const;

    const SkillTransition& skillTransition() const;

private:
    bool checkIfReady() const;

};

}


#endif // UR_ROBOT_MANAGER_SKILLTRANSITIONMANAGER_H
