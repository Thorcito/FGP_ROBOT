#ifndef UR_ROBOT_MANAGER_SKILLMANAGER_H
#define UR_ROBOT_MANAGER_SKILLMANAGER_H

#include <QVector>
#include <QString>
#include <QThread>

#include <Math/EigenDefs.h>
#include <tum_ics_ur_robot_manager/SkillTransitionManager.h>

namespace tum_ics_ur_robot_manager{

class SkillManager : private QThread
{
public:

private:
    typedef tum_ics_ur_robot_lli::RobotControllers::ControllerList ControllerList;
    typedef tum_ics_ur_robot_lli::Robot::RobotArm Robot;
    typedef tum_ics_ur_robot_lli::RobotControllers::Controller Controller;
    typedef tum_ics_ur_robot_lli::RobotControllers::ControlInterface ControlInterface;

    enum State
    {
        NOT_RUNNING_STATE = 0,
        RUNNING_STATE,
        START_SKILL_TRANSITION_STATE,
        IN_SKILL_TRANSITION_STATE
    };

    ControllerList m_ctrls;
    SkillList m_skills;
    Robot* m_robot;

    SkillTransitionManager m_transMgr;

    bool m_started;

    State m_state;
    Skill m_currSkill;
    Skill m_nextSkill;

    // tau threshold for spline-skin skills
    double m_tauThresh;
    bool m_updateCtrlWeights;

    QVector<Controller*> m_skinCtrls;
    QVector<Controller*> m_splineCtrls;

    QMutex m_mutex;

    double m_duration;
    QVector<double> m_goal;

public:
    explicit SkillManager(
            const ControllerList& ctrls = ControllerList(),
            const SkillList& skills = SkillList(),
            Robot* robot = 0,
            QObject* parent=0);

    explicit SkillManager(
            const QList<Controller*> ctrls,
            const SkillList& skills = SkillList(),
            Robot* robot = 0,
            QObject* parent=0);

    ~SkillManager();

    // fails if manager is started
    bool setRobot(Robot* robot);

    // if you set the controllers, the current skill list will be deleted
    bool setControllers(const ControllerList& ctrls);
    bool setControllers(const QVector<Controller*> ctrls);
    bool setControllers(const QList<Controller*> ctrls);

    // fails if no controllers are set yet
    // updates skill list according to controllers
    bool setSkills(const SkillList& skills);
    bool setSkills(const QString& paramRoot="~skill_list");

    // will be refused if not running or if skill transition already takes place
    // stops the current skill
    // refused if transition is already in progress
    bool requestSkillTransition();
    bool requestSkillTransition(const QString& nextSkillName);
    bool requestSkillTransition(const Skill& nextSkill);

    // sets the next skill which should be used if current skill has finished
    // this doesn't stop the current skill
    // refused if transition is already in progress
    bool setNextSkill(const QString& nextSkillName);
    bool setNextSkill(const Skill& nextSkill);

    bool setNextSkill(const QString& nextSkillName, const Tum::VectorDOFd& goal, double duration=10.0);
    bool setNextSkill(const Skill& nextSkill, const Tum::VectorDOFd& goal, double duration=10.0);

    bool setNextSkill(const QString& nextSkillName, const QVector<double>& goal, double duration=10.0);
    bool setNextSkill(const Skill& nextSkill, const QVector<double>& goal, double duration=10.0);

    // will be refused if not ready or already started
    bool start();

    // transition to idle skill, then exit thread.
    void stop();

    bool isReadyToStart() const;

    bool isRunning() const;
    bool isFinished() const;

    const SkillList& skillList() const;

    Skill currentSkill();
    Skill nextSkill();

    // skill is finished if one controller has finished
    bool skillFinished(const QString& skill) const;
    bool skillFinished(const Skill& skill) const;

    // all controllers are running
    bool skillRunning(const QString& skill) const;
    bool skillRunning(const Skill& skill) const;

    // one controller failed
    bool skillFailed(const QString& skill) const;
    bool skillFailed(const Skill& skill) const;

    // all controllers started
    bool skillStarted(const QString& skill) const;
    bool skillStarted(const Skill& skill) const;


private:
    void create();

    // run skill, execute skill transition, auto skill transition to
    //   idle skill when skill is finished
    void run();

    // blocking until loaded and started
    bool loadSkillToRobot(const QString& skillName);
    bool loadSkillToRobot(const Skill& skill);

    bool unloadSkillFromRobot(const QString& skillName);
    bool unloadSkillFromRobot(const Skill& skill);

    // the robot has to have an empty controller list and needs to be running
    bool isRobotReady() const;

    // checks if controller sets of skill are valid (have same intf type)
    bool checkSkills() const;

    // wait with timeout till initialized
    bool waitForInitialized(Controller* c, int timeout = 100); // ms

    void updateControllerTypeLists();

    // if there is a controller of type spline and skin in list in the list, then
    //  use tau of skin to adjust weights of spline controllers
    void updateControllerWeights();
    double calcControllerWeight(const Tum::VectorDOFd& tau);

    bool hasParam(const QString& param);
    bool getParamDouble(double& val, const QString& param);

};

}


#endif // UR_ROBOT_MANAGER_SKILLMANAGER_H
