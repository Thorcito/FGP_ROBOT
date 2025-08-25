#ifndef SKILLSERVER_H
#define SKILLSERVER_H

#include <tum_ics_ur_robot_manager/SkillManager.h>

#include <tum_ics_msgs/TumCommon.h>

#include <tum_ics_msgs/PlaySkill.h>
#include <tum_ics_msgs/StopSkill.h>
#include <tum_ics_msgs/GetSkillState.h>
#include <tum_ics_msgs/ActiveSkill.h>
#include <tum_ics_msgs/GetActiveSkill.h>

#include <tum_ics_lacquey_gripper_msgs/setGripperState.h>
#include <tum_ics_lacquey_gripper_msgs/getGripperState.h>


namespace tum_ics_ur_robot_manager{

class SkillServer
{
public:

private:
    ros::NodeHandle m_nh;
    ros::ServiceServer m_playSkillSrv;
    ros::ServiceServer m_stopSkillSrv;
    ros::ServiceServer m_skillStateSrv;

    ros::ServiceServer m_getActiveSkillSrv;

    ros::Publisher     m_activeSkillPub;

    ros::ServiceClient m_setGripperState;
    ros::ServiceClient m_getGripperState;   

    SkillManager *m_skillManager;

    bool m_gripperParamAvailable; 

public:
    SkillServer(SkillManager *sm);
    ~SkillServer();

    bool playSkillCallBack(tum_ics_msgs::PlaySkill::Request &req,
                           tum_ics_msgs::PlaySkill::Response &res);
    bool stopSkillCallBack(tum_ics_msgs::StopSkill::Request &req,
                           tum_ics_msgs::StopSkill::Response &res);
    bool skillStateCallBack(tum_ics_msgs::GetSkillState::Request &req,
                           tum_ics_msgs::GetSkillState::Response &res);

    bool getActiveSkillCallBack(tum_ics_msgs::GetActiveSkill::Request &req,
                           tum_ics_msgs::GetActiveSkill::Response &res);
    void init();

    void publish();

private:
    bool getParamString(QString& str, const QString& param);
    bool hasParam(const QString& param);

};



} //ns robot_manager




#endif // SKILLSERVER_H
