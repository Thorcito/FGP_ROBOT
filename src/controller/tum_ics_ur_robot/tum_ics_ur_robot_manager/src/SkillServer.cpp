#include<tum_ics_ur_robot_manager/SkillServer.h>

namespace tum_ics_ur_robot_manager{

SkillServer::SkillServer(SkillManager *sm):
    m_skillManager(sm),
    m_gripperParamAvailable(true)
{
    init();
}

SkillServer::~SkillServer()
{

}

void SkillServer::init()
{
    ROS_INFO_STREAM("Advertizing PlaySkill service");
    m_playSkillSrv=m_nh.advertiseService("PlaySkill",
                                         &tum_ics_ur_robot_manager::SkillServer::playSkillCallBack,this);

    ROS_INFO_STREAM("Advertizing StopSkill service");
    m_stopSkillSrv=m_nh.advertiseService("StopSkill",
                                         &tum_ics_ur_robot_manager::SkillServer::stopSkillCallBack,this);

    ROS_INFO_STREAM("Advertizing SkillState service");
    m_skillStateSrv=m_nh.advertiseService("SkillState",
                                          &tum_ics_ur_robot_manager::SkillServer::skillStateCallBack,this);

    ROS_INFO_STREAM("Advertizing GetActiveSkill service");
    m_getActiveSkillSrv=m_nh.advertiseService("GetActiveSkill",
                                          &tum_ics_ur_robot_manager::SkillServer::getActiveSkillCallBack,this);


    ROS_INFO_STREAM("Advertizing ActiveSkill publisher");
    m_activeSkillPub = m_nh.advertise<tum_ics_msgs::ActiveSkill>("ActiveSkill", 100);

    QString param = "~skill_server_params/gripper/setGripperState";

    QString setGripperState;
    if(!hasParam(param))
    {
        m_gripperParamAvailable = false;
    }
    else
    {
        getParamString(setGripperState,param);
        ROS_WARN_STREAM("setGripperState: " << setGripperState.toStdString());
        m_setGripperState = m_nh.serviceClient
                <tum_ics_lacquey_gripper_msgs::setGripperState>
                (setGripperState.toStdString());
    }


    param = "~skill_server_params/gripper/getGripperState";

    QString getGripperState;
    if(!hasParam(param))
    {
        m_gripperParamAvailable = false;
    }
    else
    {
        getParamString(getGripperState,param);
        ROS_WARN_STREAM("getGripperStateService: " << getGripperState.toStdString());
        m_getGripperState = m_nh.serviceClient
                <tum_ics_lacquey_gripper_msgs::getGripperState>
                (getGripperState.toStdString());
    }


    if(m_getGripperState)
    {
        ROS_INFO_STREAM("Gripper service params available.");
    }
    else
    {
        ROS_WARN_STREAM("Gripper service params NOT available.");
    }

    ROS_WARN_STREAM("Skill server attached to skill manager.");
}

void SkillServer::publish()
{
    QString currSkill = m_skillManager->currentSkill().name();
    tum_ics_msgs::ActiveSkill msg;

    msg.skill_id = currSkill.toStdString();

    m_activeSkillPub.publish(msg);
}


bool SkillServer::playSkillCallBack(tum_ics_msgs::PlaySkill::Request &req,
                                    tum_ics_msgs::PlaySkill::Response &res)
{
    if(!m_skillManager->isRunning())
    {
        res.request_result_id = Tum::Common::REJECTED_SKILLR;
        res.request_result_label = "REJECTED_SKILL: Skill manager not started.";
        return true;
    }

    QString skill = QString(req.skill_id.c_str());


    if(skill == "OPEN_GRIPPER" && m_gripperParamAvailable)
    {
        tum_ics_lacquey_gripper_msgs::setGripperState gripperSrv;
        gripperSrv.request.newState = "open";

        if(!m_setGripperState.call(gripperSrv))
        {
            res.request_result_id = Tum::Common::REJECTED_SKILLR;
            res.request_result_label = "REJECTED_SKILL: Gripper driver call to open gripper failed.";
            return true;
        }

        if(gripperSrv.response.ok == false)
        {
            res.request_result_id = Tum::Common::REJECTED_SKILLR;
            res.request_result_label = "REJECTED_SKILL: Gripper driver rejected open request.";
            return true;
        }

        res.request_result_id = Tum::Common::ACCEPTED_SKILLR;
        res.request_result_label = "ACCEPTED_SKILL: open gripper";

        return true;
    }

    if(skill == "CLOSE_GRIPPER" && m_gripperParamAvailable)
    {
        tum_ics_lacquey_gripper_msgs::setGripperState gripperSrv;
        gripperSrv.request.newState = "close";

        if(!m_setGripperState.call(gripperSrv))
        {
            res.request_result_id = Tum::Common::REJECTED_SKILLR;
            res.request_result_label = "REJECTED_SKILL: Gripper driver call to close gripper failed.";
            return true;
        }

        if(gripperSrv.response.ok == false)
        {
            res.request_result_id = Tum::Common::REJECTED_SKILLR;
            res.request_result_label = "REJECTED_SKILL: Gripper driver rejected close request.";
            return true;
        }

        res.request_result_id = Tum::Common::ACCEPTED_SKILLR;
        res.request_result_label = "ACCEPTED_SKILL: close gripper";
        return true;
    }

    QVector<double> goal;
    for(int i=0; i<req.pose_d.size(); i++)
    {
        goal.append(req.pose_d.at(i));
    }

    double duration = req.duration;

    if(m_skillManager->setNextSkill(skill,goal,duration))
    {
        QString currSkill = m_skillManager->currentSkill().name();
        if(currSkill == "IDLE")
        {
            if(!m_skillManager->requestSkillTransition())
            {
                res.request_result_id = Tum::Common::REJECTED_SKILLR;
                res.request_result_label = QString("REJECTED_SKILLR: skill transition").toStdString();
                return true;
            }
            else
            {
                res.request_result_id = Tum::Common::ACCEPTED_SKILLR;
                res.request_result_label = QString("ACCEPTED_SKILL: New skill transition to " + skill).toStdString();
                return true;
            }
        }

        res.request_result_id = Tum::Common::ACCEPTED_SKILLR;
        res.request_result_label = QString("ACCEPTED_SKILL: New pending skill " + skill + ". To execute the skill, stop first"
                                           + currSkill).toStdString();
        return true;
    }
    else
    {
        res.request_result_id = Tum::Common::REJECTED_SKILLR;
        res.request_result_label = QString("REJECTED_SKILL: " + skill).toStdString();
        ROS_ERROR("Setting skill '%s' failed.",skill.toLatin1().data());
        return true;
    }
}


bool SkillServer::stopSkillCallBack(tum_ics_msgs::StopSkill::Request &req,
                                    tum_ics_msgs::StopSkill::Response &res)
{
    QString currSkill = m_skillManager->currentSkill().name();
    QString skill = QString(req.skill_id.c_str());


    if(skill != currSkill)
    {
        res.request_result_id = Tum::Common::REJECTED_SKILLR;
        res.request_result_label = QString("REJECTED_SKILLR: skill "+ skill + "not running.").toStdString();
        return true;
    }

    if(!m_skillManager->requestSkillTransition())
    {
        res.request_result_id = Tum::Common::REJECTED_SKILLR;
        res.request_result_label = QString("REJECTED_SKILLR: pending skill transition").toStdString();
        return true;
    }

    res.request_result_id = Tum::Common::ACCEPTED_SKILLR;
    res.request_result_label = QString("ACCEPTED_SKILL: Stopping skill " + skill).toStdString();
    return true;
}



bool SkillServer::skillStateCallBack(tum_ics_msgs::GetSkillState::Request &req,
                                     tum_ics_msgs::GetSkillState::Response &res)
{


    QString skill = QString(req.skill_id.c_str());



    if(m_gripperParamAvailable)
    {
        tum_ics_lacquey_gripper_msgs::getGripperState gripperSrv;

        if(!m_getGripperState.call(gripperSrv))
        {
            res.request_result_id = Tum::Common::UNKNOWN_STATE;
            res.request_result_label = QString("ERROR: Calling getGripperState failed.").toStdString();
            //ROS_ERROR_STREAM("SkillReq: "<<req.skill_id<<" Reply: "<<res.request_result_label<<"("<<res.request_result_id<<")");
            return true;
        }

        QString state = QString(gripperSrv.response.currentState.c_str());
//        ROS_INFO_STREAM("gripper state: " << state.toStdString());

        if(skill == "OPEN_GRIPPER")
        {
            if(state == "open")
            {
                res.request_result_id = Tum::Common::FINISHED_STATE;
                res.request_result_label = QString("FINISHED_STATE").toStdString();
                //ROS_ERROR_STREAM("SkillReq: "<<req.skill_id<<" Reply: "<<res.request_result_label<<"("<<res.request_result_id<<")");
                return true;
            }
            else
            {
                res.request_result_id = Tum::Common::RUNNING_STATE;
                res.request_result_label = QString("RUNNING_STATE").toStdString();
                //ROS_ERROR_STREAM("SkillReq: "<<req.skill_id<<" Reply: "<<res.request_result_label<<"("<<res.request_result_id<<")");
                return true;
            }
        }

        if(skill == "CLOSE_GRIPPER")
        {
            if(state == "close")
            {
                res.request_result_id = Tum::Common::FINISHED_STATE;
                res.request_result_label = QString("FINISHED_STATE").toStdString();
                //ROS_ERROR_STREAM("SkillReq: "<<req.skill_id<<" Reply: "<<res.request_result_label<<"("<<res.request_result_id<<")");
                return true;
            }
            else
            {
                res.request_result_id = Tum::Common::RUNNING_STATE;
                res.request_result_label = QString("RUNNING_STATE").toStdString();
                //ROS_ERROR_STREAM("SkillReq: "<<req.skill_id<<" Reply: "<<res.request_result_label<<"("<<res.request_result_id<<")");
                return true;
            }
        }
    }

    if(!m_skillManager->skillList().hasSkill(skill))
    {
        res.request_result_id = Tum::Common::UNKNOWN_STATE;
        res.request_result_label = QString("ERROR: Invalid skill").toStdString();
        //ROS_ERROR_STREAM("SkillReq: "<<req.skill_id<<" Reply: "<<res.request_result_label<<"("<<res.request_result_id<<")");
        return true;
    }

    if(m_skillManager->skillFailed(skill))
    {
        res.request_result_id = Tum::Common::FAILED_STATE;
        res.request_result_label = QString("FAILED_STATE").toStdString();
        //ROS_ERROR_STREAM("SkillReq: "<<req.skill_id<<" Reply: "<<res.request_result_label<<"("<<res.request_result_id<<")");
        return true;
    }

    if(m_skillManager->skillStarted(skill))
    {
        res.request_result_id = Tum::Common::STARTED_STATE;
        res.request_result_label = QString("STARTED_STATE").toStdString();
        //ROS_ERROR_STREAM("SkillReq: "<<req.skill_id<<" Reply: "<<res.request_result_label<<"("<<res.request_result_id<<")");
        return true;
    }

    if(m_skillManager->skillRunning(skill))
    {
        res.request_result_id = Tum::Common::RUNNING_STATE;
        res.request_result_label = QString("RUNNING_STATE").toStdString();
        //ROS_ERROR_STREAM("SkillReq: "<<req.skill_id<<" Reply: "<<res.request_result_label<<"("<<res.request_result_id<<")");
        return true;
    }

    if(m_skillManager->skillFinished(skill))
    {
        res.request_result_id = Tum::Common::FINISHED_STATE;
        res.request_result_label = QString("FINISHED_STATE").toStdString();
        //ROS_ERROR_STREAM("SkillReq: "<<req.skill_id<<" Reply: "<<res.request_result_label<<"("<<res.request_result_id<<")");
        return true;
    }

    res.request_result_id = Tum::Common::INITIALIZED_STATE;
    res.request_result_label = QString("INITIALIZED_STATE").toStdString();
    //ROS_ERROR_STREAM("SkillReq: "<<req.skill_id<<" Reply: "<<res.request_result_label<<"("<<res.request_result_id<<")");
    return true;
}

bool SkillServer::getActiveSkillCallBack(tum_ics_msgs::GetActiveSkill::Request &req,
                       tum_ics_msgs::GetActiveSkill::Response &res)
{
    QString currSkill = m_skillManager->currentSkill().name();
    res.skill_name=currSkill.toStdString();
    return true;
}

bool SkillServer::getParamString(QString& str, const QString& param)
{
    str.clear();

    std::string p = param.toStdString();
    std::string stdStr;

    if(!ros::param::has(p))
    {
        ROS_ERROR("Parameter '%s' is missing.", p.c_str());
        return false;
    }

    if(!ros::param::get(p,stdStr))
    {
        ROS_ERROR("Couldn't get parameter '%s'.", p.c_str());
        return false;
    }

    str = QString(stdStr.c_str());
    return true;
}

bool SkillServer::hasParam(const QString& param)
{
    std::string p = param.toStdString();
    return ros::param::has(p);
}

}


