#include <tum_ics_ur_robot_manager/SkillList.h>

#include <ros/ros.h>

namespace tum_ics_ur_robot_manager{


const Skill SkillList::UndefinedSkill = Skill::Undefined();

SkillList::SkillList() :
    m_defined(false)
{

}

SkillList::SkillList(const QString& paramRoot) :
    m_defined(false)
{
    load(paramRoot);
}


SkillList::SkillList(const QVector<Skill>& skills,
                     const QVector<QString>& ctrls) :
    m_defined(false),
    m_skills(skills)
{
    if(!m_skills.isEmpty())
    {
        m_defined = true;
    }

    update(ctrls);
}

SkillList::SkillList(const SkillList& sl) :
    m_defined(sl.m_defined),
    m_skills(sl.m_skills),
    m_skillMap(sl.m_skillMap)
{

}

SkillList::~SkillList()
{

}

bool SkillList::load(const QVector<QString>& ctrls,
                     const QString& paramRoot)
{
    m_skills.clear();

    std::stringstream ss;
    std::string paramSkill;
    std::string param;

    if(!ros::param::has(paramRoot.toStdString()))
    {
        ROS_ERROR("SkillList::load(): parameter '%s' is missing.",
                  paramRoot.toStdString().c_str());

        return false;
    }

    int cnt=1;
    while(ros::ok())
    {
        std::string name;
        std::vector<std::string> load;
        std::vector<std::string> unload;
        Skill skill;

        ss.str("");
        ss << paramRoot.toStdString() << "/skill_" << cnt;
        paramSkill = ss.str();

        if(!ros::param::has(paramSkill))
        {
            break;
        }

//        qDebug("SkillList::load(): Get %s ...",paramSkill.c_str());

        ss.str("");
        ss << paramSkill << "/name";
        param = ss.str();

        if(!ros::param::has(param))
        {
            ROS_ERROR("SkillList::load(): skill_%d has no 'name' param. Skip skill",cnt);
            cnt++;
            continue;
        }

        if(!ros::param::get(param,name))
        {
            ROS_ERROR("SkillList::load(): skill_%d: Couldn't get 'name' param. Skip skill",cnt);
            cnt++;
            continue;
        }

        ss.str("");
        ss << paramSkill << "/load";
        param = ss.str();

        if(!ros::param::has(param))
        {
            ROS_ERROR("SkillList::load(): skill_%d has no 'load' param. Skip skill",cnt);
            cnt++;
            continue;
        }

        if(!ros::param::get(param,load))
        {
            ROS_ERROR("SkillList::load(): skill_%d: Couldn't get 'load' param. Skip skill",cnt);
            cnt++;
            continue;
        }

        if(load.size() == 0)
        {
            ROS_ERROR("SkillList::load(): skill_%d: load' param is empty. Skip skill",cnt);
            cnt++;
            continue;
        }

        ss.str("");
        ss << paramSkill << "/unload";
        param = ss.str();

        if(ros::param::has(param))
        {
            if(!ros::param::get(param,unload))
            {
                ROS_ERROR("SkillList::load(): skill_%d: Couldn't get 'unload' param. Skip skill",cnt);
                cnt++;
                continue;
            }
        }

        QString qname = QString(name.c_str());
        QVector<QString> qload;
        QVector<QString> qunload;

        for(int i=0; i<load.size(); i++)
        {
            qload.append(QString(load.at(i).c_str()));
        }

        for(int i=0; i<unload.size(); i++)
        {
            qunload.append(QString(unload.at(i).c_str()));
        }

        skill = Skill(qname,qload,qunload);
        if(skill.isUndefined())
        {
            ROS_WARN("SkillList::load(): Skipped invalid skill '%s'.",
                     qname.toLatin1().data());
            cnt++;
            continue;
        }

//        qDebug("Loaded %s",skill.toString().toLatin1().data());
        m_skills.append(skill);

        cnt++;
    }

    qDebug("Loading skills from parameter server finished.");

    if(m_skills.isEmpty())
    {
        m_defined = false;
        return false;
    }

    update(ctrls);

    m_defined = true;
    return true;
}

bool SkillList::load(const QString& paramRoot)
{
    return load(QVector<QString>(),paramRoot);
}

void SkillList::update(const QVector<QString>& ctrls)
{
    if(ctrls.isEmpty())
    {
        updateSkillMap();
        return;
    }

    if(isUndefined())
    {
        return;
    }

    QVector<Skill> skills;

    // go through skills
    for(int i=0; i<m_skills.size(); i++)
    {
        const Skill& s = m_skills.at(i);
        const QVector<QString> sCtrls = s.loadCtrls();

        bool flag = true;

        // go through ctrls of skills
        for(int j=0; j<sCtrls.size();j++)
        {
            if(!ctrls.contains(sCtrls.at(j)))
            {
//                qDebug("Removed skill '%s'.",s.name().toLatin1().data());
                flag = false;
                break;
            }
        }
        if(flag)
        {
            skills.append(s);
        }
    }
    m_skills = skills;
    updateSkillMap();
}

void SkillList::clear()
{
    m_defined = false;
    m_skills.clear();
    m_skillMap.clear();
}

const Skill& SkillList::operator[] (const QString& name) const
{
    int ind = m_skillMap.value(name,-1);
    if(ind == -1)
    {
        qCritical("SkillList::operator[]: Skill '%s' not in list",
                  name.toLatin1().data());

        return UndefinedSkill;
    }
    return m_skills.at(ind);
}

const Skill& SkillList::at(const QString& name) const
{
    int ind = m_skillMap.value(name,-1);
    if(ind == -1)
    {
        qCritical("ControllerList::at(): Controller '%s' not in list",
                  name.toLatin1().data());

        return UndefinedSkill;
    }
    return m_skills.at(ind);
}

bool SkillList::hasSkill(const QString& name) const
{
    int ind = m_skillMap.value(name,-1);
    if(ind == -1)
    {
        return false;
    }

    return true;
}

bool SkillList::hasSkill(const Skill& skill) const
{
    return hasSkill(skill.name());
}

bool SkillList::isDefined() const
{
    return m_defined;
}

bool SkillList::isUndefined() const
{
    return !m_defined;
}

int SkillList::size() const
{
    return m_skills.size();
}

const QVector<Skill>& SkillList::skills() const
{
    return m_skills;
}

QString SkillList::toString() const
{
    QString s;
    QString temp;

    if(m_skills.isEmpty())
    {
        s.append(temp.sprintf("SkillList: empty"));
        return s;
    }

    s.append(temp.sprintf("SkillList:\n"));

    for(int i=0; i<m_skills.size();i++)
    {
        const QString& name = m_skills.at(i).name();
        s.append(temp.sprintf("  %s\n",name.toLatin1().data()));
    }

    return s;
}

void SkillList::updateSkillMap()
{
    m_skillMap.clear();
    for(int i=0;i<m_skills.size();i++)
    {
        const Skill& s = m_skills.at(i);
        m_skillMap.insert(s.name(),i);
    }
}

}
