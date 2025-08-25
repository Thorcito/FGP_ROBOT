#ifndef UR_ROBOT_MANAGER_SKILLLIST_H
#define UR_ROBOT_MANAGER_SKILLLIST_H

#include <tum_ics_ur_robot_manager/Skill.h>

#include <QVector>
#include <QString>
#include <QMap>

namespace tum_ics_ur_robot_manager{

class SkillList
{

private:
    static const Skill UndefinedSkill;

private:
    bool m_defined;

    QVector<Skill> m_skills;
    QMap<QString,int> m_skillMap; // map: name -> index

public:
    SkillList();

    // loads list directly from parameter server
    SkillList(const QString& paramRoot);

    // empty ctrls means: load all skills, controllers not specified
    SkillList(const QVector<Skill>& skills,
              const QVector<QString>& ctrls=QVector<QString>());

    SkillList(const SkillList& sl);

    ~SkillList();

    // load from parameter server
    // empty ctrls means: load all skills, controllers not specified
    bool load(const QVector<QString>& ctrls,
              const QString& paramRoot="~skill_list");

    bool load(const QString& paramRoot="~skill_list");

    // removes all skills from list which have unsupported controllers
    void update(const QVector<QString>& ctrls);

    void clear();

    const Skill& operator[] (const QString& name) const;
    const Skill& at(const QString& name) const;

    bool hasSkill(const QString& name) const;
    bool hasSkill(const Skill& skill) const;

    bool isDefined() const;
    bool isUndefined() const;

    int size() const;

    const QVector<Skill>& skills() const;

    QString toString() const;

private:
    void updateSkillMap();
};

}


#endif // UR_ROBOT_MANAGER_SKILLLIST_H
