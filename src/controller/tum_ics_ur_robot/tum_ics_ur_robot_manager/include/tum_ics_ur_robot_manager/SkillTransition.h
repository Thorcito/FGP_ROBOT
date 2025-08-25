#ifndef UR_ROBOT_MANAGER_SKILLTRANSITION_H
#define UR_ROBOT_MANAGER_SKILLTRANSITION_H

#include <QVector>
#include <QString>

#include <tum_ics_ur_robot_manager/Skill.h>
#include <tum_ics_ur_robot_manager/SkillList.h>

namespace tum_ics_ur_robot_manager{

class SkillTransition
{
public:

private:
    QString m_name;

    Skill m_curr;
    Skill m_next;

    QVector<QString> m_startCtrls;
    QVector<QString> m_stopCtrls;
    QVector<QString> m_keepCtrls;

    bool m_defined;

public:
    SkillTransition(const Skill& current=Skill::Undefined(),
                    const Skill& next=Skill::Undefined());

    SkillTransition(const SkillTransition& st);
    ~SkillTransition();

    const QString& name() const;

    bool isDefined() const;
    bool isUndefined() const;

    const Skill& current() const;
    const Skill& next() const;

    bool isValid(const SkillList& sl) const;
    bool isIdentityTansition() const;

    // controllers to start
    const QVector<QString>& startCtrls() const;

    // controllers to stop
    const QVector<QString>& stopCtrls() const;

    // controllers to keep loaded
    const QVector<QString>& keepCtrls() const;

    const QString toString() const;

private:
    // gets controllers to keep over transition
    QVector<QString> keepControllers(const QVector<QString>& common);
};

}


#endif // UR_ROBOT_MANAGER_SKILLTRANSITION_H
