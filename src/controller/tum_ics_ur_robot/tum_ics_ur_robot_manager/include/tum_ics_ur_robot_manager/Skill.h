#ifndef UR_ROBOT_MANAGER_SKILL_H
#define UR_ROBOT_MANAGER_SKILL_H

#include <QVector>
#include <QString>

namespace tum_ics_ur_robot_manager{

class Skill
{
public:
    static Skill Undefined(); // uses empty constructor

    // does NOT check if controllers are of same type or available
    // this is done in skill list
    static bool check(const QVector<QString>& loadCtrls,
                      const QVector<QString>& unloadCtrls);

    static QVector<QString> reverse(const QVector<QString>& ctrls);

private:
    QString m_name;

    QVector<QString> m_loadCtrls;       // the load order
    QVector<QString> m_unloadCtrls;     // the unload order

public:
    // if unload ctrls are empty then use inverse order of load controllers
    Skill(const QString& name = "UNDEFINED",
          const QVector<QString>& loadCtrls = QVector<QString>(),
          const QVector<QString>& unloadCtrls = QVector<QString>());

    Skill(const Skill& s);
    ~Skill();

    bool operator == (const Skill& other) const;
    bool operator != (const Skill& other) const;

    const QString& name() const;

    bool isDefined() const;
    bool isUndefined() const;

    const QVector<QString>& loadCtrls() const;
    const QVector<QString>& unloadCtrls() const;

    QString toString() const;
};

}


#endif // UR_ROBOT_MANAGER_SKILL_H
