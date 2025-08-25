#include <tum_ics_ur_robot_manager/Skill.h>

namespace tum_ics_ur_robot_manager{

Skill Skill::Undefined()
{
    return Skill();
}


bool Skill::check(const QVector<QString>& loadCtrls,
                      const QVector<QString>& unloadCtrls)
{
    if(loadCtrls.size() != unloadCtrls.size())
    {
        return false;
    }

    for(int i=0; i<loadCtrls.size(); i++)
    {
        if(!unloadCtrls.contains(loadCtrls.at(i)))
        {
            return false;
        }
    }

    return true;
}

QVector<QString> Skill::reverse(const QVector<QString>& ctrls)
{
    if(ctrls.size() < 2)
    {
        return ctrls;
    }

    QVector<QString> out;
    for(int i=ctrls.size()-1; i>=0; i--)
    {
        out.append(ctrls.at(i));
    }

    return out;
}


Skill::Skill(const QString& name,
             const QVector<QString>& loadCtrls,
             const QVector<QString>& unloadCtrls) :
    m_name(name),
    m_loadCtrls(loadCtrls),
    m_unloadCtrls(unloadCtrls)
{
    if(m_name.isEmpty())
    {
        *this = Undefined();
    }

    if(m_loadCtrls.isEmpty())
    {
        return;
    }

    if(m_unloadCtrls.isEmpty())
    {
        m_unloadCtrls = reverse(m_loadCtrls);
    }

    if(!check(m_loadCtrls,m_unloadCtrls))
    {
        *this = Undefined();
    }
}

Skill::Skill(const Skill& s) :
    m_name(s.m_name),
    m_loadCtrls(s.m_loadCtrls),
    m_unloadCtrls(s.m_unloadCtrls)
{

}

Skill::~Skill()
{

}

bool Skill::operator == (const Skill& other) const
{
    if(m_name != other.m_name)
    {
        return false;
    }

    return true;
}

bool Skill::operator != (const Skill& other) const
{
    return !(*this == other);
}

const QString& Skill::name() const
{
    return m_name;
}

bool Skill::isDefined() const
{
    return (*this != Undefined());
}

bool Skill::isUndefined() const
{
    return (*this == Undefined());
}

const QVector<QString>& Skill::loadCtrls() const
{
    return m_loadCtrls;
}

const QVector<QString>& Skill::unloadCtrls() const
{
    return m_unloadCtrls;
}

QString Skill::toString() const
{
    QString s;
    QString temp;

    s.append(temp.sprintf("Skill '%s'\n",m_name.toLatin1().data()));
    s.append(temp.sprintf("  load ctrls\n"));

    for(int i=0; i<m_loadCtrls.size();i++)
    {
        const QString& c = m_loadCtrls.at(i);
        s.append(temp.sprintf("    %s\n",c.toLatin1().data()));
    }

    s.append(temp.sprintf("  unload ctrls\n"));

    for(int i=0; i<m_unloadCtrls.size();i++)
    {
        const QString& c = m_unloadCtrls.at(i);
        s.append(temp.sprintf("    %s\n",c.toLatin1().data()));
    }

    return s;
}

}
