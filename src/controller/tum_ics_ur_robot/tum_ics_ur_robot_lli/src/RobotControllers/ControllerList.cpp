#include<tum_ics_ur_robot_lli/RobotControllers/ControllerList.h>

namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

QVector<QString> ControllerList::names(const QVector<Controller*>& ctrls)
{
    QVector<QString> names;

    for(int i=0; i< ctrls.size(); i++)
    {
        names.append(ctrls.at(i)->name());
    }

    return names;
}

ControllerList::ControllerList()
{

}

ControllerList::ControllerList(const QVector<Controller*>& ctrls)
{
    for(int i=0; i< ctrls.size(); i++)
    {
        add(ctrls.at(i));
    }
}

ControllerList::ControllerList(const QList<Controller*>& ctrls)
{
    for(int i=0; i< ctrls.size(); i++)
    {
        add(ctrls.at(i));
    }
}

ControllerList::ControllerList(const ControllerList& cl) :
    m_ctrlsMap(cl.m_ctrlsMap)
{

}

ControllerList::~ControllerList()
{

}

bool ControllerList::add(Controller* ctrl)
{
    Controller* c = m_ctrlsMap.value(ctrl->name(),0);
    if(c != 0)
    {
        qWarning("ControllerList::add(): Couldn't add controller '%s'.\n Controller already in list." ,
                 ctrl->name().toLatin1().data());
        return false;
    }
    m_ctrlsMap.insert(ctrl->name(),ctrl);
    return true;
}

bool ControllerList::remove(Controller* ctrl)
{
    Controller* c = m_ctrlsMap.value(ctrl->name(),0);
    if(c != ctrl)
    {
        qWarning("ControllerList::remove(): Couldn't remove controller '%s'.\n Controller not in list." ,
                 ctrl->name().toLatin1().data());
        return false;
    }
    m_ctrlsMap.remove(ctrl->name());
    return true;
}

bool ControllerList::remove(const QString& name)
{
    Controller* c = m_ctrlsMap.value(name,0);
    if(c == 0)
    {
        qWarning("ControllerList::remove(): Couldn't remove controller '%s'.\n Controller not in list." ,
                 name.toLatin1().data());
        return false;
    }
    m_ctrlsMap.remove(name);
    return true;
}

bool ControllerList::isEmpty() const
{
    return m_ctrlsMap.isEmpty();
}

QVector<Controller*> ControllerList::controllers() const
{
    return m_ctrlsMap.values().toVector();
}

QVector<QString> ControllerList::names() const
{
    return names(controllers());
}

Controller* ControllerList::operator[] (const QString& name) const
{
    Controller* c = m_ctrlsMap.value(name,0);
    if(c == 0)
    {
        qCritical("ControllerList::operator[]: Controller '%s' not in list",
                  name.toLatin1().data());
    }
    return c;
}

Controller* ControllerList::at(const QString& name) const
{
    Controller* c = m_ctrlsMap.value(name,0);
    if(c == 0)
    {
        qCritical("ControllerList::at(): Controller '%s' not in list",
                  name.toLatin1().data());
    }
    return c;
}

int ControllerList::size() const
{
    return m_ctrlsMap.size();
}

}
}
