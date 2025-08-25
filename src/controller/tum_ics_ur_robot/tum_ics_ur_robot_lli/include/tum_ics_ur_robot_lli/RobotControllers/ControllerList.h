#ifndef UR_ROBOT_LLI_CONTROLLERLIST_H
#define UR_ROBOT_LLI_CONTROLLERLIST_H

#include <tum_ics_ur_robot_lli/RobotControllers/Controller.h>
#include <QVector>
#include <QMap>

namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

class Controller;

class ControllerList
{
public:
    static QVector<QString> names(const QVector<Controller*>& ctrls);

private:
    QMap<QString,Controller*> m_ctrlsMap;  // map: name -> addr

public:
    ControllerList();
    ControllerList(const QVector<Controller*>& ctrls);
    ControllerList(const QList<Controller*>& ctrls);
    ControllerList(const ControllerList& cl);

    ~ControllerList();

    // uses controller name as identifier
    bool add(Controller* ctrl);
    bool remove(Controller* ctrl);
    bool remove(const QString& name);

    bool isEmpty() const;

    QVector<Controller*> controllers() const;
    QVector<QString> names() const;

    Controller* operator[] (const QString& name) const;
    Controller* at(const QString& name) const;

    int size() const;
};

}}

#endif // UR_ROBOT_LLI_CONTROLLERLIST_H
