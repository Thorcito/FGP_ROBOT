#include <tum_ics_ur_robot_manager/SkillTransition.h>

namespace tum_ics_ur_robot_manager{

SkillTransition::SkillTransition(const Skill& current, const Skill& next) :
    m_name(current.name()+"->"+next.name()),
    m_curr(current),
    m_next(next),
    m_defined(false)
{
    if(current.isUndefined() || next.isUndefined())
    {
        return;
    }

    const QVector<QString>& lc = current.loadCtrls();
    const QVector<QString>& uc = current.unloadCtrls();
    const QVector<QString>& ln = next.loadCtrls();


    // find common controllers (unordered)
    QVector<QString> common;

    for(int i=0; i<lc.size(); i++)
    {
        const QString& c = lc.at(i);
        if(ln.contains(c))
        {
            common.append(c);
        }
    }

    // TODO: for now we keep all common controllers loaded
    m_keepCtrls = common;
//    m_keepCtrls = keepControllers(common);

    // TODO: for now we start and stop all controllers
    m_startCtrls = ln;
    m_stopCtrls = uc;

//    // if we cannot keep the common controllers, then we start and stop everything
//    if(m_keepCtrls.isEmpty())
//    {
//        m_startCtrls = ln;
//        m_stopCtrls = uc;
//    }
//    else
//    {
//        // find controllers to start (ordered)
//        for(int i=0; i<ln.size(); i++)
//        {
//            const QString& c = ln.at(i);
//            if(!m_keepCtrls.contains(c))
//            {
//                m_startCtrls.append(c);
//            }
//        }

//        // find controllers to stop (ordered)
//        for(int i=0; i<uc.size(); i++)
//        {
//            const QString& c = uc.at(i);
//            if(!m_keepCtrls.contains(c))
//            {
//                m_stopCtrls.append(c);
//            }
//        }
//    }

    m_defined = true;
}

SkillTransition::SkillTransition(const SkillTransition& st) :
    m_name(st.m_name),
    m_curr(st.m_curr),
    m_next(st.m_next),
    m_startCtrls(st.m_startCtrls),
    m_stopCtrls(st.m_stopCtrls),
    m_keepCtrls(st.m_keepCtrls),
    m_defined(st.m_defined)
{

}

SkillTransition::~SkillTransition()
{

}

const QString& SkillTransition::name() const
{
    return m_name;
}

bool SkillTransition::isDefined() const
{
    return m_defined;
}

bool SkillTransition::isUndefined() const
{
    return !m_defined;
}

const Skill& SkillTransition::current() const
{
    return m_curr;
}

const Skill& SkillTransition::next() const
{
    return m_next;
}

bool SkillTransition::isValid(const SkillList& sl) const
{
    if(sl.isUndefined())
    {
        return false;
    }

    if(isUndefined())
    {
        return false;
    }

    if(!sl.hasSkill(m_curr) || !sl.hasSkill(m_next))
    {
        return false;
    }
    return true;
}

bool SkillTransition::isIdentityTansition() const
{
    return (m_curr == m_next);
}

const QVector<QString>& SkillTransition::startCtrls() const
{
    return m_startCtrls;
}

const QVector<QString>& SkillTransition::stopCtrls() const
{
    return m_stopCtrls;
}

const QVector<QString>& SkillTransition::keepCtrls() const
{
    return m_keepCtrls;
}

const QString SkillTransition::toString() const
{
    QString s;
    QString temp;

    s.append(temp.sprintf("SkillTransition '%s'\n",m_name.toLatin1().data()));

    s.append(temp.sprintf("  stop ctrls\n"));

    for(int i=0; i<m_stopCtrls.size();i++)
    {
        const QString& c = m_stopCtrls.at(i);
        s.append(temp.sprintf("    %s\n",c.toLatin1().data()));
    }

    s.append(temp.sprintf("\n  keep ctrls\n"));

    for(int i=0; i<m_keepCtrls.size();i++)
    {
        const QString& c = m_keepCtrls.at(i);
        s.append(temp.sprintf("    %s\n",c.toLatin1().data()));
    }

    s.append(temp.sprintf("\n  start ctrls\n"));

    for(int i=0; i<m_startCtrls.size();i++)
    {
        const QString& c = m_startCtrls.at(i);
        s.append(temp.sprintf("    %s\n",c.toLatin1().data()));
    }

    return s;
}

QVector<QString> SkillTransition::keepControllers(const QVector<QString>& common)
{
    QVector<QString> keep;

    if(common.size() != 0)
    {
        return keep;
    }

    const QVector<QString>& lc = m_curr.loadCtrls();
    const QVector<QString>& ln = m_next.loadCtrls();

    QVector<QString> commonOrdered;

    // order common controllers to order of currently started controllers
    for(int i=0; i<lc.size(); i++)
    {
        const QString& c = lc.at(i);
        if(common.contains(c))
        {
            commonOrdered.append(c);
        }
    }

    // find indices of controllers to keep in next load
    QVector<int> nInd; // next indices ordered in curr

    for(int i=0; i<commonOrdered.size(); i++)
    {
        const QString& c = commonOrdered.at(i);
        nInd.append(ln.indexOf(c));
    }

    // next doesn't start with a controller from current
    if(!nInd.contains(0))
    {
        return QVector<QString>();
    }

    // controllers in curr and next load must be consecutive and in same order

    // keep controller, only one common controller, next starts with this controller
    if(common.size() == 1)
    {
        return common;
    }

    bool foundFirstElement = false;

    // find controllers to keep (find first, add everything in consecutive order)
    for(int i=0; i<nInd.size(); i++)
    {
        int curr = nInd.at(i);
        const QString& c = ln.at(curr);

        // find index 0, ignore everthing before
        // next has to start with a controller from current
        if(!foundFirstElement && curr != 0)
        {
            continue;
        }

        // first element
        if(curr == 0)
        {
            // add the first element
            keep.append(c);
            foundFirstElement = true;
        }

        // check next element, last element doesn't have next element
        if(i < nInd.size()-1)
        {
            int next = nInd.at(i+1);
            int diff = next - curr;
            const QString& n = ln.at(next);

            // not in order, keep what we have already collected
            // next element doesn't fit
            if(diff != 1)
            {
                return keep;
            }

            // add the next element
            keep.append(n);
        }
    }
    return keep;
}

}
