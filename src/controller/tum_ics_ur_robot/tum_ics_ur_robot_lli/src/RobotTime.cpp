#include<tum_ics_ur_robot_lli/RobotTime.h>


namespace tum_ics_ur_robot_lli{

RobotTime::RobotTime()
{
    reset();
}

RobotTime::RobotTime(const RobotTime &time):
    m_tRi(time.tRi()),
    m_tRc(time.tRc()),
    m_tD(time.tD())
{

}

RobotTime::RobotTime(const ros::Time &time):
    m_tRi(time),
    m_tRc(time),
    m_tD(0.0)
{

}

RobotTime::~RobotTime()
{

}

const ros::Time& RobotTime::tRi() const
{
    return m_tRi;
}
const ros::Time& RobotTime::tRc() const
{
    return m_tRc;
}
const double RobotTime::tD() const
{
    return m_tD;
}

void RobotTime::update(const ros::Time &timeC)
{
    m_tRc=timeC;
    m_tD=m_tRc.toSec()-m_tRi.toSec();
}

void RobotTime::reset()
{
    m_tRc=ros::Time::now();
    m_tRi=m_tRc;
    m_tD=0.0;
}

void RobotTime::reset(const ros::Time &time)
{
    m_tRc=time;
    m_tRi=m_tRc;
    m_tD=0.0;
}


}
