#include<tum_ics_ur_robot_lli/RobotControllers/Controller.h>

namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

Controller::Controller(const QString& name, ControlInterface intf, ControlType type, ControlSpace space, double weight, ControlSampleMode sampleMode) :
    m_failed(false),
    m_initialized(false),
    m_started(false),
    m_running(false),
    m_finished(false),
    m_error(false),
    m_name(name),
    m_ctrlIntf(intf),
    m_ctrlType(type),
    m_ctrlSpace(space),
    m_weight(weight),
    m_loaded(false),
    m_qGoal(VectorDOFd::Zero()),
    m_xGoal(Vector3d::Zero()),
    m_RGoal(Matrix3d::Identity()),
    m_XGoal(Vector6d::Zero()),
    m_runState(STATE_INIT),
    m_tauExtFlag(false),
    m_ctrlSampleMode(sampleMode)
{
}

void Controller::setWeight(double weight)
{
    m_weight = weight;
}

void Controller::setGoalTime(double gTime)
{
    m_timeGoal = gTime;
}

void Controller::setGoalJoints(const VectorDOFd& qGoal)
{
    m_qGoal = qGoal;
}

void Controller::setGoalPosition(const Vector3d& xGoal)
{
    m_xGoal = xGoal;
}

void Controller::setGoalOrientation(const Matrix3d& RGoal)
{
    m_RGoal = RGoal;
}

void Controller::setGoalPose(const Vector6d& XGoal)
{
    m_XGoal = XGoal;
    m_xGoal = m_XGoal.block(0,0,3,1);

    //TODO: add the orientation
//    m_RGoal=Tum::Tools::MathTools::Rz(m_XGoal(3))*Ry(m_XGoal(4))*Rx(m_XGoal(5));
}

void Controller::setGoalPose(const Vector3d& xGoal, const Matrix3d& RGoal)
{
    m_xGoal = xGoal;
    m_RGoal = RGoal;
}

void Controller::enableTauExtFlag()
{
    m_tauExtFlag=true;
}
void Controller::disableTauExtFlag()
{
    m_tauExtFlag=false;
}

const QString& Controller::name() const
{
    return m_name;
}

bool Controller::isInitialized() const
{
    return m_initialized;
}

bool Controller::isStarted() const
{
    return m_started;
}

bool Controller::isRunning() const
{
    return m_running;
}

bool Controller::isFinished() const
{
    return m_finished;
}

bool Controller::hasFailed() const
{
    return m_failed;
}

bool Controller::isLoaded() const
{
    return m_loaded;
}

ControlInterface Controller::getCtrlIntf() const
{
    return m_ctrlIntf;
}

ControlType Controller::getCtrlType() const
{
    return m_ctrlType;
}

ControlSpace Controller::getCtrlSpace() const
{
    return m_ctrlSpace;
}

ControlSampleMode Controller::getCtrlSampleMode() const
{
    return m_ctrlSampleMode;
}

bool Controller::error() const
{
    return m_error;
}

const QString& Controller::errorString() const
{
    return m_errorString;
}


void Controller::satTau(VectorDOFd &tauIn, const VectorDOFd &tauMax)
{
    for(int i=0;i<STD_DOF;i++)
    {
        if(tauIn(i)>tauMax(i))
        {
            tauIn(i)=tauMax(i);
        }
        else if(tauIn(i)<-tauMax(i))
        {
            tauIn(i)=-tauMax(i);
        }
    }
}

void Controller::setLoaded(bool loaded)
{
    m_loaded = loaded;
}

///*inline */Controller::~Controller(){}

//double Controller::weight() const
//{
//    return m_weight;
//}

//const VectorDOFd& Controller::goalJoints() const
//{
//    return m_qGoal;
//}

//const Vector3d& Controller::goalPosition() const
//{
//    return m_xGoal;
//}

//const Vector6d& Controller::goalPose() const
//{
//    return m_XGoal;
//}

//const QString Controller::getName() const
//{
//    return m_name;
//}

//const double Controller::timeGoal() const
//{
//    return m_timeGoal;
//}

//const VectorDOFd& Controller::qGoal() const
//{
//    return m_qGoal;
//}

//const Vector3d& Controller::xGoal() const
//{
//    return m_xGoal;
//}

//const Vector6d& Controller::XGoal() const
//{
//    return m_XGoal;
//}


}
}
