#include<tum_ics_ur_robot_controllers/SkillControls/DControl.h>

#include<tum_ics_ur_robot_msgs/ControlData.h>

namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

DControl::DControl(double weight,const QString& name):
    ControlEffort(name,STANDARD_TYPE,JOINT_SPACE,weight)
{
    m_pubCtrlData=m_n.advertise<tum_ics_ur_robot_msgs::ControlData>("DCtrlData",100);

}

DControl::~DControl()
{

}

void DControl::setQInit(const JointState& qinit)
{
    m_qInit=qinit;

}
void DControl::setQHome(const JointState& qhome)
{
    m_qHome=qhome;

}
void DControl::setQPark(const JointState& qpark)
{
    m_qPark=qpark;
}


bool DControl::init()
{

    std::string ns="~d_ctrl";
    std::stringstream s;

    if (!ros::param::has(ns))
    {
        s<<getName().toStdString()<<" init(): Control gains not defined --"<<ns<<"--, did you load them in the rosparam server??";
        m_error=true;
        m_errorString=s.str().c_str();
        ROS_ERROR_STREAM(s.str());
        return false;
    }

    VDouble p;

    /////D GAINS

    s<<ns<<"/gains_d";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<getName().toStdString()<<" init(): Wrong number of d_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_Kd(i,i)=p[i];
    }

    ROS_WARN_STREAM("Kd: \n"<<m_Kd);

    m_runState=STATE_INIT;

    return true;
}
bool DControl::start()
{
    //For now the start, update and stop functions are states in update()
    m_runState=STATE_START;
    ROS_WARN_STREAM(getName().toStdString()<<" Start() Running state changed to: "<<m_runState);
    return true;
}
VectorDOFd DControl::update(const RobotTime& time, const JointState &current)
{
    VectorDOFd tau;

    tau.setZero();

    tum_ics_ur_robot_msgs::ControlData msg;


    if(m_runState==STATE_INIT)
    {

        //reset the initialization flag (used in STATE_UPDATE to compute Bias)
        //m_startFlag=false;


        //VectorDOFd G=m_dynamicModel->G();

        m_failed=false;
        m_initialized=true;
        m_started=false;
        m_running=false;
        m_finished=false;



        //we are waiting for the start function, then we just keep the robot stable
        return tau;
    }
    if(m_runState==STATE_START)
    {
        //reset the initialization flag (used in STATE_UPDATE to compute Bias)
        //        m_startFlag=false;

        m_failed=false;
        m_initialized=false;
        m_started=true;
        m_running=false;
        m_finished=false;

        //For now nothing to do here, just change the state to UPDATE
        m_runState=STATE_UPDATE;
        ROS_WARN_STREAM(getName().toStdString()<<" STATE_START Running state changed to: "<<m_runState);

        //Since we don't do anything we just go to the next state
        //if we have an alternative control then we compute here
        // tau and enable the following line
        //return tau;

    }
    if(m_runState==STATE_UPDATE)
    {

        m_failed=false;
        m_initialized=false;
        m_started=false;
        m_running=true;
        m_finished=false;

        tau=-m_Kd*current.qp;

        for(int i=0;i<STD_DOF;i++)
        {
            if((isnan(tau(i)))||(isinf(tau(i))))
            {
                std::stringstream s;
                s<<getName().toStdString()<<" update(): The control output is nan/inf: "<<tau.transpose();
                m_error=true;
                m_errorString=s.str().c_str();
                ROS_ERROR_STREAM(m_errorString.toStdString());

                m_failed=true;
                m_initialized=false;
                m_started=false;
                m_running=true;
                m_finished=false;

                tau.setZero();
                m_mutex.lock();
                m_tau=tau;
                m_mutex.unlock();
                return tau;
            }
        }

        //TODO add saturation function for tau!!



        msg.header.stamp=time.tRc();

        msg.time=time.tD();

        for(int i=0;i<STD_DOF;i++)
        {
            msg.q[i]=current.q(i);
            msg.qp[i]=current.qp(i);
            msg.qpp[i]=current.qpp(i);

            msg.qd[i]=0.0;
            msg.qpd[i]=0.0;
            msg.qppd[i]=0.0;

            msg.Dq[i]=0.0;
            msg.Dqp[i]=0.0;

            msg.torques[i]=tau(i);
        }

        m_pubCtrlData.publish(msg);

        m_mutex.lock();
        m_tau=tau;
        m_mutex.unlock();
        return tau;
    }
    if(m_runState==STATE_STOP)
    {
        double nQp=current.qp.norm();

        if(nQp<0.001)
        {
            m_failed=false;
            m_initialized=false;
            m_started=false;
            m_running=false;
            m_finished=true;

            m_mutex.lock();
            m_tau=tau;
            m_mutex.unlock();

            return tau;
        }

        //TODO: we need to add only one stop function!!!

        tau=-m_Kd*(current.qp);

        msg.header.stamp=time.tRc();

        msg.time=time.tD();

        for(int i=0;i<STD_DOF;i++)
        {
            msg.q[i]=current.q(i);
            msg.qp[i]=current.qp(i);
            msg.qpp[i]=current.qpp(i);

            msg.qd[i]=0.0;
            msg.qpd[i]=0.0;
            msg.qppd[i]=0.0;

            msg.Dq[i]=0.0;
            msg.Dqp[i]=0.0;

            msg.torques[i]=tau(i);
        }

        m_pubCtrlData.publish(msg);

        m_mutex.lock();
        m_tau=tau;
        m_mutex.unlock();

        return tau;
    }

    std::stringstream s;
    s<<getName().toStdString()<<" update(): Unknown RunState: "<<m_runState;
    m_error=true;
    m_errorString=s.str().c_str();
    ROS_ERROR_STREAM(m_errorString.toStdString());


    m_failed=true;
    m_initialized=false;
    m_started=false;
    m_running=false;
    m_finished=false;

    m_mutex.lock();
    m_tau=tau;
    m_mutex.unlock();
    return tau;


}

bool DControl::stop()
{
    m_runState=STATE_STOP;
    return true;
  
}



}
}
