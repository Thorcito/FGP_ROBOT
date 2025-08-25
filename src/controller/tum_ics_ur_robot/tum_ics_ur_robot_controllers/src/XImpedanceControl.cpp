#include<tum_ics_ur_robot_controllers/XImpedanceControl.h>

#include<tum_ics_ur_robot_msgs/ControlData.h>

namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

XImpedanceControl::XImpedanceControl(Robot::KinematicModel *kModel,
                                     double controlPeriod,
                                     double weight,
                                     const QString& name):
    ControlEffort(name,STANDARD_TYPE,JOINT_SPACE,weight),
    m_startFlag(false),
    m_Kpx(Matrix3d::Zero()),
    m_Kdx(Matrix3d::Zero()),
    m_Kint(MatrixDOFd::Zero()),
    m_Kd(MatrixDOFd::Zero()),
    m_xGoal(Vector3d::Zero()),
    m_totalTime(100.0),
    m_DeltaX(Vector6d::Zero()),
    m_DeltaX_1(Vector6d::Zero()),
    m_DeltaXp(Vector6d::Zero()),
    m_DeltaXp_1(Vector6d::Zero()),
    m_iDeltaX(Vector6d::Zero()),
    m_iDeltaX_1(Vector6d::Zero()),
    m_controlPeriod(controlPeriod),
    m_controlPeriod_2(controlPeriod/2.0),
    m_kinematicModel(kModel),
    m_Xef(Vector6d::Zero()),
    m_Xd(Vector6d::Zero()),
    m_stopKd(MatrixDOFd::Zero())
{
    pubCtrlData=n.advertise<tum_ics_ur_robot_msgs::ControlData>("XImpCtrlData",100);

    //    m_controlPeriod_2=m_controlPeriod/2.0;

    ROS_INFO_STREAM("XImpedanceCtrl Control Period: "<<m_controlPeriod<<" ("<<m_controlPeriod_2<<")");

}

XImpedanceControl::~XImpedanceControl()
{

}

void XImpedanceControl::setQInit(const JointState& qinit)
{
    m_qInit=qinit;
}
void XImpedanceControl::setQHome(const JointState& qhome)
{
    m_qHome=qhome;
}
void XImpedanceControl::setQPark(const JointState& qpark)
{
    m_qPark=qpark;
}


bool XImpedanceControl::init()
{
    std::string ns="~x_imp_ctrl";
    std::stringstream s;

    if (!ros::param::has(ns))
    {
        s<<"XImpedanceControl init(): Control gains not defined --"<<ns<<"--, did you load them in the rosparam server??";
        m_error=true;
        m_errorString=s.str().c_str();
        ROS_ERROR_STREAM(s.str());
        return false;
    }


    VDouble p;



    /////Dx GAINS

    s<<ns<<"/gains_d";
    ros::param::get(s.str(),p);

    if(p.size()<3)
    {
        s.str("");
        s<<"XImpedanceControl init(): Wrong number of d_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<3;i++)
    {
        m_Kdx(i,i)=p[i];
    }

    ROS_WARN_STREAM("Kdx: \n"<<m_Kdx);

    /////D GAINS
    s.str("");
    s<<ns<<"/gains_dq";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"XImpedanceControl init(): Wrong number of dq_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_Kd(i,i)=p[i];
    }

    ROS_WARN_STREAM("Kdx: \n"<<m_Kdx);

    /////P GAINS
    s.str("");
    s<<ns<<"/gains_p";
    ros::param::get(s.str(),p);

    if(p.size()<3)
    {
        s.str("");
        s<<"XImpedanceControl init(): Wrong number of p_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<3;i++)
    {
        m_Kpx(i,i)=p[i];
    }
    ROS_WARN_STREAM("Kp: \n"<<m_Kpx);

    /////I GAINS
    s.str("");
    s<<ns<<"/gains_int";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"XImpedanceControl init(): Wrong number of i_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_Kint(i,i)=p[i];
    }

    ROS_WARN_STREAM("Interaction Gain KInt: \n"<<m_Kint);

    //stopKd

    s.str("");
    s<<ns<<"/gains_d_stop";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"XImpedanceControl init(): Wrong number of stopKd_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<6;i++)
    {
        m_stopKd(i,i)=p[i];
    }

    ROS_WARN_STREAM("Interaction Gain stopKd: \n"<<m_stopKd);


    /////GOAL
    s.str("");
    s<<ns<<"/goal";
    ros::param::get(s.str(),p);

    if(p.size()<3)
    {
        s.str("");
        s<<"XImpedanceControl init(): Wrong number of X position goals --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }


    for(int i=0;i<3;i++)
    {
        m_xGoal(i)=p[i];
    }


    ROS_WARN_STREAM("Goal [m]: \n"<<m_xGoal.transpose());

    return true;
}
bool XImpedanceControl::start()
{
    //For now the start, update and stop functions are states in update()
    m_runState=STATE_START;
    ROS_WARN_STREAM(getName().toStdString()<<"Start() Running state changed to: "<<m_runState);
    return true;
}
VectorDOFd XImpedanceControl::update(const RobotTime& time, const JointState &current)
{
    VectorDOFd tau;

    tum_ics_ur_robot_msgs::ControlData msg;

    tau.setZero();

    if(m_runState==STATE_INIT)
    {
        m_startFlag=false;

        m_failed=false;
        m_initialized=true;
        m_started=false;
        m_running=false;
        m_finished=false;

        m_mutex.lock();
        m_tau=tau;
        m_mutex.unlock();

        return tau;
    }


    if(m_runState==STATE_START)
    {
        m_startFlag=false;


        m_failed=false;
        m_initialized=false;
        m_started=true;
        m_running=false;
        m_finished=false;


        m_qStart=current.q;

        m_runState=STATE_UPDATE;
        ROS_WARN_STREAM(getName().toStdString()<<"STATE_START Running state changed to: "<<m_runState);


        //Set the initial X position
        m_xini=m_kinematicModel->xef_0();
        m_xGoal=m_xini;

        m_Xd<<m_xini,0,0,0;


        m_DeltaX.setZero();
        m_DeltaX_1=m_DeltaX;

        m_iDeltaX.setZero();
        m_iDeltaX_1=m_iDeltaX;
    }


    if(m_runState==STATE_UPDATE)
    {
        if(!m_startFlag)
        {
            m_qStart=current.q;

            m_failed=false;
            m_initialized=false;
            m_started=false;
            m_running=true;
            m_finished=false;

            m_startFlag=true;
        }

        //m_goal is used to define the offset for the joint compliance

        //Get the current ef position (this position is updated in robotArm update function)
        Vector3d xef_0=m_kinematicModel->xef_0();

        m_Xef<<xef_0,0,0,0;

        Vector3d DeltaX=xef_0-m_xini;
        m_DeltaX<<DeltaX,0,0,0;
        Vector3d DeltaXp=m_kinematicModel->xefp_0();//current X velocity;

        m_DeltaXp<<DeltaXp,0,0,0;

        //Virtual Force

        Vector3d Fc_0;
        Vector6d Wef_0;

//        ROS_INFO_STREAM("Xef: "<<xef_0.transpose()<<" Xini: "<<m_xini.transpose());

        Fc_0=-m_Kpx*DeltaX-m_Kdx*DeltaXp;

        Wef_0<<Fc_0,0,0,0;

//        ROS_INFO_STREAM("Wef_0: "<<Wef_0.transpose());

        tau=m_kinematicModel->Jef_0().transpose()*Wef_0-m_Kd*(current.qp);

//        ROS_WARN_STREAM("Tau: "<<tau.transpose());

        //Interaction Gain defines the amount of motion we allow to each joint is
        //a matrix between 0-1
        tau=m_Kint*tau;

        for(int i=0;i<STD_DOF;i++)
        {
            if((isnan(tau(i)))||(isinf(tau(i))))
            {
                std::stringstream s;
                s<<"XImpedanceControl update(): The control output is nan/inf: "<<tau.transpose();
                m_error=true;
                m_errorString=s.str().c_str();
                ROS_ERROR_STREAM(m_errorString.toStdString());
            }
        }

        //TODO add saturation function for tau!!

        tum_ics_ur_robot_msgs::ControlData msg;

        msg.header.stamp=time.tRc();

        msg.time=time.tD();

        VectorDOFd qd(VectorDOFd::Zero());
        VectorDOFd qpd(VectorDOFd::Zero());
        VectorDOFd qppd(VectorDOFd::Zero());

        for(int i=0;i<STD_DOF;i++)
        {
            msg.q[i]=current.q(i);
            msg.qp[i]=current.qp(i);
            msg.qpp[i]=current.qpp(i);

            msg.qd[i]=qd(i);
            msg.qpd[i]=qpd(i);
            msg.qppd[i]=qppd(i);

            msg.Dq[i]=m_DeltaX(i);
            msg.Dqp[i]=m_DeltaXp(i);

            msg.Xef_0[i]=m_Xef(i);
            msg.Xd_0[i]=m_Xd(i);

            msg.torques[i]=tau(i);
        }

        pubCtrlData.publish(msg);


        //Set history for the next loop
        m_DeltaX_1=m_DeltaX;
        m_iDeltaX_1=m_iDeltaX;

//        ROS_ERROR_STREAM("m_runState:"<<m_runState<<" m_failed:"<<m_failed<<" m_initialized:"<<m_initialized<<" m_started:"<<m_started<<" m_running: "<<m_running<<" m_finished: "<<m_finished<<" m_startFlag:"<<m_startFlag);
//        return weight()*tau;
        return tau;
    }

    if(m_runState==STATE_STOP)
    {

        //        VectorDOFd G=Gc(current);

        //Total Arm volocity
        double nQp=current.qp.norm();

        ROS_INFO_STREAM("Qp:"<<current.qp.transpose());

        if(nQp<0.01)
        {
            //reset the initialization flag (used in STATE_UPDATE to compute Bias)

            m_startFlag=false;

            m_failed=false;
            m_initialized=false;
            m_started=false;
            m_running=false;
            m_finished=true;

            ROS_ERROR_STREAM("I reached it!!!");
            //If the total velocity is close to zero just send G to keep the arm steady


            m_mutex.lock();
            m_tau=tau;
            m_mutex.unlock();

            return tau;
        }


        tau=-m_stopKd*(current.qp);

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

//        m_pubCtrlData.publish(msg);

        setWeight(1.0);

        m_mutex.lock();
        m_tau=tau;
        m_mutex.unlock();
        return tau;
    }

    setWeight(1.0);

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

bool XImpedanceControl::stop()
{
    //For now the start, update and stop functions are states in update()
    m_runState=STATE_STOP;
//    ROS_WARN_STREAM("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$STOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP!!!!!!!!!!!!!!");
    return true;
}



}
}
