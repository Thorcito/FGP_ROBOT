#include<tum_ics_ur_robot_controllers/JointImpedanceControl.h>

#include<tum_ics_ur_robot_msgs/ControlData.h>

namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

JointImpedanceControl::JointImpedanceControl(
        double controlPeriod,
        double weight,
        double cutoff,
        const QString& name):
    ControlEffort(name,STANDARD_TYPE,JOINT_SPACE,weight),
    m_startFlag(false),
    m_Kp(MatrixDOFd::Zero()),
    m_Kd(MatrixDOFd::Zero()),
    m_Kint(MatrixDOFd::Zero()),
    m_goal(VectorDOFd::Zero()),
    m_totalTime(100.0),
    m_DeltaQ(VectorDOFd::Zero()),
    m_DeltaQ_1(VectorDOFd::Zero()),
    m_DeltaQp(VectorDOFd::Zero()),
    m_DeltaQp_1(VectorDOFd::Zero()),
    m_iDeltaQ(VectorDOFd::Zero()),
    m_iDeltaQ_1(VectorDOFd::Zero()),
    m_controlPeriod(controlPeriod),
    m_controlPeriod_2(controlPeriod/2.0),
    m_bfCutof(cutoff)
{
    pubCtrlData=n.advertise<tum_ics_ur_robot_msgs::ControlData>("JointImpCtrlData",100);

//    m_controlPeriod_2=m_controlPeriod/2.0;

    ROS_INFO_STREAM("JointImpedanceCtrl Control Period: "<<m_controlPeriod<<" ("<<m_controlPeriod_2<<")");

    m_bfQ.setFilterParams(m_bfCutof,m_controlPeriod);
    m_bfQp.setFilterParams(m_bfCutof,m_controlPeriod);
    m_bfDv.setFilterParams(m_bfCutof,m_controlPeriod);

}

JointImpedanceControl::~JointImpedanceControl()
{

}

void JointImpedanceControl::setQInit(const JointState& qinit)
{
    m_qInit=qinit;
}
void JointImpedanceControl::setQHome(const JointState& qhome)
{
    m_qHome=qhome;
}
void JointImpedanceControl::setQPark(const JointState& qpark)
{
    m_qPark=qpark;
}


bool JointImpedanceControl::init()
{
    std::string ns="~joint_imp_ctrl";
    std::stringstream s;

    if (!ros::param::has(ns))
    {
        s<<"JointImpedanceControl init(): Control gains not defined --"<<ns<<"--, did you load them in the rosparam server??";
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
        s<<"JointImpedanceControl init(): Wrong number of d_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_Kd(i,i)=p[i];
    }

    ROS_WARN_STREAM("Kd: \n"<<m_Kd);

    /////P GAINS
    s.str("");
    s<<ns<<"/gains_p";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"JointImpedanceControl init(): Wrong number of p_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_Kp(i,i)=p[i];
    }
    ROS_WARN_STREAM("Kp: \n"<<m_Kp);

    /////I GAINS
    s.str("");
    s<<ns<<"/gains_int";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"JointImpedanceControl init(): Wrong number of i_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_Kint(i,i)=p[i];
    }

    ROS_WARN_STREAM("Interaction Gain KInt: \n"<<m_Kint);

    /////GOAL
    s.str("");
    s<<ns<<"/goal";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"JointImpedanceControl init(): Wrong number of joint goals --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_goal(i)=p[i];
    }


    ROS_WARN_STREAM("Goal [DEG]: \n"<<m_goal.transpose());

    m_goal=DEG2RAD(m_goal);

    ROS_WARN_STREAM("Goal [RAD]: \n"<<m_goal.transpose());

    return true;
}
bool JointImpedanceControl::start()
{
    return true;
}
VectorDOFd JointImpedanceControl::update(const RobotTime& time, const JointState &current)
{
    if(!m_startFlag)
    {
        m_qStart=current.q;

        m_goal=m_qStart;

        m_startFlag=true;

        m_DeltaQ=current.q-m_qStart;
        m_DeltaQ_1=m_DeltaQ;

        m_iDeltaQ.setZero();
        m_iDeltaQ_1=m_iDeltaQ;
    }


    VectorDOFd tau;

    tau.setZero();


    //m_goal is used to define the offset for the joint compliance

    VectorDOFd qf=m_bfQ.filter(current.q);
    VectorDOFd qpf=m_bfQp.filter(current.qp);

    m_DeltaQ=qf-m_goal;
    m_DeltaQp=qpf;


    tau=-m_Kd*m_DeltaQp-m_Kp*m_DeltaQ;

    //Interaction Gain defines the amount of motion we allow to each joint is
    //a matrix between 0-1
    tau=m_Kint*tau;

    for(int i=0;i<STD_DOF;i++)
    {
        if((isnan(tau(i)))||(isinf(tau(i))))
        {
            std::stringstream s;
            s<<"JointImpedanceControl update(): The control output is nan/inf: "<<tau.transpose();
            m_error=true;
            m_errorString=s.str().c_str();
            ROS_ERROR_STREAM(m_errorString.toStdString());
        }
    }

    //TODO add saturation function for tau!!

    tum_ics_ur_robot_msgs::ControlData msg;

    msg.header.stamp=time.tRc();

    msg.time=time.tD();

    VectorDOFd qpd(VectorDOFd::Zero());
    VectorDOFd qppd(VectorDOFd::Zero());

    for(int i=0;i<STD_DOF;i++)
    {
        msg.q[i]=current.q(i);
        msg.qp[i]=current.qp(i);
        msg.qpp[i]=current.qpp(i);

        msg.qd[i]=m_goal(i);
        msg.qpd[i]=qpd(i);
        msg.qppd[i]=qppd(i);

        msg.Dq[i]=m_DeltaQ(i);
        msg.Dqp[i]=m_DeltaQp(i);

        msg.torques[i]=tau(i);
    }

    pubCtrlData.publish(msg);


    //Set history for the next loop
    m_DeltaQ_1=m_DeltaQ;
    m_iDeltaQ_1=m_iDeltaQ;

    return weight()*tau;

}

bool JointImpedanceControl::stop()
{
    return true;
}



}
}
