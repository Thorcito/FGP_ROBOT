#include<tum_ics_ur_robot_controllers/PIDControl.h>

#include<tum_ics_ur_robot_msgs/ControlData.h>

namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

PIDControl::PIDControl(double controlPeriod, double weight, const QString& name):
    ControlEffort(name,SPLINE_TYPE,JOINT_SPACE, weight),
    m_startFlag(false),
    m_Kp(MatrixDOFd::Zero()),
    m_Kd(MatrixDOFd::Zero()),
    m_Ki(MatrixDOFd::Zero()),
    m_goal(VectorDOFd::Zero()),
    m_totalTime(100.0),
    m_DeltaQ(VectorDOFd::Zero()),
    m_DeltaQ_1(VectorDOFd::Zero()),
    m_DeltaQp(VectorDOFd::Zero()),
    m_DeltaQp_1(VectorDOFd::Zero()),
    m_iDeltaQ(VectorDOFd::Zero()),
    m_iDeltaQ_1(VectorDOFd::Zero()),
    m_controlPeriod(controlPeriod),
    m_controlPeriod_2(controlPeriod/2.0)
{
    pubCtrlData=n.advertise<tum_ics_ur_robot_msgs::ControlData>("PIDCtrlData",100);

//    m_controlPeriod_2=m_controlPeriod/2.0;

    ROS_INFO_STREAM("PIDCtrl Control Period: "<<m_controlPeriod<<" ("<<m_controlPeriod_2<<")");

}

PIDControl::~PIDControl()
{

}

void PIDControl::setQInit(const JointState& qinit)
{
    m_qInit=qinit;
}
void PIDControl::setQHome(const JointState& qhome)
{
    m_qHome=qhome;
}
void PIDControl::setQPark(const JointState& qpark)
{
    m_qPark=qpark;
}


bool PIDControl::init()
{
    std::string ns="~pid_ctrl";
    std::stringstream s;

    if (!ros::param::has(ns))
    {
        s<<"PIDControl init(): Control gains not defined --"<<ns<<"--, did you load them in the rosparam server??";
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
        s<<"PIDControl init(): Wrong number of d_gains --"<<p.size()<<"--";
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
        s<<"PIDControl init(): Wrong number of p_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_Kp(i,i)=p[i]/m_Kd(i,i);
    }
    ROS_WARN_STREAM("Kp: \n"<<m_Kp);

    /////I GAINS
    s.str("");
    s<<ns<<"/gains_i";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"PIDControl init(): Wrong number of i_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        //        m_Ki(i,i)=p[i];
        m_Ki(i,i)=m_Kp(i,i)*m_Kp(i,i)/4.0;
    }

    ROS_WARN_STREAM("Ki: \n"<<m_Ki);

    /////GOAL
    s.str("");
    s<<ns<<"/goal";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"PIDControl init(): Wrong number of joint goals --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_goal(i)=p[i];
    }
    m_totalTime=p[STD_DOF];

    if(!(m_totalTime>0))
    {
        m_totalTime=100.0;
    }

    ROS_WARN_STREAM("Goal [DEG]: \n"<<m_goal.transpose());
    ROS_WARN_STREAM("Total Time [s]: "<<m_totalTime);


    m_goal=DEG2RAD(m_goal);

    ROS_WARN_STREAM("Goal [RAD]: \n"<<m_goal.transpose());

    return true;
}
bool PIDControl::start()
{
    return true;
}
VectorDOFd PIDControl::update(const RobotTime& time, const JointState &current)
{
    if(!m_startFlag)
    {
        m_qStart=current.q;
        m_startFlag=true;

        m_DeltaQ=current.q-m_qStart;
        m_DeltaQ_1=m_DeltaQ;

        m_iDeltaQ.setZero();
        m_iDeltaQ_1=m_iDeltaQ;
    }


    VectorDOFd tau;

    tau.setZero();

    VVectorDOFd vQd;

    vQd=getSpline5<VVectorDOFd,VectorDOFd>(m_qStart,m_goal,time.tD(),m_totalTime);


    m_DeltaQ=current.q-vQd[0];
    m_DeltaQp=current.qp-vQd[1];


    for(int i=0;i<STD_DOF;i++)
    {
        IntHeun(m_DeltaQ(i),m_DeltaQ_1(i),m_iDeltaQ_1(i),m_iDeltaQ(i),m_controlPeriod_2);
    }


    JointState js_r;

    js_r.qp=vQd[1]-m_Kp*m_DeltaQ-m_Ki*m_iDeltaQ;
    js_r.qpp=vQd[2]-m_Kp*m_DeltaQp-m_Ki*m_DeltaQ;


    VectorDOFd Sq=current.qp-js_r.qp;


    tau=-m_Kd*Sq;

    for(int i=0;i<STD_DOF;i++)
    {
        if((isnan(tau(i)))||(isinf(tau(i))))
        {
            std::stringstream s;
            s<<"PIDControl update(): The control output is nan/inf: "<<tau.transpose();
            m_error=true;
            m_errorString=s.str().c_str();
            ROS_ERROR_STREAM(m_errorString.toStdString());
        }
    }

    //TODO add saturation function for tau!!

    tum_ics_ur_robot_msgs::ControlData msg;

    msg.header.stamp=time.tRc();

    msg.time=time.tD();

    for(int i=0;i<STD_DOF;i++)
    {
        msg.q[i]=current.q(i);
        msg.qp[i]=current.qp(i);
        msg.qpp[i]=current.qpp(i);

        msg.qd[i]=vQd[0](i);
        msg.qpd[i]=vQd[1](i);
        msg.qppd[i]=vQd[2](i);

        msg.Dq[i]=m_DeltaQ(i);
        msg.Dqp[i]=m_DeltaQp(i);

        msg.torques[i]=current.tau(i);
    }

    pubCtrlData.publish(msg);


    //Set history for the next loop
    m_DeltaQ_1=m_DeltaQ;
    m_iDeltaQ_1=m_iDeltaQ;

    return weight()*tau;

}

bool PIDControl::stop()
{
    return true;
}



}
}
