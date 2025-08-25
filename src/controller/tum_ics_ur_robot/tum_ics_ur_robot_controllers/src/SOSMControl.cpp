#include<tum_ics_ur_robot_controllers/SOSMControl.h>

#include<tum_ics_ur_robot_msgs/ControlData.h>

namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

SOSMControl::SOSMControl(Robot::KinematicModel *kModel,
                         Robot::DynamicModel *dModel,
                         double weight,
                         const QString& name):
    ControlEffort(name,SPLINE_TYPE,JOINT_SPACE,weight),
    m_kinematicModel(kModel),
    m_dynamicModel(dModel),
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
    m_S_dt0(VectorDOFd::Zero()),
    m_TanhSq_1(VectorDOFd::Zero()),
    m_iTanhSq_1(VectorDOFd::Zero()),
    m_initErrorS(true),
    m_kappa(0.0),
    m_lambda(0.0)
{
    pubCtrlData=n.advertise<tum_ics_ur_robot_msgs::ControlData>("SOSMCtrlData",100);

    //Get control period from dynamic model (this is obtained from config file and set for both the robot loop and dynMod loop)
    if(dModel!=NULL)
    {
        m_controlPeriod=m_dynamicModel->getCtrlPeriod();
    }
    else
    {
        m_controlPeriod=0.008; //set the control period to the standard 8 ms
    }

    m_controlPeriod_2=m_controlPeriod/2.0;

    ROS_INFO_STREAM("SimpleEffortCtrl Control Period: "<<m_controlPeriod<<" ("<<m_controlPeriod_2<<")");

}

SOSMControl::~SOSMControl()
{

}

void SOSMControl::setQInit(const JointState& qinit)
{
    m_qInit=qinit;
}
void SOSMControl::setQHome(const JointState& qhome)
{
    m_qHome=qhome;
}
void SOSMControl::setQPark(const JointState& qpark)
{
    m_qPark=qpark;
}


bool SOSMControl::init()
{
    std::string ns="~sosm_ctrl";
    std::stringstream s;

    if (!ros::param::has(ns))
    {
        s<<"SOSMControl init(): Control gains not defined --"<<ns<<"--, did you load them in the rosparam server??";
        m_error=true;
        m_errorString=s.str().c_str();
        ROS_ERROR_STREAM(s.str());
        return false;
    }


    VDouble p;

    ROS_WARN_STREAM("Kp: \n"<<m_Kp);

    /////D GAINS

    s<<ns<<"/gains_d";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"SOSMControl init(): Wrong number of d_gains --"<<p.size()<<"--";
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
        s<<"SOSMControl init(): Wrong number of p_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_Kp(i,i)=p[i]/m_Kd(i,i);
    }

    /////I GAINS
    s.str("");
    s<<ns<<"/gains_i";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"SOSMControl init(): Wrong number of i_gains --"<<p.size()<<"--";
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
        s<<"SOSMControl init(): Wrong number of joint goals --"<<p.size()<<"--";
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


    //BIAS
    s.str("");
    s<<ns<<"/lambda";
    ros::param::get(s.str(),m_lambda);
    ROS_WARN_STREAM("Bias Lambda: \n"<<m_lambda);

    s.str("");
    s<<ns<<"/kappa";
    ros::param::get(s.str(),m_kappa);
    ROS_WARN_STREAM("Bias Kappa: \n"<<m_kappa);




    Vector12d l=m_dynamicModel->L();

    ROS_WARN_STREAM("L: "<<l.transpose());

    L1=l(0);
    L2=l(1);
    L3=l(2);
    L4=l(3);
    L5=l(4);
    L6=l(5);
    L7=l(6);
    L8=l(7);
    L9=l(8);
    L10=l(9);
    L11=l(10);
    L12=l(11);

    VectorDOFd ms=m_dynamicModel->m();

    ROS_WARN_STREAM("m: "<<ms.transpose());


    m1=ms(0);
    m2=ms(1);
    m3=ms(2);
    m4=ms(3);
    m5=ms(4);
    m6=ms(5);

    return true;
}
bool SOSMControl::start()
{
return true;
}
VectorDOFd SOSMControl::update(const RobotTime& time, const JointState &current)
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

    VVectorDOFd vQd=getSpline5<VVectorDOFd,VectorDOFd>(m_qStart,m_goal,time.tD(),m_totalTime);

    //Joint position/velocity errors
    m_DeltaQ=current.q-vQd[0];
    m_DeltaQp=current.qp-vQd[1];

    for(int i=0;i<STD_DOF;i++)
    {
        IntHeun(m_DeltaQ(i),m_DeltaQ_1(i),m_iDeltaQ_1(i),m_iDeltaQ(i),m_controlPeriod_2);
    }

    VectorDOFd S=m_DeltaQp+m_Kp*m_DeltaQ;

    if(m_initErrorS)
    {
        //initial error condition
        m_S_dt0=S;
        m_initErrorS=false;
    }

//    ROS_INFO_STREAM("time: "<<time.tD());

    VectorDOFd S_d=m_S_dt0*exp(-m_kappa*time.tD());

    VectorDOFd Sp_d=-m_kappa*m_S_dt0*exp(-m_kappa*time.tD());

    VectorDOFd Sq=S-S_d;

    VectorDOFd TanhSq,iTanhSq;


    for(unsigned int i=0;i<STD_DOF;i++)
    {
        TanhSq(i)=tanh(m_lambda*Sq(i));
        IntHeun(TanhSq(i),m_TanhSq_1(i),m_iTanhSq_1(i),iTanhSq(i),m_controlPeriod_2);

    }

    JointState js_r;

    js_r.qp  = vQd[1] - m_Kp*m_DeltaQ  + S_d  - m_Ki*iTanhSq;
    js_r.qpp = vQd[2]- m_Kp*m_DeltaQp + Sp_d - m_Ki*TanhSq;

    VectorDOFd Sr=current.qp-js_r.qp;


    VectorDOFd G=m_dynamicModel->G();//Gc(current);


    MatrixRegressor Yr=m_dynamicModel->Yrc(current,js_r);
    VectorRegressor Th=m_dynamicModel->Th();
    MatrixDOFd B=m_dynamicModel->B();

    tau=-m_Kd*Sr + Yr*Th + B*current.qp;


    for(int i=0;i<STD_DOF;i++)
    {
        if((isnan(tau(i)))||(isinf(tau(i))))
        {
            std::stringstream s;
            s<<"SOSMControl update(): The control output is nan/inf: "<<tau.transpose();
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

    //ROS_INFO_STREAM("Yth: "<<Yth.transpose());



    //Set history for the next loop
    m_DeltaQ_1=m_DeltaQ;
    m_iDeltaQ_1=m_iDeltaQ;

    m_TanhSq_1=TanhSq;
    m_iTanhSq_1=iTanhSq;

    return tau;

}

bool SOSMControl::stop()
{
    return true;
}

VectorDOFd SOSMControl::Gc(const JointState& js)
{
    VectorDOFd G;

    double q1=js.q(0);
    double q2=js.q(1);
    double q3=js.q(2);
    double q4=js.q(3);
    double q5=js.q(4);


    double sq1 = sin(q1);
    double cq2 = cos(q2);
    double cq1 = cos(q1);
    double smq3Mq1mq2 = sin(-q3+q1-q2);
    double sq3Mq1Mq2 = sin(q3+q1+q2);
    double sq1mq2 = sin(q1-q2);
    double sq1Mq2 = sin(q1+q2);
    double cmq3Mq1mq2 = cos(-q3+q1-q2);
    double cq3Mq1Mq2 = cos(q3+q1+q2);
    double cq1mq2 = cos(q1-q2);
    double cq1Mq2 = cos(q1+q2);
    double cq4Mq3Mq1Mq2 = cos(q4+q3+q1+q2);
    double cmq4mq3Mq1mq2 = cos(-q4-q3+q1-q2);
    double smq4mq3Mq1mq2 = sin(-q4-q3+q1-q2);
    double sq4Mq3Mq1Mq2 = sin(q4+q3+q1+q2);
    double sq5 = sin(q5);
    double cq5 = cos(q5);
    double sq2 = sin(q2);
    double cq2Mq3 = cos(q2+q3);
    double sq2Mq3Mq4 = sin(q2+q3+q4);
    double cq2Mq3Mq4 = cos(q2+q3+q4);




    Vector3d g0=m_dynamicModel->g0();

    double gx=g0(0);
    double gy=g0(1);
    double gz=g0(2);






    G(0,0)=-m2*gx*sq1*cq2*L8+m2*gy*cq1*cq2*L8+m3*gx*(-u1_2*L9*smq3Mq1mq2-u1_2*L9*sq3Mq1Mq2-u1_2*L2*sq1mq2-u1_2*L2*sq1Mq2)+m3*gy*(u1_2*L9*cmq3Mq1mq2+u1_2*L9*cq3Mq1Mq2+u1_2*L2*cq1mq2+u1_2*L2*cq1Mq2)+m4*gx*(cq1*L10-u1_2*L3*smq3Mq1mq2-u1_2*L3*sq3Mq1Mq2-u1_2*L2*sq1mq2-u1_2*L2*sq1Mq2)+m4*gy*(sq1*L10+u1_2*L3*cmq3Mq1mq2+u1_2*L3*cq3Mq1Mq2+u1_2*L2*cq1mq2+u1_2*L2*cq1Mq2)+m5*gx*((u1_2*cq4Mq3Mq1Mq2-u1_2*cmq4mq3Mq1mq2)*L11+cq1*L4-u1_2*L3*smq3Mq1mq2-u1_2*L3*sq3Mq1Mq2-u1_2*L2*sq1mq2-u1_2*L2*sq1Mq2)+m5*gy*((-u1_2*smq4mq3Mq1mq2+u1_2*sq4Mq3Mq1Mq2)*L11+sq1*L4+u1_2*L3*cmq3Mq1mq2+u1_2*L3*cq3Mq1Mq2+u1_2*L2*cq1mq2+u1_2*L2*cq1Mq2)+m6*gx*((-(-u1_2*smq4mq3Mq1mq2-u1_2*sq4Mq3Mq1Mq2)*sq5+cq1*cq5)*L12+(u1_2*cq4Mq3Mq1Mq2-u1_2*cmq4mq3Mq1mq2)*L5+cq1*L4-u1_2*L3*smq3Mq1mq2-u1_2*L3*sq3Mq1Mq2-u1_2*L2*sq1mq2-u1_2*L2*sq1Mq2)+m6*gy*((-(u1_2*cmq4mq3Mq1mq2+u1_2*cq4Mq3Mq1Mq2)*sq5+sq1*cq5)*L12+(-u1_2*smq4mq3Mq1mq2+u1_2*sq4Mq3Mq1Mq2)*L5+sq1*L4+u1_2*L3*cmq3Mq1mq2+u1_2*L3*cq3Mq1Mq2+u1_2*L2*cq1mq2+u1_2*L2*cq1Mq2);
    G(1,0)=-m2*gx*cq1*sq2*L8-m2*gy*sq1*sq2*L8+m2*gz*cq2*L8+m3*gx*(u1_2*L9*smq3Mq1mq2-u1_2*L9*sq3Mq1Mq2+u1_2*L2*sq1mq2-u1_2*L2*sq1Mq2)+m3*gy*(u1_2*L9*cq3Mq1Mq2-u1_2*L9*cmq3Mq1mq2+u1_2*L2*cq1Mq2-u1_2*L2*cq1mq2)+m3*gz*(L9*cq2Mq3+cq2*L2)+m4*gx*(u1_2*L3*smq3Mq1mq2-u1_2*L3*sq3Mq1Mq2+u1_2*L2*sq1mq2-u1_2*L2*sq1Mq2)+m4*gy*(u1_2*L3*cq3Mq1Mq2-u1_2*L3*cmq3Mq1mq2+u1_2*L2*cq1Mq2-u1_2*L2*cq1mq2)+m4*gz*(L3*cq2Mq3+cq2*L2)+m5*gx*((u1_2*cmq4mq3Mq1mq2+u1_2*cq4Mq3Mq1Mq2)*L11+u1_2*L3*smq3Mq1mq2-u1_2*L3*sq3Mq1Mq2+u1_2*L2*sq1mq2-u1_2*L2*sq1Mq2)+m5*gy*((u1_2*sq4Mq3Mq1Mq2+u1_2*smq4mq3Mq1mq2)*L11+u1_2*L3*cq3Mq1Mq2-u1_2*L3*cmq3Mq1mq2+u1_2*L2*cq1Mq2-u1_2*L2*cq1mq2)+m5*gz*(sq2Mq3Mq4*L11+L3*cq2Mq3+cq2*L2)+m6*gx*(-(u1_2*smq4mq3Mq1mq2-u1_2*sq4Mq3Mq1Mq2)*sq5*L12+(u1_2*cmq4mq3Mq1mq2+u1_2*cq4Mq3Mq1Mq2)*L5+u1_2*L3*smq3Mq1mq2-u1_2*L3*sq3Mq1Mq2+u1_2*L2*sq1mq2-u1_2*L2*sq1Mq2)+m6*gy*(-(u1_2*cq4Mq3Mq1Mq2-u1_2*cmq4mq3Mq1mq2)*sq5*L12+(u1_2*sq4Mq3Mq1Mq2+u1_2*smq4mq3Mq1mq2)*L5+u1_2*L3*cq3Mq1Mq2-u1_2*L3*cmq3Mq1mq2+u1_2*L2*cq1Mq2-u1_2*L2*cq1mq2)+m6*gz*(-cq2Mq3Mq4*sq5*L12+sq2Mq3Mq4*L5+L3*cq2Mq3+cq2*L2);
    G(2,0)=m3*gx*(u1_2*L9*smq3Mq1mq2-u1_2*L9*sq3Mq1Mq2)+m3*gy*(u1_2*L9*cq3Mq1Mq2-u1_2*L9*cmq3Mq1mq2)+m3*gz*L9*cq2Mq3+m4*gx*(u1_2*L3*smq3Mq1mq2-u1_2*L3*sq3Mq1Mq2)+m4*gy*(u1_2*L3*cq3Mq1Mq2-u1_2*L3*cmq3Mq1mq2)+m4*gz*L3*cq2Mq3+m5*gx*((u1_2*cmq4mq3Mq1mq2+u1_2*cq4Mq3Mq1Mq2)*L11+u1_2*L3*smq3Mq1mq2-u1_2*L3*sq3Mq1Mq2)+m5*gy*((u1_2*sq4Mq3Mq1Mq2+u1_2*smq4mq3Mq1mq2)*L11+u1_2*L3*cq3Mq1Mq2-u1_2*L3*cmq3Mq1mq2)+m5*gz*(sq2Mq3Mq4*L11+L3*cq2Mq3)+m6*gx*(-(u1_2*smq4mq3Mq1mq2-u1_2*sq4Mq3Mq1Mq2)*sq5*L12+(u1_2*cmq4mq3Mq1mq2+u1_2*cq4Mq3Mq1Mq2)*L5+u1_2*L3*smq3Mq1mq2-u1_2*L3*sq3Mq1Mq2)+m6*gy*(-(u1_2*cq4Mq3Mq1Mq2-u1_2*cmq4mq3Mq1mq2)*sq5*L12+(u1_2*sq4Mq3Mq1Mq2+u1_2*smq4mq3Mq1mq2)*L5+u1_2*L3*cq3Mq1Mq2-u1_2*L3*cmq3Mq1mq2)+m6*gz*(-cq2Mq3Mq4*sq5*L12+sq2Mq3Mq4*L5+L3*cq2Mq3);
    G(3,0)=m5*gx*(u1_2*cmq4mq3Mq1mq2+u1_2*cq4Mq3Mq1Mq2)*L11+m5*gy*(u1_2*sq4Mq3Mq1Mq2+u1_2*smq4mq3Mq1mq2)*L11+m5*gz*sq2Mq3Mq4*L11+m6*gx*(-(u1_2*smq4mq3Mq1mq2-u1_2*sq4Mq3Mq1Mq2)*sq5*L12+(u1_2*cmq4mq3Mq1mq2+u1_2*cq4Mq3Mq1Mq2)*L5)+m6*gy*(-(u1_2*cq4Mq3Mq1Mq2-u1_2*cmq4mq3Mq1mq2)*sq5*L12+(u1_2*sq4Mq3Mq1Mq2+u1_2*smq4mq3Mq1mq2)*L5)+m6*gz*(-cq2Mq3Mq4*sq5*L12+sq2Mq3Mq4*L5);
    G(4,0)=m6*gx*(-(u1_2*cmq4mq3Mq1mq2+u1_2*cq4Mq3Mq1Mq2)*cq5-sq1*sq5)*L12+m6*gy*(-(u1_2*sq4Mq3Mq1Mq2+u1_2*smq4mq3Mq1mq2)*cq5+cq1*sq5)*L12-m6*gz*sq2Mq3Mq4*cq5*L12;
    G(5,0)=0.0;

    return G;
}



}
}
