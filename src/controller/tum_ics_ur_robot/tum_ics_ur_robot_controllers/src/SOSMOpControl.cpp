#include<tum_ics_ur_robot_controllers/SOSMOpControl.h>

#include<tum_ics_ur_robot_msgs/ControlData.h>

namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

SOSMOpControl::SOSMOpControl(Robot::KinematicModel *kModel,
                             Robot::DynamicModel *dModel,
                             double weight,
                             const QString& name):
    ControlEffort(name,SPLINE_TYPE,CARTESIAN_SPACE,weight),
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
    m_lambda(0.0),
    m_q3LimitReached(false)
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

SOSMOpControl::~SOSMOpControl()
{

}

void SOSMOpControl::setQInit(const JointState& qinit)
{
    m_qInit=qinit;
}
void SOSMOpControl::setQHome(const JointState& qhome)
{
    m_qHome=qhome;
}
void SOSMOpControl::setQPark(const JointState& qpark)
{
    m_qPark=qpark;
}


bool SOSMOpControl::init()
{
    std::string ns="~sosm_opctrl";
    std::stringstream s;

    if (!ros::param::has(ns))
    {
        s<<"SOSMOpControl init(): Control gains not defined --"<<ns<<"--, did you load them in the rosparam server??";
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
        s<<"SOSMOpControl init(): Wrong number of d_gains --"<<p.size()<<"--";
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
        s<<"SOSMOpControl init(): Wrong number of p_gains --"<<p.size()<<"--";
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
        s<<"SOSMOpControl init(): Wrong number of i_gains --"<<p.size()<<"--";
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

//    /////GOAL
//    s.str("");
//    s<<ns<<"/goal";
//    ros::param::get(s.str(),p);

//    if(p.size()<STD_DOF)
//    {
//        s.str("");
//        s<<"SOSMOpControl init(): Wrong number of joint goals --"<<p.size()<<"--";
//        m_error=true;
//        m_errorString=s.str().c_str();
//        return false;
//    }
//    for(int i=0;i<STD_DOF;i++)
//    {
//        m_goal(i)=p[i];
//    }
//    m_totalTime=p[STD_DOF];

//    if(!(m_totalTime>0))
//    {
//        m_totalTime=100.0;
//    }

//    ROS_WARN_STREAM("Goal [DEG]: \n"<<m_goal.transpose());
//    ROS_WARN_STREAM("Total Time [s]: "<<m_totalTime);


//    m_goal=DEG2RAD(m_goal);

//    ROS_WARN_STREAM("Goal [RAD]: \n"<<m_goal.transpose());


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
bool SOSMOpControl::start()
{
  return true;
}

VectorDOFd SOSMOpControl::update(const RobotTime& time, const JointState &current)
{
    if(!m_startFlag)
    {
        m_qStart=current.q;


        m_DeltaQ=current.q-m_qStart;
        m_DeltaQ_1=m_DeltaQ;

        m_iDeltaQ.setZero();
        m_iDeltaQ_1=m_iDeltaQ;

        //Define the initial ef starting position
        m_xefStart_0=m_kinematicModel->xef_0();
        m_xef_0=m_kinematicModel->xef_0();

        //Define the goal position
        m_xefGoal_0=m_xef_0;//xGoal();//<<m_xefStart_0(0),m_xefStart_0(1),m_xefStart_0(2)+0.1;
        m_Refd_0=RGoal();//m_kinematicModel->Ref_0();
        m_totalTime=timeGoal();//10.0;


        //Define the initial ef position error
        m_DeltaXef_0=m_xef_0-m_xefStart_0;
        m_DeltaXef_0_1=m_DeltaXef_0;
        m_iDeltaXef_0.setZero();
        m_iDeltaXef_0_1=m_iDeltaXef_0;

        //Define the initial Orientation
        m_RefStart_0=m_kinematicModel->Ref_0();
        m_Ref_0=m_kinematicModel->Ref_0();


        m_ikSign=1.0;
        m_q3_limit=DEG2RAD(15.0);

        m_eAngles_0=GetEulerAngles(m_Ref_0,m_ikSign,m_eAngles_0_1(0));
        m_eAnglesd_0=GetEulerAngles(m_Refd_0,m_ikSign,m_eAnglesd_0_1(0));
        m_eAnglesGoal_0=GetEulerAngles(m_Refd_0,m_ikSign,m_eAnglesd_0_1(0));

        m_eAngles_0_1=m_eAngles_0;
        m_eAnglesd_0_1=m_eAnglesd_0;

        m_aef_0<<m_eAngles_0(2),m_eAngles_0(1),m_eAngles_0(0);
        m_aefStart_0=m_aef_0;
        m_aefd_0<<m_eAnglesd_0(2),m_eAnglesd_0(1),m_eAnglesd_0(0);
        m_aefGoal_0<<m_eAnglesGoal_0(2),m_eAnglesGoal_0(1),m_eAnglesGoal_0(0);

        m_q3LimitReached=false;


        m_startFlag=true;
    }

    VectorDOFd tau;

    tau.setZero();

    //Spline Desired Position
    VVector3d vxef_0d;
    VVector3d vaef_0d;
    vxef_0d.resize(3);
    vaef_0d.resize(3);
    if(m_q3LimitReached)
    {
        //if the limit is reached then stop!!!
        vxef_0d[0]=m_xef_0;
        vxef_0d[1].setZero();
        vxef_0d[2].setZero();

        m_Ref_0=m_kinematicModel->Ref_0();
        m_eAngles_0=GetEulerAngles(m_Ref_0,m_ikSign,m_eAngles_0_1(0));
        m_aef_0<<m_eAngles_0(2),m_eAngles_0(1),m_eAngles_0(0);

        vaef_0d[0]=m_aef_0;
        vaef_0d[1].setZero();
        vaef_0d[2].setZero();
    }
    else
    {
        vxef_0d=getSpline5<VVector3d,Vector3d>(m_xefStart_0,m_xefGoal_0,time.tD(),m_totalTime);
        vaef_0d=getSpline5<VVector3d,Vector3d>(m_aefStart_0,m_aefGoal_0,time.tD(),m_totalTime);
    }

    //For active
//    setGoalPose(Xgoal);<--This function should be called somewhere else (e.g. skincontrol)
//    m_xefGoal = xGoal();


    //End-effector state
    m_xef_0=m_kinematicModel->xef_0();
    m_xefp_0=m_kinematicModel->xefp_0();

    m_Ref_0=m_kinematicModel->Ref_0();


    m_eAnglesd_0=GetEulerAngles(m_Refd_0,m_ikSign,m_eAnglesd_0_1(0));
    m_eAngles_0=GetEulerAngles(m_Ref_0,m_ikSign,m_eAngles_0_1(0));

    //avoid discontinuities
    ContinuousEuler(m_eAnglesd_0,m_eAnglesd_0_1);
    ContinuousEuler(m_eAngles_0,m_eAngles_0_1);

    m_eAngles_0_1=m_eAngles_0;
    m_eAnglesd_0_1=m_eAnglesd_0;


    //We need to chenge the order of the euler angles
    //m_aefd_0<<m_eAnglesd_0(2),m_eAnglesd_0(1),m_eAnglesd_0(0);
    m_aef_0<<m_eAngles_0(2),m_eAngles_0(1),m_eAngles_0(0);


    //In this case we use regulation control
    m_aefd_0=vaef_0d[0];
    m_aefpd_0=vaef_0d[1];
    m_aefppd_0=vaef_0d[2];

//    m_aefpd_0.setZero();
//    m_aefppd_0.setZero();

    Matrix3d B=Bmat(m_eAngles_0);
    Matrix3d iB=B.inverse();

    //we can compute this derivative numerically and avoid iB
    m_aefp_0=iB*m_kinematicModel->Jef_0().block(3,0,3,6)*current.qp;

    //Matrix Geometric Jacobian -> Analytic Jacobian
    Matrix6d Ma_g;
    Ma_g.setIdentity();
    Ma_g.block(3,3,3,3)=B;

    //Position Error
    Vector3d DeltaXef_0=m_xef_0-vxef_0d[0];
    Vector3d DeltaXefp_0=m_xefp_0-vxef_0d[1];

    //Orientation Error
    Vector3d DeltaAlpha_0=m_aef_0-m_aefd_0;
    Vector3d DeltaAlphap_0=m_aefp_0-m_aefpd_0;

    Vector6d DeltaOp_r,DeltaOpp_r,Sq;
    Vector3d Xp_r_0,Alphap_r_0,Xpp_r_0,Alphapp_r_0;
    Matrix6d iJa_ef_0; //iJef_0,

    Xp_r_0=vxef_0d[1] - m_Kp.block(0,0,3,3)*DeltaXef_0;
    Alphap_r_0=m_aefpd_0 - m_Kp.block(3,3,3,3)*DeltaAlpha_0;

    Xpp_r_0=vxef_0d[2]-m_Kp.block(0,0,3,3)*DeltaXefp_0;
    Alphapp_r_0=m_aefppd_0-m_Kp.block(3,3,3,3)*DeltaAlphap_0;

    DeltaOp_r<<Xp_r_0,Alphap_r_0;
    DeltaOpp_r<<Xpp_r_0,Alphapp_r_0;

    //Geometric Jacobian
//    iJef_0=this->Jef_0.inverse();

    //Analytic Jacobian
    iJa_ef_0=m_kinematicModel->Jef_0().inverse()*Ma_g;

    //TODO: Check the det(Ja) and stop before is singular!!


    JointState js_r;

    js_r.qp  = iJa_ef_0*DeltaOp_r;
    js_r.qpp = iJa_ef_0*DeltaOpp_r;

    Sq=current.qp-js_r.qp;

    double q3_upLimit, q3_loLimit;

    q3_upLimit=M_PI-m_q3_limit;
    q3_loLimit=m_q3_limit;

    double q2_n=sqrt(current.q(2)*current.q(2));

    if((q2_n<=q3_loLimit)||(q2_n>=q3_upLimit))
    {

       m_q3LimitReached=true;
        ROS_WARN_STREAM("Reached Lower/Upper Limit q3: "<<
                        RAD2DEG(q2_n)<<" vs ("<<
                        RAD2DEG(q3_loLimit)<<", "<<
                        RAD2DEG(q3_upLimit)<<")");
    }
    else
    {
      m_q3LimitReached=false;
    }

    if((q2_n<DEG2RAD(10.0))||(q2_n>DEG2RAD(170.0)))
    {
        m_runState=STATE_STOP;
        ROS_WARN_STREAM("--------------SAFETY STOP Singularity: "<<RAD2DEG(current.q.transpose()));
    }


    MatrixRegressor Yr=m_dynamicModel->YrcNoG(current,js_r);
    VectorRegressor Th=m_dynamicModel->Th();
    MatrixDOFd Bv=m_dynamicModel->B();


    tau=-m_Kd*Sq  + Bv*current.qp + Yr*Th;

    //Broadcast Xefd_0
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(vxef_0d[0](0),vxef_0d[0](1),vxef_0d[0](2)) );
    Quaterniond qed;

    qed=m_Refd_0;

    transform.setRotation(tf::Quaternion(qed.x(),qed.y(),qed.z(),qed.w()));
    m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "r_arm_joint_0", "r_arm_joint_ef_d"));




    m_mutex.lock();
    m_tau=tau;
    m_mutex.unlock();
    return weight()*tau;

}

bool SOSMOpControl::stop()
{
    return true;
}



}
}
