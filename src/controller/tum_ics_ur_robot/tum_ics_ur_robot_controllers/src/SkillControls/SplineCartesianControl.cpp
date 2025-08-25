#include<tum_ics_ur_robot_controllers/SkillControls/SplineCartesianControl.h>

#include<tum_ics_ur_robot_msgs/ControlData.h>

namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

SplineCartesianControl::SplineCartesianControl(Robot::KinematicModel *kModel,
                                               Robot::DynamicModel *dModel, ControlEffort *extCtrl,
                                               double weight,
                                               const QString& name):
    ControlEffort(name,SPLINE_TYPE,CARTESIAN_SPACE,weight),
    m_kinematicModel(kModel),
    m_dynamicModel(dModel),
    m_startFlag(false),
    m_Kp(MatrixDOFd::Zero()),
    m_Kd(MatrixDOFd::Zero()),
    m_stopKd(MatrixDOFd::Zero()),
    m_Ki(MatrixDOFd::Zero()),
    m_goal(VectorDOFd::Zero()),
    m_totalTime(100.0),
    m_minDetJ(100.0),
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
    m_q3LimitReached(false),
    m_setDefaultXef(true),
    m_extCtrl(extCtrl),
    m_useTauExtFlag(false),
    m_nExtTauThresh(100.0),
    m_startRecording(true)
{
    pubCtrlData=n.advertise<tum_ics_ur_robot_msgs::ControlData>("SplineCarCtrlData",100);

    //Get control period from dynamic model (this is obtained from config file and set for both the robot loop and dynMod loop)
    if(m_dynamicModel!=NULL)
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

SplineCartesianControl::~SplineCartesianControl()
{

}

void SplineCartesianControl::setQInit(const JointState& qinit)
{
    m_qInit=qinit;
}
void SplineCartesianControl::setQHome(const JointState& qhome)
{
    m_qHome=qhome;
}
void SplineCartesianControl::setQPark(const JointState& qpark)
{
    m_qPark=qpark;
}


bool SplineCartesianControl::init()
{
    std::string ns="~sosm_opctrl_spline";
    std::stringstream s;

    if (!ros::param::has(ns))
    {
        s<<"SplineCartesianControl init(): Control gains not defined --"<<ns<<"--, did you load them in the rosparam server??";
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
        s<<"SplineCartesianControl init(): Wrong number of d_gains --"<<p.size()<<"--";
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
        s<<"SplineCartesianControl init(): Wrong number of p_gains --"<<p.size()<<"--";
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
        s<<"SplineCartesianControl init(): Wrong number of i_gains --"<<p.size()<<"--";
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


    ////Damping gain STOP
    s.str("");
    s<<ns<<"/gains_d_stop";
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
        m_stopKd(i,i)=p[i];
    }

    ROS_WARN_STREAM("stopKd: \n"<<m_stopKd);


    s.str("");
    s<<ns<<"/ext_ntau_thresh";
    ros::param::get(s.str(),m_nExtTauThresh);
    ROS_WARN_STREAM("Norm Ext Tau Threshold: \n"<<m_nExtTauThresh);


    ////det(J) threshold
    s.str("");
    s<<ns<<"/min_detJ";
    ros::param::get(s.str(),m_minDetJ);

    ROS_WARN_STREAM("min_detJ: \n"<<m_minDetJ);



    ////min q3 limit
    s.str("");
    s<<ns<<"/q3_limit";
    ros::param::get(s.str(),m_q3_limit);

    ROS_WARN_STREAM("m_q3_limit [deg]: \n"<<m_q3_limit);
    m_q3_limit=DEG2RAD( m_q3_limit);
    ROS_WARN_STREAM("m_q3_limit [rad]: \n"<<m_q3_limit);


    ////Delta X Threshold
    s.str("");
    s<<ns<<"/delta_x";
    ros::param::get(s.str(),m_deltaX);

    ROS_WARN_STREAM("Threshold DX [m]: \n"<<m_deltaX);

    ////Delta A Threshold
    s.str("");
    s<<ns<<"/delta_a";
    ros::param::get(s.str(),m_deltaA);

    ROS_WARN_STREAM("Threshold DA [rad]: \n"<<m_deltaA);

    ////Delta Q Threshold
    s.str("");
    s<<ns<<"/delta_q";
    ros::param::get(s.str(),m_deltaQ);

    ROS_WARN_STREAM("Threshold DQ [rad]: \n"<<m_deltaQ);


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

    m_setDefaultXef=true;

    return true;
}
bool SplineCartesianControl::start()
{

    //For now the start, update and stop functions are states in update()
    m_runState=STATE_START;
    ROS_WARN_STREAM(getName().toStdString()<<"Start() Running state changed to: "<<m_runState);
    return true;
}
VectorDOFd SplineCartesianControl::update(const RobotTime& time, const JointState &current)
{

    VectorDOFd tau;
    double DTime;
    tum_ics_ur_robot_msgs::ControlData msg;

    tau.setZero();

    bool taskFailed=false;


    if(m_runState==STATE_INIT)
    {

        //reset the initialization flag (used in STATE_UPDATE to compute Bias)
        m_startFlag=false;


        if(m_setDefaultXef)
        {//Define the goal position as the initial position
            setGoalPose(m_kinematicModel->xef_w(),
                        m_kinematicModel->Ref_w());
            m_setDefaultXef=false;
        }

        //VectorDOFd G=m_dynamicModel->G();


        m_failed=false;
        m_initialized=true;
        m_started=false;
        m_running=false;
        m_finished=false;



        //we are waiting for the start function, then we just keep the robot stable
        m_mutex.lock();
        m_tau=tau;
        m_mutex.unlock();
        return weight()*tau;
    }

    if(m_runState==STATE_START)
    {
        //reset the initialization flag (used in STATE_UPDATE to compute Bias)
        m_startFlag=false;

        //Define the goal position as the initial position (FOR NOW we use the current Orientation)
        setGoalOrientation(m_kinematicModel->Ref_w());

        //Define the goal position wrt w
        m_xefGoal_w=xGoal();
        m_RefGoal_w=RGoal();
        m_totalTime=timeGoal();

        //Desired Pose wrt 0 link
        m_xefGoal_0=m_kinematicModel->Tw_0()*m_xefGoal_w.homogeneous();
        m_RefGoal_0=m_kinematicModel->Tw_0().rotation()*m_RefGoal_w;

        //For now nothing to do here, just change the state to UPDATE
        m_runState=STATE_UPDATE;
        ROS_WARN_STREAM(getName().toStdString()<<"STATE_START Running state changed to: "<<m_runState);

        m_failed=false;
        m_initialized=false;
        m_started=true;
        m_running=false;
        m_finished=false;


        //the pointer is defined at construction and the flag will be defined externally, e.g. SkillManager
        if((!(m_extCtrl==NULL))&&(m_tauExtFlag))
        {
            m_useTauExtFlag=true;

            m_updatedDTotalTime=false;
        }
        m_startRecording=true;


        //Since we don't do anything we just go to the next state
        //if we have an alternative control then we compute here
        // tau and enable the following line
        //return tau;
    }

    if(m_runState==STATE_UPDATE)
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

            //Define the goal position wrt w
            m_xefGoal_w=xGoal();
            m_RefGoal_w=RGoal();
            m_totalTime=timeGoal();

            //Desired Pose wrt 0 link
            m_xefGoal_0=m_kinematicModel->Tw_0()*m_xefGoal_w.homogeneous();
            m_RefGoal_0=m_kinematicModel->Tw_0().rotation()*m_RefGoal_w;

            ROS_WARN_STREAM("m_xefStart_w: "<<m_kinematicModel->xef_w().transpose());
            ROS_WARN_STREAM("m_xefGoal_w: "<<m_xefGoal_w.transpose());


            //Define the initial ef position error
            m_DeltaXef_0=m_xef_0-m_xefStart_0;
            m_DeltaXef_0_1=m_DeltaXef_0;
            m_iDeltaXef_0.setZero();
            m_iDeltaXef_0_1=m_iDeltaXef_0;

            //Define the initial Orientation
            m_RefStart_0=m_kinematicModel->Ref_0();
            m_Ref_0=m_kinematicModel->Ref_0();


            m_ikSign=1.0;
            //            m_q3_limit=DEG2RAD(15.0);

            m_eAngles_0=GetEulerAngles(m_Ref_0,m_ikSign,m_eAngles_0_1(0));
            //            m_eAnglesd_0=GetEulerAngles(m_RefGoal_0,m_ikSign,m_eAnglesd_0_1(0));
            m_eAnglesGoal_0=GetEulerAngles(m_RefGoal_0,m_ikSign,m_eAnglesd_0_1(0));

            m_eAngles_0_1=m_eAngles_0;
            m_eAnglesd_0_1=m_eAnglesd_0;

            m_aef_0<<m_eAngles_0(2),m_eAngles_0(1),m_eAngles_0(0);
            m_aefStart_0=m_aef_0;
            //            m_aefd_0<<m_eAnglesd_0(2),m_eAnglesd_0(1),m_eAnglesd_0(0);
            m_aefGoal_0<<m_eAnglesGoal_0(2),m_eAnglesGoal_0(1),m_eAnglesGoal_0(0);

            ROS_WARN_STREAM("m_aefStart_w: "<<m_aefStart_0.transpose());
            ROS_WARN_STREAM("m_aefGoal_w: "<<m_aefGoal_0.transpose());


            m_localTime=time.tD();
            m_DeltaTotalTime=m_totalTime;

            m_q3LimitReached=false;

            m_failed=false;
            m_initialized=false;
            m_started=false;
            m_running=true;
            m_finished=false;

            m_startFlag=true;
        }

        //To include Skin Tau
        if(m_useTauExtFlag)
        {
            Vector6d tauExt=m_extCtrl->getTau();

            double maxTau=0.0;
            for(int i=0;i<STD_DOF;i++)
            {
              double tabs=std::fabs(tauExt(i));
                if(tabs>maxTau)
                {
                    maxTau=tabs;
                }
            }

            if(maxTau>m_nExtTauThresh)
            {
                m_qStart=current.q;
                m_localTime=time.tD();

                m_DeltaQ.setZero();
                m_DeltaQ_1=m_DeltaQ;

                m_iDeltaQ.setZero();
                m_iDeltaQ_1=m_iDeltaQ;

                m_TanhSq_1.setZero();
                m_iTanhSq_1.setZero();


                //Re-set the initial ef starting position
                m_xefStart_0=m_kinematicModel->xef_0();



                //Re-set the initial ef position error
                m_DeltaXef_0.setZero();
                m_DeltaXef_0_1=m_DeltaXef_0;
                m_iDeltaXef_0.setZero();
                m_iDeltaXef_0_1=m_iDeltaXef_0;

                //Define the initial Orientation
                m_RefStart_0=m_kinematicModel->Ref_0();

                m_eAngles_0=GetEulerAngles(m_RefStart_0,m_ikSign,m_eAngles_0_1(0));
                m_eAngles_0_1=m_eAngles_0;

                m_aefStart_0<<m_eAngles_0(2),m_eAngles_0(1),m_eAngles_0(0);


                if(m_startRecording)
                {
                    m_eventTime_i=time.tD();
                    m_startRecording=false;
                    m_updatedDTotalTime=true;
                }

                m_elapsedTime=time.tD()-m_eventTime_i;
            }
            else
            {
                //reset the clock flag
                m_startRecording=true;

                if(m_updatedDTotalTime)
                {
                    m_DeltaTotalTime-=m_elapsedTime;

                    double maxAllowedTime=20.0;
                    double maxAllowedVel=1.0/maxAllowedTime;

                    double maxCVel=0.0;
                    double maxDx=0;



                    for(int i=0;i<3;i++)
                    {
                        double dx=abs(m_xefGoal_0(i)-m_xefStart_0(i));
                        double vx=dx/m_DeltaTotalTime;

                        if(vx>maxCVel)
                        {
                            maxCVel=vx;
                            maxDx=dx;
                        }
                    }
                    if(maxCVel>maxAllowedVel)
                    {
                        m_DeltaTotalTime=maxDx/maxAllowedVel;
                    }

                    if(m_DeltaTotalTime<0.05)
                    {
                        m_DeltaTotalTime=0.05;
                    }

                    m_updatedDTotalTime=false;
                }
            }
        }


        DTime=time.tD()-m_localTime;

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
            vxef_0d=getSpline5<VVector3d,Vector3d>(m_xefStart_0,m_xefGoal_0,DTime,m_totalTime);
            vaef_0d=getSpline5<VVector3d,Vector3d>(m_aefStart_0,m_aefGoal_0,DTime,m_totalTime);

//            vxef_0d=getSpline5<VVector3d,Vector3d>(m_xefStart_0,m_xefGoal_0,DTime,m_DeltaTotalTime);
//            vaef_0d=getSpline5<VVector3d,Vector3d>(m_aefStart_0,m_aefGoal_0,DTime,m_DeltaTotalTime);
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
        m_xefd_0=vxef_0d[0];
        m_xefpd_0=vxef_0d[1];
        m_xefppd_0=vxef_0d[2];

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
        Vector3d DeltaXef_0=m_xef_0-m_xefd_0;
        Vector3d DeltaXefp_0=m_xefp_0-m_xefpd_0;

        //Orientation Error
        Vector3d DeltaAlpha_0=m_aef_0-m_aefd_0;
        Vector3d DeltaAlphap_0=m_aefp_0-m_aefpd_0;

        Vector6d DeltaOp_r,DeltaOpp_r,Sq;
        Vector3d Xp_r_0,Alphap_r_0,Xpp_r_0,Alphapp_r_0;
        Matrix6d iJa_ef_0; //iJef_0,

        Xp_r_0=m_xefpd_0 - m_Kp.block(0,0,3,3)*DeltaXef_0;
        Alphap_r_0=m_aefpd_0 - m_Kp.block(3,3,3,3)*DeltaAlpha_0;

        Xpp_r_0=m_xefppd_0-m_Kp.block(0,0,3,3)*DeltaXefp_0;
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
            ROS_WARN_STREAM(getName().toStdString()<<" Reached Lower/Upper Limit q3: "<<
                            RAD2DEG(q2_n)<<" vs ("<<
                            RAD2DEG(q3_loLimit)<<", "<<
                            RAD2DEG(q3_upLimit)<<")");
        }
        else
        {
            m_q3LimitReached=false;
        }


        double detB=B.determinant();
        double nDetB=sqrt(detB*detB);

        if (nDetB<m_minDetJ)
        {
            m_runState=STATE_STOP;
            taskFailed=true;
            ROS_WARN_STREAM(getName().toStdString()<<"--------------SAFETY STOP B is reaching a singularity: "<<detB);
        }

        double detJef=m_kinematicModel->Jef_0().determinant();
        double nDetJef=sqrt(detJef*detJef);

        if (nDetJef<m_minDetJ)
        {
            m_runState=STATE_STOP;
            taskFailed=true;
            ROS_WARN_STREAM(getName().toStdString()<<"--------------SAFETY STOP Jef is reaching a singularity: "<<detJef);
        }

        if((q2_n<DEG2RAD(10.0))||(q2_n>DEG2RAD(170.0)))
        {
            m_runState=STATE_STOP;
            taskFailed=true;
            ROS_WARN_STREAM(getName().toStdString()<<" --------------SAFETY STOP Singularity: "<<RAD2DEG(current.q.transpose()));
        }


        MatrixRegressor Yr=m_dynamicModel->YrcNoG(current,js_r);
        VectorRegressor Th=m_dynamicModel->Th();
        MatrixDOFd Bv=m_dynamicModel->B();


        tau=-m_Kd*Sq  + Bv*current.qp + Yr*Th;

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


        VectorDOFd maxTau;
        maxTau<<180.0,180.0,180.0,180.0,180.0,180.0;

        satTau(tau,maxTau);


        //Broadcast Xefd_0
        tf::Transform transform;

        Quaterniond qed;

        qed=m_Refd_0;


        transform.setOrigin( tf::Vector3(m_xefd_0(0),m_xefd_0(1),m_xefd_0(2)) );

        transform.setRotation(tf::Quaternion(qed.x(),qed.y(),qed.z(),qed.w()));

        m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "r_arm_joint_0", "r_arm_joint_ef_d"));


        msg.header.stamp=time.tRc();

        msg.time=DTime;

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

            msg.torques[i]=weight()*tau(i);
        }

        for(int i=0;i<3;i++)
        {
            msg.Xef_0[i]=m_xef_0(i);
            msg.Xef_0[i+3]=m_aef_0(i);

            msg.Xd_0[i]=m_xefd_0(i);
            msg.Xd_0[i+3]=m_aefd_0(i);

            msg.DX[i]=DeltaXef_0(i);
            msg.DX[i+3]=DeltaAlpha_0(i);

            msg.DXp[i]=DeltaXefp_0(i);
            msg.DXp[i+3]=DeltaAlphap_0(i);
        }

        pubCtrlData.publish(msg);

        //Joint Position error Virtual vs Real
        VectorDOFd Dq;

        Dq=current.q-current.qReal;

        //TODO: add also the minimum error!
        //TODO: we add 3 sec to the convergence time to let the real robot catch the virtual up.

//        ROS_INFO_STREAM("Dq: "<<Dq.norm()<<"vs ("<<m_deltaQ<<")");

        if((DTime>=m_totalTime)&&
                (DeltaXef_0.norm()<m_deltaX)&&
                (DeltaAlpha_0.norm()<m_deltaA)&&
                (Dq.norm()<m_deltaQ))
        {
            //            ROS_WARN_STREAM(getName().toStdString()<<"total time reached and no vel");
            m_runState=STATE_STOP;
        }


        if(!m_startRecording)
        {
            tau.setZero();
        }
        m_mutex.lock();
        m_tau=tau;
        m_mutex.unlock();
        return tau;
    }
    if(m_runState==STATE_STOP)
    {

        //        VectorDOFd G=Gc(current);

        //Total Arm volocity
        double nQp=current.qp.norm();

        if(nQp<0.001)
        {
            //reset the initialization flag (used in STATE_UPDATE to compute Bias)

            m_startFlag=false;


            m_failed=taskFailed;
            m_initialized=false;
            m_started=false;
            m_running=false;
            m_finished=true;

            //            ROS_WARN_STREAM(getName().toStdString()<<" Finshed");

            //If the total velocity is close to zero just send G to keep the arm steady

            //This will disable the flag set by the skill manager
            disableTauExtFlag();

            m_mutex.lock();
            m_tau=tau;
            m_mutex.unlock();
            return tau;
        }

        //End-effector state
        m_xef_0=m_kinematicModel->xef_0();
        m_xefp_0=m_kinematicModel->xefp_0();

        m_Ref_0=m_kinematicModel->Ref_0();

        m_eAngles_0=GetEulerAngles(m_Ref_0,m_ikSign,m_eAngles_0_1(0));

        //avoid discontinuities
        ContinuousEuler(m_eAngles_0,m_eAngles_0_1);

        m_eAngles_0_1=m_eAngles_0;

        //We need to chenge the order of the euler angles
        //m_aefd_0<<m_eAnglesd_0(2),m_eAnglesd_0(1),m_eAnglesd_0(0);
        m_aef_0<<m_eAngles_0(2),m_eAngles_0(1),m_eAngles_0(0);

        m_DeltaQp=current.qp;

        tau=-m_stopKd*(m_DeltaQp);

        DTime=time.tD()-m_localTime;


        //Broadcast Xefd_0
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(m_xefd_0(0),m_xefpd_0(1),m_xefpd_0(2)) );
        Quaterniond qed;

        qed=m_Refd_0;

        transform.setRotation(tf::Quaternion(qed.x(),qed.y(),qed.z(),qed.w()));
        m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "r_arm_joint_0", "r_arm_joint_ef_d"));

        msg.header.stamp=time.tRc();

        msg.time=DTime;

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



            msg.torques[i]=weight()*tau(i);
        }

        for(int i=0;i<3;i++)
        {
            msg.Xef_0[i]=m_xef_0(i);
            msg.Xef_0[i+3]=m_aef_0(i);

            msg.Xd_0[i]=m_xefd_0(i);
            msg.Xd_0[i+3]=m_aefd_0(i);

            msg.DX[i]=0.0;
            msg.DX[i+3]=0.0;

            msg.DXp[i]=0.0;
            msg.DXp[i+3]=0.0;
        }

        pubCtrlData.publish(msg);

        m_mutex.lock();
        m_tau=tau;
        m_mutex.unlock();
        return weight()*tau;
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
    return weight()*tau;
}

bool SplineCartesianControl::stop()
{
    //For now the start, update and stop functions are states in update()
    m_runState=STATE_STOP;
    return true;
}



}
}
