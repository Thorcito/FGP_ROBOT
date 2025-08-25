#include<tum_ics_ur_robot_controllers/SkillControls/SplineJointControl.h>
#include<tum_ics_ur_robot_msgs/ControlData.h>

namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

SplineJointControl::SplineJointControl(Robot::KinematicModel *kModel,
                                       Robot::DynamicModel *dModel, ControlEffort *extCtrl,
                                       double weight,
                                       const QString& name):
    ControlEffort(name,SPLINE_TYPE,JOINT_SPACE,weight),
    m_kinematicModel(kModel),
    m_dynamicModel(dModel),
    m_startFlag(false),
    m_Kp(MatrixDOFd::Zero()),
    m_Kd(MatrixDOFd::Zero()),
    m_stopKd(MatrixDOFd::Zero()),
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
    m_extCtrl(extCtrl),
    m_useTauExtFlag(false),
    m_nExtTauThresh(100.0),
    m_startRecording(true)
{
    pubCtrlData=n.advertise<tum_ics_ur_robot_msgs::ControlData>("SplineJointCtrlData",100);

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

    m_torqueMax=0;



}

SplineJointControl::~SplineJointControl()
{

}

void SplineJointControl::setQInit(const JointState& qinit)
{
    m_qInit=qinit;
}
void SplineJointControl::setQHome(const JointState& qhome)
{
    m_qHome=qhome;
}
void SplineJointControl::setQPark(const JointState& qpark)
{
    m_qPark=qpark;
}


bool SplineJointControl::init()
{


    std::string ns="~sosm_ctrl_spline";
    std::stringstream s;

    if (!ros::param::has(ns))
    {
        s<<"SplineJointControl init(): Control gains not defined --"<<ns<<"--, did you load them in the rosparam server??";
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
        s<<"SplineJointControl init(): Wrong number of d_gains --"<<p.size()<<"--";
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
        s<<"SplineJointControl init(): Wrong number of p_gains --"<<p.size()<<"--";
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
        s<<"SplineJointControl init(): Wrong number of i_gains --"<<p.size()<<"--";
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



//    /////GOAL
//    s.str("");
//    s<<ns<<"/goal";
//    ros::param::get(s.str(),p);

//    if(p.size()<STD_DOF)
//    {
//        s.str("");
//        s<<"SplineJointControl init(): Wrong number of joint goals --"<<p.size()<<"--";
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


    s.str("");
    s<<ns<<"/ext_ntau_thresh";
    ros::param::get(s.str(),m_nExtTauThresh);
    ROS_WARN_STREAM("Norm Ext Tau Threshold: \n"<<m_nExtTauThresh);


    ////Delta Q Threshold
    s.str("");
    s<<ns<<"/delta_q";
    ros::param::get(s.str(),m_deltaQ);

    ROS_WARN_STREAM("Threshold DQ [rad]: \n"<<m_deltaQ);


    ////Delta Q Threshold
    s.str("");
    s<<ns<<"/delta_qreal";
    ros::param::get(s.str(),m_deltaQReal);

    ROS_WARN_STREAM("Threshold DQreal [rad]: \n"<<m_deltaQReal);



    //BIAS
    s.str("");
    s<<ns<<"/lambda";
    ros::param::get(s.str(),m_lambda);
    ROS_WARN_STREAM("Bias Lambda: \n"<<m_lambda);

    s.str("");
    s<<ns<<"/kappa";
    ros::param::get(s.str(),m_kappa);
    ROS_WARN_STREAM("Bias Kappa: \n"<<m_kappa);


    s.str("");
    s<<ns<<"/max_deltaq";
    ros::param::get(s.str(),m_maxDeltaQ);

    ROS_WARN_STREAM("Max Avg. Joint Error [deg]: \n"<<m_maxDeltaQ);
    m_maxDeltaQ=DEG2RAD(m_maxDeltaQ);
    ROS_WARN_STREAM("Max Avg. Joint Error [rad]: \n"<<m_maxDeltaQ);




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

    m_runState=STATE_INIT;

    return true;
}
bool SplineJointControl::start()
{
    //For now the start, update and stop functions are states in update()
    m_runState=STATE_START;
    ROS_WARN_STREAM(getName().toStdString()<<"Start() Running state changed to: "<<m_runState);
    return true;
}
VectorDOFd SplineJointControl::update(const RobotTime& time, const JointState &current)
{
    VectorDOFd tau;
    double DTime;
    tum_ics_ur_robot_msgs::ControlData msg;

    tau.setZero();

    if(m_runState==STATE_INIT)
    {
        //reset the initialization flag (used in STATE_UPDATE to compute Bias)
        m_startFlag=false;
//        m_startLimitClock=false;



        //VectorDOFd G=m_dynamicModel->G();


        m_failed=false;
        m_initialized=true;
        m_started=false;
        m_running=false;
        m_finished=false;

//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);

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
//        m_startLimitClock=false;

//        m_sLimitCicleTime=0.0;
//        m_dLimitCicleTime=0.0;


        //TODO: Check the initial value for safety!
        // load user goal to local goal
        m_goal = qGoal();
        m_totalTime = timeGoal();

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
//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);

        //Since we don't do anything we just go to the next state
        //if we have an alternative control then we compute here
        // tau and enable the following line
        //return tau;
    }
    if(m_runState==STATE_UPDATE)
    {
        //ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
        if(!m_startFlag)
        {
            m_qStart=current.q;
            m_startFlag=true;
//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
            m_DeltaQ=current.q-m_qStart;
            m_DeltaQ_1=m_DeltaQ;

            m_iDeltaQ.setZero();
            m_iDeltaQ_1=m_iDeltaQ;

            m_failed=false;
            m_initialized=false;
            m_started=false;
            m_running=true;
            m_finished=false;

            m_localTime=time.tD();

            m_DeltaTotalTime=m_totalTime;

        }

//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
        if(m_useTauExtFlag)
        {
            //ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
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
//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
            if(maxTau>m_nExtTauThresh)
            {
//              ROS_WARN_STREAM("maxTau: "<< maxTau<<" ("<<m_nExtTauThresh<<")");
                m_qStart=current.q;
                m_localTime=time.tD();

                m_DeltaQ.setZero();
                m_DeltaQ_1=m_DeltaQ;

                m_iDeltaQ.setZero();
                m_iDeltaQ_1=m_iDeltaQ;

                m_TanhSq_1.setZero();
                m_iTanhSq_1.setZero();

                //m_startExtTau=true;

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
                    double maxAllowedVel=M_PI/maxAllowedTime;

                    double maxCVel=0.0;
                    double maxDq=0;



                    for(int i=0;i<STD_DOF;i++)
                    {
                        double dQ=abs(m_goal(i)-m_qStart(i));
                        double vQ=dQ/m_DeltaTotalTime;

                        if(vQ>maxCVel)
                        {
                            maxCVel=vQ;
                            maxDq=dQ;
                        }
                    }
                    if(maxCVel>maxAllowedVel)
                    {
                        m_DeltaTotalTime=maxDq/maxAllowedVel;
                    }

                    if(m_DeltaTotalTime<3.0)
                    {
                        m_DeltaTotalTime=3.0;
                    }
//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
                    m_updatedDTotalTime=false;
                }
            }
        }
//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
        DTime=time.tD()-m_localTime;

        VVectorDOFd vQd=getSpline5<VVectorDOFd,VectorDOFd>(m_qStart,m_goal,DTime/*pausedTime*/,m_DeltaTotalTime/*m_totalTime*/);


//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
        //Joint position/velocity errors
        m_DeltaQ=current.q-vQd[0];
        m_DeltaQp=current.qp-vQd[1];

        for(int i=0;i<STD_DOF;i++)
        {
            IntHeun(m_DeltaQ(i),m_DeltaQ_1(i),m_iDeltaQ_1(i),m_iDeltaQ(i),m_controlPeriod_2);
        }
//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
        VectorDOFd S=m_DeltaQp+m_Kp*m_DeltaQ;

        if(m_initErrorS)
        {
            //initial error condition
            m_S_dt0=S;
            m_initErrorS=false;
        }
//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
        //    ROS_INFO_STREAM("time: "<<time.tD());

        VectorDOFd S_d=m_S_dt0*exp(-m_kappa*DTime);

        VectorDOFd Sp_d=-m_kappa*m_S_dt0*exp(-m_kappa*DTime);

        VectorDOFd Sq=S-S_d;

        VectorDOFd TanhSq,iTanhSq;
//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);

        for(unsigned int i=0;i<STD_DOF;i++)
        {
            TanhSq(i)=tanh(m_lambda*Sq(i));
            IntHeun(TanhSq(i),m_TanhSq_1(i),m_iTanhSq_1(i),iTanhSq(i),m_controlPeriod_2);

        }
//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
        JointState js_r;

        js_r.qp  = vQd[1] - m_Kp*m_DeltaQ  + S_d  - m_Ki*iTanhSq;
        js_r.qpp = vQd[2]- m_Kp*m_DeltaQp + Sp_d - m_Ki*TanhSq;

        VectorDOFd Sr=current.qp-js_r.qp;


        MatrixRegressor Yr=m_dynamicModel->YrcNoG(current,js_r);
        VectorRegressor Th=m_dynamicModel->Th();
        MatrixDOFd B=m_dynamicModel->B();

//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
        tau=-m_Kd*Sr  + B*current.qp + Yr*Th;

        //ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__<<" tau: "<<tau);

        for(int i=0;i<STD_DOF;i++)
        {
            if((isnan(tau(i)))||(isinf(tau(i))))
            {
                std::stringstream s;
                s<<getName().toStdString()<<" update(): The control output is nan/inf: "<<tau.transpose();
                m_error=true;
                m_errorString=s.str().c_str();
                ROS_ERROR_STREAM(m_errorString.toStdString());
                m_runState=STATE_STOP;


                //TODO add a m_failed flag!
                m_startFlag=false;
//                m_startLimitClock=false;

//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
                m_failed=true;
                m_initialized=false;
                m_started=false;
                m_running=true;
                m_finished=false;

                tau.setZero();
                m_mutex.lock();
                m_tau=tau;
                m_mutex.unlock();
            }
        }

//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
        //Saturation for tau (to avoid overshot with pertubations)

        VectorDOFd maxTau;
        maxTau<<180.0,180.0,180.0,180.0,180.0,180.0;

        satTau(tau,maxTau);
//        ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__<<" tau: "<<tau);


        //TODO add saturation function for tau!!


//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
        msg.header.stamp=time.tRc();

        msg.time=DTime;

        //TODO: add local_time to the msg

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

            msg.torques[i]=weight()*tau(i);
        }
//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
        pubCtrlData.publish(msg);


        //ROS_INFO_STREAM("Yth: "<<Yth.transpose());



        //Set history for the next loop
        m_DeltaQ_1=m_DeltaQ;
        m_iDeltaQ_1=m_iDeltaQ;

        m_TanhSq_1=TanhSq;
        m_iTanhSq_1=iTanhSq;

//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__<<"DTime: "<<DTime<<" m_DeltaTotalTime: "<<m_DeltaTotalTime);

        //Joint Position error Virtual vs Real
        VectorDOFd Dq;

        Dq=current.q-current.qReal;


        //TODO: we add 3 sec to the convergence time to let the real robot catch the virtual up.
        if((DTime>=m_DeltaTotalTime)&&
                (m_DeltaQ.norm()<m_deltaQ)&&
                (Dq.norm()<m_deltaQReal))
        {
            //            ROS_WARN_STREAM(getName().toStdString()<<"total time reached and no vel");
            m_runState=STATE_STOP;
        }
//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);

        if(!m_startRecording)
        {
            //ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
            tau.setZero();
        }
        m_mutex.lock();
        m_tau=tau;
        m_mutex.unlock();
//        ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__<<" tau: "<<tau);

        return tau;
    }
    if(m_runState==STATE_STOP)
    {
////ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
        //        VectorDOFd G=Gc(current);

        //Total Arm volocity
        double nQp=current.qp.norm();

        if(nQp<0.001)
        {
            //reset the initialization flag (used in STATE_UPDATE to compute Bias)

            m_startFlag=false;
//            m_startLimitClock=false;

            m_failed=false;
            m_initialized=false;
            m_started=false;
            m_running=false;
            m_finished=true;

            //            ROS_WARN_STREAM(getName().toStdString()<<" Finshed");

            //If the total velocity is close to zero just send G to keep the arm steady
////ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);

            m_mutex.lock();
            m_tau=tau;
            m_mutex.unlock();

            //This will disable the flag set by the skill manager
            disableTauExtFlag();

            return tau;
        }


        m_DeltaQp=current.qp;

        tau=-m_stopKd*(m_DeltaQp);

        DTime=time.tD()-m_localTime;

//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
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
//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
            msg.torques[i]=tau(i);
        }

        pubCtrlData.publish(msg);

        m_mutex.lock();
        m_tau=tau;
        m_mutex.unlock();
        return weight()*tau;
    }

//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
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
//ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
    m_mutex.lock();
    m_tau=tau;
    m_mutex.unlock();
    return weight()*tau;
}

bool SplineJointControl::stop()
{
    //For now the start, update and stop functions are states in update()
    m_runState=STATE_STOP;

    //ROS_INFO_STREAM(__FILE__<<" - "<<__LINE__);
    return true;
}



}
}
