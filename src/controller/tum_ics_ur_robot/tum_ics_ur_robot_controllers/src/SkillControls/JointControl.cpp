#include<tum_ics_ur_robot_controllers/SkillControls/JointControl.h>
#include<tum_ics_ur_robot_msgs/ControlData.h>

namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

JointControl::JointControl(Robot::KinematicModel *kModel,
                           Robot::DynamicModel *dModel,
                           double weight,
                           const QString& name):
    ControlEffort(name,STANDARD_TYPE,JOINT_SPACE,weight),
    m_kinematicModel(kModel),
    m_dynamicModel(dModel),
    m_startFlag(false),
    m_startLimitClock(false),
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
    m_getQdFromTopic(false)
{
    m_pubCtrlData=m_n.advertise<tum_ics_ur_robot_msgs::ControlData>("JointCtrlData",100);

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

    ROS_INFO_STREAM(getName().toStdString()<<" Control Period: "<<m_controlPeriod<<" ("<<m_controlPeriod_2<<")");

    m_subsQd = m_n.subscribe("joint_desired_cmd",100,
                             &JointControl::getQdfromTopic,this,
                             ros::TransportHints().tcpNoDelay());


}

JointControl::~JointControl()
{
}

void JointControl::setQInit(const JointState& qinit)
{
    m_qInit=qinit;
}
void JointControl::setQHome(const JointState& qhome)
{
    m_qHome=qhome;
}
void JointControl::setQPark(const JointState& qpark)
{
    m_qPark=qpark;
}

void JointControl::enableQdTopic()
{
    m_getQdFromTopic=true;
}

void JointControl::disableQdTopic()
{
    m_getQdFromTopic=false;
}

const bool JointControl::isQdTopicEnabled() const
{
    return m_getQdFromTopic;
}


bool JointControl::init()
{


    std::string ns="~sosm_ctrl";
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

    /////P GAINS
    s.str("");
    s<<ns<<"/gains_p";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<getName().toStdString()<<" init(): Wrong number of p_gains --"<<p.size()<<"--";
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
        s<<getName().toStdString()<<" init(): Wrong number of i_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_Ki(i,i)=p[i]*p[i]/4.0;
        //        m_Ki(i,i)=m_Kp(i,i)*m_Kp(i,i)/4.0;
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



    /////GOAL
    s.str("");
    s<<ns<<"/goal";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<getName().toStdString()<<" init(): Wrong number of joint goals --"<<p.size()<<"--";
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


    s.str("");
    s<<ns<<"/max_deltaq";
    ros::param::get(s.str(),m_maxDeltaQ);

    ROS_WARN_STREAM("Max Avg. Joint Error [deg]: \n"<<m_maxDeltaQ);
    m_maxDeltaQ=DEG2RAD(m_maxDeltaQ);
    ROS_WARN_STREAM("Max Avg. Joint Error [rad]: \n"<<m_maxDeltaQ);




    Vector12d l=m_dynamicModel->L();

//    ROS_WARN_STREAM("L: "<<l.transpose());

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

//    ROS_WARN_STREAM("m: "<<ms.transpose());


    m1=ms(0);
    m2=ms(1);
    m3=ms(2);
    m4=ms(3);
    m5=ms(4);
    m6=ms(5);

    m_runState=STATE_INIT;

    return true;
}
bool JointControl::start()
{
    //For now the start, update and stop functions are states in update()
    m_runState=STATE_START;
    ROS_WARN_STREAM(getName().toStdString()<<"Start() Running state changed to: "<<m_runState);
    return true;
}
VectorDOFd JointControl::update(const RobotTime& time, const JointState &current)
{
    VectorDOFd tau;
    VectorDOFd zerosDof;
    tum_ics_ur_robot_msgs::ControlData msg;

    zerosDof.setZero();

    double DTime;

    tau.setZero();

    if(m_runState==STATE_INIT)
    {
        //reset the initialization flag (used in STATE_UPDATE to compute Bias)
        m_startFlag=false;
        m_startLimitClock=false;
        m_limitCicleTime=0.0;


        //VectorDOFd G=m_dynamicModel->G();


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
        m_startLimitClock=false;
        m_limitCicleTime=0.0;
        m_sLimitCicleTime=0.0;
        m_dLimitCicleTime=0.0;


        // load user goal to local goal

        setGoalJoints(current.q);

        m_totalTime = 0.0;//timeGoal();

        //For now nothing to do here, just change the state to UPDATE
        m_runState=STATE_UPDATE;
        ROS_WARN_STREAM(getName().toStdString()<<"STATE_START Running state changed to: "<<m_runState);

        m_initialized=false;
        m_started=true;
        m_running=false;
        m_finished=false;


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
            m_startFlag=true;

            m_DeltaQ=current.q-m_qStart;
            m_DeltaQ_1=m_DeltaQ;

            m_iDeltaQ.setZero();
            m_iDeltaQ_1=m_iDeltaQ;

            m_initialized=false;
            m_started=false;
            m_running=true;
            m_finished=false;

            m_localTime=time.tD();
            m_limitCicleTime_1=0.0;
            m_sLimitCicleTime=0.0;
            m_dLimitCicleTime=0.0;

        }




        DTime=time.tD()-m_localTime;


        //Joint position/velocity errors

        m_goal = qGoal();

        //        ROS_INFO_STREAM("qd: "<<m_goal.transpose());

        m_DeltaQ=current.q-m_goal;
        m_DeltaQp=current.qp;

        if(m_DeltaQ.norm()>m_maxDeltaQ)
        {
            if(!m_startLimitClock)
            {
                m_limitCicleTime_1=DTime;
                m_startLimitClock=true;
            }

            m_limitCicleTime=DTime-m_limitCicleTime_1;
            m_sLimitCicleTime=DTime-m_dLimitCicleTime;
        }
        else
        {
            m_dLimitCicleTime=m_limitCicleTime;
            m_limitCicleTime_1=m_sLimitCicleTime;
        }




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

        VectorDOFd S_d=m_S_dt0*exp(-m_kappa*DTime);

        VectorDOFd Sp_d=-m_kappa*m_S_dt0*exp(-m_kappa*DTime);

        VectorDOFd Sq=S-S_d;

        VectorDOFd TanhSq,iTanhSq;


        for(unsigned int i=0;i<STD_DOF;i++)
        {
            TanhSq(i)=tanh(m_lambda*Sq(i));
            IntHeun(TanhSq(i),m_TanhSq_1(i),m_iTanhSq_1(i),iTanhSq(i),m_controlPeriod_2);

        }

        JointState js_r;

        js_r.qp  =  - m_Kp*m_DeltaQ  + S_d  - m_Ki*iTanhSq;
        js_r.qpp =  - m_Kp*m_DeltaQp + Sp_d - m_Ki*TanhSq;

        VectorDOFd Sr=current.qp-js_r.qp;


        MatrixRegressor Yr=m_dynamicModel->YrcNoG(current,js_r);
        VectorRegressor Th=m_dynamicModel->Th();
        MatrixDOFd B=m_dynamicModel->B();


        tau=-m_Kd*Sr  + B*current.qp + Yr*Th;



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
                m_startLimitClock=false;
                m_limitCicleTime=0.0;

                m_failed=true;
                m_initialized=false;
                m_started=false;
                m_running=true;
                m_finished=false;

                tau.setZero();
            }
        }

        //TODO add saturation function for tau!!



        msg.header.stamp=time.tRc();

        msg.time=DTime;

        //TODO: add local_time to the msg

        for(int i=0;i<STD_DOF;i++)
        {
            msg.q[i]=current.q(i);
            msg.qp[i]=current.qp(i);
            msg.qpp[i]=current.qpp(i);

            msg.qd[i]=m_goal(i);
            msg.qpd[i]=0.0;
            msg.qppd[i]=0.0;

            msg.Dq[i]=m_DeltaQ(i);
            msg.Dqp[i]=m_DeltaQp(i);

            msg.torques[i]=tau(i);
        }

        m_pubCtrlData.publish(msg);
//        ROS_INFO_STREAM("Q: "<<current.q.transpose());


        //ROS_INFO_STREAM("Yth: "<<Yth.transpose());



        //Set history for the next loop
        m_DeltaQ_1=m_DeltaQ;
        m_iDeltaQ_1=m_iDeltaQ;

        m_TanhSq_1=TanhSq;
        m_iTanhSq_1=iTanhSq;


        m_mutex.lock();
        m_tau=tau;
        m_mutex.unlock();
        return weight()*tau;
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
            m_startLimitClock=false;


            m_initialized=false;
            m_started=false;
            m_running=false;
            m_finished=true;

            //If the total velocity is close to zero just send G to keep the arm steady


            m_mutex.lock();
            m_tau=tau;
            m_mutex.unlock();
            return weight()*tau;
        }



        m_DeltaQp=current.qp;

        tau=-m_stopKd*(m_DeltaQp);

        DTime=time.tD()-m_localTime;

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
        return weight()*tau;
    }


    std::stringstream s;
    s<<getName().toStdString()<<" update(): Unknown RunState: "<<m_runState;
    m_error=true;
    m_errorString=s.str().c_str();
    ROS_ERROR_STREAM(m_errorString.toStdString());

    for(int ii=0; ii < STD_DOF; ii++)
    tau(ii,0) = 0.5;

    m_mutex.lock();
    m_tau=tau;
    m_mutex.unlock();
    return weight()*tau;
}

bool JointControl::stop()
{
    //For now the start, update and stop functions are states in update()
    m_runState=STATE_STOP;
    return true;  
}

void JointControl::getQdfromTopic(const sensor_msgs::JointState::ConstPtr &msg)
{

    if(m_getQdFromTopic)
    {
        VectorDOFd qd;
        for(int i=0;i<STD_DOF;i++)
        {
            qd(i)=msg->position[i];
        }

        setGoalJoints(qd);
    }


    //    if(this->get_qd_from_topic)
    //    {
    //        //ROS_INFO_STREAM("Getting Topic /tom/joint_desired_cmd");
    //        unsigned int be;

    //        if(!this->joint_group_name.compare("right_arm"))
    //        {
    //            be=4;
    //            //be=0; //This is only for Erwin joint_state topic
    //        }
    //        else
    //        {
    ////            be=10;
    //            be=0;
    //        }


    //        for(unsigned int j=0;j<STD_DOF;j++)
    //        {
    //            q_d(j)=msg->position[j+be];
    //            //ROS_INFO_STREAM("be-> "<<be<<" q_d: "<<q_d.transpose());
    //        }

    //    }
}



}
}
