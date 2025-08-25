#include<tum_ics_ur_robot_controllers/SkillControls/GControl.h>

#include<tum_ics_ur_robot_msgs/ControlData.h>

namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

GControl::GControl(Robot::DynamicModel *dModel, double weight,const QString& name):
    ControlEffort(name,STANDARD_TYPE,JOINT_SPACE,weight),
    m_dynamicModel(dModel)
{
    pubCtrlData=n.advertise<tum_ics_ur_robot_msgs::ControlData>("GCtrlData",100);

}

GControl::~GControl()
{
    qDebug("GControl class destroyed.");

}

void GControl::setQInit(const JointState& qinit)
{
    m_qInit=qinit;

}
void GControl::setQHome(const JointState& qhome)
{
    m_qHome=qhome;

}
void GControl::setQPark(const JointState& qpark)
{
    m_qPark=qpark;

}


bool GControl::init()
{


    if(m_dynamicModel==NULL)
    {
        ROS_ERROR_STREAM("GControl init(): You need to define the dynamic model for this control");
        return false;
    }



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
bool GControl::start()
{
    //For now the start, update and stop functions are states in update()
    m_runState=STATE_START;
    ROS_WARN_STREAM(getName().toStdString()<<" Start() Running state changed to: "<<m_runState);
    return true;
}
VectorDOFd GControl::update(const RobotTime& time, const JointState &current)
{
    VectorDOFd tau;

    tau.setZero();


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
        return Gc(current);
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

        tau=Gc(current);

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

        tum_ics_ur_robot_msgs::ControlData msg;

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

        pubCtrlData.publish(msg);

        m_mutex.lock();
        m_tau=tau;
        m_mutex.unlock();
        return tau;
    }
    if(m_runState==STATE_STOP)
    {
        //Return Zeros inmediatly

        m_failed=false;
        m_initialized=false;
        m_started=false;
        m_running=false;
        m_finished=true;

        tau=Gc(current);

        m_mutex.lock();
        m_tau=tau;
        m_mutex.unlock();
        return tau;
    }

    std::stringstream s;
    s<<"SplineJointControl update(): Unknown RunState: "<<m_runState;
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

bool GControl::stop()
{
    m_runState=STATE_STOP;
    return true;
}

VectorDOFd GControl::Gc(const JointState& js)
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
