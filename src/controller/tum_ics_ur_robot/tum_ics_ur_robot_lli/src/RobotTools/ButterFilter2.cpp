/*****************************************************************************//**
    @file butter2_class.cpp

    Class for 2nd order butterworth filter Butter2

    August 2015, Jun Nakanishi
**********************************************************************************/
#include<ros/ros.h>
#include <ur_robot_lli/RobotTools/ButterFilter2.h>


namespace ur_robot_lli{
namespace RobotTools{
/*********************************************************************************
    myButter2 constructor
**********************************************************************************/
Butter2::Butter2():
    m_vx(Vector3d::Zero()),
    m_vy(Vector3d::Zero()),
    m_error(false),
    m_firstTime(true),
    m_vFirstTime(true),
    m_Xf(Vector6d::Zero())
{
    m_vX.resize(URDOF);
    m_vY.resize(URDOF);


    for(uint i=0;i<URDOF;i++)
    {
        m_vX[i].setZero();
        m_vY[i].setZero();
    }
}

Butter2::Butter2(double coff, double sampleT):
    m_cutoff(coff),
    m_T(sampleT),
    m_vx(Vector3d::Zero()),
    m_vy(Vector3d::Zero()),
    m_error(false),
    m_firstTime(true),
    m_vFirstTime(true),
    m_Xf(Vector6d::Zero())
{
    /// Constructor
    /// @param[in] coff cutoff frequency (Hz)
    /// @param[in] sampleT sampling time (sec)

    if(m_cutoff > 0.5/m_T)
    {
        m_error=true;
        std::stringstream s;
        s<<"Butter2(): cutoff frequency --"<<m_cutoff<<"-- must be smaller than half of sampling rate! ("<<m_T<<")";

        m_errorString=s.str().c_str();
        ROS_ERROR_STREAM(m_errorString.toStdString());
    }

    getCoefficients();


    m_vX.resize(URDOF);
    m_vY.resize(URDOF);


    for(uint i=0;i<URDOF;i++)
    {
        m_vX[i].setZero();
        m_vY[i].setZero();
    }
}

Butter2::~Butter2()
{

}

bool Butter2::error() const
{
    return m_error;
}
const QString& Butter2::errorString() const
{
    return m_errorString;
}

/*********************************************************************************
    get coefficients of 2nd order butterworth filter
**********************************************************************************/
void Butter2::getCoefficients(){
    /// Get coefficients of 2nd order butterworth filter
    /// b: numerator, a: denominator

    double wc_a = 2.0*M_PI*m_cutoff; // cutoff frequency (rad/s)
    double wc = 2.0/m_T * tan(wc_a * m_T/2.0); // prewarp frequency

    double T_2=m_T*m_T;
    double wc_2=wc*wc;
    double den = (T_2)*(wc_2)+sqrt(2.0)*m_T*wc*2.0+4.0;

    m_b(0) = (T_2)*(wc_2)/den;
    m_b(1) = (T_2)*(wc_2)*2.0/den;
    m_b(2) = (T_2)*(wc_2)/den;

    m_a(0) = 0.0; //in order to use xf(t)=b^T*x-a^T*xf(t-1)
    m_a(1) = ((T_2)*(wc_2)*2.0-8.0)/den;
    m_a(2) = ((T_2)*(wc_2)-sqrt(2.0)*m_T*wc*2.0+4.0)/den;
}

void Butter2::setFilterParams(double coff, double sampleT)
{
    m_cutoff=coff;
    m_T=sampleT;

    if(m_cutoff > 0.5/m_T)
    {
        m_error=true;
        std::stringstream s;
        s<<"Butter2(): cutoff frequency --"<<m_cutoff<<"-- must be smaller than half of sampling rate! ("<<m_T<<")";

        m_errorString=s.str().c_str();
        ROS_ERROR_STREAM(m_errorString.toStdString());
    }

    getCoefficients();
}

/*********************************************************************************
    filter
**********************************************************************************/
double Butter2::filter(double x)
{
    /// Butterworth filter implementation
    /// @param[in] x filter input (scalar)
    /// @param[out] y filter output (scalar)

    if(m_firstTime)
    { // avoid transient at the beginning
        // observe: b(0)+b(1)+b(2)-a(1)-a(2)=1
        // thus setting all internal variables to x makes initial output y=x
        // instead of setting all of them to zero

        m_vx<<x,x,x;
        m_vy<<x,x,x;

        m_firstTime = false;
    }
    else
    {
        m_vx(2) = m_vx(1);
        m_vx(1) = m_vx(0);
        m_vx(0) = x;

        m_vy(2) = m_vy(1);
        m_vy(1) = m_vy(0);
    }

    double r=m_b.transpose()*m_vx;
    double rf=m_a.transpose()*m_vy;

    m_vy(0)=r-rf;
    //m_vy(0) = m_b(0)*m_vx(0) + m_b(1)*vx(1) + m_b(2)*vx(2) - m_a(1)*m_vy(1) - m_a(2)*m_vy(2);


    return m_vy(0); // filter out
}

const Vector6d& Butter2::filter(const Vector6d& x)
{
    /// Butterworth filter implementation
    /// @param[in] x filter input (scalar)
    /// @param[out] y filter output (scalar)

    if(m_vFirstTime)
    { // avoid transient at the beginning
        // observe: b(0)+b(1)+b(2)-a(1)-a(2)=1
        // thus setting all internal variables to x makes initial output y=x
        // instead of setting all of them to zero
        for(uint i=0;i<URDOF;i++)
        {
            m_vX[i]<<x(i),x(i),x(i);
            m_vY[i]<<x(i),x(i),x(i);
        }

        m_vFirstTime = false;
    }
    else
    {
        for(uint i=0;i<URDOF;i++)
        {
            m_vX[i](2) = m_vX[i](1);
            m_vX[i](1) = m_vX[i](0);
            m_vX[i](0) = x(i);

            m_vY[i](2) = m_vY[i](1);
            m_vY[i](1) = m_vY[i](0);
        }
    }

    for(uint i=0;i<URDOF;i++)
    {
        double r=m_b.transpose()*m_vX[i];
        double rf=m_a.transpose()*m_vY[i];

        m_vY[i](0)=r-rf;
        m_Xf(i)=m_vY[i](0);
    }


    return m_Xf; // filter out
}

}//ns RobotTools
}//ns ur_robot_lli
