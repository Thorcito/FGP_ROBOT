#include<ur_robot_lli/RobotTools/MathTools.h>


namespace ur_robot_lli{
namespace RobotTools{


const double MathTools::u1_2 = 1.0/2.0;
const double MathTools::u1_4 = 1.0/4.0;
const double MathTools::u1_8 = 1.0/8.0;
const double MathTools::u1_16 = 1.0/16.0;
const double MathTools::u3_16 = 3.0/16.0;
const double MathTools::u1_32 = 1.0/32.0;
const double MathTools::u3_4 = 3.0/4.0;
const double MathTools::u3_8 = 3.0/8.0;

Vector3d MathTools::Z3()
{
    Vector3d z(0,0,0);
    return z;
}

Vector3d MathTools::Zz()
{
    Vector3d z(0,0,1);
    return z;
}

Vector4d MathTools::Z4()
{
    Vector4d z(0,0,0,0);
    return z;
}

Vector4d MathTools::Zh()
{
    Vector4d z(0,0,0,1);
    return z;
}

Matrix3d MathTools::Rx(double theta_rad)
{
    Matrix3d R;

    R<<1,0,0,0,cos(theta_rad),-sin(theta_rad),0,sin(theta_rad),cos(theta_rad);
    return R;
}

Matrix3d MathTools::Ry(double theta_rad)
{
    Matrix3d R;

    R<<cos(theta_rad),0, sin(theta_rad),0,1,0,-sin(theta_rad),0,cos(theta_rad);
    return R;
}

Matrix3d MathTools::Rz(double theta_rad)
{
    Matrix3d R;

    R<<cos(theta_rad),-sin(theta_rad),0,sin(theta_rad),cos(theta_rad),0,0,0,1;
    return R;
}

Matrix3d MathTools::Rxd(double theta_deg)
{
    Matrix3d R;

    double theta_rad=DEG2RAD(theta_deg);

    R<<1,0,0,0,cos(theta_rad),-sin(theta_rad),0,sin(theta_rad),cos(theta_rad);
    return R;
}

Matrix3d MathTools::Ryd(double theta_deg)
{
    Matrix3d R;

    double theta_rad=DEG2RAD(theta_deg);
    R<<cos(theta_rad),0, sin(theta_rad),0,1,0,-sin(theta_rad),0,cos(theta_rad);
    return R;
}

Matrix3d MathTools::Rzd(double theta_deg)
{
    Matrix3d R;

    double theta_rad=DEG2RAD(theta_deg);
    R<<cos(theta_rad),-sin(theta_rad),0,sin(theta_rad),cos(theta_rad),0,0,0,1;
    return R;
}


void MathTools::Vector2ADouble(Vector6d &in, VDouble &out)
{
    out.resize(6);
    for(unsigned int i=0;i<6;i++)
    {
        out[i]=in(i);
    }
}


Vector3d MathTools::GetEulerAngles(const Matrix3d &R, double sign, double phi_old)
{
    Vector3d eAngles;
    double aR33 = std::abs(R(2,2));
    double phi,theta,psi;

    if (aR33!=1.0)
    {
        double ct=R(2,2);
        double st=sign*sqrt(1.0-(R(2,2)*R(2,2)));

        theta=atan2(st,ct);
        phi=atan2(sign*R(1,2),sign*R(0,2));
        psi=atan2(sign*R(2,1),-sign*R(2,0));
    }
    else
    {
        theta=0;
        phi=phi_old;
        psi=atan2(R(1,0),R(0,0))-phi;
    }

    eAngles<<phi,theta,psi;

    return (eAngles);

    //    eAngles=[phi;theta;psi_];


}

void MathTools::ContinuousEuler(Vector3d &eulerAd, Vector3d &eulerAd_Old)
{
    Vector3d velEulerd=eulerAd-eulerAd_Old;

    for (unsigned int j=0;j<3;j++)
    {
        double jump=abs(velEulerd(j));
        if (jump>DEG2RAD(90))
        {
            if (eulerAd(j)>0.0)
            {
                eulerAd(j)=-2.0*M_PI+eulerAd(j);
            }
            else if (eulerAd(j)<0.0)
            {
                eulerAd(j)=2.0*M_PI+eulerAd(j);
            }
        }
    }
}

Matrix3d MathTools::Bmat(Vector3d eulerAngles)
{
    Matrix3d out;
    double phi,theta;

    phi=eulerAngles(0);
    theta=eulerAngles(1);

    out<<cos(phi)*sin(theta),-sin(phi),0,sin(phi)*sin(theta),cos(phi),0,cos(theta),0,1;
    return(out);
}

//aButter(0) should be = 0!!
void MathTools::FilterButter(Vector3d &x, Vector3d &xf, const Vector3d &bButter, const Vector3d &aButter)
{
    //Compute the filtered signal
    double r=bButter.transpose()*x;
    double rf=aButter.transpose()*xf;

    xf(0)=r-rf;

    //set the history
    x(2)=x(1);
    x(1)=x(0);

    xf(2)=xf(1);
    xf(1)=xf(0);
}



VVector6d MathTools::getJointPVT5(const Vector6d& start, const Vector6d& goal, double t_current, double t_total)
{

    //Saturate time variable to avoid using the pol function out of the desired time interval
    if(t_current>t_total)
    {
        t_current=t_total;
    }
    else if (t_current<0.0)
    {
        t_current=0.0;
    }

    double r = t_current/t_total;
    double r2 =r*r;
    double r3=r2*r;
    double r4=r3*r;
    double r5=r4*r;
    double a1 = 10*r3 - 15*r4 + 6*r5;
    double a2 = (30*r2 - 60*r3 + 30*r4)/(t_total);
    double a3 = (60*r - 180*r2 + 120*r3)/(t_total*t_total);
    Vector6d q_d, qp_d, qpp_d;
    VVector6d out;

    for (int idx = 0; idx < URDOF; idx++)
    {
        q_d(idx) = a1*(goal(idx) - start(idx)) + start(idx);
        qp_d(idx) = a2*(goal(idx) - start(idx));
        qpp_d(idx) = a3*(goal(idx) - start(idx));
    }

    //    ROS_ERROR_STREAM("q_d: "<<q_d.transpose());
    //    ROS_ERROR_STREAM("qp_d: "<<qp_d.transpose());
    //    ROS_ERROR_STREAM("qpp_d: "<<qpp_d.transpose());
    out.push_back(q_d);
    out.push_back(qp_d);
    out.push_back(qpp_d);

    return out;
}

Vector3d MathTools::getJointPVT5(const double start, const double goal, double t_current, double t_total)
{
    //Saturate time variable to avoid using the pol function out of the desired time interval
    if(t_current>t_total)
    {
        t_current=t_total;
    }
    else if (t_current<0.0)
    {
        t_current=0.0;
    }

    double r = t_current/t_total;
    double r2 =r*r;
    double r3=r2*r;
    double r4=r3*r;
    double r5=r4*r;
    double a1 = 10*r3 - 15*r4 + 6*r5;
    double a2 = (30*r2 - 60*r3 + 30*r4)/(t_total);
    double a3 = (60*r - 180*r2 + 120*r3)/(t_total*t_total);

    Vector3d q_d;


    q_d(0) = a1*(goal - start) + start;
    q_d(1) = a2*(goal - start);
    q_d(2) = a3*(goal - start);


    return q_d;
}

void MathTools::IntHeun(double x, double x_1, double ix_1, double &ix, double midh)
{
    ix = ix_1 + midh * (x + x_1);
}

MathTools::MathTools()
{

}

//MathTools::~MathTools()
//{

//}

}
}
