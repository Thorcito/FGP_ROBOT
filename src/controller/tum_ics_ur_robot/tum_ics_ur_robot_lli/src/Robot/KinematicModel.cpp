#include <tum_ics_ur_robot_lli/Robot/KinematicModel.h>

#include <QFileInfo>
#include <QSettings>

namespace tum_ics_ur_robot_lli
{
namespace Robot
{
KinematicModel::KinematicModel(const QString &configFilePath, const QString &groupName, const QString &robotDescription)
  : m_error(false)
  , m_l(Vector7d::Zero())
  , m_T0_w(Affine3d::Identity())
  , m_Tw_0(Affine3d::Identity())
  , m_Tef_0(Affine3d::Identity())
  , m_xef_0(Vector3d::Zero())
  , m_Ref_0(Matrix3d::Identity())
  , m_Jef_0(MatrixJd::Zero())
  , m_Xefp_0(Vector6d::Zero())
  , m_xefp_0(Vector3d::Zero())
  , m_wefp_0(Vector3d::Zero())
  , m_x_j_0(STD_DOF + 1)
  , m_y_j_0(STD_DOF + 1)
  , m_z_j_0(STD_DOF + 1)
  , m_t_j_0(STD_DOF + 1)
  , m_publishEfFlag(true)
  , m_groupName(groupName)
  , m_TIdentity(Affine3d::Identity())

{
  // We only accept left = "l"  or right = "r"
  if (m_groupName == "left")
  {
    m_namePrefix = "l";
  }
  else if (m_groupName == "right")
  {
    m_namePrefix = "r";
  }
  else
  {
    m_namePrefix = "dh";
  }

  ROS_INFO_STREAM("Joint Name Prefix: " << m_namePrefix.toStdString());

  parseConfig(configFilePath);

  m_Ti_0.resize(STD_DOF);
  m_Ti_0_pred.resize(STD_DOF);

  for (int i = 0; i < STD_DOF; i++)
  {
    m_Ti_0[i].setIdentity();
    m_Ti_0_pred[i].setIdentity();

    //        ROS_WARN_STREAM("m_T"<<i<<"_0: \n"<<m_Ti_0.at(i).matrix());
  }

  updateJointAxes_0();

  m_pubEFPose_0 = m_node.advertise<geometry_msgs::Pose>("Xef_0", 100);
  m_pubEFPose_w = m_node.advertise<geometry_msgs::Pose>("Xef_w", 100);
  m_pubEFPose_w_real = m_node.advertise<geometry_msgs::Pose>("Xef_w_real", 100);
}

KinematicModel::~KinematicModel()
{
}

void KinematicModel::setNamePrefix(const QString &namePrefix)
{
  m_namePrefix = namePrefix;
  ROS_INFO_STREAM("Joint Name Prefix: " << m_namePrefix.toStdString());
}

bool KinematicModel::error()
{
  return m_error;
}
const QString &KinematicModel::errorString()
{
  return m_errorString;
}

bool KinematicModel::update(const RobotTime &time, const JointState &current)
{
  //    here we calculate all the kinematic models FK DFK, TH, etc. etc

  m_q = current;

  // update all Ti_0s
  updateTi_0();
  updateTi_0_pred();

  // update all x_j_0, y_j_0, z_j_0, t_j_0
  updateJointAxes_0();

  // update End effector State updates Tef_0, xef_0, Ref_0, Jef_0, Xefp_0, etc
  updateEFState();

  return true;
}

void KinematicModel::setT0_W(const Affine3d &Twcf)
{
  m_T0_w = Twcf;
  m_Tw_0 = m_T0_w.inverse();
}

const Affine3d &KinematicModel::T0_w() const
{
  return m_T0_w;
}
const Affine3d &KinematicModel::Tw_0() const
{
  return m_Tw_0;
}

const Vector7d &KinematicModel::L() const
{
  return m_l;
}

const Affine3d &KinematicModel::Ti_0(uint jointID) const
{
  //    if((jointID>STD_DOF)||(jointID==0))
  //    {
  //        m_error=true;
  //        std::stringstream s;
  //        s<<"KinematicModel() Ti_0: The requested joint --"<<jointID<<"-- is not correct";
  //        m_errorString=s.str().c_str();
  //        ROS_ERROR_STREAM(m_errorString.toStdString());
  //    }
  return m_Ti_0[jointID - 1];
}

const Affine3d &KinematicModel::Ti_0_pred(uint jointID) const
{
  //    if((jointID>STD_DOF)||(jointID==0))
  //    {
  //        m_error=true;
  //        std::stringstream s;
  //        s<<"KinematicModel() Ti_0: The requested joint --"<<jointID<<"-- is not correct";
  //        m_errorString=s.str().c_str();
  //        ROS_ERROR_STREAM(m_errorString.toStdString());
  //    }
  return m_Ti_0_pred[jointID - 1];
}

Vector3d KinematicModel::Zi_0(uint jointID) const
{
  //    if(jointID>STD_DOF)
  //    {
  //        m_error=true;
  //        std::stringstream s;
  //        s<<"KinematicModel() Zi_0: The requested joint --"<<jointID<<"-- is not correct";
  //        m_errorString=s.str().c_str();
  //        ROS_ERROR_STREAM(m_errorString.toStdString());
  //    }
  if (jointID == 0)
    return Zz();
  else
    return m_Ti_0[jointID - 1].rotation().block(0, 2, 3, 1);
}
Vector3d KinematicModel::Pi_0(uint jointID) const
{
  //    if(jointID>STD_DOF)
  //    {
  //        m_error=true;
  //        std::stringstream s;
  //        s<<"KinematicModel() Pi_0: The requested joint --"<<jointID<<"-- is not correct";
  //        m_errorString=s.str().c_str();
  //        ROS_ERROR_STREAM(m_errorString.toStdString());
  //    }
  if (jointID == 0)
    return Z3();
  else
    return m_Ti_0[jointID - 1].translation();  // matrix().block(0,3,3,1);
}

MatrixJvd KinematicModel::JSingleL(Vector3d &pi_0)
{
  MatrixJvd J;

  J.setZero();
  J.block(0, 0, 3, 1) = Zi_0(0).cross(pi_0 - Pi_0(0));

  return J;
}

MatrixJd KinematicModel::JLi_0(Vector3d &pi_0, uint link)
{
  MatrixJd J;

  J.setZero();

  for (unsigned int i = 0; i < link; i++)
  {
    J.block(0, i, 3, 1) = Zi_0(i).cross(pi_0 - Pi_0(i));
    J.block(3, i, 3, 1) = Zi_0(i);
  }

  return J;
}

MatrixJd KinematicModel::JLi_0(Vector4d &pi_0, uint link)
{
  MatrixJd J;

  J.setZero();

  Vector3d aux = pi_0.block(0, 0, 3, 1);

  for (unsigned int i = 0; i < link; i++)
  {
    J.block(0, i, 3, 1) = Zi_0(i).cross(aux - Pi_0(i));
    J.block(3, i, 3, 1) = Zi_0(i);
  }

  return J;
}

Affine3d KinematicModel::Tef_0(const VectorDOFd &qc)
{
  Affine3d Tef;

  double q1 = qc(0);
  double q2 = qc(1);
  double q3 = qc(2);
  double q4 = qc(3);
  double q5 = qc(4);
  double q6 = qc(5);

  double cm4m3M1m2 = cos(-q4 - q3 + q1 - q2);
  double sm4m3M1m2 = sin(-q4 - q3 + q1 - q2);
  double cM4M3M1M2 = cos(q4 + q3 + q1 + q2);
  double sM4M3M1M2 = sin(q4 + q3 + q1 + q2);

  double s1s5 = sin(q1) * sin(q5);
  double s1c5 = sin(q1) * cos(q5);

  double c1c5 = cos(q1) * cos(q5);
  double c1s5 = cos(q1) * sin(q5);

  double c5c6 = cos(q5) * cos(q6);
  double c5s6 = cos(q5) * sin(q6);

  double cm3M1m2 = cos(-q3 + q1 - q2);
  double sm3M1m2 = sin(-q3 + q1 - q2);

  double cM3M1M2 = cos(q3 + q1 + q2);
  double sM3M1M2 = sin(q3 + q1 + q2);

  double cM1m2 = cos(q1 - q2);
  double sM1m2 = sin(q1 - q2);

  double c12 = cos(q1 + q2);
  double s12 = sin(q1 + q2);

  double c234 = cos(q2 + q3 + q4);
  double s234 = sin(q2 + q3 + q4);

  double s23 = sin(q2 + q3);

  double c5 = cos(q5);
  double s5 = sin(q5);

  double c6 = cos(q6);
  double s6 = sin(q6);

  double s2 = sin(q2);

  double c1 = cos(q1);
  double s1 = sin(q1);

  Tef.matrix() << ((u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * c5 + s1s5) * c6 + (u1_2 * sm4m3M1m2 - u1_2 * sM4M3M1M2) * s6,
      -((u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * c5 + s1s5) * s6 + (u1_2 * sm4m3M1m2 - u1_2 * sM4M3M1M2) * c6,
      -(u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * s5 + s1c5,
      (-(u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * s5 + s1c5) * L6 + (-u1_2 * sm4m3M1m2 + u1_2 * sM4M3M1M2) * L5 +
          s1 * L4 + u1_2 * L3 * cm3M1m2 + u1_2 * L3 * cM3M1M2 + u1_2 * L2 * cM1m2 + u1_2 * L2 * c12,
      ((u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * c5 - c1s5) * c6 + (u1_2 * cM4M3M1M2 - u1_2 * cm4m3M1m2) * s6,
      -((u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * c5 - c1s5) * s6 + (u1_2 * cM4M3M1M2 - u1_2 * cm4m3M1m2) * c6,
      -(u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * s5 - c1c5,
      (-(u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * s5 - c1c5) * L6 + (-u1_2 * cM4M3M1M2 + u1_2 * cm4m3M1m2) * L5 -
          c1 * L4 + u1_2 * L3 * sM3M1M2 + u1_2 * L3 * sm3M1m2 + u1_2 * L2 * s12 + u1_2 * L2 * sM1m2,
      s234 * c5c6 + c234 * s6, -s234 * c5s6 + c234 * c6, -s234 * s5,
      -s234 * s5 * L6 - c234 * L5 + L3 * s23 + s2 * L2 + L1, 0, 0, 0, 1;

  return Tef;
}

MatrixJd KinematicModel::Jef_0(const VectorDOFd &qc)
{
  MatrixJd Jef;

  double q1 = qc(0);
  double q2 = qc(1);
  double q3 = qc(2);
  double q4 = qc(3);
  double q5 = qc(4);

  double cm5M4M3M1M2 = cos(-q5 + q4 + q3 + q1 + q2);
  double sm5M4M3M1M2 = sin(-q5 + q4 + q3 + q1 + q2);

  double cM5M4M3M1M2 = cos(q5 + q4 + q3 + q1 + q2);
  double sM5M4M3M1M2 = sin(q5 + q4 + q3 + q1 + q2);

  double cM5m4m3M1m2 = cos(q5 - q4 - q3 + q1 - q2);
  double sM5m4m3M1m2 = sin(q5 - q4 - q3 + q1 - q2);

  double cm5m4m3M1m2 = cos(-q5 - q4 - q3 + q1 - q2);
  double sm5m4m3M1m2 = sin(-q5 - q4 - q3 + q1 - q2);

  double cM1m5 = cos(q1 - q5);
  double sM1m5 = sin(q1 - q5);

  double cM1M5 = cos(q1 + q5);
  double sM1M5 = sin(q1 + q5);

  double cM4M3M1M2 = cos(q4 + q3 + q1 + q2);
  double sM4M3M1M2 = sin(q4 + q3 + q1 + q2);

  double cm4m3M1m2 = cos(-q4 - q3 + q1 - q2);
  double sm4m3M1m2 = sin(-q4 - q3 + q1 - q2);

  double c1 = cos(q1);
  double s1 = sin(q1);

  double c2 = cos(q2);

  double cM3M1M2 = cos(q3 + q1 + q2);
  double sM3M1M2 = sin(q3 + q1 + q2);

  double cm3M1m2 = cos(-q3 + q1 - q2);
  double sm3M1m2 = sin(-q3 + q1 - q2);

  double cM2M3M4 = cos(q2 + q3 + q4);
  double sM2M3M4 = sin(q2 + q3 + q4);

  double cM2M3M4m5 = cos(q2 + q3 + q4 - q5);
  double sM2M3M4m5 = sin(q2 + q3 + q4 - q5);

  double cM2M3M4M5 = cos(q2 + q3 + q4 + q5);
  double sM2M3M4M5 = sin(q2 + q3 + q4 + q5);

  double c12 = cos(q1 + q2);
  double s12 = sin(q1 + q2);

  double cM1m2 = cos(q1 - q2);
  double sM1m2 = sin(q1 - q2);

  double c23 = cos(q2 + q3);

  Jef(0, 0) = u1_4 * L6 * cm5M4M3M1M2 - u1_4 * L6 * cM5M4M3M1M2 + u1_4 * L6 * cm5m4m3M1m2 - u1_4 * L6 * cM5m4m3M1m2 +
              u1_2 * L6 * cM1m5 + u1_2 * L6 * cM1M5 + u1_2 * L5 * cM4M3M1M2 - u1_2 * L5 * cm4m3M1m2 + c1 * L4 -
              u1_2 * L3 * sM3M1M2 - u1_2 * L3 * sm3M1m2 - u1_2 * L2 * s12 - u1_2 * L2 * sM1m2;
  Jef(0, 1) = u1_4 * L6 * cM5m4m3M1m2 + u1_4 * L6 * cm5M4M3M1M2 - u1_4 * L6 * cm5m4m3M1m2 - u1_4 * L6 * cM5M4M3M1M2 +
              u1_2 * L5 * cm4m3M1m2 + u1_2 * L5 * cM4M3M1M2 - u1_2 * L3 * sM3M1M2 + u1_2 * L3 * sm3M1m2 -
              u1_2 * L2 * s12 + u1_2 * L2 * sM1m2;
  Jef(0, 2) = u1_4 * L6 * cM5m4m3M1m2 + u1_4 * L6 * cm5M4M3M1M2 - u1_4 * L6 * cm5m4m3M1m2 - u1_4 * L6 * cM5M4M3M1M2 +
              u1_2 * L5 * cm4m3M1m2 + u1_2 * L5 * cM4M3M1M2 - u1_2 * L3 * sM3M1M2 + u1_2 * L3 * sm3M1m2;
  Jef(0, 3) = u1_4 * L6 * cM5m4m3M1m2 + u1_4 * L6 * cm5M4M3M1M2 - u1_4 * L6 * cm5m4m3M1m2 - u1_4 * L6 * cM5M4M3M1M2 +
              u1_2 * L5 * cm4m3M1m2 + u1_2 * L5 * cM4M3M1M2;
  Jef(0, 4) = u1_2 * L6 * cM1M5 - u1_2 * L6 * cM1m5 - u1_4 * L6 * cm5m4m3M1m2 - u1_4 * L6 * cm5M4M3M1M2 -
              u1_4 * L6 * cM5m4m3M1m2 - u1_4 * L6 * cM5M4M3M1M2;
  Jef(0, 5) = 0;

  Jef(1, 0) = -u1_4 * L6 * sM5m4m3M1m2 + u1_4 * L6 * sm5m4m3M1m2 - u1_4 * L6 * sM5M4M3M1M2 + u1_4 * L6 * sm5M4M3M1M2 +
              u1_2 * L6 * sM1M5 + u1_2 * L6 * sM1m5 - u1_2 * L5 * sm4m3M1m2 + u1_2 * L5 * sM4M3M1M2 + s1 * L4 +
              u1_2 * L3 * cm3M1m2 + u1_2 * L3 * cM3M1M2 + u1_2 * L2 * cM1m2 + u1_2 * L2 * c12;
  Jef(1, 1) = u1_4 * L6 * sm5M4M3M1M2 + u1_4 * L6 * sM5m4m3M1m2 - u1_4 * L6 * sM5M4M3M1M2 - u1_4 * L6 * sm5m4m3M1m2 +
              u1_2 * L5 * sM4M3M1M2 + u1_2 * L5 * sm4m3M1m2 - u1_2 * L3 * cm3M1m2 + u1_2 * L3 * cM3M1M2 -
              u1_2 * L2 * cM1m2 + u1_2 * L2 * c12;
  Jef(1, 2) = u1_4 * L6 * sm5M4M3M1M2 + u1_4 * L6 * sM5m4m3M1m2 - u1_4 * L6 * sM5M4M3M1M2 - u1_4 * L6 * sm5m4m3M1m2 +
              u1_2 * L5 * sM4M3M1M2 + u1_2 * L5 * sm4m3M1m2 - u1_2 * L3 * cm3M1m2 + u1_2 * L3 * cM3M1M2;
  Jef(1, 3) = u1_4 * L6 * sm5M4M3M1M2 + u1_4 * L6 * sM5m4m3M1m2 - u1_4 * L6 * sM5M4M3M1M2 - u1_4 * L6 * sm5m4m3M1m2 +
              u1_2 * L5 * sM4M3M1M2 + u1_2 * L5 * sm4m3M1m2;
  Jef(1, 4) = u1_2 * L6 * sM1M5 - u1_2 * L6 * sM1m5 - u1_4 * L6 * sM5M4M3M1M2 - u1_4 * L6 * sM5m4m3M1m2 -
              u1_4 * L6 * sm5M4M3M1M2 - u1_4 * L6 * sm5m4m3M1m2;
  Jef(1, 5) = 0;

  Jef(2, 0) = 0;
  Jef(2, 1) = L5 * sM2M3M4 + L3 * c23 + u1_2 * L6 * sM2M3M4m5 - u1_2 * L6 * sM2M3M4M5 + c2 * L2;
  Jef(2, 2) = L5 * sM2M3M4 + L3 * c23 + u1_2 * L6 * sM2M3M4m5 - u1_2 * L6 * sM2M3M4M5;
  Jef(2, 3) = u1_2 * L6 * sM2M3M4m5 - u1_2 * L6 * sM2M3M4M5 + L5 * sM2M3M4;
  Jef(2, 4) = -u1_2 * L6 * sM2M3M4m5 - u1_2 * L6 * sM2M3M4M5;
  Jef(2, 5) = 0;

  Jef(3, 0) = 0;
  Jef(3, 1) = s1;
  Jef(3, 2) = s1;
  Jef(3, 3) = s1;
  Jef(3, 4) = -u1_2 * sm4m3M1m2 + u1_2 * sM4M3M1M2;
  Jef(3, 5) =
      -u1_4 * sM5m4m3M1m2 + u1_4 * sm5m4m3M1m2 - u1_4 * sM5M4M3M1M2 + u1_4 * sm5M4M3M1M2 + u1_2 * sM1M5 + u1_2 * sM1m5;

  Jef(4, 0) = 0;
  Jef(4, 1) = -c1;
  Jef(4, 2) = -c1;
  Jef(4, 3) = -c1;
  Jef(4, 4) = -u1_2 * cM4M3M1M2 + u1_2 * cm4m3M1m2;
  Jef(4, 5) =
      -u1_4 * cm5M4M3M1M2 + u1_4 * cM5M4M3M1M2 - u1_4 * cm5m4m3M1m2 + u1_4 * cM5m4m3M1m2 - u1_2 * cM1m5 - u1_2 * cM1M5;

  Jef(5, 0) = 1;
  Jef(5, 1) = 0;
  Jef(5, 2) = 0;
  Jef(5, 3) = 0;
  Jef(5, 4) = -cM2M3M4;
  Jef(5, 5) = -u1_2 * cM2M3M4m5 + u1_2 * cM2M3M4M5;

  return Jef;
}

const Affine3d &KinematicModel::Tef_0() const
{
  return m_Tef_0;
}
const Vector3d &KinematicModel::xef_0() const
{
  return m_xef_0;
}
const Matrix3d &KinematicModel::Ref_0() const
{
  return m_Ref_0;
}

const Affine3d &KinematicModel::Tef_w() const
{
  return m_Tef_w;
}
const Vector3d &KinematicModel::xef_w() const
{
  return m_xef_w;
}
const Matrix3d &KinematicModel::Ref_w() const
{
  return m_Ref_w;
}

const Affine3d &KinematicModel::FK() const
{
  return m_Ti_0[5];
}
const MatrixJd &KinematicModel::Jef_0() const
{
  return m_Jef_0;
}
const Vector6d &KinematicModel::Xefp_0() const
{
  return m_Xefp_0;
}
const Vector3d &KinematicModel::xefp_0() const
{
  return m_xefp_0;
}
const Vector3d &KinematicModel::wefp_0() const
{
  return m_wefp_0;
}

const Vector3d &KinematicModel::xefp_w() const
{
  return m_xefp_w;
}

const Eigen::Affine3d &KinematicModel::T_j_0(int j) const
{
  if (j == 0)
  {
    //        return Eigen::Affine3d::Identity();
    return m_TIdentity;
  }
  return m_Ti_0.at(j - 1);
}

const Eigen::Vector3d &KinematicModel::x_j_0(int j) const
{
  return m_x_j_0.at(j);
}

const Eigen::Vector3d &KinematicModel::y_j_0(int j) const
{
  return m_y_j_0.at(j);
}

const Eigen::Vector3d &KinematicModel::z_j_0(int j) const
{
  return m_z_j_0.at(j);
}

const Eigen::Vector3d &KinematicModel::t_j_0(int j) const
{
  return m_t_j_0.at(j);
}

void KinematicModel::publish_FK()
{
  // broadcast the DH tfs to visualize them in RVIZ

  geometry_msgs::Pose msg;
  Quaterniond q_r;

  if (m_publishEfFlag)
  {
    q_r = m_Ref_0;

    msg.position.x = m_xef_0(0);
    msg.position.y = m_xef_0(1);
    msg.position.z = m_xef_0(2);

    msg.orientation.x = q_r.x();
    msg.orientation.y = q_r.y();
    msg.orientation.z = q_r.z();
    msg.orientation.w = q_r.w();

    m_pubEFPose_0.publish(msg);

    q_r = m_Ref_w;

    msg.position.x = m_xef_w(0);
    msg.position.y = m_xef_w(1);
    msg.position.z = m_xef_w(2);

    msg.orientation.x = q_r.x();
    msg.orientation.y = q_r.y();
    msg.orientation.z = q_r.z();
    msg.orientation.w = q_r.w();

    m_pubEFPose_w.publish(msg);
  }

  tf::Transform transform;

  std::stringstream index_frame_name;
  Vector3d t0_0;
  Matrix3d R0_0;
  Quaterniond qe;

  std::stringstream n0;
  std::string root_name = "_arm_joint_";

  //    std::string name_prefix="r";

  //    n0<<"/"<<m_namePrefix.toStdString()<<"_base_link";
  n0 << "/" << m_baseName.toStdString();

  // Rotation from Rviz to Dh_0 (it is the same as Dh_0 wrt Rviz). Obtained from inspection!!
  R0_0 << -1, 0, 0, 0, -1, 0, 0, 0, 1;
  qe = R0_0;
  t0_0 << 0.0, 0.0, 0.0;
  transform.setRotation(tf::Quaternion(qe.x(), qe.y(), qe.z(), qe.w()));
  transform.setOrigin(tf::Vector3(t0_0(0), t0_0(1), t0_0(2)));

  index_frame_name << "/" << m_namePrefix.toStdString() << root_name << 0;

  m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), n0.str(), index_frame_name.str()));
  index_frame_name.str("");
  n0.str("");

  n0 << "/" << m_namePrefix.toStdString() << root_name << 0;
  for (unsigned int i = 0; i < STD_DOF; i++)
  {
    R0_0 = Ti_0(i + 1).rotation();  // block(0,0,3,3);
    qe = R0_0;
    t0_0 = Ti_0(i + 1).translation();  // block(0,3,3,1);

    transform.setRotation(tf::Quaternion(qe.x(), qe.y(), qe.z(), qe.w()));
    transform.setOrigin(tf::Vector3(t0_0(0), t0_0(1), t0_0(2)));

    index_frame_name << "/" << m_namePrefix.toStdString() << root_name << i + 1;
    m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), n0.str(), index_frame_name.str()));
    index_frame_name.str("");
  }

  // enf-effector
  // R0_0=Ref_0();//block(0,0,3,3);
  qe = Ref_0();
  t0_0 = xef_0();  // block(0,3,3,1);

  transform.setRotation(tf::Quaternion(qe.x(), qe.y(), qe.z(), qe.w()));
  transform.setOrigin(tf::Vector3(t0_0(0), t0_0(1), t0_0(2)));
  index_frame_name << "/" << m_namePrefix.toStdString() << root_name << "ef";
  m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), n0.str(), index_frame_name.str()));
  index_frame_name.str("");

  n0.str("");
}

void KinematicModel::publish_FK(const VectorDOFd &qc, const std::string &posfix)
{
  // broadcast the DH tfs to visualize them in RVIZ

  tf::Transform transform;
  std::stringstream index_frame_name;
  Vector3d t0_0;
  Matrix3d R0_0;
  Quaterniond qe;

  std::stringstream n0;
  std::string root_name = "_arm_joint_";

  //    std::string name_prefix="r";

  //    n0<<"/"<<m_namePrefix.toStdString()<<"_base_link";

  n0 << "/" << m_baseName.toStdString();

  // Rotation from Rviz to Dh_0 (it is the same as Dh_0 wrt Rviz). Obtained from inspection!!
  R0_0 << -1, 0, 0, 0, -1, 0, 0, 0, 1;
  qe = R0_0;
  t0_0 << 0.0, 0.0, 0.0;
  transform.setRotation(tf::Quaternion(qe.x(), qe.y(), qe.z(), qe.w()));
  transform.setOrigin(tf::Vector3(t0_0(0), t0_0(1), t0_0(2)));

  index_frame_name << "/" << m_namePrefix.toStdString() << root_name << 0 << posfix;

  m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), n0.str(), index_frame_name.str()));
  index_frame_name.str("");
  n0.str("");

  n0 << "/" << m_namePrefix.toStdString() << root_name << 0;

  VAffine3d Ti_0_s;

  Ti_0_s.resize(STD_DOF);

  updateTi_0(Ti_0_s, qc);

  for (unsigned int i = 0; i < STD_DOF; i++)
  {
    R0_0 = Ti_0_s.at(i).rotation();  // block(0,0,3,3);
    qe = R0_0;
    t0_0 = Ti_0_s.at(i).translation();  // block(0,3,3,1);

    transform.setRotation(tf::Quaternion(qe.x(), qe.y(), qe.z(), qe.w()));
    transform.setOrigin(tf::Vector3(t0_0(0), t0_0(1), t0_0(2)));

    index_frame_name << "/" << m_namePrefix.toStdString() << root_name << i + 1 << posfix;
    m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), n0.str(), index_frame_name.str()));
    index_frame_name.str("");
  }

  // enf-effector

  qe = Ti_0_s.at(5).rotation();
  t0_0 = Ti_0_s.at(5).translation();  // xef_0();//block(0,3,3,1);

  transform.setRotation(tf::Quaternion(qe.x(), qe.y(), qe.z(), qe.w()));
  transform.setOrigin(tf::Vector3(t0_0(0), t0_0(1), t0_0(2)));
  index_frame_name << "/" << m_namePrefix.toStdString() << root_name << "ef" << posfix;
  m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), n0.str(), index_frame_name.str()));
  index_frame_name.str("");

  n0.str("");

  // End effector wrt world

  Affine3d Tef_w = m_T0_w * Ti_0_s[5];

  Vector3d tef_w = Tef_w.translation();

  Quaterniond qef_w;

  qef_w = Tef_w.rotation();

  geometry_msgs::Pose msg;

  msg.position.x = tef_w(0);
  msg.position.y = tef_w(1);
  msg.position.z = tef_w(2);

  msg.orientation.x = qef_w.x();
  msg.orientation.y = qef_w.y();
  msg.orientation.z = qef_w.z();
  msg.orientation.w = qef_w.w();

  //    Vector3d squeeze;

  //    squeeze<<0.54267,0.01390,-0.4664;

  //    double dx=squeeze(0)-msg.position.x;
  //    double dy=squeeze(1)-msg.position.y;

  //    double diff=sqrt(dx*dx+dy*dy);

  //    if(diff>0.50)
  //    {
  //        ROS_ERROR_STREAM("EF real: ("<<msg.position.x<<", "<<msg.position.y<<", "<<msg.position.z<<" vs
  //        sq:"<<squeeze.transpose());
  //    }

  m_pubEFPose_w_real.publish(msg);
}

bool KinematicModel::parseConfig(const QString &configFilePath)
{
  QFileInfo config(configFilePath);
  if (!config.exists())
  {
    m_error = true;
    m_errorString = "KinematicModel(): Error reading the config file. File: " + configFilePath + " does not exist!!!";
    ROS_ERROR_STREAM(m_errorString.toStdString());

    return false;
  }

  QSettings iniFile(configFilePath, QSettings::IniFormat);

  iniFile.beginGroup("ROBOT_KINEMATIC_PARAMETERS");

  std::stringstream s;

  for (int i = 0; i < 6; i++)
  {
    s << "L" << i + 1;
    m_l[i] = iniFile.value(s.str().c_str(), 0.110).toDouble();

    ROS_INFO_STREAM(s.str() << ": " << m_l[i]);
    s.str("");
  }
  m_l[6] = iniFile.value("L_EEF", 0.110).toDouble();

  ROS_INFO_STREAM("L_EEF: " << m_l[6]);

  Matrix3d R0_w;
  Vector3d t0_w;

  double wx, wy, wz;
  wx = DEG2RAD(iniFile.value("RX0_W", 0.0).toDouble());
  wy = DEG2RAD(iniFile.value("RY0_W", 0.0).toDouble());
  wz = DEG2RAD(iniFile.value("RZ0_W", 0.0).toDouble());

  t0_w(0) = iniFile.value("TX0_W", 0.0).toDouble();
  t0_w(1) = iniFile.value("TY0_W", 0.0).toDouble();
  t0_w(2) = iniFile.value("TZ0_W", 0.0).toDouble();

  m_baseName = iniFile.value("BASE_NAME", "base_link").toString();

  ROS_INFO_STREAM("BASE_NAME: " << m_baseName.toStdString());

  R0_w = Rz(wz) * Ry(wy) * Rx(wx);

  // sets T_0_w and computes T_w_0
  Affine3d T0_w = Translation3d(t0_w) * AngleAxisd(R0_w);
  setT0_W(T0_w);

  ROS_INFO_STREAM("T0_w: \n" << T0_w.matrix());

  iniFile.endGroup();

  // Initialize constant parameters (depending on user defined params)
  initParams();

  return true;
}

void KinematicModel::initParams()
{
  L1 = m_l(0);
  L2 = m_l(1);
  L3 = m_l(2);
  L4 = m_l(3);
  L5 = m_l(4);
  // L6 is the original offset m_l(5) + the tool offset m_l(6) a.k.a gripper
  L6 = m_l(5) + m_l(6);

  L2_2 = L2 * L2;
  L3_2 = L3 * L3;
  L4_2 = L4 * L4;
  L5_2 = L5 * L5;
}

void KinematicModel::updateTi_0(VAffine3d &inT, const VectorDOFd &qc)
{
  double q1 = qc(0);
  double q2 = qc(1);
  double q3 = qc(2);
  double q4 = qc(3);
  double q5 = qc(4);
  double q6 = qc(5);

  double cm4m3M1m2 = cos(-q4 - q3 + q1 - q2);
  double sm4m3M1m2 = sin(-q4 - q3 + q1 - q2);
  double cM4M3M1M2 = cos(q4 + q3 + q1 + q2);
  double sM4M3M1M2 = sin(q4 + q3 + q1 + q2);

  double s1s5 = sin(q1) * sin(q5);
  double s1c5 = sin(q1) * cos(q5);

  double c1c5 = cos(q1) * cos(q5);
  double c1s5 = cos(q1) * sin(q5);

  double c5c6 = cos(q5) * cos(q6);
  double c5s6 = cos(q5) * sin(q6);

  double cm3M1m2 = cos(-q3 + q1 - q2);
  double sm3M1m2 = sin(-q3 + q1 - q2);

  double cM3M1M2 = cos(q3 + q1 + q2);
  double sM3M1M2 = sin(q3 + q1 + q2);

  double cM1m2 = cos(q1 - q2);
  double sM1m2 = sin(q1 - q2);

  double c12 = cos(q1 + q2);
  double s12 = sin(q1 + q2);

  double c234 = cos(q2 + q3 + q4);
  double s234 = sin(q2 + q3 + q4);

  double s23 = sin(q2 + q3);
  double c23 = cos(q2 + q3);

  double c5 = cos(q5);
  double s5 = sin(q5);

  double c6 = cos(q6);
  double s6 = sin(q6);

  double c2 = cos(q2);
  double s2 = sin(q2);

  double c1 = cos(q1);
  double s1 = sin(q1);

  inT[0].matrix() << c1, 0, s1, 0, s1, 0, -c1, 0, 0, 1, 0, L1, 0, 0, 0, 1;

  inT[1].matrix() << c1 * c2, -c1 * s2, s1, c1 * c2 * L2, s1 * c2, -s1 * s2, -c1, s1 * c2 * L2, s2, c2, 0, s2 * L2 + L1,
      0, 0, 0, 1;

  inT[2].matrix() << u1_2 * cm3M1m2 + u1_2 * cM3M1M2, u1_2 * sm3M1m2 - u1_2 * sM3M1M2, s1,
      u1_2 * L3 * cm3M1m2 + u1_2 * L3 * cM3M1M2 + u1_2 * L2 * cM1m2 + u1_2 * L2 * c12, u1_2 * sM3M1M2 + u1_2 * sm3M1m2,
      u1_2 * cM3M1M2 - u1_2 * cm3M1m2, -c1,
      u1_2 * L3 * sM3M1M2 + u1_2 * L3 * sm3M1m2 + u1_2 * L2 * s12 + u1_2 * L2 * sM1m2, s23, c23, 0,
      L3 * s23 + s2 * L2 + L1, 0, 0, 0, 1;

  inT[3].matrix() << u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2, s1, -u1_2 * sm4m3M1m2 + u1_2 * sM4M3M1M2,
      s1 * L4 + u1_2 * L3 * cm3M1m2 + u1_2 * L3 * cM3M1M2 + u1_2 * L2 * cM1m2 + u1_2 * L2 * c12,
      u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2, -c1, -u1_2 * cM4M3M1M2 + u1_2 * cm4m3M1m2,
      -c1 * L4 + u1_2 * L3 * sM3M1M2 + u1_2 * L3 * sm3M1m2 + u1_2 * L2 * s12 + u1_2 * L2 * sM1m2, s234, 0, -c234,
      L3 * s23 + s2 * L2 + L1, 0, 0, 0, 1;

  inT[4].matrix() << (u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * c5 + s1s5, u1_2 * sm4m3M1m2 - u1_2 * sM4M3M1M2,
      -(u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * s5 + s1c5,
      (-u1_2 * sm4m3M1m2 + u1_2 * sM4M3M1M2) * L5 + s1 * L4 + u1_2 * L3 * cm3M1m2 + u1_2 * L3 * cM3M1M2 +
          u1_2 * L2 * cM1m2 + u1_2 * L2 * c12,
      (u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * c5 - c1s5, u1_2 * cM4M3M1M2 - u1_2 * cm4m3M1m2,
      -(u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * s5 - c1c5,
      (-u1_2 * cM4M3M1M2 + u1_2 * cm4m3M1m2) * L5 - c1 * L4 + u1_2 * L3 * sM3M1M2 + u1_2 * L3 * sm3M1m2 +
          u1_2 * L2 * s12 + u1_2 * L2 * sM1m2,
      s234 * c5, c234, -s234 * s5, -c234 * L5 + L3 * s23 + s2 * L2 + L1, 0, 0, 0, 1;

  inT[5].matrix() << ((u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * c5 + s1s5) * c6 +
                         (u1_2 * sm4m3M1m2 - u1_2 * sM4M3M1M2) * s6,
      -((u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * c5 + s1s5) * s6 + (u1_2 * sm4m3M1m2 - u1_2 * sM4M3M1M2) * c6,
      -(u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * s5 + s1c5,
      (-(u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * s5 + s1c5) * L6 + (-u1_2 * sm4m3M1m2 + u1_2 * sM4M3M1M2) * L5 +
          s1 * L4 + u1_2 * L3 * cm3M1m2 + u1_2 * L3 * cM3M1M2 + u1_2 * L2 * cM1m2 + u1_2 * L2 * c12,
      ((u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * c5 - c1s5) * c6 + (u1_2 * cM4M3M1M2 - u1_2 * cm4m3M1m2) * s6,
      -((u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * c5 - c1s5) * s6 + (u1_2 * cM4M3M1M2 - u1_2 * cm4m3M1m2) * c6,
      -(u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * s5 - c1c5,
      (-(u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * s5 - c1c5) * L6 + (-u1_2 * cM4M3M1M2 + u1_2 * cm4m3M1m2) * L5 -
          c1 * L4 + u1_2 * L3 * sM3M1M2 + u1_2 * L3 * sm3M1m2 + u1_2 * L2 * s12 + u1_2 * L2 * sM1m2,
      s234 * c5c6 + c234 * s6, -s234 * c5s6 + c234 * c6, -s234 * s5,
      -s234 * s5 * L6 - c234 * L5 + L3 * s23 + s2 * L2 + L1, 0, 0, 0, 1;
}

void KinematicModel::updateTi_0()
{
  double q1 = m_q.q(0);
  double q2 = m_q.q(1);
  double q3 = m_q.q(2);
  double q4 = m_q.q(3);
  double q5 = m_q.q(4);
  double q6 = m_q.q(5);

  double cm4m3M1m2 = cos(-q4 - q3 + q1 - q2);
  double sm4m3M1m2 = sin(-q4 - q3 + q1 - q2);
  double cM4M3M1M2 = cos(q4 + q3 + q1 + q2);
  double sM4M3M1M2 = sin(q4 + q3 + q1 + q2);

  double s1s5 = sin(q1) * sin(q5);
  double s1c5 = sin(q1) * cos(q5);

  double c1c5 = cos(q1) * cos(q5);
  double c1s5 = cos(q1) * sin(q5);

  double c5c6 = cos(q5) * cos(q6);
  double c5s6 = cos(q5) * sin(q6);

  double cm3M1m2 = cos(-q3 + q1 - q2);
  double sm3M1m2 = sin(-q3 + q1 - q2);

  double cM3M1M2 = cos(q3 + q1 + q2);
  double sM3M1M2 = sin(q3 + q1 + q2);

  double cM1m2 = cos(q1 - q2);
  double sM1m2 = sin(q1 - q2);

  double c12 = cos(q1 + q2);
  double s12 = sin(q1 + q2);

  double c234 = cos(q2 + q3 + q4);
  double s234 = sin(q2 + q3 + q4);

  double s23 = sin(q2 + q3);
  double c23 = cos(q2 + q3);

  double c5 = cos(q5);
  double s5 = sin(q5);

  double c6 = cos(q6);
  double s6 = sin(q6);

  double c2 = cos(q2);
  double s2 = sin(q2);

  double c1 = cos(q1);
  double s1 = sin(q1);

  // TODO: add as m_Ti_0[0] the identity matrix and change the JointId-1 to simple JointId everywhere!!!!

  m_Ti_0[0].matrix() << c1, 0, s1, 0, s1, 0, -c1, 0, 0, 1, 0, L1, 0, 0, 0, 1;

  m_Ti_0[1].matrix() << c1 * c2, -c1 * s2, s1, c1 * c2 * L2, s1 * c2, -s1 * s2, -c1, s1 * c2 * L2, s2, c2, 0,
      s2 * L2 + L1, 0, 0, 0, 1;

  m_Ti_0[2].matrix() << u1_2 * cm3M1m2 + u1_2 * cM3M1M2, u1_2 * sm3M1m2 - u1_2 * sM3M1M2, s1,
      u1_2 * L3 * cm3M1m2 + u1_2 * L3 * cM3M1M2 + u1_2 * L2 * cM1m2 + u1_2 * L2 * c12, u1_2 * sM3M1M2 + u1_2 * sm3M1m2,
      u1_2 * cM3M1M2 - u1_2 * cm3M1m2, -c1,
      u1_2 * L3 * sM3M1M2 + u1_2 * L3 * sm3M1m2 + u1_2 * L2 * s12 + u1_2 * L2 * sM1m2, s23, c23, 0,
      L3 * s23 + s2 * L2 + L1, 0, 0, 0, 1;

  m_Ti_0[3].matrix() << u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2, s1, -u1_2 * sm4m3M1m2 + u1_2 * sM4M3M1M2,
      s1 * L4 + u1_2 * L3 * cm3M1m2 + u1_2 * L3 * cM3M1M2 + u1_2 * L2 * cM1m2 + u1_2 * L2 * c12,
      u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2, -c1, -u1_2 * cM4M3M1M2 + u1_2 * cm4m3M1m2,
      -c1 * L4 + u1_2 * L3 * sM3M1M2 + u1_2 * L3 * sm3M1m2 + u1_2 * L2 * s12 + u1_2 * L2 * sM1m2, s234, 0, -c234,
      L3 * s23 + s2 * L2 + L1, 0, 0, 0, 1;

  m_Ti_0[4].matrix() << (u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * c5 + s1s5, u1_2 * sm4m3M1m2 - u1_2 * sM4M3M1M2,
      -(u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * s5 + s1c5,
      (-u1_2 * sm4m3M1m2 + u1_2 * sM4M3M1M2) * L5 + s1 * L4 + u1_2 * L3 * cm3M1m2 + u1_2 * L3 * cM3M1M2 +
          u1_2 * L2 * cM1m2 + u1_2 * L2 * c12,
      (u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * c5 - c1s5, u1_2 * cM4M3M1M2 - u1_2 * cm4m3M1m2,
      -(u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * s5 - c1c5,
      (-u1_2 * cM4M3M1M2 + u1_2 * cm4m3M1m2) * L5 - c1 * L4 + u1_2 * L3 * sM3M1M2 + u1_2 * L3 * sm3M1m2 +
          u1_2 * L2 * s12 + u1_2 * L2 * sM1m2,
      s234 * c5, c234, -s234 * s5, -c234 * L5 + L3 * s23 + s2 * L2 + L1, 0, 0, 0, 1;

  m_Ti_0[5].matrix() << ((u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * c5 + s1s5) * c6 +
                            (u1_2 * sm4m3M1m2 - u1_2 * sM4M3M1M2) * s6,
      -((u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * c5 + s1s5) * s6 + (u1_2 * sm4m3M1m2 - u1_2 * sM4M3M1M2) * c6,
      -(u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * s5 + s1c5,
      (-(u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * s5 + s1c5) * L6 + (-u1_2 * sm4m3M1m2 + u1_2 * sM4M3M1M2) * L5 +
          s1 * L4 + u1_2 * L3 * cm3M1m2 + u1_2 * L3 * cM3M1M2 + u1_2 * L2 * cM1m2 + u1_2 * L2 * c12,
      ((u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * c5 - c1s5) * c6 + (u1_2 * cM4M3M1M2 - u1_2 * cm4m3M1m2) * s6,
      -((u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * c5 - c1s5) * s6 + (u1_2 * cM4M3M1M2 - u1_2 * cm4m3M1m2) * c6,
      -(u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * s5 - c1c5,
      (-(u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * s5 - c1c5) * L6 + (-u1_2 * cM4M3M1M2 + u1_2 * cm4m3M1m2) * L5 -
          c1 * L4 + u1_2 * L3 * sM3M1M2 + u1_2 * L3 * sm3M1m2 + u1_2 * L2 * s12 + u1_2 * L2 * sM1m2,
      s234 * c5c6 + c234 * s6, -s234 * c5s6 + c234 * c6, -s234 * s5,
      -s234 * s5 * L6 - c234 * L5 + L3 * s23 + s2 * L2 + L1, 0, 0, 0, 1;
}

void KinematicModel::updateTi_0_pred()
{
  double q1 = m_q.q(0) + m_q.qp(0) * 0.08;
  double q2 = m_q.q(1) + m_q.qp(1) * 0.08;
  double q3 = m_q.q(2) + m_q.qp(2) * 0.08;
  double q4 = m_q.q(3) + m_q.qp(3) * 0.08;
  double q5 = m_q.q(4) + m_q.qp(1) * 0.08;
  double q6 = m_q.q(5) + m_q.qp(5) * 0.08;

  double cm4m3M1m2 = cos(-q4 - q3 + q1 - q2);
  double sm4m3M1m2 = sin(-q4 - q3 + q1 - q2);
  double cM4M3M1M2 = cos(q4 + q3 + q1 + q2);
  double sM4M3M1M2 = sin(q4 + q3 + q1 + q2);

  double s1s5 = sin(q1) * sin(q5);
  double s1c5 = sin(q1) * cos(q5);

  double c1c5 = cos(q1) * cos(q5);
  double c1s5 = cos(q1) * sin(q5);

  double c5c6 = cos(q5) * cos(q6);
  double c5s6 = cos(q5) * sin(q6);

  double cm3M1m2 = cos(-q3 + q1 - q2);
  double sm3M1m2 = sin(-q3 + q1 - q2);

  double cM3M1M2 = cos(q3 + q1 + q2);
  double sM3M1M2 = sin(q3 + q1 + q2);

  double cM1m2 = cos(q1 - q2);
  double sM1m2 = sin(q1 - q2);

  double c12 = cos(q1 + q2);
  double s12 = sin(q1 + q2);

  double c234 = cos(q2 + q3 + q4);
  double s234 = sin(q2 + q3 + q4);

  double s23 = sin(q2 + q3);
  double c23 = cos(q2 + q3);

  double c5 = cos(q5);
  double s5 = sin(q5);

  double c6 = cos(q6);
  double s6 = sin(q6);

  double c2 = cos(q2);
  double s2 = sin(q2);

  double c1 = cos(q1);
  double s1 = sin(q1);

  // TODO: add as m_Ti_0[0] the identity matrix and change the JointId-1 to simple JointId everywhere!!!!

  m_Ti_0_pred[0].matrix() << c1, 0, s1, 0, s1, 0, -c1, 0, 0, 1, 0, L1, 0, 0, 0, 1;

  m_Ti_0_pred[1].matrix() << c1 * c2, -c1 * s2, s1, c1 * c2 * L2, s1 * c2, -s1 * s2, -c1, s1 * c2 * L2, s2, c2, 0,
      s2 * L2 + L1, 0, 0, 0, 1;

  m_Ti_0_pred[2].matrix() << u1_2 * cm3M1m2 + u1_2 * cM3M1M2, u1_2 * sm3M1m2 - u1_2 * sM3M1M2, s1,
      u1_2 * L3 * cm3M1m2 + u1_2 * L3 * cM3M1M2 + u1_2 * L2 * cM1m2 + u1_2 * L2 * c12, u1_2 * sM3M1M2 + u1_2 * sm3M1m2,
      u1_2 * cM3M1M2 - u1_2 * cm3M1m2, -c1,
      u1_2 * L3 * sM3M1M2 + u1_2 * L3 * sm3M1m2 + u1_2 * L2 * s12 + u1_2 * L2 * sM1m2, s23, c23, 0,
      L3 * s23 + s2 * L2 + L1, 0, 0, 0, 1;

  m_Ti_0_pred[3].matrix() << u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2, s1, -u1_2 * sm4m3M1m2 + u1_2 * sM4M3M1M2,
      s1 * L4 + u1_2 * L3 * cm3M1m2 + u1_2 * L3 * cM3M1M2 + u1_2 * L2 * cM1m2 + u1_2 * L2 * c12,
      u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2, -c1, -u1_2 * cM4M3M1M2 + u1_2 * cm4m3M1m2,
      -c1 * L4 + u1_2 * L3 * sM3M1M2 + u1_2 * L3 * sm3M1m2 + u1_2 * L2 * s12 + u1_2 * L2 * sM1m2, s234, 0, -c234,
      L3 * s23 + s2 * L2 + L1, 0, 0, 0, 1;

  m_Ti_0_pred[4].matrix() << (u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * c5 + s1s5, u1_2 * sm4m3M1m2 - u1_2 * sM4M3M1M2,
      -(u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * s5 + s1c5,
      (-u1_2 * sm4m3M1m2 + u1_2 * sM4M3M1M2) * L5 + s1 * L4 + u1_2 * L3 * cm3M1m2 + u1_2 * L3 * cM3M1M2 +
          u1_2 * L2 * cM1m2 + u1_2 * L2 * c12,
      (u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * c5 - c1s5, u1_2 * cM4M3M1M2 - u1_2 * cm4m3M1m2,
      -(u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * s5 - c1c5,
      (-u1_2 * cM4M3M1M2 + u1_2 * cm4m3M1m2) * L5 - c1 * L4 + u1_2 * L3 * sM3M1M2 + u1_2 * L3 * sm3M1m2 +
          u1_2 * L2 * s12 + u1_2 * L2 * sM1m2,
      s234 * c5, c234, -s234 * s5, -c234 * L5 + L3 * s23 + s2 * L2 + L1, 0, 0, 0, 1;

  m_Ti_0_pred[5].matrix() << ((u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * c5 + s1s5) * c6 +
                                 (u1_2 * sm4m3M1m2 - u1_2 * sM4M3M1M2) * s6,
      -((u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * c5 + s1s5) * s6 + (u1_2 * sm4m3M1m2 - u1_2 * sM4M3M1M2) * c6,
      -(u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * s5 + s1c5,
      (-(u1_2 * cm4m3M1m2 + u1_2 * cM4M3M1M2) * s5 + s1c5) * L6 + (-u1_2 * sm4m3M1m2 + u1_2 * sM4M3M1M2) * L5 +
          s1 * L4 + u1_2 * L3 * cm3M1m2 + u1_2 * L3 * cM3M1M2 + u1_2 * L2 * cM1m2 + u1_2 * L2 * c12,
      ((u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * c5 - c1s5) * c6 + (u1_2 * cM4M3M1M2 - u1_2 * cm4m3M1m2) * s6,
      -((u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * c5 - c1s5) * s6 + (u1_2 * cM4M3M1M2 - u1_2 * cm4m3M1m2) * c6,
      -(u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * s5 - c1c5,
      (-(u1_2 * sM4M3M1M2 + u1_2 * sm4m3M1m2) * s5 - c1c5) * L6 + (-u1_2 * cM4M3M1M2 + u1_2 * cm4m3M1m2) * L5 -
          c1 * L4 + u1_2 * L3 * sM3M1M2 + u1_2 * L3 * sm3M1m2 + u1_2 * L2 * s12 + u1_2 * L2 * sM1m2,
      s234 * c5c6 + c234 * s6, -s234 * c5s6 + c234 * c6, -s234 * s5,
      -s234 * s5 * L6 - c234 * L5 + L3 * s23 + s2 * L2 + L1, 0, 0, 0, 1;
}

void KinematicModel::updateJef_0()
{
  double q1 = m_q.q(0);
  double q2 = m_q.q(1);
  double q3 = m_q.q(2);
  double q4 = m_q.q(3);
  double q5 = m_q.q(4);

  double cm5M4M3M1M2 = cos(-q5 + q4 + q3 + q1 + q2);
  double sm5M4M3M1M2 = sin(-q5 + q4 + q3 + q1 + q2);

  double cM5M4M3M1M2 = cos(q5 + q4 + q3 + q1 + q2);
  double sM5M4M3M1M2 = sin(q5 + q4 + q3 + q1 + q2);

  double cM5m4m3M1m2 = cos(q5 - q4 - q3 + q1 - q2);
  double sM5m4m3M1m2 = sin(q5 - q4 - q3 + q1 - q2);

  double cm5m4m3M1m2 = cos(-q5 - q4 - q3 + q1 - q2);
  double sm5m4m3M1m2 = sin(-q5 - q4 - q3 + q1 - q2);

  double cM1m5 = cos(q1 - q5);
  double sM1m5 = sin(q1 - q5);

  double cM1M5 = cos(q1 + q5);
  double sM1M5 = sin(q1 + q5);

  double cM4M3M1M2 = cos(q4 + q3 + q1 + q2);
  double sM4M3M1M2 = sin(q4 + q3 + q1 + q2);

  double cm4m3M1m2 = cos(-q4 - q3 + q1 - q2);
  double sm4m3M1m2 = sin(-q4 - q3 + q1 - q2);

  double c1 = cos(q1);
  double s1 = sin(q1);

  double c2 = cos(q2);

  double cM3M1M2 = cos(q3 + q1 + q2);
  double sM3M1M2 = sin(q3 + q1 + q2);

  double cm3M1m2 = cos(-q3 + q1 - q2);
  double sm3M1m2 = sin(-q3 + q1 - q2);

  double cM2M3M4 = cos(q2 + q3 + q4);
  double sM2M3M4 = sin(q2 + q3 + q4);

  double cM2M3M4m5 = cos(q2 + q3 + q4 - q5);
  double sM2M3M4m5 = sin(q2 + q3 + q4 - q5);

  double cM2M3M4M5 = cos(q2 + q3 + q4 + q5);
  double sM2M3M4M5 = sin(q2 + q3 + q4 + q5);

  double c12 = cos(q1 + q2);
  double s12 = sin(q1 + q2);

  double cM1m2 = cos(q1 - q2);
  double sM1m2 = sin(q1 - q2);

  double c23 = cos(q2 + q3);

  m_Jef_0(0, 0) = u1_4 * L6 * cm5M4M3M1M2 - u1_4 * L6 * cM5M4M3M1M2 + u1_4 * L6 * cm5m4m3M1m2 -
                  u1_4 * L6 * cM5m4m3M1m2 + u1_2 * L6 * cM1m5 + u1_2 * L6 * cM1M5 + u1_2 * L5 * cM4M3M1M2 -
                  u1_2 * L5 * cm4m3M1m2 + c1 * L4 - u1_2 * L3 * sM3M1M2 - u1_2 * L3 * sm3M1m2 - u1_2 * L2 * s12 -
                  u1_2 * L2 * sM1m2;
  m_Jef_0(0, 1) = u1_4 * L6 * cM5m4m3M1m2 + u1_4 * L6 * cm5M4M3M1M2 - u1_4 * L6 * cm5m4m3M1m2 -
                  u1_4 * L6 * cM5M4M3M1M2 + u1_2 * L5 * cm4m3M1m2 + u1_2 * L5 * cM4M3M1M2 - u1_2 * L3 * sM3M1M2 +
                  u1_2 * L3 * sm3M1m2 - u1_2 * L2 * s12 + u1_2 * L2 * sM1m2;
  m_Jef_0(0, 2) = u1_4 * L6 * cM5m4m3M1m2 + u1_4 * L6 * cm5M4M3M1M2 - u1_4 * L6 * cm5m4m3M1m2 -
                  u1_4 * L6 * cM5M4M3M1M2 + u1_2 * L5 * cm4m3M1m2 + u1_2 * L5 * cM4M3M1M2 - u1_2 * L3 * sM3M1M2 +
                  u1_2 * L3 * sm3M1m2;
  m_Jef_0(0, 3) = u1_4 * L6 * cM5m4m3M1m2 + u1_4 * L6 * cm5M4M3M1M2 - u1_4 * L6 * cm5m4m3M1m2 -
                  u1_4 * L6 * cM5M4M3M1M2 + u1_2 * L5 * cm4m3M1m2 + u1_2 * L5 * cM4M3M1M2;
  m_Jef_0(0, 4) = u1_2 * L6 * cM1M5 - u1_2 * L6 * cM1m5 - u1_4 * L6 * cm5m4m3M1m2 - u1_4 * L6 * cm5M4M3M1M2 -
                  u1_4 * L6 * cM5m4m3M1m2 - u1_4 * L6 * cM5M4M3M1M2;
  m_Jef_0(0, 5) = 0;

  m_Jef_0(1, 0) = -u1_4 * L6 * sM5m4m3M1m2 + u1_4 * L6 * sm5m4m3M1m2 - u1_4 * L6 * sM5M4M3M1M2 +
                  u1_4 * L6 * sm5M4M3M1M2 + u1_2 * L6 * sM1M5 + u1_2 * L6 * sM1m5 - u1_2 * L5 * sm4m3M1m2 +
                  u1_2 * L5 * sM4M3M1M2 + s1 * L4 + u1_2 * L3 * cm3M1m2 + u1_2 * L3 * cM3M1M2 + u1_2 * L2 * cM1m2 +
                  u1_2 * L2 * c12;
  m_Jef_0(1, 1) = u1_4 * L6 * sm5M4M3M1M2 + u1_4 * L6 * sM5m4m3M1m2 - u1_4 * L6 * sM5M4M3M1M2 -
                  u1_4 * L6 * sm5m4m3M1m2 + u1_2 * L5 * sM4M3M1M2 + u1_2 * L5 * sm4m3M1m2 - u1_2 * L3 * cm3M1m2 +
                  u1_2 * L3 * cM3M1M2 - u1_2 * L2 * cM1m2 + u1_2 * L2 * c12;
  m_Jef_0(1, 2) = u1_4 * L6 * sm5M4M3M1M2 + u1_4 * L6 * sM5m4m3M1m2 - u1_4 * L6 * sM5M4M3M1M2 -
                  u1_4 * L6 * sm5m4m3M1m2 + u1_2 * L5 * sM4M3M1M2 + u1_2 * L5 * sm4m3M1m2 - u1_2 * L3 * cm3M1m2 +
                  u1_2 * L3 * cM3M1M2;
  m_Jef_0(1, 3) = u1_4 * L6 * sm5M4M3M1M2 + u1_4 * L6 * sM5m4m3M1m2 - u1_4 * L6 * sM5M4M3M1M2 -
                  u1_4 * L6 * sm5m4m3M1m2 + u1_2 * L5 * sM4M3M1M2 + u1_2 * L5 * sm4m3M1m2;
  m_Jef_0(1, 4) = u1_2 * L6 * sM1M5 - u1_2 * L6 * sM1m5 - u1_4 * L6 * sM5M4M3M1M2 - u1_4 * L6 * sM5m4m3M1m2 -
                  u1_4 * L6 * sm5M4M3M1M2 - u1_4 * L6 * sm5m4m3M1m2;
  m_Jef_0(1, 5) = 0;

  m_Jef_0(2, 0) = 0;
  m_Jef_0(2, 1) = L5 * sM2M3M4 + L3 * c23 + u1_2 * L6 * sM2M3M4m5 - u1_2 * L6 * sM2M3M4M5 + c2 * L2;
  m_Jef_0(2, 2) = L5 * sM2M3M4 + L3 * c23 + u1_2 * L6 * sM2M3M4m5 - u1_2 * L6 * sM2M3M4M5;
  m_Jef_0(2, 3) = u1_2 * L6 * sM2M3M4m5 - u1_2 * L6 * sM2M3M4M5 + L5 * sM2M3M4;
  m_Jef_0(2, 4) = -u1_2 * L6 * sM2M3M4m5 - u1_2 * L6 * sM2M3M4M5;
  m_Jef_0(2, 5) = 0;

  m_Jef_0(3, 0) = 0;
  m_Jef_0(3, 1) = s1;
  m_Jef_0(3, 2) = s1;
  m_Jef_0(3, 3) = s1;
  m_Jef_0(3, 4) = -u1_2 * sm4m3M1m2 + u1_2 * sM4M3M1M2;
  m_Jef_0(3, 5) =
      -u1_4 * sM5m4m3M1m2 + u1_4 * sm5m4m3M1m2 - u1_4 * sM5M4M3M1M2 + u1_4 * sm5M4M3M1M2 + u1_2 * sM1M5 + u1_2 * sM1m5;

  m_Jef_0(4, 0) = 0;
  m_Jef_0(4, 1) = -c1;
  m_Jef_0(4, 2) = -c1;
  m_Jef_0(4, 3) = -c1;
  m_Jef_0(4, 4) = -u1_2 * cM4M3M1M2 + u1_2 * cm4m3M1m2;
  m_Jef_0(4, 5) =
      -u1_4 * cm5M4M3M1M2 + u1_4 * cM5M4M3M1M2 - u1_4 * cm5m4m3M1m2 + u1_4 * cM5m4m3M1m2 - u1_2 * cM1m5 - u1_2 * cM1M5;

  m_Jef_0(5, 0) = 1;
  m_Jef_0(5, 1) = 0;
  m_Jef_0(5, 2) = 0;
  m_Jef_0(5, 3) = 0;
  m_Jef_0(5, 4) = -cM2M3M4;
  m_Jef_0(5, 5) = -u1_2 * cM2M3M4m5 + u1_2 * cM2M3M4M5;
}

void KinematicModel::updateEFState()
{
  m_Tef_0 = FK();
  m_xef_0 = m_Tef_0.translation();
  m_Ref_0 = m_Tef_0.rotation();

  m_Tef_w = m_T0_w * m_Tef_0;
  m_xef_w = m_Tef_w.translation();
  m_Ref_w = m_Tef_w.rotation();

  // This function updates m_Jef_0
  updateJef_0();

  // Velocity EF
  m_Xefp_0 = m_Jef_0 * m_q.qp;
  m_xefp_0 = m_Xefp_0.block(0, 0, 3, 1);
  m_wefp_0 = m_Xefp_0.block(3, 0, 3, 1);

  Vector4d xefp_0h, xefp_wh;

  // We can not use homogenous() since will add a 1 at the 4th row!
  xefp_0h << m_xefp_0, 0;
  xefp_wh = m_T0_w * xefp_0h;
  m_xefp_w << xefp_wh.block(0, 0, 3, 1);
}

void KinematicModel::updateJointAxes_0()
{
  for (int i = 0; i < STD_DOF; i++)
  {
    Vector3d temp;
    m_x_j_0[i] = MathTools::x(T_j_0(i));
    m_y_j_0[i] = MathTools::y(T_j_0(i));
    m_z_j_0[i] = MathTools::z(T_j_0(i));
    m_t_j_0[i] = MathTools::t(T_j_0(i));
  }
}

}  // namespace Robot
}  // namespace tum_ics_ur_robot_lli
