#ifndef UR_ROBOT_LLI_TRAJECTORIES_H_
#define UR_ROBOT_LLI_TRAJECTORIES_H_

#include <ow_core/types.h>
#include <ow_core/trajectory/state_trajectories.h>
#include <ow_core/trajectory/trajectories.h>

/**
 * @brief Create a Cartesian Circle Trajectory
 * 
 */
namespace ow
{

/**
 * @brief 5th order Spline in JointSpace
 */
inline ow_core::PolynomialTrajectory<ow::JointPosition, ow::Scalar> JointPolynomial(
    ow::Scalar period,
    const ow::JointPosition &q_start,
    const ow::JointPosition &q_end)
{
  static ow::JointPosition zero = ow::JointPosition::Zero();
  return ow_core::Polynomial5Order<ow::JointPosition>(
      period, q_start, zero, zero, q_end, zero, zero);
}

/**
 * @brief 5th order Spline in CartesianSpace
 */
inline ow_core::PolynomialTrajectory<ow::CartesianPosition, ow::Scalar> CartesianPolynomial(
    ow::Scalar period,
    const ow::CartesianPosition &X_start,
    const ow::CartesianPosition &X_end)
{
  static ow::CartesianPosition zero(0,0,0,0,0,0,0);
  return ow_core::Polynomial5Order<ow::CartesianPosition>(
      period, X_start, zero, zero, X_end, zero, zero);
}

inline ow_core::PolynomialTrajectory<ow::CartesianPosition, ow::Scalar> CartesianStatePolynomial(
    ow::Scalar period,
    const ow::CartesianPosition &X_start,
    const ow::CartesianPosition &X_end)
{
  static ow::CartesianPosition zero(0,0,0,0,0,0,0);
  return ow_core::Polynomial5Order<ow::CartesianPosition>(
      period, X_start, zero, zero, X_end, zero, zero);
}

class CircularCartesianTrajectory
{
public: 
  /**
   * @brief Construct a new Circular Cartesian Trajectory object
   * 
   * @param period of the circular motion 
   * @param x_i_w inital starting point of the motion
   * @param n_w axis perpendicular to the plane of the circle
   * @param d_w point on that axis to define the center of the circle in space
   */
    CircularCartesianTrajectory(
      ow::Scalar period,
      const ow::LinearPosition& x_i_w, 
      const ow::LinearPosition& n_w, 
      const ow::LinearPosition& d_w) : 
      omega_(2.0*M_PI/period),
      x_i_w_(x_i_w),
      n_w_(n_w),
      d_w_(d_w)
    {
      defineRotation();
    }
    
    virtual ~CircularCartesianTrajectory()
    {
    }

    /**
     * @brief evaluate the trajectory at given time
     * 
     * @param time 
     * @return ow::LinearState 
     */
    ow::LinearState evaluate(ow::Scalar time) 
    {
        // phase
        ow::Scalar s = omega_*time;

        // postion, tangential, curvature vector in the circle frame
        ow::Vector3 p, t, c;
        p << cos(s), sin(s), 0;
        t << -sin(s), cos(s), 0;
        c << -cos(s), -sin(s), 0;
        
        // pos, velocity, acceleration in the world frame
        ow::LinearState x_state;
        x_state.pos() = r_*R_c_w_*p + c_w_;
        x_state.vel() = omega_*r_*R_c_w_*t;
        x_state.acc() = omega_*omega_*r_*R_c_w_*c;

        return x_state;
    }

    /**
     * @brief convert to ros message type
     * 
     * @param n_steps 
     * @return std::vector<geometry_msgs::PoseStamped> 
     */
    std::vector<geometry_msgs::PoseStamped> toNavPath(size_t n_steps)
    {
      std::vector<geometry_msgs::PoseStamped> path;
      geometry_msgs::PoseStamped pose;

      double dt_eval = 2*M_PI/(omega_*ow::Scalar(n_steps));

      for(size_t i = 0; i < n_steps; ++i)
      {
        ow::Scalar time = i*dt_eval;
        pose.pose.position = evaluate(time).pos();
        pose.pose.orientation = R_c_w_;
        path.push_back(pose);
      }
      return path;
    }


private:
    void defineRotation() 
    {
        // compute center point c_w such that c_w and x_i_w lie in the same 
        // plane perpendicular to axis a_w
        ow::Vector3 a_w = n_w_.normalized();
        c_w_ = d_w_ + (x_i_w_ - d_w_).dot(a_w) * a_w;

        // define a circle frame at center c_w such that its x-axis points into
        // the direction of x_i_w and the z-axis point along the axis a_w.
        r_ = (x_i_w_ - c_w_).norm();
        ow::Vector3 x_axis = (x_i_w_ - c_w_) / r_;
        ow::Vector3 z_axis = a_w;
        ow::Vector3 y_axis = z_axis.cross(x_axis);

        // make a rotation of the circle frame to world frame
        R_c_w_ << x_axis, y_axis, z_axis;
    }

private:
  ow::Scalar omega_;
  ow::Scalar r_;
  ow::LinearPosition x_i_w_;
  ow::LinearPosition d_w_;
  ow::Vector3 n_w_;
  ow::LinearPosition c_w_;
  ow::Rotation3 R_c_w_;
};

}

#endif
