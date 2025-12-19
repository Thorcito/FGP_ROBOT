#pragma once

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <ur10_robot_model/model_ur10.h>
#include <ow_core/trajectory/state_trajectories.h>
#include <tum_ics_ur10_controller_tutorial/trajectories.h>
#include <tum_ics_ur10_controller_tutorial/EETarget.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>



namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    class BallCatcherController : public ControlEffort
    {
    public:
      typedef ow_core::StateTrajectory<ow::JointState> JointStateTrajectory;

      enum State
      {
        CONSTRUCTED,        // 0
        INITIALIZED,        // 1
        JOINT_SPLINE,       // 2
        CARTESIAN_SPLINE,   // 3
        IDLE                // 4   
      };

    private:
      ros::NodeHandle nh_;
      // trajectroy
      ros::Publisher path_pub_;
      ros::Subscriber ee_target_sub_;
      ros::ServiceServer homing_srv_;

      State state_, prev_state_;

      JointState q_init_;
      JointState q_home_;
      JointState q_park_;

      // joint space gains
      ow::MatrixDof Kp_j_;
      ow::MatrixDof Kd_j_;

      // cartesian space gains
      ow::MatrixDof Kp_c_;
      ow::MatrixDof Kd_c_;
      ow::MatrixDof Ki_c_;

      Vector6d i_delta_x_;

      // joint space trajectory
      std::unique_ptr<JointStateTrajectory> joint_spline_;
      ow::VectorDof q_goal_;
      double joint_spline_dur_;
      ros::Time joint_spline_t_start_;

      // cartesian space REACH_HOME
      std::unique_ptr<ow::CartesianStateTrajectory> cartesian_spline_;
      ros::Time cartesian_spline_t_start_;

      ur::UR10Model model_;
      ur::UR10Model::Parameters theta_;

      ow::Vector6 tau_;
      ros::Time time_, prev_time_;

      bool start_cartesian_spline_;
      bool cartesian_spline_started_;
      ow::CartesianPosition X_goal_;
      double cs_spline_duration_;

      //Nads variables
      ros::Publisher controller_hb_pub_;
      ros::Publisher controller_ee_pub_;
      ros::Publisher ref_pose_pub_;
      ros::Publisher ref_vel_pub_;
      ros::Subscriber desired_pose_sub_;
      ros::Subscriber desired_twist_sub_;
      ros::Subscriber desired_time_sub_;

      bool have_des_pose_{false};
      bool have_des_twist_{false};
      bool have_des_time_{false};

      ow::CartesianPosition X_des_;
      ow::CartesianVelocity V_des_;
      double T_des_{0.0};
      ros::Time des_stamp_;




      
    public:
      BallCatcherController(double weight = 1.0, const QString &name = "BallCatcherController");

      ~BallCatcherController();

      void setQInit(const JointState &qinit);

      void setQHome(const JointState &qhome);

      void setQPark(const JointState &qpark);

    private:
      bool init();

      bool start();

      Vector6d update(const RobotTime &time, const JointState &current);

      bool stop();

    private:

      State transitState(const JointState &current);

      Vector6d joint_space_controller(const JointState &current, const ow::JointState &ref, const ros::Duration duration);

      Vector6d cartesian_space_controller(const JointState &current, const ow::CartesianState &ref, const ros::Duration duration);

      void start_cartesian_spline(const JointState &current, const ow::CartesianPosition &goal, double spline_duration);

      void eeTargetCallback(const tum_ics_ur10_controller_tutorial::EETargetConstPtr &msg);

      void desiredPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

      void desiredTwistCallback(const geometry_msgs::TwistStampedConstPtr &msg);

      void desiredTimeCallback(const std_msgs::Float32ConstPtr &msg);

      bool homingHandler(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

      void startLinearInterpolation(const ow::CartesianPosition &goal, double duration, const JointState &current);
    };

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
