#include <tum_ics_ur10_controller_tutorial/ball_catcher_controller.h>

#include <ow_core/common/parameter.h>

#include <nav_msgs/Path.h>
#include <ow_core/math.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    BallCatcherController::BallCatcherController(double weight, const QString &name) : ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
                                                                                       state_(CONSTRUCTED),
                                                                                       prev_state_(CONSTRUCTED),
                                                                                       Kp_j_(Matrix6d::Zero()),
                                                                                       Kd_j_(Matrix6d::Zero()),
                                                                                       Kp_c_(Matrix6d::Zero()),
                                                                                       Kd_c_(Matrix6d::Zero()),
                                                                                       Ki_c_(Matrix6d::Zero()),
                                                                                       model_("ur10_model"),
                                                                                       start_cartesian_spline_(false)
    {
      // ros publisher
      path_pub_ = nh_.advertise<nav_msgs::Path>("cartesian_path", 1);
      //ee_target_sub_ = nh_.subscribe("ee_target", 1, &BallCatcherController::eeTargetCallback, this);
      desired_pose_sub_  = nh_.subscribe("desired_pose",  1, &BallCatcherController::desiredPoseCallback, this);
      desired_twist_sub_ = nh_.subscribe("desired_twist", 1, &BallCatcherController::desiredTwistCallback, this);
      desired_time_sub_  = nh_.subscribe("desired_time",  1, &BallCatcherController::desiredTimeCallback, this);
      homing_srv_ = nh_.advertiseService("homing", &BallCatcherController::homingHandler, this);
      controller_hb_pub_ = nh_.advertise<std_msgs::Empty>("controller_heartbeat", 1);
      controller_ee_pub_ = nh_.advertise<geometry_msgs::PointStamped>("controller_ee", 1);
      ref_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("controller_ref_pose", 1);
      ref_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("controller_ref_vel", 1);


      // robot model
      if (!model_.initRequest(nh_))
      {
        ROS_ERROR("BallCatcherController: cannot initalize the model");
      }
    }

    BallCatcherController::~BallCatcherController()
    {
    }

    void BallCatcherController::setQInit(const JointState &qinit)
    {
      q_init_ = qinit;
    }
    void BallCatcherController::setQHome(const JointState &qhome)
    {
      q_home_ = qhome;
    }
    void BallCatcherController::setQPark(const JointState &qpark)
    {
      q_park_ = qpark;
    }

    bool BallCatcherController::init()
    {
      ROS_WARN_STREAM("BallCatcherController::init");

      // check namespace
      std::string ns = "~ball_catcher_controller/";
      if (!ros::param::has(ns))
      {
        ROS_ERROR_STREAM("BallCatcherController init(): Control gains not defined in:" << ns);
        return false;
      }

      std::vector<double> vec;

      //////////////////////////////////////////////////////////////////////////
      // joint space gains
      //////////////////////////////////////////////////////////////////////////
      // D gains
      ros::param::get(ns + "jointspace/gains_d", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("jointspace/gains_d: wrong number of dimensions:" << vec.size());
        return false;
      }
      for (size_t i = 0; i < STD_DOF; ++i)
      {
        Kd_j_(i, i) = vec[i];
      }
      // P gains
      ros::param::get(ns + "jointspace/gains_p", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("jointspace/gains_p: wrong number of dimensions:" << vec.size());
        return false;
      }
      for (size_t i = 0; i < STD_DOF; ++i)
      {
        Kp_j_(i, i) = vec[i];
      }
      // spline params
      ros::param::get(ns + "jointspace/goal", vec);
      if (vec.size() != STD_DOF)
      {
        ROS_ERROR_STREAM("jointspace/goal: wrong number of dimensions:" << vec.size());
        return false;
      }
      q_goal_ = ow::VectorDof::Map(vec.data(), vec.size());
      ros::param::get(ns + "jointspace/duration", joint_spline_dur_);

      //////////////////////////////////////////////////////////////////////////
      // cartesian space gains
      //////////////////////////////////////////////////////////////////////////
      // D gains
      ros::param::get(ns + "cartesianspace/gains_d", vec);
      if (vec.size() != 6)
      {
        ROS_ERROR_STREAM("cartesianspace/gains_d: wrong number of dimensions:" << vec.size());
        return false;
      }
      for (size_t i = 0; i < 6; ++i)
      {
        Kd_c_(i, i) = vec[i];
      }
      // P gains
      ros::param::get(ns + "cartesianspace/gains_p", vec);
      if (vec.size() != 6)
      {
        ROS_ERROR_STREAM("cartesianspace/gains_p: wrong number of dimensions:" << vec.size());
        return false;
      }
      for (size_t i = 0; i < 6; ++i)
      {
        Kp_c_(i, i) = vec[i];
      }

      ROS_INFO_STREAM("Joint Space Gains: \n"
                      << "Kp_j: \n"
                      << Kp_j_ << "\n"
                      << "Kd_j: \n"
                      << Kd_j_ << "\n");
      ROS_INFO_STREAM("Joint Space Spline: \n"
                      << "q_goal: \n"
                      << q_goal_.transpose() << "\n"
                      << "joint_spline_dur_: " << joint_spline_dur_ << "\n");
      ROS_INFO_STREAM("Cartesian Space Gains: \n"
                      << "Kp_c: \n"
                      << Kp_c_ << "\n"
                      << "Kd_c: \n"
                      << Kd_c_ << "\n");

      // reset
      // DeltaX_i.setZero();
      state_ = INITIALIZED;
      theta_ = model_.parameterInitalGuess();
      prev_time_ = ros::Time::now();
      return true;
    }

    bool BallCatcherController::start()
    {
      ROS_WARN_STREAM("BallCatcherController::start");
      return true;
    }

    Vector6d BallCatcherController::update(const RobotTime &time, const JointState &current)
    {
      state_ = transitState(current);

      ROS_WARN_STREAM_THROTTLE(0.5, "state: " << state_);

      model_.broadcastFrames(current.q, ros::Time::now());

      ow::CartesianPosition X_ee_w = ow::CartesianPosition(model_.T_tool_B(current.q));
      ROS_INFO_STREAM_THROTTLE(0.5, "X_ee_w: " << X_ee_w.transpose());

      // dt
      ros::Time t = ros::Time::now();
      ros::Duration period = t - prev_time_;
      prev_time_ = t;

      if (state_ == JOINT_SPLINE)
      {
        ros::Duration elapses = t - joint_spline_t_start_;
        ow::JointState js_ref = joint_spline_->evaluate(elapses.toSec());
        tau_ = joint_space_controller(current, js_ref, period);
      }

      // if (state_ == IDLE)
      // {
      //   ow::CartesianState cs_ref;
      //   cs_ref.pos() = X_goal_;
      //   cs_ref.vel().setZero();
      //   cs_ref.acc().setZero();
      //   tau_ = cartesian_space_controller(current, cs_ref, period);
      // }

      // if (state_ == CARTESIAN_SPLINE)
      // {
      //   ros::Duration elapses = t - cartesian_spline_t_start_;
      //   ROS_WARN_STREAM("elapses: " << elapses.toSec());
      //   ow::CartesianState cs_ref = cartesian_spline_->evaluate(elapses.toSec());
      //   ROS_INFO_STREAM("X_ref: " << cs_ref.pos().transpose());
      //   geometry_msgs::PoseStamped ref_pose_msg;
      //   ow::CartesianPosition X_ref = cs_ref.pos();
      //   ref_pose_msg.header.stamp = t;
      //   ref_pose_msg.header.frame_id = "ur10_model_dh_5";
      //   ref_pose_msg.pose.position.x = X_ref(0);
      //   ref_pose_msg.pose.position.y = X_ref(1);
      //   ref_pose_msg.pose.position.z = X_ref(2);
      //   ref_pose_msg.pose.orientation.x = X_ref(3);
      //   ref_pose_msg.pose.orientation.y = X_ref(4);
      //   ref_pose_msg.pose.orientation.z = X_ref(5);
      //   ref_pose_msg.pose.orientation.w = X_ref(6);
      //   ref_pose_pub_.publish(ref_pose_msg);

      //   geometry_msgs::TwistStamped ref_twist_msg;
      //   ow::CartesianVelocity V_ref = cs_ref.vel();
      //   ref_twist_msg.header = ref_pose_msg.header;
      //   ref_twist_msg.twist.linear.x  = V_ref(0);
      //   ref_twist_msg.twist.linear.y  = V_ref(1);
      //   ref_twist_msg.twist.linear.z  = V_ref(2);
      //   ref_twist_msg.twist.angular.x = V_ref(3);
      //   ref_twist_msg.twist.angular.y = V_ref(4);
      //   ref_twist_msg.twist.angular.z = V_ref(5);
      //   ref_vel_pub_.publish(ref_twist_msg);

      //   tau_ = cartesian_space_controller(current, cs_ref, period);
      // }

      if (state_ == IDLE)
      {
        ow::CartesianState cs_ref;

        if (have_des_pose_)
        {
          cs_ref.pos() = X_des_;
          if (have_des_twist_) cs_ref.vel() = V_des_;
          else cs_ref.vel().setZero();
          cs_ref.acc().setZero();

          // Optional: publish what you're tracking (nice for RViz/debug)
          geometry_msgs::PoseStamped ref_pose_msg;
          ref_pose_msg.header.stamp = t;
          ref_pose_msg.header.frame_id = "ur10_model_dh_0";
          ref_pose_msg.pose.position.x = X_des_(0);
          ref_pose_msg.pose.position.y = X_des_(1);
          ref_pose_msg.pose.position.z = X_des_(2);
          ref_pose_msg.pose.orientation.x = X_des_(3);
          ref_pose_msg.pose.orientation.y = X_des_(4);
          ref_pose_msg.pose.orientation.z = X_des_(5);
          ref_pose_msg.pose.orientation.w = X_des_(6);
          ref_pose_pub_.publish(ref_pose_msg);

          geometry_msgs::TwistStamped ref_twist_msg;
          ref_twist_msg.header = ref_pose_msg.header;
          ref_twist_msg.twist.linear.x  = V_des_(0);
          ref_twist_msg.twist.linear.y  = V_des_(1);
          ref_twist_msg.twist.linear.z  = V_des_(2);
          ref_twist_msg.twist.angular.x = V_des_(3);
          ref_twist_msg.twist.angular.y = V_des_(4);
          ref_twist_msg.twist.angular.z = V_des_(5);
          ref_vel_pub_.publish(ref_twist_msg);
        }
        else
        {
          // fallback: hold current pose (or X_goal_) with zero vel
          cs_ref.pos() = X_goal_;
          cs_ref.vel().setZero();
          cs_ref.acc().setZero();
        }

        tau_ = cartesian_space_controller(current, cs_ref, period);
      }


      std_msgs::Empty hb;
      controller_hb_pub_.publish(hb);
      geometry_msgs::PointStamped ee_pos;
      ee_pos.header.stamp = t;
      ee_pos.header.frame_id = "ur10_model_dh_0";
      ee_pos.point.x = X_ee_w(0);
      ee_pos.point.y = X_ee_w(1);
      ee_pos.point.z = X_ee_w(2);
      controller_ee_pub_.publish(ee_pos);
      return tau_;
    }

    bool BallCatcherController::stop()
    {
      return true;
    }

    BallCatcherController::State BallCatcherController::transitState(const JointState &current)
    {
      State next_state = state_;
      bool state_changed = false;

      if (state_ == INITIALIZED)
      {
        q_init_ = current;
        joint_spline_ = std::make_unique<JointStateTrajectory>(ow::JointPolynomial(joint_spline_dur_, q_init_.q, q_goal_));
        joint_spline_t_start_ = ros::Time::now();
        next_state = JOINT_SPLINE;
        state_changed = true;
      }

      if (state_ == JOINT_SPLINE && (ros::Time::now() - joint_spline_t_start_).toSec() >= joint_spline_dur_)
      {
        i_delta_x_.setZero();
        X_goal_ = ow::CartesianPosition(model_.T_tool_0(current.q));
        ROS_INFO_STREAM("X_goal for IDLE: " << X_goal_.transpose());
        next_state = IDLE;
        state_changed = true;
      }

      // if ((state_ == IDLE || state_ == CARTESIAN_SPLINE) && start_cartesian_spline_)
      // {
      //   start_cartesian_spline(current, X_goal_, cs_spline_duration_);
      //   start_cartesian_spline_ = false;
      //   next_state = CARTESIAN_SPLINE;
      //   // state_changed = true;
      // }

      // if (state_ == CARTESIAN_SPLINE && (ros::Time::now() - cartesian_spline_t_start_).toSec() >= cs_spline_duration_)
      // {
      //   i_delta_x_.setZero();
      //   //X_goal_ = ow::CartesianPosition(model_.T_tool_0(current.q));
      //   next_state = IDLE;
      //   state_changed = true;
      // }

      if (state_changed)
        ROS_WARN_STREAM("Starting state: " << next_state);
      return next_state;
    }

    Vector6d BallCatcherController::joint_space_controller(const JointState &current, const ow::JointState &ref, const ros::Duration period)
    {
      // errors
      ow::Vector6 q_delta = current.q - ref.q();
      ow::Vector6 qp_delta = current.qp - ref.qP();

      // reference
      JointState js_r;
      js_r.qp = ref.qP() - Kp_j_ * q_delta;
      constexpr double QDOT_MAX = 2.53;  // almost 145 deg/s in rad/s
      for (int i = 0; i < 6; ++i) {
        if (js_r.qp[i] >  QDOT_MAX) js_r.qp[i] =  QDOT_MAX;
        if (js_r.qp[i] < -QDOT_MAX) js_r.qp[i] = -QDOT_MAX;
      }
      js_r.qpp = ref.qPP() - Kp_j_ * qp_delta;
      constexpr double QDDOT_MAX = 6.4; // rad/s^2 
      for (int i = 0; i < 6; ++i) {
        if (js_r.qpp[i] >  QDDOT_MAX) js_r.qpp[i] =  QDDOT_MAX;
        if (js_r.qpp[i] < -QDDOT_MAX) js_r.qpp[i] = -QDDOT_MAX;
      }

      Vector6d Sq = current.qp - js_r.qp;
      ur::UR10Model::Regressor Yr = model_.regressor(current.q, current.qp, js_r.qp, js_r.qpp);

      Eigen::MatrixXd Gamma(81, 81);
      Gamma.setIdentity();
      ur::UR10Model::Parameters thp = -(Gamma.inverse() * Yr.transpose() * Sq);
      theta_ += period.toSec() * thp;

      Vector6d tau = -Kd_j_ * Sq + Yr * theta_;

      return tau;
    }

    Vector6d BallCatcherController::cartesian_space_controller(const JointState &current, const ow::CartesianState &ref, const ros::Duration period)
    {
      // Jacobian
      ow::Matrix6 J_tool_0 = model_.J_tool_0(current.q);
      ow::Matrix6 Jp_tool_0 = model_.Jp_tool_0(current.q, current.qp);

      ow::CartesianState cs_current;
      cs_current.pos() = ow::CartesianPosition(model_.T_tool_0(current.q));
      cs_current.vel() = ow::CartesianVelocity(J_tool_0 * current.qp);
      cs_current.acc().setZero();

      // errors
      Vector6d delta_x = ow::cartesianError(cs_current.pos(), ref.pos());
      Vector6d delta_xp = cs_current.vel() - ref.vel();
      i_delta_x_ += period.toSec() * delta_x;

      // // reference (PID)
      // Vector6d Xp_r = ref.vel() - Kp_c_ * delta_x - Ki_c_ * i_delta_x_;
      // Vector6d Xpp_r = ref.acc() - Kp_c_ * delta_xp - Ki_c_ * delta_x;

      // reference (PDD)
      Vector6d Xp_r = ref.vel() - Kp_c_ * delta_x;
      Vector6d Xpp_r = ref.acc() - Kp_c_ * delta_xp;

      JointState js_r;
      js_r.qp = J_tool_0.inverse() * Xp_r;
      ROS_INFO_STREAM_THROTTLE(0.5, "Desired Joint Velocities: " << js_r.qp.transpose());
      constexpr double QDOT_MAX = 2.53;  // almost 145 deg/s in rad/s
      for (int i = 0; i < 6; ++i) {
        if (js_r.qp[i] >  QDOT_MAX) js_r.qp[i] =  QDOT_MAX;
        if (js_r.qp[i] < -QDOT_MAX) js_r.qp[i] = -QDOT_MAX;
      }
      ROS_INFO_STREAM_THROTTLE(0.5, "Desired Joint Velocities Filtered: " << js_r.qp.transpose());
      js_r.qpp = J_tool_0.inverse() * (Xpp_r - Jp_tool_0 * js_r.qp);
      constexpr double QDDOT_MAX = 6.4; // rad/s^2 
      for (int i = 0; i < 6; ++i) {
        if (js_r.qpp[i] >  QDDOT_MAX) js_r.qpp[i] =  QDDOT_MAX;
        if (js_r.qpp[i] < -QDDOT_MAX) js_r.qpp[i] = -QDDOT_MAX;
      }
      Vector6d Sq = current.qp - js_r.qp;
      ur::UR10Model::Regressor Yr = model_.regressor(current.q, current.qp, js_r.qp, js_r.qpp);

      Eigen::MatrixXd Gamma(81, 81);
      Gamma.setIdentity();
      ur::UR10Model::Parameters thp = -(Gamma.inverse() * Yr.transpose() * Sq);
      theta_ += period.toSec() * thp;

      Vector6d tau = -Kd_c_ * Sq + Yr * theta_;
      return tau;
    }

    // void BallCatcherController::start_cartesian_spline(const JointState &current, const ow::CartesianPosition &goal, double spline_duration)
    // {
    //   ow::CartesianPosition X_current = ow::CartesianPosition(model_.T_tool_0(current.q));
    //   ow::Matrix6 J_tool_0 = model_.J_tool_0(current.q);
    //   ow::Vector6 v_ee     = J_tool_0 * current.qp;
    //   ow::CartesianPosition Xp_current;
    //   Xp_current.setZero();
    //   Xp_current.head(6) = v_ee;
    //   Xp_current(6) = 0.0;
    //   ow::CartesianPosition Xp_goal(0,0,0,0,0,0,0);


    //   ROS_WARN_STREAM("New Cartesian Spline");
    //   ROS_WARN_STREAM("start: " << X_current.transpose());
    //   ROS_WARN_STREAM("goal: " << goal.transpose());
    //   ROS_WARN_STREAM("duration: " << spline_duration);
    //   ROS_WARN_STREAM("velocity: " << v_ee.head<3>().transpose());

    //   // create a cartesian spline
    //   //cartesian_spline_ = std::make_unique<ow::CartesianStateTrajectory>(ow::CartesianPolynomial(spline_duration, X_current, goal));
    //   cartesian_spline_ = std::make_unique<ow::CartesianStateTrajectory>(ow_core::Polynomial3Order<ow::CartesianPosition>(spline_duration, X_current, Xp_current, goal, Xp_goal));
    //   cartesian_spline_t_start_ = ros::Time::now();
    // }

    // void BallCatcherController::eeTargetCallback(const tum_ics_ur10_controller_tutorial::EETargetConstPtr &msg)
    // {
    //   ow::CartesianPosition X_goal_w;
    //   X_goal_w = msg->ee_target;
    //   X_goal_ = ow::CartesianPosition(model_.T_0_B()) * X_goal_w; // to base frame
    //   cs_spline_duration_ = msg->duration;
    //   start_cartesian_spline_ = true;
    //   ROS_ERROR_STREAM("NEW EE GOAL: " << X_goal_.transpose() << "  TtoGo: " << cs_spline_duration_ << " s");
    // }

    void BallCatcherController::desiredPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
      ow::CartesianPosition X_w;
      X_w << msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w;

      X_des_ = ow::CartesianPosition(model_.T_0_B()) * X_w;  // B -> 0
      des_stamp_ = msg->header.stamp;
      have_des_pose_ = true;
    }

    void BallCatcherController::desiredTwistCallback(const geometry_msgs::TwistStampedConstPtr &msg)
    {
      Eigen::Matrix3d R_0B = model_.T_0_B().rotation();  // rotation from B to 0
      Eigen::Vector3d vB(msg->twist.linear.x,
                        msg->twist.linear.y,
                        msg->twist.linear.z);
      Eigen::Vector3d wB(msg->twist.angular.x,
                        msg->twist.angular.y,
                        msg->twist.angular.z);
      Eigen::Vector3d v0 = R_0B * vB;
      Eigen::Vector3d w0 = R_0B * wB;
      V_des_(0) = v0(0);
      V_des_(1) = v0(1);
      V_des_(2) = v0(2);
      V_des_(3) = w0(0);
      V_des_(4) = w0(1);
      V_des_(5) = w0(2);

      des_stamp_ = msg->header.stamp;
      have_des_twist_ = true;
    }

    void BallCatcherController::desiredTimeCallback(const std_msgs::Float32ConstPtr &msg)
    {
      T_des_ = msg->data;
      have_des_time_ = true;
    }


    bool BallCatcherController::homingHandler(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
      ROS_WARN_STREAM("Start Homing");
      state_ = INITIALIZED;
      return true;
    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
