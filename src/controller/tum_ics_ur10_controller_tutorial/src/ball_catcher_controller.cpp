#include <tum_ics_ur10_controller_tutorial/ball_catcher_controller.h>

#include <ow_core/common/parameter.h>

#include <nav_msgs/Path.h>
#include <ow_core/math.h>
#include <Eigen/Geometry>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PointStamped.h>

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
      ee_target_sub_ = nh_.subscribe("ee_target", 1, &BallCatcherController::eeTargetCallback, this);
      homing_srv_ = nh_.advertiseService("homing", &BallCatcherController::homingHandler, this);
      controller_hb_pub_ = nh_.advertise<std_msgs::Empty>("controller_heartbeat", 1);
      controller_ee_pub_ = nh_.advertise<geometry_msgs::PointStamped>("controller_ee", 1);

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
      return true;
    }

    bool BallCatcherController::start()
    {
      ROS_WARN_STREAM("BallCatcherController::start");
      return true;
    }

    void BallCatcherController::startLinearInterpolation(const ow::CartesianPosition &goal, double duration, const JointState &current)
    {
      const Eigen::Affine3d T0 = model_.T_tool_0(current.q);
      X_start_ = ow::CartesianPosition(T0);
      p0_ = T0.translation();  
      p1_ = goal.head<3>();      // goal position (already in base frame)
      dp_ = p1_ - p0_;
      const Eigen::Matrix3d R0 = T0.rotation();
      q0_ = Eigen::Quaterniond(T0.linear()).normalized();
      //ROS_WARN_STREAM("q0_ = " << "w: " << q0_.w() << ", x: " << q0_.x() << ", y: " << q0_.y() << ", z: " << q0_.z());
      // goal layout is [px, py, pz, qx, qy, qz, qw]
      const double qx = goal(3);
      const double qy = goal(4);
      const double qz = goal(5);
      const double qw = goal(6);
      q1_ = Eigen::Quaterniond(qw, qx, qy, qz).normalized();  // w,x,y,z
      if (q0_.dot(q1_) < 0.0) {
        q1_.coeffs() *= -1.0;   // flip sign of q1_ to avoid the π jump
      }
      // ---------- Feasibility checks ----------
      // 1) Angular speed: angle / T <= omega_max  ->  T >= angle / omega_max
      Eigen::Quaterniond q_rel = q0_.conjugate() * q1_;
      q_rel.normalize();
      Eigen::AngleAxisd aa(q_rel);
      const double angle = std::abs(aa.angle());                       // [rad]
      const double T_min_angle = (angle > 1e-6) ? angle / omega_max_rad_ : 0.0;

      // 2) Linear speed: ||dp|| / T <= v_max      ->  T >= ||dp|| / v_max   (optional but useful)
      const double dist = dp_.norm();
      const double T_min_pos = dist / std::max(v_max_mps_, 1e-6);

      // Requested T (duration) may come from Tgo; stretch if needed
      const double T_req = std::max(1e-6, duration);
      /*
      cs_spline_duration_ = std::max({T_req, T_min_angle, T_min_pos});
      if (cs_spline_duration_ > T_req + 1e-6) {
        ROS_ERROR_STREAM("Stretching T: req=" << T_req
                        << "  →  used=" << cs_spline_duration_
                        << " (T_min_angle=" << T_min_angle
                        << ", T_min_pos=" << T_min_pos << ")");
      }
      */
      duration_ = 0.0;
      cs_spline_duration_ = T_req;
      start_interpolation = true;
    }

    Vector6d BallCatcherController::update(const RobotTime &time, const JointState &current)
    {
      state_ = transitState(current);

      ROS_WARN_STREAM_THROTTLE(0.5, "state: " << state_);

      model_.broadcastFrames(current.q, ros::Time::now());
      ow::CartesianPosition X_ee = ow::CartesianPosition(model_.T_tool_0(current.q));
      ROS_INFO_STREAM_THROTTLE(0.5, "X_ee_w: " << X_ee.transpose());

      //ROS_INFO_STREAM("VELOCITY: " << current.qp.transpose());
      //ROS_INFO_STREAM("ACCELERATION: " << current.qpp.transpose());

      // dt
      ros::Time t = ros::Time::now();
      ros::Duration period = prev_time_.isZero() ? ros::Duration(0.0) : (t - prev_time_);
      prev_time_ = t;
      //ROS_INFO_STREAM("Control cycle: " << period);

      if (state_ == JOINT_SPLINE)
      {
        ros::Duration elapses = t - joint_spline_t_start_;
        ow::JointState js_ref = joint_spline_->evaluate(elapses.toSec());
        tau_ = joint_space_controller(current, js_ref, period);
      }

      if (state_ == IDLE)
      {
        ow::CartesianState cs_ref;
        cs_ref.pos() = X_goal_;
        cs_ref.vel().setZero();
        cs_ref.acc().setZero();
        tau_ = cartesian_space_controller(current, cs_ref, period);
      }

      if (state_ == CARTESIAN_SPLINE)
      {
        ow::CartesianState cs_ref;
        // -- Time and progress scalars --
        const double dt = period.toSec();
        duration_ += dt;

        const double T = std::max(1e-6, cs_spline_duration_);
        double s = duration_ / T;
        if (s > 1.0) s = 1.0;
        const double sdot = (s < 1.0) ? (1.0 / T) : 0.0;
        // const double sddot = 0.0; // constant-speed profile

        // -- Linear position interpolation --
        const Eigen::Vector3d p_ref = p0_ + s * dp_;
        const Eigen::Vector3d v_ref = sdot * dp_;

        // -- Orientation interpolation (SLERP) --
        const Eigen::Quaterniond q_ref = q0_.slerp(s, q1_);

        // Angular velocity: first-order along shortest rotation
        Eigen::Quaterniond q_rel = q0_.conjugate() * q1_;
        q_rel.normalize();
        Eigen::AngleAxisd aa(q_rel);
        //ROS_WARN_STREAM("Angle= " << aa.angle() << " rad");
        const double ang = std::abs(aa.angle());
        Eigen::Vector3d phi;
        if (ang > 1e-9) {
            phi = aa.axis() * ang;
        } else {
            phi = Eigen::Vector3d::Zero();
        }
        //ROS_WARN_STREAM("Rot Vector= " << phi);
        const Eigen::Vector3d w_ref = sdot * phi;               // approx spatial ω

        // -- Build reference pose and twist --
        Eigen::Affine3d T_ref = Eigen::Affine3d::Identity();
        T_ref.linear()      = q_ref.toRotationMatrix();
        T_ref.translation() = p_ref;

        cs_ref.pos() = ow::CartesianPosition(T_ref);

        Vector6d Xp_ref;
        Xp_ref << v_ref, w_ref;
        cs_ref.vel() = ow::CartesianVelocity(Xp_ref);

        cs_ref.acc().setZero(); // constant-speed straight line

        // -- Stop generating waypoints once we reach the end of the segment --
        if (s >= 1.0) {
          start_interpolation = false;
        }

        // Run the controller on this interpolated reference
        tau_ = cartesian_space_controller(current, cs_ref, period);
        //ROS_INFO_STREAM("VELOCITAAAT: " << current.qp.transpose());
      }
      std_msgs::Empty hb;
      controller_hb_pub_.publish(hb);
      geometry_msgs::PointStamped ee_pos;
      ee_pos.header.stamp = t;
      ee_pos.header.frame_id = "ur10_model_dh_5";
      ee_pos.point.x = X_ee(0);
      ee_pos.point.y = X_ee(1);
      ee_pos.point.z = X_ee(2);
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

      if ((state_ == IDLE || state_ == CARTESIAN_SPLINE) && start_cartesian_spline_)
      {
        startLinearInterpolation(X_goal_, cs_spline_duration_, current);
        start_cartesian_spline_ = false;
        next_state = CARTESIAN_SPLINE;
        state_changed = true;
      }

      if (state_ == CARTESIAN_SPLINE)
      {
        static ros::Time start_time = ros::Time::now();  // initialize once per state entry
        static bool first_call = true;

        // ---- Get Current Time ----
        ros::Time current_time = ros::Time::now();

        // ---- Compute delta since start ----
        double delta_time = (current_time - start_time).toSec();

        // ---- Reset timer when entering the state (first iteration) ----
        if (first_call) {
          start_time = current_time;
          delta_time = 0.0;
          first_call = false;
        }
        ow::CartesianPosition X = ow::CartesianPosition(model_.T_tool_0(current.q));
        ow::Matrix6 J = model_.J_tool_0(current.q);
        ow::CartesianVelocity V = ow::CartesianVelocity(J * current.qp);
        ///////////////////////////////////////////
        Vector6d dx = ow::cartesianError(X, X_goal_);
        //////////////////////////////////////////////////
        double pos_err = dx.head<3>().norm();
        //ROS_INFO_STREAM_THROTTLE(0.1, "ERROR: " << pos_err);
        double lin_spd = V.head<3>().norm();
        //ROS_INFO_STREAM_THROTTLE(0.1, "VEL_MES: " << lin_spd);
        double ang_err = dx.tail<3>().norm();   // radians of orientation error
        //ROS_INFO_STREAM_THROTTLE(0.1, "ANGLE: " << ang_err);
        //ROS_INFO_STREAM_THROTTLE(0.1, "Δt: " << delta_time << " s");
        //ROS_INFO_STREAM("ERROR: " << pos_err << "m ANGLE: " << ang_err << "rad Δt: " << delta_time << " s");
        if (pos_err < 0.01 && lin_spd < 0.1 && ang_err < 0.05) { 
          ROS_WARN_STREAM("PARA IDLE");
          //ROS_WARN_STREAM("ERROR: " << pos_err);
          //ROS_WARN_STREAM("SPEED: " << lin_spd);
          //ROS_WARN_STREAM("ANGLE: " << ang_err);
          i_delta_x_.setZero();
          next_state = IDLE;
          start_interpolation = false;
          state_changed = true;
        }
      }

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
      js_r.qpp = ref.qPP() - Kp_j_ * qp_delta;

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
      //ROS_ERROR_STREAM("Velocidad_current: " << cs_current.vel().transpose());
      //ROS_ERROR_STREAM("Velocidad_ref: " << ref.vel().transpose());
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
      constexpr double QDOT_MAX = 2.09;  // almost 180 deg/s in rad/s
      for (int i = 0; i < 6; ++i) {
        if (js_r.qp[i] >  QDOT_MAX) js_r.qp[i] =  QDOT_MAX;
        if (js_r.qp[i] < -QDOT_MAX) js_r.qp[i] = -QDOT_MAX;
      }
      js_r.qpp = J_tool_0.inverse() * (Xpp_r - Jp_tool_0 * js_r.qp);
      constexpr double QDDOT_MAX = 4.4; // rad/s^2 
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

    /*
    void BallCatcherController::start_cartesian_spline(const JointState &current, const ow::CartesianPosition &goal, double spline_duration)
    {
      ow::CartesianPosition X_current = ow::CartesianPosition(model_.T_tool_0(current.q));

      ROS_WARN_STREAM("New Cartesian Spline");
      ROS_WARN_STREAM("start: " << X_current.transpose());
      ROS_WARN_STREAM("goal: " << goal.transpose());
      ROS_WARN_STREAM("duration: " << spline_duration);

      // create a cartesian spline
      cartesian_spline_ = std::make_unique<ow::CartesianStateTrajectory>(ow::CartesianPolynomial(spline_duration, X_current, goal));
      //cartesian_spline_ = std::make_unique<ow::CartesianStateTrajectory>(ow::Polynomial5Order<ow::CartesianPosition>(period, X_start, zero, zero, X_end, zero, zero));
      cartesian_spline_t_start_ = ros::Time::now();

    }
    */

    void BallCatcherController::eeTargetCallback(const tum_ics_ur10_controller_tutorial::EETargetConstPtr &msg)
    { 
      ow::CartesianPosition X_goal_w;
      X_goal_w = msg->ee_target;
      ow::CartesianPosition X_new = ow::CartesianPosition(model_.T_0_B()) * X_goal_w; // to base frame

      const double POS_EPS = 0.002;     // 2 mm
      //const double ANG_EPS = 0.25 * M_PI/180.0; // 1 deg in rad
      const double ANG_EPS = 0.0;

      bool should_update = true;
      if (has_goal_ &&  (state_ == CARTESIAN_SPLINE || state_ == IDLE)) {
        Eigen::Vector3d p_old = X_goal_.head<3>();
        Eigen::Vector3d p_new = X_new.head<3>();
        const double dp = (p_new - p_old).norm();

        // Orientation distance via quaternion angle
        Eigen::Quaterniond q_old(X_goal_(6), X_goal_(3), X_goal_(4), X_goal_(5));
        Eigen::Quaterniond q_new(X_new(6),   X_new(3),   X_new(4),   X_new(5));
        if (q_old.dot(q_new) < 0.0) q_new.coeffs() *= -1.0;
        Eigen::AngleAxisd aa(q_old.conjugate() * q_new);
        const double dtheta = std::abs(aa.angle());
        should_update = (dp > POS_EPS) || (dtheta > ANG_EPS);
      }

      if (should_update) {
        X_goal_ = X_new;
        cs_spline_duration_ = msg->duration;
        start_cartesian_spline_ = true;
        has_goal_ = true;
        ROS_ERROR_STREAM("NEW EE GOAL: " << X_goal_.transpose() << "  TtoGo: " << cs_spline_duration_ << " s");
      } else {
        // No restart; just update the stored goal quietly (optional)
        X_goal_ = X_new;
      }
    }

    bool BallCatcherController::homingHandler(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
      ROS_WARN_STREAM("Start Homing");
      state_ = INITIALIZED;
      return true;
    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
