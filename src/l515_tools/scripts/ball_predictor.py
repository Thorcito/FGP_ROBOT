#!/usr/bin/env python3
"""
BallKFPredictor (plane-interception criterion)

Purpose
-------
- Consume 3D ball measurements in the *camera optical frame*.
- Run a constant-acceleration Kalman Filter (state: position + velocity).
- Apply robust measurement handling with Mahalanobis gating (soft/hard).
- Predict the ballistic trajectory under gravity expressed in the camera frame.
- Compute intercept with a chosen plane ("x" -> YZ plane at x=const, "z" -> XY plane at z=const).
- Publish filtered states, predicted future points/path, plane-hit point/time, and diagnostics.
- (Optional) Log CSV rows per update (currently disabled by setting csv_path=None on purpose).

Inputs
------
/ball_meas/point : geometry_msgs/PointStamped (X,Y,Z in camera frame)

Outputs (selected)
------------------
/ball_pred/point_filt    : geometry_msgs/PointStamped (filtered position)
/ball_pred/pred_point    : geometry_msgs/PointStamped (lookahead prediction)
/ball_pred/pred_path     : nav_msgs/Path              (future ballistic samples)
/ball_pred/path_filt     : nav_msgs/Path              (history of filtered positions)
/ball_pred/hit_point     : geometry_msgs/PointStamped (intercept on selected plane)
/ball_pred/hit_time_s    : std_msgs/Float32           (time-to-plane along ballistic path)
/ball_pred/hit_history   : nav_msgs/Path              (recent hits for RViz)
/ball_pred/static_plane  : visualization_msgs/Marker  (plane visualization)
/ball_pred/current_vel   : geometry_msgs/Vector3Stamped (KF velocity at current time)
/ball_pred/pred_vel_hit  : geometry_msgs/Vector3Stamped (velocity at intercept time)

Key params (~private)
---------------------
~frame_id                : output frame id (default "camera_color_optical_frame")
~input_topic             : input measurement topic (default "/ball_meas/point")
~g_cam                   : gravity in camera frame (default [0, 9.81, 0])
~sigma_pos_xyz           : per-axis measurement std dev [m]
~sigma_acc_xyz           : per-axis process (acceleration) std dev [m/s^2]
~init_pos_std_m          : init position std dev [m]
~init_vel_std_mps        : init velocity std dev [m/s]
~lookahead_s             : time ahead for single predicted point [s]
~pred_horizon_s          : total prediction horizon [s]
~pred_dt_s               : sampling step for pred_path [s]
~min_dt_s, ~max_dt_s     : clamp dt between updates
~keep_filtered_path      : enable filtered path publication
~filtered_path_max       : max stored filtered poses
~meas_gate_*             : gating thresholds (soft/hard) and warmup counters
~plane_mode              : "x" (YZ plane) or "z" (XY plane)
~x_plane_m, ~z_plane_m   : plane positions (meters, in camera frame)
~hit_horizon_s           : max allowed intercept time [s]
~hit_history_max         : stored hits for RViz
~csv_path                : CSV output path (ignored when set to empty/None)

Notes
-----
- All kinematics/planes live in the *camera* frame. If you need world/base, do it downstream.
- CSV logging is explicitly disabled below via self.csv_path = None (kept intentionally).
"""

import rospy, numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped
from nav_msgs.msg import Path
from std_msgs.msg import Header, Float32
from visualization_msgs.msg import Marker
import csv, os
from datetime import datetime

class BallKFPredictor:
    def __init__(self):
        # ---------- params ----------
        self.frame_id      = rospy.get_param("~frame_id", "camera_color_optical_frame")
        self.input_topic   = rospy.get_param("~input_topic", "/ball_meas/point")

        # gravity in CAMERA frame (+Y is down for your camera)
        # Example: for RealSense L515 mounted with +Y downward in optical frame.
        self.g_cam = np.array(rospy.get_param("~g_cam", [0.0, 9.81, 0.0]), dtype=np.float64)

        # per-axis measurement / process (accel) noise
        # R = diag(sigma_pos_xyz^2), Q derived from sigma_acc_xyz via continuous white-noise model.
        self.sigma_pos_xyz = np.array(rospy.get_param("~sigma_pos_xyz", [0.1, 0.007, 0.07]), dtype=np.float64)
        self.sigma_acc_xyz = np.array(rospy.get_param("~sigma_acc_xyz", [8.0, 8.0, 6.0]), dtype=np.float64)

        # proper initialization std devs (used only at first detection)
        self.init_pos_std  = float(rospy.get_param("~init_pos_std_m", 0.02))
        self.init_vel_std  = float(rospy.get_param("~init_vel_std_mps", 2.0))

        # prediction outputs
        self.lookahead_s    = float(rospy.get_param("~lookahead_s", 0.10))
        self.pred_horizon_s = float(rospy.get_param("~pred_horizon_s", 1.0))
        self.pred_dt_s      = float(rospy.get_param("~pred_dt_s", 0.02))

        self.min_dt_s = float(rospy.get_param("~min_dt_s", 1e-4))
        self.max_dt_s = float(rospy.get_param("~max_dt_s", 0.20)) 

        # filtered path buffer
        self.keep_filtered_path = bool(rospy.get_param("~keep_filtered_path", True))
        self.filtered_path_max  = int(rospy.get_param("~filtered_path_max", 50))
        self.filtered_path = []

        # ------ Mahalanobis gating params ------
        # Chi-square thresholds for 3 DoF innovations (position-only):
        # ~11.34 ≈ 99%, ~16.27 ≈ 99.9%.
        self.meas_gate_chi2_soft = float(rospy.get_param("~meas_gate_chi2_soft", 11.34))  # 3DoF ~99%
        self.meas_gate_chi2_hard = float(rospy.get_param("~meas_gate_chi2_hard", 16.27))  # 3DoF ~99.9%
        self.meas_soft_factor_max = float(rospy.get_param("~meas_soft_factor_max", 15.0))
        self.meas_min_abs_jump_m = float(rospy.get_param("~meas_min_abs_jump_m", 0.15))
        self.gating_warmup_updates = int(rospy.get_param("~gating_warmup_updates", 3))

        # ---- plane-intersection params ----
        # "x" -> intersect YZ plane at x = x_plane_m
        # "z" -> intersect XY plane at z = z_plane_m
        self.plane_mode = rospy.get_param("~plane_mode", "z")   # "x" | "z"
        self.x_plane_m  = float(rospy.get_param("~x_plane_m", 0.7))  # YZ plane at x = const 
        self.z_plane_m  = float(rospy.get_param("~z_plane_m", 0.9))  # XY plane at z = const 0.5 is the min reading
        self.exp_tag = float(rospy.get_param("~exp_tag", 10))
        self.hit_horizon_s = float(rospy.get_param("~hit_horizon_s", 1.3))

        # ---- hit history for RViz ----
        self.hit_history = []
        self.hit_history_max = int(rospy.get_param("~hit_history_max", 20))

        # ---------- state ----------
        # x = [px,py,pz, vx,vy,vz]^T
        self.x = None
        self.P = None
        self.t_last = None

        # gating state
        self.z_prev = None              # last accepted measurement
        self.updates_since_init = 0     # to handle warm-up

        # ---------- pubs/sub ----------
        self.pub_filt_pt   = rospy.Publisher("/ball_pred/point_filt", PointStamped, queue_size=10)
        self.pub_pred_pt   = rospy.Publisher("/ball_pred/pred_point", PointStamped, queue_size=10)
        self.pub_pred_path = rospy.Publisher("/ball_pred/pred_path", Path, queue_size=10)
        self.pub_filt_path = rospy.Publisher("/ball_pred/path_filt", Path, queue_size=10)
        self.pub_hit_point = rospy.Publisher("/ball_pred/hit_point", PointStamped, queue_size=10)
        self.pub_hit_time  = rospy.Publisher("/ball_pred/hit_time_s", Float32, queue_size=10)
        self.pub_hit_history = rospy.Publisher("/ball_pred/hit_history", Path, queue_size=10)
        self.pub_markers_static = rospy.Publisher("/ball_pred/static_plane", Marker, queue_size=1, latch=True)
        self.pub_current_vel = rospy.Publisher("/ball_pred/current_vel", Vector3Stamped, queue_size=10)
        self.pub_pred_vel_hit = rospy.Publisher("/ball_pred/pred_vel_hit", Vector3Stamped, queue_size=10)

        # diagnostics for PlotJuggler
        self.pub_innov_x = rospy.Publisher("/ball_kf/innov_x", Float32, queue_size=20)
        self.pub_innov_y = rospy.Publisher("/ball_kf/innov_y", Float32, queue_size=20)
        self.pub_innov_z = rospy.Publisher("/ball_kf/innov_z", Float32, queue_size=20)
        self.pub_vx = rospy.Publisher("/ball_kf/vx", Float32, queue_size=20)
        self.pub_vy = rospy.Publisher("/ball_kf/vy", Float32, queue_size=20)
        self.pub_vz = rospy.Publisher("/ball_kf/vz", Float32, queue_size=20)
        self.pub_sigma_px = rospy.Publisher("/ball_kf/sigma_px", Float32, queue_size=20)
        self.pub_sigma_py = rospy.Publisher("/ball_kf/sigma_py", Float32, queue_size=20)
        self.pub_sigma_pz = rospy.Publisher("/ball_kf/sigma_pz", Float32, queue_size=20)
        self.pub_maha_d2  = rospy.Publisher("/ball_kf/maha_d2", Float32, queue_size=20)

        rospy.Subscriber(self.input_topic, PointStamped, self.cb_meas, queue_size=50)

        rospy.loginfo("BallKFPredictor: frame=%s, g_cam=%s, lookahead=%.2fs, horizon=%.2fs@%.0fHz",
                      self.frame_id, self.g_cam.tolist(), self.lookahead_s,
                      self.pred_horizon_s, 1.0/self.pred_dt_s)

        # prebuild constant H and R
        self.H = np.zeros((3,6)); self.H[0,0]=self.H[1,1]=self.H[2,2]=1.0
        self.R = np.diag(self.sigma_pos_xyz**2)
        self.publish_static_plane_markers()

        # ---------- CSV logging ----------
        # If it's relative, it will be created relative to the process CWD.
        csv_param = rospy.get_param("~csv_path", "~/Desktop/Robo_Project_ws/src/l515_tools/scripts/Predictions.csv")
        self.csv_path = os.path.expanduser(csv_param)
        self.csv_path = None #done on purpose to dont store data
        self._csv_header_written = False

        # Log where we're writing and from which CWD (helps when using roslaunch)
        try:
            cwd_now = os.getcwd()
        except Exception:
            cwd_now = "<unavailable>"
        rospy.loginfo("CSV CWD: %s", cwd_now)
        rospy.loginfo("CSV path (user-specified): %s", self.csv_path)

        if self.csv_path:
            d = os.path.dirname(self.csv_path)
            if d:
                try:
                    os.makedirs(d, exist_ok=True)
                except Exception as e:
                    rospy.logwarn("CSV: failed to create dir '%s': %s", d, e)
            if os.path.exists(self.csv_path):
                self._csv_header_written = True
            rospy.loginfo("CSV logging enabled -> %s", self.csv_path)
        else:
            rospy.logwarn("CSV logging disabled: ~csv_path is empty")



    # ---------- model helpers ----------
    def F_Q_gvec(self, dt):
        """
        Build discrete-time transition F, process covariance Q for dt,
        and deterministic input due to gravity g_vec for constant-acceleration model.

        State order: [px, py, pz, vx, vy, vz]^T

        F:
            [ I3  dt*I3 ]
            [ 0     I3  ]

        Q (per-axis white-noise acceleration q = sigma_acc^2):
            For each axis: q * [[dt^3/3, dt^2/2],
                                [dt^2/2, dt    ]]
            mapped into (p, v) block for that axis.

        g_vec (gravity contribution):
            [ 0.5*g*dt^2 ; g*dt ]
        """
        F = np.eye(6)
        F[0,3]=dt; F[1,4]=dt; F[2,5]=dt
        dt2, dt3 = dt*dt, dt*dt*dt

        # per-axis continuous white-noise accel
        Q = np.zeros((6,6))
        for i in range(3):
            q = self.sigma_acc_xyz[i]**2
            Q1 = q * np.array([[dt3/3.0, dt2/2.0],
                               [dt2/2.0, dt      ]], dtype=np.float64)
            ii, vv = i, 3+i
            Q[ii,ii]+=Q1[0,0]; Q[ii,vv]+=Q1[0,1]
            Q[vv,ii]+=Q1[1,0]; Q[vv,vv]+=Q1[1,1]

        g = self.g_cam
        g_vec = np.array([0.5*g[0]*dt2, 0.5*g[1]*dt2, 0.5*g[2]*dt2,
                          g[0]*dt,      g[1]*dt,      g[2]*dt     ], dtype=np.float64)
        return F, Q, g_vec

    # ---------- predict / update ----------
    def predict_step(self, dt):
        """Time-update (prediction) with gravity input."""
        F, Q, g_vec = self.F_Q_gvec(dt)
        self.x = F.dot(self.x) + g_vec
        self.P = F.dot(self.P).dot(F.T) + Q

    def update_step(self, z):
        """
        Measurement-update (correction) with Mahalanobis gating.

        Args:
            z (np.ndarray shape=(3,)): position measurement [mx,my,mz]

        Returns:
            accepted_meas (bool): True if measurement used, False if hard-rejected.
            d2 (float): Mahalanobis distance squared for diagnostics.

        Logic:
        - Compute innovation y = z - Hx and innovation covariance S = HPHᵀ + R.
        - d2 = yᵀ S⁻¹ y. If large:
            * If above hard threshold (and jump condition met), reject.
            * Else above soft threshold, inflate R by factor to down-weight update.
        - Compute Kalman gain K = P Hᵀ S⁻¹ and update (x,P).
        - Publish innovation/uncertainty diagnostics on accepted updates.
        """
        # innovation
        y = z - self.H.dot(self.x)
        S = self.H.dot(self.P).dot(self.H.T) + self.R

        # Mahalanobis distance^2
        Sinv_y = np.linalg.solve(S, y)          # S * x = y  -> x = S⁻¹ y
        d2 = float(y.dot(Sinv_y))
        self.pub_maha_d2.publish(Float32(data=d2))

        # ---- GATING LOGIC ----
        do_gate = (self.updates_since_init >= self.gating_warmup_updates)

        R_eff = self.R
        if do_gate:
            jump_ok = False
            if self.z_prev is not None:
                jump_norm = float(np.linalg.norm(z - self.z_prev))
                jump_ok = (jump_norm >= self.meas_min_abs_jump_m)

            if d2 > self.meas_gate_chi2_hard and (self.z_prev is None or jump_ok):
                rospy.logwarn_throttle(1.0, f"KF: hard-rejected meas (d2={d2:.2f}, jump={jump_ok})")
                return False, d2
            elif d2 > self.meas_gate_chi2_soft:
                factor = min(self.meas_soft_factor_max, d2 / self.meas_gate_chi2_soft)
                R_eff = self.R * factor
                S = self.H.dot(self.P).dot(self.H.T) + R_eff

        # K = P Hᵀ S⁻¹  (solve instead of invert)
        K = self.P.dot(self.H.T)
        K = np.linalg.solve(S.T, K.T).T  # (Sᵀ \ (P Hᵀ))ᵀ

        # posterior
        self.x = self.x + K.dot(y)
        I = np.eye(6)
        self.P = (I - K.dot(self.H)).dot(self.P)

        # diagnostics (only meaningful on accepted updates)
        self.pub_innov_x.publish(Float32(data=float(y[0])))
        self.pub_innov_y.publish(Float32(data=float(y[1])))
        self.pub_innov_z.publish(Float32(data=float(y[2])))
        self.pub_vx.publish(Float32(data=float(self.x[3])))
        self.pub_vy.publish(Float32(data=float(self.x[4])))
        self.pub_vz.publish(Float32(data=float(self.x[5])))
        self.pub_sigma_px.publish(Float32(data=float(np.sqrt(self.P[0,0]))))
        self.pub_sigma_py.publish(Float32(data=float(np.sqrt(self.P[1,1]))))
        self.pub_sigma_pz.publish(Float32(data=float(np.sqrt(self.P[2,2]))))

        # remember accepted measurement
        self.z_prev = z.copy()
        self.updates_since_init += 1

        return True, d2
    
    def ballistic_pos(self, dt):
        """Ballistic propagation under constant gravity in camera frame."""
        p0 = self.x[0:3]; v0 = self.x[3:6]; a = self.g_cam
        return p0 + v0*dt + 0.5*a*(dt*dt)

    @staticmethod
    def solve_time_to_plane_1d(a, b, c, t_min=0.0, t_max=None, eps=1e-9):
        """
        Solve a*t^2 + b*t + c = 0 for the earliest valid root in [t_min, t_max].
        Handles degenerate linear/constant cases and negative discriminant.

        Returns:
            t (float or None): earliest feasible intercept time.
        """
        # earliest admissible root in [t_min, t_max]
        if abs(a) < eps:
            if abs(b) < eps:
                if abs(c) < eps:
                    t = 0.0
                else:
                    return None
            else:
                t = -c / b
            if t < t_min or (t_max is not None and t > t_max):
                return None
            return float(t)
        D = b*b - 4.0*a*c
        if D < 0.0:
            return None
        sqrtD = np.sqrt(D)
        t1 = (-b - sqrtD) / (2.0*a)
        t2 = (-b + sqrtD) / (2.0*a)
        candidates = []
        for t in (t1, t2):
            if t >= t_min and (t_max is None or t <= t_max):
                candidates.append(float(t))
        if not candidates:
            return None
        return min(candidates)

    # ---------- single inline publisher ----------
    def publish_all(self, stamp, accepted_meas: bool):
        """
        Publish all outputs using the current KF state.

        Behavior:
        - Always publish the current velocity and the lookahead predicted point/path.
        - Publish filtered point and filtered path only when the measurement was accepted.
        - Compute intercept with both planes, choose based on self.plane_mode, and publish hit point/time.
        - Maintain a small history of hit points for RViz inspection.
        """
        hdr = Header(stamp=stamp, frame_id=self.frame_id)

        # 1)Velocity of the ball
        vel = Vector3Stamped()
        vel.header = hdr
        vel.vector.x = self.x[3]
        vel.vector.y = self.x[4]
        vel.vector.z = self.x[5]
        self.pub_current_vel.publish(vel)

        # 2) Filtered (only if accepted)
        if accepted_meas:
            pt_f = PointStamped()
            pt_f.header = hdr
            pt_f.point.x, pt_f.point.y, pt_f.point.z = self.x[0], self.x[1], self.x[2]
            self.pub_filt_pt.publish(pt_f)

            if self.keep_filtered_path:
                pose = PoseStamped(); pose.header = hdr
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pt_f.point.x, pt_f.point.y, pt_f.point.z
                pose.pose.orientation.w = 1.0
                self.filtered_path.append(pose)
                if len(self.filtered_path) > self.filtered_path_max:
                    self.filtered_path = self.filtered_path[-self.filtered_path_max:]
                path_f = Path(); path_f.header = hdr; path_f.poses = self.filtered_path
                self.pub_filt_path.publish(path_f)

        # 3) Prediction point (lookahead) + prediction path
        pL = self.ballistic_pos(self.lookahead_s)
        pt_p = PointStamped()
        pt_p.header = hdr
        pt_p.point.x, pt_p.point.y, pt_p.point.z = float(pL[0]), float(pL[1]), float(pL[2])
        self.pub_pred_pt.publish(pt_p)

        path = Path(); path.header = hdr
        n = max(2, int(self.pred_horizon_s / self.pred_dt_s))
        for i in range(n+1):
            dt = i * self.pred_dt_s
            p = self.ballistic_pos(dt)
            ps = PoseStamped(); ps.header = hdr
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(p[0]), float(p[1]), float(p[2])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_pred_path.publish(path)

        # 4) Plane hits (compute both, then select by mode)
        p0 = self.x[0:3]; v0 = self.x[3:6]; g = self.g_cam

        # YZ plane (x = const)
        a_x = 0.5 * g[0]; b_x = v0[0]; c_x = p0[0] - self.x_plane_m
        t_yz = self.solve_time_to_plane_1d(a_x, b_x, c_x, t_min=0.0, t_max=self.hit_horizon_s)
        p_yz = p0 + v0*t_yz + 0.5*g*(t_yz*t_yz) if t_yz is not None else None

        # XY plane (z = const)
        a_z = 0.5 * g[2]; b_z = v0[2]; c_z = p0[2] - self.z_plane_m
        t_xy = self.solve_time_to_plane_1d(a_z, b_z, c_z, t_min=0.0, t_max=self.hit_horizon_s)
        p_xy = p0 + v0*t_xy + 0.5*g*(t_xy*t_xy) if t_xy is not None else None

        mode = (self.plane_mode).lower()
        if mode == "x":
            t_sel, p_sel = t_yz, p_yz
        else:  # "z"
            t_sel, p_sel = t_xy, p_xy

        if (t_sel is not None) and (p_sel is not None):
            pt_h = PointStamped()
            pt_h.header = hdr
            pt_h.point.x, pt_h.point.y, pt_h.point.z = float(p_sel[0]), float(p_sel[1]), float(p_sel[2])
            self.pub_hit_point.publish(pt_h)
            self.pub_hit_time.publish(Float32(data=float(t_sel)))
            rospy.loginfo_throttle(0.0, f"Time to intercept: (t={t_sel:.6f})")

            pose_h = PoseStamped(); pose_h.header = hdr
            pose_h.pose.position.x = pt_h.point.x
            pose_h.pose.position.y = pt_h.point.y
            pose_h.pose.position.z = pt_h.point.z
            pose_h.pose.orientation.w = 1.0
            self.hit_history.append(pose_h)
            if len(self.hit_history) > self.hit_history_max:
                self.hit_history = self.hit_history[-self.hit_history_max:]

            path_h = Path(); path_h.header = hdr
            path_h.poses = self.hit_history
            self.pub_hit_history.publish(path_h)

            
            vel_hit = Vector3Stamped()
            vel_hit.header = hdr
            v0 = self.x[3:6]
            g = self.g_cam
            v_hit = v0 + g * t_sel
            vel_hit.vector.x = v_hit[0]
            vel_hit.vector.y = v_hit[1]
            vel_hit.vector.z = v_hit[2]
            self.pub_pred_vel_hit.publish(vel_hit)
            

            return t_sel, p_sel
        else:
            return None, None

    # ---------- callback ----------
    def cb_meas(self, msg: PointStamped):
        """
        Handle incoming measurement:
        - Initialize the KF on the first message.
        - Thereafter, compute dt, run predict+update (with gating), and publish outputs.
        - Optionally log a CSV row per update (currently disabled).
        """
        t_cb_start = rospy.Time.now()


        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            rospy.logwarn_throttle(1.0, "Incoming frame %s != %s, treating as camera frame",
                                   msg.header.frame_id, self.frame_id)

        t = msg.header.stamp.to_sec()
        z_meas = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)

        if self.x is None:
            # initialize
            self.x = np.zeros(6); self.x[0:3] = z_meas
            self.P = np.diag([self.init_pos_std**2]*3 + [self.init_vel_std**2]*3)
            self.t_last = t
            self.z_prev = z_meas.copy()
            self.updates_since_init = 0
            # unified publishing on first detection (treat as accepted)
            t_hit, p_hit = self.publish_all(msg.header.stamp, accepted_meas=True)

            # CSV row for first frame
            if self.csv_path:
                self.write_csv(msg.header.stamp, z_meas, True, p_hit, t_hit)
            return

        dt_raw = t - self.t_last
        if dt_raw <= 0.0:
            return
        dt = float(np.clip(dt_raw, self.min_dt_s, self.max_dt_s))
        self.t_last = t

        self.predict_step(dt)
        accepted, d2 = self.update_step(z_meas)
        t_pub = rospy.Time.now()
        latency_proc = (t_pub - t_cb_start).to_sec()
        rospy.loginfo(f"Latency: ({latency_proc}) s")
        t_hit, p_hit = self.publish_all(msg.header.stamp, accepted)

        if self.csv_path:
            self.write_csv(msg.header.stamp, z_meas, accepted, p_hit, t_hit)

    # ---------- static plane markers (unchanged except color range) ----------
    def publish_static_plane_markers(self):
        """
        Publish a translucent CUBE marker representing the active intercept plane.
        Only the plane matching self.plane_mode is published (latching publisher).
        """
        PLANE_THICKNESS = 0.005
        YZ_SIZE_Y = 4.0
        YZ_SIZE_Z = 6.0
        XY_SIZE_X = 6.0
        XY_SIZE_Y = 4.0
        PLANE_ALPHA = 0.3

        now = rospy.Time.now()

        if self.plane_mode == "x":
            yz = Marker()
            yz.header.frame_id = self.frame_id
            yz.header.stamp = now
            yz.ns = "planes"; yz.id = 1
            yz.type = Marker.CUBE; yz.action = Marker.ADD
            yz.pose.orientation.w = 1.0
            yz.pose.position.x = float(self.x_plane_m)
            yz.pose.position.y = 0.0
            yz.pose.position.z = 0.0
            yz.scale.x = max(PLANE_THICKNESS, 1e-4)
            yz.scale.y = YZ_SIZE_Y
            yz.scale.z = YZ_SIZE_Z
            yz.color.r, yz.color.g, yz.color.b, yz.color.a = (1.0, 1.0, 0.0, PLANE_ALPHA)
            self.pub_markers_static.publish(yz)

        if self.plane_mode == "z":
            xy = Marker()
            xy.header.frame_id = self.frame_id
            xy.header.stamp = now
            xy.ns = "planes"; xy.id = 2
            xy.type = Marker.CUBE; xy.action = Marker.ADD
            xy.pose.orientation.w = 1.0
            xy.pose.position.x = 0.0
            xy.pose.position.y = 0.0
            xy.pose.position.z = float(self.z_plane_m)
            xy.scale.x = XY_SIZE_X
            xy.scale.y = XY_SIZE_Y
            xy.scale.z = max(PLANE_THICKNESS, 1e-4)
            xy.color.r, xy.color.g, xy.color.b, xy.color.a = (1.0, 1.0, 0.0, PLANE_ALPHA)
            self.pub_markers_static.publish(xy)
    
    def write_csv(self, stamp_ros, meas, accepted_meas, p_hit, t_hit):
        """
        Append a CSV row with timestamps, measurement, filtered state, and hit info.
        No-ops when self.csv_path is None or empty (as configured above).
        """
        if not self.csv_path:
            return
        stamp_iso = datetime.now().isoformat()
        stamp_ros_s = float(stamp_ros.to_sec())

        meas_x, meas_y, meas_z = map(float, meas.tolist())

        if accepted_meas:
            fx, fy, fz = float(self.x[0]), float(self.x[1]), float(self.x[2])
        else:
            fx = fy = fz = ""
        if p_hit is not None:
            hx, hy, hz = map(float, p_hit.tolist())
        else:
            hx = hy = hz = ""
        thit = float(t_hit) if t_hit is not None else ""

        header = [
            "exp_tag",
            "stamp_iso", "stamp_ros_s",
            "meas_x", "meas_y", "meas_z",
            "filt_x", "filt_y", "filt_z",
            "hit_x", "hit_y", "hit_z",
            "t_hit_s", "accepted"
        ]

        row = [
            self.exp_tag,
            stamp_iso, stamp_ros_s,
            meas_x, meas_y, meas_z,
            fx, fy, fz,
            hx, hy, hz,
            thit, int(bool(accepted_meas))
        ]

        try:
            write_header_now = not self._csv_header_written
            with open(self.csv_path, "a", newline="") as f:
                w = csv.writer(f)
                if write_header_now:
                    w.writerow(header)
                    self._csv_header_written = True
                w.writerow(row)
        except Exception as e:
            rospy.logwarn("CSV: failed to write '%s': %s", self.csv_path, e)

if __name__ == "__main__":
    rospy.init_node("ball_kf_predictor")
    BallKFPredictor()
    rospy.spin()
