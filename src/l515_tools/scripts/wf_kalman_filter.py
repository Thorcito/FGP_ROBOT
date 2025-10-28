#!/usr/bin/env python3
import rospy, numpy as np, os, csv
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped
from nav_msgs.msg import Path
from std_msgs.msg import Header, Float32
from datetime import datetime

class KalmanFilter_world:
    def __init__(self):
        # ---------- params ----------
        self.frame_id      = rospy.get_param("~frame_id", "world")
        self.input_topic   = rospy.get_param("~input_topic", "/world_ball_meas/point")
        
        # gravity in world frame
        self.g_world = np.array(rospy.get_param("~g_world", [0.0, 0.0, -9.81]), dtype=np.float64)

        # per-axis measurement / process (accel) noise
        # R = diag(sigma_pos_xyz^2); Q derived from sigma_acc_xyz via white-noise accel model
        self.sigma_pos_xyz = np.array(rospy.get_param("~sigma_pos_xyz", [0.0722, 0.0736, 0.0576]), dtype=np.float64)
        self.sigma_acc_xyz = np.array(rospy.get_param("~sigma_acc_xyz", [7.30, 7.02, 6.84]), dtype=np.float64)

        # proper initialization std devs (used only at first detection)
        self.init_pos_std  = float(rospy.get_param("~init_pos_std_m", 0.02))
        self.init_vel_std  = float(rospy.get_param("~init_vel_std_mps", 2.0))

        # prediction overlays (for RViz/PlotJuggler)
        self.lookahead_s    = float(rospy.get_param("~lookahead_s", 0.05))
        self.pred_horizon_s = float(rospy.get_param("~pred_horizon_s", 1.0))
        self.pred_dt_s      = float(rospy.get_param("~pred_dt_s", 0.02))

        self.min_dt_s = float(rospy.get_param("~min_dt_s", 1e-4))
        self.max_dt_s = float(rospy.get_param("~max_dt_s", 0.20))

        # filtered path buffer
        self.keep_filtered_path = bool(rospy.get_param("~keep_filtered_path", True))
        self.filtered_path_max  = int(rospy.get_param("~filtered_path_max", 50))
        self.filtered_path = []

        # ------ Mahalanobis gating params ------
        # 3 DoF chi-square thresholds: ~11.34 ≈ 99%, ~16.27 ≈ 99.9%
        self.meas_gate_chi2_soft = float(rospy.get_param("~meas_gate_chi2_soft", 11.34))  # 3DoF ~99%
        self.meas_gate_chi2_hard = float(rospy.get_param("~meas_gate_chi2_hard", 16.27))  # 3DoF ~99.9%
        self.meas_soft_factor_max = float(rospy.get_param("~meas_soft_factor_max", 15.0))
        self.meas_min_abs_jump_m = float(rospy.get_param("~meas_min_abs_jump_m", 0.15))
        self.gating_warmup_updates = int(rospy.get_param("~gating_warmup_updates", 3))

        # ---------- state ----------
        # x = [px,py,pz, vx,vy,vz]^T
        self.x = None
        self.P = None
        self.t_last = None

        # gating state
        self.z_prev = None
        self.updates_since_init = 0

        # ---------- pubs/sub ----------
        # Existing useful pubs
        self.pub_current_pos   = rospy.Publisher("/world_ball_pred/current_pos", PointStamped, queue_size=10)
        self.pub_pred_pt   = rospy.Publisher("/world_ball_pred/pred_point", PointStamped, queue_size=10)
        self.pub_pred_path = rospy.Publisher("/world_ball_pred/pred_path", Path, queue_size=10)
        self.pub_filt_path = rospy.Publisher("/world_ball_pred/path_filt", Path, queue_size=10)
        self.pub_current_vel = rospy.Publisher("/world_ball_pred/current_vel", Vector3Stamped, queue_size=10)


        rospy.Subscriber(self.input_topic, PointStamped, self.cb_meas, queue_size=50)

        # prebuild constant H and R for position-only measurement model
        self.H = np.zeros((3,6)); self.H[0,0]=self.H[1,1]=self.H[2,2]=1.0
        self.R = np.diag(self.sigma_pos_xyz**2)
        rospy.loginfo("KF world: frame=%s, input=%s, g_world=%s, horizon=%.2fs @ %.0fHz",
              self.frame_id, self.input_topic, self.g_world.tolist(),
              self.pred_horizon_s, 1.0/self.pred_dt_s)


    def F_Q_gvec(self, dt):
        """
        Build discrete-time transition F, process covariance Q, and gravity input g_vec
        for a constant-acceleration model with per-axis white-noise acceleration.

        State: x = [px, py, pz, vx, vy, vz]^T

        F:
            [ I3  dt*I3 ]
            [ 0     I3  ]

        Q per axis (q = sigma_acc^2):
            q * [[dt^3/3, dt^2/2],
                 [dt^2/2, dt    ]]

        g_vec (gravity contribution):
            [ 0.5*g*dt^2 ; g*dt ]
        """
        F = np.eye(6)
        F[0,3]=dt; F[1,4]=dt; F[2,5]=dt
        dt2, dt3 = dt*dt, dt*dt*dt

        Q = np.zeros((6,6))
        for i in range(3):
            q = self.sigma_acc_xyz[i]**2
            Q1 = q * np.array([[dt3/3.0, dt2/2.0],
                               [dt2/2.0, dt      ]], dtype=np.float64)
            ii, vv = i, 3+i
            Q[ii,ii]+=Q1[0,0]; Q[ii,vv]+=Q1[0,1]
            Q[vv,ii]+=Q1[1,0]; Q[vv,vv]+=Q1[1,1]

        g = self.g_world
        g_vec = np.array([0.5*g[0]*dt2, 0.5*g[1]*dt2, 0.5*g[2]*dt2,
                          g[0]*dt,      g[1]*dt,      g[2]*dt     ], dtype=np.float64)
        return F, Q, g_vec
    
    # ---------- predict / update ----------
    def predict_step(self, dt):
        """Time-update with gravity input."""
        F, Q, g_vec = self.F_Q_gvec(dt)
        self.x = F.dot(self.x) + g_vec
        self.P = F.dot(self.P).dot(F.T) + Q

    def update_step(self, z):
        """
        Measurement-update with Mahalanobis gating.

        Returns:
            accepted (bool), d2 (float): whether measurement accepted and its innovation distance^2.
        """
        # innovation
        y = z - self.H.dot(self.x)
        S = self.H.dot(self.P).dot(self.H.T) + self.R

        # Mahalanobis distance^2
        Sinv_y = np.linalg.solve(S, y)
        d2 = float(y.dot(Sinv_y))

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
        K = np.linalg.solve(S.T, K.T).T

        # posterior
        self.x = self.x + K.dot(y)
        I = np.eye(6)
        self.P = (I - K.dot(self.H)).dot(self.P)
        # remember accepted measurement
        self.z_prev = z.copy()
        self.updates_since_init += 1
        return True, d2
     # ---------- kinematics ----------
    def ballistic_pos(self, dt):
        """Ballistic propagation under constant gravity in world frame."""
        p0 = self.x[0:3]; v0 = self.x[3:6]; a = self.g_world
        return p0 + v0*dt + 0.5*a*(dt*dt)
    
    # ---------- publishers ----------
    def _publish_overlays(self, hdr, accepted_meas: bool):
        """
        Publish the filtered point/path (if accepted), current velocity,
        the lookahead predicted point, and a sampled prediction path.
        """
        # Filtered point & path (p0)
        if accepted_meas:
            pt_f = PointStamped(); pt_f.header = hdr
            pt_f.point.x, pt_f.point.y, pt_f.point.z = self.x[0], self.x[1], self.x[2]
            self.pub_current_pos.publish(pt_f)

            # Current velocity (for PlotJuggler) (v0)
            vel = Vector3Stamped(); vel.header = hdr
            vel.vector.x, vel.vector.y, vel.vector.z = self.x[3], self.x[4], self.x[5]
            self.pub_current_vel.publish(vel)

            if self.keep_filtered_path:
                pose = PoseStamped(); pose.header = hdr
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.x[0], self.x[1], self.x[2]
                pose.pose.orientation.w = 1.0
                self.filtered_path.append(pose)
                if len(self.filtered_path) > self.filtered_path_max:
                    self.filtered_path = self.filtered_path[-self.filtered_path_max:]
                path_f = Path(); path_f.header = hdr; path_f.poses = self.filtered_path
                self.pub_filt_path.publish(path_f)

        # Lookahead point
        pL = self.ballistic_pos(self.lookahead_s)
        pt_p = PointStamped(); pt_p.header = hdr
        pt_p.point.x, pt_p.point.y, pt_p.point.z = float(pL[0]), float(pL[1]), float(pL[2])
        self.pub_pred_pt.publish(pt_p)

        # Predicted path (sampled along horizon)
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
    
    # ---------- callback ----------
    def cb_meas(self, msg: PointStamped):
        """
        Handle incoming measurement:
        - Initialize on first message.
        - Thereafter: compute dt, run predict+update (with gating), then publish overlays.
        """
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            rospy.logwarn_throttle(1.0, "Incoming frame %s != %s, treating as world frame",
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

            hdr = Header(stamp=msg.header.stamp, frame_id=self.frame_id)
            self._publish_overlays(hdr, accepted_meas=True)
            return

        dt_raw = t - self.t_last
        if dt_raw <= 0.0:
            return
        dt = float(np.clip(dt_raw, self.min_dt_s, self.max_dt_s))
        self.t_last = t

        self.predict_step(dt)
        accepted, _ = self.update_step(z_meas)

        hdr = Header(stamp=msg.header.stamp, frame_id=self.frame_id)
        self._publish_overlays(hdr, accepted_meas=accepted)
    


if __name__ == "__main__":
    rospy.init_node("Kalman_Filter_world")
    KalmanFilter_world()
    rospy.spin()