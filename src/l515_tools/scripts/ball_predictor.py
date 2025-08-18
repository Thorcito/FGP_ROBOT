#!/usr/bin/env python3
import rospy, numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header, Float32

class BallKFPredictor:
    def __init__(self):
        # ---------- params ----------
        self.frame_id      = rospy.get_param("~frame_id", "camera_color_optical_frame")
        self.input_topic   = rospy.get_param("~input_topic", "/ball/point")

        # gravity in CAMERA frame (+Y is down for your camera)
        self.g_cam = np.array(rospy.get_param("~g_cam", [0.0, 9.81, 0.0]), dtype=np.float64)

        # per-axis measurement / process (accel) noise
        self.sigma_pos_xyz = np.array(rospy.get_param("~sigma_pos_xyz", [0.006, 0.012, 0.006]), dtype=np.float64)
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
        self.filtered_path_max  = int(rospy.get_param("~filtered_path_max", 500))
        self.filtered_path = []

        # ---------- state ----------
        # x = [px,py,pz, vx,vy,vz]^T
        self.x = None
        self.P = None
        self.t_last = None

        # ---------- pubs/sub ----------
        self.pub_filt_pt   = rospy.Publisher("/ball_cam/point_filt", PointStamped, queue_size=20)
        self.pub_pred_pt   = rospy.Publisher("/ball_cam/pred_point", PointStamped, queue_size=20)
        self.pub_pred_path = rospy.Publisher("/ball_cam/pred_path", Path, queue_size=10)
        self.pub_filt_path = rospy.Publisher("/ball_cam/path_filt", Path, queue_size=10)

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

        rospy.Subscriber(self.input_topic, PointStamped, self.cb_meas, queue_size=50)

        rospy.loginfo("BallKFPredictor: frame=%s, g_cam=%s, lookahead=%.2fs, horizon=%.2fs@%.0fHz",
                      self.frame_id, self.g_cam.tolist(), self.lookahead_s,
                      self.pred_horizon_s, 1.0/self.pred_dt_s)

        # prebuild constant H and R
        self.H = np.zeros((3,6)); self.H[0,0]=self.H[1,1]=self.H[2,2]=1.0
        self.R = np.diag(self.sigma_pos_xyz**2)

    # ---------- model helpers ----------
    def F_Q_gvec(self, dt):
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
        F, Q, g_vec = self.F_Q_gvec(dt)
        self.x = F.dot(self.x) + g_vec
        self.P = F.dot(self.P).dot(F.T) + Q

    def update_step(self, z, stamp):
        # innovation
        y = z - self.H.dot(self.x)
        S = self.H.dot(self.P).dot(self.H.T) + self.R
        # K = P Hᵀ S⁻¹  (solve instead of invert)
        K = self.P.dot(self.H.T)
        K = np.linalg.solve(S.T, K.T).T  # (Sᵀ \ (P Hᵀ))ᵀ

        # posterior
        self.x = self.x + K.dot(y)
        I = np.eye(6)
        self.P = (I - K.dot(self.H)).dot(self.P)

        # diagnostics
        self.pub_innov_x.publish(Float32(data=float(y[0])))
        self.pub_innov_y.publish(Float32(data=float(y[1])))
        self.pub_innov_z.publish(Float32(data=float(y[2])))
        self.pub_vx.publish(Float32(data=float(self.x[3])))
        self.pub_vy.publish(Float32(data=float(self.x[4])))
        self.pub_vz.publish(Float32(data=float(self.x[5])))
        self.pub_sigma_px.publish(Float32(data=float(np.sqrt(self.P[0,0]))))
        self.pub_sigma_py.publish(Float32(data=float(np.sqrt(self.P[1,1]))))
        self.pub_sigma_pz.publish(Float32(data=float(np.sqrt(self.P[2,2]))))

        self.publish_filtered(stamp)
        self.publish_prediction(stamp)

    # ---------- callback ----------
    def cb_meas(self, msg: PointStamped):
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            rospy.logwarn_throttle(1.0, "Incoming frame %s != %s, treating as camera frame",
                                   msg.header.frame_id, self.frame_id)

        t = msg.header.stamp.to_sec()
        z_meas = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)

        if self.x is None:
            # initialize sensibly
            self.x = np.zeros(6); self.x[0:3] = z_meas
            self.P = np.diag([self.init_pos_std**2]*3 + [self.init_vel_std**2]*3)
            self.t_last = t
            self.publish_filtered(msg.header.stamp)
            self.publish_prediction(msg.header.stamp)
            return

        
        dt_raw = t - self.t_last
        if dt_raw <= 0.0:
            return
        dt = float(np.clip(dt_raw, self.min_dt_s, self.max_dt_s))
        self.t_last = t

        self.predict_step(dt)
        self.update_step(z_meas, msg.header.stamp)

    # ---------- publishing ----------
    def publish_filtered(self, stamp):
        pt = PointStamped()
        pt.header = Header(stamp=stamp, frame_id=self.frame_id)
        pt.point.x, pt.point.y, pt.point.z = self.x[0], self.x[1], self.x[2]
        self.pub_filt_pt.publish(pt)

        if self.keep_filtered_path:
            pose = PoseStamped(); pose.header = pt.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pt.point.x, pt.point.y, pt.point.z
            pose.pose.orientation.w = 1.0
            self.filtered_path.append(pose)
            if len(self.filtered_path) > self.filtered_path_max:
                self.filtered_path = self.filtered_path[-self.filtered_path_max:]
            path = Path(); path.header = pose.header; path.poses = self.filtered_path
            self.pub_filt_path.publish(path)

    def ballistic_pos(self, dt):
        p0 = self.x[0:3]; v0 = self.x[3:6]; a = self.g_cam
        return p0 + v0*dt + 0.5*a*(dt*dt)

    def publish_prediction(self, stamp):
        # single look-ahead
        pL = self.ballistic_pos(self.lookahead_s)
        pt = PointStamped()
        pt.header = Header(stamp=stamp, frame_id=self.frame_id)
        pt.point.x, pt.point.y, pt.point.z = float(pL[0]), float(pL[1]), float(pL[2])
        self.pub_pred_pt.publish(pt)

        # path
        path = Path()
        path.header = Header(stamp=stamp, frame_id=self.frame_id)
        n = max(2, int(self.pred_horizon_s / self.pred_dt_s))
        for i in range(n+1):
            dt = i * self.pred_dt_s
            p = self.ballistic_pos(dt)
            ps = PoseStamped(); ps.header = path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(p[0]), float(p[1]), float(p[2])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_pred_path.publish(path)

if __name__ == "__main__":
    rospy.init_node("ball_kf_predictor")
    BallKFPredictor()
    rospy.spin()
