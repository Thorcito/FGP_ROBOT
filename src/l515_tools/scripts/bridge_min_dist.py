#!/usr/bin/env python3
import rospy, math
from tum_ics_ur10_controller_tutorial.msg import EETarget
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped, Vector3Stamped
from std_msgs.msg import Float32
from nav_msgs.msg import Path
import numpy as np

import tf2_ros
from tf2_geometry_msgs import do_transform_point

class KFtoEETargetBridgeMinDist:
    def __init__(self):
        # -------- params --------
        self.position_topic = rospy.get_param("~position_topic", "/ball_pred/point_filt")
        self.velocity_topic = rospy.get_param("~velocity_topic", "/ball_pred/current_vel")

        self.world_frame    = rospy.get_param("~world_frame", "world")
        self.ee_frame       = rospy.get_param("~ee_frame", "ur10_model_dh_5")

        # EE motion limits
        self.v_max_mps      = float(rospy.get_param("~v_max_mps", 1.6))
        self.t_min_s        = float(rospy.get_param("~t_min_s", 0.12))
        self.t_max_s        = float(rospy.get_param("~t_max_s", 1.20))
        self.t_tol_s        = float(rospy.get_param("~t_tol_s", 0.002))

        # Gravity in world (+Z up)
        self.g_world = np.array(rospy.get_param("~g_world", [0.0, 0.0, -9.81]), dtype=np.float64)

        # ---- WORKSPACE (AABB in world frame) ----
        # Set these to your UR10 feasible Cartesian box
        self.use_workspace_gate = bool(rospy.get_param("~use_workspace_gate", True))
        self.ws_xmin = float(rospy.get_param("~ws_xmin",  -0.8))
        self.ws_xmax = float(rospy.get_param("~ws_xmax",  0.0))
        self.ws_ymin = float(rospy.get_param("~ws_ymin", -0.30))
        self.ws_ymax = float(rospy.get_param("~ws_ymax",  0.30))
        self.ws_zmin = float(rospy.get_param("~ws_zmin",  0.2))
        self.ws_zmax = float(rospy.get_param("~ws_zmax",  1.20))
        self.ws_margin = float(rospy.get_param("~ws_margin", 0.02))  # small inward margin

        # If r* is outside, try to find the earliest valid t by stepping:
        self.ws_rescan_dt = float(rospy.get_param("~ws_rescan_dt", 0.02))  # 50 Hz scan

        # Safety
        self.max_step_m     = rospy.get_param("~max_step_m", 0.20)

        # Camera TF
        self.publish_camera_tf = rospy.get_param("~publish_camera_tf", True)
        self.camera_parent     = rospy.get_param("~camera_parent", "world")
        self.camera_frame      = rospy.get_param("~camera_frame", "camera_color_optical_frame")
        self.cam_xyz           = rospy.get_param("~camera_xyz", [0.05, 0.0, 1.0])
        self.cam_quat_xyzw     = rospy.get_param("~camera_quat_xyzw", [-0.5, -0.5, 0.5, 0.5])  # x y z w

        # Orientation tracking
        self.vel_min_speed = float(rospy.get_param("~vel_min_speed_mps", 0.6))
        self.up_world = np.array(rospy.get_param("~up_world", [0.0, 0.0, 1.0]), dtype=np.float64)
        self.dir_sign = -1.0
        self.dir_alpha = float(rospy.get_param("~dir_alpha", 0.5))

        # -------- state --------
        self.pos_ema = None
        self.old_x = self.old_y = self.old_z = None
        self.pts = []

        # latest velocity (world) with stamp
        self.v_world = None
        self.v_stamp = None

        # -------- TF2 --------
        self.buf = tf2_ros.Buffer(cache_time=rospy.Duration(2.0))
        self.listener = tf2_ros.TransformListener(self.buf)
        if self.publish_camera_tf:
            self._publish_static_camera_tf()

        # -------- pubs/subs --------
        self.pub = rospy.Publisher("ee_target", EETarget, queue_size=1)
        self.pub_point = rospy.Publisher("Bridge_minDist/punto", PointStamped, queue_size=10)
        self.pub_path  = rospy.Publisher("Bridge_minDist/path", Path, queue_size=10)
        self.pub_best_t    = rospy.Publisher("Bridge_minDist/best_t", Float32, queue_size=10)
        self.pub_best_cost = rospy.Publisher("Bridge_minDist/best_cost", Float32, queue_size=10)

        rospy.Subscriber(self.position_topic, PointStamped, self.on_point, queue_size=10)
        rospy.Subscriber(self.velocity_topic, Vector3Stamped, self.on_v, queue_size=10)

        rospy.loginfo(f"[bridge_minDist] ready  world={self.world_frame}  ee={self.ee_frame}  t∈[{self.t_min_s},{self.t_max_s}] v_max={self.v_max_mps:.2f} m/s")

    # ---------- helpers ----------
    def _publish_static_camera_tf(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.camera_parent
        t.child_frame_id  = self.camera_frame
        t.transform.translation.x = float(self.cam_xyz[0])
        t.transform.translation.y = float(self.cam_xyz[1])
        t.transform.translation.z = float(self.cam_xyz[2])
        qx, qy, qz, qw = self.cam_quat_xyzw
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_broadcaster.sendTransform(t)

    def _rotate_vec_by_tf(self, v_cam, world_frame, cam_frame):
        Tcw = self.buf.lookup_transform(world_frame, cam_frame, rospy.Time(0), rospy.Duration(0.05))
        q = Tcw.transform.rotation
        qc = np.array([q.w, q.x, q.y, q.z], dtype=np.float64)  # [w,x,y,z]
        x, y, z = v_cam
        qw, qx, qy, qz = qc
        t = 2.0 * np.cross([qx, qy, qz], [x, y, z])
        v_world = [x, y, z] + qw * t + np.cross([qx, qy, qz], t)
        return np.array(v_world, dtype=np.float64)

    @staticmethod
    def _unit(v):
        n = float(np.linalg.norm(v))
        return v / (n + 1e-12)

    # --- orientation helpers (same as before) ---
    def _R_from_quat_xyzw(self, q):
        x, y, z, w = q
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        return np.array([
            [1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)],
            [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)],
            [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)]
        ], dtype=np.float64)

    def _quat_from_R(self, R):
        tr = np.trace(R)
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2.0
            qw = 0.25 * S
            qx = (R[2,1] - R[1,2]) / S
            qy = (R[0,2] - R[2,0]) / S
            qz = (R[1,0] - R[0,1]) / S
        else:
            i = np.argmax([R[0,0], R[1,1], R[2,2]])
            if i == 0:
                S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
                qw = (R[2,1] - R[1,2]) / S; qx = 0.25 * S; qy = (R[0,1] + R[1,0]) / S; qz = (R[0,2] + R[2,0]) / S
            elif i == 1:
                S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
                qw = (R[0,2] - R[2,0]) / S; qx = (R[0,1] + R[1,0]) / S; qy = 0.25 * S; qz = (R[1,2] + R[2,1]) / S
            else:
                S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
                qw = (R[1,0] - R[0,1]) / S; qx = (R[0,2] + R[2,0]) / S; qy = (R[1,2] + R[2,1]) / S; qz = 0.25 * S
        q = np.array([qx, qy, qz, qw], dtype=np.float64)
        q /= (np.linalg.norm(q) + 1e-12)
        return q

    def _rot_about_axis_matrix(self, k, phi):
        k = self._unit(k)
        K = np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]], dtype=np.float64)
        I = np.eye(3)
        return I*math.cos(phi) + math.sin(phi)*K + (1-math.cos(phi))*np.outer(k,k)

    def _frame_from_dir_with_up_z(self, dir_world, up_world):
        z = self._unit(dir_world)
        up = self._unit(up_world)
        if abs(np.dot(z, up)) > 0.98:
            up = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        x = self._unit(np.cross(up, z))
        y = np.cross(z, x)
        return np.column_stack([x, y, z])

    def _candidate_for_z(self, dir_world, up_world, q_now_xyzw):
        R_now = self._R_from_quat_xyzw(q_now_xyzw)
        R_base = self._frame_from_dir_with_up_z(dir_world, up_world)
        z = R_base[:, 2]
        x_now = R_now[:, 0]
        def proj_plane(v): 
            return self._unit(v - z * np.dot(z, v))
        x_p = proj_plane(x_now)
        x0  = R_base[:, 0]
        s = np.dot(z, np.cross(x0, x_p))
        c = np.dot(x0, x_p)
        phi = math.atan2(s, c)
        R_opt = self._rot_about_axis_matrix(z, phi) @ R_base
        # 180° roll candidate
        R_flip = R_base.copy(); R_flip[:, 0] *= -1.0; R_flip[:, 1] *= -1.0
        x0f = R_flip[:, 0]
        s2 = np.dot(z, np.cross(x0f, x_p)); c2 = np.dot(x0f, x_p)
        phi2 = math.atan2(s2, c2)
        R_opt2 = self._rot_about_axis_matrix(z, phi2) @ R_flip

        q_now = self._quat_from_R(R_now)
        q1 = self._quat_from_R(R_opt)
        q2 = self._quat_from_R(R_opt2)
        def quat_angle(qA, qB):
            dot = float(np.dot(qA, qB)); dot = abs(max(min(dot, 1.0), -1.0))
            return 2.0 * math.acos(dot)
        return (q1, quat_angle(q_now, q1)) if quat_angle(q_now, q1) <= quat_angle(q_now, q2) else (q2, quat_angle(q_now, q2))

    # ---------- workspace helpers ----------
    def _inside_ws(self, p):
        if not self.use_workspace_gate:
            return True
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        eps = self.ws_margin
        return (self.ws_xmin+eps <= x <= self.ws_xmax-eps and
                self.ws_ymin+eps <= y <= self.ws_ymax-eps and
                self.ws_zmin+eps <= z <= self.ws_zmax-eps)

    # ---------- subscriptions ----------
    def on_v(self, msg: Vector3Stamped):
        cam_frame = msg.header.frame_id or self.camera_frame
        try:
            v_cam = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=np.float64)
            v_world = self._rotate_vec_by_tf(v_cam, self.world_frame, cam_frame)
        except Exception as e:
            rospy.logwarn_throttle(0.5, f"[bridge_minDist] v TF rot failed: {e}")
            return

        if np.linalg.norm(v_world) >= self.vel_min_speed:
            p_raw = self._unit(self.dir_sign * v_world)
            if self.pos_ema is None: self.pos_ema = p_raw
            self.pos_ema = self._unit((1.0 - self.dir_alpha) * self.pos_ema + self.dir_alpha * p_raw)

        self.v_world = v_world
        self.v_stamp = msg.header.stamp

    def on_point(self, p_cam: PointStamped):
        if self.v_world is None or self.v_stamp is None:
            return
        if abs((p_cam.header.stamp - self.v_stamp).to_sec()) > 0.01:
            return  # keep the pair synchronized

        # Transform camera → world for position
        cam_frame = p_cam.header.frame_id or self.camera_frame
        try:
            Tcw = self.buf.lookup_transform(self.world_frame, cam_frame,
                                            rospy.Time(0), rospy.Duration(0.05))
            p_world_geo = do_transform_point(p_cam, Tcw)
            p_world = np.array([p_world_geo.point.x, p_world_geo.point.y, p_world_geo.point.z], dtype=np.float64)
        except Exception as e:
            rospy.logwarn_throttle(0.0, f"[bridge_minDist] TF transform failed ({cam_frame}→{self.world_frame}): {e}")
            return

        # Current EE pose
        try:
            T_w_ee = self.buf.lookup_transform(self.world_frame, self.ee_frame,
                                               rospy.Time(0), rospy.Duration(0.02))
            pe = np.array([T_w_ee.transform.translation.x,
                           T_w_ee.transform.translation.y,
                           T_w_ee.transform.translation.z], dtype=np.float64)
            qee = T_w_ee.transform.rotation
            q_ee = np.array([qee.x, qee.y, qee.z, qee.w], dtype=np.float64)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[bridge_minDist] EE TF lookup failed ({self.ee_frame}→{self.world_frame}): {e}")
            return
        
         # High-level info log (ball & EE positions + workspace)
        rospy.loginfo_throttle(
            0.3,
            f"[bridge_minDist] EE=({pe[0]:+.3f},{pe[1]:+.3f},{pe[2]:+.3f})  "
            f"Ball p0=({p_world[0]:+.3f},{p_world[1]:+.3f},{p_world[2]:+.3f})  "
            f"v0=({self.v_world[0]:+.3f},{self.v_world[1]:+.3f},{self.v_world[2]:+.3f})  "
            f"x[{self.ws_xmin:.2f},{self.ws_xmax:.2f}] "
            f"y[{self.ws_ymin:.2f},{self.ws_ymax:.2f}] "
            f"z[{self.ws_zmin:.2f},{self.ws_zmax:.2f}] (margin {self.ws_margin:.2f})")

        # Optional step sanity (doesn't block)
        if self.old_x is not None:
            prev = np.array([self.old_x, self.old_y, self.old_z], dtype=np.float64)
            step = np.linalg.norm(prev - p_world)
            if step > self.max_step_m:
                rospy.logwarn_throttle(0.0, f"[bridge_minDist] big step {step:.3f} > {self.max_step_m:.3f}; continuing")

        p0 = p_world
        v0 = self.v_world
        g  = self.g_world

        def p_ball(t): return p0 + v0*t + 0.5*g*(t*t)

        def f_cost(t):
            d = p_ball(t) - pe
            dist = float(np.linalg.norm(d))
            return max(0.0, dist - self.v_max_mps * t)

        # --- Golden-section search on [t_min, t_max] ---
        def golden(a, b, tol):
            invphi  = (math.sqrt(5.0) - 1.0) / 2.0
            invphi2 = (3.0 - math.sqrt(5.0)) / 2.0
            c = a + invphi2 * (b - a)
            d = a + invphi  * (b - a)
            fc, fd = f_cost(c), f_cost(d)
            iters = 0
            while (b - a) > tol:
                iters += 1
                if fc <= fd:
                    b, d, fd = d, c, fc
                    c = a + invphi2 * (b - a)
                    fc = f_cost(c)
                else:
                    a, c, fc = c, d, fd
                    d = a + invphi * (b - a)
                    fd = f_cost(d)
            t_star = 0.5 * (a + b)
            return t_star, f_cost(t_star), iters

        # First pass
        t_lo = max(0.0, float(self.t_min_s))
        t_hi = float(self.t_max_s)
        if t_hi <= t_lo + 1e-6:
            rospy.logwarn("[bridge_minDist] invalid t window")
            return
        t_star, f_star, iters= golden(t_lo, t_hi, self.t_tol_s)

        # Build reachable target r* at t*
        pb = p_ball(t_star)
        dvec = pb - pe
        dist = float(np.linalg.norm(dvec))
        reach = self.v_max_mps * t_star
        r_star = pe + (dvec * (min(dist, reach) / max(dist, 1e-12)))

        rospy.loginfo_throttle(
            0.3,
            f"[bridge_minDist] GSS iters={iters} t*={t_star:.3f}s  "
            f"pb(t*)=({pb[0]:+.3f},{pb[1]:+.3f},{pb[2]:+.3f})  "
            f"dist={dist:.3f}  reach={reach:.3f}  cost={f_star:.3f}"
        )

        # ---- WORKSPACE GATE ----
        # If r_star not inside, try to find first t ≥ t_star that gives in-box r(t)
        if self.use_workspace_gate and not self._inside_ws(r_star):
            rospy.logwarn_throttle(0.3, f"[bridge_minDist] r* outside WS, rescanning forward at {self.ws_rescan_dt:.3f}s steps...")
            found = False
            t_scan = t_star
            while t_scan <= t_hi + 1e-9:
                pb_s = p_ball(t_scan)
                d_s  = pb_s - pe
                dist_s = float(np.linalg.norm(d_s))
                reach_s = self.v_max_mps * t_scan
                r_s = pe + (d_s * (min(dist_s, reach_s) / max(dist_s, 1e-12)))
                rospy.logdebug(f"[bridge_minDist] scan t={t_scan:.3f} r=({r_s[0]:+.3f},{r_s[1]:+.3f},{r_s[2]:+.3f}) WS={self._inside_ws(r_s)}")
                if self._inside_ws(r_s):
                    # refine only in the valid tail [t_scan, t_hi]
                    t_star, f_star, iters2 = golden(t_scan, t_hi, self.t_tol_s)
                    pb = p_ball(t_star)
                    dvec = pb - pe
                    dist = float(np.linalg.norm(dvec))
                    reach = self.v_max_mps * t_star
                    r_star = pe + (dvec * (min(dist, reach) / max(dist, 1e-12)))
                    rospy.loginfo(f"[bridge_minDist] Rescan OK: t*={t_star:.3f}s r*=({r_star[0]:+.3f},{r_star[1]:+.3f},{r_star[2]:+.3f})")
                    found = True
                    break
                t_scan += self.ws_rescan_dt

            if not found:
                # Nothing inside workspace for this segment → skip publishing this cycle
                rospy.logwarn_throttle(0.5, "[bridge_minDist] No feasible r(t) inside workspace in [t*, t_max] → skip publish")
                return

        # Orientation facing ball velocity at t*
        v_t = v0 + g * t_star
        if np.linalg.norm(v_t) >= self.vel_min_speed:
            p_raw = self._unit(self.dir_sign * v_t)
            if self.pos_ema is None: self.pos_ema = p_raw
            self.pos_ema = self._unit((1.0 - self.dir_alpha) * self.pos_ema + self.dir_alpha * p_raw)

        try:
            T_w_ee = self.buf.lookup_transform(self.world_frame, self.ee_frame,
                                               rospy.Time(0), rospy.Duration(0.02))
            qee = T_w_ee.transform.rotation
            q_ee = np.array([qee.x, qee.y, qee.z, qee.w], dtype=np.float64)
        except Exception:
            q_ee = np.array([0,0,0,1], dtype=np.float64)

        if self.pos_ema is not None:
            q_tgt_xyzw, _ = self._candidate_for_z(self.pos_ema, self.up_world, q_ee)
            use_q = q_tgt_xyzw
        else:
            use_q = q_ee

        # breadcrumbs
        stamp = p_cam.header.stamp if p_cam.header.stamp.to_sec() > 0 else rospy.Time.now()

        pto_w = PointStamped()
        pto_w.header.stamp = stamp
        pto_w.header.frame_id = self.world_frame
        pto_w.point.x, pto_w.point.y, pto_w.point.z = r_star[0], r_star[1], r_star[2]
        self.pub_point.publish(pto_w)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.world_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = r_star[0], r_star[1], r_star[2]
        pose.pose.orientation.w = 1.0
        self.pts.append(pose)
        if len(self.pts) > 20: self.pts = self.pts[-20:]
        path_msg = Path(); path_msg.header.stamp = stamp; path_msg.header.frame_id = self.world_frame
        path_msg.poses = list(self.pts)
        self.pub_path.publish(path_msg)
        rospy.loginfo_throttle(
            0.3,
            f"[bridge_minDist] COMMAND? r*=({r_star[0]:+.3f},{r_star[1]:+.3f},{r_star[2]:+.3f}) "
            f"insideWS={self._inside_ws(r_star)}  t*={t_star:.3f}s  cost={f_star:.3f}"
        )

        # Command only if inside workspace (double-check)
        if not self._inside_ws(r_star):
            return

        msg = EETarget()
        msg.ee_target.position.x = float(r_star[0])
        msg.ee_target.position.y = float(r_star[1])
        msg.ee_target.position.z = float(r_star[2])
        msg.ee_target.orientation.x = float(use_q[0])
        msg.ee_target.orientation.y = float(use_q[1])
        msg.ee_target.orientation.z = float(use_q[2])
        msg.ee_target.orientation.w = float(use_q[3])
        msg.duration = float(t_star)
        self.pub.publish(msg)

        # Debug scalars
        self.pub_best_t.publish(Float32(data=float(t_star)))
        self.pub_best_cost.publish(Float32(data=float(f_star)))

        # Update last commanded point
        self.old_x, self.old_y, self.old_z = r_star[0], r_star[1], r_star[2]

        rospy.loginfo_throttle(
            0.2,
            f"[bridge_minDist] t*={t_star:.3f}s  r*=({r_star[0]:+.3f},{r_star[1]:+.3f},{r_star[2]:+.3f})"
        )

if __name__ == "__main__":
    rospy.init_node("kf_to_ee_target_bridge_minDist")
    KFtoEETargetBridgeMinDist()
    rospy.spin()
