#!/usr/bin/env python3
import rospy, math
from tum_ics_ur10_controller_tutorial.msg import EETarget
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped, Vector3Stamped
from std_msgs.msg import Float32
from nav_msgs.msg import Path
import numpy as np

import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_point
from tf.transformations import quaternion_from_matrix, quaternion_slerp

class KFtoEETargetBridge:
    def __init__(self):
        # -------- params --------
        self.position_topic = rospy.get_param("~position_topic", "/ball_pred/hit_point")
        self.velocity_topic = rospy.get_param("~velocity_topic", "/ball_pred/current_vel")
        self.ttg_topic      = rospy.get_param("~t_to_int_topic", "/ball_pred/hit_time_s")
        self.world_frame    = rospy.get_param("~world_frame", "world")
        self.ee_frame       = rospy.get_param("~ee_frame", "ur10_model_dh_5")

        self.pts = []
        self.min_duration   = rospy.get_param("~min_duration_s", 0.1)

        # Camera TF params
        self.publish_camera_tf = rospy.get_param("~publish_camera_tf", True)
        self.camera_parent     = rospy.get_param("~camera_parent", "world")
        self.camera_frame      = rospy.get_param("~camera_frame", "camera_color_optical_frame")
        self.cam_xyz           = rospy.get_param("~camera_xyz", [0.05, 0.0, 1.0])
        self.cam_quat_xyzw     = rospy.get_param("~camera_quat_xyzw", [-0.5, -0.5, 0.5, 0.5])  # x y z w

        self.max_step_m    = rospy.get_param("~max_step_m", 0.13)

        # ORIENTATION
        # NOTE: set ~align_axis to "+x","-x","+y","-y","+z","+z","-z" **or** "auto"
        self.align_axis = rospy.get_param("~align_axis", "auto")
        self.vel_min_speed = float(rospy.get_param("~vel_min_speed_mps", 1.0))
        self.max_vel_change = rospy.get_param("~max_vel_change", 0.3)
        self.up_world = np.array(rospy.get_param("~up_world", [0.0, 0.0, 1.0]), dtype=np.float64)
        self.dir_sign = float(rospy.get_param("~dir_sign", -1.0))  # -1 faced your ball before

        # EMA & stability (kept for non-continuous mode)
        self.dir_alpha = float(rospy.get_param("~dir_alpha", 0.2))
        self.stable_count_lock = int(rospy.get_param("~stable_count_lock", 3))
        self.stable_eps_deg = float(rospy.get_param("~stable_eps_deg", 6.0))
        self.unstable_eps_deg = float(rospy.get_param("~unstable_eps_deg", 12.0))

        # Lock behaviours (used when NOT continuous)
        self.lock_on_first_publish = bool(rospy.get_param("~lock_on_first_publish", False))
        self.lock_dir_source = rospy.get_param("~lock_dir_source", "look_at")  # look_at | velocity
        self.omega_max = float(rospy.get_param("~omega_max_rad", 2.0))        # rad/s at lock
        self.rot_margin = float(rospy.get_param("~rot_time_margin_s", 0.03))  # s
        self.fallback_ttg_s = float(rospy.get_param("~fallback_ttg_s", 0.25))

        # --- NEW: continuous mode (always publish updated orientation) ---
        self.continuous_orientation = bool(rospy.get_param("~continuous_orientation", True))
        self.track_deadband_deg = float(rospy.get_param("~track_deadband_deg", 2.0))
        self.track_omega_max_rad = float(rospy.get_param("~track_omega_max_rad", 4.0))
        self.last_track_stamp = None

        # --- state ---
        self.orientation_locked = False      # ignored when continuous_orientation=True
        self.catch_quat_world = None         # XYZW
        self.pos_ema = None
        self.pos_prev = None
        self.stable_count = 0
        self.chosen_axis_last = None         # which tool axis we’re aligning (+x/-x/…)
        self.v_world_prev = None

        # -------- TF2 --------
        self.buf = tf2_ros.Buffer(cache_time=rospy.Duration(2.0))
        self.listener = tf2_ros.TransformListener(self.buf)
        if self.publish_camera_tf:
            self._publish_static_camera_tf()

        # -------- pubs/subs --------
        self.pub = rospy.Publisher("ee_target", EETarget, queue_size=1)
        self.pub_point = rospy.Publisher("Bridge/punto", PointStamped, queue_size=10)
        self.pub_path = rospy.Publisher("Bridge/path", Path, queue_size=10)
        rospy.Subscriber(self.position_topic, PointStamped, self.on_point, queue_size=10)
        rospy.Subscriber(self.velocity_topic, Vector3Stamped, self.on_vhit, queue_size=10)
        rospy.Subscriber(self.ttg_topic, Float32, self.on_ttg, queue_size=10)

        self.last_ttg = None
        self.old_x = self.old_y = self.old_z = None
        self.get_orientation = False

        rospy.loginfo(f"[bridge] min_duration_s={self.min_duration:.2f}  ee_frame={self.ee_frame}")

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
        qc = np.array([q.w, q.x, q.y, q.z], dtype=np.float64)
        x, y, z = v_cam
        qw, qx, qy, qz = qc
        t = 2.0 * np.cross([qx, qy, qz], [x, y, z])
        v_world = [x, y, z] + qw * t + np.cross([qx, qy, qz], t)
        return np.array(v_world, dtype=np.float64)

    def _unit(self, v):
        n = float(np.linalg.norm(v))
        return v / (n + 1e-12)

    def _frame_from_dir_with_up(self, axis_str, dir_world, up_world):
        axis_str = axis_str.lower()
        idx, sgn = {"+x": (0,+1), "-x": (0,-1),
                    "+y": (1,+1), "-y": (1,-1),
                    "+z": (2,+1), "-z": (2,-1)}[axis_str]
        d = self._unit(dir_world); a_dir = sgn * d
        up = self._unit(up_world)
        if abs(np.dot(a_dir, up)) > 0.98:
            up = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        b = self._unit(np.cross(a_dir, up))
        c = np.cross(b, a_dir)
        R = np.eye(3)
        if idx == 0:   R[:,0]=a_dir; R[:,1]=c;           R[:,2]=np.cross(a_dir,c)
        elif idx == 1: R[:,1]=a_dir; R[:,2]=c;           R[:,0]=np.cross(R[:,1],R[:,2])
        else:          R[:,2]=a_dir; R[:,0]=c;           R[:,1]=np.cross(R[:,2],R[:,0])
        return R, idx

    def _quat_from_R(self, R):
        tr = np.trace(R)
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2.0
            qw = 0.25*S; qx = (R[2,1]-R[1,2])/S; qy=(R[0,2]-R[2,0])/S; qz=(R[1,0]-R[0,1])/S
        else:
            i = np.argmax([R[0,0],R[1,1],R[2,2]])
            if i==0:
                S=math.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])*2.0
                qw=(R[2,1]-R[1,2])/S; qx=0.25*S; qy=(R[0,1]+R[1,0])/S; qz=(R[0,2]+R[2,0])/S
            elif i==1:
                S=math.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])*2.0
                qw=(R[0,2]-R[2,0])/S; qx=(R[0,1]+R[1,0])/S; qy=0.25*S; qz=(R[1,2]+R[2,1])/S
            else:
                S=math.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])*2.0
                qw=(R[1,0]-R[0,1])/S; qx=(R[0,2]+R[2,0])/S; qy=(R[1,2]+R[2,1])/S; qz=0.25*S
        q = np.array([qx,qy,qz,qw], dtype=np.float64)
        q /= (np.linalg.norm(q)+1e-12)
        return q

    def _R_from_quat_xyzw(self, q):
        x,y,z,w = q
        xx,yy,zz = x*x,y*y,z*z; xy=x*y; xz=x*z; yz=y*z; wx=w*x; wy=w*y; wz=w*z
        return np.array([
            [1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)],
            [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)],
            [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)]
        ], dtype=np.float64)

    def _quat_angle_rad(self, q1, q2):
        dot = float(np.dot(q1,q2)); dot = abs(max(min(dot,1.0),-1.0))
        return 2.0*math.acos(dot)

    def _rot_about_axis_matrix(self, k, phi):
        k = self._unit(k)
        K = np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]], dtype=np.float64)
        I = np.eye(3)
        return I*math.cos(phi) + math.sin(phi)*K + (1-math.cos(phi))*np.outer(k,k)

    def _minimize_roll_about_axis(self, R_base, axis_idx, R_now):
        a_dir = R_base[:,axis_idx]
        x_now,y_now = R_now[:,0],R_now[:,1]
        proj = lambda v: v - a_dir*np.dot(a_dir,v)
        x_p = self._unit(proj(x_now)); y_p = self._unit(proj(y_now))
        lat = [i for i in [0,1,2] if i!=axis_idx]
        u0 = self._unit(R_base[:,lat[0]])
        def phi(u,v): return math.atan2(np.dot(a_dir,np.cross(u,v)), np.dot(u,v))
        R_x0 = self._rot_about_axis_matrix(a_dir, phi(u0,x_p)) @ R_base
        R_y0 = self._rot_about_axis_matrix(a_dir, phi(u0,y_p)) @ R_base
        q_now = self._quat_from_R(R_now)
        th_x = self._quat_angle_rad(q_now, self._quat_from_R(R_x0))
        th_y = self._quat_angle_rad(q_now, self._quat_from_R(R_y0))
        return (R_x0, th_x) if th_x <= th_y else (R_y0, th_y)

    def _candidate_for_axis(self, axis_str, dir_world, up_world, q_now_xyzw):
        R_now = self._R_from_quat_xyzw(q_now_xyzw)
        R_base, idx = self._frame_from_dir_with_up(axis_str, dir_world, up_world)
        R_opt, th = self._minimize_roll_about_axis(R_base, idx, R_now)
        # 180° roll variant
        R_flip = R_base.copy()
        if idx==2: R_flip[:,0]*=-1; R_flip[:,1]*=-1
        elif idx==1: R_flip[:,0]*=-1; R_flip[:,2]*=-1
        else: R_flip[:,1]*=-1; R_flip[:,2]*=-1
        Rf_opt, thf = self._minimize_roll_about_axis(R_flip, idx, R_now)
        if thf < th: R_opt, th = Rf_opt, thf
        return self._quat_from_R(R_opt), th, idx

    def _quat_xyzw_to_wxyz(self, q): return np.array([q[3],q[0],q[1],q[2]], float)
    def _quat_wxyz_to_xyzw(self, q): return np.array([q[1],q[2],q[3],q[0]], float)

    # ---------- callbacks ----------
    def on_ttg(self, msg: Float32):
        self.last_ttg = float(msg.data)

    def on_vhit(self, msg: Vector3Stamped):
        if not self.get_orientation:
            return

        cam_frame = msg.header.frame_id or self.camera_frame
        try:
            v_cam = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=np.float64)
            v_world = self._rotate_vec_by_tf(v_cam, self.world_frame, cam_frame)
        except Exception as e:
            rospy.logwarn_throttle(0.5, f"[bridge] vhit TF rot failed: {e}")
            return

        if np.linalg.norm(v_world) < self.vel_min_speed:
            return

        # EMA direction
        p_raw = self._unit(self.dir_sign * v_world)
        if self.pos_ema is None:
            self.pos_ema = p_raw
            self.pos_prev = p_raw

        self.pos_ema = self._unit((1.0 - self.dir_alpha) * self.pos_ema + self.dir_alpha * p_raw)

        # ----- CONTINUOUS ORIENTATION MODE -----
        if self.continuous_orientation:
            # dt for rate limit
            now = msg.header.stamp if msg.header.stamp.to_sec()>0 else rospy.Time.now()
            dt = 0.02 if self.last_track_stamp is None else max(1e-3, (now - self.last_track_stamp).to_sec())
            self.last_track_stamp = now

            # initial command/orientation + choose axis once
            if self.catch_quat_world is None:
                try:
                    T_w_ee = self.buf.lookup_transform(self.world_frame, self.ee_frame,
                                                       rospy.Time(0), rospy.Duration(0.02))
                    q = T_w_ee.transform.rotation
                    q_now = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
                    q_now /= (np.linalg.norm(q_now)+1e-12)
                except Exception as e:
                    rospy.logwarn_throttle(0.5, f"[bridge] EE TF lookup (init continuous) failed: {e}")
                    return
                # choose axis (AUTO or fixed) minimizing rotation from current EE
                axes_list = ["+x","-x","+y","-y","+z","-z"] if self.align_axis.lower()=="auto" else [self.align_axis.lower()]
                best = None
                for ax in axes_list:
                    q_cand, theta, _ = self._candidate_for_axis(ax, self.pos_ema, self.up_world, q_now)
                    if best is None or theta < best[1]:
                        best = (ax, theta)
                self.chosen_axis_last = best[0]
                self.catch_quat_world = q_now  # start commanding current EE

            # compute target w.r.t current command, then rate-limit
            q_tgt, theta, _ = self._candidate_for_axis(self.chosen_axis_last, self.pos_ema, self.up_world, self.catch_quat_world)
            if math.degrees(theta) > self.track_deadband_deg:
                step = min(1.0, (self.track_omega_max_rad * dt) / max(theta, 1e-6))
                q_cmd_wxyz = quaternion_slerp(self._quat_xyzw_to_wxyz(self.catch_quat_world),
                                              self._quat_xyzw_to_wxyz(q_tgt), step)
                self.catch_quat_world = self._quat_wxyz_to_xyzw(q_cmd_wxyz)
            self.orientation_locked = True  # so on_point uses catch_quat_world
            return
        # ----- END CONTINUOUS MODE -----

        # (non-continuous path keeps your previous stability/lock logic)
        dotv = float(np.clip(np.dot(self.pos_prev, self.pos_ema), -1.0, 1.0))
        ang_deg = math.degrees(math.acos(dotv))
        self.pos_prev = self.pos_ema
        if ang_deg <= self.stable_eps_deg:
            self.stable_count = min(self.stable_count + 1, self.stable_count_lock)
        elif ang_deg >= self.unstable_eps_deg:
            self.stable_count = 0
        else:
            if self.stable_count > 0: self.stable_count -= 1

        rospy.loginfo_throttle(0.05, f"stability: ang={ang_deg:.1f}° count={self.stable_count}/{self.stable_count_lock}")

        if self.stable_count >= self.stable_count_lock and not self.orientation_locked:
            try:
                T_w_ee = self.buf.lookup_transform(self.world_frame, self.ee_frame,
                                                   rospy.Time(0), rospy.Duration(0.02))
                q = T_w_ee.transform.rotation
                q_now = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
                q_now /= (np.linalg.norm(q_now)+1e-12)
            except Exception as e:
                rospy.logwarn_throttle(0.5, f"[bridge] EE TF lookup for lock failed: {e}")
                return
            self._do_lock_with_dir(self.pos_ema, q_now)
            return

        ttg = self.last_ttg if self.last_ttg is not None else 1e9
        if (not self.orientation_locked) and (ttg <= self.fallback_ttg_s):
            try:
                T_w_ee = self.buf.lookup_transform(self.world_frame, self.ee_frame,
                                                   rospy.Time(0), rospy.Duration(0.02))
                q = T_w_ee.transform.rotation
                q_now = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
                q_now /= (np.linalg.norm(q_now)+1e-12)
            except Exception as e:
                rospy.logwarn_throttle(0.5, f"[bridge] EE TF lookup for fallback lock failed: {e}")
                return
            rospy.loginfo(f"[bridge] fallback lock: ttg={ttg:.3f}s")
            self._do_lock_with_dir(self.pos_ema, q_now)

    def _do_lock_with_dir(self, dir_world, q_now_xyzw):
        ax_param = self.align_axis.lower()
        axes_list = ["+x","-x","+y","-y","+z","-z"] if ax_param=="auto" else [ax_param]
        best = None
        for ax in axes_list:
            q_cand, theta, _ = self._candidate_for_axis(ax, dir_world, self.up_world, q_now_xyzw)
            if (best is None) or (theta < best[1]):
                best = (q_cand, theta, ax)
        q_target, theta, chosen_axis = best
        self.chosen_axis_last = chosen_axis
        if ax_param=="auto":
            rospy.loginfo(f"[bridge] align_axis AUTO → chose {chosen_axis} (θ={math.degrees(theta):.1f}°)")
        ttg = self.last_ttg if self.last_ttg is not None else 1e9
        t_need = theta/max(self.omega_max,1e-6) + self.rot_margin
        if t_need > ttg:
            q_cmd_wxyz = quaternion_slerp(self._quat_xyzw_to_wxyz(q_now_xyzw),
                                          self._quat_xyzw_to_wxyz(q_target),
                                          min(1.0, max(0.0, (self.omega_max*max(ttg-self.rot_margin,0.0))/max(theta,1e-6))))
            self.catch_quat_world = self._quat_wxyz_to_xyzw(q_cmd_wxyz)
            rospy.loginfo(f"[bridge] Orientation LOCKED (slewed) θ={math.degrees(theta):.1f}° ttg={ttg:.3f}s")
        else:
            self.catch_quat_world = q_target
            rospy.loginfo(f"[bridge] Orientation LOCKED (full) θ={math.degrees(theta):.1f}° ttg={ttg:.3f}s")
        self.orientation_locked = True

    def on_point(self, p_cam: PointStamped):
        if self.last_ttg is None:
            return
        dur = float(self.last_ttg)
        if dur < self.min_duration:
            rospy.loginfo_throttle(0.0, f"[bridge] Tgo={dur:.3f}s < min {self.min_duration:.2f}s → skip")
            return

        cam_frame = p_cam.header.frame_id or self.camera_frame
        try:
            Tcw = self.buf.lookup_transform(self.world_frame, cam_frame,
                                            rospy.Time(0), rospy.Duration(0.05))
            p_world_geo = do_transform_point(p_cam, Tcw)
            p_world = p_world_geo.point
        except Exception as e:
            rospy.logwarn_throttle(0.0, f"[bridge] TF transform failed ({cam_frame}→{self.world_frame}): {e}")
            return

        # catching area
        if not (-0.14 <= p_world.y <= 0.46 and 0.52 <= p_world.z <= 1.12):
            rospy.logerr_throttle(0.0, f"[bridge] outside catching area; Tgo={dur:.3f}")
            self.old_x, self.old_y, self.old_z = p_world.x, p_world.y, p_world.z
            self.get_orientation = False
            if not self.continuous_orientation:
                self.orientation_locked = False
                self.catch_quat_world = None
                self.pos_prev = None
                self.pos_ema = None
                self.stable_count = 0
                self.chosen_axis_last = None
            return

        # distance gate
        if self.old_x is not None:
            dx = p_world.x - self.old_x; dy = p_world.y - self.old_y; dz = p_world.z - self.old_z
            step = math.sqrt(dx*dx + dy*dy + dz*dz)
            if step > self.max_step_m:
                rospy.logerr_throttle(0.0, f"[bridge] step {step:.3f} m > max {self.max_step_m:.3f} m → reject")
                self.old_x, self.old_y, self.old_z = p_world.x, p_world.y, p_world.z
                return

        # Current EE orientation for pass-through (if not using catch_quat_world yet)
        try:
            T_w_ee = self.buf.lookup_transform(self.world_frame, self.ee_frame,
                                               rospy.Time(0), rospy.Duration(0.02))
            q = T_w_ee.transform.rotation
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[bridge] EE TF lookup failed ({self.ee_frame}→{self.world_frame}): {e}")
            return

        # Publish pose
        use_q = None
        if self.catch_quat_world is not None:
            use_q = self.catch_quat_world
        else:
            use_q = np.array([q.x, q.y, q.z, q.w], float)

        stamp = p_cam.header.stamp if p_cam.header.stamp.to_sec() > 0 else rospy.Time.now()

        pto_w = PointStamped()
        pto_w.header.stamp = stamp
        pto_w.header.frame_id = self.world_frame
        pto_w.point.x, pto_w.point.y, pto_w.point.z = p_world.x, p_world.y, p_world.z
        self.pub_point.publish(pto_w)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.world_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p_world.x, p_world.y, p_world.z
        pose.pose.orientation.w = 1.0
        self.pts.append(pose);  self.pts = self.pts[-20:]
        path_msg = Path(); path_msg.header.stamp = stamp; path_msg.header.frame_id = self.world_frame
        path_msg.poses = list(self.pts)
        self.pub_path.publish(path_msg)

        msg = EETarget()
        msg.ee_target.position.x = p_world.x
        msg.ee_target.position.y = p_world.y
        msg.ee_target.position.z = p_world.z
        msg.ee_target.orientation.x = float(use_q[0])
        msg.ee_target.orientation.y = float(use_q[1])
        msg.ee_target.orientation.z = float(use_q[2])
        msg.ee_target.orientation.w = float(use_q[3])
        msg.duration = dur
        self.pub.publish(msg)

        self.old_x, self.old_y, self.old_z = p_world.x, p_world.y, p_world.z
        self.get_orientation = True

    # (kept for non-continuous mode)
    def _do_lock_with_dir(self, dir_world, q_now_xyzw):
        ax_param = self.align_axis.lower()
        axes_list = ["+x","-x","+y","-y","+z","-z"] if ax_param=="auto" else [ax_param]
        best = None
        for ax in axes_list:
            q_cand, theta, _ = self._candidate_for_axis(ax, dir_world, self.up_world, q_now_xyzw)
            if (best is None) or (theta < best[1]):
                best = (q_cand, theta, ax)
        q_target, theta, chosen_axis = best
        self.chosen_axis_last = chosen_axis
        ttg = self.last_ttg if self.last_ttg is not None else 1e9
        t_need = theta/max(self.omega_max,1e-6) + self.rot_margin
        if t_need > ttg:
            q_cmd_wxyz = quaternion_slerp(self._quat_xyzw_to_wxyz(q_now_xyzw),
                                          self._quat_xyzw_to_wxyz(q_target),
                                          min(1.0, max(0.0, (self.omega_max*max(ttg-self.rot_margin,0.0))/max(theta,1e-6))))
            self.catch_quat_world = self._quat_wxyz_to_xyzw(q_cmd_wxyz)
        else:
            self.catch_quat_world = q_target
        self.orientation_locked = True

if __name__ == "__main__":
    rospy.init_node("kf_to_ee_target_bridge")
    KFtoEETargetBridge()
    rospy.spin()
