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


class KFtoEETargetBridge:
    def __init__(self):
        # -------- params --------
        self.position_topic = rospy.get_param("~position_topic", "/ball_pred/hit_point")
        self.velocity_topic = rospy.get_param("~velocity_topic", "/ball_pred/current_vel")
        self.ttg_topic      = rospy.get_param("~t_to_int_topic", "/ball_pred/hit_time_s")

        self.world_frame    = rospy.get_param("~world_frame", "world")
        self.ee_frame       = rospy.get_param("~ee_frame", "ur10_model_dh_5")

        self.min_duration   = rospy.get_param("~min_duration_s", 0.1)
        self.max_step_m     = rospy.get_param("~max_step_m", 0.13)

        # Camera TF (optional)
        self.publish_camera_tf = rospy.get_param("~publish_camera_tf", True)
        self.camera_parent     = rospy.get_param("~camera_parent", "world")
        self.camera_frame      = rospy.get_param("~camera_frame", "camera_color_optical_frame")
        self.cam_xyz           = rospy.get_param("~camera_xyz", [0.05, 0.0, 1.0])
        self.cam_quat_xyzw     = rospy.get_param("~camera_quat_xyzw", [-0.5, -0.5, 0.5, 0.5])  # x y z w

        # Orientation tracking (continuous)
        self.vel_min_speed = float(rospy.get_param("~vel_min_speed_mps", 1.0))
        self.up_world = np.array(rospy.get_param("~up_world", [0.0, 0.0, 1.0]), dtype=np.float64)
        self.dir_sign = -1.0  # fixed: face the ball (your validated setting)
        self.dir_alpha = float(rospy.get_param("~dir_alpha", 0.5))  # EMA on direction

        # Resets to avoid stale state between throws
        self.reset_on_outside = bool(rospy.get_param("~reset_on_outside", True))
        self.reset_on_big_step = bool(rospy.get_param("~reset_on_big_step", True))

        # -------- state --------
        self.pos_ema = None            # filtered direction

        self.last_ttg = None
        self.old_x = self.old_y = self.old_z = None
        self.get_orientation = False   # start tracking only after first accepted point
        self.pts = []

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

        rospy.loginfo(f"[bridge] ready  frame={self.world_frame}  ee={self.ee_frame}  min_dur={self.min_duration:.2f}s")

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

    def _rot_about_axis_matrix(self, k, phi): #Rodriguey formula
        k = self._unit(k)
        K = np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]], dtype=np.float64)
        I = np.eye(3)
        return I*math.cos(phi) + math.sin(phi)*K + (1-math.cos(phi))*np.outer(k,k)

    # Fixed-axis (+Z) frame builder with roll stabilization vs up_world
    def _frame_from_dir_with_up_z(self, dir_world, up_world):
        z = self._unit(dir_world)
        up = self._unit(up_world)
        if abs(np.dot(z, up)) > 0.98:
            up = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        x = self._unit(np.cross(up, z))  # lateral
        y = np.cross(z, x)
        R = np.column_stack([x, y, z])   # columns are world axes of tool
        return R  # axis_idx for +Z is 2

    # Choose roll that minimizes rotation from current orientation; also try 180° roll
    def _candidate_for_z(self, dir_world, up_world, q_now_xyzw):
        R_now = self._R_from_quat_xyzw(q_now_xyzw)
        R_base = self._frame_from_dir_with_up_z(dir_world, up_world)

        # minimize roll about Z
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

        # also consider 180° roll (flip x,y)
        R_flip = R_base.copy()
        R_flip[:, 0] *= -1.0
        R_flip[:, 1] *= -1.0
        x0f = R_flip[:, 0]
        s2 = np.dot(z, np.cross(x0f, x_p))
        c2 = np.dot(x0f, x_p)
        phi2 = math.atan2(s2, c2)
        R_opt2 = self._rot_about_axis_matrix(z, phi2) @ R_flip

        q_now = self._quat_from_R(R_now)
        q1 = self._quat_from_R(R_opt)
        q2 = self._quat_from_R(R_opt2)

        def quat_angle(qA, qB):
            dot = float(np.dot(qA, qB))
            dot = abs(max(min(dot, 1.0), -1.0))
            return 2.0 * math.acos(dot)

        return (q1, quat_angle(q_now, q1)) if quat_angle(q_now, q1) <= quat_angle(q_now, q2) else (q2, quat_angle(q_now, q2))

    def _reset_orientation_filter(self, why=""):
        self.pos_ema = None
        if why:
            rospy.logerr(f"[bridge] reset orientation ({why})")

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

        # EMA direction (fixed sign = -1 → face ball)
        p_raw = self._unit(self.dir_sign * v_world)
        if self.pos_ema is None:
            self.pos_ema = p_raw
        self.pos_ema = self._unit((1.0 - self.dir_alpha) * self.pos_ema + self.dir_alpha * p_raw)

    def on_point(self, p_cam: PointStamped):
        if self.last_ttg is None:
            return

        dur = float(self.last_ttg)
        if dur < self.min_duration:
            rospy.logerr_throttle(0.0, f"[bridge] Tgo={dur:.3f}s < min {self.min_duration:.2f}s → skip")
            return

        cam_frame = p_cam.header.frame_id or self.camera_frame

        # Transform camera → world
        try:
            Tcw = self.buf.lookup_transform(self.world_frame, cam_frame,
                                            rospy.Time(0), rospy.Duration(0.05))
            p_world_geo = do_transform_point(p_cam, Tcw)
            p_world = p_world_geo.point
        except Exception as e:
            rospy.logwarn_throttle(0.0, f"[bridge] TF transform failed ({cam_frame}→{self.world_frame}): {e}")
            return

        # Catching area gate
        if not (-0.14 <= p_world.y <= 0.46 and 0.52 <= p_world.z <= 1.12):
            rospy.logerr_throttle(0.0, f"[bridge] outside catching area; Tgo={dur:.3f}")
            self.old_x, self.old_y, self.old_z = p_world.x, p_world.y, p_world.z
            self.get_orientation = False
            if self.reset_on_outside:
                self._reset_orientation_filter("outside area")
            return

        # Distance gate
        if self.old_x is not None:
            dx = p_world.x - self.old_x
            dy = p_world.y - self.old_y
            dz = p_world.z - self.old_z
            step = math.sqrt(dx*dx + dy*dy + dz*dz)
            if step > self.max_step_m:
                rospy.logerr_throttle(0.0, f"[bridge] step {step:.3f} m > max {self.max_step_m:.3f} m → reject")
                self.old_x, self.old_y, self.old_z = p_world.x, p_world.y, p_world.z
                if self.reset_on_big_step:
                    self._reset_orientation_filter(f"big step {step:.3f} m")
                return

        # Current EE orientation (for logging / pass-through if needed)
        try:
            T_w_ee = self.buf.lookup_transform(self.world_frame, self.ee_frame,
                                               rospy.Time(0), rospy.Duration(0.02))
            qee = T_w_ee.transform.rotation
            q_ee = np.array([qee.x, qee.y, qee.z, qee.w], dtype=np.float64)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[bridge] EE TF lookup failed ({self.ee_frame}→{self.world_frame}): {e}")
            return

        # Choose orientation for this publish
        if self.pos_ema is not None:
            q_tgt_xyzw, _ = self._candidate_for_z(self.pos_ema, self.up_world, q_ee)
            use_q = q_tgt_xyzw
        else:
            use_q = q_ee

        # Publish point & short trail (kept for RViz)
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
        self.pts.append(pose)
        if len(self.pts) > 20: self.pts = self.pts[-20:]
        path_msg = Path(); path_msg.header.stamp = stamp; path_msg.header.frame_id = self.world_frame
        path_msg.poses = list(self.pts)
        self.pub_path.publish(path_msg)

        # Publish EETarget
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

        # Update state & single critical log
        self.old_x, self.old_y, self.old_z = p_world.x, p_world.y, p_world.z
        self.get_orientation = True

        rospy.loginfo_throttle(
            0.0,
            f"[bridge] → ee_target @{self.world_frame}: "
            f"p=({p_world.x:+.3f},{p_world.y:+.3f},{p_world.z:+.3f}) "
            f"q=({use_q[0]:+.3f},{use_q[1]:+.3f},{use_q[2]:+.3f},{use_q[3]:+.3f}) "
            #f"q_ee=({q_ee[0]:+.3f},{q_ee[1]:+.3f},{q_ee[2]:+.3f},{q_ee[3]:+.3f}) "
            f"dur={dur:.3f}s"
        )


if __name__ == "__main__":
    rospy.init_node("kf_to_ee_target_bridge")
    KFtoEETargetBridge()
    rospy.spin()
