#!/usr/bin/env python3
import rospy, math
from tum_ics_ur10_controller_tutorial.msg import EETarget
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped, Vector3Stamped
from std_msgs.msg import Float32, Header
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import numpy as np

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from tf.transformations import quaternion_matrix
import message_filters

class Bridge_Plane_Interception:
    def __init__(self):
        # -------- params --------
        self.position_topic = rospy.get_param("~position_topic", "/ball_pred/point_filt")
        self.velocity_topic = rospy.get_param("~velocity_topic", "/ball_pred/current_vel")

        self.world_frame    = rospy.get_param("~world_frame", "world")
        self.ee_frame       = rospy.get_param("~ee_frame", "ur10_model_dh_5")

        # Gravity in world (+Z up)
        self.g_world = np.array(rospy.get_param("~g_world", [0.0, 0.0, -9.81]), dtype=np.float64)


        # Camera TF
        self.publish_camera_tf = rospy.get_param("~publish_camera_tf", True)
        self.camera_parent     = rospy.get_param("~camera_parent", "world")
        self.camera_frame      = rospy.get_param("~camera_frame", "camera_color_optical_frame")
        self.cam_xyz           = rospy.get_param("~camera_xyz", [-0.155, 0.0, -0.45])
        self.cam_quat_xyzw     = rospy.get_param("~camera_quat_xyzw", [-0.5, -0.5, 0.5, 0.5])  # x y z w
        self.rot_angle = rospy.get_param("~rot_angle", 47)

        # Orientation tracking
        self.vel_min_speed = float(rospy.get_param("~vel_min_speed_mps", 0.6))
        self.up_world = np.array(rospy.get_param("~up_world", [0.0, 0.0, 1.0]), dtype=np.float64)
        self.dir_sign = -1.0
        self.dir_alpha = float(rospy.get_param("~dir_alpha", 0.5))

        #Plane of interception
        self.x_plane_m  = float(rospy.get_param("~x_plane_m", -0.7))
        self.hit_horizon_s = float(rospy.get_param("~hit_horizon_s", 1.3))
        self.hit_history_max = int(rospy.get_param("~hit_history_max", 20))

        # -------- state --------
        self.pos_ema = None
        self.hit_history = []

        # -------- TF2 --------
        self.buf = tf2_ros.Buffer(cache_time=rospy.Duration(2.0))
        self.listener = tf2_ros.TransformListener(self.buf)

        # -------- pubs --------
        self.pub = rospy.Publisher("ee_target", EETarget, queue_size=1)
        self.pub_hit_point = rospy.Publisher("/Bridge_PlaneInt/hit_point", PointStamped, queue_size=10)
        self.pub_hit_time  = rospy.Publisher("/Bridge_PlaneInt/hit_time_s", Float32, queue_size=10)
        self.pub_hit_history = rospy.Publisher("/Bridge_PlaneInt/hit_history", Path, queue_size=10)
        self.pub_markers_static = rospy.Publisher("/Bridge_PlaneInt/static_plane", Marker, queue_size=1, latch=True)
        self.pub_pred_vel_hit = rospy.Publisher("/Bridge_PlaneInt/pred_vel_hit", Vector3Stamped, queue_size=10)

        if self.publish_camera_tf:
            self._publish_static_camera_tf()
        self.publish_static_plane_markers()

        # -------- exact time sync subscribers --------
        # Use exact timestamps to pair position and velocity from the predictor.
        pos_sub = message_filters.Subscriber(self.position_topic, PointStamped)
        vel_sub = message_filters.Subscriber(self.velocity_topic, Vector3Stamped)
        ts = message_filters.TimeSynchronizer([pos_sub, vel_sub], queue_size=25)
        ts.registerCallback(self.on_pair)

        rospy.loginfo(f"[bridge_planeIntercept] ready")

    # ---------- helpers ----------
    def _publish_static_camera_tf(self):
        """Optionally publish a static transform for the camera (parent -> camera_frame)."""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.camera_parent
        t.child_frame_id  = self.camera_frame
        t.transform.translation.x = float(self.cam_xyz[0])
        t.transform.translation.y = float(self.cam_xyz[1])
        t.transform.translation.z = float(self.cam_xyz[2])
        q1x, q1y, q1z, q1w = self.cam_quat_xyzw
        q2x, q2y, q2z, q2w = np.array([np.sin(np.radians(self.rot_angle)/2), 0, 0 , np.cos(np.radians(self.rot_angle)/2)])
        qx = q1w*q2x + q1x*q2w + q1y*q2z - q1z*q2y
        qy = q1w*q2y - q1x*q2z + q1y*q2w + q1z*q2x
        qz = q1w*q2z + q1x*q2y - q1y*q2x + q1z*q2w
        qw = q1w*q2w - q1x*q2x - q1y*q2y - q1z*q2z
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_broadcaster.sendTransform(t)

    @staticmethod
    def _unit(v):
        """Safe unit vector helper (avoids division by zero)."""
        n = float(np.linalg.norm(v))
        return v / (n + 1e-12)

    # --- orientation helpers (unchanged) ---
    def _R_from_quat_xyzw(self, q):
        """Quaternion [x,y,z,w] → rotation matrix."""
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
        """Rotation matrix → normalized quaternion [x,y,z,w]."""
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
        """Rodrigues' rotation formula for rotation about axis k by angle phi."""
        k = self._unit(k)
        K = np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]], dtype=np.float64)
        I = np.eye(3)
        return I*math.cos(phi) + math.sin(phi)*K + (1-math.cos(phi))*np.outer(k,k)

    def _frame_from_dir_with_up_z(self, dir_world, up_world):
        """
        Build a tool frame with +Z aligned to dir_world and roll stabilized by up_world.
        Returns R whose columns are the tool axes in WORLD (col 2 is +Z).
        """
        z = self._unit(dir_world)
        up = self._unit(up_world)
        if abs(np.dot(z, up)) > 0.98:
            up = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        x = self._unit(np.cross(up, z))
        y = np.cross(z, x)
        return np.column_stack([x, y, z])

    def _candidate_for_z(self, dir_world, up_world, q_now_xyzw):
        """
        Choose between minimal-roll and 180°-roll solutions, relative to current EE pose.
        Returns (best_quaternion_xyzw, angle_to_current).
        """
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
    
    def publish_static_plane_markers(self):
        """
        Publish a translucent CUBE marker representing the active intercept plane.
        Only the plane matching self.plane_mode is published (latching publisher).
        """
        PLANE_THICKNESS = 0.005
        YZ_SIZE_Y = 4.0   # span along world Y
        YZ_SIZE_Z = 4.0   # span along world Z
        PLANE_ALPHA = 0.3
        m = Marker()
        m.header.frame_id = self.world_frame
        m.header.stamp = rospy.Time.now()
        m.ns = "planes"; m.id = 1
        m.type = Marker.CUBE; m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.pose.position.x = float(self.x_plane_m)
        m.pose.position.y = 0.0
        m.pose.position.z = 0.0
        m.scale.x = max(PLANE_THICKNESS, 1e-4)   # thin along X
        m.scale.y = YZ_SIZE_Y
        m.scale.z = YZ_SIZE_Z
        m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 1.0, 0.0, PLANE_ALPHA)
        self.pub_markers_static.publish(m)
    
    # ---------- paired callback ----------
    def on_pair(self, p_cam: PointStamped, v_cam: Vector3Stamped):
        cam_frame = self.camera_frame
        stamp = p_cam.header.stamp
        hdr = Header(stamp=stamp, frame_id=self.world_frame)

        # 1) Transform velocity (vector: rotation only) to WORLD at the SAME stamp
        try:
            Tcw = self.buf.lookup_transform(self.world_frame, cam_frame, stamp, rospy.Duration(0.05))
            q = Tcw.transform.rotation
            R_w_c = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            v_cam_np = np.array([v_cam.vector.x, v_cam.vector.y, v_cam.vector.z], dtype=np.float64)
            v_world = R_w_c.dot(v_cam_np)
        except Exception as e:
            rospy.logwarn_throttle(0.5, f"[bridge] v_world TF rot failed: {e}")
            return

        if np.linalg.norm(v_world) < self.vel_min_speed:
            return

        # Current EE pose
        try:
            T_w_ee = self.buf.lookup_transform(self.world_frame, self.ee_frame,
                                               rospy.Time(0), rospy.Duration(0.02))
            qee = T_w_ee.transform.rotation
            q_ee = np.array([qee.x, qee.y, qee.z, qee.w], dtype=np.float64)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[bridge_minDist] EE TF lookup failed ({self.ee_frame}→{self.world_frame}): {e}")
            return
        
        # Transform camera → world for filtered point
        try:
            Tcw_same = Tcw
            p_world_geo = do_transform_point(p_cam, Tcw_same)
            p_world = p_world_geo.point
            point = np.array([p_world.x, p_world.y, p_world.z], dtype=np.float64)
        except Exception as e:
            rospy.logwarn_throttle(0.0, f"[bridge] TF transform failed ({cam_frame}→{self.world_frame}): {e}")
            return

        # Plane hits
        a_x = 0.5 * self.g_world[0]
        b_x = v_world[0]
        c_x = point[0] - self.x_plane_m
        t_hit = self.solve_time_to_plane_1d(a_x, b_x, c_x, t_min=0.0, t_max=self.hit_horizon_s)
        if t_hit is None:
            return
        p_hit = point + v_world*t_hit + 0.5*self.g_world*(t_hit*t_hit)
        v_hit_world = v_world + self.g_world * t_hit

        # Publish
        if (t_hit is not None) and (p_hit is not None):
            pt_h = PointStamped()
            pt_h.header = hdr
            pt_h.point.x, pt_h.point.y, pt_h.point.z = float(p_hit[0]), float(p_hit[1]), float(p_hit[2])
            self.pub_hit_point.publish(pt_h)
            self.pub_hit_time.publish(Float32(data=float(t_hit)))
            rospy.loginfo_throttle(0.0, f"Time to intercept: (t={t_hit:.6f})")

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

            vel_hit_msg = Vector3Stamped()
            vel_hit_msg.header = hdr
            vel_hit_msg.vector.x, vel_hit_msg.vector.y, vel_hit_msg.vector.z = v_hit_world.tolist()
            self.pub_pred_vel_hit.publish(vel_hit_msg)

            # EMA direction (fixed sign = -1 → face ball)
            p_raw = self._unit(self.dir_sign * v_hit_world)
            if self.pos_ema is None:
                self.pos_ema = p_raw
            self.pos_ema = self._unit((1.0 - self.dir_alpha) * self.pos_ema + self.dir_alpha * p_raw)

            # Choose orientation for this publish (EMA-based if available; else current EE)
            if self.pos_ema is not None:
                q_tgt_xyzw, _ = self._candidate_for_z(self.pos_ema, self.up_world, q_ee)
                use_q = q_tgt_xyzw
            else:
                use_q = q_ee

            # Controller command: EETarget uses Optimization 2 result (as in your code)
            msg = EETarget()
            msg.ee_target.position.x = pt_h.point.x
            msg.ee_target.position.y = pt_h.point.y
            msg.ee_target.position.z = pt_h.point.z
            msg.ee_target.orientation.x = float(use_q[0])
            msg.ee_target.orientation.y = float(use_q[1])
            msg.ee_target.orientation.z = float(use_q[2])
            msg.ee_target.orientation.w = float(use_q[3])
            msg.duration = t_hit
            self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("Bridge_Plane_Interception")
    Bridge_Plane_Interception()
    rospy.spin()