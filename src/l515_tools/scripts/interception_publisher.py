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
from tf.transformations import quaternion_from_matrix

class KFtoEETargetBridge:
    def __init__(self):
        # -------- params --------
        self.position_topic = rospy.get_param("~position_topic", "/ball_pred/hit_point")
        self.velocity_topic = rospy.get_param("~velocity_topic", "/ball_pred/pred_vel_hit")
        self.ttg_topic      = rospy.get_param("~t_to_int_topic", "/ball_pred/hit_time_s")
        self.world_frame    = rospy.get_param("~world_frame", "world")   # controller's B/world
        self.ee_frame       = rospy.get_param("~ee_frame", "ur10_model_dh_5")      

        self.pts = []
        self.min_duration   = rospy.get_param("~min_duration_s", 0.1)

        # Camera TF params (publish a static TF here)
        self.publish_camera_tf = rospy.get_param("~publish_camera_tf", True)
        self.camera_parent     = rospy.get_param("~camera_parent", "world")
        self.camera_frame      = rospy.get_param("~camera_frame", "camera_color_optical_frame")
        self.cam_xyz           = rospy.get_param("~camera_xyz", [0.05, 0.0, 1.0])             # meters
        self.cam_quat_xyzw     = rospy.get_param("~camera_quat_xyzw", [-0.5, -0.5, 0.5, 0.5])# x y z w

        self.max_step_m    = rospy.get_param("~max_step_m", 0.13)  # 10 cm gate between accepted points

        #ORIENTATION 
        # --- params ---
        self.vel_angle_deg_thresh = float(rospy.get_param("~vel_angle_deg_thresh", 8.0))
        self.vel_min_speed = float(rospy.get_param("~vel_min_speed_mps", 0.4))
        self.vel_stable_count_needed = int(rospy.get_param("~vel_stable_count", 3))
        self.align_axis = rospy.get_param("~align_axis", "-z")  # options: +x,-x,+y,-y,+z,-z

        # --- state ---
        self.v_world_prev = None
        self.v_world_stable_count = 0
        self.orientation_locked = False    # becomes True after we switch to velocity orientation
        self.catch_quat_world = None       # quaternion we’ll publish once locked


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
        self.old_x = None
        self.old_y = None
        self.old_z = None
        
        rospy.loginfo(f"[bridge] min_duration_s={self.min_duration:.2f}  ee_frame={self.ee_frame}")

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
        rospy.loginfo(f"[bridge] Published static TF {self.camera_parent} -> {self.camera_frame} "
                      f"xyz={self.cam_xyz} quat={self.cam_quat_xyzw}")
        
    def _rotate_vec_by_tf(self, v_cam, world_frame, cam_frame):
        # get rotation only
        Tcw = self.buf.lookup_transform(world_frame, cam_frame, rospy.Time(0), rospy.Duration(0.05))
        q = Tcw.transform.rotation
        # rotate v_cam by q (world←camera)
        # Convert to numpy
        qc = np.array([q.w, q.x, q.y, q.z], dtype=np.float64)
        # v' = q * v * q_conj
        # expand manually:
        x, y, z = v_cam
        qw, qx, qy, qz = qc
        # q*v as pure quaternion
        # Efficient vector rotation:
        t = 2.0 * np.cross([qx, qy, qz], [x, y, z])
        v_world = [x, y, z] + qw * t + np.cross([qx, qy, qz], t)
        return np.array(v_world, dtype=np.float64)
    
    def _quat_align_axis_to_vector(self, axis_str, v_target_world):
        # axis_str in {+x,-x,+y,-y,+z,-z}; returns wxyz quaternion
        A = {
            "+x": np.array([1,0,0.],float), "-x": np.array([-1,0,0.],float),
            "+y": np.array([0,1,0.],float), "-y": np.array([0,-1,0.],float),
            "+z": np.array([0,0,1.],float), "-z": np.array([0,0,-1.],float),
        }[axis_str.lower()]
        b = v_target_world / (np.linalg.norm(v_target_world) + 1e-12)
        a = A / (np.linalg.norm(A) + 1e-12)

        # Handle near-parallel and anti-parallel robustly
        dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
        if dot > 1.0 - 1e-9:
            # already aligned
            return np.array([1.0, 0.0, 0.0, 0.0], float)  # identity (wxyz)
        if dot < -1.0 + 1e-9:
            # 180°: choose any orthogonal axis
            ortho = np.array([1,0,0.],float) if abs(a[0]) < 0.9 else np.array([0,1,0.],float)
            rot_axis = np.cross(a, ortho); rot_axis /= (np.linalg.norm(rot_axis)+1e-12)
            return np.array([0.0, rot_axis[0], rot_axis[1], rot_axis[2]], float)  # 180° (w=0)
        # general case
        rot_axis = np.cross(a, b); s = np.linalg.norm(rot_axis)
        rot_axis /= (s + 1e-12)
        angle = math.acos(dot)
        half = 0.5*angle
        return np.array([math.cos(half), *(rot_axis*math.sin(half))], float)

    def on_ttg(self, msg: Float32):
        self.last_ttg = float(msg.data)

    def on_vhit(self, msg: Vector3Stamped):
        if self.orientation_locked:
            return  # we already decided the catch orientation

        cam_frame = msg.header.frame_id or self.camera_frame
        try:
            v_cam = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=np.float64)
            v_world = self._rotate_vec_by_tf(v_cam, self.world_frame, cam_frame)
        except Exception as e:
            rospy.logwarn_throttle(0.5, f"[bridge] vhit TF rot failed: {e}")
            return

        speed = float(np.linalg.norm(v_world))
        if speed < self.vel_min_speed:
            self.v_world_prev = v_world
            self.v_world_stable_count = 0
            return

        if self.v_world_prev is None:
            self.v_world_prev = v_world
            self.v_world_stable_count = 1
            return

        # stability check
        dot = float(np.dot(v_world, self.v_world_prev) / (np.linalg.norm(v_world)*np.linalg.norm(self.v_world_prev)+1e-12))
        dot = max(-1.0, min(1.0, dot))
        thresh_dot = math.cos(math.radians(self.vel_angle_deg_thresh))
        rospy.logwarn_throttle(0.0, f"[bridge] AngleRef: ({thresh_dot}→ AngleMes: {dot})")
        if dot >= thresh_dot:
            self.v_world_stable_count += 1
        else:
            self.v_world_stable_count = 0

        self.v_world_prev = v_world

        if self.v_world_stable_count >= self.vel_stable_count_needed:
            # lock orientation once
            q_wxyz = self._quat_align_axis_to_vector(self.align_axis, v_world)
            # store as geometry-msg-style (x,y,z,w) for publishing
            self.catch_quat_world = np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], float)
            self.orientation_locked = True
            rospy.loginfo(f"[bridge] catch orientation locked from velocity (axis {self.align_axis}, "
                        f"angle_thresh={self.vel_angle_deg_thresh}°, N={self.vel_stable_count_needed})")


    def on_point(self, p_cam: PointStamped):
        # need duration to form a spline
        if self.last_ttg is None:
            return

        dur = float(self.last_ttg)
        if dur < self.min_duration:
            rospy.loginfo_throttle(0.0, f"[bridge] Tgo={dur:.3f}s < min {self.min_duration:.2f}s → skip")
            return

        cam_frame = p_cam.header.frame_id or self.camera_frame

        # --- Transform camera -> world ---
        try:
            Tcw = self.buf.lookup_transform(self.world_frame, cam_frame,
                                            rospy.Time(0), rospy.Duration(0.05))
            p_world_geo = do_transform_point(p_cam, Tcw)
            p_world = p_world_geo.point
        except Exception as e:
            rospy.logwarn_throttle(0.0, f"[bridge] TF transform failed ({cam_frame}→{self.world_frame}): {e}")
            return

        # --- Gate by catching area---
        if not (-0.14 <= p_world.y <= 0.46 and 0.52 <= p_world.z <= 1.12):
            rospy.logerr_throttle(0.0, f"[bridge] outside catching area; Tgo={dur:.3f}")
            self.old_x, self.old_y, self.old_z = p_world.x, p_world.y, p_world.z
            self.orientation_locked = False
            self.catch_quat_world = None
            self.v_world_prev = None
            self.v_world_stable_count = 0
            return

        # --- Distance gate to avoid big jumps between measurements ---
        if self.old_x is not None:
            dx = p_world.x - self.old_x
            dy = p_world.y - self.old_y
            dz = p_world.z - self.old_z
            step = math.sqrt(dx*dx + dy*dy + dz*dz)
            if step > self.max_step_m:
                rospy.logwarn_throttle(0.0, f"[bridge] step {step:.3f} m > max {self.max_step_m:.3f} m → reject")
                self.old_x, self.old_y, self.old_z = p_world.x, p_world.y, p_world.z
                self.orientation_locked = False
                self.catch_quat_world = None
                self.v_world_prev = None
                self.v_world_stable_count = 0
                return
        # else: first point, no gating

        # --- Get current EE orientation for the message ---
        try:
            T_w_ee = self.buf.lookup_transform(self.world_frame, self.ee_frame,
                                            rospy.Time(0), rospy.Duration(0.02))
            q = T_w_ee.transform.rotation
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[bridge] EE TF lookup failed ({self.ee_frame}→{self.world_frame}): {e}")
            return
        
        use_q = None
        if self.orientation_locked and self.catch_quat_world is not None:
            # use locked velocity-based orientation
            use_q = self.catch_quat_world
        else:
            # use current EE orientation (world←ee)
            use_q = np.array([q.x, q.y, q.z, q.w], float)

        # --- Publish accepted point + path (only accepted points make the path) ---
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
        if len(self.pts) > 20:
            self.pts = self.pts[-20:]
        path_msg = Path()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self.world_frame
        path_msg.poses = list(self.pts)
        self.pub_path.publish(path_msg)

        # --- Build and publish EETarget ---
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

        rospy.loginfo_throttle(
            0.0,
            f"[bridge] → ee_target @{self.world_frame}: ({p_world.y:.3f},{p_world.z:.3f}), "
            f"({use_q[0]:.3f}, {use_q[1]:.3f}, {use_q[2]:.3f}, {use_q[3]:.3f}) "
            f"dur={dur:.3f}s"
        )


if __name__ == "__main__":
    rospy.init_node("kf_to_ee_target_bridge")
    KFtoEETargetBridge()
    rospy.spin()
