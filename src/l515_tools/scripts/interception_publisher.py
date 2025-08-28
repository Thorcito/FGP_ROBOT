#!/usr/bin/env python3
import rospy, math
from tum_ics_ur10_controller_tutorial.msg import EETarget
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import Float32

import tf2_ros
from tf2_geometry_msgs import do_transform_point

class KFtoEETargetBridge:
    def __init__(self):
        # -------- params --------
        self.position_topic = rospy.get_param("~position_topic", "/ball_pred/hit_point")
        self.ttg_topic      = rospy.get_param("~t_to_int_topic", "/ball_pred/hit_time_s")
        self.world_frame    = rospy.get_param("~world_frame", "world")
        self.fixed_q_xyzw   = rospy.get_param("~fixed_orientation_xyzw",
                                              [0.296595 , 0.206707 , -0.529188 , -0.767635])
          

        # Settling gate: publish only when two consecutive points are close
        self.stable_thresh_m = rospy.get_param("~stable_thresh_m", 0.3)  # 30cm cm default

        # Camera TF params (publish a static TF here)
        self.publish_camera_tf = rospy.get_param("~publish_camera_tf", True)
        self.camera_parent     = rospy.get_param("~camera_parent", "world")
        self.camera_frame      = rospy.get_param("~camera_frame", "camera_color_optical_frame")
        self.cam_xyz           = rospy.get_param("~camera_xyz", [0.5, 0.0, 1.0])             # meters
        self.cam_quat_xyzw     = rospy.get_param("~camera_quat_xyzw", [-0.5, -0.5, 0.5, 0.5])# x y z w

        # -------- TF2 --------
        self.buf = tf2_ros.Buffer(cache_time=rospy.Duration(2.0))
        self.listener = tf2_ros.TransformListener(self.buf)

        if self.publish_camera_tf:
            self._publish_static_camera_tf()

        self._wait_for_tf(self.world_frame, self.camera_frame, timeout=1.0)

        # -------- pubs/subs --------
        self.pub = rospy.Publisher("ee_target", EETarget, queue_size=1)
        rospy.Subscriber(self.position_topic, PointStamped, self.on_point, queue_size=10)
        rospy.Subscriber(self.ttg_topic, Float32, self.on_ttg, queue_size=10)

        # -------- state --------
        self.last_ttg = None
        self.prev_world_xyz = None  # for settling gate

        rospy.loginfo(f"[bridge] stable_thresh_m={self.stable_thresh_m:.3f} m")

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

    def _wait_for_tf(self, target, source, timeout=1.0):
        try:
            ok = self.buf.can_transform(target, source, rospy.Time(0), rospy.Duration(timeout))
            if not ok:
                rospy.logwarn(f"[bridge] TF {target} <- {source} not available after {timeout}s")
        except Exception:
            pass

    def on_ttg(self, msg: Float32):
        self.last_ttg = float(msg.data)

    def on_point(self, p_cam: PointStamped):
        # need duration to form a spline
        if self.last_ttg is None:
            return

        cam_frame = p_cam.header.frame_id or self.camera_frame
        try:
            # camera -> world; Time(0) is fine for static camera
            Tcw = self.buf.lookup_transform(self.world_frame, cam_frame,
                                            rospy.Time(0), rospy.Duration(0.05))
            p_world = do_transform_point(p_cam, Tcw).point
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[bridge] TF transform failed ({cam_frame}→{self.world_frame}): {e}")
            return

        xyz = (p_world.x, p_world.y, p_world.z)

        # ---- settling gate ----
        if self.prev_world_xyz is None:
            # First measurement: store only
            self.prev_world_xyz = xyz
            rospy.loginfo("[bridge] Settling: stored first measurement, not publishing yet.")
            return

        # Compare new vs previous; only publish if close (stable)
        dx = xyz[0] - self.prev_world_xyz[0]
        dy = xyz[1] - self.prev_world_xyz[1]
        dz = xyz[2] - self.prev_world_xyz[2]
        dist = math.sqrt(dy*dy + dz*dz)
        rospy.logerr(dist)

        if dist > self.stable_thresh_m:
            # Too different → update reference and wait for next close sample
            self.prev_world_xyz = xyz
            rospy.loginfo_throttle(0.0, f"Settling: Δ={dist:.3f} m > {self.stable_thresh_m:.3f} m → "
                                        f"update reference, skip publish.")
            return

        # Stable → publish and update reference
        self.prev_world_xyz = xyz

        msg = EETarget()
        msg.ee_target.position.x = xyz[0]
        msg.ee_target.position.y = xyz[1]
        msg.ee_target.position.z = xyz[2]
        msg.ee_target.orientation.x = float(self.fixed_q_xyzw[0])
        msg.ee_target.orientation.y = float(self.fixed_q_xyzw[1])
        msg.ee_target.orientation.z = float(self.fixed_q_xyzw[2])
        msg.ee_target.orientation.w = float(self.fixed_q_xyzw[3])
        msg.duration = float(self.last_ttg)
        self.pub.publish(msg)

        rospy.loginfo_throttle(0.0, f"PUBLISHED{self.world_frame}: "
                                    f"{xyz:.3f}  dur={self.last_ttg:.3f}s")

if __name__ == "__main__":
    rospy.init_node("kf_to_ee_target_bridge")
    KFtoEETargetBridge()
    rospy.spin()
