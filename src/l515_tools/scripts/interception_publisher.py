#!/usr/bin/env python3
import rospy, math
from tum_ics_ur10_controller_tutorial.msg import EETarget
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped
from std_msgs.msg import Float32
from nav_msgs.msg import Path

import tf2_ros
from tf2_geometry_msgs import do_transform_point

class KFtoEETargetBridge:
    def __init__(self):
        # -------- params --------
        self.position_topic = rospy.get_param("~position_topic", "/ball_pred/hit_point")
        self.ttg_topic      = rospy.get_param("~t_to_int_topic", "/ball_pred/hit_time_s")
        self.world_frame    = rospy.get_param("~world_frame", "world")   # controller's B/world
        self.ee_frame       = rospy.get_param("~ee_frame", "ur10_model_dh_5")      

        self.pts = []
        self.min_duration   = rospy.get_param("~min_duration_s", 0.05)

        # Camera TF params (publish a static TF here)
        self.publish_camera_tf = rospy.get_param("~publish_camera_tf", True)
        self.camera_parent     = rospy.get_param("~camera_parent", "world")
        self.camera_frame      = rospy.get_param("~camera_frame", "camera_color_optical_frame")
        self.cam_xyz           = rospy.get_param("~camera_xyz", [0.05, 0.0, 1.0])             # meters
        self.cam_quat_xyzw     = rospy.get_param("~camera_quat_xyzw", [-0.5, -0.5, 0.5, 0.5])# x y z w

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
        rospy.Subscriber(self.ttg_topic, Float32, self.on_ttg, queue_size=10)

        self.last_ttg = None

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

    def on_ttg(self, msg: Float32):
        self.last_ttg = float(msg.data)

    def on_point(self, p_cam: PointStamped):
        # need duration to form a spline
        if self.last_ttg is None:
            return

        dur = float(self.last_ttg)
        if dur < self.min_duration:
            rospy.loginfo_throttle(0.0, f"[bridge] Tgo={dur:.3f}s < min {self.min_duration:.2f}s → skip")
            return

        cam_frame = p_cam.header.frame_id or self.camera_frame
        try:
            # camera -> world; Time(0) ok for static camera
            Tcw = self.buf.lookup_transform(self.world_frame, cam_frame,
                                            rospy.Time(0), rospy.Duration(0.05))
            p_world = do_transform_point(p_cam, Tcw).point
        except Exception as e:
            rospy.logwarn_throttle(0.0, f"[bridge] TF transform failed ({cam_frame}→{self.world_frame}): {e}")
            return
        
        stamp = p_cam.header.stamp if p_cam.header.stamp.to_sec() > 0 else rospy.Time.now()
        pto_w = PointStamped()
        pto_w.header.stamp = stamp
        pto_w.header.frame_id = self.world_frame
        pto_w.point.x = p_world.x
        pto_w.point.y = p_world.y
        pto_w.point.z = p_world.z
        self.pub_point.publish(pto_w)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.world_frame
        pose.pose.position.x = p_world.x
        pose.pose.position.y = p_world.y
        pose.pose.position.z = p_world.z
        pose.pose.orientation.w = 1.0
        self.pts.append(pose)
        if len(self.pts) > 20:
            self.pts = self.pts[-20:]
        path_msg = Path()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self.world_frame
        path_msg.poses = list(self.pts)
        self.pub_path.publish(path_msg)

        #Confirms is inside the circle before publihsin
        try:
            T_w_ee = self.buf.lookup_transform(self.world_frame, self.ee_frame,
                                            rospy.Time(0), rospy.Duration(0.02))
            if not (-0.05 <= p_world.y <= 0.55 and 0.54 <= p_world.z <= 1.14):
                rospy.logerr_throttle(0.0, f"[bridge] outside catching area time: {dur:.3f}")
                return
            q = T_w_ee.transform.rotation
        except Exception as e:
             rospy.logwarn_throttle(1.0, f"[bridge] EE TF lookup failed ({self.ee_frame}→{self.world_frame}): {e}")
             return
        


        # Build EETarget
        msg = EETarget()
        msg.ee_target.position.x = p_world.x
        msg.ee_target.position.y = p_world.y
        msg.ee_target.position.z = p_world.z
        msg.ee_target.orientation.x = q.x
        msg.ee_target.orientation.y = q.y
        msg.ee_target.orientation.z = q.z
        msg.ee_target.orientation.w = q.w

        msg.duration = dur
        self.pub.publish(msg)

        rospy.loginfo_throttle(
            0.0,
            f"[bridge] → ee_target @{self.world_frame}: "
            f"({p_world.y:.3f},{p_world.z:.3f}), "
            f"dur={dur:.3f}s  "
            f"{'(current EE q)'}"
        )

if __name__ == "__main__":
    rospy.init_node("kf_to_ee_target_bridge")
    KFtoEETargetBridge()
    rospy.spin()
