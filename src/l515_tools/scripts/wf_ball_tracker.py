#!/usr/bin/env python3
import rospy, cv2, numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from message_filters import Subscriber, ApproximateTimeSynchronizer
import tf2_ros
from tf2_geometry_msgs import do_transform_point

class BallTrackerSync:
    """Synchronized RGB-D ball tracker with 3D back-projection and debug outputs."""

    def __init__(self):
        self.bridge = CvBridge()

        # ---------------------------
        # Topic names and frame IDs
        # ---------------------------
        self.rgb_topic   = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.cam_info_topic = rospy.get_param("~cam_info_topic", "/camera/color/camera_info")
        self.frame_id    = rospy.get_param("~frame_id", "camera_color_optical_frame")

        # Transformation to world frame
        self.parent_frame     = rospy.get_param("~parent_frame", "world")
        self.cam_xyz           = rospy.get_param("~camera_xyz", [0.3026, -0.0604, 0.9295])
        self.cam_quat_xyzw     = rospy.get_param("~camera_quat_xyzw", [-0.5, -0.5, 0.5, 0.5])  # x y z w
        self.rot_angle = rospy.get_param("~rot_angle", 0)
        self.static_sent = False

         # -------- TF2 --------
        self.buf = tf2_ros.Buffer(cache_time=rospy.Duration(2.0))
        self.listener = tf2_ros.TransformListener(self.buf)

        # ---------------------------
        # Time synchronization setup
        # ---------------------------
        self.slop_s      = float(rospy.get_param("~sync_slop_s", 0.005))
        self.max_dt_s    = float(rospy.get_param("~max_dt_s", 0.006)) 
        self.queue_size  = int(rospy.get_param("~queue_size", 10))

        # ---------------------------
        # Depth handling / validation
        # ---------------------------
        # depth_scale converts raw depth units to meters. min_valid_z rejects too-close returns / zeros.
        self.depth_scale = float(rospy.get_param("~depth_scale", 0.001))
        self.min_valid_z = float(rospy.get_param("~min_valid_z", 0.5))

        # ---------------------------------
        # HSV segmentation & morphology ops
        # ---------------------------------
        # Fixed thresholds are assumed tuned for your ball color and lighting.
        self.hsv_lower = np.array(rospy.get_param("~hsv_lower", [105, 55,  33]), dtype=np.uint8)
        self.hsv_upper = np.array(rospy.get_param("~hsv_upper", [170, 160, 255]), dtype=np.uint8)
        self.blur_ksize = int(rospy.get_param("~blur", 1))   
        self.open_it    = int(rospy.get_param("~open", 3))
        self.close_it   = int(rospy.get_param("~close", 1))

        # ---------------------------------
        # Geometry filters & outlier gating
        # ---------------------------------
        self.min_area_px     = int(rospy.get_param("~min_area_px", 200)) #150min 
        self.min_circularity = float(rospy.get_param("~min_circularity", 0.5))
        self.max_jump_m = float(rospy.get_param("~max_jump_m", 0.40))
        self._last_xyz = None  # last accepted 3D point (for jump gating)

        # ---------------------------
        # Depth sampling patch size
        # ---------------------------
        self.depth_patch = int(rospy.get_param("~depth_patch", 10))

        # ---------------------------
        # Path management for RViz
        # ---------------------------
        self.path_publish = bool(rospy.get_param("~publish_path", True))
        self.path_maxlen  = int(rospy.get_param("~path_maxlen", 50))
        self._poses = []  # sliding buffer of PoseStamped
        self._poses_w = []  # sliding buffer of PoseStamped

        # ---------------------------
        # Camera intrinsics (from CameraInfo)
        # ---------------------------
        self.fx = self.fy = self.cx = self.cy = None

        # ---------------------------
        # Publishers
        # ---------------------------
        # Note: topic names are fixed here; consumers must subscribe accordingly.
        self.pub_point   = rospy.Publisher("/cam_ball_meas/point", PointStamped, queue_size=10)
        self.pub_pose    = rospy.Publisher("/cam_ball_meas/pose", PoseStamped, queue_size=10)
        self.pub_path    = rospy.Publisher("/cam_ball_meas/path", Path, queue_size=10) if self.path_publish else None
        self.pub_point_world   = rospy.Publisher("/world_ball_meas/point", PointStamped, queue_size=10)
        self.pub_pose_world   = rospy.Publisher("/world_ball_meas/pose", PoseStamped, queue_size=10)
        self.pub_path_world   = rospy.Publisher("/world_ball_meas/path", Path, queue_size=10) if self.path_publish else None
        self.pub_debug   = rospy.Publisher("/cam_ball_meas/debug_image", Image, queue_size=1)
        self.pub_mask    = rospy.Publisher("/cam_ball_meas/debug_mask", Image, queue_size=1)

        # ---------------------------
        # Subscribers
        # ---------------------------
        # CameraInfo provides intrinsics (K) and authoritative frame_id.
        rospy.Subscriber(self.cam_info_topic, CameraInfo, self.caminfo_cb, queue_size=1)

        # Use message_filters to approximately synchronize RGB and Depth.
        self.sub_rgb   = Subscriber(self.rgb_topic, Image)
        self.sub_depth = Subscriber(self.depth_topic, Image)
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth],
            queue_size=self.queue_size, slop=self.slop_s, allow_headerless=False
        )
        self.sync.registerCallback(self.sync_cb)

        rospy.loginfo("BallTrackerSync: ATS slop=%.3f s, max_dt=%.3f s, queue=%d",
                      self.slop_s, self.max_dt_s, self.queue_size)
        
    def _publish_static_camera_tf(self):
        """Optionally publish a static transform for the camera (parent -> camera_frame)."""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.parent_frame
        t.child_frame_id  = self.frame_id
        t.transform.translation.x = float(self.cam_xyz[0])
        t.transform.translation.y = float(self.cam_xyz[1])
        t.transform.translation.z = float(self.cam_xyz[2])
        q1x, q1y, q1z, q1w = self.cam_quat_xyzw
        q2x, q2y, q2z, q2w = np.array([np.sin(np.radians(self.rot_angle)/2), 0, 0 , np.cos(np.radians(self.rot_angle)/2)])
        qx = q1w*q2x + q1x*q2w + q1y*q2z - q1z*q2y
        qy = q1w*q2y - q1x*q2z + q1y*q2w + q1z*q2x
        qz = q1w*q2z + q1x*q2y - q1y*q2x + q1z*q2w
        qw = q1w*q2w - q1x*q2x - q1y*q2y - q1z*q2z
        rospy.loginfo(f"Rot Quat: ({qx}, {qy}, {qz}, {qw})")
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_broadcaster.sendTransform(t)

    def caminfo_cb(self, msg: CameraInfo):
        """Cache intrinsics (fx, fy, cx, cy) and adopt frame_id if provided by the camera."""
        self.fx, self.fy, self.cx, self.cy = msg.K[0], msg.K[4], msg.K[2], msg.K[5]
        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id
            if not self.static_sent:
                self._publish_static_camera_tf()
                self.static_sent = True
            

    def sync_cb(self, rgb_msg: Image, depth_msg: Image):
        """
        Main synchronized callback:
        - Validates time difference.
        - Converts to OpenCV images.
        - Segments ball via HSV, selects best circular contour.
        - Extracts Z from aligned depth median patch at (u,v).
        - Back-projects (u,v,Z) -> (X,Y,Z) using intrinsics.
        - Publishes point/pose/path and debug images (camera frame)
        - Published point/pose/path (world frame)
        """
        t_cb_start = rospy.Time.now()

        # Ensure intrinsics are ready; skip until CameraInfo arrives.
        if self.fx is None:
            return

        # Hard check timestamp gap; prevents pairing distant frames.
        dt = abs((rgb_msg.header.stamp - depth_msg.header.stamp).to_sec())
        if dt > self.max_dt_s:
            rospy.logwarn_throttle(1.0, "RGB/Depth dt=%.3f s > %.3f s (skipping)", dt, self.max_dt_s)
            return

        # Convert ROS images to OpenCV mats.
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_u16 = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")

        H, W = depth_u16.shape[:2]

        # 1) Color segmentation in HSV space.
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # Optional denoising and blob clean-up.
        if self.blur_ksize > 1:
            k = self.blur_ksize | 1  # enforce odd kernel size
            mask = cv2.GaussianBlur(mask, (k, k), 0)
        if self.open_it > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8), iterations=self.open_it)
        if self.close_it > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8), iterations=self.close_it)

        # 2) Find candidate contours; score by circularity and area.
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for c in cnts:
            area = cv2.contourArea(c)
            if area <= self.min_area_px:
                rospy.logerr(f"AREA: {area}")
                continue
            peri = cv2.arcLength(c, True)
            if peri <= 0:
                rospy.logerr("PERIMETRO")
                continue
            circ = 4.0*np.pi*area/(peri*peri)  # circularity in [0,1]
            if circ < self.min_circularity:
                rospy.logerr("CIRCULARIDAD")
                continue
            (x, y), r = cv2.minEnclosingCircle(c)
            # Heuristic score: prefer more circular & larger area.
            score = circ**2 * area
            if best is None or score > best[0]:
                best = (score, int(x), int(y), float(r), c)

        debug = rgb.copy()
        if best is None:
            # No valid contour: publish diagnostics and exit.
            self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
            return

        _, u, v, r, contour = best
        # Draw detection for visual debugging.
        cv2.circle(debug, (u, v), int(r), (0,255,0), 2)
        cv2.circle(debug, (u, v), 3, (0,0,255), -1)

        # 3) Robust Z from depth: median over a small patch centered at (u,v).
        half = self.depth_patch // 2
        rad = int(0.8 * r)
        x0 = max(0, u - rad); x1 = min(W, u + rad + 1)
        y0 = max(0, v - rad); y1 = min(H, v + rad + 1)

        patch = depth_u16[y0:y1, x0:x1].astype(np.float32) * self.depth_scale
        #cv2.rectangle(debug, (x0, y0), (x1, y1), (255, 255, 255), 1)
        yy, xx = np.ogrid[y0:y1, x0:x1]
        circle_mask = (xx-u)**2 + (yy-v)**2 <= (rad*rad)
        cv2.circle(debug, (u,v), rad, (255,255,255), 2)

        # Valid depth filter: finite, positive, and above min_valid_z.
        valid = np.isfinite(patch) & (patch > 0) & (patch >= self.min_valid_z) & circle_mask
        if np.count_nonzero(valid) == 0:
            # No usable depth: publish diagnostics and skip.
            self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
            return
        Z = float(np.median(patch[valid]))

        # 4) Pinhole back-projection to 3D (camera optical frame).
        # X = (u - cx) * Z / fx ; Y = (v - cy) * Z / fy
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        
        # Optional: reject sudden jumps in 3D to suppress outliers.
        if self.max_jump_m > 0:
            if self._last_xyz is not None:
                dx = X - self._last_xyz[0]
                dy = Y - self._last_xyz[1]
                dz = Z - self._last_xyz[2]
                dist = (dx**2 + dy**2 + dz**2)**0.5
                if dist > self.max_jump_m:
                    rospy.logwarn_throttle(1.0, f"Bad Measurement (distance={dist:.2f})")
                    self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
                    self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
                    self._last_xyz = (X,Y,Z)  # keep state but do not publish this one
                    return
        self._last_xyz = (X,Y,Z)

        # Use the RGB timestamp as the synchronized stamp for outputs.
        stamp = rgb_msg.header.stamp

        # Latency (diagnostic): processing time from callback start to now.
        t_pub = rospy.Time.now()
        latency_proc = (t_pub - t_cb_start).to_sec()
        # rospy.loginfo(f"Latency: ({latency_proc}) s")
        # rospy.loginfo(f"Point: ({X}, {Y}, {Z})")

        # 5) Publish point (for numeric consumers / PlotJuggler).
        pt = PointStamped()
        pt.header = Header(stamp=stamp, frame_id=self.frame_id)
        pt.point.x, pt.point.y, pt.point.z = X, Y, Z
        self.pub_point.publish(pt)
        rospy.loginfo(f"Point: ({X}, {Y}, {Z})")

        # Pose with identity orientation (no yaw/pitch/roll inference here).
        pose = PoseStamped()
        pose.header = pt.header
        pose.pose.position.x = X
        pose.pose.position.y = Y
        pose.pose.position.z = Z
        pose.pose.orientation.w = 1.0
        self.pub_pose.publish(pose)

        # Maintain and publish a short Path history for RViz visualization.
        if self.path_publish and self.pub_path:
            self._poses.append(pose)
            if len(self._poses) > self.path_maxlen:
                self._poses = self._poses[-self.path_maxlen:]
            path_msg = Path()
            path_msg.header = pose.header
            path_msg.poses = self._poses
            self.pub_path.publish(path_msg)

        #6) Publish point on world frame
        try:
            Tcw = self.buf.lookup_transform(self.parent_frame, self.frame_id, stamp, rospy.Duration(0.05))
            point_world = do_transform_point(pt, Tcw)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Transformation to World ERROR: {e}")
            return
        
        self.pub_point_world.publish(point_world)
        rospy.loginfo(f"Point in World: ({point_world.point.x}, {point_world.point.y}, {point_world.point.z})")

        pose_world = PoseStamped()
        pose_world.header = point_world.header
        pose_world.pose.position.x = point_world.point.x
        pose_world.pose.position.y = point_world.point.y
        pose_world.pose.position.z = point_world.point.z
        pose_world.pose.orientation.w = 1.0
        self.pub_pose_world.publish(pose_world)

        if self.path_publish and self.pub_path_world:
            self._poses_w.append(pose_world)
            if len(self._poses_w) > self.path_maxlen:
                self._poses_w = self._poses_w[-self.path_maxlen:]
            path_w_msg = Path()
            path_w_msg.header = pose_world.header
            path_w_msg.poses = self._poses_w
            self.pub_path_world.publish(path_w_msg)



        # Debug overlays (text + mask + annotated RGB).
        cv2.putText(debug, f"XYZ=({X:.3f},{Y:.3f},{Z:.3f})m",
                    (u+8, max(20, v-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,220,220), 2, cv2.LINE_AA)
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))

if __name__ == "__main__":
    rospy.init_node("ball_tracker_sync")
    try:
        BallTrackerSync()
        rospy.spin()
    finally:
        try: cv2.destroyAllWindows()
        except: pass

