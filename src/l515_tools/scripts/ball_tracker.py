#!/usr/bin/env python3
import rospy, cv2, numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from message_filters import Subscriber, ApproximateTimeSynchronizer

class BallTrackerSync:
    def __init__(self):
        self.bridge = CvBridge()

        # Topics / params
        self.rgb_topic   = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.cam_info_topic = rospy.get_param("~cam_info_topic", "/camera/color/camera_info")
        self.frame_id    = rospy.get_param("~frame_id", "camera_color_optical_frame")

        # Sync tuning
        self.slop_s      = float(rospy.get_param("~sync_slop_s", 0.005))
        self.max_dt_s    = float(rospy.get_param("~max_dt_s", 0.006)) 
        self.queue_size  = int(rospy.get_param("~queue_size", 10))

        # Depth handling
        self.depth_scale = float(rospy.get_param("~depth_scale", 0.001))
        self.min_valid_z = float(rospy.get_param("~min_valid_z", 0.49))

        # HSV & morphology (your fixed settings)
        self.hsv_lower = np.array(rospy.get_param("~hsv_lower", [108, 115,  0]), dtype=np.uint8)
        self.hsv_upper = np.array(rospy.get_param("~hsv_upper", [170, 255, 255]), dtype=np.uint8)
        self.blur_ksize = int(rospy.get_param("~blur", 1))
        self.open_it    = int(rospy.get_param("~open", 2))
        self.close_it   = int(rospy.get_param("~close", 5))

        # Geometry filters
        self.min_area_px     = int(rospy.get_param("~min_area_px", 80))
        self.min_circularity = float(rospy.get_param("~min_circularity", 0.5))
        self.max_jump_m = float(rospy.get_param("~max_jump_m", 0.40))
        self._last_xyz = None  # stores last accepted (X,Y,Z)

        # Depth sampling
        self.depth_patch = int(rospy.get_param("~depth_patch", 1))



        # Path for RViz
        self.path_publish = bool(rospy.get_param("~publish_path", True))
        self.path_maxlen  = int(rospy.get_param("~path_maxlen", 50))
        self._poses = []

        # Camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None

        # Publishers
        self.pub_point   = rospy.Publisher("/ball_meas/point", PointStamped, queue_size=10)
        self.pub_pose    = rospy.Publisher("/ball_meas/pose", PoseStamped, queue_size=10)
        self.pub_path    = rospy.Publisher("/ball_meas/path", Path, queue_size=10) if self.path_publish else None
        self.pub_debug   = rospy.Publisher("/ball_meas/debug_image", Image, queue_size=1)
        self.pub_mask    = rospy.Publisher("/ball_meas/debug_mask", Image, queue_size=1)

        # Subscribers (camera info + synchronized image pair)
        rospy.Subscriber(self.cam_info_topic, CameraInfo, self.caminfo_cb, queue_size=1)

        self.sub_rgb   = Subscriber(self.rgb_topic, Image)
        self.sub_depth = Subscriber(self.depth_topic, Image)
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth],
            queue_size=self.queue_size, slop=self.slop_s, allow_headerless=False
        )
        self.sync.registerCallback(self.sync_cb)

        rospy.loginfo("BallTrackerSync: ATS slop=%.3f s, max_dt=%.3f s, queue=%d",
                      self.slop_s, self.max_dt_s, self.queue_size)

    def caminfo_cb(self, msg: CameraInfo):
        self.fx, self.fy, self.cx, self.cy = msg.K[0], msg.K[4], msg.K[2], msg.K[5]
        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id

    def sync_cb(self, rgb_msg: Image, depth_msg: Image):
        # Ensure intrinsics ready
        if self.fx is None:
            return

        # Hard check timestamp gap
        dt = abs((rgb_msg.header.stamp - depth_msg.header.stamp).to_sec())
        if dt > self.max_dt_s:
            rospy.logwarn_throttle(1.0, "RGB/Depth dt=%.3f s > %.3f s (skipping)", dt, self.max_dt_s)
            return

        # Convert images
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_u16 = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")

        H, W = depth_u16.shape[:2]

        # 1) HSV mask
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        if self.blur_ksize > 1:
            k = self.blur_ksize | 1
            mask = cv2.GaussianBlur(mask, (k, k), 0)
        if self.open_it > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8), iterations=self.open_it)
        if self.close_it > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8), iterations=self.close_it)

        # 2) Contours + circularity
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for c in cnts:
            area = cv2.contourArea(c)
            if area <= self.min_area_px:
                #rospy.loginfo("AREA MINIMA")
                #rospy.loginfo(area)
                continue
            peri = cv2.arcLength(c, True)
            #rospy.loginfo(area)
            if peri <= 0:
                #rospy.loginfo("PERIMETRO MINIMO")
                #rospy.loginfo(peri)
                continue
            circ = 4.0*np.pi*area/(peri*peri)
            #rospy.logerr(circ)
            if circ < self.min_circularity:
                #rospy.loginfo("CIRC ERROR")
                #rospy.loginfo(circ)
                continue
            (x, y), r = cv2.minEnclosingCircle(c)
            score = circ**2 * area
            if best is None or score > best[0]:
                best = (score, int(x), int(y), float(r), c)
                #rospy.loginfo(score)

        debug = rgb.copy()
        if best is None:
            self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
            #rospy.logwarn("NO BALL")
            return

        _, u, v, r, contour = best
        cv2.circle(debug, (u, v), int(r), (0,255,0), 2)
        cv2.circle(debug, (u, v), 3, (0,0,255), -1)

        '''Pixel depth gathering
        z_center = float(depth_u16[v,u]) * self.depth_scale
        if z_center <= 0:
            rospy.logwarn("XXXX")
            return
        Z = z_center
        '''
        # 3) Depth Z median around center (from synchronized depth frame)
        half = self.depth_patch // 2
        x0 = max(0, u - half); x1 = min(W, u + half + 1)
        y0 = max(0, v - half); y1 = min(H, v + half + 1)

        patch = depth_u16[y0:y1, x0:x1].astype(np.float32) * self.depth_scale
        cv2.rectangle(debug, (x0, y0), (x1, y1), (255, 255, 255), 4)
        valid = np.isfinite(patch) & (patch > 0) & (patch >= self.min_valid_z)
        if np.count_nonzero(valid) == 0:
            self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
            #rospy.logerr("No depth data")
            return
        Z = float(np.median(patch[valid]))

        # 4) Back-project to 3D in camera_color_optical_frame
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        #rospy.loginfo(f"XYpix=({u:.3f},{v:.3f})pix")
        #rospy.loginfo(f"XYZ=({X:.3f},{Y:.3f},{Z:.6f})m")
        
        if self.max_jump_m > 0:
            if self._last_xyz is not None:
                dx = X - self._last_xyz[0]
                dy = Y - self._last_xyz[1]
                dz = Z - self._last_xyz[2]
                dist = (dx**2 + dy**2 + dz**2)**0.5
                #rospy.loginfo(dist)
                if dist > self.max_jump_m:
                    rospy.logwarn_throttle(1.0, f"Bad Measurement (distance={dist:.2f})")
                    self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
                    self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
                    self._last_xyz = (X,Y,Z)
                    return
        self._last_xyz = (X,Y,Z)
        stamp = rgb_msg.header.stamp  # use the synchronized timestamp

        # 5) Publish PlotJuggler-friendly topics
        pt = PointStamped()
        pt.header = Header(stamp=stamp, frame_id=self.frame_id)
        pt.point.x, pt.point.y, pt.point.z = X, Y, Z
        self.pub_point.publish(pt)

        pose = PoseStamped()
        pose.header = pt.header
        pose.pose.position.x = X
        pose.pose.position.y = Y
        pose.pose.position.z = Z
        pose.pose.orientation.w = 1.0
        self.pub_pose.publish(pose)

        if self.path_publish and self.pub_path:
            self._poses.append(pose)
            if len(self._poses) > self.path_maxlen:
                self._poses = self._poses[-self.path_maxlen:]
            path_msg = Path()
            path_msg.header = pose.header
            path_msg.poses = self._poses
            self.pub_path.publish(path_msg)

        # Debug overlays
        
        cv2.putText(debug, f"XYZ=({X:.3f},{Y:.3f},{Z:.6f})m",
                    (u+8, max(20, v-10)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,220,220), 4, cv2.LINE_AA)
        '''
        cv2.putText(debug, f"Z=({Z:.6f})m",
                    (u+8, max(20, v-10)), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (0,220,220), 4, cv2.LINE_AA)
        '''
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))

if __name__ == "__main__":
    rospy.init_node("ball_tracker_sync")
    # Set TCP no-delay to reduce latency (rospy supports TransportHints)
    rospy.Subscriber.__init__
    try:
        BallTrackerSync()
        rospy.spin()
    finally:
        try: cv2.destroyAllWindows()
        except: pass
