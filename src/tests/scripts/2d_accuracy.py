#!/usr/bin/env python3
import rospy, cv2, numpy as np, os, csv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from message_filters import Subscriber, ApproximateTimeSynchronizer

class BallTrackerSync:
    def __init__(self):
        self.bridge = CvBridge()

        # --- Topics / params ---
        self.rgb_topic      = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
        self.depth_topic    = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.cam_info_topic = rospy.get_param("~cam_info_topic", "/camera/color/camera_info")
        self.frame_id       = rospy.get_param("~frame_id", "camera_color_optical_frame")

        # Sync
        self.slop_s     = float(rospy.get_param("~sync_slop_s", 0.005))
        self.max_dt_s   = float(rospy.get_param("~max_dt_s",   0.006))
        self.queue_size = int(rospy.get_param("~queue_size",   10))

        # HSV / morph (your values)
        self.hsv_lower  = np.array(rospy.get_param("~hsv_lower", [105, 20,  40]), dtype=np.uint8)
        self.hsv_upper  = np.array(rospy.get_param("~hsv_upper", [170, 255, 255]), dtype=np.uint8)
        self.blur_ksize = int(rospy.get_param("~blur",  1))
        self.open_it    = int(rospy.get_param("~open",  6))
        self.close_it   = int(rospy.get_param("~close", 1))
        self.min_area_px     = int(rospy.get_param("~min_area_px", 400))
        self.min_circularity = float(rospy.get_param("~min_circularity", 0.5))

        # CSV & run tag
        self.csv_path  = rospy.get_param("~csv_path", "src/tests/scripts/2d_grav_5.csv")
        self.run_tag   = rospy.get_param("~tag", "run1")

        # Ballistic model options (vertical in pixels)
        self.use_fixed_g_px = bool(rospy.get_param("~use_fixed_g_px", False))
        self.g_px           = float(rospy.get_param("~g_px", 2000.0))  # px/s^2; set from a quick calibration if desired

        # Debug pubs (kept)
        self.pub_debug = rospy.Publisher("/ball_meas/debug_image", Image, queue_size=1)
        self.pub_mask  = rospy.Publisher("/ball_meas/debug_mask",  Image, queue_size=1)

        # Data buffers
        self.t_list, self.u_list, self.v_list = [], [], []
        self.t0 = None  # first timestamp → time base

        # Camera intrinsics (only for frame_id consistency)
        self.fx = self.fy = self.cx = self.cy = None
        rospy.Subscriber(self.cam_info_topic, CameraInfo, self.caminfo_cb, queue_size=1)

        # Sync subscribers
        self.sub_rgb   = Subscriber(self.rgb_topic, Image)
        self.sub_depth = Subscriber(self.depth_topic, Image)
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth],
            queue_size=self.queue_size, slop=self.slop_s, allow_headerless=False
        )
        self.sync.registerCallback(self.sync_cb)

        # Prepare CSV
        self._prepare_csv()

        rospy.loginfo("2D ballistic accuracy node ready. CSV → %s", self.csv_path)

    # ----------------- Callbacks -----------------
    def caminfo_cb(self, msg: CameraInfo):
        self.fx, self.fy, self.cx, self.cy = msg.K[0], msg.K[4], msg.K[2], msg.K[5]
        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id

    def sync_cb(self, rgb_msg: Image, depth_msg: Image):
        # Ensure intrinsics ready (for frame_id only)
        if self.fx is None:
            return

        # Time sync constraint
        if abs((rgb_msg.header.stamp - depth_msg.header.stamp).to_sec()) > self.max_dt_s:
            return

        # Convert images
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        _   = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")  # not used; only for sync

        # --- Detect ball (mask → contours) ---
        hsv  = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        if self.blur_ksize > 1:
            k = self.blur_ksize | 1
            mask = cv2.GaussianBlur(mask, (k, k), 0)
        if self.open_it > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((3,3), np.uint8), iterations=self.open_it)
        if self.close_it > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8), iterations=self.close_it)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for c in cnts:
            area = cv2.contourArea(c)
            if area <= self.min_area_px:
                continue
            peri = cv2.arcLength(c, True)
            if peri <= 0:
                continue
            circ = 4.0*np.pi*area/(peri*peri)
            if circ < self.min_circularity:
                continue
            (x, y), r = cv2.minEnclosingCircle(c)
            score = (circ*circ) * area
            if best is None or score > best[0]:
                best = (score, int(x), int(y), float(r), c)

        debug = rgb.copy()
        if best is None:
            self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
            return

        _, u, v, r, contour = best
        cv2.circle(debug, (u, v), int(r), (0,255,0), 2)
        cv2.circle(debug, (u, v), 3, (0,0,255), -1)

        # Time base
        t_stamp = rgb_msg.header.stamp
        if self.t0 is None:
            self.t0 = t_stamp
        t_s = (t_stamp - self.t0).to_sec()

        # Append measurement
        self.t_list.append(t_s)
        self.u_list.append(float(u))
        self.v_list.append(float(v))

        # --- Online fitting (needs a few points) ---
        N = len(self.t_list)
        u_fit = np.nan
        v_fit = np.nan
        res_u = np.nan
        res_v = np.nan
        rms_u = rms_v = bias_u = bias_v = jitter_u = jitter_v = np.nan
        a_u = b_u = np.nan
        a_v = b_v = c_v = np.nan
        g_px_est = np.nan

        # --- Online fitting (needs a few points) ---
        if N >= 5:
            t = np.asarray(self.t_list, dtype=np.float64)
            u_meas = np.asarray(self.u_list, dtype=np.float64)
            v_meas = np.asarray(self.v_list, dtype=np.float64)

            # v(t) ~ linear
            A_v = np.column_stack([np.ones_like(t), t])  # [a_v, b_v]
            theta_v, _, _, _ = np.linalg.lstsq(A_v, v_meas, rcond=None)
            a_v, b_v = theta_v
            v_hat_all = A_v.dot(theta_v)
            v_fit = a_v + b_v * t[-1]

            # u(t) ~ quadratic (ballistic)
            if self.use_fixed_g_px:
                c_u = -0.5 * self.g_px
                A_u = np.column_stack([np.ones_like(t), t])
                y_u = u_meas - c_u * (t**2)
                theta_u, _, _, _ = np.linalg.lstsq(A_u, y_u, rcond=None)
                a_u, b_u = theta_u
                u_hat_all = a_u + b_u * t + c_u * (t**2)
                g_px_est = self.g_px
            else:
                A_u = np.column_stack([np.ones_like(t), t, t**2])
                theta_u, _, _, _ = np.linalg.lstsq(A_u, u_meas, rcond=None)
                a_u, b_u, c_u = theta_u
                u_hat_all = A_u.dot(theta_u)
                g_px_est = -2.0 * c_u

            u_fit = a_u + b_u * t[-1] + c_u * (t[-1]**2)

            # Residuals
            ru = u_meas - u_hat_all
            rv = v_meas - v_hat_all

            # Metrics (running, over all samples so far)
            rms_u  = float(np.sqrt(np.mean(ru**2)))
            rms_v  = float(np.sqrt(np.mean(rv**2)))
            bias_u = float(np.mean(ru))
            bias_v = float(np.mean(rv))
            jitter_u = float(np.std(np.diff(ru))) if len(ru) > 1 else np.nan
            jitter_v = float(np.std(np.diff(rv))) if len(rv) > 1 else np.nan
            # Residuals at current frame
            res_u = float(ru[-1])
            res_v = float(rv[-1])

        # --- Write CSV row per frame ---
        self._append_csv_row(
            stamp=t_stamp.to_sec(), tag=self.run_tag,
            t_s=t_s, u_meas=u, v_meas=v,
            u_fit=u_fit, v_fit=v_fit, res_u=res_u, res_v=res_v,
            rms_u=rms_u, rms_v=rms_v, bias_u=bias_u, bias_v=bias_v,
            jitter_u=jitter_u, jitter_v=jitter_v,
            a_u=a_u, b_u=b_u, a_v=a_v, b_v=b_v, c_v=c_v, g_px_est=g_px_est
        )

        # --- Debug overlays (kept) ---
        cv2.putText(debug, f"u={u:.0f}px v={v:.0f}px  t={t_s:.3f}s",
                    (max(5, u+8), max(20, v-10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,220,220), 2, cv2.LINE_AA)
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))

    # ----------------- CSV helpers -----------------
    def _prepare_csv(self):
        dname = os.path.dirname(self.csv_path)
        if dname:
            os.makedirs(dname, exist_ok=True)
        write_header = not os.path.exists(self.csv_path)
        if write_header:
            with open(self.csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    "timestamp","tag","t_s",
                    "u_meas","v_meas","u_fit","v_fit",
                    "res_u","res_v","rms_u","rms_v","bias_u","bias_v",
                    "jitter_u","jitter_v",
                    "a_u","b_u","a_v","b_v","c_v","g_px_est",
                    "use_fixed_g_px","g_px_param"
                ])

    def _append_csv_row(self, **kw):
        try:
            with open(self.csv_path, "a", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    f"{kw['stamp']:.6f}", kw["tag"], f"{kw['t_s']:.6f}",
                    fmt(kw["u_meas"]), fmt(kw["v_meas"]), fmt(kw["u_fit"]), fmt(kw["v_fit"]),
                    fmt(kw["res_u"]), fmt(kw["res_v"]), fmt(kw["rms_u"]), fmt(kw["rms_v"]),
                    fmt(kw["bias_u"]), fmt(kw["bias_v"]),
                    fmt(kw["jitter_u"]), fmt(kw["jitter_v"]),
                    fmt(kw["a_u"]), fmt(kw["b_u"]), fmt(kw["a_v"]), fmt(kw["b_v"]), fmt(kw["c_v"]),
                    fmt(kw["g_px_est"]),
                    int(self.use_fixed_g_px), fmt(self.g_px),
                ])
        except Exception as e:
            rospy.logwarn_throttle(1.0, "CSV append failed: %s", e)

def fmt(x):
    try:
        if x is None or (isinstance(x, float) and (np.isnan(x) or np.isinf(x))):
            return ""
        return f"{float(x):.6f}"
    except Exception:
        return ""

if __name__ == "__main__":
    rospy.init_node("2d_ballistic_accuracy_node")
    try:
        BallTrackerSync()
        rospy.spin()
    finally:
        try: cv2.destroyAllWindows()
        except: pass
