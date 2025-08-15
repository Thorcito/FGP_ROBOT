#!/usr/bin/env python3
import rospy, cv2, numpy as np, yaml, os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def odd(n): return int(n)|1

class HSVTuner:
    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_topic   = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
        self.save_path   = rospy.get_param("~save_path", os.path.expanduser("~/Desktop/Robo_Project_ws/src/l515_tools/scripts/hsv_magenta.yaml"))
        self.window      = "HSV Tuner (keys: r=ROI, a=auto, p=print, s=save, q=quit)"
        self.pub_mask    = rospy.Publisher("/hsv_tuner/mask", Image, queue_size=1)
        self.pub_preview = rospy.Publisher("/hsv_tuner/preview", Image, queue_size=1)

        # Default magenta-ish bands (OpenCV H in [0..179])
        self.H_low,  self.H_high  = 100, 179
        self.S_low,  self.S_high  = 0,  255
        self.V_low,  self.V_high  = 0,  255
        self.blur_k  = 1
        self.open_it = 1
        self.close_it= 1

        self.roi = None  # (x,y,w,h), in full image
        self.frame = None

        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        for name, init, maxv in [
            ("H_low", self.H_low, 179), ("H_high", self.H_high, 179),
            ("S_low", self.S_low, 255), ("S_high", self.S_high, 255),
            ("V_low", self.V_low, 255), ("V_high", self.V_high, 255),
            ("Gaussian blur", self.blur_k, 31),  ("open (noise removal)", self.open_it, 10), ("close (join holes)", self.close_it, 10)
        ]:
            cv2.createTrackbar(name, self.window, int(init), maxv, lambda v: None)

        rospy.Subscriber(self.rgb_topic, Image, self.rgb_cb, queue_size=1)
        rospy.loginfo("HSV Tuner listening to %s", self.rgb_topic)

    def read_tb(self):
        self.H_low  = cv2.getTrackbarPos("H_low",  self.window)
        self.H_high = cv2.getTrackbarPos("H_high", self.window)
        self.S_low  = cv2.getTrackbarPos("S_low",  self.window)
        self.S_high = cv2.getTrackbarPos("S_high", self.window)
        self.V_low  = cv2.getTrackbarPos("V_low",  self.window)
        self.V_high = cv2.getTrackbarPos("V_high", self.window)
        self.blur_k = max(1, cv2.getTrackbarPos("Gaussian blur",  self.window))
        self.open_it= cv2.getTrackbarPos("open (noise removal)",  self.window)
        self.close_it= cv2.getTrackbarPos("close (join holes)", self.window)

    def rgb_cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.read_tb()
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        # Handle Hue wrap-around: if low<=high → one band ; else → two bands
        lower = np.array([self.H_low,  self.S_low,  self.V_low],  dtype=np.uint8)
        upper = np.array([self.H_high, self.S_high, self.V_high], dtype=np.uint8)
        if self.H_low <= self.H_high:
            mask = cv2.inRange(hsv, lower, upper)
        else:
            mask1 = cv2.inRange(hsv, np.array([self.H_low, self.S_low, self.V_low], dtype=np.uint8),
                                     np.array([179,       self.S_high, self.V_high], dtype=np.uint8))
            mask2 = cv2.inRange(hsv, np.array([0,         self.S_low, self.V_low], dtype=np.uint8),
                                     np.array([self.H_high, self.S_high, self.V_high], dtype=np.uint8))
            mask = cv2.bitwise_or(mask1, mask2)

        if self.blur_k > 1:
            mask = cv2.GaussianBlur(mask, (odd(self.blur_k), odd(self.blur_k)), 0)
        if self.open_it > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8), iterations=self.open_it)
        if self.close_it > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8), iterations=self.close_it)

        preview = self.apply_preview(mask)
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
        self.pub_preview.publish(self.bridge.cv2_to_imgmsg(preview, "bgr8"))

        cv2.imshow(self.window, preview)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('r'):
            self.pick_roi()
        elif key == ord('a'):
            self.auto_from_roi(hsv)
        elif key == ord('p'):
            self.print_params()
        elif key == ord('s'):
            self.save_params()
        elif key == ord('q'):
            rospy.signal_shutdown("User quit")

    def apply_preview(self, mask):
        overlay = self.frame.copy()
        # colorize mask
        color = np.zeros_like(self.frame)
        color[..., 1] = mask  # green mask overlay
        preview = cv2.addWeighted(overlay, 1.0, color, 0.5, 0)
        if self.roi is not None:
            x,y,w,h = self.roi
            cv2.rectangle(preview, (x,y), (x+w, y+h), (0,0,255), 2)
        return preview

    def pick_roi(self):
        if self.frame is None: return
        r = cv2.selectROI(self.window, self.frame, showCrosshair=True, fromCenter=False)
        if r == (0,0,0,0): return
        self.roi = tuple(map(int, r))
        rospy.loginfo("ROI set: x=%d y=%d w=%d h=%d", *self.roi)

    def auto_from_roi(self, hsv):
        if self.roi is None:
            rospy.logwarn("Select ROI first (press 'r').")
            return
        x,y,w,h = self.roi
        patch = hsv[y:y+h, x:x+w].reshape(-1,3)
        if patch.size == 0:
            return
        H = patch[:,0].astype(np.int16)
        S = patch[:,1].astype(np.int16)
        V = patch[:,2].astype(np.int16)

        # Robust percentiles
        pH_lo, pH_hi = np.percentile(H, [10, 90])
        pS_lo, pS_hi = np.percentile(S, [10, 90])
        pV_lo, pV_hi = np.percentile(V, [10, 90])

        # Detect wrap-around: if range > ~90°, try shifting values < 90 by +180 and recompute
        if (pH_hi - pH_lo) > 90:
            H2 = H.copy()
            H2[H2 < 90] += 180
            pH_lo, pH_hi = np.percentile(H2, [10, 90])
            # map back to 0..179
            pH_lo = int(round(pH_lo)) % 180
            pH_hi = int(round(pH_hi)) % 180
            # Result likely crosses 179→0 boundary → set low>high to indicate wrap
            H_low, H_high = pH_lo, pH_hi
        else:
            H_low, H_high = int(round(pH_lo)), int(round(pH_hi))

        # Margins
        h_margin = 8; s_margin = 20; v_margin = 20
        S_low = max(0, int(round(pS_lo - s_margin)))
        S_high= min(255, int(round(pS_hi + s_margin)))
        V_low = max(0, int(round(pV_lo - v_margin)))
        V_high= min(255, int(round(pV_hi + v_margin)))

        # Push to trackbars (Hue may wrap)
        cv2.setTrackbarPos("S_low",  self.window, S_low)
        cv2.setTrackbarPos("S_high", self.window, S_high)
        cv2.setTrackbarPos("V_low",  self.window, V_low)
        cv2.setTrackbarPos("V_high", self.window, V_high)

        if H_low <= H_high:
            cv2.setTrackbarPos("H_low",  self.window, H_low)
            cv2.setTrackbarPos("H_high", self.window, H_high)
        else:
            # Indicate wrap by setting low > high (mask code handles the split)
            cv2.setTrackbarPos("H_low",  self.window, H_low)
            cv2.setTrackbarPos("H_high", self.window, H_high)

        rospy.loginfo("Auto HSV from ROI → H:[%d..%d] S:[%d..%d] V:[%d..%d]",
                      H_low, H_high, S_low, S_high, V_low, V_high)

    def current_params(self):
        params = dict(
            hsv1_lower=[int(self.H_low), int(self.S_low), int(self.V_low)],
            hsv1_upper=[int(self.H_high), int(self.S_high), int(self.V_high)],
            use_band2=False,  # if H_low>H_high, you’ll export as two bands below
            blur=self.blur_k, open=self.open_it, close=self.close_it
        )
        # If H wraps (low>high) export as two bands for convenience
        if self.H_low > self.H_high:
            params = dict(
                hsv1_lower=[int(self.H_low), int(self.S_low), int(self.V_low)],
                hsv1_upper=[179,            int(self.S_high), int(self.V_high)],
                hsv2_lower=[0,              int(self.S_low), int(self.V_low)],
                hsv2_upper=[int(self.H_high), int(self.S_high), int(self.V_high)],
                use_band2=True,
                blur=self.blur_k, open=self.open_it, close=self.close_it
            )
        return params

    def print_params(self):
        p = self.current_params()
        rospy.loginfo("Suggested params:\n%s", yaml.dump(p, sort_keys=False))
        print("\nRun your tracker with, e.g.:")
        if p.get("use_band2", False):
            print(f"_hsv1_lower:=\"{p['hsv1_lower']}\" _hsv1_upper:=\"{p['hsv1_upper']}\" "
                  f"_hsv2_lower:=\"{p['hsv2_lower']}\" _hsv2_upper:=\"{p['hsv2_upper']}\" _use_band2:=true")
        else:
            print(f"_hsv_lower:=\"{p['hsv1_lower']}\" _hsv_upper:=\"{p['hsv1_upper']}\"")

    def save_params(self):
        p = self.current_params()
        os.makedirs(os.path.dirname(self.save_path), exist_ok=True) if os.path.dirname(self.save_path) else None
        with open(self.save_path, "w") as f:
            yaml.safe_dump(p, f, sort_keys=False)
        rospy.loginfo("Saved HSV params to %s", self.save_path)

if __name__ == "__main__":
    rospy.init_node("hsv_tuner")
    HSVTuner()
    rospy.spin()
    try:
        cv2.destroyAllWindows()
    except: pass
