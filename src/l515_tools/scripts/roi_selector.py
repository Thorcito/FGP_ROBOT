#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber

class ROISelectorandRepublisher:
    def __init__(self):
        # --- state ---
        self.bridge = CvBridge()
        self.img = None
        self.dragging = False
        self.x0 = self.y0 = self.x1 = self.y1 = 0
        self.last_rect = None
        self.pad = rospy.get_param("~pad", 0)   # optional padding in pixels
        self.alpha = rospy.get_param("~alpha", 0.5)  # overlay blending [0..1]
        self.gui_ok = False
        self.win = "ROI selector (drag on image)"

        # --- publishers ---
        self.pub_roi_msg   = rospy.Publisher("/roi/selected", RegionOfInterest, queue_size=1)
        self.pub_rgb_roi   = rospy.Publisher("/camera/color/image_roi", Image, queue_size=1)
        self.pub_depth_roi = rospy.Publisher("/camera/aligned_depth_to_color/image_roi", Image, queue_size=1)
        self.pub_overlay   = rospy.Publisher("/camera/overlay/image", Image, queue_size=1)

        # --- synced subscribers (RGB + aligned depth) ---
        self.sub_rgb   = Subscriber("/camera/color/image_raw", Image)
        self.sub_depth = Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        self.sync = ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.sync_cb)

        # --- GUI (safe init) ---
        try:
            cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(self.win, self.on_mouse)
            rospy.loginfo("ROI window ready. Drag to select. 'c' clears, 'q' quits.")
            self.gui_ok = True
        except Exception as e:
            rospy.logwarn("OpenCV window could not be created: %s. Running headless.", e)

    # -------- mouse handling for ROI --------
    def on_mouse(self, event, x, y, flags, param):
        if self.img is None:
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.x0, self.y0 = x, y
            self.x1, self.y1 = x, y
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.x1, self.y1 = x, y
        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging = False
            x_min, x_max = sorted([self.x0, self.x1])
            y_min, y_max = sorted([self.y0, self.y1])
            self.last_rect = (x_min, y_min, x_max, y_max)

            roi = RegionOfInterest()
            roi.x_offset = int(x_min)
            roi.y_offset = int(y_min)
            roi.width    = max(1, int(x_max - x_min))
            roi.height   = max(1, int(y_max - y_min))
            roi.do_rectify = True
            self.pub_roi_msg.publish(roi)
            rospy.loginfo("ROI set: x=%d y=%d w=%d h=%d", roi.x_offset, roi.y_offset, roi.width, roi.height)

    # -------- main callback (synced RGB/Depth) --------
    def sync_cb(self, rgb_msg, depth_msg):
        try:
            rgb   = self.bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logwarn("cv_bridge conversion failed: %s", e)
            return

        self.img = rgb

        # Publish full-frame overlay (RGB + depth colormap)
        depth_color = self._depth_to_viz(depth)
        overlay = cv2.addWeighted(rgb, 1.0 - self.alpha, depth_color, self.alpha, 0.0)
        msg_overlay = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        msg_overlay.header = rgb_msg.header
        self.pub_overlay.publish(msg_overlay)

        # If ROI exists, publish cropped ROI topics
        if self.last_rect is not None:
            x0, y0, x1, y1 = self.last_rect
            h, w = rgb.shape[:2]
            x = max(0, x0 - self.pad)
            y = max(0, y0 - self.pad)
            w_roi = min((x1 - x0) + 2*self.pad, w - x)
            h_roi = min((y1 - y0) + 2*self.pad, h - y)

            if w_roi > 0 and h_roi > 0:
                rgb_roi   = rgb[y:y+h_roi, x:x+w_roi]
                depth_roi = depth[y:y+h_roi, x:x+w_roi]

                out_rgb = self.bridge.cv2_to_imgmsg(rgb_roi, encoding="bgr8")
                out_rgb.header = rgb_msg.header
                self.pub_rgb_roi.publish(out_rgb)

                out_depth = self.bridge.cv2_to_imgmsg(depth_roi, encoding=depth_msg.encoding)
                out_depth.header = depth_msg.header
                self.pub_depth_roi.publish(out_depth)

        # Minimal GUI: show RGB with live green drag (no persistent boxes on output topics)
        if self.gui_ok:
            vis = rgb.copy()
            if self.dragging:
                cv2.rectangle(vis, (self.x0, self.y0), (self.x1, self.y1), (0, 255, 0), 2)
            cv2.imshow(self.win, vis)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rospy.signal_shutdown("Closed by user")
            elif key == ord('c'):
                self.last_rect = None

    # -------- helper: depth to colored viz --------
    def _depth_to_viz(self, depth):
        d = depth.astype(np.float32)
        valid = (d > 0) & np.isfinite(d)
        if np.count_nonzero(valid) < 10:
            return np.zeros((depth.shape[0], depth.shape[1], 3), dtype=np.uint8)
        vmin = np.percentile(d[valid], 5)
        vmax = np.percentile(d[valid], 95)
        d = np.clip((d - vmin) / max(1e-6, (vmax - vmin)), 0, 1)
        d = (d * 255).astype(np.uint8)
        return cv2.applyColorMap(d, cv2.COLORMAP_JET)

if __name__ == "__main__":
    rospy.init_node("roi_selector_and_republisher")
    ROISelectorandRepublisher()
    rospy.spin()
    try:
        cv2.destroyAllWindows()
    except:
        pass
