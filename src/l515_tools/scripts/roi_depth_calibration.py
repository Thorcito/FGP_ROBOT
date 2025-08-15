#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import csv
from datetime import datetime

class DepthAccuracyNode:
    def __init__(self):
        self.bridge = CvBridge()

        # ---- Params
        self.depth_topic   = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_roi")
        self.color_topic   = rospy.get_param("~color_topic", "/camera/color/image_roi")  # optional (for future use)
        self.ground_truth  = float(rospy.get_param("~ground_truth_m", 1.40))
        self.num_frames    = int(rospy.get_param("~num_frames", 200))
        self.csv_path      = rospy.get_param("~csv_path", "src/l515_tools/scripts/Calibration_depth.csv")   # if empty, no CSV
        self.run_tag       = rospy.get_param("~tag", "result")        # optional label for the measurement point

        # ---- State
        self.values = []
        self.done = False

        # ---- Subscriptions
        self.sub_depth = rospy.Subscriber(self.depth_topic, Image, self.depth_cb, queue_size=10)

        rospy.loginfo("DepthAccuracyNode listening on %s (GT=%.3f m, frames=%d). CSV: %s",
                      self.depth_topic, self.ground_truth, self.num_frames,
                      (self.csv_path if self.csv_path else "disabled"))

    def depth_cb(self, msg):
        if self.done:
            return

        # Convert depth ROI
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logwarn("cv_bridge conversion failed: %s", e)
            return

        # Convert to meters depending on encoding
        l515_depth_scale = 0.001 #m per unit
        depth_m = depth.astype(np.float32) * l515_depth_scale
        #rospy.logerr_once(depth.astype(np.float32))
        # Extract valid values
        valid = np.isfinite(depth_m) & (depth_m > 0)
        if not np.any(valid):
            return

        # Average entire ROI (it’s already your selected patch)
        roi_vals = depth_m[valid]
        mean_z = float(np.mean(roi_vals))
        self.values.append(mean_z)

        if len(self.values) >= self.num_frames:
            self.finalize(roi_vals_all_last_frame=roi_vals)

    def finalize(self, roi_vals_all_last_frame=None):
        self.done = True
        data = np.array(self.values, dtype=np.float32)

        mean_over_time = float(np.mean(data))
        std_over_time  = float(np.std(data))
        med_over_time  = float(np.median(data))

        # Also report distribution from the last frame (optional)
        p5 = float(np.percentile(roi_vals_all_last_frame, 5))  if roi_vals_all_last_frame is not None else float('nan')
        p95= float(np.percentile(roi_vals_all_last_frame, 95)) if roi_vals_all_last_frame is not None else float('nan')

        err_m  = mean_over_time - self.ground_truth
        err_pct= (err_m / self.ground_truth) * 100.0 if self.ground_truth != 0 else float('nan')

        rospy.loginfo("==== Depth Accuracy (ROI) ====")
        rospy.loginfo("Frames: %d", len(self.values))
        rospy.loginfo("Ground truth (m):   %.6f", self.ground_truth)
        rospy.loginfo("Mean Z (m):        %.6f", mean_over_time)
        rospy.loginfo("Median Z (m):      %.6f", med_over_time)
        rospy.loginfo("Std dev over time: %.6f m", std_over_time)
        rospy.loginfo("ROI p05..p95 (m):  %.6f .. %.6f", p5, p95)
        rospy.loginfo("Error:             %.6f m (%.3f%%)", err_m, err_pct)

        # Optional CSV append
        if self.csv_path:
            header = [
                "timestamp", "tag", "gt_m", "mean_m", "median_m", "std_m",
                "p05_m", "p95_m", "error_m", "error_pct", "frames"
            ]
            row = [
                datetime.now().isoformat(), self.run_tag, self.ground_truth, mean_over_time,
                med_over_time, std_over_time, p5, p95, err_m, err_pct, len(self.values)
            ]
            write_header = not os.path.exists(self.csv_path)
            try:
                os.makedirs(os.path.dirname(self.csv_path), exist_ok=True) if os.path.dirname(self.csv_path) else None
                with open(self.csv_path, "a", newline="") as f:
                    w = csv.writer(f)
                    if write_header:
                        w.writerow(header)
                    w.writerow(row)
                rospy.loginfo("Appended results to %s", self.csv_path)
            except Exception as e:
                rospy.logwarn("Failed to write CSV '%s': %s", self.csv_path, e)

        # end
        rospy.signal_shutdown("Depth accuracy measurement complete.")

if __name__ == "__main__":
    rospy.init_node("depth_accuracy_node")
    DepthAccuracyNode()
    rospy.spin()
