#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import cv2.aruco as aruco
import tf2_ros
from tf.transformations import quaternion_from_matrix
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import Header
import os, csv, datetime

class ROIToXYZ:
    def __init__(self):
        self.bridge = CvBridge()

        # --- Params ---
        self.depth_topic    = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_roi")
        self.rgb_topic      = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
        self.cam_info_topic = rospy.get_param("~cam_info_topic", "/camera/color/camera_info")
        self.roi_topic      = rospy.get_param("~roi_topic", "/roi/selected")
        self.depth_scale    = float(rospy.get_param("~depth_scale", 0.001))
        self.min_valid_z    = float(rospy.get_param("~min_valid_z", 0.49))
        self.center_patch   = int(rospy.get_param("~center_patch", 11))
        self.frame_id       = rospy.get_param("~frame_id", "camera_color_optical_frame")
        self.marker_length  = float(rospy.get_param("~marker_length", 0.1))  # m

        # ArUco setup (dictionary + id)
        self.aruco_id      = int(rospy.get_param("~aruco_id", 9))
        self.aruco_dict_id = rospy.get_param("~aruco_dict", "DICT_4X4_50")
        self.aruco_dict    = aruco.getPredefinedDictionary(getattr(aruco, self.aruco_dict_id))
        self.detector_params = aruco.DetectorParameters_create()

        # Logging & sample collection
        self.point_log_thresh = float(rospy.get_param("~point_log_thresh_m", 0.1))
        self.max_measurements = int(rospy.get_param("~max_measurements", 10))
        self.time_tolerance_s = float(rospy.get_param("~time_tolerance_s", 0.10))  # sync tolerance RGB<->Depth
        self.csv_path = rospy.get_param("~csv_path", os.path.expanduser("~/Desktop/Robo_Project_ws/src/l515_tools/scripts/aruco_roi_results.csv"))
        self.tag = rospy.get_param("~tag", "prueba_5")  # optional label

        # --- State ---
        self._last_point = None               # latest ROI XYZ
        self._last_point_stamp = None         # ros time for latest ROI sample
        self.samples = []                     # collected measurements (rows)
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.roi = None

        # --- Publishers ---
        self.pub_center = rospy.Publisher("/roi_center/point", PointStamped, queue_size=1)
        self.pub_aruco  = rospy.Publisher("/aruco/point", PointStamped, queue_size=1)

        # --- TF Broadcaster ---
        self.br = tf2_ros.TransformBroadcaster()

        # --- Subscribers ---
        rospy.Subscriber(self.cam_info_topic, CameraInfo, self.caminfo_cb, queue_size=1)
        rospy.Subscriber(self.roi_topic, RegionOfInterest, self.roi_cb, queue_size=5)
        rospy.Subscriber(self.depth_topic, Image, self.depth_cb, queue_size=5)
        rospy.Subscriber(self.rgb_topic, Image, self.rgb_cb, queue_size=1)

        rospy.loginfo("ROIToXYZ ready: depth=%s, rgb=%s, cam_info=%s, scale=%.6f; saving %d samples to %s",
                      self.depth_topic, self.rgb_topic, self.cam_info_topic, self.depth_scale,
                      self.max_measurements, self.csv_path)

    # ---------- Callbacks ----------
    def caminfo_cb(self, msg: CameraInfo):
        self.fx, self.fy, self.cx, self.cy = msg.K[0], msg.K[4], msg.K[2], msg.K[5]
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)
        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id

    def roi_cb(self, msg: RegionOfInterest):
        self.roi = msg

    def depth_cb(self, msg: Image):
        if None in (self.fx, self.fy, self.cx, self.cy) or self.roi is None:
            return

        depth_u16 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth_m = depth_u16.astype(np.float32) * self.depth_scale
        H, W = depth_m.shape[:2]
        if H == 0 or W == 0:
            return

        k = max(1, self.center_patch | 1)  # force odd
        half = k // 2
        cx_roi = W // 2
        cy_roi = H // 2
        y0 = max(0, cy_roi - half); y1 = min(H, cy_roi + half + 1)
        x0 = max(0, cx_roi - half); x1 = min(W, cx_roi + half + 1)
        patch = depth_m[y0:y1, x0:x1]

        valid = np.isfinite(patch) & (patch > 0) & (patch >= self.min_valid_z)
        if np.count_nonzero(valid) == 0:
            return

        z = float(np.median(patch[valid]))
        u = int(self.roi.x_offset + cx_roi)
        v = int(self.roi.y_offset + cy_roi)

        X = (u - self.cx) * z / self.fx
        Y = (v - self.cy) * z / self.fy
        Z = z

        # log if changed significantly
        changed = True
        if self._last_point is not None:
            dx, dy, dz = X - self._last_point[0], Y - self._last_point[1], Z - self._last_point[2]
            changed = (dx*dx + dy*dy + dz*dz) ** 0.5 > self.point_log_thresh
        if changed:
            rospy.loginfo("ROI center: XYZ=(%.4f, %.4f, %.4f) m", X, Y, Z)

        # publish ROI point
        pt = PointStamped()
        pt.header = Header(stamp=msg.header.stamp, frame_id=self.frame_id)
        pt.point.x, pt.point.y, pt.point.z = X, Y, Z
        self.pub_center.publish(pt)

        # update last ROI point & stamp
        self._last_point = (X, Y, Z)
        self._last_point_stamp = msg.header.stamp

    def rgb_cb(self, msg: Image):
        if self.camera_matrix is None:
            return

        # detect aruco
        img  = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)
        if ids is None or len(ids) == 0:
            return
        ids = ids.flatten()
        match = np.where(ids == self.aruco_id)[0]
        if match.size == 0:
            return
        idx = int(match[0])

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.camera_matrix, self.dist_coeffs
        )
        rvec = rvecs[idx][0]
        tvec = tvecs[idx][0]  # (x,y,z) in camera_color_optical_frame

        # publish aruco center point
        pt = PointStamped()
        pt.header = Header(stamp=msg.header.stamp, frame_id=self.frame_id)
        pt.point.x, pt.point.y, pt.point.z = map(float, tvec)
        self.pub_aruco.publish(pt)

        # TF broadcast
        R, _ = cv2.Rodrigues(rvec)
        M = np.eye(4, dtype=np.float64); M[:3, :3] = R
        qx, qy, qz, qw = quaternion_from_matrix(M)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = "aruco_marker_{}".format(self.aruco_id)
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self.br.sendTransform(t)

        # ---------- take a measurement if ROI is available & time-aligned ----------
        if self._last_point is None or self._last_point_stamp is None:
            return
        # time sync: require depth ROI to be recent w.r.t. this RGB frame
        dt = abs((msg.header.stamp - self._last_point_stamp).to_sec())
        if dt > self.time_tolerance_s:
            # too far apart in time, skip this sample
            return

        roi_xyz = np.array(self._last_point, dtype=np.float32)
        aruco_xyz = np.array(tvec, dtype=np.float32)
        diff = aruco_xyz - roi_xyz
        err_norm = float(np.linalg.norm(diff))

        # store sample
        self.samples.append({
            "timestamp": datetime.datetime.utcnow().isoformat(),
            "tag": self.tag,
            "aruco_id": self.aruco_id,
            "marker_length_m": self.marker_length,
            "roi_x": float(roi_xyz[0]), "roi_y": float(roi_xyz[1]), "roi_z": float(roi_xyz[2]),
            "aruco_x": float(aruco_xyz[0]), "aruco_y": float(aruco_xyz[1]), "aruco_z": float(aruco_xyz[2]),
            "dx": float(diff[0]), "dy": float(diff[1]), "dz": float(diff[2]),
            "err_norm": err_norm,
            "frame_id": self.frame_id
        })

        rospy.loginfo("Saved sample %d/%d  Δ=[%.4f, %.4f, %.4f] | |Δ|=%.4f m (dt=%.3fs)",
                      len(self.samples), self.max_measurements, diff[0], diff[1], diff[2], err_norm, dt)

        # finalize if we hit the target number
        if len(self.samples) >= self.max_measurements:
            self.finalize_and_shutdown()

    # ---------- finalize ----------
    def finalize_and_shutdown(self):
        # write CSV (append, create header if needed)
        header = ["timestamp","tag","aruco_id","marker_length_m",
                  "roi_x","roi_y","roi_z","aruco_x","aruco_y","aruco_z",
                  "dx","dy","dz","err_norm","frame_id"]
        write_header = not os.path.exists(self.csv_path)
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True) if os.path.dirname(self.csv_path) else None

        with open(self.csv_path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=header)
            if write_header:
                w.writeheader()
            for row in self.samples:
                w.writerow(row)

        rospy.loginfo("Wrote %d samples to %s. Shutting down, move the camera.",
                      len(self.samples), self.csv_path)
        rospy.signal_shutdown("Done collecting samples")

if __name__ == "__main__":
    rospy.init_node("roi_to_xyz_calibration")
    ROIToXYZ()
    rospy.spin()

