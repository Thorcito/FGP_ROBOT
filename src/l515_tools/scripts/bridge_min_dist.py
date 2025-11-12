#!/usr/bin/env python3
"""
KFtoEETargetBridgeMinDist (minimum-distance criterion)

Purpose
-------
- Subscribe to synchronized (exact time) filtered ball position and current velocity,
  both in the camera frame, then transform/compute targets in the WORLD frame.
- Build a continuous end-effector (EE) orientation that faces the incoming ball using
  an EMA (exponential moving average) of the predicted velocity direction.
- Solve two minimum-distance targeting strategies:
    1) Analytic "minimize_time" via stationary points of d^2(t) (cubic roots in [0,1]s).
    2) Discrete "minimize_distance" over a sampled future trajectory (uniform time grid).
- Publish:
    - A *primary* EETarget command using the discrete optimum (Optimization 2).
    - WORLD-space visualization of both optima (Point + short Path trails).
- Optionally publish a static camera TF if one is not provided elsewhere (for sim/demo).

Inputs (topics)
---------------
~position_topic : geometry_msgs/PointStamped   (default "/ball_pred/point_filt")
~velocity_topic : geometry_msgs/Vector3Stamped (default "/ball_pred/current_vel")
  NOTE: These are synchronized with message_filters.TimeSynchronizer (exact time).

Frames / TF
-----------
~world_frame      : World frame for control (default "world")
~ee_frame         : Current end-effector frame (default "ur10_model_dh_5")
~publish_camera_tf: If True, publish a static transform for the camera (for sim)
~camera_parent    : Parent of camera frame (default "world")
~camera_frame     : Camera optical frame id (default "camera_color_optical_frame")
~camera_xyz       : Camera translation wrt parent
~camera_quat_xyzw : Camera orientation wrt parent (x,y,z,w)

Orientation Logic
-----------------
- Rotate ball velocity (camera→world), require a minimum speed (vel_min_speed).
- Fixed dir_sign = -1.0 so the tool +Z faces the incoming ball.
- EMA smoothing (dir_alpha) provides a stable direction.
- "_candidate_for_z" chooses between the minimal-roll solution and a 180°-roll flip,
  selecting the one closest to the current EE orientation.

Targeting
---------
- Optimization 1 (analytic):
    * distance_squared(pe, p0, v0, t, g) := || pe - (p0 + v0 t + 1/2 g t^2) ||^2
    * derivative is cubic; roots in [0,1] are candidates, plus endpoints {0,1}.
    * Minimizes d^2(t) and returns (t*, d^2(t*)).
- Optimization 2 (discrete sampling):
    * Sample future trajectory for t ∈ [0, t_total] uniformly.
    * Evaluate squared distances to pe and choose argmin.

Publishing
----------
- If Optimization 1 yields t_star != 0 and f_star <= 0.1, publish:
    * "Bridge_minDist/punto" + "Bridge_minDist/path" for p_star,
    * "Bridge_minDist/punto_op2" + "Bridge_minDist/path_op2" for p_desired_opt2,
    * EETarget at the *Optimization 2* point/time (matching existing behavior).
- All topic names, frames, parameters, and logic are left untouched.

Notes
-----
- Gravity is expressed in WORLD: g_world = [0, 0, -9.81] (default).
- ExactTime sync is used (TimeSynchronizer), so both inputs must have identical stamps.
"""

import rospy, math, time
from tum_ics_ur10_controller_tutorial.msg import EETarget
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped, Vector3Stamped
from std_msgs.msg import Float32
from nav_msgs.msg import Path
import numpy as np
from collections import deque  # [CHG STAB]

import tf2_ros
from tf2_geometry_msgs import do_transform_point
import message_filters  # <-- exact time synchronization

class KFtoEETargetBridgeMinDist:
    def __init__(self):
        # -------- params --------
        self.position_topic = rospy.get_param("~position_topic", "/ball_pred/point_filt")
        self.velocity_topic = rospy.get_param("~velocity_topic", "/ball_pred/current_vel")

        self.world_frame    = rospy.get_param("~world_frame", "world")
        self.ee_frame       = rospy.get_param("~ee_frame", "ur10_model_dh_5")

        # Gravity in world (+Z up)
        self.g_world = np.array(rospy.get_param("~g_world", [0.0, 0.0, -9.81]), dtype=np.float64)

        # Camera TF
        self.publish_camera_tf = rospy.get_param("~publish_camera_tf", True)
        self.camera_parent     = rospy.get_param("~camera_parent", "world")
        self.camera_frame      = rospy.get_param("~camera_frame", "camera_color_optical_frame")
        self.cam_xyz           = rospy.get_param("~camera_xyz", [0.3026, -0.0604, 0.9295])
        self.cam_quat_xyzw     = rospy.get_param("~camera_quat_xyzw", [-0.5, -0.5, 0.5, 0.5])  # x y z w
        self.rot_angle = rospy.get_param("~rot_angle", 0)

        # Orientation tracking
        self.vel_min_speed = float(rospy.get_param("~vel_min_speed_mps", 0.6))
        self.up_world = np.array(rospy.get_param("~up_world", [0.0, 0.0, 1.0]), dtype=np.float64)
        self.dir_sign = -1.0
        self.dir_alpha = float(rospy.get_param("~dir_alpha", 0.5))

        # -------- state --------
        self.pos_ema = None
        self.old_x = self.old_y = self.old_z = None
        self.pts = []   # trail for Optimization 1 visualization
        self.pts_2 = [] # trail for Optimization 2 visualization

        # -------- TF2 --------
        self.buf = tf2_ros.Buffer(cache_time=rospy.Duration(2.0))
        self.listener = tf2_ros.TransformListener(self.buf)
        if self.publish_camera_tf:
            self._publish_static_camera_tf()

        # -------- pubs --------
        self.pub = rospy.Publisher("ee_target", EETarget, queue_size=1)
        self.pub_point = rospy.Publisher("Bridge_minDist/punto", PointStamped, queue_size=10)
        self.pub_path  = rospy.Publisher("Bridge_minDist/path", Path, queue_size=10)
        self.pub_point_op2 = rospy.Publisher("Bridge_minDist/punto_op2", PointStamped, queue_size=10)
        self.pub_path_op2  = rospy.Publisher("Bridge_minDist/path_op2", Path, queue_size=10)

        # -------- exact time sync subscribers --------
        # Use exact timestamps to pair position and velocity from the predictor.
        pos_sub = message_filters.Subscriber(self.position_topic, PointStamped)
        vel_sub = message_filters.Subscriber(self.velocity_topic, Vector3Stamped)
        ts = message_filters.TimeSynchronizer([pos_sub, vel_sub], queue_size=25)
        ts.registerCallback(self.on_pair)

        rospy.loginfo(f"[bridge_minDist] ready")

    # ---------- helpers ----------
    def _publish_static_camera_tf(self):
        """Optionally publish a static transform for the camera (parent -> camera_frame)."""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.camera_parent
        t.child_frame_id  = self.camera_frame
        t.transform.translation.x = float(self.cam_xyz[0])
        t.transform.translation.y = float(self.cam_xyz[1])
        t.transform.translation.z = float(self.cam_xyz[2])
        q1x, q1y, q1z, q1w = self.cam_quat_xyzw
        q2x, q2y, q2z, q2w = np.array([np.sin(np.radians(self.rot_angle)/2), 0, 0 , np.cos(np.radians(self.rot_angle)/2)])
        qx = q1w*q2x + q1x*q2w + q1y*q2z - q1z*q2y
        qy = q1w*q2y - q1x*q2z + q1y*q2w + q1z*q2x
        qz = q1w*q2z + q1x*q2y - q1y*q2x + q1z*q2w
        qw = q1w*q2w - q1x*q2x - q1y*q2y - q1z*q2z
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_broadcaster.sendTransform(t)

    def _rotate_vec_by_tf(self, v_cam, world_frame, cam_frame):
        """
        Rotate a vector given in cam_frame into world_frame using the TF orientation.
        Only orientation is applied; translation is irrelevant for vectors.
        """
        Tcw = self.buf.lookup_transform(world_frame, cam_frame, rospy.Time(0), rospy.Duration(0.05))
        q = Tcw.transform.rotation
        qc = np.array([q.w, q.x, q.y, q.z], dtype=np.float64)  # [w,x,y,z]
        x, y, z = v_cam
        # Standard quaternion-vector rotation: v' = v + 2w(q_vec×v) + 2(q_vec×(q_vec×v))
        qw, qx, qy, qz = qc
        t = 2.0 * np.cross([qx, qy, qz], [x, y, z])
        v_world = [x, y, z] + qw * t + np.cross([qx, qy, qz], t)
        return np.array(v_world, dtype=np.float64)

    @staticmethod
    def _unit(v):
        """Safe unit vector helper (avoids division by zero)."""
        n = float(np.linalg.norm(v))
        return v / (n + 1e-12)

    # --- orientation helpers (unchanged) ---
    def _R_from_quat_xyzw(self, q):
        """Quaternion [x,y,z,w] → rotation matrix."""
        x, y, z, w = q
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        return np.array([
            [1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)],
            [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)],
            [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)]
        ], dtype=np.float64)

    def _quat_from_R(self, R):
        """Rotation matrix → normalized quaternion [x,y,z,w]."""
        tr = np.trace(R)
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2.0
            qw = 0.25 * S
            qx = (R[2,1] - R[1,2]) / S
            qy = (R[0,2] - R[2,0]) / S
            qz = (R[1,0] - R[0,1]) / S
        else:
            i = np.argmax([R[0,0], R[1,1], R[2,2]])
            if i == 0:
                S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
                qw = (R[2,1] - R[1,2]) / S; qx = 0.25 * S; qy = (R[0,1] + R[1,0]) / S; qz = (R[0,2] + R[2,0]) / S
            elif i == 1:
                S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
                qw = (R[0,2] - R[2,0]) / S; qx = (R[0,1] + R[1,0]) / S; qy = 0.25 * S; qz = (R[1,2] + R[2,1]) / S
            else:
                S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
                qw = (R[1,0] - R[0,1]) / S; qx = (R[0,2] + R[2,0]) / S; qy = (R[1,2] + R[2,1]) / S; qz = 0.25 * S
        q = np.array([qx, qy, qz, qw], dtype=np.float64)
        q /= (np.linalg.norm(q) + 1e-12)
        return q

    def _rot_about_axis_matrix(self, k, phi):
        """Rodrigues' rotation formula for rotation about axis k by angle phi."""
        k = self._unit(k)
        K = np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]], dtype=np.float64)
        I = np.eye(3)
        return I*math.cos(phi) + math.sin(phi)*K + (1-math.cos(phi))*np.outer(k,k)

    def _frame_from_dir_with_up_z(self, dir_world, up_world):
        """
        Build a tool frame with +Z aligned to dir_world and roll stabilized by up_world.
        Returns R whose columns are the tool axes in WORLD (col 2 is +Z).
        """
        z = self._unit(dir_world)
        up = self._unit(up_world)
        if abs(np.dot(z, up)) > 0.98:
            up = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        x = self._unit(np.cross(up, z))
        y = np.cross(z, x)
        return np.column_stack([x, y, z])

    def _candidate_for_z(self, dir_world, up_world, q_now_xyzw):
        """
        Choose between minimal-roll and 180°-roll solutions, relative to current EE pose.
        Returns (best_quaternion_xyzw, angle_to_current).
        """
        R_now = self._R_from_quat_xyzw(q_now_xyzw)
        R_base = self._frame_from_dir_with_up_z(dir_world, up_world)
        z = R_base[:, 2]
        x_now = R_now[:, 0]
        def proj_plane(v):
            return self._unit(v - z * np.dot(z, v))
        x_p = proj_plane(x_now)
        x0  = R_base[:, 0]
        s = np.dot(z, np.cross(x0, x_p))
        c = np.dot(x0, x_p)
        phi = math.atan2(s, c)
        R_opt = self._rot_about_axis_matrix(z, phi) @ R_base
        # 180° roll candidate
        R_flip = R_base.copy(); R_flip[:, 0] *= -1.0; R_flip[:, 1] *= -1.0
        x0f = R_flip[:, 0]
        s2 = np.dot(z, np.cross(x0f, x_p)); c2 = np.dot(x0f, x_p)
        phi2 = math.atan2(s2, c2)
        R_opt2 = self._rot_about_axis_matrix(z, phi2) @ R_flip

        q_now = self._quat_from_R(R_now)
        q1 = self._quat_from_R(R_opt)
        q2 = self._quat_from_R(R_opt2)
        def quat_angle(qA, qB):
            dot = float(np.dot(qA, qB)); dot = abs(max(min(dot, 1.0), -1.0))
            return 2.0 * math.acos(dot)
        return (q1, quat_angle(q_now, q1)) if quat_angle(q_now, q1) <= quat_angle(q_now, q2) else (q2, quat_angle(q_now, q2))
    
    def distance_squared(self, pos_robot, p0_ball, v0_ball, t, g):
        """Squared distance between robot position and ball position at time t."""
        ball_tray = p0_ball + v0_ball*t + 0.5*g*t*t
        dist = pos_robot - ball_tray
        return float(dist@dist)
    
    def derivate_distance_squared(self, pos_robot, p0_ball, v0_ball, g):
        """
        Solve d/dt || pe - (p0 + v0 t + 1/2 g t^2) ||^2 = 0
        => cubic A t^3 + B t^2 + C t + D = 0 (coefficients below).
        Returns complex roots; caller filters real roots in [t_min, t_max].
        """
        #Given the equation At3+Bt2+Ct+D=0
        A = 0.5*(g@g) #0.5g²*t³
        B = -1.5*g@v0_ball #-1.5*g*v*t²
        C = v0_ball@v0_ball+g@(pos_robot-p0_ball) #(v²+g(p_r-p_b))
        D = -(pos_robot-p0_ball)@v0_ball #-v(p_r-p_b)
        coeff = np.array([A, B, C, D])
        roots = np.roots(coeff)
        #rospy.logerr(f"[brrdge] ROOTS: ({roots})." )
        return roots
    
    def future_trajectories(self, p0_ball, v0_ball, g, t_total=1.0, n_points=10):
        """
        Sample the future ballistic trajectory at n_points over [0, t_total].
        Returns (times, trajectory_points[N,3]).
        """
        # Generate evenly spaced time steps from 0 to t_total
        t = np.linspace(0, t_total, n_points)
        t_col = t[:, None]                                 
        traj = p0_ball + v0_ball*t_col + 0.5*g*(t_col**2)   
        return t, traj
    
    def minimize_distance(self, pos_robot, trajectories, times):
        """
        Discrete argmin over sampled positions: choose index with minimum squared distance.
        Returns (time, closest_point[3]).
        """
        pos_robot = np.asarray(pos_robot).reshape(1, 3)
        trajectories = np.asarray(trajectories)
        diff = trajectories - pos_robot                     
        min_distances = 0.5 * np.sum(diff**2, axis=1)                   
        idx = int(np.argmin(min_distances))
        closet_point = trajectories[idx]
        time = times[idx]

        return time, closet_point
    
    def minimize_time(self, pos_robot, p0_ball, v0_ball, g):
        """
        Analytic candidate times from cubic roots (and endpoints) over [0,1]s.
        Returns (best_time, best_distance_sq).
        """
        #set minimun and maximun times to solve
        t_min = 0.0
        t_max = 1.0
        t_possible = self.derivate_distance_squared(pos_robot, p0_ball, v0_ball, g)
        candidates = []
        for r in t_possible:
            if abs(r.imag) < 1e-9: #real root
                t_real = float(r.real)
                if t_min <= t_real <= t_max:
                    candidates.append(t_real)
        candidates.extend([t_min, t_max])
        distance = [self.distance_squared(pos_robot, p0_ball, v0_ball, t_real, g) for t_real in candidates]
        desired_time = int(np.argmin(distance))
        return float(candidates[desired_time]), float(distance[desired_time])
    
    # ---------- paired callback ----------
    def on_pair(self, p_cam: PointStamped, v_cam: Vector3Stamped):
        """
        Exact-time synchronized callback with filtered position and current velocity.
        - Rotate velocity cam→world; require minimal speed.
        - Update EMA direction for orientation.
        - Transform filtered position cam→world.
        - Compute Optimization 1 (analytic) and 2 (discrete sampling).
        - Publish RViz markers for both; send EETarget using Optimization 2 result.
        """
        cam_frame = self.camera_frame
        try:
            velocity = np.array([v_cam.vector.x, v_cam.vector.y, v_cam.vector.z], dtype=np.float64)
            v_world = self._rotate_vec_by_tf(velocity, self.world_frame, cam_frame)
        except Exception as e:
            rospy.logwarn_throttle(0.5, f"[bridge] vhit TF rot failed: {e}")
            return

        if np.linalg.norm(v_world) < self.vel_min_speed:
            return

        # EMA direction (fixed sign = -1 → face ball)
        p_raw = self._unit(self.dir_sign * v_world)
        if self.pos_ema is None:
            self.pos_ema = p_raw
        self.pos_ema = self._unit((1.0 - self.dir_alpha) * self.pos_ema + self.dir_alpha * p_raw)

        # Current EE pose
        try:
            T_w_ee = self.buf.lookup_transform(self.world_frame, self.ee_frame,
                                               rospy.Time(0), rospy.Duration(0.02))
            pe = np.array([T_w_ee.transform.translation.x,
                           T_w_ee.transform.translation.y,
                           T_w_ee.transform.translation.z], dtype=np.float64)
            qee = T_w_ee.transform.rotation
            q_ee = np.array([qee.x, qee.y, qee.z, qee.w], dtype=np.float64)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[bridge_minDist] EE TF lookup failed ({self.ee_frame}→{self.world_frame}): {e}")
            return
        
        # Transform camera → world for filtered point
        try:
            Tcw = self.buf.lookup_transform(self.world_frame, cam_frame,
                                            rospy.Time(0), rospy.Duration(0.05))
            p_world_geo = do_transform_point(p_cam, Tcw)
            p_world = p_world_geo.point
            point = np.array([p_world.x, p_world.y, p_world.z], dtype=np.float64)
        except Exception as e:
            rospy.logwarn_throttle(0.0, f"[bridge] TF transform failed ({cam_frame}→{self.world_frame}): {e}")
            return
        
        # Optimization 1 (analytic)
        vel_no_q = np.array([v_world[0], v_world[1], v_world[2]], dtype=np.float64)
        t_star, f_star = self.minimize_time(pe, point, vel_no_q, self.g_world)
        p_star = point + vel_no_q*t_star + 0.5*self.g_world*(t_star*t_star)
    
        # Optimization 2 (discrete sampling)
        times_opt2, poss_opt2 = self.future_trajectories(point, vel_no_q, self.g_world, t_total=1, n_points=10)
        t_desired_opt2, p_desired_opt2 = self.minimize_distance(pe, poss_opt2, times_opt2)

        if t_star != 0:
            rospy.logwarn(f"[bridge] T: {t_star} , Min_Dist: {f_star}" )
            rospy.loginfo(f"[bridge] POS: {p_star} " )
            rospy.loginfo(f"[bridge] EE: {pe} " )
            rospy.logwarn(f"[bridge] T_OPT2: {t_desired_opt2} , POS_OPT: ({p_desired_opt2})" )
        else:
            rospy.logerr(f"[bridge] No valid root found T: {t_star} " )

        # Choose orientation for this publish (EMA-based if available; else current EE)
        if self.pos_ema is not None:
            q_tgt_xyzw, _ = self._candidate_for_z(self.pos_ema, self.up_world, q_ee)
            use_q = q_tgt_xyzw
        else:
            use_q = q_ee

        # Publish only if analytic solution is non-zero and close enough (f_star threshold)
        if t_star != 0 and f_star <= 0.5:
            stamp = p_cam.header.stamp if p_cam.header.stamp.to_sec() > 0 else rospy.Time.now()

            # Publish Optimization 1 point & short trail
            pto_w = PointStamped()
            pto_w.header.stamp = stamp
            pto_w.header.frame_id = self.world_frame
            pto_w.point.x, pto_w.point.y, pto_w.point.z = p_star[0], p_star[1], p_star[2]
            self.pub_point.publish(pto_w)

            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.world_frame
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p_star[0], p_star[1], p_star[2]
            pose.pose.orientation.w = 1.0
            self.pts.append(pose)
            if len(self.pts) > 20: self.pts = self.pts[-20:]
            path_msg = Path(); path_msg.header.stamp = stamp; path_msg.header.frame_id = self.world_frame
            path_msg.poses = list(self.pts)
            self.pub_path.publish(path_msg)

            # Publish Optimization 2 point & short trail (for comparison)
            stamp = p_cam.header.stamp if p_cam.header.stamp.to_sec() > 0 else rospy.Time.now()
            pto_op2 = PointStamped()
            pto_op2.header.stamp = stamp
            pto_op2.header.frame_id = self.world_frame
            pto_op2.point.x, pto_op2.point.y, pto_op2.point.z = p_desired_opt2[0], p_desired_opt2[1], p_desired_opt2[2]
            self.pub_point_op2.publish(pto_op2)

            pose_op2 = PoseStamped()
            pose_op2.header.stamp = stamp
            pose_op2.header.frame_id = self.world_frame
            pose_op2.pose.position.x, pose_op2.pose.position.y, pose_op2.pose.position.z = p_desired_opt2[0], p_desired_opt2[1], p_desired_opt2[2]
            pose_op2.pose.orientation.w = 1.0
            self.pts_2.append(pose_op2)
            if len(self.pts_2) > 20: self.pts_2 = self.pts_2[-20:]
            path_msg_op2 = Path(); path_msg_op2.header.stamp = stamp; path_msg_op2.header.frame_id = self.world_frame
            path_msg_op2.poses = list(self.pts_2)
            self.pub_path_op2.publish(path_msg_op2)

            # Controller command: EETarget uses Optimization 2 result (as in your code)
            msg = EETarget()
            msg.ee_target.position.x = p_star[0]
            msg.ee_target.position.y = p_star[1]
            msg.ee_target.position.z = p_star[2]
            msg.ee_target.orientation.x = float(use_q[0])
            msg.ee_target.orientation.y = float(use_q[1])
            msg.ee_target.orientation.z = float(use_q[2])
            msg.ee_target.orientation.w = float(use_q[3])
            msg.duration = t_star
            self.pub.publish(msg)
            rospy.loginfo(f"Enviado" )


        # Update state (last published Optimization 1 point; used as previous point)
        self.old_x, self.old_y, self.old_z = p_star[0], p_star[1], p_star[2]

if __name__ == "__main__":
    rospy.init_node("kf_to_ee_target_bridgeMinDist_stable_latch")
    KFtoEETargetBridgeMinDist()
    rospy.spin()
