#!/usr/bin/env python3
import rospy, math
from tum_ics_ur10_controller_tutorial.msg import EETarget
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped
from std_msgs.msg import Float32, Header
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import numpy as np

import tf2_ros
from message_filters import ApproximateTimeSynchronizer, Subscriber

class Minimum_Distance:
    def __init__(self):
        # -------- params --------
        self.position_topic = rospy.get_param("~position_topic", "/world_ball_pred/current_pos")
        self.velocity_topic = rospy.get_param("~velocity_topic", "/world_ball_pred/current_vel")

        self.world_frame    = rospy.get_param("~world_frame", "world")
        self.ee_frame       = rospy.get_param("~ee_frame", "ur10_model_dh_5")

        # Gravity in world (+Z up)
        self.g_world = np.array(rospy.get_param("~g_world", [0.0, 0.0, -9.81]), dtype=np.float64)

        # Orientation tracking
        self.vel_min_speed = float(rospy.get_param("~vel_min_speed_mps", 0.6))
        self.up_world = np.array(rospy.get_param("~up_world", [0.0, 0.0, 1.0]), dtype=np.float64)
        self.dir_sign = -1.0
        self.dir_alpha = float(rospy.get_param("~dir_alpha", 0.5))

        # -------- state --------
        self.pos_ema = None
        self.hit_history = []
        self.old_x = self.old_y = self.old_z = None
        self.last_t_hit = None
        self.max_step_m     = rospy.get_param("~max_step_m", 0.13)
        self.first_reading = False
        self.hit_history_max = int(rospy.get_param("~hit_history_max", 20))
        self.option = rospy.get_param("~option", 1)

        # -------- TF2 --------
        self.buf = tf2_ros.Buffer(cache_time=rospy.Duration(2.0))
        self.listener = tf2_ros.TransformListener(self.buf)

        # -------- pubs --------
        self.pub = rospy.Publisher("ee_target", EETarget, queue_size=1)
        self.pub_hit_point = rospy.Publisher("/MinDist/hit_point", PointStamped, queue_size=10)
        self.pub_hit_time  = rospy.Publisher("/MinDist/hit_time_s", Float32, queue_size=10)
        self.pub_hit_history = rospy.Publisher("/MinDist/hit_history", Path, queue_size=10)
        self.pub_pred_vel_hit = rospy.Publisher("/MinDist/pred_vel_hit", Vector3Stamped, queue_size=10)

        # -------- exact time sync subscribers --------
        pos_sub = Subscriber(self.position_topic, PointStamped)
        vel_sub = Subscriber(self.velocity_topic, Vector3Stamped)
        ats_slop = float(rospy.get_param("~sync_slop_s", 0.05))  # 20 ms default
        ts = ApproximateTimeSynchronizer([pos_sub, vel_sub], queue_size=25, slop=ats_slop, allow_headerless=False)
        ts.registerCallback(self.on_pair)

        rospy.loginfo(f"[bridge_minimunDistance] ready option : {self.option}")

    # ---------- helpers ----------

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
    def on_pair(self, p_world: PointStamped, v_world: Vector3Stamped):
        stamp = p_world.header.stamp
        hdr = Header(stamp=stamp, frame_id=self.world_frame)
        v_world_np = np.array([v_world.vector.x, v_world.vector.y, v_world.vector.z], dtype=np.float64)
        if np.linalg.norm(v_world_np) < self.vel_min_speed:
            rospy.logerr("No movement")
            return

        # Current EE pose
        try:
            T_w_ee = self.buf.lookup_transform(self.world_frame, self.ee_frame,
                                               rospy.Time(0), rospy.Duration(0.02))
            qee = T_w_ee.transform.rotation
            q_ee = np.array([qee.x, qee.y, qee.z, qee.w], dtype=np.float64)
            p_ee = np.array([T_w_ee.transform.translation.x,
                           T_w_ee.transform.translation.y,
                           T_w_ee.transform.translation.z], dtype=np.float64)
            #rospy.loginfo(f"End effector: {q_ee}")
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[bridge_minDist] EE TF lookup failed ({self.ee_frame}→{self.world_frame}): {e}")
            return
        
        point = np.array([p_world.point.x , p_world.point.y, p_world.point.z], dtype=np.float64)

        # Minimun Distance Criterion
        # Optimization 1 (analytic)
        vel_no_q = v_world_np
        t_star, f_star = self.minimize_time(p_ee, point, vel_no_q, self.g_world)
        p_star = point + vel_no_q*t_star + 0.5*self.g_world*(t_star*t_star)
        if t_star != 0:
            rospy.logwarn(f"[bridge] time: {t_star} , pos: {p_star} , error: {f_star}")
        

        # Optimization 2 (discrete sampling)
        times_opt2, poss_opt2 = self.future_trajectories(point, vel_no_q, self.g_world, t_total=1, n_points=10)
        t_desired_opt2, p_desired_opt2 = self.minimize_distance(p_ee, poss_opt2, times_opt2)

        if t_star != 0:
            rospy.logerr(f"[bridge] Valid root found T: {t_star} " )
        else:
            rospy.logerr(f"[bridge] No valid root found T: {t_star} " )

        if self.option == 1:
            v_hit_world = v_world_np + self.g_world * t_star
        else:
            v_hit_world = v_world_np + self.g_world * t_desired_opt2
        
        # Publish
        if (t_star != 0) and (f_star <= 0.15):
            pt_h = PointStamped()
            pt_h.header = hdr
            if self.option == 1:
                pt_h.point.x, pt_h.point.y, pt_h.point.z = float(p_star[0]), float(p_star[1]), float(p_star[2])
            else: 
                pt_h.point.x, pt_h.point.y, pt_h.point.z = float(p_desired_opt2[0]), float(p_desired_opt2[1]), float(p_desired_opt2[2])
            self.pub_hit_point.publish(pt_h)
            self.pub_hit_time.publish(Float32(data=float(t_star)))
            rospy.loginfo_throttle(0.0, f"Time to intercept: (t={t_star:.6f})")

            pose_h = PoseStamped(); pose_h.header = hdr
            pose_h.pose.position.x = pt_h.point.x
            pose_h.pose.position.y = pt_h.point.y
            pose_h.pose.position.z = pt_h.point.z
            pose_h.pose.orientation.w = 1.0
            self.hit_history.append(pose_h)
            if len(self.hit_history) > self.hit_history_max:
                self.hit_history = self.hit_history[-self.hit_history_max:]

            path_h = Path(); path_h.header = hdr
            path_h.poses = self.hit_history
            self.pub_hit_history.publish(path_h)

            vel_hit_msg = Vector3Stamped()
            vel_hit_msg.header = hdr
            vel_hit_msg.vector.x, vel_hit_msg.vector.y, vel_hit_msg.vector.z = v_hit_world.tolist()
            self.pub_pred_vel_hit.publish(vel_hit_msg)

            # EMA direction (fixed sign = -1 → face ball)
            p_raw = self._unit(self.dir_sign * v_hit_world)
            if self.pos_ema is None:
                self.pos_ema = p_raw
            self.pos_ema = self._unit((1.0 - self.dir_alpha) * self.pos_ema + self.dir_alpha * p_raw)

            # Choose orientation for this publish (EMA-based if available; else current EE)
            if self.pos_ema is not None:
                q_tgt_xyzw, _ = self._candidate_for_z(self.pos_ema, self.up_world, q_ee)
                use_q = q_tgt_xyzw
            else:
                use_q = q_ee

            # Controller command: EETarget uses Optimization 2 result (as in your code)

            half = float(rospy.get_param("~ee_gate_half_m", 0.50))
            if self.option == 1:
                #rospy.loginfo({abs(p_star[1] - p_ee[1])})
                inside_square = (abs(p_star[1] - p_ee[1]) <= half) and (abs(p_star[2] - p_ee[2]) <= half) and (abs(p_star[0] - p_ee[0])<= half)
                #bounce = point[0] <= self.old_x
            else:
                #rospy.loginfo({abs(p_desired_opt2[1] - p_ee[1])})
                inside_square = (abs(p_desired_opt2[1] - p_ee[1]) <= half) and (abs(p_desired_opt2[2] - p_ee[2]) <= half) and (abs(p_desired_opt2[0] - p_ee[0])<= half)
                #bounce = point[0] <= self.old_x
            #if not bounce:
                rospy.logerr("BOUNCE DETECTED")
            if inside_square:
                rospy.loginfo(f"[bridge] EE_POS: {p_ee} " )
                rospy.logwarn(f"[bridge] T_HIT: {t_star} , P_HIT: {p_star}" )
                rospy.logwarn(f"[bridge] T_HIT_2: {t_desired_opt2} , P_HIT_2: ({p_desired_opt2})" )
                msg = EETarget()
                msg.ee_target.position.x = pt_h.point.x
                msg.ee_target.position.y = pt_h.point.y
                msg.ee_target.position.z = pt_h.point.z
                msg.ee_target.orientation.x = float(use_q[0])
                msg.ee_target.orientation.y = float(use_q[1])
                msg.ee_target.orientation.z = float(use_q[2])
                msg.ee_target.orientation.w = float(use_q[3])
                if self.option == 1:
                    msg.duration = t_star
                else:
                    msg.duration = t_desired_opt2
                self.pub.publish(msg)

        if self.option == 1:
            self.old_x, self.old_y, self.old_z = p_star[0], p_star[1], p_star[2]
            self.last_t_hit = t_star
            self.first_reading = True
        else:
            self.old_x, self.old_y, self.old_z = p_desired_opt2[0], p_desired_opt2[1], p_desired_opt2[2]
            self.last_t_hit = t_desired_opt2
            self.first_reading = True

if __name__ == "__main__":
    rospy.init_node("Minimum_Distance")
    Minimum_Distance()
    rospy.spin()