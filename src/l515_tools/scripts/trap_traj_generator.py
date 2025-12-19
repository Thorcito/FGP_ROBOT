#!/usr/bin/env python3
import math
import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped, TwistStamped
from tum_ics_ur10_controller_tutorial.msg import EETarget
from std_msgs.msg import Float32

# ----------------------------
# Quaternion helpers (x,y,z,w)
# ----------------------------
def quat_norm(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x/n, y/n, z/n, w/n)

def quat_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)

def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    # Hamilton product
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    return (x, y, z, w)

def quat_inv(q):
    # assume unit quaternion
    return quat_conj(q)

def quat_to_axis_angle(q):
    """
    q should be unit quaternion representing rotation.
    returns (axis_x, axis_y, axis_z, angle) with angle in [0, pi]
    """
    x, y, z, w = quat_norm(q)

    # Clamp w for numeric stability
    w = max(-1.0, min(1.0, w))

    angle = 2.0 * math.acos(w)  # [0, 2pi]
    if angle > math.pi:
        # Use the shorter rotation: flip quaternion
        x, y, z, w = (-x, -y, -z, -w)
        w = max(-1.0, min(1.0, w))
        angle = 2.0 * math.acos(w)

    s = math.sqrt(max(0.0, 1.0 - w*w))  # = |sin(angle/2)|
    if s < 1e-9 or angle < 1e-9:
        # Axis arbitrary if angle ~ 0
        return (1.0, 0.0, 0.0, 0.0)

    ax = x / s
    ay = y / s
    az = z / s
    return (ax, ay, az, angle)

def axis_angle_to_quat(axis, angle):
    ax, ay, az = axis
    half = 0.5 * angle
    s = math.sin(half)
    return quat_norm((ax*s, ay*s, az*s, math.cos(half)))

# ----------------------------
# Trapezoidal (LSPB) on [0, D]
# with v_peak chosen so it fits exactly into duration T
# This is a symmetric accel/decel profile (no jerk limiting)
# ----------------------------
def lspb_profile(D, T):
    """
    Build a symmetric trapezoid/triangle profile for distance D in time T.
    Returns a dict with parameters used by eval_lspb().
    Works with D >= 0.
    """

    D = abs(D)
    if T <= 1e-6 or D <= 1e-12:
        # trivial
        return {
            "D": D, "T": max(T, 1e-6),
            "triangular": True,
            "tb": 0.0,
            "v": 0.0,
            "a": 0.0,
        }

    # For a given blend time tb (0 < tb <= T/2),
    # distance is D = v*(T - tb), with v = a*tb.
    # With symmetry: a = v/tb.
    # Choose tb = T/4 -> gives a reasonable trapezoid (or triangle if needed)
    # but we need to ensure it's feasible. We'll solve for tb:
    #
    # If we select v (peak velocity), then:
    #   tb = T - D/v
    #   a  = v/tb
    # and require 0 < tb <= T/2.
    #
    # Pick v as the minimum that avoids negative tb:
    # v must be > D/T.
    # We'll pick tb = T/4 target, solve v = D/(T - tb).
    tb_target = 0.25 * T
    v = D / max(1e-9, (T - tb_target))
    tb = T - (D / max(1e-12, v))

    # If tb too large -> triangle case (no constant-velocity segment)
    if tb > 0.5 * T:
        # Triangular: tb = T/2, v = 2D/T, a = 4D/T^2
        tb = 0.5 * T
        v = (2.0 * D) / T
        a = v / tb if tb > 1e-9 else 0.0
        return {"D": D, "T": T, "triangular": True, "tb": tb, "v": v, "a": a}

    # Normal trapezoid
    a = v / tb if tb > 1e-9 else 0.0
    return {"D": D, "T": T, "triangular": False, "tb": tb, "v": v, "a": a}

def eval_lspb(t, prof):
    """
    Evaluate position s(t) and velocity sdot(t) for LSPB from 0 to D over T.
    """
    D = prof["D"]
    T = prof["T"]
    tb = prof["tb"]
    v = prof["v"]
    a = prof["a"]

    if t <= 0.0:
        return 0.0, 0.0
    if t >= T:
        return D, 0.0

    if prof["triangular"]:
        # accel then decel, switch at tb = T/2
        if t < tb:
            s = 0.5 * a * t*t
            sd = a * t
            return s, sd
        else:
            # mirror
            td = T - t
            s_from_end = 0.5 * a * td*td
            s = D - s_from_end
            sd = a * td
            return s, sd

    # trapezoid
    if t < tb:
        s = 0.5 * a * t*t
        sd = a * t
        return s, sd
    elif t <= (T - tb):
        s = 0.5 * a * tb*tb + v * (t - tb)
        sd = v
        return s, sd
    else:
        td = T - t
        s_from_end = 0.5 * a * td*td
        s = D - s_from_end
        sd = a * td
        return s, sd

# ----------------------------
# Planner Node
# ----------------------------
class CartesianTrapezoidPlanner:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.ee_target_topic = rospy.get_param("~ee_target_topic", "ee_target")
        self.des_pose_topic = rospy.get_param("~desired_pose_topic", "desired_pose")
        self.des_twist_topic = rospy.get_param("~desired_twist_topic", "desired_twist")
        self.publish_rate = rospy.get_param("~rate", 200.0)  # Hz
        self.ee_meas_topic = rospy.get_param("~ee_meas_topic", "/PlaneInt/end_effector")
        self.des_time_topics = rospy.get_param("~desired_time_topics", "desired_time")
        


        self.pub_pose = rospy.Publisher(self.des_pose_topic, PoseStamped, queue_size=1)
        self.pub_twist = rospy.Publisher(self.des_twist_topic, TwistStamped, queue_size=1)
        self.sub = rospy.Subscriber(self.ee_target_topic, EETarget, self.on_target, queue_size=1)
        self.sub_ee_meas = rospy.Subscriber(self.ee_meas_topic, PoseStamped, self.on_ee_meas, queue_size=1)
        self.pub_time = rospy.Publisher(self.des_time_topics, Float32, queue_size=1)


        self.have_meas = False
        self.meas_p = (0.0, 0.0, 0.0)
        self.meas_q = (0.0, 0.0, 0.0, 1.0)
        self.meas_stamp = rospy.Time(0)

        # Current planned segment
        self.active = False
        self.t0 = None
        self.T = 0.0

        # Start and goal pose
        self.p0 = (0.0, 0.0, 0.0)
        self.q0 = (0.0, 0.0, 0.0, 1.0)
        self.pf = (0.0, 0.0, 0.0)
        self.qf = (0.0, 0.0, 0.0, 1.0)

        # Trapezoids for xyz and angle
        self.prof_x = None
        self.prof_y = None
        self.prof_z = None
        self.prof_ang = None

        # Orientation axis-angle parameters (in start frame sense)
        self.axis = (1.0, 0.0, 0.0)
        self.total_angle = 0.0

        # We need an initial "current pose" estimate.
        # For now, we bootstrap from the first target message (start = current desired output),
        # and then we continue from last published desired state on updates.
        self.last_des_p = None
        self.last_des_q = None

        # in __init__
        self.last_cmd_v = np.zeros(3)   # last published linear vel
        self.last_cmd_w = np.zeros(3)   # last published angular vel (optional)
        self.have_last_cmd = False


        rospy.loginfo("CartesianTrapezoidPlanner ready. Sub: %s Pub: %s, %s",
                      self.ee_target_topic, self.des_pose_topic, self.des_twist_topic)
    
    def on_ee_meas(self, msg: PoseStamped):
        self.meas_p = (msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z)

        self.meas_q = quat_norm((msg.pose.orientation.x,
                                msg.pose.orientation.y,
                                msg.pose.orientation.z,
                                msg.pose.orientation.w))

        self.meas_stamp = msg.header.stamp
        self.have_meas = True


    def on_target(self, msg: EETarget):
        # Target pose
        pf = (msg.ee_target.position.x,
              msg.ee_target.position.y,
              msg.ee_target.position.z)
        qf = (msg.ee_target.orientation.x,
              msg.ee_target.orientation.y,
              msg.ee_target.orientation.z,
              msg.ee_target.orientation.w)
        qf = quat_norm(qf)

        T = float(msg.duration)
        if T <= 1e-3:
            T = 1e-3

        # Start pose: measured EE pose (actual robot), fallback to last desired if needed
        if self.have_meas:
            p0 = self.meas_p
            q0 = self.meas_q
        else:
            # fallback (only if measured pose hasn't arrived yet)
            if self.last_des_p is None or self.last_des_q is None:
                p0 = pf
                q0 = qf
            else:
                p0 = self.last_des_p
                q0 = self.last_des_q

        # Compute quaternion error from start to goal: q_err = qf * inv(q0)
        q_err = quat_mul(qf, quat_inv(q0))
        ax, ay, az, ang = quat_to_axis_angle(q_err)

        self.p0, self.q0 = p0, q0
        self.pf, self.qf = pf, qf
        self.T = T
        self.t0 = rospy.Time.now()
        self.active = True

        dx = pf[0] - p0[0]
        dy = pf[1] - p0[1]
        dz = pf[2] - p0[2]

        self.prof_x = lspb_profile(abs(dx), T)
        self.prof_y = lspb_profile(abs(dy), T)
        self.prof_z = lspb_profile(abs(dz), T)
        self.prof_ang = lspb_profile(abs(ang), T)

        self.axis = (ax, ay, az) if ang > 1e-9 else (1.0, 0.0, 0.0)
        self.total_angle = ang

        rospy.logwarn("NEW TARGET: T=%.3f | dp=(%.4f, %.4f, %.4f) | angle=%.4f rad axis=(%.3f, %.3f, %.3f)",
                      T, dx, dy, dz, ang, self.axis[0], self.axis[1], self.axis[2])

    def step(self):
        if not self.active or self.t0 is None:
            return

        t = (rospy.Time.now() - self.t0).to_sec()
        if t < 0.0:
            t = 0.0

        # --- Translation ---
        dx = self.pf[0] - self.p0[0]
        dy = self.pf[1] - self.p0[1]
        dz = self.pf[2] - self.p0[2]

        sx, sdx = eval_lspb(t, self.prof_x)
        sy, sdy = eval_lspb(t, self.prof_y)
        sz, sdz = eval_lspb(t, self.prof_z)

        # Restore sign
        x = self.p0[0] + math.copysign(sx, dx)
        y = self.p0[1] + math.copysign(sy, dy)
        z = self.p0[2] + math.copysign(sz, dz)

        vx = math.copysign(sdx, dx)
        vy = math.copysign(sdy, dy)
        vz = math.copysign(sdz, dz)

        # --- Orientation via axis-angle trapezoid on angle ---
        s_ang, sdot_ang = eval_lspb(t, self.prof_ang)
        # Restore sign to follow shortest-path direction already encoded in total_angle (>=0)
        alpha = s_ang  # [0..total_angle]
        alpha_dot = sdot_ang

        q_inc = axis_angle_to_quat(self.axis, alpha)
        q_des = quat_mul(q_inc, self.q0)
        q_des = quat_norm(q_des)

        # Angular velocity (approx) = axis * alpha_dot
        wx = self.axis[0] * alpha_dot
        wy = self.axis[1] * alpha_dot
        wz = self.axis[2] * alpha_dot

        # Publish desired pose
        now = rospy.Time.now()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = q_des[0]
        pose_msg.pose.orientation.y = q_des[1]
        pose_msg.pose.orientation.z = q_des[2]
        pose_msg.pose.orientation.w = q_des[3]

        # Publish desired twist
        twist_msg = TwistStamped()
        twist_msg.header = pose_msg.header
        twist_msg.twist.linear.x = vx
        twist_msg.twist.linear.y = vy
        twist_msg.twist.linear.z = vz
        twist_msg.twist.angular.x = wx
        twist_msg.twist.angular.y = wy
        twist_msg.twist.angular.z = wz

        self.pub_pose.publish(pose_msg)
        self.pub_twist.publish(twist_msg)
        #self.pub_time.publish(self.T)
        self.pub_time.publish(max(0.0, self.T - t))


        # Update last desired (so new targets replan smoothly)
        self.last_des_p = (x, y, z)
        self.last_des_q = q_des

        # Stop when finished
        if t >= self.T:
            self.active = False

def main():
    rospy.init_node("cartesian_trapezoid_planner")
    node = CartesianTrapezoidPlanner()
    rate = rospy.Rate(node.publish_rate)

    while not rospy.is_shutdown():
        node.step()
        rate.sleep()

if __name__ == "__main__":
    main()
