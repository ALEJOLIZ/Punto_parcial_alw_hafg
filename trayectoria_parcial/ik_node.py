#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import PointStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class IKNode(Node):
    def __init__(self):
        super().__init__("ik_node")
        
        # Publisher to send the calculated joint values
        self.joint_state_pub = self.create_publisher(Twist, 'scara_conf', 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.goal_pose_sub = self.create_subscription(
            PointStamped,
            'goal_pose_trajectory',
            self.goal_pose_callback,
            qos)

        # -------- Parameters (Geometric) coherent with the Xacro --------
        self.declare_parameter("scale", 100.0)
        self.scale = self.get_parameter("scale").get_parameter_value().double_value

        # Xacro: l1x=0.20, l2x=0.15, blen=0.10, tool_offset=0.02, tool_len=0.11
        self.declare_parameter("a_12", 20.0/self.scale)   # = l1x
        self.declare_parameter("a_23", 15.0/self.scale)   # = l2x
        self.declare_parameter("s_1",  10.0/self.scale)   # = blen
        self.declare_parameter("s_3",   2.0/self.scale)   # = tool_offset (+Z before prismatic)
        self.declare_parameter("tool_len", 11.0/self.scale)

        # tool_stroke = max(0, s_1 + s_3 - tool_len)   (same as Xacro logic)
        s_1 = self.get_parameter("s_1").get_parameter_value().double_value
        s_3 = self.get_parameter("s_3").get_parameter_value().double_value
        tool_len = self.get_parameter("tool_len").get_parameter_value().double_value
        tool_stroke_default = max(0.0, s_1 + s_3 - tool_len)
        self.declare_parameter("tool_stroke", tool_stroke_default)
        
        self.declare_parameter("goal_scale", 0.001)

    def goal_pose_callback(self, msg: PointStamped):
        """
        Process incoming goal pose and compute algebraic IK.
        """
        # Escala de entrada (mm->m si goal_scale=0.001)
        scale_in = self.get_parameter("goal_scale").get_parameter_value().double_value

        # Desired position (apply scale): metros
        x_d = msg.point.x * scale_in
        y_d = msg.point.y * scale_in
        z_d = msg.point.z * scale_in

        # Geometric parameters
        a_12 = self.get_parameter("a_12").get_parameter_value().double_value
        a_23 = self.get_parameter("a_23").get_parameter_value().double_value
        s_1  = self.get_parameter("s_1").get_parameter_value().double_value
        s_3  = self.get_parameter("s_3").get_parameter_value().double_value
        tool_len    = self.get_parameter("tool_len").get_parameter_value().double_value
        tool_stroke = self.get_parameter("tool_stroke").get_parameter_value().double_value

        # --- Planar 2R IK in XY ---
        rx = x_d
        ry = y_d
        L_sq = rx**2 + ry**2

        # Reachability check (outer circle)
        if L_sq > (a_12 + a_23)**2:
            self.get_logger().warn(
                "Goal XY position is out of reach! "
                f"L={math.sqrt(L_sq):.3f} > a12+a23={a_12 + a_23:.3f}"
            )
            return

        # theta_2 via cosine law (elbow-down branch)
        cos_theta2 = (L_sq - a_12**2 - a_23**2) / (2.0 * a_12 * a_23)
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
        try:
            theta_2 = math.acos(cos_theta2)
        except ValueError:
            self.get_logger().error("acos domain error for theta_2.")
            return

        # theta_1 via 2x2 algebra (same structure you used)
        k1 = a_12 + a_23 * math.cos(theta_2)
        k2 = a_23 * math.sin(theta_2)
        A = np.array([[k1, -k2],
                      [k2,  k1]])
        B = np.array([[rx],
                      [ry]])
        try:
            X = np.linalg.inv(A) @ B
        except np.linalg.LinAlgError:
            self.get_logger().error("Singular matrix when solving for theta_1.")
            return

        cos_theta1_num = float(X[0, 0])
        sin_theta1_num = float(X[1, 0])
        theta_1 = math.atan2(sin_theta1_num, cos_theta1_num)

        # --- Prismatic IK along -Z (URDF axis = [0, 0, -1]) ---
        # z_tip = s_1 + s_3 - s_4 - tool_len   =>   s_4 = (s_1 + s_3 - tool_len) - z_d
        s_4 = (s_1 + s_3 - tool_len) - z_d

        # Clamp to [0, tool_stroke] (URDF joint limits)
        s_4_clamped = max(0.0, min(s_4, tool_stroke))
        if abs(s_4_clamped - s_4) > 1e-9:
            z_min = (s_1 + s_3 - tool_len) - tool_stroke
            z_max = (s_1 + s_3 - tool_len)
            self.get_logger().warn(
                f"z_d={z_d:.3f} out of range [{z_min:.3f}, {z_max:.3f}]. "
                f"Clamping s_4 to {s_4_clamped:.4f} (limits [0, {tool_stroke:.4f}])."
            )
            s_4 = s_4_clamped

        # --- Publish the result (same message you use for DK) ---
        joint_cmd_msg = Twist()
        joint_cmd_msg.linear.x = theta_1
        joint_cmd_msg.linear.y = theta_2
        joint_cmd_msg.linear.z = s_4

        self.get_logger().info(
            f"IK -> theta_1={theta_1:.3f} rad, theta_2={theta_2:.3f} rad, s_4={s_4:.4f} m"
        )
        self.joint_state_pub.publish(joint_cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
