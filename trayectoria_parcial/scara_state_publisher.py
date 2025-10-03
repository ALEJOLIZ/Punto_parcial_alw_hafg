#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState  # referencia

class SCARAStatePublisher(Node):  # <--- MISMO NOMBRE
    def __init__(self):
        super().__init__("scara_state_publisher")  # <--- MISMO NODO

        # ---- Nombres de joints (defaults del Xacro) ----
        self.declare_parameter('arm_1', 'joint1')
        self.declare_parameter('arm_2', 'joint2')
        self.declare_parameter('prismatic', 'joint_tool')

        # ---- Geometría para calcular stroke como en el Xacro ----
        self.declare_parameter('blen', 0.10)         # altura base_cyl
        self.declare_parameter('tool_offset', 0.02)  # offset +Z del prismatic
        self.declare_parameter('tool_len', 0.11)     # longitud del cilindro vertical

        blen = float(self.get_parameter('blen').value)
        tool_offset = float(self.get_parameter('tool_offset').value)
        tool_len = float(self.get_parameter('tool_len').value)
        tool_stroke_default = max(0.0, blen + tool_offset - tool_len)

        # ---- Límites (coherentes con tu Xacro) ----
        self.declare_parameter('upper_limit_angular_1',  math.pi/2)    # +90°
        self.declare_parameter('lower_limit_angular_1', -math.pi/2)    # -90°
        self.declare_parameter('upper_limit_angular_2',  2*math.pi/3)  # +120°
        self.declare_parameter('lower_limit_angular_2', -2*math.pi/3)  # -120°

        # Prismático (eje [0 0 -1]): variable s4 ∈ [0, stroke]
        self.declare_parameter('upper_limit_linear', tool_stroke_default)
        self.declare_parameter('lower_limit_linear', 0.0)

        # ---- Frecuencia de publicación ----
        self.declare_parameter('publish_rate', 50.0)  # Hz

        # ---- Lee parámetros ----
        self.joint_1 = self.get_parameter('arm_1').get_parameter_value().string_value
        self.joint_2 = self.get_parameter('arm_2').get_parameter_value().string_value
        self.joint_3 = self.get_parameter('prismatic').get_parameter_value().string_value

        self.up_l_a_1  = float(self.get_parameter('upper_limit_angular_1').value)
        self.low_l_a_1 = float(self.get_parameter('lower_limit_angular_1').value)
        self.up_l_a_2  = float(self.get_parameter('upper_limit_angular_2').value)
        self.low_l_a_2 = float(self.get_parameter('lower_limit_angular_2').value)
        self.up_l_l    = float(self.get_parameter('upper_limit_linear').value)
        self.low_l_l   = float(self.get_parameter('lower_limit_linear').value)

        self.rate_hz = float(self.get_parameter('publish_rate').value)

        # ---- Estados de joints (cmd) ----
        self.cmd_th1 = 0.0
        self.cmd_th2 = 0.0
        self.cmd_linear = 0.0  # s4 en metros

        # ---- Pub/Sub ----
        qos = QoSProfile(depth=10)
        self.create_subscription(Twist, 'scara_conf', self._on_cmd_vel, qos)
        self.pub_js = self.create_publisher(JointState, '/joint_states', qos)

        # ---- Timer ----
        self.last_time = None
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self._on_timer)

        self.get_logger().info(
            f"Joints: {self.joint_1}, {self.joint_2}, {self.joint_3} | rate={self.rate_hz} Hz | "
            f"limits: th1[{self.low_l_a_1:.3f},{self.up_l_a_1:.3f}] rad, "
            f"th2[{self.low_l_a_2:.3f},{self.up_l_a_2:.3f}] rad, "
            f"s4[{self.low_l_l:.4f},{self.up_l_l:.4f}] m"
        )

    def _on_cmd_vel(self, msg: Twist):
        # MISMO MAPE0: x=theta1 (rad), y=theta2 (rad), z=s4 (m)
        self.cmd_th1 = float(msg.linear.x)
        self.cmd_th2 = float(msg.linear.y)
        self.cmd_linear = float(msg.linear.z)

    def _on_timer(self):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        th1_des = self.cmd_th1
        th2_des = self.cmd_th2
        s4_des  = self.cmd_linear

        # Saturación según URDF
        if s4_des >= self.up_l_l:   s4_des = self.up_l_l
        if s4_des <= self.low_l_l:  s4_des = self.low_l_l

        if th1_des >= self.up_l_a_1:   th1_des = self.up_l_a_1
        if th1_des <= self.low_l_a_1:  th1_des = self.low_l_a_1
        if th2_des >= self.up_l_a_2:   th2_des = self.up_l_a_2
        if th2_des <= self.low_l_a_2:  th2_des = self.low_l_a_2

        # Publica joint_states
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [self.joint_1, self.joint_2, self.joint_3]
        js.position = [th1_des, th2_des, s4_des]
        js.velocity = [0.0, 0.0, 0.0]
        self.pub_js.publish(js)

def main():
    rclpy.init()
    node = SCARAStatePublisher()  # <--- MISMO
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
