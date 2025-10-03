#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import PointStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DKNode(Node): 
    def __init__(self):
        super().__init__("dk_node")
        
        # Publisher para visualizar la posición del efector final
        self.end_effector_pub = self.create_publisher(PointStamped, 'end_effector_pose', 10)

        # ---------- Parámetros (m), coherentes con tu Xacro ----------
        # Usamos scale=100 para escribir longitudes en "cm/100 = m"
        self.declare_parameter("scale", 100.0)
        self.scale = self.get_parameter("scale").get_parameter_value().double_value

        # l1x=0.20, l2x=0.15, blen=0.10, tool_offset=0.02, tool_len=0.11
        self.declare_parameter("a_12", 20.0/self.scale)       # = l1x
        self.declare_parameter("a_23", 15.0/self.scale)       # = l2x
        self.declare_parameter("alpha_23", math.pi)           # invierte Z (coherente con prismatico -Z)
        self.declare_parameter("s_1", 10.0/self.scale)        # = blen (altura a joint1)
        self.declare_parameter("s_3", 2.0/self.scale)         # = tool_offset (+Z antes del prismático)

        # Extra fijo para llegar a la punta (origen de base_tool está arriba del cilindro)
        self.declare_parameter("tool_len", 11.0/self.scale)   # longitud del cilindro vertical

        # Valores articulares
        self.theta_1 = 0.0
        self.theta_2 = 0.0
        self.s_4 = 0.0
        
        # QoS de la suscripción (igual que tu código)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber de comandos articulares
        self.subscription = self.create_subscription(Twist, '/scara_conf', self._on_cmd_vel, qos)

        # Timer para publicar la pose periódicamente
        self.timer = self.create_timer(0.1, self.timer_callback)

    def _on_cmd_vel(self, msg: Twist):
        # Actualiza los valores articulares desde el mensaje entrante
        self.theta_1 = msg.linear.x    # rad
        self.theta_2 = msg.linear.y    # rad
        self.s_4     = msg.linear.z    # m (extensión positiva)

    def timer_callback(self):
        # Lee parámetros en runtime (como en tu diseño)
        a_12      = self.get_parameter("a_12").get_parameter_value().double_value
        a_23      = self.get_parameter("a_23").get_parameter_value().double_value
        alpha_23  = self.get_parameter("alpha_23").get_parameter_value().double_value
        s_1       = self.get_parameter("s_1").get_parameter_value().double_value
        s_3       = self.get_parameter("s_3").get_parameter_value().double_value
        tool_len  = self.get_parameter("tool_len").get_parameter_value().double_value
        
        # ---------- Tabla DH [a, alpha, d, theta] ----------
        # Fila extra final [0,0,tool_len,0] para ir del origen del prismatic a la PUNTA
        DH = np.array([
            [0.0,   0.0,   s_1,        self.theta_1],  # base -> J1 (altura blen)
            [a_12,  0.0,   0.0,        self.theta_2],  # link1
            [a_23,  alpha_23, s_3,     0.0],           # link2 + offset +Z (alpha=pi invierte Z)
            [0.0,   0.0,   self.s_4,   0.0],           # prismático (d>0 baja en -Z mundo)
            [0.0,   0.0,   tool_len,   0.0],           # hasta la PUNTA de la herramienta
        ])
        
        # Matriz de transformación final
        T_final = self.transform_matrix(DH)
        
        # Extrae posición (x, y, z)
        end_effector_x = float(T_final[0, 3])
        end_effector_y = float(T_final[1, 3])
        end_effector_z = float(T_final[2, 3])
        
        self.get_logger().info(
            f"End-effector (x,y,z): ({end_effector_x:.3f}, {end_effector_y:.3f}, {end_effector_z:.3f})"
        )
        
        # Publica la posición como PointStamped en base_link (marco visual por defecto)
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = "base_link"
        point_msg.point.x = end_effector_x
        point_msg.point.y = end_effector_y
        point_msg.point.z = end_effector_z
        self.end_effector_pub.publish(point_msg)

    def transform_matrix(self, DH_matrix):
        T_ij = []
        for i in range(DH_matrix.shape[0]):
            a, alpha, d, theta = DH_matrix[i, :]
            # Mantengo EXACTA tu convención de T(a, alpha, d, theta)
            T = np.array([
                [math.cos(theta), -math.sin(theta),                 0.0,                 a],
                [math.sin(theta)*math.cos(alpha),  math.cos(theta)*math.cos(alpha), -math.sin(alpha),  -d*math.sin(alpha)],
                [math.sin(theta)*math.sin(alpha),  math.cos(theta)*math.sin(alpha),  math.cos(alpha),   d*math.cos(alpha)],
                [0.0,              0.0,                           0.0,                 1.0]
            ])
            T_ij.append(T)
            
        T_final = np.identity(4)
        for T in T_ij:
            T_final = T_final @ T
        return T_final

def main(args=None):
    rclpy.init(args=args)
    node = DKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
