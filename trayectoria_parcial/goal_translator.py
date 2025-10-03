import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose, Twist, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class GoalTranslator(Node): 
    def __init__(self):
        super().__init__("goal_translator")

        self.x_des = 0.0
        self.y_des = 0.0
        self.z_des = 0.0
        
        # Publisher
        self.end_effector_pub = self.create_publisher(Twist, 'desired_pose', 10)
       
        # QoS para la suscripción
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber
        self.subscription = self.create_subscription(PoseStamped, 'goal_pose', self._on_goal_pose, qos)

        # Parámetro de escala
        self.declare_parameter("scale", 1000.0)
        self.scale = self.get_parameter("scale").get_parameter_value().double_value

    def _on_goal_pose(self, msg: PoseStamped):
        # Guardar valores
        self.x_des = msg.pose.position.x
        self.y_des = msg.pose.position.y
        self.z_des = msg.pose.position.z

        self.get_logger().info(
            f"Received (x,y,z): ({self.x_des:.2f}, {self.y_des:.2f}, {self.z_des:.2f})"
        )

        # Publicar ya escalado
        self.publish_scaled_pose()

    def publish_scaled_pose(self):
        # Escalar
        x_des_s = self.x_des * self.scale
        y_des_s = -self.y_des * self.scale
        z_des_s = self.z_des * self.scale
        
        self.get_logger().info(
            f"Scaled (x,y,z): ({x_des_s:.2f}, {y_des_s:.2f}, {z_des_s:.2f})"
        )
        
        # Publicar en Twist
        point_msg = Twist()
        point_msg.linear.x = x_des_s
        point_msg.linear.y = y_des_s
        point_msg.linear.z = z_des_s
        self.end_effector_pub.publish(point_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoalTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()