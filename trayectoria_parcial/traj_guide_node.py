#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped

class TrajGuideNode(Node):
    def __init__(self):
        super().__init__('traj_guide_node')
        self.sub = self.create_subscription(Path, 'planned_trajectory', self.cb, 10)
        self.pub = self.create_publisher(PointStamped, 'goal_pose_trajectory', 10)
        self.timer = None
        self.poses = []
        self.index = 0

    def cb(self, msg):
        self.poses = msg.poses
        self.index = 0
        if self.poses: 
            if not self.timer:
                self.timer = self.create_timer(0.1, self.publish_next)  
        else:
            self.get_logger().warn("Received empty trajectory")

    def publish_next(self):
        if self.index < len(self.poses):
            pose = self.poses[self.index]

            # Convert PoseStamped → PointStamped
            point_msg = PointStamped()
            point_msg.header = pose.header
            point_msg.point.x = pose.pose.position.x
            point_msg.point.y = pose.pose.position.y
            point_msg.point.z = pose.pose.position.z

            self.pub.publish(point_msg)
            self.get_logger().info(
                f" Sent goal {self.index}: "
                f"x={point_msg.point.x:.3f}, "
                f"y={point_msg.point.y:.3f}, "
                f"z={point_msg.point.z:.3f}"
            )
            self.index += 1
        else:
            self.get_logger().info(" Finished trajectory")
            self.timer.cancel()
            self.timer = None

def main():
    rclpy.init()
    node = TrajGuideNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()