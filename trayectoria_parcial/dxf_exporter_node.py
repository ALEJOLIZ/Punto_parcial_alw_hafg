#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Pose
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from nav_msgs.msg import Path

import ezdxf
import pathlib 
import math
import csv 
import yaml 

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

class DXFParserNode(Node):
    def __init__(self):
        super().__init__('dxf_parser_node')

        # Dictionary to map entity types to integers
        self.ENTITY_TYPE_MAP = {
            "LINE": 1,
            "LWPOLYLINE": 2,
            "POLYLINE": 3,
            "ARC": 4,
            "CIRCLE": 5,
            "SPLINE": 6
        }

        # Parameters
        self.declare_parameter('dxf_file', '/home/alejo/ros2_trayectoria/src/trayectoria_parcial/dxf/Test_2.dxf')
        dxf_path = self.get_parameter('dxf_file').get_parameter_value().string_value
        self.get_logger().info(f"Reading DXF file: {dxf_path}")
        
        # Publisher
        self.path_pub = self.create_publisher(Path, 'dxf_path', 10)
        self.pc_pub = self.create_publisher(PointCloud, 'dxf_pointcloud', 10)
        
        # Parse and publish waypoints
        self.waypoints = self.parse_dxf(dxf_path)
        self.export_to_csv()
        self.publish_waypoints()
        self.publish_pointcloud()




    def parse_dxf(self, path):
        if not pathlib.Path(path).exists():
            self.get_logger().error(f"File does not exist: {path}")
            return []

        doc = ezdxf.readfile(path)
        msp = doc.modelspace()

        waypoints = []
        entity_id = 0
        
        self.get_logger().info("Found DXF entities:")
        for e in msp:
            self.get_logger().info(f"- Entity Type: {e.dxftype()}")
            
            entity_type = e.dxftype()
            
            if entity_type == 'LINE':
                waypoints.extend([
                    (e.dxf.start.x, e.dxf.start.y, 0.0, entity_type, entity_id),
                    (e.dxf.end.x, e.dxf.end.y, 0.0, entity_type, entity_id)
                ])

            elif entity_type == 'LWPOLYLINE':
                for x, y, *_ in e.get_points():
                    waypoints.append((x, y, 0.0, entity_type, entity_id))
            
            elif entity_type == 'POLYLINE':
                 for p in e.points():
                     waypoints.append((p.x, p.y, 0.0, entity_type, entity_id))
            
            elif entity_type == 'ARC':
                arc_points = self._approximate_arc(
                    e.dxf.center.x,
                    e.dxf.center.y,
                    e.dxf.radius,
                    e.dxf.start_angle,
                    e.dxf.end_angle
                )
                for point in arc_points:
                    waypoints.append((point.x, point.y, point.z, entity_type, entity_id))
                
            elif entity_type == 'CIRCLE':
                circle_points = self._approximate_arc(
                    e.dxf.center.x,
                    e.dxf.center.y,
                    e.dxf.radius,
                    0.0,
                    360.0
                )
                for point in circle_points:
                    waypoints.append((point.x, point.y, point.z, entity_type, entity_id))

            elif entity_type == 'SPLINE':
                for vec in e.flattening(0.5):
                    waypoints.append((vec.x, vec.y, 0.0, entity_type, entity_id))
            
            else:
                self.get_logger().warn(f"Entity type '{e.dxftype()}' not handled. Skipping.")
            
            entity_id += 1


        self.get_logger().info(f"Extracted {len(waypoints)} waypoints from DXF.")
        return waypoints

    def _approximate_arc(self, cx, cy, radius, start_angle_deg, end_angle_deg, num_points=30):
        start_angle = math.radians(start_angle_deg)
        end_angle = math.radians(end_angle_deg)
        if end_angle < start_angle:
            end_angle += 2 * math.pi

        arc_points = []
        for i in range(num_points + 1):
            angle = start_angle + i * (end_angle - start_angle) / num_points
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            arc_points.append(Point(x=x, y=y, z=0.0))
        return arc_points


    def publish_waypoints(self):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i, wp in enumerate(self.waypoints):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wp[0] / 100.0
            pose.pose.position.y = wp[1] / 100.0
            pose.pose.position.z = wp[2] / 100.0

            # Guardamos entity_id y type en orientation
            entity_type = wp[3]
            entity_id = wp[4]

            pose.pose.orientation.x = float(entity_id)  
            pose.pose.orientation.y = float(self.ENTITY_TYPE_MAP.get(entity_type, 0))  
            pose.pose.orientation.z = 0.0  
            pose.pose.orientation.w = 1.0  

            path_msg.poses.append(pose)

        self.get_logger().info(f"Publishing path with {len(path_msg.poses)} poses.")
        self.path_pub.publish(path_msg)


    def publish_pointcloud(self):
        pc = PointCloud()
        pc.header.frame_id = "map"
        pc.header.stamp = self.get_clock().now().to_msg()

        for wp in self.waypoints:
            pc.points.append(Point32(x=wp[0]/10.0, y=wp[1]/10.0, z=wp[2]/10.0))
            
        self.pc_pub.publish(pc)

    def export_to_csv(self, filename='/home/alejo/ros2_trayectoria/src/trayectoria_parcial/dxf/dxf_waypoints_v5.csv'):
        try:
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['x', 'y', 'z', 'entity_type', 'entity_id']) # Header with 'entity_id' column
                for point in self.waypoints:
                    writer.writerow([point[0], point[1], point[2], point[3], point[4]]) # NEW: Writing all columns
            self.get_logger().info(f"Waypoints exported to CSV at: {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DXFParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()