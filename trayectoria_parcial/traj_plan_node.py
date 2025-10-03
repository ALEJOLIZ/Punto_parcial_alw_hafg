#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv
import numpy as np
import os


class TrajPlanNode(Node):
    def __init__(self):
        super().__init__("traj_plan_node")

        self.get_logger().info("Trajectory Planner Node initialized. Waiting for DXF path...")

        self.waypoints = []
        with open('/home/alejo/ros2_trayectoria/src/trayectoria_parcial/dxf/dxf_waypoints_v5.csv', 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                x = float(row['x'])
                y = float(row['y'])
                z = float(row['z'])
                entity_type = row['entity_type']
                entity_id = int(row['entity_id'])
                self.waypoints.append((x, y, z, entity_type, entity_id))

        self.get_logger().info(f"Waypoints cargados: {len(self.waypoints)}")

        self.declare_parameter("interpolation_type", "cubic")  # choices: linear, cubic, quintic
        self.interpolation_type = self.get_parameter("interpolation_type").get_parameter_value().string_value
        self.get_logger().info(f"Usando interpolación: {self.interpolation_type}")


        self.trajectory_pub = self.create_publisher(Path, "planned_trajectory", 10)

        self.final_pts = self.final_points(self.waypoints)
        self.get_logger().info(f"Puntos interpolados generados: {len(self.final_pts)}")
        self.export_interpolated_to_csv(self.final_pts)


    def linear_interpolation(self, q0, qf, T, n_points=50):
        t = np.linspace(0, T, n_points)
        q = q0 + (qf - q0) * (t / T)
        return q

    def cubic_interpolation(self, q0, qf, T, v0=0.0, vf=0.0, n_points=50):
        """Interpolación cúbica con velocidades inicial y final"""
        t = np.linspace(0, T, n_points)
        a0 = q0
        a1 = v0
        a2 = (3 * (qf - q0) / T**2) - (2 * v0 + vf) / T
        a3 = (2 * (q0 - qf) / T**3) + (v0 + vf) / T**2
        q = a0 + a1 * t + a2 * t**2 + a3 * t**3
        return q
    def quintic_interpolation(self, q0, qf, T, v0=0.0, vf=0.0, a0=0.0, af=0.0, n_points=50):
        t = np.linspace(0, T, n_points)
        a0_c = q0
        a1_c = v0
        a2_c = a0 / 2
        a3_c = (20 * (qf - q0) - (8 * vf + 12 * v0) * T - (3 * a0 - af) * T**2) / (2 * T**3)
        a4_c = (30 * (q0 - qf) + (14 * vf + 16 * v0) * T + (3 * a0 - 2 * af) * T**2) / (2 * T**4)
        a5_c = (12 * (qf - q0) - (6 * vf + 6 * v0) * T - (a0 - af) * T**2) / (2 * T**5)
        q = a0_c + a1_c * t + a2_c * t**2 + a3_c * t**3 + a4_c * t**4 + a5_c * t**5
        return q


   

    def choose_num_points(self, entity_type):
        """Define la densidad de puntos según el tipo de entidad DXF"""
        if entity_type in ["LINE", "LWPOLYLINE", "POLYLINE"]:
            return 50    # Lines get to have more points
        elif entity_type in ["CIRCLE", "ARC"]:
            return 3     # Fewer points for curves
        elif entity_type == "SPLINE":
            return 15
        else:
            return 20

    def interpolate(self, q0, qf, T, N):
        """Usa la interpolación elegida desde parámetros"""
        if self.interpolation_type == "linear":
            return self.linear_interpolation(q0, qf, T, n_points=N)
        elif self.interpolation_type == "quintic":
            return self.quintic_interpolation(q0, qf, T, n_points=N)
        else:  #We use cubic by default
            return self.cubic_interpolation(q0, qf, T, n_points=N)


        
    def final_points(self, waypoints):
        """Interpola entre puntos, cierra cada entidad, añade movimiento vertical (prismático) y suaviza transiciones."""
        final_pts = []
        if not waypoints:
            return final_pts

        entity_start_point = waypoints[0]   # First point of the current entity
        current_entity_id = entity_start_point[4]

        for i in range(len(waypoints) - 1):
            p1 = waypoints[i]
            p2 = waypoints[i + 1]
            id1 = p1[4]
            id2 = p2[4]
            type1 = p1[3]

            if id1 == id2:
                N = self.choose_num_points(type1)
                T = 5.0
                x_vals = self.interpolate(p1[0], p2[0], T, N)
                y_vals = self.interpolate(p1[1], p2[1], T, N)
                z_vals = self.interpolate(p1[2], p2[2], T, N)
                for x, y, z in zip(x_vals, y_vals, z_vals):
                    final_pts.append((x, y, z, type1))

            else:
                # close the current entity if needed
                if (p1[0], p1[1], p1[2]) != (entity_start_point[0], entity_start_point[1], entity_start_point[2]):
                    N_close = self.choose_num_points(type1)
                    T = 5.0
                    x_vals = self.interpolate(p1[0], entity_start_point[0], T, N_close)
                    y_vals = self.interpolate(p1[1], entity_start_point[1], T, N_close)
                    z_vals = self.interpolate(p1[2], entity_start_point[2], T, N_close)
                    for x, y, z in zip(x_vals, y_vals, z_vals):
                        final_pts.append((x, y, z, type1))
                    self.get_logger().info(f" Cierre de entidad {id1}: +{N_close} pts")

                # Slow prismatic lift
                if final_pts:
                    last_x, last_y, last_z = final_pts[-1][0], final_pts[-1][1], final_pts[-1][2]
                    N_up, T_up = 20, 2.0
                    z_vals = self.interpolate(0.0, 0.072, T_up, N_up)
                    for z in z_vals:
                        final_pts.append((last_x, last_y, z, "PRISMATIC_UP"))
                    self.get_logger().info(f" Subida prismática: +{N_up} pts")

                # Z Movement (Keeping z=0.072)
                next_start = p2
                N_trans, T_trans = 30, 3.0
                x_vals = self.interpolate(last_x, next_start[0], T_trans, N_trans)
                y_vals = self.interpolate(last_y, next_start[1], T_trans, N_trans)
                z_vals = [0.072] * N_trans
                for x, y, z in zip(x_vals, y_vals, z_vals):
                    final_pts.append((x, y, z, "TRANSITION"))
                self.get_logger().info(f" Transición XY: +{N_trans} pts")

                N_down, T_down = 20, 2.0
                z_vals = self.interpolate(0.072, 0.0, T_down, N_down)
                for z in z_vals:
                    final_pts.append((next_start[0], next_start[1], z, "PRISMATIC_DOWN"))
                self.get_logger().info(f" Bajada prismática: +{N_down} pts")

                entity_start_point = p2
                current_entity_id = id2

        # Final entity closure if needed
        last_entity_id = waypoints[-1][4]
        first_point_last_entity = next((p for p in waypoints if p[4] == last_entity_id), None)
        if first_point_last_entity is not None and final_pts:
            last_added = final_pts[-1]
            if (last_added[0], last_added[1], last_added[2]) != (first_point_last_entity[0], first_point_last_entity[1], first_point_last_entity[2]):
                N_close = self.choose_num_points(first_point_last_entity[3])
                T = 5.0
                x_vals = self.interpolate(last_added[0], first_point_last_entity[0], T, N_close)
                y_vals = self.interpolate(last_added[1], first_point_last_entity[1], T, N_close)
                z_vals = self.interpolate(last_added[2], first_point_last_entity[2], T, N_close)
                for x, y, z in zip(x_vals, y_vals, z_vals):
                    final_pts.append((x, y, z, first_point_last_entity[3]))
                self.get_logger().info(f" Cierre final de entidad {last_entity_id}: +{N_close} pts")

        return final_pts
    

        

    def trajectory_planner(self, points):
        
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for p in points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = p[2]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.trajectory_pub.publish(path_msg)
        self.get_logger().info(f"Trayectoria publicada con {len(points)} puntos")
    # ------------------ Debug ------------------ #
    def export_interpolated_to_csv(self, points):
        output_path = "/home/alejo/ros2_trayectoria/src/trayectoria_parcial/dxf/trajectory.csv"
        try:
            with open(output_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['x', 'y', 'z', 'entity_type'])
                for p in points:
                    # Asegurar que cada punto tenga 4 valores
                    if len(p) == 4:
                        writer.writerow([p[0], p[1], p[2], p[3]])
                    else:
                        self.get_logger().warn(f" Punto inválido: {p}")
            self.get_logger().info(f" Puntos interpolados exportados a: {output_path}")
        except Exception as e:
            self.get_logger().error(f" Error al exportar CSV: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajPlanNode()
    node.trajectory_planner(node.final_pts) 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()