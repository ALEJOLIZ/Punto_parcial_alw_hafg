#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
import xacro

pkg_folder = 'trayectoria_parcial'
robot_file = 'ms2rp_alw_hafg.urdf.xacro'
rviz_file  = 'rviz_config.rviz'

def generate_launch_description():
    pkg_path = str(get_package_share_path(pkg_folder))
    default_model_path = os.path.join(pkg_path, 'model', robot_file)
    default_rviz_config_path = os.path.join(pkg_path, 'model', rviz_file)

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )

    # robot_description desde xacro (igual que tu ejemplo)
    robot_description_config = xacro.process_file(default_model_path)
    params = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # === NODOS (solo cambio de nombres a los reales) ===

    dxf_exporter_node = Node(
        package='trayectoria_parcial',
        executable='dxf_exporter_node',
        name='dxf_exporter_node'
    )

    # Si en tu ejemplo estaba comentado, lo dejo igual:
    traj_plan_node = Node(
        package='trayectoria_parcial',
        executable='traj_plan_node',
        name='traj_plan_node'
    )

    traj_guide_node = Node(
        package='trayectoria_parcial',
        executable='traj_guide_node',
        name='traj_guide_node'
    )

    ik_node = Node(
        package='trayectoria_parcial',
        executable='ik_node',
        name='ik_node'
    )

    dk_node = Node(
        package='trayectoria_parcial',
        executable='dk_node',
        name='dk_node'
    )

    scara_state_publisher_node = Node(
        package='trayectoria_parcial',
        executable='scara_state_publisher',
        name='scara_state_publisher'
    )

    # goal_translator_node = Node(
    #     package='trayectoria_parcial',
    #     executable='goal_translator',
    #     name='goal_translator'
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path]
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        dxf_exporter_node,
        dk_node,
        # traj_plan_node,        
        traj_guide_node,
        ik_node,
        scara_state_publisher_node,
        # goal_translator_node,  # <- opcional
        rviz_node,
    ])
