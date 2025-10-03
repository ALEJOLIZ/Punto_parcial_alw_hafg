#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

pkg_folder = 'trayectoria_parcial'
rviz_file  = 'rviz_config.rviz'

def generate_launch_description():
    pkg_path = str(get_package_share_path(pkg_folder))
    default_rviz_config_path = os.path.join(pkg_path, 'model', rviz_file)

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        rviz_arg,
        rviz_node,
    ])
