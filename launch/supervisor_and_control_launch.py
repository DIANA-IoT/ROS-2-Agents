# Project Title: ROS2_AGENTS
# File: launch/supervisor_and_control_launch.py
# Author: José-Borja Castillo-Sánchez, DIANA Group UMA
# Date: 2025
# (c) Copyright by Universidad de Málaga
# License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import LoadComposableNodes
from launch.actions import TimerAction
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml
from datetime import datetime

def generate_launch_description():
    mwsn_dir = get_package_share_directory('ros2_agents')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    current_dir = get_package_share_directory('ros2_agents')

    map = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    use_logger = LaunchConfiguration('use_logger')
    bag_path = LaunchConfiguration('bag_path')
    use_nav2 = LaunchConfiguration('use_nav2')

    declare_map_file_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(mwsn_dir, 'maps', 'newsim20v3.yaml'),
        description='Full path to map yaml file to load'
    )

    declare_nav_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to nav2 params file'
    )

    declare_logger_cmd = DeclareLaunchArgument(
        'use_logger',
        default_value='False',
        description='Central node logging active: True or False'
    )

    declare_bag_path_cmd = DeclareLaunchArgument(
        'bag_path',
        default_value=os.path.join(os.path.expanduser( '~' ), 'stage_sim', '20_robot', datetime.now().strftime("%d_%m_%Y_%H:%M:%S")),
        description='Bag logging path if active'
    )

    declare_use_nav2_cmd = DeclareLaunchArgument(
        'use_nav2',
        default_value='False',
        description='Wether to use nav2 or not.'
    )

    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

    robots = [
        {'name': 'robot0', 'x_pose': -2.0000, 'y_pose': 0.0,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0000},
        {'name': 'robot1', 'x_pose': -2.0000, 'y_pose': 1.0,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0000},
        {'name': 'robot2', 'x_pose': -1.0000, 'y_pose': 0.0,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0000},
        {'name': 'robot3', 'x_pose': -1.0000, 'y_pose': 1.0,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0000},
        {'name': 'robot4', 'x_pose': 0.0, 'y_pose': 0.0,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0000},
        {'name': 'robot5', 'x_pose': 0.0, 'y_pose': 1.0,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0000},
        {'name': 'robot6', 'x_pose': 1.0000, 'y_pose': 0.0,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0000},
        {'name': 'robot7', 'x_pose': 1.0000, 'y_pose': 1.0,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0000},
        {'name': 'robot8', 'x_pose': 2.0000, 'y_pose': 0.0,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0000},
        {'name': 'robot9', 'x_pose': 2.0000, 'y_pose': 1.0,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0000},
        ]
    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')


    
    instances = []

    ld = LaunchDescription()
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_nav_file_cmd)
    ld.add_action(declare_logger_cmd)
    ld.add_action(declare_bag_path_cmd)
    ld.add_action(declare_use_nav2_cmd)
    
    old_period = 0.5
    for simulation_instance_cmd in instances:
        ld.add_action(TimerAction(period=old_period, actions=[simulation_instance_cmd]))
        old_period += 0.5
    
    ld.add_action(GroupAction(
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                parameters=[
                    {
                        'yaml_filename': map
                    }
                ]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                parameters=[
                    {
                        'autostart': True,
                        'node_names': ['map_server']
                    }
                ]
            ),
        ]
    )
    )

    ld.add_action(
        GroupAction(
            actions= [             
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name="map_to_odom_broadcaster",
                    arguments=[
                        "0",
                        "0",
                        "0",
                        "0",
                        "0",
                        "0",
                        "map",
                        "odom"
                    ]
                )])         
    )
    # 
    old_period += 1.0
    instances = []
    for robot in robots:
       instances.append(
           GroupAction(
               # condition=IfCondition(PythonExpression(['not ', use_nav2])),
               actions= [
                   Node(package='tf2_ros',
                       executable='static_transform_publisher',
                       name=f"{robot['name']}_tf_broadcaster",
                       arguments=[
                           #  str(robot['x_pose']), 
                           #  str(robot['y_pose']),
                           #  str(robot['z_pose']),
                           #  str(robot['roll']),
                           #  str(robot['pitch']),
                           #  str(robot['yaw']),
                           "0",
                           "0",
                           "0",
                           "0",
                           "0",
                           "0",
                           'odom',
                           f"{robot['name']}/odom"
                       ])
               ]
           )
       )
    for simulation_instance_cmd in instances:
       ld.add_action(TimerAction(period=old_period, actions=[simulation_instance_cmd]))
    old_period += 0.5


    instances = []
    for robot in robots:
        instances.append(
            Node(
                package='ros2_agents',
                executable='pub_initial_pose.py',
                namespace=robot['name'],
                parameters=[
                    {'x_pose': robot['x_pose'],
                    'y_pose': robot['y_pose'],
                    'z_pose': robot['z_pose'],
                    'roll': robot['roll'],
                    'pitch': robot['pitch'],
                    'yaw': robot['yaw']}],
                 remappings=remappings
            ))
    
    old_period += 3.0 
    for simulation_instance_cmd in instances:
        ld.add_action(TimerAction(period=old_period, actions=[simulation_instance_cmd]))
        old_period += 1.0
    ld.add_action(
        Node(
            package='ros2_agents',
            executable='supervisor',
            arguments=[
                {
                    'log_level': 'info'
                }
            ],
            parameters=[
                {
                    'use_logger' : use_logger,
                    'bag_path': bag_path,
                    'number_of_robots': 10,
                }
            ]
        )
    )
    
    return ld
