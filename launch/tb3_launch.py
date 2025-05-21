# Project Title: ROS2_AGENTS
# File: launch/tb3_launch.py
# Author: José-Borja Castillo-Sánchez, DIANA Group UMA
# Date: 2025
# (c) Copyright by Universidad de Málaga
# License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, LocalSubstitution, NotSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnShutdown
from launch.events.process import ShutdownProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    config = LaunchConfiguration('config')
    namespace = LaunchConfiguration('namespace')
    profiling = LaunchConfiguration('profiling')
    
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch', 
                                                   'robot.launch.py'))
    )

    controller_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                                get_package_share_directory('ros2_agents'), 'launch', 
                                'robot_controller_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'log_level': 'info',
            'config': config,
            'localization': 'Odometry',
        }.items(),
        condition=IfCondition(NotSubstitution(profiling))
        )
    
    controller_profiled_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                                get_package_share_directory('ros2_agents'), 'launch', 
                                'profiled_robot_controller_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'log_level': 'info',
            'config': config,
            'localization': 'Odometry',
        }.items(),
        condition=IfCondition(profiling)
    )
    
    declare_config_cmd = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(
            get_package_share_directory('ros2_agents'),
            'config',
            'parameters_robot1.yaml'),
        description='Full path to Robot controller configuration file'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='controller0',
        description='Namespace to apply to the controller node'
    )

    declare_profiling_cmd = DeclareLaunchArgument(
        'profiling',
        default_value='False',
        description='Wether to profile Robot Controller node.'
    )

    return LaunchDescription([
        bringup_cmd,
        controller_cmd,
        controller_profiled_cmd,
        declare_config_cmd,
        declare_namespace_cmd,
        declare_profiling_cmd
    ])