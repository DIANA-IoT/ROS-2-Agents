# Project Title: ROS2_AGENTS
# File: launch/robot_controller_launch.py
# Author: José-Borja Castillo-Sánchez, DIANA Group UMA
# Date: 2025
# (c) Copyright by Universidad de Málaga
# License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration, LocalSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnShutdown
from launch.events.process import ShutdownProcess
from launch.conditions import LaunchConfigurationEquals, IfCondition

def generate_launch_description():
    localization = LaunchConfiguration('localization')
    log_level = LaunchConfiguration('log_level')
    namespace = LaunchConfiguration('namespace')
    
    config = LaunchConfiguration(
        'config',
        default=os.path.join(
            get_package_share_directory('ros2_agents'),
            'config',
            'parameters_robot1.yaml'))
                                        
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
        
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='',
        description='Localization mode: AMCL, GroundTruth or odometry'
    )

    localization_amcl = LaunchConfiguration('localization_amcl', 
                                            default=os.path.join(get_package_share_directory('ros2_agents'), 'config', 'localization_amcl.yaml'))
    localization_gt = LaunchConfiguration('localization_gt', 
                                            default=os.path.join(get_package_share_directory('ros2_agents'), 'config', 'localization_gt.yaml'))
    localization_odom = LaunchConfiguration('localization_odom', 
                                            default=os.path.join(get_package_share_directory('ros2_agents'), 'config', 'localization_odometry.yaml'))

    robot_controller_node_amcl = Node(
        condition=LaunchConfigurationEquals('localization', 'AMCL'),
        package='ros2_agents',
        executable='robot_controller',
        namespace=namespace,
        arguments=['--ros-args', '--log-level', log_level, '--disable-rosout-logs'],
        # remappings=[
            # (f'/{namespace}/',f'~/{namespace}/')
        # ],
        parameters=[config, localization_amcl]
        )
    
    robot_controller_node_gt = Node(
        condition=LaunchConfigurationEquals('localization', 'GroundTruth'),
        package='ros2_agents',
        executable='robot_controller',
        namespace=namespace,
        arguments=['--ros-args', '--log-level', log_level, '--disable-rosout-logs'],
        # remappings=[
            # (f'/{namespace}/',f'~/{namespace}/')
        # ],
        parameters=[config, localization_gt]
        )
    
    robot_controller_node_odom = Node(
        condition=LaunchConfigurationEquals('localization', 'Odometry'),
        package='ros2_agents',
        executable='robot_controller',
        namespace=namespace,
        arguments=['--ros-args', '--log-level', log_level, '--disable-rosout-logs'],
        # remappings=[
            # (f'/{namespace}/',f'~/{namespace}/')
        # ],
        parameters=[config, localization_odom]
        )
                    
    return LaunchDescription([
    	declare_namespace_cmd,
    	declare_log_level_cmd,
        declare_localization_cmd,
    	robot_controller_node_amcl,
        robot_controller_node_gt,
        robot_controller_node_odom,
    	RegisterEventHandler(
    		OnShutdown(
    			on_shutdown=[ 
    				LogInfo(
                    msg=['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
    			)]
    		),
        )]
)