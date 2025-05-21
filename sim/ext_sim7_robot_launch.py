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
from datetime import datetime

def generate_launch_description():
    mwsn_dir = get_package_share_directory('ros2_agents')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    current_dir = get_package_share_directory('ros2_agents')

    world = LaunchConfiguration('world')
    map = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    use_logger = LaunchConfiguration('use_logger')
    use_battery_sim = LaunchConfiguration('use_battery_sim')
    bag_path = LaunchConfiguration('bag_path')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='/home/borja/ros2/mapas_mundos/gazebo/worlds/large_empty.world',
        description='Full path to Gazebo world file'
    )

    declare_map_file_cmd = DeclareLaunchArgument(
        'map',
        default_value= '/home/borja/ros2/mapas_mundos/large_empty/large_empty.yaml',
        description='Full path to map yaml file to load'
    )

    declare_nav_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to nav2 params file'
    )
    
    declare_logger_cmd = DeclareLaunchArgument(
        'use_logger',
        default_value='True',
        description='Central node logging active: True or False'
    )

    declare_battery_sim_cmd = DeclareLaunchArgument(
        'use_battery_sim',
        default_value='False',
        description='Central node robot battery simulator active: True or False'
    )

    declare_bag_path_cmd = DeclareLaunchArgument(
        'bag_path',
        default_value=os.path.join('/home/borja/ros2/bags', 'fc_sigma2_25_controlledSim', 'ext_7_robot', datetime.now().strftime("%d_%m_%Y_%H:%M:%S")),
        description='Bag logging path if active'
    )

    start_gazebo_cmd = ExecuteProcess(
        cmd = ['gazebo', '-s', 'libgazebo_ros_init.so',
                                     '-s', 'libgazebo_ros_factory.so', world]
    )

    robots = [
        {'name': 'robot1', 'x_pose': 0.8000, 'y_pose': 0.0000,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0000},
        {'name': 'robot2', 'x_pose': 0.4988, 'y_pose': 0.6255,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.8976},
        {'name': 'robot3', 'x_pose': -0.1780, 'y_pose': 0.7799,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 1.7952},
        {'name': 'robot4', 'x_pose': -0.7208, 'y_pose': 0.3471,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 2.6928},
        {'name': 'robot5', 'x_pose': -0.7208, 'y_pose': -0.3471,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 3.5904},
        {'name': 'robot6', 'x_pose': -0.1780, 'y_pose': -0.7799,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 4.4880},
        {'name': 'robot7', 'x_pose': 0.4988, 'y_pose': -0.6255,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 5.3856},
        ]
    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    

    urdf = os.path.join(get_package_share_directory(
        'turtlebot3_gazebo'), 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    robot_file = os.path.join(get_package_share_directory(
        'turtlebot3_gazebo'), 'models', 'turtlebot3_burger', 'model.sdf')
    
    instances = []

    ld = LaunchDescription()
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_nav_file_cmd)
    ld.add_action(declare_logger_cmd)
    ld.add_action(declare_battery_sim_cmd)
    ld.add_action(declare_bag_path_cmd)

    ld.add_action(start_gazebo_cmd)
    
    for robot in robots: 
        group = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(current_dir, 'launch', 'spawn_robot_and_state_publisher_launch.py')),
                launch_arguments={
                    'namespace': robot['name'],
                    'robot_name': robot['name'],
                    'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                    'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                    'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                    'roll': TextSubstitution(text=str(robot['roll'])),
                    'pitch': TextSubstitution(text=str(robot['pitch'])),
                    'yaw': TextSubstitution(text=str(robot['yaw']))
                }.items()
            ),
            LogInfo(msg=['Gazebo: Launching ', robot['name']])
        ])
        instances.append(group)
    
    old_period = 0.5
    for simulation_instance_cmd in instances:
        ld.add_action(TimerAction(period=old_period, actions=[simulation_instance_cmd]))
        old_period += 0.5

    instances = []
    for robot in robots: 
        instances.append(GroupAction(
            actions=[
            PushRosNamespace(robot['name']),
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav2_bringup_dir, 'launch', 'localization_launch.py')),
            launch_arguments={
                'namespace': robot['name'],
                'use_namespace': 'True',
                # 'slam': 'False',
                'map': map,
                'use_sim_time': 'True',
                'params_file': os.path.join(current_dir, 'nav2_params', f"nav2_params_{robot['name']}.yaml"),
                'autostart': 'True',
                'use_composition': 'False',
                # 'container_name' : 'localization_container',
                'use_respawn': 'False'
            }.items())
                ])
            )

    old_period += 3.0 
    for simulation_instance_cmd in instances:
        ld.add_action(TimerAction(period=old_period, actions=[simulation_instance_cmd]))
        old_period += 3.0
    
    instances = []
    for robot in robots:
        instances.append(GroupAction(
            actions=[IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(os.path.join(
                                mwsn_dir, 'launch', 'agent_launch.py')),
                            launch_arguments={
                                'namespace': robot['name'],
                                'log_level': 'info',
                                'config': os.path.join(mwsn_dir, 'config', f"parameters_{robot['name']}.yaml"),
                                'localization': 'GroundTruth',
                            }.items()), 
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
                            )]
            ))
    
    old_period += 3.0 
    for simulation_instance_cmd in instances:
        ld.add_action(TimerAction(period=old_period, actions=[simulation_instance_cmd]))
        old_period += 1.0
    
    ld.add_action(
        Node(
            package='ros2_agents',
            executable='central',
            arguments=[
                {
                    'log_level': 'debug'
                }
            ],
            parameters=[
                {
                    'use_logger' : use_logger,
                    'use_battery_sim' : use_battery_sim,
                    'bag_path': bag_path,
                    'number_of_robots': 7
                }
            ]
        )
    )

    return ld
