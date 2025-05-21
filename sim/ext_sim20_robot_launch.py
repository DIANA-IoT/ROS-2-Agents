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

def generate_launch_description():
    mwsn_dir = get_package_share_directory('ros2_agents')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    current_dir = get_package_share_directory('ros2_agents')

    world = LaunchConfiguration('world')
    map = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Full path to Gazebo world file'
    )

    declare_map_file_cmd = DeclareLaunchArgument(
        'map',
        default_value= '',
        description='Full path to map yaml file to load'
    )

    declare_nav_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to nav2 params file'
    )

    start_gazebo_cmd = ExecuteProcess(
        cmd = ['gazebo', '-s', 'libgazebo_ros_init.so',
                                     '-s', 'libgazebo_ros_factory.so', world]
    )

    robots = [
        {'name': 'robot1', 'x_pose': 1.0000, 'y_pose': 0.0000,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0000},
        {'name': 'robot2', 'x_pose': 0.9511, 'y_pose': 0.3090,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.3142},
        {'name': 'robot3', 'x_pose': 0.8090, 'y_pose': 0.5878,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.6283},
        {'name': 'robot4', 'x_pose': 0.5878, 'y_pose': 0.8090,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.9425},
        {'name': 'robot5', 'x_pose': 0.3090, 'y_pose': 0.9511,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 1.2566},
        {'name': 'robot6', 'x_pose': 0.0000, 'y_pose': 1.0000,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 1.5708},
        {'name': 'robot7', 'x_pose': -0.3090, 'y_pose': 0.9511,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 1.8850},
        {'name': 'robot8', 'x_pose': -0.5878, 'y_pose': 0.8090,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 2.1991},
        {'name': 'robot9', 'x_pose': -0.8090, 'y_pose': 0.5878,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 2.5133},
        {'name': 'robot10', 'x_pose': -0.9511, 'y_pose': 0.3090,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 2.8274},
        {'name': 'robot11', 'x_pose': -1.0000, 'y_pose': 0.0000,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 3.1416},
        {'name': 'robot12', 'x_pose': -0.9511, 'y_pose': -0.3090,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 3.4558},
        {'name': 'robot13', 'x_pose': -0.8090, 'y_pose': -0.5878,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 3.7699},
        {'name': 'robot14', 'x_pose': -0.5878, 'y_pose': -0.8090,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 4.0841},
        {'name': 'robot15', 'x_pose': -0.3090, 'y_pose': -0.9511,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 4.3982},
        {'name': 'robot16', 'x_pose': -0.0000, 'y_pose': -1.0000,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 4.7124},
        {'name': 'robot17', 'x_pose': 0.3090, 'y_pose': -0.9511,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 5.0265},
        {'name': 'robot18', 'x_pose': 0.5878, 'y_pose': -0.8090,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 5.3407},
        {'name': 'robot19', 'x_pose': 0.8090, 'y_pose': -0.5878,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 5.6549},
        {'name': 'robot20', 'x_pose': 0.9511, 'y_pose': -0.3090,'z_pose': 0.0, 
            'roll': 0.0, 'pitch': 0.0, 'yaw': 5.9690},
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

    # old_period += 3.0 
    # for simulation_instance_cmd in instances:
    #     ld.add_action(TimerAction(period=old_period, actions=[simulation_instance_cmd]))
    #     old_period += 3.0
    
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
        old_period += 0.5
    
    return ld