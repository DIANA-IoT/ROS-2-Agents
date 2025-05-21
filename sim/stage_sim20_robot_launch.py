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

    world = LaunchConfiguration('world')
    map = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    use_logger = LaunchConfiguration('use_logger')
    bag_path = LaunchConfiguration('bag_path')
    use_nav2 = LaunchConfiguration('use_nav2')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(mwsn_dir, 'maps', 'newsim20v3.world'),
        description='Full path to simulation world file'
    )

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

    # remappings = [('/tf', 'tf'),
                # ('/tf_static', 'tf_static')]

    start_sim_cmd = Node(
        package='stage_ros2',
        executable='stage_ros2',
        name='stage',
        parameters=[{
            'world_file': world,
            'use_static_transfomations' : 'false',
        }],
    )

    robots = [
        {'name': 'robot0', },
        {'name': 'robot1', },
        {'name': 'robot2',},
        {'name': 'robot3',},
        {'name': 'robot4'},
        {'name': 'robot5'},
        {'name': 'robot6'},
        {'name': 'robot7'},
        {'name': 'robot8'},
        {'name': 'robot9'},
        {'name': 'robot10', },
        {'name': 'robot11', },
        {'name': 'robot12',},
        {'name': 'robot13',},
        {'name': 'robot14'},
        {'name': 'robot15'},
        {'name': 'robot16'},
        {'name': 'robot17'},
        {'name': 'robot18'},
        {'name': 'robot19'},
        ]
    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')


    
    instances = []

    ld = LaunchDescription()
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_nav_file_cmd)
    ld.add_action(declare_logger_cmd)
    ld.add_action(declare_bag_path_cmd)
    ld.add_action(declare_use_nav2_cmd)

    ld.add_action(start_sim_cmd)
    
    old_period = 0.5
    for simulation_instance_cmd in instances:
        ld.add_action(TimerAction(period=old_period, actions=[simulation_instance_cmd]))
        old_period += 0.5

# NAV2 Localization: 1 map server for each robot.
    # instances = []
    # for robot in robots: 
    #     instances.append(GroupAction(
    #         actions=[
    #         PushRosNamespace(robot['name']),
    #         IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(
    #             nav2_bringup_dir, 'launch', 'localization_launch.py')),
    #         launch_arguments={
    #             'namespace': robot['name'],
    #             'use_namespace': 'True',
    #             'map': map,
    #             'use_sim_time': 'True',
    #             'params_file': os.path.join(current_dir, 'nav2_params', f"nav2_params_{robot['name']}.yaml"),
    #             'autostart': 'True',
    #             'use_composition': 'False',
    #             'use_respawn': 'False'
    #         }.items())
    #             ])
    #         )

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
        param_substitutions = {
            'use_sim_time': 'True',
            'yaml_filename': map}
        configured_params = RewrittenYaml(
            source_file=os.path.join(current_dir, 'nav2_params', f"nav2_params_{robot['name']}.yaml"),
            root_key=robot['name'],
            param_rewrites=param_substitutions,
            convert_types=True)
        instances.append(GroupAction(
            condition=IfCondition(use_nav2),
            actions= [
                PushRosNamespace(robot['name']),
                Node(package='nav2_amcl',
                     executable='amcl',
                     name="amcl",
                     respawn=False,
                     parameters=[configured_params]),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name=f"{robot['name']}_lifecycle_manager_localization",
                    output='screen',
                    parameters=[{'use_sim_time': True},
                                {'autostart': True},
                                {'node_names': [ "amcl"] }])
            ]
        ))

    for simulation_instance_cmd in instances:
        ld.add_action(TimerAction(period=old_period, actions=[simulation_instance_cmd]))
    old_period += 2.0

    ld.add_action(
        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(os.path.join(
                                mwsn_dir, 'launch', 'robot_controller_launch.py')),
                            launch_arguments={
                                'namespace': 'controller0',
                                'log_level': 'info',
                                'config': os.path.join(mwsn_dir, 'config', f"parameters_controller0.yaml"),
                                'localization': 'GroundTruth',
                            }.items())
    )
    ld.add_action(
        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(os.path.join(
                                mwsn_dir, 'launch', 'robot_controller_launch.py')),
                            launch_arguments={
                                'namespace': 'controller1',
                                'log_level': 'info',
                                'config': os.path.join(mwsn_dir, 'config', f"parameters_controller1.yaml"),
                                'localization': 'GroundTruth',
                            }.items())
    )


    instances = []
    instances.append(
        Node(
            package='ros2_agents',
            executable='pub_initial_pose.py',
            parameters=[
                {'number_of_robots': 20,
                'pose_file': '/home/borja/ros2/ros2_ws/src/ros2_agents/sim/poses_20_robots.yaml',}],
        ))
        
            # GroupAction(
            # actions=[IncludeLaunchDescription(
            #                 PythonLaunchDescriptionSource(os.path.join(
            #                     mwsn_dir, 'launch', 'robot_controller_launch.py')),
            #                 launch_arguments={
            #                     'namespace': robot['name'],
            #                     'log_level': 'info',
            #                     'config': os.path.join(mwsn_dir, 'config', f"parameters_{robot['name']}.yaml"),
            #                     'localization': 'GroundTruth',
            #                 }.items()), 
            #                 ]
            # )
    
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
                    'number_of_robots': 20,
                    'use_sim_time': True,
                    'type': "Agents",
                }
            ]
        )
    )
    
    return ld
