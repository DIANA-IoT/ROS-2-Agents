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
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='0.00'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    
    robot_ns_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot3_burger',
        description='robot name'
    )
    robot_sdf_arg = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(get_package_share_directory(
        'turtlebot3_gazebo'), 'models', 'turtlebot3_burger', 'model.sdf'),
        #default_value=os.path.join(nav2_bringup_dir, 'worlds', 'waffle.model'),
        # default_value=os.path.join(nav2_bringup_dir, 'worlds', 'waffle.model'),
        description='Full path to robot sdf file to spawn the robot in gazebo'
    )

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    urdf = os.path.join(get_package_share_directory(
        'turtlebot3_gazebo'), 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    declare_robot_spawn_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', robot_sdf,
            '-robot_namespace', namespace,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']
        ]
    )
    declare_robot_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': robot_description}],
        remappings=remappings
    )

    ld = LaunchDescription()
    ld.add_action(robot_ns_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(robot_sdf_arg)
    ld.add_action(declare_robot_spawn_cmd)
    ld.add_action(declare_robot_publisher_cmd)
    return ld