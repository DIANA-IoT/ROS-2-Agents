#!/usr/bin/python3
# Project Title: ROS2_AGENTS
# File: src/pub_initial_pose.py
# Author: José-Borja Castillo-Sánchez, DIANA Group UMA
# Date: 2025
# (c) Copyright by Universidad de Málaga
# License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import tf2_ros
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
from ros2_mwsn_msgs.msg import Supervisor2Controller
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import ast

class PosePublication(Node):
    def __init__(self):
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=5
        )
        super().__init__('posepublication')
        self.declare_parameter('pose_file', Parameter.Type.STRING)
        self.declare_parameter('number_of_robots', Parameter.Type.INTEGER)
        self.subscriber_ = self.create_subscription(Supervisor2Controller, '/control_info', 
                                                    self.control_callback, 10)
        self.publishers_ = []
        self.n_robots = int(self.get_parameter('number_of_robots').value)
        for index in range(self.n_robots):
            topic = f"/robot{index}/initialpose"
            self.publishers_.append(self.create_publisher(PoseWithCovarianceStamped, 
                                                topic, qos_profile))
        path = str(self.get_parameter('pose_file').value)
        self.robot_poses = self.read_robot_data(path)
        self.robot_msgs = []
        self.poses_to_msgs()
        # del self.robot_poses        
        self.publish_msgs()
        
    def control_callback(self, msg):
        if msg.type == 0 and msg.data[0] == 0:
            self.publish_msgs()
            rclpy.shutdown()
            
    
    def poses_to_msgs(self):
        # Converts every pose stored in the dictionary to a ROS 2 message
        for key in self.robot_poses.keys():
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.pose.pose.position.x = self.robot_poses[key].get('x_pose')
            msg.pose.pose.position.y = self.robot_poses[key].get('y_pose')
            msg.pose.pose.position.z = self.robot_poses[key].get('z_pose')
            roll = self.robot_poses[key].get('roll')
            pitch = self.robot_poses[key].get('pitch')
            yaw = self.robot_poses[key].get('yaw')
            q_tuple = quaternion_from_euler(roll, pitch, yaw)
            q = Quaternion()
            q.w = q_tuple[3]
            q.x = q_tuple[0]
            q.y = q_tuple[1]
            q.z = q_tuple[2]
            msg.pose.pose.orientation = q
            self.robot_msgs.append(msg)
    
    def publish_msgs(self):
        self.get_logger().info('Sending initial poses')
        for k in range(self.n_robots):
            self.publishers_[k].publish(self.robot_msgs[k])
        
    
    def read_robot_data(self, file_path):
        with open(file_path, 'r') as file:
            # Read the file content and parse it using the abstract syntax tree
            file_content = file.read()
            # Safely evaluate the list of robots using `ast.literal_eval`
            robot_data = ast.literal_eval(file_content.split('=')[1].strip())
        
        # Convert the list of robots into a dictionary using robot name as the key
        robot_dict = {robot['name']: robot for robot in robot_data}
        
        return robot_dict


def main(args=None):
    rclpy.init(args=args)
    publisher_node = PosePublication()
    rclpy.spin(publisher_node)
    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
