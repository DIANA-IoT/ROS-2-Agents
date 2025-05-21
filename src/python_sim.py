#!/usr/bin/python3
# Project Title: ROS2_AGENTS
# File: src/python_sim.py
# Author: José-Borja Castillo-Sánchez, DIANA Group UMA
# Date: 2025
# (c) Copyright by Universidad de Málaga
# License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

import os, sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import LoadComposableNodes
from launch.actions import TimerAction
from launch_ros.actions import Node, PushRosNamespace

# ROS Subscriber
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Python
from threading import Thread, Event
import csv, time

# Please start reading bottom-up

# Global Variables
is_sim_ok = False
ev = Event() # IPC variable
ls = LaunchService()

# ROS Subscriber class 
# Subscribes to "/sim_status" topic. Which contains
# a string defining sim result: OK / Fail
class ROS_Subscriber(Node):
    def __init__(self):
        super().__init__('ROS_Subscriber')
        self.get_logger().info('Spawned sub')
        self.sub = self.create_subscription(String, 
                                            '/sim_status',
                                            self.listener_callback,
                                            2)
        self.sub
    def listener_callback(self, msg):
        self.get_logger().info('Received msg: "%s"' % msg.data)
        global ls
        global is_sim_ok
        # Simulation ends, kills the instance
        ls.shutdown()
        if "OK" in str(msg.data):
            is_sim_ok = True
            ev.set()
        elif "Fail" in str(msg.data):
            is_sim_ok = False
            ev.set()

class ROS_Subscriber_Thread():
    def __init__(self) -> None:
        rclpy.init(args=None)
        sc = ROS_Subscriber()
        rclpy.spin(sc)
        sc.destroy_node()
        rclpy.shutdown()

def launchProcess(ls, file, logfile):
    ld = LaunchDescription()
    if logfile:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(file),
            launch_arguments={
                'use_logger': 'True',
                'bag_path': logfile
            }.items()
        ))
    else:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(file)))
    ls.include_launch_description(ld)
    ls.run()

def main(args=sys.argv[1]):
    ROS_thread = Thread(target=ROS_Subscriber_Thread)
    ROS_thread.start()
    global ls, is_sim_ok
    # File by args
    file = args
    with open(file, mode='r') as csvfile:
        csv_line = csv.DictReader(csvfile)
        for line in csv_line:
            launch_file = line["launch_file"]
            reps = line["repetitions"]
            filep = line["path"]
            file = line["filename"]
            for rep in range(int(reps)):
                if not filep or not file:
                    logfile = ""
                else:
                    logfile = f"{filep}/{file}{rep}"
                ls = LaunchService()
                launchProcess(ls, launch_file, logfile)
                ev.wait()
                del ls
                if is_sim_ok == False:
                    os.close(csvfile)
                    sys.exit(-1)
                time.sleep(20)
    return 0

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("No text file fed. Please type text file path to load simulations.")
        sys.exit(-1)
    main(sys.argv[1])
