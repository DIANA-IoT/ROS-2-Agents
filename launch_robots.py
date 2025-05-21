# Project Title: ROS2_AGENTS
# File: launch_robots.py
# Author: José-Borja Castillo-Sánchez, DIANA Group UMA
# Date: 2025
# (c) Copyright by Universidad de Málaga
# License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

### Script to command a set of robots. Intended to be used within a central point.
# The number of robots and the roles they take can be set by the variables 'hosts' and 'package_bin_tuple'.
import os, time, threading, subprocess
from concurrent.futures import ThreadPoolExecutor

## Global variables
## Hosts array that will execute the SSH commands
hosts = []
## Path where the SSH key is stored.
private_key_path = ''

### Function to run commands over SSH (background)
def run_command_on_background(host, username, private_key_path, command, executor_index):
    try:
        # Construct the SSH command for the single command
        ssh_command = f"""ssh -i {private_key_path} {username}@{host} -f 'nohup {command} > /dev/null 2>&1 &'"""

        # Execute the SSH command using subprocess
        subprocess.run(ssh_command, shell=True, check=False, text=True)

        # Print the command information
        t = time.localtime()
        current_time = time.strftime("%H:%M:%S", t)
        print(f"At: {current_time}, Command '{command}' for executor index {executor_index} on {host} started in the background")

    except subprocess.CalledProcessError as e:
        print(f"Error executing command '{command}' on {host} from executor index {executor_index}: {e}")

### Function to run commands over SSH
def run_command(host, username, private_key_path, command, executor_index):
    try:
        # Construct the SSH command for the single command
        ssh_command = f"""ssh -i {private_key_path} {username}@{host} '{command}'"""

        # Execute the SSH command using subprocess
        subprocess.run(ssh_command, shell=True, check=False)

        # Print the command information
        t = time.localtime()
        current_time = time.strftime("%H:%M:%S", t)
        print(f"At: {current_time}, Command '{command}' for executor index {executor_index} on {host} executed")

    except subprocess.CalledProcessError as e:
        print(f"Error executing command '{command}' on {host} from executor index {executor_index}: {e}")

### Function that performs the neccessary substitutions and invokes the SSH command to run ROS 2 and auxiliary code.
#    @param host: host's IP address
#    @param username: username to login
#    @param private_key_path: central point SSH key location
#    @param package_bin_tuple: a Python tuple that contains a tuple string with [ROS2_PKG, ROS2_EXEC].
#    @param host_index: index used to identify both the package 
#    @param command_barrier: rendez-vous synchronization mechanism.
#    @param do_log: boolean value to enable (default) or disable logging.
###
def run_commands_on_remote(host, username, private_key_path, package_bin_tuple, host_index, command_barrier, do_log):

    try:
        # Hard-coded configuration file path. Improve this
        config_filepath = "/home/ubuntu/ros2_ws/ws/src/ros2_agents/config"
        config_filepath += f"/parameters_robot{host_index}.yaml"
        
        package = package_bin_tuple[host_index][0]
        executable = package_bin_tuple[host_index][1]
        # Create new variable for the run command
        ros2_command = ""
        ros2_command += f"ros2 launch {package} {executable}" 
        ros2_command += f" namespace:=/robot{host_index}"
        ros2_command += f" config:={config_filepath} localization:='Odometry'"
                                #     launch_arguments={
                                # 'namespace': 'controller0',
                                # 'log_level': 'info',
                                # 'config': os.path.join(mwsn_dir, 'config', f"parameters_controller0.yaml"),
                                # 'localization': 'Odometry',
        # ros2_command += cmd + f" -p file_name:=/home/ubuntu/dds_logs/robot{host_index}_run{cmd_index}.csv" 
        # ros2_command += f" -r __node:=node{host_index} > /dev/null 2>&1"

        commands = [
            "ros2 launch turtlebot3_bringup robot.launch.py",
            ros2_command,
            ]

        for index, command in enumerate(commands):
            if index == 0:
                run_command_on_background(host, username, private_key_path, command, host_index)
                time.sleep(1)
            else:
                run_command(host, username, private_key_path, command, host_index)
                time.sleep(1)

            command_barrier.wait()

    except Exception as e:
        print(f"Error connecting to {host}: {e}")

### Runner function for each system thread, it allows a distinction based on the index
#    @param index: thread index and future SSH-running-machine index.
#    @param command_barrier: Rendez-vous IPC mechanism
###
def worker_wrapper(index, command_barrier):
    global private_key_path, package_bin_tuple, hosts
    run_commands_on_remote(hosts[index], 'ubuntu', private_key_path, package_bin_tuple, index, command_barrier, False)


if __name__ == "__main__":
    
    node_names = []

    # global hosts, private_key_path, config, config2, package_bin_tuple

    hosts = [
        '192.168.1.102',
        '192.168.1.103',
        '192.168.1.104', 
        '192.168.1.105',
        '192.168.1.106',
        '192.168.1.107',
        '192.168.1.108',
        '192.168.1.109', 
        '192.168.1.110',
        '192.168.1.111'
        ]

    package_bin_tuple = [
        ("ros2_agents", "robot_controller_launch.py"),
        ("ros2_agents", "robot_controller_launch.py"),
        ("ros2_agents", "robot_controller_launch.py"),
        ("ros2_agents", "robot_controller_launch.py"),
        ("ros2_agents", "robot_controller_launch.py"),
        ("ros2_agents", "robot_controller_launch.py"),
        ("ros2_agents", "robot_controller_launch.py"),
        ("ros2_agents", "robot_controller_launch.py"),
        ("ros2_agents", "robot_controller_launch.py"),
        ("ros2_agents", "robot_controller_launch.py"),
        ]
    
    n_threads = len(hosts)

    command_barrier = threading.Barrier(n_threads)
    # Review this
    private_key_path = os.path.join(os.path.expanduser('~'), '.ssh/id_ed25519')
    with ThreadPoolExecutor(max_workers=n_threads) as executor:
        for i in range(n_threads):
            fut = executor.submit(worker_wrapper, i, command_barrier)
        # Wait for all threads to finish
        executor.shutdown()
5