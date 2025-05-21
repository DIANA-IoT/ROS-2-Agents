# Project Title: ROS2_AGENTS
# File: sim/convert_spider.py
# Author: José-Borja Castillo-Sánchez, DIANA Group UMA
# Date: 2025
# (c) Copyright by Universidad de Málaga
# License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

import sys
# Reads from a stage world file and outputs data in a Python dictionary-friendly way.
# Requires one argument: the stage world file to read.
# Optionally a second argument can be passed, which is the destination file.
# If this second argument is not provided, results are print on stdout.
def convert_spider_line(line):
    # Check if the line starts with "spider("
    if not line.startswith("spider("):
        return None  # Ignore lines that do not match
    # Step 1: Extract the robot name
    name_start = line.find('name "') + len('name "')
    name_end = line.find('"', name_start)
    name = line[name_start:name_end]

    # Step 2: Extract pose values
    pose_start = line.find('pose [') + len('pose [')
    pose_end = line.find(']', pose_start)
    pose = list(map(float, line[pose_start:pose_end].split()))

    robot_data = {
        'name': name,
        'x_pose': pose[0] / 1.0,
        'y_pose': pose[1] / 1.0,
        'z_pose': pose[2] / 1.0,
        'roll': 0.0,
        'pitch': 0.0,
        'yaw': pose[3]
    }

    return robot_data

def main():
# Check the number of arguments
    if len(sys.argv) < 2:
        print("Usage: python program.py <input_file> [output_file]")
        return

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None

    try:
        # Read the input file
        with open(input_file, 'r') as file:
            lines = file.readlines()

        # Convert each line and collect results, ignoring lines that aren't "spider"
        robots = [convert_spider_line(line.strip()) for line in lines if line.strip().startswith("spider(")]

        # Remove None entries that might have been returned for non-matching lines
        robots = [robot for robot in robots if robot is not None]

        if output_file:
            # Write to the output file if specified
            with open(output_file, 'w') as file:
                file.write("robots = [\n")
                for robot in robots:
                    file.write(f"    {repr(robot)},\n")
                file.write("]\n")
            print(f"Data successfully written to {output_file}")
        else:
            # Print to console
            print("robots = [")
            for robot in robots:
                print(f"    {repr(robot)},")
            print("]")

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    main()