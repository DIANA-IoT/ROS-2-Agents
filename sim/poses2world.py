# Project Title: ROS2_AGENTS
# File: sim/poses2world.py
# Author: José-Borja Castillo-Sánchez, DIANA Group UMA
# Date: 2025
# (c) Copyright by Universidad de Málaga
# License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

#!/usr/bin/python3
# Reads from a Python dictionary-friendly file and creates a stage world.
# Requires several arguments: the file with robot identifiera and positions to be read.
# The output directory where world file will be generated.
# The third argument if present and set to "single", only a world file is created.
# Otherwise, one world is created per robot.
import sys, ast
def load_robots_from_file(file_path):
    with open(file_path, 'r') as file:
        # Read the entire file and evaluate it as Python data
        data = file.read()
         # Strip out the assignment if present
        if data.startswith("robots ="):
            data = data.split("=", 1)[1].strip()  # Remove 'robots =' part
        robots_dict = ast.literal_eval(data)
        return robots_dict

def format_spider_line(robot):
    """Format a single robot dictionary into 'spider(...)' line."""
    return (f'spider( name "{robot["name"]}" localization "gps" '
            f'localization_origin [0 0 0 0] '
            f'pose [ {robot["x_pose"]} {robot["y_pose"]} '
            f'{robot["z_pose"]} {robot["yaw"]} ])')

def generate_output(robots, output_dir=None, single_file=True):
    preamble = """include "map.inc"
include "spiderv4.inc"

# size of the whole simulation
# resolution 0.02

# configure the GUI window	
window
(
    size [ 6000.0 6000.0 ]
    # 600/30 rounded up a bit
    scale 5  
    rotate [ 0  0 ]	
    center [ 0 0 ]
    show_data 1              # 1=on 0=off
) 

# load an environment bitmap
floorplan
(
    name "empty"
    bitmap "bitmaps/empty.png"
    size [ 150.000 150.000 2.00 ]
    pose [0 0 0 0]
    ranger_return 1
    obstacle_return 1
    gui_nose 0
)

# set the resolution of the underlying raytrace model in meters
resolution 0.02
interval_sim 100  # simulation timestep in milliseconds"""
    spider_lines = [format_spider_line(robot) for robot in robots]
    n = len(robots)
    if single_file:
        # Write all data to a single file with a preamble
        output_file = output_dir + f"/sim{n}.world"
        with open(output_file, 'w') as file:
            file.write(preamble + "\n" + "\n".join(spider_lines))
        print(f"Data successfully written to {output_file}")
    else:
        # Write each robot's spider line to a separate file with a preamble
        for index, line in enumerate(spider_lines):
            file_name = output_dir+ f"/simrobot{index}.world"
            with open(file_name, 'w') as file:
                file.write(preamble + "\n" + line)
            print(f"Data for {robots[index]['name']} written to {file_name}")

def main():
    # Check for the correct number of arguments
    if len(sys.argv) < 3:
        print("Usage: python poses2world.py <input_yaml_file> <output_dir> [single|combined]")
        return

    input_file = sys.argv[1]
    output_dir = sys.argv[2]
    single_file = True if len(sys.argv) > 3 and sys.argv[3] == "single" else False

    try:
        # Load robot data from file
        data = load_robots_from_file(input_file)

        # Generate and output spider lines
        generate_output(data, output_dir, single_file)

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
