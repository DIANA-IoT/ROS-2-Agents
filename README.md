# ROS2_Agents

### Repo struct
```
repo 
    | config
    | include / ros2_agents
    | launch
    | LICENSE
    | sim
    | src
```

- config: YAML files for ROS 2 nodes configuration.
- include / ros2_agents: C++ header files.
- launch: ROS 2 launch file for robot controller.
- sim: Simulation related files.
- src: C++ and python source files (including Python simulation control and pose settings).

## How to launch controlled simulations.
In order to launch simulation control and automation, a CSV-looking file (extension does not matter) should be created. <br>
The file contents must follow the next scheme: `launch_file,repetitions` where `launch_file`
must contain the path to the simulation file.
<br>
The aim of the above mentioned file is to list, and therefore, launch every file so each simulation will take place the number of times specified at `repetitions`.
### Turtlebot 3 Gazebo Classic simulation.
Inside sim/ directory model.sdf is found, which corresponds to robot's simulation model intentionally modified to publish ground-truth pose.<br>
The path will like similar to this ```/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models/turtlebot3_burger```
<br>

## Control sim
A Python file called python_sim.py is given for the deed of launching/stopping simulations. <br>
Remember that it is mandatory to source both system-wide install and workspace install folders. <br>

Invokation: `ros2 run ros2_agents python_sim.py CSV-looking_text_file`.
<br>

## Standalone sim
In this case, you are responsible of launching and halting simulation by yourself. <br>
`ros2 launch ros2_agents launch_file.py`

## Using Stage simulator.
Several launch files are prepared to work with stage (sim/stage_sim...). <br>
It is crucial to have previously compiled and sourced stage_ros2 installation. <br>
In order to simulate, stage needs a world file, this world file is passed to stage's launch file by an upper-level
launch file (stage_sim...). <br>
The world file can be treated as a parameter to the launch system or modified as a default setting in the Python file.

## Debug symbols compilation.
`colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug`

## Dependencies 
### GitLab dependencies and repos.
* Ros2_mwsn_msgs: ROS 2 package that defines the interface of the messages used between controllers and supervisor.
* Stage: Stage simulator core.
* Stage_ros2: ROS 2 Stage simulator port from TU Wien.
### ROS 2 and system dependencies.
* NAV2 common: ` ros-humble-nav2-common + ros-humble-nav2-bringup. `
* Transforms 3D: ` sudo pip3 install transforms3d. ` + `ros-humble-tf-transformations`.
* Colcon build system: `sudo apt install python3-colcon-common-extensions`.
### Some (hopefully) helpful notes
Sometimes dependency meeting won't work or the build system gets 'stuck'. Some tips:
* Try to clean the workspace by deleting build/ install/ log/ folders.
* Compile with the least amount of options possible, i.e., `colcon build --symlink-install` (By symlinking installing a quite large time is saved). <br>

## Troubleshooting
Some weird errors might happen from time to time, here there is a list of the most common ones:
* Node communication fails, nodes are not identified by CLI tools. Likely cause RMW implementation: *Cyclone DDS* works moreless fine (`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`).
* Usually it is helpful to enable local-only communications if simulations are used. See `ROS_LOCALHOST_ONLY`.
* Nodes are crashing. Likely cause: configuration files or memory corruption.
* Stage simulator crashes. Likely cause: world file is incorrect.
* If the depedency `ros2_mwsn_msgs`cannot be found or the proyect cannot be built, firstly ensure to build <br>
`ros2_mwsn_msgs` by typing `colcon build --packages-select ros2_mwsn_msgs`, then source the workspace and proceed building the remaining packages.

## Building documentation
This documentation relies on Doxygen. To install Doxygen and its dependencies use `sudo apt install doxygen-doc doxygen-doxyparse doxygen-gui doxygen-latex`
* To build HTML and LaTeX, simply run `doxygen Doxyfile`.
* If PDF documentation is needed, from `doc_doxygen/latex`, run `make`.

## Cross compiling and running (real Turtlebot 3)
### Via Turtlebot's sysroot and native toolchain
To cross-compile we need to be able to access the Turtlebot 3 sysroot. This can be achived by either mounting the filesystem from an SD card 
image or by mounting an actual running robot using SSHFS with the option `-o transform_symlinks`. 
Due to the fact that the toolchain file likely overrides colcon's default building configuration, and accounting that this project depends on 
**ros2_agents_msgs** project, some special steps are needed.
1. Cross-compile first ros2_agents_msgs: `colcon build --packages-select ros2_agents_msgs --cmake-force-configure --cmake-args -DCMAKE_TOOLCHAIN_FILE=PATH_TO_TOOLCHAIN_FILE` 
This will generate build files in the directory called `build` and install files in `install`.
2. Source the installation directory of the above project. `source install\setup.bash`
3. Cross-compile ros2_agents project: `colcon build --packages-select ros2_agents --cmake-force-configure --cmake-args -DCMAKE_TOOLCHAIN_FILE=PATH_TO_TOOLCHAIN_FILE -DCMAKE_PREFIX_PATH=PATH_TO_PI_INSTALL_LOC`

</ol>
Those steps, especially if they are executed with SSHFS filesystems, can take a while to execute. 

### Using QEMU + docker containers (quicker and easier than native compiling)
Accounting that a usable docker installation is done, it is neccessary to install the QEMU support package for `aarch64`. This can be done by:
`sudo apt-get install qemu binfmt-support qemu-user-static` <br>
`docker run --rm --privileged multiarch/qemu-user-static --reset -p yes` <br>
The docker image that can be built with the following docker building instructions: 
```
FROM arm64v8/ros:humble
RUN apt -y update && apt -y install git \
	cmake \
	g++ \
	libjpeg8-dev \
	libpng-dev \ 
	libglu1-mesa-dev \
	libltdl-dev \
	libfltk1.1-dev\
	ros-humble-nav2-common \
	ros-humble-nav2-bringup \
	ros-humble-tf-transformations \
	python3-pip \
	python3-colcon-common-extensions \
	google-perftools \
	libgoogle-perftools-dev
RUN pip3 install transforms3d
RUN mkdir -p /home/ubuntu/ros2_ws/ws
WORKDIR /home/ubuntu/ros2_ws/ws
``` 
To compile the package, simply mount the workspace folder and run colcon, like shown:
`docker run --rm -it -v $WORKSPACE_DIR:/home/ubuntu/ros2_ws/ws DOCKER_IMAGE:VERSION colcon build`

### Running
To run this project, the TB3 filesystem could easily be mounted by SSHFS and directories `build` and `install` copied to 
the robot, eg, `/home/robot_user/ros2_ws/` (do not forget to source this environment on robot's terminal). <br>
Another possibility, quicker and that introduces less wear on the SD card can be found by `rsync -av ORIGIN robotUSR@HOSTNAME:ROBOTDIR`

In order to launch the processes, a ROS 2 launch file called `tb3_launch.py` is provided. 
This launch file shold be called with the arguments "namespace" and "config". Eg.:
`ros2 launch ros2_agents tb3_launch.py namespace:='controller0' config:='/home/ubuntu/ros2_ws/install/ros2_agents/share/ros2_agents/config/parameters_robot0.yaml'"`

## Credits

This software was actively developed and contributed by:
 - José Manuel Cano García
 - José Borja Castillo Sánchez
 - Eva González Parada
 - Mirgita Frasheri


## Copyright and License

This software is distributed under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This package is at Copyright (c) University of Malaga. This copyright information is specified in the headers of the corresponding files.

## Acknowledgements

This work has been supported by project TED2021-
130456B-I00, funded by MCIN/AEI/10.13039/501100011033
and EU ”NextGenerationEU/PRTR” program and the Malaga
University project B4-2023-12.