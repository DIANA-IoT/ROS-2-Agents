cmake_minimum_required(VERSION 3.20)
project(ros2_agents)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_mwsn_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
include_directories(include)

set(dependencies "rclcpp" "sensor_msgs" "nav_msgs" "tf2" "tf2_geometry_msgs" "rosbag2_cpp" "std_msgs" "ros2_mwsn_msgs") 

set(library_name ${PROJECT_NAME}_core)
add_subdirectory(src)

install(DIRECTORY include/
  DESTINATION include/)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY 
  launch/
  sim/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py")

install(DIRECTORY
  sim/
  DESTINATION share/${PROJECT_NAME}/maps
  PATTERN "*.py" EXCLUDE) 

install(TARGETS
   robot_controller
   supervisor 
   DESTINATION lib/${PROJECT_NAME})

install(FILES
  src/pub_initial_pose.py
  src/python_sim.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
