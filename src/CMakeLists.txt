add_library(${library_name} SHARED
    negotiation_class.cpp
    network_simulator.cpp
    robot_class.cpp
    robot_logger.cpp
    spf.cpp
    spf_utilities.cpp
    supervisor_msgs.cpp
    supervisor_negotiation.cpp
    supervisor_node.cpp
    topic_utilities.cpp
    visualization_class.cpp)
    
ament_target_dependencies(${library_name}
    rclcpp
    sensor_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
    rosbag2_cpp
    ros2_mwsn_msgs
    std_msgs
    visualization_msgs)

add_executable(robot_controller robot_controller.cpp)
add_executable (supervisor supervisor_main.cpp)
target_link_libraries(robot_controller ${library_name})
target_link_libraries(supervisor ${library_name})
install(TARGETS
    ${library_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
