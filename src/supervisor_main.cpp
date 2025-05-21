// Project Title: ROS2_AGENTS
// File: src/supervisor_main.cpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_agents/supervisor.hpp"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::sleep_for(std::chrono::seconds(10));
	rclcpp::executors::SingleThreadedExecutor executor;
	auto node = std::make_shared<Supervisor>();
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}