// Project Title: ROS2_AGENTS
// File: src/robot_controller.cpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#include <functional>
#include <string>
#include <memory>
#include <chrono>
#include <csignal>
#include <algorithm>
#include <math.h>
#include <cassert>
#include <assert.h>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "ros2_agents/robot_controller.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

RobotController::RobotController(const rclcpp::NodeOptions &node_options) : Node("RobotController", node_options)
{
}

RobotController::~RobotController()
{
}

void RobotController::initialize_new_robot(const std::string &robotId)
{
	auto parent_ptr = this;
	RobotClass *robot = new RobotClass(parent_ptr, robotId, controller);
	robot->initialize();
	/* Store the newly created robot */
	robotList_.push_back(robot);
}

void RobotController::initialize_system(void)
{
	std::string _loc_type, _controller_type;
	this->declare_parameter<int>("controller_index", 0);
	this->declare_parameter<int>("robots_to_control", 1);
	this->declare_parameter<int>("total_robots", 100);
	this->declare_parameter<std::string>("localization_type", "");
	this->declare_parameter<std::string>("type", "");
	/* Declare parameters even though we are not going to use them inmediatly. 
		This avoids the 'already declared parameter' exception. */
	this->declare_parameter<float>("max_vel", 0.22);
    this->declare_parameter<float>("laser_min_range", 0.2);
    this->declare_parameter<float>("laser_max_range", 3.5);
    this->declare_parameter<float>("forces.repulsion_sigma_robot", 0.5);
    this->declare_parameter<float>("forces.repulsion_n_robot", 0.2);
    this->declare_parameter<float>("forces.repulsion_sigma_object", 0.5);
    this->declare_parameter<float>("forces.repulsion_n_object", 0.2);
    this->declare_parameter<float>("forces.attraction_k1", 1.0);
    this->declare_parameter<float>("forces.attraction_k2", 1.0);
	this->declare_parameter<float>("forces.cohesion_alpha", 5.0);
    this->declare_parameter<int>("force_update_ms", 500);
    this->declare_parameter<float>("_movement_threshold", 0.2);
	this->declare_parameter<float>("noise_k", 1.0);

	this->get_parameter_or<int>("controller_index", controller_index_, 0);
	this->get_parameter_or<int>("robots_to_control", robots_to_control_, 1);
	this->get_parameter_or<int>("total_robots", number_of_robots_, 100);
	this->get_parameter_or<std::string>("localization_type", _loc_type, "");
	this->get_parameter_or<std::string>("type", _controller_type, "");
	if (_loc_type == "AMCL")
	{
		localizationUsed = LocType_t::AMCL;
	}
	else if (_loc_type == "GroundTruth")
	{
		localizationUsed = LocType_t::GroundTruth;
	}
	else if (_loc_type == "Odometry")
	{
		localizationUsed = LocType_t::Odometry;
	}
	else
	{
		RCLCPP_FATAL(this->get_logger(), "Unknown localization type. Aborting");
		rclcpp::shutdown();
	}
	if (_controller_type == "Agents")
	{
		controller = ControllerTypes::CONTROLLER_TYPE_AAGENTS;
	}
	else if (_controller_type == "SPF")
	{
		controller = ControllerTypes::CONTROLLER_TYPE_SPF;
	}
	else
	{
		RCLCPP_FATAL(this->get_logger(), "Unknown controller type. Aborting");
		rclcpp::shutdown();
	}
	rclcpp::sleep_for(5s); // This should not be neccesary in a future.
	/* Resizes each array to fit and arrange different robot variables. */
	/* This array could be merged into _robotRoles_, we are keeping them
		separated for simplicity. */
	// _robotIsDeadList_.resize(number_of_robots_);
	auto index = controller_index_ * robots_to_control_;
	std::string loc;
	/* Initialize each controlled robot. */
	while (index < ((controller_index_ + 1) * robots_to_control_))
	{
		// RCLCPP_INFO(this->get_logger(), "Initializing robot %d", index);
		std::string new_robotId = "/robot" + std::to_string(index);
		initialize_new_robot(new_robotId);
		index++;
	}
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<RobotController>();
	/* Call to initialize_system once constructor exits, so that shared pointers
	are valid. */
	node->initialize_system();
	rclcpp::spin(node->get_node_base_interface());
	rclcpp::shutdown();
	return 0;
}
