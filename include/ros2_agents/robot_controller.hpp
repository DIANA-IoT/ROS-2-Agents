// Project Title: ROS2_AGENTS
// File: include/ros2_agents/robot_controller.hpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#ifndef __ROBOT_CONTROLLER_HPP__
#define __ROBOT_CONTROLLER_HPP__

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "ros2_agents/defs.hpp"
#include "ros2_agents/robot_class.hpp"


/// @brief Robot Controller class: A ROS 2 node to use one or more instances or RobotClass
/*!
	The RobotController class handles ROS 2 API class in order to control real or virtual robots.
	This class can accept more than one RobotClass as long as it is not controlling real robots.
	It is crucial that parameters parsed at initialization are corred, as they will be used to determine
	which robots are to be controlled.
*/
class RobotController: public rclcpp::Node
{
public:
	/*!
		@brief Default RobotController constructor, as it inherits from RCLCPP::Node,
		some options can be passed.
	*/
	explicit RobotController(const rclcpp::NodeOptions &node_options =
			rclcpp::NodeOptions());
	virtual ~RobotController();
	void initialize_system(void);
private:
	/* Private funcs. */
	void initialize_new_robot(const std::string &robotId);

	/* Internal variables */
	LocType_t localizationUsed;
	ControllerTypes controller;
	
	std::vector<RobotClass *> robotList_;

	/* Parameters */
	/* Controller index: My index*/
	int controller_index_;
	/* Number of robots I am in charge of. */
	int robots_to_control_;
	/* Total number of robots. */
	int number_of_robots_;
};

#endif
