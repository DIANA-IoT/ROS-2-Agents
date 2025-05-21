// Project Title: ROS2_AGENTS
// File: src/supervisor_msgs.cpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#include "ros2_agents/supervisor.hpp"
#include <random>

/*! @brief Source file where supervisor communication-functions are implemented. */

using namespace std::chrono_literals;

void Supervisor::assign_battery_levels(void)
{
	const int battery_min = 90;
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> distr(battery_min, 100);
	for (int i = 0; i < n_robots_; i++)
	{
		uint8_t bat;
		bat = (uint8_t)distr(gen);
		auto msg = ros2_mwsn_msgs::msg::Supervisor2Controller();
		msg.destination = i;
		msg.type = MESSAGE_SET_BATTERY;
		msg.data.push_back(bat);
		robotCommander_pub_->publish(msg);
		rclcpp::sleep_for(10ms);
	}
}

void Supervisor::kill_stage_robot(const int16_t &id)
{
	auto new_msg = std_msgs::msg::Int16();
	new_msg.data = id;
	/* Kills robot communications */
	killRobot_pub_->publish(std::move(new_msg));
}

void Supervisor::position_callback(const ros2_mwsn_msgs::msg::Position &msg)
{
	RobotLoggerList_.at(msg.origin)->set_robotPosition(msg.x, msg.y, msg.theta);
}

void Supervisor::robot_callback(const ros2_mwsn_msgs::msg::Controller2Supervisor &msg)
{
	RobotLoggerList_[msg.origin]->robot_feedback_callback(msg);
	if (msg.type == MESSAGE_SET_ROLE)
	{
		if (msg.data[0] == ROLE_DEAD)
		{
			RobotDeadList_.at(msg.origin) = true;
			kill_stage_robot(msg.origin);
			/* Notify a dead robot using Supervisor2Controller MSG */
			auto new_msg = ros2_mwsn_msgs::msg::Supervisor2Controller();
			new_msg.destination = MSG_DESTINATION_BROADCAST;
			new_msg.type = MESSAGE_ROBOT_DIED;
			new_msg.data.resize(sizeof(MessageDiedRobotParameter));
			memcpy(&new_msg.data[0], &msg.origin, sizeof(int16_t));
			robotCommander_pub_->publish(std::move(new_msg));
		}
		cv->notify_all();
	}
	
}

void Supervisor::run_fast(void)
{
	auto msg = std_msgs::msg::String();
	msg.data = "fast";
	sim_control_pub_->publish(msg);
}

void Supervisor::run_realtime(void)
{
	auto msg = std_msgs::msg::String();
	msg.data = "real-time";
	sim_control_pub_->publish(msg);
}

void Supervisor::send_deploy_msg(void)
{
	auto msg = ros2_mwsn_msgs::msg::Supervisor2Controller();
	msg.type = MessageTypes::MESSAGE_RUN;
	msg.data.resize(sizeof(MessageRoleSetParameter));
	MessageRoleSetParameter payload;
	payload.role = RoleTypes::ROLE_DEPLOYING;
	msg.destination = MSG_DESTINATION_BROADCAST;
	memcpy(&msg.data[0], &payload, sizeof(payload));
	robotCommander_pub_->publish(std::move(msg));
}

void Supervisor::send_depot_msg(const int16_t &robot_id)
{
	auto msg = ros2_mwsn_msgs::msg::Supervisor2Controller();
	msg.type = MessageTypes::MESSAGE_SET_ROLE;
	msg.data.resize(sizeof(MessageRoleSetParameter));
	MessageRoleSetParameter payload;
	payload.role = RoleTypes::ROLE_RETRIEVING;
	msg.destination = robot_id;
	memcpy(&msg.data[0], &payload, sizeof(payload));
	robotCommander_pub_->publish(std::move(msg));
}

void Supervisor::send_end_negotiation_msg(void)
{
	auto msg = ros2_mwsn_msgs::msg::Supervisor2Controller();
	msg.type = MessageTypes::MESSAGE_DONE_NEGOTIATION;
	/* Right now, I think it is easier just to treat this message as a
		broadcast message. */
	msg.destination = MSG_DESTINATION_BROADCAST;
	robotCommander_pub_->publish(std::move(msg));
	RCLCPP_INFO(this->get_logger(), "Negotiation round complete");
}

void Supervisor::send_replace_msg(const int16_t &origin, const int16_t &destination, const float &x, const float &y)
{
	auto msg = ros2_mwsn_msgs::msg::Supervisor2Controller();
	msg.destination = destination; /* The identifier to whom the message its directed. */
	msg.type = MESSAGE_SET_DESTINATION;
	msg.data.resize(sizeof(MessageHelpParameter));
	MessageHelpParameter payload;
	payload.origin = origin; /* The robot who needs help*/
	payload.x = x;
	payload.y = y;
	memcpy(&msg.data[0], &payload, sizeof(payload));
	robotCommander_pub_->publish(std::move(msg));
}

void Supervisor::send_static_msg(void)
{
	auto msg = ros2_mwsn_msgs::msg::Supervisor2Controller();
	msg.destination = MSG_DESTINATION_BROADCAST;
	msg.type = MessageTypes::MESSAGE_SET_ROLE;
	msg.data.resize(sizeof(MessageRoleSetParameter));
	MessageRoleSetParameter payload;
	payload.role = RoleTypes::ROLE_STATIC;
	memcpy(&msg.data[0], &payload, sizeof(payload));
	robotCommander_pub_->publish(std::move(msg));
}

void Supervisor::set_AP(void)
{
	/* Create the variable for the (0,0) coordinates. */
	geometry_msgs::msg::Point centre;
	centre.x = 0.0;
	centre.y = 0.0;
	/* Distances from roboti to (0,0). */
	double closest = 100.0;
	double distance;
	int16_t index = -1;
	/* Determine which one is the closest. */
	for (int16_t i = 0; i < n_robots_; i++)
	{
		RobotLogger *ptr_to_robot = RobotLoggerList_.at(i);
		distance = spf_util::compute_point_distances(centre,
														 ptr_to_robot->get_currentPosition());
		if (distance < closest)
		{
			closest	= distance;
			index = i;
		}
	}
	RCLCPP_INFO(this->get_logger(), "Setting robot %d as AP", index);
	indexAP = index;
	/* Send message. */
	auto msg = ros2_mwsn_msgs::msg::Supervisor2Controller();
	msg.destination = index;
	msg.type = MESSAGE_SET_ROLE;
	msg.data.resize(sizeof(RoleTypes));
	msg.data.insert(msg.data.begin(), (unsigned char)RoleTypes::ROLE_AP);
	robotCommander_pub_->publish(std::move(msg));
}

void Supervisor::start_simulation(void)
{
	/* Firstly, assigns randomly a battery level for each robot. */
	/// assign_battery_levels();
	rclcpp::sleep_for(10s);
	RCLCPP_INFO(this->get_logger(), "Starting simulation");
	set_AP();
	/* Starts simulation by telling each robot to change their Role/State.
		to Deploying mode, except for the AP. */
	current_state_ = NetworkStates::DEPLOYMENT;
	send_deploy_msg();
	/* Speedup simulation. */
	run_fast();
}
