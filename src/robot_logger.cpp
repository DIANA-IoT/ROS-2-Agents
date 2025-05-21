// Project Title: ROS2_AGENTS
// File: src/robot_logger.cpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#include "ros2_agents/robot_logger.hpp"
#include "ros2_agents/topic_utilities.hpp"
#include "ros2_agents/spf_utilities.hpp"
#include <cmath>

RobotLogger::RobotLogger(const std::string &id)
{
	m = new std::mutex();
	current_pose_ = geometry_msgs::msg::Point();
	movement_threshold_ = 0.05;
	btc_ = 100.0; // 100% until first battery report
	id_ = id;
	nid_ = 0;
	(void)topic_util::getRobotNumber(id, nid_);
	total_movement_.data = 0.0;
	is_stationary_ = false;
	current_role_ = RoleTypes::ROLE_STATIONARY;

	distance_walked_ = 0.0;
}

RobotLogger::RobotLogger(const RobotLogger &r)
{
	btc_ = r.btc_;
	current_pose_ = r.current_pose_;
	current_role_ = r.current_role_;
	id_ = r.id_;
	is_stationary_ = r.is_stationary_;
	initial_pose_ = r.initial_pose_;
	m = r.m;
	movement_threshold_ = r.movement_threshold_;
	nid_ = r.nid_;
	total_movement_ = r.total_movement_;

	distance_walked_ = r.distance_walked_;
}

RobotLogger::~RobotLogger()
{
}

float RobotLogger::get_batteryPercentage(void) const
{
	std::scoped_lock lock{*m};
	return btc_;
}

geometry_msgs::msg::Point RobotLogger::get_currentPosition(void) const
{
	std::scoped_lock lock{*m};
	return current_pose_;
}

RoleTypes RobotLogger::get_currentRole(void) const
{
	std::scoped_lock lock{*m};
	return current_role_;
}

double RobotLogger::get_distanceWalked(void) const
{
	std::scoped_lock lock{*m};
	return distance_walked_;
}

bool RobotLogger::get_isRobotStatic(void) const
{
	std::scoped_lock lock{*m};
	return (current_role_ == ROLE_STATIC);
}

bool RobotLogger::get_isRobotStationary(void) const
{
	std::scoped_lock lock{*m};
	return is_stationary_;
}

std::string RobotLogger::get_RobotId(void) const
{
	std::scoped_lock lock{*m};
	return id_;
}

int16_t RobotLogger::get_RobotNId(void) const
{
	std::scoped_lock lock{*m};
	return nid_;
}

double RobotLogger::get_totalMovement(void) const
{
	std::scoped_lock lock{*m};
	return total_movement_.data;
}

void RobotLogger::initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
{
	std::scoped_lock lock{*m};
	initial_pose_ = msg;
	current_pose_ = initial_pose_.pose.pose.position;
}

void RobotLogger::robot_feedback_callback(const ros2_mwsn_msgs::msg::Controller2Supervisor &msg)
{
	std::scoped_lock lock{*m};
	if (msg.origin != nid_)
	{
		return;
	}
	switch (msg.type)
	{
	case MessageTypes::MESSAGE_RUN:
		current_role_ = static_cast<RoleTypes>(*msg.data.begin());
		is_stationary_ = (current_role_ == RoleTypes::ROLE_STATIONARY);
		break;
	case MessageTypes::MESSAGE_SET_BATTERY:
	{
		double bl = 0;
		memcpy(&bl, &msg.data[0], sizeof(double));
		btc_ = (float)bl;
		break;
	}
	case MessageTypes::MESSAGE_SET_DESTINATION:
		break;
	case MessageTypes::MESSAGE_SET_ROLE:
		current_role_ = static_cast<RoleTypes>(*msg.data.begin());
		is_stationary_ = (current_role_ == RoleTypes::ROLE_STATIONARY);
		break;
	default:
		break;
	}
}

void RobotLogger::set_frozenPose(void)
{
	std::scoped_lock lock{*m};
	frozen_pose_ = current_pose_;
}

void RobotLogger::set_robotPosition(const float &x, const float &y, const float &theta)
{
	std::scoped_lock lock{*m};
	geometry_msgs::msg::Point tmp;
	tmp.x = x;
	tmp.y = y;
	distance_walked_ += spf_util::compute_point_distances(tmp, current_pose_);
	current_pose_.x = x;
	current_pose_.y = y;
	current_pose_.z = theta;
}

