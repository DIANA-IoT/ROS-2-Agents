// Project Title: ROS2_AGENTS
// File: src/spf_utilities.cpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#include "ros2_agents/defs.hpp"
#include "ros2_agents/spf_utilities.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace spf_util
{

	double calcExp(double dis, double k1, double k2)
	{
		// return (k1 * (1 - exp(k2 * dis)));
		return (k1 * (exp(dis / k2) - 1));
	}

	double calcPot(double dis, double sigma, double n)
	{
		return (sigma / std::pow(dis, n));
	}

	double compute_critical_level(const double &distance)
	{
		return (100.0 * (distance * BATTERY_WASTE_PER_DISTANCE + 0.05 * ROBOT_INITIAL_BATTERY) / BATTERY_CAPACITY_MAH);
	}

	std::vector<int16_t> compute_neighbours(const int16_t &my_id, const std::vector<geometry_msgs::msg::Point> &robot_pos,
										  const std::vector<bool> &dead_list_, const float &distance_threshold)
	{
		std::vector<int16_t> neighbours_id;
		auto vector_size = robot_pos.size();
		for (int16_t i = 0; i < (int16_t) vector_size; i++)
		{
			// Do not lookup for myself or if the robot is dead
			if (i == my_id || dead_list_[i])
				continue;
			if (compute_point_distances(robot_pos[my_id], robot_pos[i]) < distance_threshold)
				neighbours_id.emplace_back(i);
		}
		return neighbours_id;
	}

	double compute_point_angles(const geometry_msgs::msg::Point &p1,
								const geometry_msgs::msg::Point &p2)
	{
		return std::atan2(p2.y - p1.y, p2.x - p1.x);
	}

	double compute_point_distances(const geometry_msgs::msg::Point &p1,
								   const geometry_msgs::msg::Point &p2)
	{
		/* We don't care about Z-coordinate. */
		return std::hypot(p2.x - p1.x, p2.y - p1.y);
	}

	int16_t count_dead(const std::vector<bool> &dead_list_)
	{
		return std::count(dead_list_.begin(), dead_list_.end(), true);
	}

	geometry_msgs::msg::Pose transform_pose_rel2abs(const geometry_msgs::msg::Pose &initial, const geometry_msgs::msg::Pose &translation)
	{
		/* Transform utility holding the initial pose in TF2 format. */
		tf2::Transform origin2pose;
		auto initial_position = initial.position;
		auto initial_orientation = initial.orientation;
		/* Establish initial Coordinates and rotation. 	*/
		origin2pose.setOrigin(tf2::Vector3(initial_position.x, initial_position.y, initial_position.z));
		origin2pose.setRotation(tf2::Quaternion(initial_orientation.x, initial_orientation.y,
												initial_orientation.z, initial_orientation.w));
		/* Current Odometry position in TF2 format. */
		tf2::Transform pose2odometry;
		pose2odometry.setOrigin(tf2::Vector3(translation.position.x, translation.position.y, translation.position.z));
		pose2odometry.setRotation(tf2::Quaternion(translation.orientation.x,
												  translation.orientation.y,
												  translation.orientation.z,
												  translation.orientation.w));
		/* Current odometry in absolute coordinates. */
		auto tf2_result = (origin2pose * pose2odometry);
		auto result = geometry_msgs::msg::Pose();
		result.position.x = tf2_result.getOrigin().getX();
		result.position.y = tf2_result.getOrigin().getY();
		result.position.z = tf2_result.getOrigin().getZ();
		result.orientation.w = tf2_result.getRotation().getW();
		result.orientation.x = tf2_result.getRotation().getX();
		result.orientation.y = tf2_result.getRotation().getY();
		result.orientation.z = tf2_result.getRotation().getZ();
		return result;
	}

}
