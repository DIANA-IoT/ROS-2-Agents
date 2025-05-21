// Project Title: ROS2_AGENTS
// File: include/ros2_agents/spf_utilities.hpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

#ifndef __SPF_UTILITIES_HPP_
#define __SPF_UTILITIES_HPP_

#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace spf_util
{
    double calcExp(double dis, double k1, double k2);
    double calcPot(double dis, double sigma, double n);
    /// @brief Computes the critical battery level according to the distance to a target.
    /// @param distance Distance to a target or depot. Measured in meters.
    /// @return The battery percentage considered the absolute minimum to reach that distance.
    double compute_critical_level(const double &distance);
    /// @brief Compute the vector of nearby neighbours.
    /// @param my_id Robot id whose neighbours shall be found.
    /// @param robot_pos Vector with every robot in the network pose.
    /// @param dead_list_ Vector with the dead robots marked as true.
    /// @param distance_threshold Threshold (usually a radio coverage) for a robot to considered neighbour.
    /// @return A vector with all neighbours for a robot my_id.
    std::vector<int16_t> compute_neighbours(const int16_t &my_id, const std::vector<geometry_msgs::msg::Point> &robot_pos,
                                          const std::vector<bool> &dead_list_, const float &distance_threshold);
    /// @brief  Computes the angle between two points.
    /// \param p1, p2: Points to compute their angles.
    /// \return Angle in radians.
    double compute_point_angles(const geometry_msgs::msg::Point &p1,
                                const geometry_msgs::msg::Point &p2);

    /// @brief  Computes the distance between two points.
    /// \param p1, p2: Points to compute their distance.
    /// \return Distance in meters.
    double compute_point_distances(const geometry_msgs::msg::Point &p1,
                                   const geometry_msgs::msg::Point &p2);
    /// @brief Counts the number of dead nodes (or at least, those who have been reported dead).
    /// @param boolean vector indexed with the robot id. True if robot dead.
    /// @return Int16 with the number of depleted nodes.
    int16_t count_dead(const std::vector<bool> &dead_list_);
    /// @brief Performs the translation and rotation between the points initial and translation
    /*!
        This function should be used to "convert" proprioceptive odometry into an absolute point in the reference frame.
    */
    /// @param initial geometry_msg with the initial pose.
    /// @param translation geometry_msg with the translation (aka, the displacement respect initial).
    /// @return The pose in absolute coordinates.
    geometry_msgs::msg::Pose transform_pose_rel2abs(const geometry_msgs::msg::Pose &initial, const geometry_msgs::msg::Pose &translation);
}

#endif