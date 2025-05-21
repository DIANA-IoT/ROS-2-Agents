// Project Title: ROS2_AGENTS
// File: include/ros2_agents/robot_logger.hpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#ifndef __ROBOT_LOGGER_HPP__
#define __ROBOT_LOGGER_HPP__

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros2_agents/defs.hpp"
#include "ros2_mwsn_msgs/msg/controller2_supervisor.hpp"
#include "std_msgs/msg/float64.hpp"

/// @brief Robot Logger class: A logging class to be used by class Supervisor
/*!
    This class logs (almost) every relevant aspect in the robot such as:
        - The pose given by each robot.
        - Battery level.
        - Role.
    These data is especially useful to detect robot shutdowns or to get the Supervisor involved
    in the negotiation process.
*/
class RobotLogger
{
public:
    //! Parametrized RobotLogger constructor.
    /// @param id: ID of the robot.
    explicit RobotLogger(const std::string &id);
    //! Copy constructor for RobotLogger
    RobotLogger(const RobotLogger &r);
    virtual ~RobotLogger();
    /* Subscription callbacks */
    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);
    void robot_feedback_callback(const ros2_mwsn_msgs::msg::Controller2Supervisor &msg);
    /* Gets battery percentage (0-100%) */
    float get_batteryPercentage(void) const;
    /// @brief Retrieves current 2-D robot poisition (X,Y). Z is the orientation value.
    /// @param  None
    /// @return A point message type with X,Y positions and orientation (Z).
    geometry_msgs::msg::Point get_currentPosition(void) const;
    /* Return current robot role. */
    RoleTypes get_currentRole(void) const;
    /* Return the total distance the robot has moved. */
    double get_distanceWalked(void) const;
    /* Returns true if robot is static. */
    bool get_isRobotStatic(void) const;
    /* Returns true if robot is stationary. */
    bool get_isRobotStationary(void) const;
    /* Get Robot ID */
    std::string get_RobotId(void) const;
    /* Get Robot Numeric ID */
    int16_t get_RobotNId(void) const;
    /* Get total moved distance (m) */
    double get_totalMovement(void) const;
    /* To detect if the robot is actually moving, freeze current pose. Currently unused */
    //! Unused
    void set_frozenPose(void);
    /// @brief Sets robot position in X, Y, theta.
    void set_robotPosition(const float &x, const float &y, const float &theta);

private:
    /* Internal mutex */
    std::mutex *m;
    /* Class attributes */
    float movement_threshold_;
    geometry_msgs::msg::Point current_pose_, frozen_pose_; /* Important: Z-value will be treated as Theta (orientation)*/
    float btc_;
    std_msgs::msg::Float64 total_movement_;
    std::string id_;
    int16_t nid_;
    bool is_stationary_ = false;
    RoleTypes current_role_;
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_;

    double distance_walked_;
};

#endif