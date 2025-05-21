// Project Title: ROS2_AGENTS
// File: include/ros2_agents/robot_class.hpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#ifndef __ROBOT_CLASS_HPP__
#define __ROBOT_CLASS_HPP__

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "ros2_agents/consumption_model.hpp"
#include "ros2_agents/defs.hpp"
#include "ros2_agents/negotiation_class.hpp"
#include "ros2_agents/spf_utilities.hpp"
#include "ros2_mwsn_msgs/msg/supervisor2_controller.hpp"
#include "ros2_agents/spf.hpp"

/// @brief A single instance of a robot.
class RobotClass
{
public:
    //! RobotClass parametrized constructor.
    /*!
        The constructor creates a new instance of a differential robot
        capable of moving.
        \param id: String containing robot identifier.
    */
    // explicit RobotClass(const std::string &id);
    explicit RobotClass(rclcpp::Node *parent, const std::string &id, const ControllerTypes &type);
    //! RobotClass copy constructor.
    // RobotClass(const RobotClass &r);
    virtual ~RobotClass();
    /// @brief Initialize RobotClass (create pubs, subs, construct additional classes, etc.)
    void initialize(void);

private:
    /// @brief Callback triggered when the robot reaches the value of BATTERY_CRITICAL_PER
    /// @param value Current battery value (below BATTERY_CRITICAL_PER)
    void battery_CriticalCallback(const double &value);
    /// @brief Callback to store the initial pose
    /// @param msg the message containing the well-known initial pose
    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);
    /// @brief Function to send role changes to the supervisor. The msg type is MESSAGE_SET_ROLE
    /// @param role the newly acquired role.
    void notify_RoleChange(const RoleTypes &role);
    /// @brief Auxiliary function to check and set if the commanded role by the supervisor can be applied.
    /// @param recv_role The role the supervisor is suggesting.
    /// @return The role received at recv_role if appliable, if not, returns the same.
    RoleTypes set_newRoleAgents(const RoleTypes &recv_role);
    /// @brief Auxiliary function to check and set if the commanded role by the supervisor can be applied.
    /// @param recv_role The role the supervisor is suggesting.
    /// @return The role received at recv_role if appliable, if not, returns the same.
    RoleTypes set_newRoleSPF(const RoleTypes &recv_role);
    /// @brief Observer callback to handle changes in SPF role
    /// @param role the role notified by the SPF
    void spf_role_callback(const RoleTypes &role);
    /// @brief Callback triggered as a response of the arrival of a Supervisor message
    /// @param msg Supervisor2Controller message.
    void supervisor_msg_callback(const ros2_mwsn_msgs::msg::Supervisor2Controller &msg);

    /* Internal variables*/

    ControllerTypes _controller;
    LocType_t _localization;
    geometry_msgs::msg::Pose _my_initial_pose;
    geometry_msgs::msg::Point _my_pose;
    int16_t _numWhoAmI;
    RoleTypes _current_role;

    rclcpp::Publisher<ros2_mwsn_msgs::msg::Controller2Supervisor>::SharedPtr robot_pub_;
    rclcpp::Subscription<ros2_mwsn_msgs::msg::Supervisor2Controller>::SharedPtr controller_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
    rclcpp::Node *parent_ptr;

    std::string _whoAmI;

    /* Internal classes */
    ConsumptionModel *consumption_model_;
    // NegotiationRobot *nc_;
    SPF *spf_;
};

#endif