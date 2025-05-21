// Project Title: ROS2_AGENTS
// File: include/ros2_agents/spf.hpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#ifndef __SPF_HPP__
#define __SPF_HPP__

#include <functional>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/time.hpp"
#include "defs.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_container.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros2_mwsn_msgs/msg/position.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::placeholders;

/// @brief Social Potential Field class for movement.
class SPF
{
public:
    explicit SPF(rclcpp::Node *parent, const int16_t &id, const LocType_t &localization);
    virtual ~SPF();
    /// @brief Get the list of dead robots
    /// @param None
    /// @return Returns the pointer to the list of dead robots
    std::vector<bool> *get_DeadRobots(void);
    /// @brief Returns the current pose
    /// @param None 
    /// @return A point message with the current pose
    geometry_msgs::msg::Point get_Pose(void);
    /// @brief Continue SPF execution, must be initiliazed first. Not to be used during redeployment.
    /// @param  None
    void resume(void);
    /// @brief Add an attractor to the SPF
    /// @param point the 3D position of the attractor, although 'Z' parameter is not used.
    void set_Attractor(const geometry_msgs::msg::Point &point);
    /// @brief Update the list of dead robots so that their last known positions do not take effect anymore.
    /// @param id The id of the dead robot
    void set_DeadRobot(const int16_t &id);
    /// @brief Set the initial pose. Useful when using odometry positioning.
    /// @param pose The well-known initial pose.
    void set_InitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped &pose);
    /// @brief Add function to notify SPF role changes
    /// @param func The function to be used as observer. 
    void set_NotificationFunc(std::function<void(RoleTypes)> func);
    /// @brief Sets new role to use within SPF
    /// @param new_role The new role to be applied.
    void set_Role(const RoleTypes &new_role);
    /// @brief Halt SPF execution, to continue invoke resume function
    /// @param None
    void stop(void);

private:
    /// @brief Computes the forces due to the existence of an attractor.
    /// @param[out] force_x force in the X-Axis
    /// @param[out] force_y force in the Y-Axis.
    void compute_attractive_forces(double &force_x, double &force_y);
    /// @brief Computes the cohesion forces to keep the robots within network reach
    /// @param force_x force in the X-Axis
    /// @param force_y force in the Y-Axis.
    void compute_cohesion_forces(double &force_x, double &force_y);
    /// @brief Computes the forces due to laser-caught obstacles (objects and another robots)
    /// @param force_x force in the X-Axis
    /// @param force_y force in the Y-Axis
    /// @param module resulting module.
    void compute_obstacle_forces(double &force_x, double &force_y, double &module);
    /// @brief Computes the repulsion force due to the existence of other robots nearby. Requieres the knowledge (pose and liveliness) of those robots.
    /// @param force_x force in the X-Axis
    /// @param force_y force in the Y-Axis
    void compute_robot_forces(double &force_x, double &force_y);
    //! Computes differential velocity for the robot.
    /*!
        Computes differential velocity for the robot considering its internal state, or role.
        This method is intended to be called periodically (eg. from a timer) in order to update properly
        the movement commands.
        \return Differential robot speed in axis x (linear.x) and angular speed (angular.z)
    */
    geometry_msgs::msg::Twist compute_velocity(void);
    /// @brief Returns the differential movement that a robot should take depending on angle generated due to the resulting sum of the forces.
    /// @param angle Angle in rads of the total force.
    /// @return Differential movement that the robot should apply.
    geometry_msgs::msg::Twist determine_movement(const double &angle);
    /// @brief Auxiliary function to extract Yaw from a Quaternion
    /// @param quat
    /// @return floating value of the yaw
    float extract_yaw(geometry_msgs::msg::Quaternion &quat);
    /// @brief Publish my current pose to the common position topic.
    void publish_pose(void);

    /* Callbacks. */
    /*! Callback when using AMCL location.*/
    void amcl_sub_callback(std::shared_ptr<rclcpp::SerializedMessage> msg);
    /// @brief Periodic callback triggered by a timer.
    /*!
        This callback checks in first instance, if a robot is still alive.
        Then, it updates the robot pose and heading in order to update the forces and, thus,
        publishing a message that will update robot's velocity.
    */
    void force_callback(void);
    /*! Callback when using Odometry location. Relative to each robot. Also valid for Ground-Truth. */
    void odom_sub_callback(const std::shared_ptr<rclcpp::SerializedMessage> &msg);
    /// @brief
    /// @param
    void laser_callback(const sensor_msgs::msg::LaserScan &msg);
    /*! @brief Callback triggered when received a common position message.
        This callback can be used to fetch every robot pose.
    */
    void robotsPose_callback(const ros2_mwsn_msgs::msg::Position &msg);

    /* Parameters. */
    float attractor_k1_;
    float attractor_k2_;
    float cohesion_alpha_;
    int16_t _numWhoAmI;
    int force_update_;
    float _max_speed;
    float noise_k;
    float laser_min_range;
    float laser_max_range;
    float repulsion_sigma_robot_;
    float repulsion_n_robot_;
    float repulsion_sigma_object_;
    float repulsion_n_object_;
    float _last_yaw;
    float _movement_threshold_;

    /* Internal variables. */
    bool is_agents_; /* true if agents, false if spf-only. */
    bool is_first_deploy; /* true if this is the first deployment. */
    geometry_msgs::msg::Pose _my_initial_pose;
    geometry_msgs::msg::Point _my_pose;
    geometry_msgs::msg::Point _frozen_pose; /* Pose to check when readjustment and replacement to avoid oscillations. */
    /* Total number of robots. */
    int number_of_robots_;
    LocType_t _localization;
    rclcpp::Node *parent_ptr;
    rclcpp::TimerBase::SharedPtr force_update_timer_;
    RoleTypes current_role_;
    /// @brief Vector that holds the liveliness state of each robot (true if dead).
    std::vector<bool> _robotIsDeadList_;
    /// @brief One shared generic publisher for the selected localization type.
    rclcpp::Publisher<ros2_mwsn_msgs::msg::Position>::SharedPtr _robotLocPub_;
    /// @brief Subscription to this robot relative pose.
    std::shared_ptr<rclcpp::GenericSubscription> _robotPoseSub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _robotScanSub_;
    /// @brief Subscription to a generic publisher of absolute poses.
    rclcpp::Subscription<ros2_mwsn_msgs::msg::Position>::SharedPtr _robotAbsPoseSub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _robotVelPub_;
    rclcpp::Time _frozen_time_;

    /*!
        @brief An observer function to notify the parent class of a change in
        an internal variable. In this case, it will notify the RobotClass about
        a role change.
    */
    std::function<void(RoleTypes)> notifier_;

    MessageContainer<geometry_msgs::msg::Point> *_robotPosses_;

    // List with the numerical ids of the nearby neighbours.
    std::vector<int16_t> _neighbour_list_;
    std::vector<geometry_msgs::msg::Point> _attractor_points_;
    std::vector<geometry_msgs::msg::Point> _robotPos_;
    /* Poses to use when simulating losses.*/
    std::vector<geometry_msgs::msg::Point> _robotPosLoss_;
    std::vector<float> _robotYaw_;

    sensor_msgs::msg::LaserScan _last_scan;
    /**< Initial deployment mode, obstacle + robot forces apply. */
    const unsigned char SPF_MODE_DEPLOYMENT = 0;
    /**< Mode after network simulation, obstacle + cohesion forces apply. */
    const unsigned char SPF_MODE_REDEPLOYMENT = 1;
    /**< Mode in which this robot is going to replace another one. Obstacle + attractive forces apply. */
    const unsigned char SPF_MODE_REPLACEMENT = 2;
    unsigned char spf_mode_;
};

#endif