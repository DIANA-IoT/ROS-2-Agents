// Project Title: ROS2_AGENTS
// File: include/ros2_agents/visualization_class.hpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#ifndef __VISUALIZATION_CLASS_HPP__
#define __VISUALIZATION_CLASS_HPP__

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "defs.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"

/// @brief Class used to visualize relevant MWSN paremeters on RViz.
class Visualization
{
    private:
        size_t num_elements;
        typedef struct {
            visualization_msgs::msg::Marker bat_marker;
            visualization_msgs::msg::Marker text_marker;
        } robot_markers_t;

        robot_markers_t *marker_array;

        rclcpp::Clock::SharedPtr internal_clock_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

        std_msgs::msg::ColorRGBA double2Color(const double &v, const bool &is_gradient);
        void update_batMarker(visualization_msgs::msg::Marker *m, const double &b, std_msgs::msg::Header &h, geometry_msgs::msg::Pose &p, const std::string &ns);
        void update_textMarker(visualization_msgs::msg::Marker *m, std_msgs::msg::Header &h, const int16_t &id, geometry_msgs::msg::Pose &p ,const RoleTypes &r);

        #define BAT_MARKER_TYPE visualization_msgs::msg::Marker::CYLINDER
        #define BAT_MARKER_SCALE_AXIS 0.4
        #define TEXT_MARKER_SCALE_AXIS 0.25

    public:
        //! Explicit parametrized constructor
        /*!
            The constructor creates all the necessary objects and structures to hold and
            publish relevant information. 
            \param number_of_robots: Number of robots in the system.
            \param pub: A shared pointer to an already existing Marker publisher.
            \param ptr_to_clock: A shared pointer to RCLCPP clock.
            \return Nothing, but due to memory allocation checks, this constructor can trigger assertions and, thus, halt the execution.
        */
        explicit Visualization(size_t number_of_robots, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, rclcpp::Clock::SharedPtr ptr_to_clock);
        ~Visualization();

        //! Publishes all the stored data, so that the information displayed on RViz is updated.
        /*!
            No params or returned values. However, this function should not be called too frequently
            as it can overload with ease the system (due to copy/iterations through the RMW) or the network
            (published messages can be heavy).
        */
        void publish_markers();

        //! Updates the arguments that will be displayed on graphical and text forms.
        /*!
            The arguments passed will conform the displayed data on RViz.
            Right now, both text (general info) and graphical (battery status) are displayed.
            \param id: Id of the robot to update.
            \param b: Current battery level of the robot.
            \param p: Current position of the robot.
            \param r: Current robot role.
        */
        void update_args(const int16_t &id, const double &b, const geometry_msgs::msg::Point &p, const RoleTypes &r);
};

#endif