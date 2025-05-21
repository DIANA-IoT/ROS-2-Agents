// Project Title: ROS2_AGENTS
// File: src/visualization_class.cpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#include "ros2_agents/defs.hpp"
#include "ros2_agents/visualization_class.hpp"

static const char *Role2str[] = 
{ 
	"Deploying",
	"Going to",
	"Gateway",
	"Stationary",
	"Retrieving",
	"Dead",
};

Visualization::Visualization(size_t number_of_robots, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, rclcpp::Clock::SharedPtr ptr_to_clock)
{
    marker_pub_ = pub;
    marker_array = new robot_markers_t[number_of_robots];
    assert(marker_array != nullptr);
    num_elements = number_of_robots;
    internal_clock_ = ptr_to_clock;
}

Visualization::~Visualization()
{
    delete marker_array;
    marker_pub_.reset();
}

std_msgs::msg::ColorRGBA Visualization::double2Color(const double &v, const bool &is_gradient)
{
    std_msgs::msg::ColorRGBA c;
    if (is_gradient)
    {
        c.a = 1.0; /* Alpha parameter: 0 is transparent. */
        c.b = 0.0;
        if (v < 50.0) {
            c.r = 1.0;
            c.g = v / 100.0;
        } else {
            c.g = 1.0;
            c.r = 2.0 - 2.0 * (v/100.0);
        }
    }
    return c;
}

void Visualization::publish_markers()
{
    for (int16_t i = 0; i < (int16_t) num_elements; i++)
    {
        marker_array[i].bat_marker.header.stamp = internal_clock_->now();
        marker_pub_->publish(marker_array[i].bat_marker);
        marker_array[i].text_marker.header.stamp = internal_clock_->now();
        marker_pub_->publish(marker_array[i].text_marker);
    }
}

void Visualization::update_args(const int16_t &id, const double &b, const geometry_msgs::msg::Point &p, const RoleTypes &r)
{
    /* Basic bound checks. */
    assert(id < num_elements && id >= 0);
    std::string ns_ = "robot" + std::to_string(id);
    std_msgs::msg::Header marker_header;
    marker_header.stamp = internal_clock_->now();
    marker_header.frame_id = ns_ + "/odom";
    geometry_msgs::msg::Pose marker_pose;
    marker_pose.position.x = p.x;
    marker_pose.position.y = p.y;
    marker_pose.position.z += 0.1;
    /* Update battery marker. */
    update_batMarker(&marker_array[id].bat_marker, b, marker_header, marker_pose, ns_);
    /* Update text marker. */
    marker_pose.position.z += 1.2;
    update_textMarker(&marker_array[id].text_marker, marker_header, id, marker_pose, r);
}

void Visualization::update_batMarker(visualization_msgs::msg::Marker *m, const double &b, std_msgs::msg::Header &h, geometry_msgs::msg::Pose &p, const std::string &ns)
{
    m->header = h;
    m->type = visualization_msgs::msg::Marker::CYLINDER;
    m->id = 0;
    m->action = visualization_msgs::msg::Marker::ADD;
    m->ns = ns + "/battery";
    p.orientation.w = 1.0;
    m->pose = p;
    m->scale.x = BAT_MARKER_SCALE_AXIS;
    m->scale.y = BAT_MARKER_SCALE_AXIS;
    m->scale.z = BAT_MARKER_SCALE_AXIS;
    m->color = double2Color(b, true);
}

void Visualization::update_textMarker(visualization_msgs::msg::Marker *m, std_msgs::msg::Header &h, const int16_t &id, geometry_msgs::msg::Pose &p ,const RoleTypes &r)
{
    m->header = h;
    m->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m->id = 0;
    m->action = visualization_msgs::msg::Marker::ADD;
    m->ns = "robot" + std::to_string(id) + "/text";
    m->pose = p;
    m->scale.x = TEXT_MARKER_SCALE_AXIS;
    m->scale.y = TEXT_MARKER_SCALE_AXIS;
    m->scale.z = TEXT_MARKER_SCALE_AXIS;
    m->color.a = 1.0;
    m->color.r = 0.0;
    std::string str;
    str = "Robot " + std::to_string(id) + " " + std::to_string(p.position.x) + " m, " + std::to_string(p.position.y) + " m. ";
    str += "Role: " + std::string(Role2str[r]);
    m->text = str;
}