// Project Title: ROS2_AGENTS
// File: src/negotiation_class.cpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#include "ros2_agents/consumption_model.hpp"
#include "ros2_agents/negotiation_class.hpp"
#include "ros2_agents/spf_utilities.hpp"

using namespace std::placeholders;

namespace Negotiation
{
    Negotiation::Negotiation()
    {
    }

    Negotiation::~Negotiation()
    {
    }
    double calc_utility(const double &battery, const std::vector<geometry_msgs::msg::Point> &points, const int &my_robot, const int &robot_2help, int AP_id, double w, const std::vector<int> &packets_per_agent)
    {
        // Calculate the utility based on the distance the robot would have to go
        double b_c_AP, b_newpoint, b_np_AP, b_state;
        double utility = 0.0;
        // calculate how much battery I need to from the current to AP
        b_c_AP = 100.0 * spf_util::compute_point_distances(points[my_robot], points[AP_id]) * BATTERY_WASTE_PER_DISTANCE / BATTERY_CAPACITY_MAH;
        // calculate how much battery I need to go to the new point
        b_newpoint = 100.0 * spf_util::compute_point_distances(points[my_robot], points[robot_2help]) * BATTERY_WASTE_PER_DISTANCE / BATTERY_CAPACITY_MAH;
        // calculate how much battery I need to go from the new point to AP (or any collection point)
        b_np_AP = 100.0 * spf_util::compute_point_distances(points[robot_2help], points[AP_id]) * BATTERY_WASTE_PER_DISTANCE / BATTERY_CAPACITY_MAH;
        // Calculate what battery is left after robot moves to new point and AP
        b_state = 1.0 - (b_newpoint + b_np_AP - b_c_AP) / battery;

        if (packets_per_agent[my_robot] == 0 && packets_per_agent[robot_2help] == 0)
            utility = b_state;
        else
        {
            if (b_state >= 0.3)
            {
                double dpackets = (double)(packets_per_agent[robot_2help] - packets_per_agent[my_robot]) / (double)(packets_per_agent[robot_2help] + packets_per_agent[my_robot]);
                utility = b_state + dpackets;
            }
            else
                utility = -w;

        }
        return utility;
    }

    double calc_willingness(const double &battery, const geometry_msgs::msg::Point &goal_point, const geometry_msgs::msg::Point &my_point, double &w_h, double &w_h20)
    {
        double willingness = 0.0;
        double bl0 = 0.0;
        double bl1 = 0.0;
        double blh = 0.0;
        double blh20 = 0.0;

        /* Battery drain due to motor movement. */
        double distance = spf_util::compute_point_distances(my_point, goal_point);
        bl0 = spf_util::compute_critical_level(distance);
        bl1 = bl0 + 0.3 * bl0;
        blh = bl0 + 0.1 * bl0;
        blh20 = blh;

        /* Calculate the willingness. */
        if (battery <= bl0)
            willingness = -1.0;
        else
            willingness = (battery - bl1) / battery;

        w_h = (blh - bl1) / blh;
        w_h20 = (blh20 - bl1) / blh20;

        return willingness;
    }

    bool Negotiation::is_helpRequired(const geometry_msgs::msg::Point &point, ros2_mwsn_msgs::msg::IntraNetwork *&msg)
    {
        (void) point;
        (void) msg;
    return false;
    }
}
