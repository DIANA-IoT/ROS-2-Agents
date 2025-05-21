// Project Title: ROS2_AGENTS
// File: include/ros2_agents/negotiation_class.hpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#ifndef __NEGOTIATION_CLASS_HPP__
#define __NEGOTIATION_CLASS_HPP__

#include "defs.hpp"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/point.hpp>
#include "ros2_mwsn_msgs/msg/controller2_supervisor.hpp"
#include "ros2_mwsn_msgs/msg/intra_network.hpp"
#include "ros2_mwsn_msgs/msg/position.hpp"
#include "ros2_agents/robot_logger.hpp"

namespace Negotiation
{
    /// @brief Calculates the utility of moving to somewhere
    /// @param battery: Double value representing battery level.
    /// @param points Vector of 3-D points (my position and the other position)
    /// @param my_robot My identifier in the vector points.
    /// @param robot_2help Robot in need identifier / index in the vector points.
    /// @param AP_id Index of the AP or depot in the vector points.
    /// @param w Willingness to interact with robot robot2_help.
    /// @param packets_per_agent Array that contains how much traffic each node is routing.
    /// @return The utility of moving to a certain point, as a correcting factor of the willingness.
    double calc_utility(const double &battery, const std::vector<geometry_msgs::msg::Point> &points, const int &my_robot, const int &robot_2help, int AP_id, double w, const std::vector<int> &packets_per_agent);
    /// @brief Calculates the willingness of the robot to move to a certain poit.
    /// @param battery Current battery level. A double in range [0.0, 100.0]
    /// @param goal_point 3-D point, it may represent the pose of another robot.
    /// @param my_point 3-D point, current robot location.
    /// @param w_h WH double value.
    /// @param w_h20 Wh20 double value.
    /// @return The willingness to interact represent as a double value in the range [-1, 1].
    double calc_willingness(const double &battery, const geometry_msgs::msg::Point &goal_point, const geometry_msgs::msg::Point &my_point, double &w_h, double &w_h20);

    /// @brief Base class for negotiation operations. Provides common functionality.
    class Negotiation
    {
    public:
        Negotiation();
        /// @brief Default constructor for Negotiation.
        /// @param ptr
        /// @param Id of the node that will negotiate.
        /// @param nrobots
        explicit Negotiation(rclcpp::Node *ptr, int16_t id, int16_t nrobots);
        /// @brief This method returns true if the robot needs help, constructing in that case a preallocated message.
        /// @param[in] point: 3-D point where the robot is located.
        /// @param[out] msg: A pointer to a MESSAGE_HELP_REQUEST if this function returns true. Nullptr otherwise.
        /// @return Boolean: true if help is required, and thus, a message requesting help needs to be publicated.
        /*!
            This method returns true if the robot needs help. This condition is met when its battery level falls below bl0, with
            bl0 being the minimum battery required to safely reach the depot point.
            When this happens, a MESSAGE_HELP_REQUEST will be allocated indicating the robot ID and its pose, with the aim
            of publishing over a IntraNetwork message.
            When this condition is not met, false is return and msg equals to nullptr, so no need to publish.
        */
        bool is_helpRequired(const geometry_msgs::msg::Point &point, ros2_mwsn_msgs::msg::IntraNetwork *&msg);
        virtual ~Negotiation();

    protected:
        geometry_msgs::msg::Point AP_loc;

    private:
        /// @brief Updates the battery required to reach a goal point.
        /// @param goal_pose 3-D point of the goal pose.
        /// @param my_pose 3-D point of current robot position.
        void update_blFromPose(const geometry_msgs::msg::Point &goal_pose, const geometry_msgs::msg::Point &my_pose);
    };
#if 0
/// @brief Negotiation class for the Supervisor.
/*!
    NegotiationAP is the class in charge of supervising the negotiations that are taking place in the network.
    It also serves as a "backup" in case a node did not negociate in time.
*/
class NegotiationAP : public Negotiation
{
public:
    /// @brief Explicit constructor for NegotiationAP
    /// @param parent A pointer to the RCLCPP-inherited parent class
    /// @param robot_id My ID.
    /// @param nrobots
    /*!
        TODO: Update this documentation
        Explicit constructor. Requires the ID of the logger that the negotiation will use plus a shared pointer to a ROS timer.
        As the, quite likely, most convenient way to give the nodes time to negotiate is to use a timer,
        it needs to be passed as a shared pointer. This class additionally requires to know the number of robots.
    */
    NegotiationAP(rclcpp::Node *parent, int16_t robot_id, int16_t nrobots);
    /// @brief Handler to manage IntraNetwork messages.
    /// @param in_msg Newly arrived message.
    /// @param logger Reference to a RCLCPP Logger, mainly for aiding debugging.
    /// @return A nullptr is no messages need to be sent in response. Otherwise, a pointer to Intranetwork message.
    ros2_mwsn_msgs::msg::IntraNetwork *handle_negotiation_state(const ros2_mwsn_msgs::msg::IntraNetwork &in_msg, rclcpp::Logger logger);
    /// @brief Handler after timer timeout.
    /// @param logger Reference to RCLCPP Logger, mainly for debugging reasons.
    /// @param rl Constant reference to the RobotLogger class, so that the negotation is fully aware of the state of each robot.
    /// @return A nullptr is no messages need to be sent in response. Otherwise, a pointer to Intranetwork message.
    ros2_mwsn_msgs::msg::IntraNetwork *handle_timer_timeout(rclcpp::Logger logger, std::vector<RobotLogger *> &rl);
    /// @brief Function that checks if there is a negotiation taking place.
    /// @return True is this robot is negotiating.
    bool is_Negotiating(void) const;

private:
    void negotiation_callback(const ros2_mwsn_msgs::msg::IntraNetwork &msg);
    /*!
        Enumeration describing the different states each node is phasing.
        1. IDLE -> Robot has enough battery to work normally.
        2. RECEIVED_REQ -> Robot is asking for a replacement.
        3. ASSIGN -> Asking robot now has a replacement.
        4. DONE ->  Done.
    */
    typedef enum
    {
        IDLE,
        RECEIVED_REQ,
        ASSIGN,
        DONE
    } ap_states_t;

    ap_states_t current_negotiation_state;

    bool am_I_in_charge; // Deprecated?

    rclcpp::TimerBase::SharedPtr negotiation_timer_;
    /*!
     Create a map with the robot w as key and the ID as value.
     By doing so, we benefit for the sorting feature of the map container.
     To find out which robot had we highest w, we index using iterators.
     The sorting algorithm puts the lowest key at the first index.
     */
    std::map<double, int16_t> robot_willingness;
    int16_t n_robots_;
    int16_t robot_id_;
};
#endif
#if 0
/// @brief NegotiationRobot is the class that will be executed in each robot locally.

class NegotiationRobot : public Negotiation
{
public:
    /// @brief Explicit constructor for NegotiationRobot<
    /// @param parent A pointer to the RCLCPP-inherited parent class
    /// @param robot_id My ID.
    /// @param nrobots
    NegotiationRobot(rclcpp::Node *parent, int16_t robot_id, int16_t nrobots);

    void answer_help_request(const MessageHelpParameter &help_request);
    /// @brief Clear parameters after reaching destination.
    void clear_afterDestination(void);
    /// @brief Returns the next location the robot should direct to.
    geometry_msgs::msg::Point get_nextLocation(const bool &is_depot) const;
    /// @brief Suggests the new role depending on bl and the existence of assigned targets.
    RoleTypes get_nextRole(void);
    /// @brief Returns true if battery is higher than bl1
    bool is_SPFPossible(void) const;
    /// @brief Updates the location of the robot I am replacing.
    /// @param des_point 3-D Point of the destination.
    void update_destination(const geometry_msgs::msg::Point &des_point);

private:
    void negotiation_callback(const ros2_mwsn_msgs::msg::IntraNetwork &msg);
};
#endif
}

#endif
