// Project Title: ROS2_AGENTS
// File: include/ros2_agents/defs.hpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#ifndef __DEFS_HPP__
#define __DEFS_HPP__

#include <cstdint>
#include <string>

#define MSG_DESTINATION_BROADCAST -1
#define BATTERY_CAPACITY_MAH 3000
#define ROBOT_INITIAL_BATTERY BATTERY_CAPACITY_MAH
#define BATTERY_CRITICAL_PER 5.0F						   /**< Critical battery level (%), below this, robots are considered dead */
#define BATTERY_WASTE_PER_DISTANCE (200.0 / (0.5 * 3600)) /**< motor current / (speed * 3600 s) [mAh/m] */
/*!
	Hard constant, should be kept?
*/
#define ROBOT_WASTE_PER_PACKET 0.000138
#define ROBOT_DUTYCYCLE_WASTE 0.15		/**< mAh drained of network operation. */
#define ROBOT_PACKET_RATE_HOUR 360.0	/**< */
#define STATIONARY_THRESHOLD_INT 80		/**< Integer percentage of alive robots that should be stationary to consider stationary state*/
#define SIMULATION_END_THRESHOLD_INT 70 /**< Integer threshold percentage to end simulation */
#define SIMULATION_NUM_ITERATIONS 20
#define SIMULATION_MIN_INTERVAL 10
#define RADIO_COVERAGE_SPF 25 /**< Distance (m) for radio coverage. */
#define ROBOT_WASTE_INCREASE_DISTANCE 0.1
/** Routing strategies  */
#define NO_STRATEGY 0
#define ONLY_IF_CLOSER 1

#define GRID_CELD 0.2 // Scaling factor?
#define ROBOT_COVERAGE 16
#define MAP_SIZE 150.0		 // meters
#define MIN_WIFI_DISTANCE 15 // meters

#define NUM_MAX_RETX 1
#define PROB_LOSS 0.0
#define COCIENTE_PRXMIN_K 0.01			   // ??
#define DISTANCE_OSCILLATION_THRESHOLD 0.3 // meters

enum class LocType_t
{
	AMCL,
	GroundTruth,
	Odometry
};

const std::string battery_topic_ = "battery";
const std::string drain_topic_ = "battery_drain";
const std::string initial_pose_topic_ = "initialpose";
const std::string scan_topic_ = "scan";
const std::string vel_topic_ = "cmd_vel";

/** @brief The enumeration of the message types used in the different control topics.
 *
 */
typedef enum
{
	MESSAGE_RUN,
	MESSAGE_SET_ROLE,
	MESSAGE_SET_DESTINATION,
	MESSAGE_SET_BATTERY,	  /**<Initial battery message */
	MESSAGE_HELP_REQUEST,	  /**<Message indicating that a robot needs help */
	MESSAGE_HELP_RESPONSE,	  /**<Reply to the previous msg. */
	MESSAGE_DO_ASSIGN,		  /**<Robot in neeed should assign a replacing robot. */
	MESSAGE_HELP_ASSIGN,	  /**<Confirmation to assign a robot. */
	MESSAGE_DONE_NEGOTIATION, /**<No more negotiations, robots need to relocate. */
	MESSAGE_ROBOT_DIED,		  /**<Message indicating a robot has died */
	MESSAGE_RADIO_UPDATE,	  /**<Message to update radio consumption. */
							  /* Add message types */
} MessageTypes;

typedef enum
{
	CONTROLLER_TYPE_SPF,
	CONTROLLER_TYPE_AAGENTS,
} ControllerTypes;

typedef enum
{
	SUPERVISOR_SPF,
	SUPERVISOR_AAGENTS
} SupervisorTypes;

/// @brief Possible roles of each robot.
/*!
	\typedef RoleTypes
*/
typedef enum
{
	ROLE_DEPLOYING,	 /**<Robot deploying by SPF. */
	ROLE_ADJUST,	 /**<Robot performing network balancing */
	ROLE_GOING_TO,	 /**<Robot moving to a determined destination. */
	ROLE_STATIONARY, /**<Robot not moving but computing forces. */
	ROLE_STATIC,	 /**<Robot neither computing forces nor moving. */
	ROLE_RETRIEVING, /**<Robot with low battery returning to depot. */
	ROLE_DEAD,		 /**<Robot not able to use the communications anymore due to insufficient battery. */
	ROLE_AP,		 /**<Robot connected to a power supply, it does not move. */
} RoleTypes;

/// @brief Message to set Robot Role. Used only in Supervisor to Controller messages.
/*!
	\param role: Role that the robot should take.
*/
typedef struct
{
	RoleTypes role;
} MessageRoleSetParameter;

/// @brief Message that shows 2-D Coordinates.
/*!
	\param x,y: X and Y coordinates of a point.
*/
typedef struct
{
	double x;
	double y;
} Message2DPositionParameter;

/// @brief Message that shows robot in need of help.
/*!
	This message could be originated in two scenarios:
	1. After a robot has reached a critical battery level.
	2. To mark the coordinates of a robot in need to an already assigned replacing robot.
	In case 1: broadcast message, 2: a directed message to the chosen robot.
	\param origin: Identifier of the robot asking for help.
	\param x, y: 2-D coordinates of the robot.
*/
typedef struct
{
	int16_t origin;
	double x;
	double y;
} MessageHelpParameter;

/// @brief Message that shows the willingness of interact.
/*!
	Message sent in response to a help request.
	Only positive willingness should be seen through this message.
	w belongs to the range [-1,1].
	Non-broadcast message.
	\param origin: The robot replying to the help request.
	\param w: Willingness of the previous parameter to replace another robot.
*/
typedef struct
{
	int16_t origin;
	double w;
} MessageWillingnessParameter;

/// @brief  Indicates a robot has depleted its battery.
/*!
	A similar message to MQTT's last-will message. It is sent by
	an about-to-die robot before its transceiver shuts down due to
	insufficient battery.
	By its nature, it is a broadcast message.
	\param robot: Identifier of the depleted robot.
*/
typedef struct
{
	int16_t robot;
} MessageDiedRobotParameter;

/// @brief Shows how much battery has drained due to transceiver usage.
/*!
	Estimation of how much battery has been used by the radio transceiver
	between consecutive calls of this message.
	Non-broadcast message.
	\param percentage: Percentage of battery drained between this message and the previous one.
*/
typedef struct
{
	double percentage;
} MessageRadioUpdateParameter;

enum class PacketTypes
{
	TX,		/**< Transmission packet, from node i to j. */
	OWN_TX, /**< Transmission packet, from node i to i. */
	RX,		/**< RX packet from node k to l. */
	OWN_RX	/**< RX packet from node k to k. */
};

#endif