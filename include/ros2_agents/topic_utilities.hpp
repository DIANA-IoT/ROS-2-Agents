// Project Title: ROS2_AGENTS
// File: include/ros2_agents/topic_utilities.hpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#ifndef __TOPIC_UTILITIES_HPP__
#define __TOPIC_UTILITIES_HPP__

#include <cstdio>
#include <vector>
#include <string>
#include "ros2_agents/defs.hpp"

namespace topic_util
{
	/// @brief Extracts robot ID from and frame_id or frame_prefix string
	/// @param[in] prefix string containing the frame_id
	/// @param[out] ok boolean that indicates the state of success of the operation
	/// @return identifier contained in prefix, unknown if boolean returns false.
	int16_t extractIDFromPrefix(const std::string &prefix, bool &ok);

	/// @brief Extract a set of namespace and topics and splits them into separate string vectors.
	/// @param orig: Original string containing namespaces + multiple strings.
	/// @param separator: Separator between topics.
	/// @return String vector as result of splitting the original topic.
	std::vector<std::string> extractNamespaceTopics(const std::string &orig,
		const char separator);

	/// @brief Boolean comparising if "robotXX" is found.
	/*! 
		By passing robot Id string, it returns true if it finds
		the pattern "robotXX", otherwise false. 
		\param topic: topic in "robotXX" fashion.
		\param id: Number to find in topic.
		\return True if id was found, false otherwise.
	*/

	/// @brief Get Robot number from topic with success checking.
	/// @param[in] topic: Topic in which to seek the ID.
	/// @param[out] id: Found ID. Not valid if this function returns false.
	/// @return True if found a numerical ID. False otherwise.
	bool getRobotNumber(const std::string &topic, int16_t &id);
	
	bool isOtherLocalizationTopic(const std::string &other, const std::string &myRobotId, 
		const LocType_t &locType);
	bool isWithinBounds(const int16_t &nid, const int16_t &cindex, const int16_t &n_for_index);
	bool isWithinBounds(const std::string &id, const int16_t &cindex, const int16_t &n_for_index);
}

#endif
