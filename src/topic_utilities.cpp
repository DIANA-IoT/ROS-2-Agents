// Project Title: ROS2_AGENTS
// File: src/topic_utilities.cpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#include "ros2_agents/topic_utilities.hpp"

namespace topic_util
{
	std::vector<std::string> extractNamespaceTopics(const std::string &orig,
		const char separator)
{
	std::vector<std::string> splitted;
	// if (orig[0] != '/')
	// {
	// 	return splitted;
	// }
	size_t startIndex = 0, endIndex = 0;
	for (size_t i = 0; i <= orig.size(); i++)
	{
		if (orig[i] == separator || i == orig.size())
		{
			endIndex = i;
			std::string temp = "/";
			temp.append(orig, startIndex, endIndex - startIndex);
			splitted.push_back(temp);
			startIndex = endIndex + 1;
		}
	}
	// This is quite improvable...
	if (splitted[0] == "/")
		splitted.erase(splitted.begin());
	return splitted;
}

int16_t extractIDFromPrefix(const std::string &prefix, bool &ok)
{
	auto sv = extractNamespaceTopics(prefix, '/');
	int16_t id = 0;
	ok = getRobotNumber(sv[0], id);
	return id;
}

bool getRobotNumber(const std::string &topic, int16_t &id)
{
	return (sscanf(topic.c_str(), "/robot%hi", &id) > 0);
}

bool isOtherLocalizationTopic(const std::string &other, const std::string &myRobotId,
	const LocType_t &locType)
{
	// If the topic is not an absolute topic, we won't check any further.
	if (other[0] != '/' || myRobotId[0] != '/')
	{
		return false;
	}
	std::vector<std::string> subTopics = extractNamespaceTopics(other, '/');
	if (subTopics.empty())
	{
		return false;
	}

	switch(locType)
	{
		case LocType_t::AMCL:
			return (subTopics[0] != myRobotId) && (subTopics[1] == ("/amcl_pose"));
		break;
		case LocType_t::Odometry:
			return (subTopics[0] != myRobotId) && (subTopics[1] == ("/odom"));
		break;
		case LocType_t::GroundTruth:
			return (subTopics[0] != myRobotId) && (subTopics[1] == ("/pose_ground_truth"));
		break;
	}
	return false;
}

bool isWithinBounds(const int16_t &nid, const int16_t &cindex, const int16_t &n_for_index)
{
	bool is_above_minimum = nid >= (cindex * n_for_index);
	bool is_below_maximum = nid < ((cindex + 1) * n_for_index);
	return is_above_minimum && is_below_maximum;
}

bool isWithinBounds(const std::string &id, const int16_t &cindex, const int16_t &n_for_index)
{
	int16_t nid = 0;
	if (!getRobotNumber(id, nid))
	{
		return false;
	}
	return isWithinBounds(nid, cindex, n_for_index);
}

}
