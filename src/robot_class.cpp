// Project Title: ROS2_AGENTS
// File: src/robot_class.cpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#include <cmath>
#include "ros2_agents/robot_class.hpp"
#include "ros2_agents/topic_utilities.hpp"

static geometry_msgs::msg::Point depot_point;

RobotClass::RobotClass(rclcpp::Node *parent, const std::string &id, const ControllerTypes &type)
{
	parent_ptr = parent;
	_whoAmI = id;
	auto ok = true;
	_numWhoAmI = topic_util::extractIDFromPrefix(id, ok);
	_controller = type;
	_current_role = ROLE_STATIC;
	(void)topic_util::getRobotNumber(id, _numWhoAmI);
	std::string temp;
	parent_ptr->get_parameter_or<std::string>("localization_type", temp, "");
	if (temp == "AMCL")
	{
		_localization = LocType_t::AMCL;
	}
	else if (temp == "GroundTruth")
	{
		_localization = LocType_t::GroundTruth;
	}
	else if (temp == "Odometry")
	{
		_localization = LocType_t::Odometry;
	}
	else
	{
		RCLCPP_FATAL(parent_ptr->get_logger(), "Unknown localization type. Aborting");
		rclcpp::shutdown();
	}
	depot_point.x = 0.0;
	depot_point.y = 0.0;
}

RobotClass::~RobotClass()
{
}

void RobotClass::battery_CriticalCallback(const double &value)
{
	RCLCPP_INFO(parent_ptr->get_logger(), "Robot %d notified critical battery value %f",
				_numWhoAmI, value);
	_current_role = ROLE_DEAD;
	auto r = RoleTypes::ROLE_DEAD;
	notify_RoleChange(r);
	spf_->stop();
}

void RobotClass::initialize(void)
{
	consumption_model_ = new ConsumptionModel(parent_ptr, _numWhoAmI);
	consumption_model_->set_NotificationFunc(std::bind(&RobotClass::battery_CriticalCallback, this, _1));
	auto control_qos = rclcpp::QoS(20);
	control_qos.reliable();
	controller_sub_ = parent_ptr->create_subscription<ros2_mwsn_msgs::msg::Supervisor2Controller>("/control_info",
																								  control_qos, std::bind(&RobotClass::supervisor_msg_callback, this, std::placeholders::_1));
	control_qos.reliable();
	control_qos.keep_last(20);
	robot_pub_ = parent_ptr->create_publisher<ros2_mwsn_msgs::msg::Controller2Supervisor>("/robot_feedback", control_qos);
	if (_controller == ControllerTypes::CONTROLLER_TYPE_SPF)
	{
		// RCLCPP_INFO(parent_ptr->get_logger(), "Chosen SPF");
		spf_ = new SPF(parent_ptr, _numWhoAmI, _localization);
		spf_->set_NotificationFunc(std::bind(&RobotClass::spf_role_callback, this, _1));
	}
	else if (_controller == ControllerTypes::CONTROLLER_TYPE_AAGENTS)
	{
		// RCLCPP_INFO(parent_ptr->get_logger(), "Chosen Agents");
		spf_ = new SPF(parent_ptr, _numWhoAmI, _localization);
		spf_->set_NotificationFunc(std::bind(&RobotClass::spf_role_callback, this, _1));
	}
	else
	{
		RCLCPP_INFO(parent_ptr->get_logger(), "Unknown controller");
	}

	/* Create subscription for initial pose. */
	std::function<void(const geometry_msgs::msg::PoseWithCovarianceStamped)> ip_func_ = std::bind(&RobotClass::initial_pose_callback, this, _1);
	control_qos.keep_last(2);
	control_qos.reliable();
	control_qos.transient_local();
	std::string topic = std::string("/robot") + std::to_string(_numWhoAmI) + std::string("/") + initial_pose_topic_;
	initialpose_sub_ = parent_ptr->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(topic, control_qos, ip_func_);
}

void RobotClass::initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
{
	_my_initial_pose = msg.pose.pose;
	if (_controller == ControllerTypes::CONTROLLER_TYPE_SPF)
	{
		spf_->set_InitialPose(msg);
	}
}

void RobotClass::notify_RoleChange(const RoleTypes &role)
{
	RCLCPP_DEBUG(parent_ptr->get_logger(),
				 "Robot %d changing role from %d to %d",
				 _numWhoAmI, _current_role, role);
	_current_role = role;
	auto msg = ros2_mwsn_msgs::msg::Controller2Supervisor();
	msg.origin = _numWhoAmI;
	msg.type = MESSAGE_SET_ROLE;
	unsigned char value = (unsigned char)role;
	msg.data.insert(msg.data.begin(), value);
	robot_pub_->publish(std::move(msg));
	switch (role)
	{
	case ROLE_ADJUST:
	case ROLE_DEPLOYING:
	case ROLE_GOING_TO:
	case ROLE_RETRIEVING:
		consumption_model_->start_publication();
		break;
	case ROLE_AP:
	case ROLE_DEAD:
	case ROLE_STATIC:
	case ROLE_STATIONARY:
		consumption_model_->stop_publication();
		break;
	}
}

RoleTypes RobotClass::set_newRoleAgents(const RoleTypes &recv_role)
{
	RoleTypes new_role = _current_role;
	// RCLCPP_INFO(parent_ptr->get_logger(),
	// 					"Robot %d received role %d while in %d",
	// 					_numWhoAmI, recv_role, _current_role);
	switch (_current_role)
	{
	case ROLE_DEPLOYING:
	{
		if (recv_role == ROLE_STATIC)
		{
			new_role = ROLE_STATIC;
		}
		else
		{
			RCLCPP_INFO(parent_ptr->get_logger(),
						"Robot %d received role %d while in ROLE_DEPLOYING",
						_numWhoAmI, recv_role);
		}
	}
	break;
	case ROLE_ADJUST:
	{
		if (recv_role == ROLE_STATIC)
		{
			new_role = ROLE_STATIC;
		}
	}
	break;
	case ROLE_GOING_TO:
	{
		if (recv_role == ROLE_STATIC)
		{
			new_role = ROLE_STATIC;
		}
	}
	break;
	case ROLE_STATIONARY:
	{
		if (recv_role == ROLE_DEAD)
		{
			new_role = ROLE_DEAD;
			spf_->stop();
		}
		else if (recv_role == ROLE_STATIC)
		{
			new_role = ROLE_STATIC;
		}
		else if (recv_role == ROLE_AP)
		{
			new_role = ROLE_AP;
			spf_->stop();
		}
		else if (recv_role == ROLE_RETRIEVING)
		{
			new_role = ROLE_RETRIEVING;
			spf_->set_Attractor(depot_point);
		}
	}
	break;
	case ROLE_STATIC:
	{
		if (recv_role == ROLE_RETRIEVING)
		{
			new_role = ROLE_RETRIEVING;
			spf_->set_Attractor(depot_point);
		}
		else if (recv_role == ROLE_AP)
		{
			new_role = ROLE_AP;
			spf_->stop();
		}
	}
	break;
	case ROLE_RETRIEVING:
	case ROLE_DEAD:
	case ROLE_AP:
		break;
	default:
		break;
	}
	return new_role;
}

RoleTypes RobotClass::set_newRoleSPF(const RoleTypes &recv_role)
{
	RoleTypes new_role = _current_role;
	switch (_current_role)
	{
	case ROLE_DEPLOYING:
	{
		if (recv_role == ROLE_STATIC)
		{
			new_role = ROLE_STATIC;
		}
		else
		{
			RCLCPP_INFO(parent_ptr->get_logger(),
						"Robot %d received role %d while in ROLE_DEPLOYING",
						_numWhoAmI, recv_role);
		}
	}
	break;
	case ROLE_ADJUST:
	{
	}
	break;
	case ROLE_GOING_TO:
	{
	}
	break;
	case ROLE_STATIONARY:
	{
		if (recv_role == ROLE_DEAD)
		{
			new_role = ROLE_DEAD;
			spf_->stop();
		}
		else if (recv_role == ROLE_STATIC)
		{
			new_role = ROLE_STATIC;
		}
		else if (recv_role == ROLE_AP)
		{
			new_role = ROLE_AP;
			spf_->stop();
		}
	}
	break;
	case ROLE_STATIC:
	{
		if (recv_role == ROLE_RETRIEVING)
		{
			new_role = ROLE_RETRIEVING;
		}
		else if (recv_role == ROLE_AP)
		{
			new_role = ROLE_AP;
			spf_->stop();
		}
	}
	break;
	case ROLE_RETRIEVING:
	case ROLE_DEAD:
	case ROLE_AP:
		break;
	default:
		break;
	}
	return new_role;
}

void RobotClass::spf_role_callback(const RoleTypes &role)
{
	RCLCPP_DEBUG(parent_ptr->get_logger(), "Robot %d notified SPF role change to %d", _numWhoAmI, role);
	notify_RoleChange(role);
}

void RobotClass::supervisor_msg_callback(const ros2_mwsn_msgs::msg::Supervisor2Controller &msg)
{
	RoleTypes new_role = _current_role;
	/* Firstly, we should check if am the receiver (or the message was broadcasted (< 0)). */
	bool message_to_me = (msg.destination >= 0 && msg.destination == _numWhoAmI) ||
						 (msg.destination == MSG_DESTINATION_BROADCAST);
	if (!message_to_me)
		return;

	switch (msg.type)
	{
	case MessageTypes::MESSAGE_RUN:
	{
		/* Robots should only deploy if they have a reason to do so. If they are dead, AP or retrieving,
			they should ignore this call. Any other cases should not happen.*/
		if (_current_role != ROLE_STATIC || _current_role == ROLE_AP)
		{
			return;
		}
		assert(msg.data.size() >= sizeof(MessageRoleSetParameter));
		memcpy(&new_role, &msg.data[0], sizeof(new_role));
		spf_->resume();
		break;
	}
	case MessageTypes::MESSAGE_SET_ROLE:
	{
		if (_current_role == ROLE_DEAD || _current_role == ROLE_AP)
		{
			return;
		}
		assert(msg.data.size() >= sizeof(MessageRoleSetParameter));
		MessageRoleSetParameter recv_role;
		memcpy(&recv_role, &msg.data[0], sizeof(recv_role));
		if (_controller == ControllerTypes::CONTROLLER_TYPE_SPF)
		{
			new_role = set_newRoleSPF(recv_role.role);
		}
		else if (_controller == ControllerTypes::CONTROLLER_TYPE_AAGENTS)
		{
			new_role = set_newRoleAgents(recv_role.role);
		}
		break;
	}
	case MessageTypes::MESSAGE_SET_DESTINATION:
	{
		if (_controller != ControllerTypes::CONTROLLER_TYPE_AAGENTS)
		{
			return;
		}
		if (_current_role == ROLE_DEAD || _current_role == ROLE_AP)
		{
			return;
		}
		assert(msg.data.size() == sizeof(MessageHelpParameter));
		new_role = ROLE_GOING_TO;
		MessageHelpParameter payload;
		memcpy(&payload, &msg.data[0], sizeof(MessageHelpParameter));
		auto tmp = geometry_msgs::msg::Point();
		tmp.x = payload.x;
		tmp.y = payload.y;
		RCLCPP_INFO(parent_ptr->get_logger(), "Robot %d going to replace %d, my pos %f %f, dest pos %f %f",
					_numWhoAmI, payload.origin, spf_->get_Pose().x,
					spf_->get_Pose().y,
					tmp.x, tmp.y);
		spf_->set_Attractor(tmp);
		break;
	}
	case MessageTypes::MESSAGE_SET_BATTERY:
	{
		double value;
		memcpy(&value, &msg.data[0], sizeof(value));
		consumption_model_->set_initialPercentage(value);
		break;
	}
	case MessageTypes::MESSAGE_DONE_NEGOTIATION:
	{
		if (_current_role == ROLE_DEAD)
			return;
		if (_controller != CONTROLLER_TYPE_SPF && _controller != CONTROLLER_TYPE_AAGENTS)
		{
			return;
		}
		if (_current_role == ROLE_STATIC || _current_role == ROLE_STATIONARY)
		{
			new_role = ROLE_ADJUST;
		}
		break;
	}
	case MESSAGE_ROBOT_DIED:
	{
		if (_current_role == ROLE_DEAD)
			return;
		assert(msg.data.size() == sizeof(int16_t));
		MessageDiedRobotParameter p;
		memcpy(&p, &msg.data[0], sizeof(p));
		spf_->set_DeadRobot(p.robot);
		break;
	}
	default:
		RCLCPP_INFO(parent_ptr->get_logger(), "Received supervisor msg %d",
					msg.type);
		break;
	}
	if (new_role != _current_role)
	{
		notify_RoleChange(new_role);
		spf_->set_Role(new_role);
	}
}
