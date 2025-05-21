// Project Title: ROS2_AGENTS
// File: src/supervisor_node.cpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#include <functional>
#include <future>
#include <string>
#include <memory>
#include <chrono>
#include <csignal>
#include <algorithm>
#include <math.h>
#include <cassert>
#include <assert.h>
#include <cstring>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/serialization.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "ros2_agents/logger.hpp"
#include "ros2_agents/topic_utilities.hpp"
#include "ros2_agents/spf_utilities.hpp"
#include "ros2_agents/supervisor.hpp"
#include "ros2_agents/robot_controller.hpp"
#include "ros2_agents/robot_logger.hpp"
#include "ros2_mwsn_msgs/msg/controller2_supervisor.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"

/* Not the prettiest thing. */
static rclcpp::Clock::SharedPtr my_clock = nullptr;

using namespace std::chrono_literals;
using rclcpp::QoS;

Supervisor::Supervisor(const rclcpp::NodeOptions &node_options) : Node("Supervisor", node_options)
{
	this->declare_parameter<bool>("use_logger", false);
	this->declare_parameter<int>("negotiation_algorithm", 0);
	this->declare_parameter<int>("number_of_robots", 3);
	this->declare_parameter<std::string>("bag_path", "");
	this->declare_parameter<std::string>("type", "");

	this->get_parameter_or<bool>("use_logger", use_logger_, false);
	this->get_parameter<int>("negotiation_algorithm", algorithm_);
	this->get_parameter_or<int>("number_of_robots", n_robots_, 3);
	this->get_parameter_or<std::string>("bag_path", bag_path, "");
	std::string temp;
	this->get_parameter<std::string>("type", temp);

	if (temp == "SPF")
	{
		supervisor_type_ = SupervisorTypes::SUPERVISOR_SPF;
	}
	else if (temp == "Agents")
	{
		supervisor_type_ = SupervisorTypes::SUPERVISOR_AAGENTS;
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(), "Unknown type: SPF/Agents");
		rclcpp::shutdown();
	}
	assert(algorithm_ >= 0 && algorithm_ < 3);
	m = new std::mutex();
	cv = new std::condition_variable();

	data_mutex_ = new std::mutex();
	if (sem_init(&simulation_sem_, 0, 1) != 0)
	{
		RCLCPP_ERROR(this->get_logger(), "Error creating semaphore");
		rclcpp::shutdown();
	}

	my_clock = this->get_clock();
	if (use_logger_)
	{
		assert(!bag_path.empty());
		logger = new LoggerClass(bag_path);
	}
	/* Whether the Network simulation is being performed or not,
		initialize simulation time */
	simulation_time_ = 0.0;

	initialize_supervisor();
	simulation_thread_ = new std::thread(&Supervisor::simulation_thread, this);
	visualization_timer_ = rclcpp::create_timer(this, this->get_clock(), 2s, std::bind(&Supervisor::visualization_timeout_callback, this));
	start_time_ = std::chrono::steady_clock::now();
	// Disabled self-discharged battery for all nodes.
	//  update_radio_timer_ = rclcpp::create_timer(this, this->get_clock(), 60s, std::bind(&Supervisor::update_radio_callback, this));
}

Supervisor::~Supervisor()
{
}

bool Supervisor::check_end(int16_t dead_nodes)
{
	auto end_condition = false;
	// If more than 4 hours have elapsed, probably simulation is stuck.
	auto c_time_ = std::chrono::steady_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::hours>(c_time_ - start_time_);
	end_condition = (100.0 * dead_nodes / n_robots_) > SIMULATION_END_THRESHOLD_INT ||
					(elapsed > 8h);
	if (end_condition)
	{
		auto msg = std_msgs::msg::String();
		msg.data = "Status: Finished OK";
		sim_status_pub_->publish(msg);
		return true;
	}
	return false;
}

int16_t Supervisor::count_stationary(void)
{
	int16_t count = 0;
	data_mutex_->lock();
	for (size_t i = 0; i < RobotLoggerList_.size(); i++)
	{
		RobotLogger *ptr_to_robot = RobotLoggerList_.at(i);
		if (ptr_to_robot->get_isRobotStationary() || ptr_to_robot->get_isRobotStatic())
		{
			count++;
		}
	}
	data_mutex_->unlock();
	return count;
}

void Supervisor::initialize_performaceVectors(void)
{
	packets_sent_.assign(n_robots_, 0);
	own_packets_sent_.assign(n_robots_, 0);
	packets_rx_.assign(n_robots_, 0);
	own_packets_rx_.assign(n_robots_, 0);
	wasted_energy_.assign(n_robots_, 0.0);
}

void Supervisor::initialize_supervisor(void)
{
	alive_remaining_ = n_robots_;
	current_state_ = NetworkStates::IDLE;
	RobotLoggerList_ = std::vector<RobotLogger *>();
	RobotDeadList_.resize(n_robots_);
	/* Initially, every robot should be allive. */
	RobotDeadList_.assign(RobotDeadList_.size(), false);
	/* Performance and routing vectors. */
	msg_tx.resize(n_robots_);
	msg_tx.assign(n_robots_, false);
	packets_per_agent_.resize(n_robots_);
	giving_help_.resize(n_robots_ * n_robots_);
	requested_help_.resize(n_robots_);

	packets_sent_.resize(n_robots_);
	own_packets_sent_.resize(n_robots_);
	packets_rx_.resize(n_robots_);
	own_packets_rx_.resize(n_robots_);
	wasted_energy_.resize(n_robots_);
	net_mean_power_.resize(n_robots_);
	acc_packet_negotiation_.resize(n_robots_);
	acc_packet_total_.resize(n_robots_);
	msgs_exchanged_.resize(n_robots_);
	started_negotiation_.resize(n_robots_);
	tasks_assigned_.resize(n_robots_ * n_robots_);
	xpos_.resize(n_robots_ * n_robots_);
	ypos_.resize(n_robots_ * n_robots_);

	acc_packet_negotiation_.assign(n_robots_, 0);
	acc_packet_total_.assign(n_robots_, 0);
	msgs_exchanged_.assign(n_robots_, 0);
	started_negotiation_.assign(n_robots_, 0);
	tasks_assigned_.assign(n_robots_ * n_robots_, 0);
	xpos_.assign(n_robots_ * n_robots_, 0.0);
	ypos_.assign(n_robots_ * n_robots_, 0.0);
	initialize_performaceVectors();

	/* Negotiation vectors */
	w.resize(n_robots_);
	w_h.resize(n_robots_);
	w_h2.resize(n_robots_);
	previous_step_w.resize(n_robots_);
	previous_step_w.assign(1.0, n_robots_);

	auto qos = rclcpp::QoS(100);
	qos.reliable();
	robotCommander_pub_ = this->create_publisher<ros2_mwsn_msgs::msg::Supervisor2Controller>("/control_info", qos);
	sim_status_pub_ = this->create_publisher<std_msgs::msg::String>("/sim_status", 2);
	sim_control_pub_ = this->create_publisher<std_msgs::msg::String>("/sim_control", 2);

	killRobot_pub_ = this->create_publisher<std_msgs::msg::Int16>("/kill_robot", 10);

	marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/marker", 2 * n_robots_);
	visualization_ = new Visualization(n_robots_, marker_pub_, my_clock);
	auto qos_settings = QoS(100); // This could be changed to reliable if needed
	qos_settings.reliable();
	robotFeedbackSub_ = this->create_subscription<ros2_mwsn_msgs::msg::Controller2Supervisor>("/robot_feedback",
																							  qos_settings,
																							  std::bind(&Supervisor::robot_callback, this, std::placeholders::_1));
	position_sub_ = this->create_subscription<ros2_mwsn_msgs::msg::Position>("/position",
																			 QoS(2 * n_robots_).best_effort(),
																			 std::bind(&Supervisor::position_callback, this, std::placeholders::_1));

	for (int i = 0; i < n_robots_; i++)
	{
		std::string robotId_ = "/robot" + std::to_string(i);
		// RCLCPP_INFO(this->get_logger(), "Adding robot %s to the base", robotId_.c_str());
		RobotLogger *rb = new RobotLogger(robotId_);
		std::function<void(const geometry_msgs::msg::PoseWithCovarianceStamped)> ip_func = std::bind(&RobotLogger::initial_pose_callback, rb,
																									 std::placeholders::_1);
		robotIPSubList_.push_back(this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(robotId_ + "/initialpose",
																										   rclcpp::QoS(2).transient_local(), ip_func));
		RobotLoggerList_.push_back(rb);
		auto battery_qos = QoS(5).best_effort();
		battery_pub_.push_back(this->create_publisher<ros2_mwsn_msgs::msg::Supervisor2Controller>(robotId_ + "/battery_drain",
																								  battery_qos));
		std::function<void(const ros2_mwsn_msgs::msg::Controller2Supervisor)> feedback_func = std::bind(&RobotLogger::robot_feedback_callback, rb,
																										std::placeholders::_1);
		RobotBatSub_.push_back(this->create_subscription<ros2_mwsn_msgs::msg::Controller2Supervisor>(robotId_ + "/battery",
																									 battery_qos,
																									 feedback_func));
		rclcpp::sleep_for(5ms);
	}
}

bool Supervisor::is_simulation_stationary(void)
{
	auto stationary_count = count_stationary();
	// RCLCPP_INFO(this->get_logger(), "Stationary count %d", stationary_count);
	return ((100 * stationary_count) / alive_remaining_ > STATIONARY_THRESHOLD_INT);
}

void Supervisor::log_robot_metrics(void)
{
	/*
		What to do with the deployment time?
	*/
	for (size_t i = 0; i < RobotLoggerList_.size(); i++)
	{
		RobotLogger *ptr_to_robot = RobotLoggerList_[i];
		logger->write_entry_robots(ptr_to_robot->get_RobotNId(),
								   ptr_to_robot->get_currentPosition().x,
								   ptr_to_robot->get_currentPosition().y,
								   ptr_to_robot->get_currentPosition().z,
								   ptr_to_robot->get_batteryPercentage(),
								   ptr_to_robot->get_currentRole(),
								   simulation_time_);
	}
}

void Supervisor::simulation_thread(void)
{
	std::unique_lock<std::mutex> lk(*m);
	/* Deployment phase: Robots deploy using SPF. */
	start_simulation();
	/* Wait until robots have deployed and the simulation
		is stationary. */
	while (!is_simulation_stationary())
	{
		/*
			Wait on condition variable.
			Triggered when received Controller2Supervisor message
			with a new role.
		*/
		cv->wait(lk);
	}

	RCLCPP_INFO(this->get_logger(), "Simulation deployment completed");
	rclcpp::sleep_for(30s);
	/*
		Running phase: Robots deplete their batteries by traffic routing.
		Depending on the supervisor type, two simulations can be performed:
		Agents or SPF. */
	while (true)
	{
		send_static_msg();
		// LOGGING results
		if (use_logger_)
		{
			log_robot_metrics();
			save_performance();
			logger->write_entry_individual_packets(packets_per_agent_, simulation_time_);
		}
		// Perform routing simulation
		network_simulation();
		// if (use_logger_)
		// {
		// 	log_robot_metrics();
		// 	save_performance();
		// }
		rclcpp::sleep_for(1s);
		if (supervisor_type_ == SUPERVISOR_SPF)
		{
			rclcpp::sleep_for(10s);
		}
		if (supervisor_type_ == SUPERVISOR_AAGENTS)
		{
			/* Agents negotiation */
			perform_negotiation();
			rclcpp::sleep_for(10s);
			/* Wait until robots have reached their final positions and the
				simulation becomes stationary. */
			do
			{
				auto result = cv->wait_for(lk, std::chrono::seconds(45));
				std::string str;
				int n_moving = 0;
				for (int16_t i = 0; i < n_robots_; i++)
				{
					auto crole = RobotLoggerList_[i]->get_currentRole();
					if (crole == ROLE_RETRIEVING || crole == ROLE_ADJUST ||
						crole == ROLE_GOING_TO)
						n_moving++;
				}
				if (result == std::cv_status::timeout)
				{
					str = "Replacement timeout. N moving " +
						  std::to_string(n_moving) + "\n";
					logger->write_entry_negotiation_result(str);
					break;
				}
				else
				{
					if (n_moving == 0)
					{
						str = "Replacement done. No more robots are moving";
						logger->write_entry_negotiation_result(str);
						break;
					}
				}
			} while (true);
			/* After all moving agents have reached their target locations, the SPF
				is run to balance the network. */
		}
		/* Network balancing and readjustment. */
		send_end_negotiation_msg();
		RCLCPP_INFO(this->get_logger(), "Balancing the network");
		do
		{
			auto result = cv->wait_for(lk, std::chrono::seconds(30));
			std::string str;
			int n_moving = 0;
			for (int16_t i = 0; i < n_robots_; i++)
			{
				auto crole = RobotLoggerList_[i]->get_currentRole();
				if (crole == ROLE_RETRIEVING || crole == ROLE_DEPLOYING ||
					crole == ROLE_ADJUST || crole == ROLE_GOING_TO)
					n_moving++;
			}
			if (result == std::cv_status::timeout)
			{
				str = "Balancing timeout. N moving " +
						   std::to_string(n_moving) + "\n";
				logger->write_entry_negotiation_result(str);
				break;
			}
			else
			{
				if (n_moving == 0)
				{
					str = "Balancing done: no more robots were moving\n";
					logger->write_entry_negotiation_result(str);
					break;
				}
			}
		} while (true);
		auto dead = spf_util::count_dead(RobotDeadList_);
		alive_remaining_ = n_robots_ - dead;
		RCLCPP_INFO(this->get_logger(), "%d dead robots",
					dead);
		if (check_end(dead))
		{
			RCLCPP_INFO(this->get_logger(), "Simulation ended");
			rclcpp::sleep_for(1s);
			rclcpp::shutdown();
		}
	}
}

void Supervisor::visualization_timeout_callback(void)
{
	for (size_t index = 0; index < RobotLoggerList_.size(); index++)
	{
		visualization_->update_args(index,
									RobotLoggerList_[index]->get_batteryPercentage(),
									RobotLoggerList_[index]->get_currentPosition(),
									RobotLoggerList_[index]->get_currentRole());
	}
	visualization_->publish_markers();
}
