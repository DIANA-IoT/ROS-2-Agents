// Project Title: ROS2_AGENTS
// File: include/ros2_agents/consumption_model.hpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#ifndef __CONSUMPTION_MODEL_HPP__
#define __CONSUMPTION_MODEL_HPP__

#include <cmath>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros2_agents/defs.hpp"
#include "ros2_mwsn_msgs/msg/controller2_supervisor.hpp"
#include "ros2_mwsn_msgs/msg/supervisor2_controller.hpp"

/// @brief Consumption model for robot class.
class ConsumptionModel
{
private:
    /// @brief Supervisor2Controller callback for message /roboti/battery_drain.
    /// @param msg Supervisor2Controller message
    /*!
        It substracts a % of current battery level. Asserts both destination and
        message type are right.
        After the computation is done, it publishes absolute battery level.
    */
    void drain_callback(const ros2_mwsn_msgs::msg::Supervisor2Controller &msg)
    {
        assert(msg.destination == id);
        assert(msg.type == MESSAGE_RADIO_UPDATE);
        MessageRadioUpdateParameter temp;
        mutex_->lock();
        memcpy(&temp, &msg.data[0], sizeof(MessageRadioUpdateParameter));
        // Updates current percentage (absolute value)
        current_percentage -= temp.percentage;
        mutex_->unlock();
        publish_percentage();
    }
    /// @brief Function to publish current battery percentage and check if the battery exceeded critical level.
    void publish_percentage(void)
    {
        auto new_msg = ros2_mwsn_msgs::msg::Controller2Supervisor();
        new_msg.origin = id;
        new_msg.type = MESSAGE_SET_BATTERY;
        mutex_->lock();
        new_msg.data.resize(sizeof(MessageRadioUpdateParameter));
        memcpy(&new_msg.data[0], &current_percentage,
               sizeof(MessageRadioUpdateParameter));
        mutex_->unlock();
        battery_pub_->publish(std::move(new_msg));
        if (current_percentage <= BATTERY_CRITICAL_PER)
        {
            notifier_(current_percentage);
        }
    }
    /// @brief Callback as response to the publication timer timeout
    void publicacion_callback(void)
    {
        publish_percentage();
    }

    /// @brief Callback for /roboti/cmd_vel message.
    /// @param vel Twist message indicating differential speed.
    /*!
        This callback substracts a certain amont of battery depending on current
        different speed (vel param), the constant BATTERY_WASTE_PER_DISTANCE and
        time2update.
    */
    void vel_callback(const geometry_msgs::msg::Twist &vel)
    {
        mutex_->lock();
        /*  A possible improvement could imply substituting time2update
            by actual ROS time.
        */
        mov_inc = 0.0;
        mov_inc = hypot(vel.linear.x, vel.angular.z) * time2update;
        current_percentage -= 100.0 * (mov_inc * BATTERY_WASTE_PER_DISTANCE) / BATTERY_CAPACITY_MAH;
        total_movement += mov_inc;
        mutex_->unlock();
        if (current_percentage <= BATTERY_CRITICAL_PER)
        {
            notifier_(current_percentage);
        }
    }
    /* Internal variables: */
    int16_t id;
    double current_percentage;                    /* % (0.0-100.0) */
    const double capacity = BATTERY_CAPACITY_MAH; /* mAh */
    const double time2update = 0.5;               /* seconds, better if taken from ROS. */
    double total_movement;                        /* m */
    double mov_inc;
    /* ROS variables: */
    rclcpp::Node *parent_ptr;
    rclcpp::Publisher<ros2_mwsn_msgs::msg::Controller2Supervisor>::SharedPtr battery_pub_;
    rclcpp::TimerBase::SharedPtr publication_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Subscription<ros2_mwsn_msgs::msg::Supervisor2Controller>::SharedPtr battery_sub_;
    std::function<void(double)> notifier_;
    std::mutex *mutex_;

public:
    //! ConsumptionModel default constructor
    explicit ConsumptionModel(rclcpp::Node *parent, const int16_t &node_id)
    {
        parent_ptr = parent;
        id = node_id;
        current_percentage = 100.0;
        total_movement = 0.0;
        mutex_ = new std::mutex();
        auto qos = rclcpp::QoS(5);
        qos.best_effort();
        std::string topic = std::string("/robot") + std::to_string(id) + std::string("/") + vel_topic_;
        vel_sub_ = parent_ptr->create_subscription<geometry_msgs::msg::Twist>(topic,
                                                                              qos, std::bind(&ConsumptionModel::vel_callback, this, std::placeholders::_1));
        topic = std::string("/robot") + std::to_string(id) + std::string("/") + drain_topic_;
        battery_sub_ = parent_ptr->create_subscription<ros2_mwsn_msgs::msg::Supervisor2Controller>(topic, qos,
                                                                                                   std::bind(&ConsumptionModel::drain_callback, this, std::placeholders::_1));
        topic = std::string("/robot") + std::to_string(id) + std::string("/") + battery_topic_;
        battery_pub_ = parent_ptr->create_publisher<ros2_mwsn_msgs::msg::Controller2Supervisor>(topic, qos);
        publication_timer_ = rclcpp::create_timer(parent_ptr, parent_ptr->get_clock(),
                                                  std::chrono::seconds(10), std::bind(&ConsumptionModel::publicacion_callback, this));
        publication_timer_->cancel();
    }
    virtual ~ConsumptionModel() {}

    ConsumptionModel(ConsumptionModel &cm)
    {
        current_percentage = cm.current_percentage;
    }

    //! Method that returns battery percentage.
    /*!
        This method returns a double precision float containing
        current battery estimation.
        \return percentage in range [0.0,100.0]
    */
    double get_percentage(void) const
    {
        mutex_->lock();
        double copy = current_percentage;
        mutex_->unlock();
        if (current_percentage <= BATTERY_CRITICAL_PER)
            notifier_(current_percentage);
        return copy;
    }
    //! Simulate an initial battery level different than 100.0 %
    /*!
        This method is used to set an estimated battery level.
        Useful when non-ideal start conditions are going to be tested.
        \param level: Double precision float point representing the battery level to be set.
    */
    void set_initialPercentage(const double &level)
    {
        mutex_->lock();
        current_percentage = level;
        mutex_->unlock();
    }
    /// @brief Add the function passed as argument to notify when the critial level is reached.
    /// @param func a parent member function returning void and double argument
    void set_NotificationFunc(std::function<void(double)> func)
    {
        notifier_ = func;
    }
    /// @brief Enables a periodic publication of the current battery percentage.
    /// @param None.
    void start_publication(void)
    {
        mutex_->lock();
        if (current_percentage > BATTERY_CRITICAL_PER)
            publication_timer_->reset();
        mutex_->unlock();
    }
    /// @brief Disables periodic publication of battery percentage messages (unless a drain message is received)
    /// @param
    void stop_publication(void)
    {
        publication_timer_->cancel();
    }
};

#endif
