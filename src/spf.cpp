// Project Title: ROS2_AGENTS
// File: src/spf.cpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#include "rclcpp/serialization.hpp"
#include "ros2_agents/spf.hpp"
#include "ros2_agents/spf_utilities.hpp"

#include <random>

using namespace std::chrono_literals;

SPF::SPF(rclcpp::Node *parent, const int16_t &id, const LocType_t &localization)
{
    _numWhoAmI = id;
    parent_ptr = parent;

    parent_ptr->get_parameter_or<int>("total_robots", number_of_robots_, 100);
    parent_ptr->get_parameter_or<float>("max_vel", _max_speed, 0.22);
    parent_ptr->get_parameter_or<float>("laser_min_range", laser_min_range, 0.2);
    parent_ptr->get_parameter_or<float>("laser_max_range", laser_max_range, 3.5);
    parent_ptr->get_parameter_or<float>("forces.repulsion_sigma_robot", repulsion_sigma_robot_,
                                        0.5);
    parent_ptr->get_parameter_or<float>("forces.repulsion_n_robot", repulsion_n_robot_, 0.2);
    parent_ptr->get_parameter_or<float>("forces.repulsion_sigma_object", repulsion_sigma_object_,
                                        0.5);
    parent_ptr->get_parameter_or<float>("forces.repulsion_n_object", repulsion_n_object_, 0.2);
    parent_ptr->get_parameter_or<float>("forces.attraction_k1", attractor_k1_, 1.0);
    parent_ptr->get_parameter_or<float>("forces.attraction_k2", attractor_k2_, 1.0);
    parent_ptr->get_parameter_or<float>("forces.cohesion_alpha", cohesion_alpha_, 5.0);
    parent_ptr->get_parameter_or<int>("force_update_ms", force_update_, 500);
    parent_ptr->get_parameter_or<float>("_movement_threshold", _movement_threshold_,
                                        0.2);
    parent_ptr->get_parameter_or<float>("noise_k", noise_k, 1.0);
    _robotIsDeadList_.resize(number_of_robots_);
    _robotPos_.resize(number_of_robots_);
    _robotPosLoss_.resize(number_of_robots_);
    _robotPosses_ = new MessageContainer<geometry_msgs::msg::Point>(10);
    _robotYaw_.resize(number_of_robots_);
    /* Initially, every robot should be allive. */
    _robotIsDeadList_.assign(_robotIsDeadList_.size(), false);

    current_role_ = ROLE_STATIONARY;
    is_first_deploy = true;

    auto qos_settings = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_settings.best_effort(); /* Set RELIABILITY_QOS_POLICY to BEST_EFFORT to match hlds node policy. */
    std::function<void(const sensor_msgs::msg::LaserScan)> scan_func_ = std::bind(&SPF::laser_callback, this, _1);
    std::string topic = std::string("/robot") + std::to_string(_numWhoAmI) + std::string("/") + scan_topic_;
    _robotScanSub_ = parent_ptr->create_subscription<sensor_msgs::msg::LaserScan>(topic, qos_settings, scan_func_);
    topic = std::string("/robot") + std::to_string(_numWhoAmI) + std::string("/") + vel_topic_;
    _robotVelPub_ = parent_ptr->create_publisher<geometry_msgs::msg::Twist>(topic, qos_settings.reliable());
    auto position_qos = rclcpp::QoS(rclcpp::KeepLast(100));
    position_qos.best_effort();
    _robotLocPub_ = parent_ptr->create_publisher<ros2_mwsn_msgs::msg::Position>("/position", position_qos);
    _robotAbsPoseSub_ = parent_ptr->create_subscription<ros2_mwsn_msgs::msg::Position>("/position", position_qos,
                                                                                       std::bind(&SPF::robotsPose_callback, this, _1));
    /* Create each robot relative position subscription. Not sure if this is valid when using AMCL. */
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback;
    std::string loc;
    std::string topic_type;
    _localization = localization;
    switch (localization)
    {
    case LocType_t::AMCL:
        // callback = std::bind(&RobotController::amcl_sub_callback, this, _1, robotId);
        // topic_type = "geometry_msgs/msg/PoseWithCovarianceStamped";
        rclcpp::exceptions::UnimplementedError("AMCL localization is not implemented");
        break;
    case LocType_t::GroundTruth:
        loc = std::string("/robot") + std::to_string(_numWhoAmI) + std::string("/pose_ground_truth");
        callback = std::bind(&SPF::odom_sub_callback, this, _1);
        topic_type = "nav_msgs/msg/Odometry";
        break;
    case LocType_t::Odometry:
        loc = std::string("/robot") + std::to_string(_numWhoAmI) + std::string("/odom");
        callback = std::bind(&SPF::odom_sub_callback, this, _1);
        topic_type = "nav_msgs/msg/Odometry";
        break;
    default:
        rclcpp::exceptions::UnimplementedError("Localization not implemented");
        break;
    }
    std::string temp;
    parent_ptr->get_parameter_or<std::string>("type", temp, "");
    if (temp == "SPF")
    {
        is_agents_ = false;
    }
    else if (temp == "Agents")
    {
        is_agents_ = true;
    }
    else
    {
        RCLCPP_FATAL(parent_ptr->get_logger(), "Unknown controller type. Aborting");
        rclcpp::shutdown();
    }
    _robotPoseSub_ = parent_ptr->create_generic_subscription(loc, topic_type, rclcpp::QoS(rclcpp::KeepLast(10)), callback);
    force_update_timer_ = rclcpp::create_timer(parent_ptr, parent_ptr->get_clock(),
                                               std::chrono::milliseconds(force_update_), std::bind(&SPF::force_callback, this));
    // Remain in cancelled state until explicitely the SPF needs to be executed.
    force_update_timer_->cancel();
    spf_mode_ = SPF_MODE_DEPLOYMENT;
}

SPF::~SPF()
{
}

void SPF::compute_attractive_forces(double &force_x, double &force_y)
{
    double distanceToAttractor = spf_util::compute_point_distances(_attractor_points_[0],
                                                                   _my_pose);
    double angleToAttractor = spf_util::compute_point_angles(_attractor_points_[0],
                                                             _my_pose);
    force_x -= 2.0 * atan(distanceToAttractor) * cos(angleToAttractor);
    force_y -= 2.0 * atan(distanceToAttractor) * sin(angleToAttractor);
#if 0
    for (const auto &point : _attractor_points_)
    {
        double distanceToAttractor = spf_util::compute_point_distances(point,
                                                                       _my_pose);
        double angleToAttractor = spf_util::compute_point_angles(point,
                                                                 _my_pose);
        force_x -= 2000.0 * atan(distanceToAttractor) * cos(angleToAttractor);
        force_y -= 2000.0 * atan(distanceToAttractor) * sin(angleToAttractor);
    }
#endif
}

void SPF::compute_cohesion_forces(double &force_x, double &force_y)
{
    double n_rangeAP = 0.0;
    for (int16_t i = 0; i < number_of_robots_; i++)
    {
        if (i == _numWhoAmI || _robotIsDeadList_[i])
            continue;
        double distanceToAP = spf_util::compute_point_distances(_robotPos_[i], geometry_msgs::msg::Point());
        if (distanceToAP <= RADIO_COVERAGE_SPF)
            n_rangeAP += 1.0;
    }

    double f = 0.0;
    f = (cohesion_alpha_ * spf_util::count_dead(_robotIsDeadList_)) / (number_of_robots_ * n_rangeAP);
    geometry_msgs::msg::Point zero;
    zero.x = zero.y = 0.0;
    double distanceToZero = spf_util::compute_point_distances(zero, _my_pose);
    double angleToZero = spf_util::compute_point_angles(zero, _my_pose);
    force_x -= f * cos(angleToZero) * atan(distanceToZero);
    force_y -= f * sin(angleToZero) * atan(distanceToZero);
}

void SPF::compute_obstacle_forces(double &force_x, double &force_y, double &module)
{
    float curr_angle = 0.0;
    int n_in_range = 0;
    double relative_force_x = 0.0;
    double relative_force_y = 0.0;
    double relative_angle = 0.0;
    force_x = 0.0;
    force_y = 0.0;
    module = 0.0;
    /* LIDAR objects are sensed relatively to the robot frame. */
    for (size_t i = 0; i < _last_scan.ranges.size(); i++)
    {
        if (std::isnan(_last_scan.ranges[i]))
            continue;
        if (_last_scan.ranges[i] > laser_min_range && _last_scan.ranges[i] < laser_max_range)
        {
            curr_angle = (_last_scan.angle_min + i * _last_scan.angle_increment); // radians
            relative_force_x += cos(curr_angle) * spf_util::calcPot((double)_last_scan.ranges[i], -repulsion_sigma_object_, repulsion_n_object_);
            relative_force_y += sin(curr_angle) * spf_util::calcPot((double)_last_scan.ranges[i], -repulsion_sigma_object_, repulsion_n_object_);
            n_in_range++;
        }
    }
    /* The resulting vector will head in a relative heading (to the robot frame). */
    relative_angle = atan2(relative_force_y, relative_force_x);
    if (n_in_range > 0)
    {
        module = hypot(relative_force_x, relative_force_y) / (double)n_in_range;
    }
    /* Final forces in system coordinates. */
    force_x = module * cos(relative_angle + _last_yaw);
    force_y = module * sin(relative_angle + _last_yaw);
}

void SPF::compute_robot_forces(double &force_x, double &force_y)
{
    force_x = 0.0;
    force_y = 0.0;
    for (const auto &id : _robotPosses_->getAllIds())
    {
        auto pose = _robotPosses_->get(id);
        double distanceToRobot = spf_util::compute_point_distances(pose, _my_pose);
        double angleToRobot = spf_util::compute_point_angles(pose, _my_pose);
        force_x += cos(angleToRobot + M_PI) * spf_util::calcPot(distanceToRobot, -repulsion_sigma_robot_, repulsion_n_robot_);
        force_y += sin(angleToRobot + M_PI) * spf_util::calcPot(distanceToRobot, -repulsion_sigma_robot_, repulsion_n_robot_);
    }
}

geometry_msgs::msg::Twist SPF::compute_velocity(void)
{
    /* Hysteresis variable to change between Navigating and STATIONARY */
    static unsigned char hysteresis = 0;
    auto command = geometry_msgs::msg::Twist();
    double force_object_x = 0.0;
    double force_object_y = 0.0;
    double force_object_module = 0.0;
    double force_attractive_x = 0.0;
    double force_attractive_y = 0.0;
    double force_robot_x = 0.0;
    double force_robot_y = 0.0;
    double force_total_x = 0.0;
    double force_total_y = 0.0;
    double module = 0.0, angle;

    /* Simulation ONLY! Simulate which robots are in close proximity. */
    _neighbour_list_ = spf_util::compute_neighbours(_numWhoAmI, _robotPos_,
                                                    _robotIsDeadList_, RADIO_COVERAGE_SPF);

    switch (current_role_)
    {
    case ROLE_DEPLOYING:
        /* Expansion (repulsion) forces due to the other robots. */
        compute_robot_forces(force_robot_x, force_robot_y);
        /* Repulsion forces due to laser-caught obstacles. Always present. */
        compute_obstacle_forces(force_object_x, force_object_y,
                                force_object_module);
        break;
    case ROLE_ADJUST:
        /* Cohesion forces to keep the MWSN together. */
        compute_cohesion_forces(force_attractive_x, force_attractive_y);
        compute_robot_forces(force_robot_x, force_robot_y);
        /* Repulsion forces due to laser-caught obstacles. Always present. */
        compute_obstacle_forces(force_object_x, force_object_y,
                                force_object_module);
        break;
    case ROLE_GOING_TO:
        /* Attractive force due to the existance of an attractor. */
        compute_attractive_forces(force_attractive_x, force_attractive_y);
        /* Repulsion forces due to laser-caught obstacles. Always present. */
        compute_obstacle_forces(force_object_x, force_object_y,
                                force_object_module);
        break;
    case ROLE_RETRIEVING:
        /* Attractive force due to the existance of an attractor. */
        compute_attractive_forces(force_attractive_x, force_attractive_y);
        /* Repulsion forces due to laser-caught obstacles. Always present. */
        compute_obstacle_forces(force_object_x, force_object_y,
                                force_object_module);
        break;

    case ROLE_STATIONARY:
        /* Expansion (repulsion) forces due to the other robots. */
        compute_robot_forces(force_robot_x, force_robot_y);
        /* Repulsion forces due to laser-caught obstacles. Always present. */
        compute_obstacle_forces(force_object_x, force_object_y,
                                force_object_module);
        break;
    default:
        break;
    }
    // Total forces.
    force_total_x = force_object_x + force_robot_x + force_attractive_x;
    force_total_y = force_object_y + force_robot_y + force_attractive_y;
    module = sqrt(
        force_total_x * force_total_x + force_total_y * force_total_y);

    if (current_role_ == RoleTypes::ROLE_STATIONARY)
    {
        if (module > (1.5 * _movement_threshold_))
        {
            hysteresis++;
        }
        if (hysteresis == 4 && is_first_deploy)
        {
            hysteresis = 0;
            current_role_ = RoleTypes::ROLE_DEPLOYING;
            notifier_(current_role_);
        }
    }
    else if (current_role_ == ROLE_ADJUST || current_role_ == ROLE_DEPLOYING ||
             current_role_ == ROLE_RETRIEVING || current_role_ == ROLE_GOING_TO)
    {
        /* The module is greather than the threshold: Robot should move. */
        angle = atan2(force_total_y / module, force_total_x / module);
        /* Determines, based on current module and destination angle, the direction and
         * speed the robot should apply. */
        command = determine_movement(angle);
        if (module < _movement_threshold_)
        {
            hysteresis++;
        }
        if (hysteresis == 4)
        {
            hysteresis = 0;
            current_role_ = RoleTypes::ROLE_STATIONARY;
            notifier_(current_role_);
            /* If an attractor was set, erase it. */
            // if (_attractor_points_.size() > 0)
            //     _attractor_points_.clear();
        }
    }
    // command = check_oscillation(command);
    return command;
}

geometry_msgs::msg::Twist SPF::determine_movement(const double &angle)
{
    auto command = geometry_msgs::msg::Twist();
    auto angle_d = fabs(_last_yaw - angle);
    if (angle_d > 0.1)
    {
        if (angle > 0)
        {
            if (_last_yaw < angle)
            {
                command.linear.x = 0.0; // m/s
                command.angular.z = _max_speed;
            }
            else
            {
                command.linear.x = 0.0; // m/s
                command.angular.z = -_max_speed;
            }
        }
        else
        {
            if (_last_yaw > angle)
            {
                command.linear.x = 0.0; // m/s
                command.angular.z = -_max_speed;
            }
            else
            {
                command.linear.x = 0.0; // m/s
                command.angular.z = _max_speed;
            }
        }
    }
    else
    {
        command.linear.x = _max_speed; // m/s
        command.angular.z = 0.0;
    }
    return command;
}

float SPF::extract_yaw(geometry_msgs::msg::Quaternion &quat)
{
    tf2::Quaternion q;
    tf2::convert(quat, q);
    tf2::Matrix3x3 m(q);
    double d_yaw, roll, pitch;
    m.getRPY(roll, pitch, d_yaw);
    return (float)d_yaw;
}

void SPF::force_callback(void)
{
    /* Publishes the location of each robot at the same rate that is needed. */
    publish_pose();
    /* SPF */
    auto vel = compute_velocity();
    _robotVelPub_->publish(std::move(vel));
}

std::vector<bool> *SPF::get_DeadRobots(void)
{
    return &_robotIsDeadList_;
}

geometry_msgs::msg::Point SPF::get_Pose(void)
{
    return _my_pose;
}

void SPF::laser_callback(const sensor_msgs::msg::LaserScan &msg)
{
    _last_scan = msg;
}

void SPF::odom_sub_callback(const std::shared_ptr<rclcpp::SerializedMessage> &msg)
{
    auto deserializer = rclcpp::Serialization<nav_msgs::msg::Odometry>();
    nav_msgs::msg::Odometry temp;
    deserializer.deserialize_message(msg.get(), &temp);
    if (_localization == LocType_t::Odometry)
    {
        auto abs_pose = spf_util::transform_pose_rel2abs(_my_initial_pose, temp.pose.pose);
        _my_pose.x = abs_pose.position.x;
        _my_pose.y = abs_pose.position.y;
        _last_yaw = extract_yaw(abs_pose.orientation);
    }
    else if (_localization == LocType_t::GroundTruth)
    {
        _my_pose.x = temp.pose.pose.position.x;
        _my_pose.y = temp.pose.pose.position.y;
        _last_yaw = extract_yaw(temp.pose.pose.orientation);
    }
}

void SPF::publish_pose(void)
{
    auto msg = ros2_mwsn_msgs::msg::Position();
    msg.origin = _numWhoAmI;
    msg.x = _my_pose.x;
    msg.y = _my_pose.y;
    msg.theta = _last_yaw;
    _robotLocPub_->publish(std::move(msg));
}

void SPF::resume(void)
{
    force_update_timer_->reset();
}

void SPF::robotsPose_callback(const ros2_mwsn_msgs::msg::Position &msg)
{
    /* Random distribution parameters */
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> distr(0.0, 1.0);
    /* Parse the pose without simulating losses. */
    int16_t robot_index = msg.origin;
    _robotPos_[robot_index].x = msg.x;
    _robotPos_[robot_index].y = msg.y;
    _robotYaw_[robot_index] = msg.theta;

    if (spf_util::compute_point_distances(_my_pose, _robotPos_[robot_index]) < RADIO_COVERAGE_SPF)
    {
        if (robot_index == _numWhoAmI)
            return;
        if (distr(gen) >= 0.4)
        {
            _robotPosses_->set(robot_index, _robotPos_[robot_index]);
        }
    }
}

void SPF::set_Attractor(const geometry_msgs::msg::Point &point)
{
    _attractor_points_.clear();
    _attractor_points_.emplace_back(point);
}

void SPF::set_DeadRobot(const int16_t &id)
{
    _robotIsDeadList_[id] = true;
}

void SPF::set_InitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped &pose)
{
    _my_initial_pose = pose.pose.pose;
}

void SPF::set_NotificationFunc(std::function<void(RoleTypes)> func)
{
    notifier_ = func;
}

void SPF::set_Role(const RoleTypes &new_role)
{
    current_role_ = new_role;
    is_first_deploy = (new_role == ROLE_DEPLOYING);
    notifier_(current_role_);
}

void SPF::stop(void)
{
    force_update_timer_->cancel();
}
