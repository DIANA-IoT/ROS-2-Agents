// Project Title: ROS2_AGENTS
// File: src/supervisor_negotiation.cpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#include "ros2_agents/logger.hpp"
#include "ros2_agents/supervisor.hpp"
#include <random>

void Supervisor::ask_for_help(const int16_t &robot_id)
{
    // TODO: TRAFFIC CALC: Robot with id robot_id, sends a help request to every other robot minus itself and AP, that is to n-2 alive robots.
    // INFO: TRAFFIC CALC: sendAdhoc simulates the sending of data from a node to its destination and accumulates
    //				      the packets sent and energy wasted in the traffic statistics (d_data->p_data->packets_sent[] and so on)
    int index;
    for (index = 0; index < n_robots_; index++)
    {
        if ((index != indexAP) && (index != robot_id) && (RobotLoggerList_[index]->get_currentRole() != ROLE_DEAD))
        {
            // Only send information to those nodes that are not broken and are not the AP
            msg_tx.assign(n_robots_, false);
            send_AdHoc(robot_id, robot_id, index, ONLY_IF_CLOSER);
        }
    }
    // TODO --> JMCG: here we could reduce the levels of battery inmediately and reset the traffic statistics for further computing or keep accumulating. We have to decide where do this.
    RCLCPP_INFO(this->get_logger(), "Robot %d started the negotiation due to negative willingness", robot_id);
    std::string tmp;
    tmp += "Robot " + std::to_string(robot_id) + " started the negotiation due to negative willingness";
    logger->write_entry_negotiation_result(tmp);
    started_negotiation_[robot_id] = 1;
    total_negotiations++;
    requested_help_[robot_id] = 1;
    msgs_exchanged_[robot_id]++;
}

void Supervisor::handle_helpRequest(const int16_t &robot_id, std::vector<std::pair<float, int>> &robots_w)
{
    /* Temporary fix: create a vector with each robot pose that will be used in calc_utility */
    std::vector<geometry_msgs::msg::Point> robotpoints;
    for (size_t i = 0; i < RobotLoggerList_.size(); i++)
    {
        robotpoints.emplace_back(RobotLoggerList_[i]->get_currentPosition());
    }
    for (int x = 0; x < (int)robots_w.size(); x++)
    {
        auto helperid = robots_w[x].second;
        double u;
        msgs_exchanged_[robot_id]++;
        u = Negotiation::calc_utility(RobotLoggerList_[helperid]->get_batteryPercentage(),
                                      robotpoints,
                                      helperid,
                                      robot_id,
                                      indexAP,
                                      w[helperid],
                                      packets_per_agent_);
        // RCLCPP_INFO(this->get_logger(), "Utility from robot %d to %d: %f", helperid, robot_id, u);
        giving_help_[robot_id * n_robots_ + helperid] = w[helperid] + u;
        if ((w[helperid] + u) > 0.0000001)
        {
            /* TRAFFIC CALC if this is positive then a reply is sent from agents with positive willingness.
            Specifically agent with id robot_id, sends one message to agent  with id x */
            msg_tx.assign(n_robots_, false);
            send_AdHoc(robot_id, helperid, robot_id, ONLY_IF_CLOSER);
        }
    }
}

void Supervisor::clear_tasks(void)
{
    for (int i = 0; i < n_robots_; i++)
    {
        for (int j = 0; j < n_robots_; j++)
        {
            if (RobotLoggerList_[i]->get_currentRole() == ROLE_DEAD)
            {
                if (tasks_assigned_[i * n_robots_ + j] == 1)
                {
                    tasks_assigned_[i * n_robots_ + j] = -1;
                    // negotiation_round = 1;
                }
                if (started_negotiation_[i] == 1)
                {
                    unfinished_negotiations++;
                    started_negotiation_[i] = 0;
                }
            }
        }
    }
}

void Supervisor::perform_negotiation(void)
{
    clear_tasks();

    // Reset performance and traffic vectors?
    geometry_msgs::msg::Point depot;
    depot.x = 0.0;
    depot.y = 0.0;
    RobotLogger *ptr_to_robot = nullptr;
    for (int16_t i = 0; i < (int16_t)RobotLoggerList_.size(); i++)
    {
        ptr_to_robot = RobotLoggerList_[i];
        if (ptr_to_robot->get_currentRole() == ROLE_DEAD || ptr_to_robot->get_currentRole() == ROLE_RETRIEVING)
        {
            w[i] = -2.0;
            continue;
        }
        requested_help_[i] = 0;
        w[i] = Negotiation::calc_willingness(ptr_to_robot->get_batteryPercentage(),
                                             depot,
                                             ptr_to_robot->get_currentPosition(),
                                             w_h[i],
                                             w_h2[i]);
        if (w[i] == -1.0 && previous_step_w[i] <= -1.0)
        {
            w[i] = -2.0;
        }
        else if (w[i] == -1.0)
        {
            w[i] = -1.0;
            RCLCPP_INFO(this->get_logger(), "Willingness %f for robot %d, w_H %f, w_H20 %f",
                        w[i], i, w_h[i], w_h2[i]);
        }
        previous_step_w[i] = w[i];
    }
    // Write entry with willingness
    logger->write_entry_willingness(w, simulation_time_);
    /* Robots that need help */
    std::vector<std::pair<float, int>> robots_help = sort_willingness(true);
    /* Robots that are willing to help */
    auto robots_helping = sort_willingness(false);
    auto negotiations = robots_help.size();
    /* Random generator for algorithm 0. */
    std::random_device rd;
    std::mt19937 gen(rd());
    /* Vector to mark those robots who are already replaced*/
    std::vector<int> replaced;
    replaced.resize(n_robots_);
    replaced.assign(n_robots_, 0);
    /* Temporary fix: create a vector with each robot pose that will be used in calc_utility */
    std::vector<geometry_msgs::msg::Point> robotpoints;
    for (size_t i = 0; i < RobotLoggerList_.size(); i++)
    {
        robotpoints.emplace_back(RobotLoggerList_[i]->get_currentPosition());
    }
    /* Iterate through each robot that needs help. */
    for (unsigned int k = 0; k < negotiations; k++)
    {
        unsigned int negotiating_robot;
        /* "Flipped" coin algorithm:  */
        if (algorithm_ == 0)
        {
            std::uniform_int_distribution<> distr(0, robots_help.size() - 1);
            negotiating_robot = robots_help[distr(gen)].second;
        }
        /* Logically sorted algorithm: */
        else if (algorithm_ == 1)
        {
            negotiating_robot = robots_help[k].second;
        }
        else if (algorithm_ == 2)
        {
            if (k < robots_helping.size())
            {
                auto helperid = robots_helping[k].second;
                // update utility helperid to network.
                double u;
                double bestu = -1.0;
                int bestid = -1;
                int robot_id;
                /* Index to iterate in the negotiations array (second parameter equals to robotid in need). */
                for (int index = 0; index < (int)negotiations; index++)
                {
                    robot_id = robots_help[index].second;
                    u = Negotiation::calc_utility(RobotLoggerList_[helperid]->get_batteryPercentage(),
                                                  robotpoints,
                                                  helperid,
                                                  robot_id,
                                                  indexAP,
                                                  w[helperid],
                                                  packets_per_agent_);
                    if (u > bestu && replaced[robot_id] == 0)
                    {
                        bestu = u;
                        bestid = robot_id;
                    }
                }

                if (bestu + w[helperid] > 0.0000001 && bestid != -1)
                {
                    RCLCPP_INFO(this->get_logger(), "Robot %d is helping to %d with u %f",
                                helperid, bestid, bestu);
                    std::string log_str;
                    log_str = "Robot " + std::to_string(helperid) + " is helping to " + std::to_string(bestid);
                    log_str += " with u " + std::to_string(bestu);
                    logger->write_entry_negotiation_result(log_str);
                    replaced[bestid] = 1;
                    RCLCPP_INFO(this->get_logger(), "Robotid: %d isreplaced: %d", robot_id, replaced[bestid]);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "No candidate was found to replace %d",
                                bestid);
                    std::string log_str;
                    log_str = "No candiate was found to replace " + std::to_string(bestid);
                    logger->write_entry_negotiation_result(log_str);
                    continue;
                }

                /* Emulate help request */
                int index;
                for (index = 0; index < n_robots_; index++)
                {
                    if ((index != indexAP) && (index != robot_id) && (RobotLoggerList_[index]->get_currentRole() != ROLE_DEAD))
                    { // Only send information to those nodes that are not broken and are not the AP
                        msg_tx.assign(n_robots_, false);
                        send_AdHoc(robot_id, robot_id, index, ONLY_IF_CLOSER);
                    }
                }

                /* Emulate reply from helperid to robot in need*/
                /* TRAFFIC CALC if this is positive then a reply is sent from agents with positive willingness.
                        Specifically agent with id robot_id, sends one message to agent  with id x */
                msg_tx.assign(n_robots_, false);
                send_AdHoc(helperid, helperid, robot_id, ONLY_IF_CLOSER);

                /* Emulate assignment for robot in need to helperid*/
                msg_tx.assign(n_robots_, false);
                send_AdHoc(robot_id, robot_id, helperid, ONLY_IF_CLOSER);

                /* Internal message */
                auto coord = RobotLoggerList_[robot_id]->get_currentPosition();
                send_replace_msg(robot_id, helperid, coord.x, coord.y);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "No more robots were available to help");
                logger->write_entry_negotiation_result("No more robots were available to help");
            }
            continue;
        }

        /* Robot with id: `negotiating_robot`asks for help*/
        ask_for_help(negotiating_robot);
        /* Robots with positive willingness and those who are not replacing
         anyone reply to the help request */
        handle_helpRequest(negotiating_robot, robots_helping);
        /* Choose a replacement among those who are available. */
        int replacement = pick_robot(negotiating_robot);
        /* Mark this robot as unavailable*/
        if (replacement != -1)
        {
            robots_helping.erase(std::remove_if(robots_helping.begin(), robots_helping.end(), 
            [this, replacement](const std::pair<float, int> &p)
                                { return p.second == replacement; }),
                                robots_helping.end());
        }
        /* If random algorithm, delete element. */
        if (algorithm_ == 0)
        {
            robots_help.erase(std::remove_if(robots_help.begin(), robots_help.end(), 
            [this, negotiating_robot](const std::pair<float, int> &p)
                                { return p.second == negotiating_robot; }),
                                robots_help.end());
        }
    }

    for (int i = 0; i < n_robots_; i++)
    {
        auto role = RobotLoggerList_[i]->get_currentRole();
        if (role != ROLE_DEAD && role != ROLE_AP)
        {
            /* Robot with possitive willingness move towards targets? */
            /* Robots with low battery shall move towards the depot. */
            if (w[i] < w_h[i] && w[i]>=-1.0)
            {
                std::string log_str = "Robot " + std::to_string(i) + "with w " +
                                      std::to_string(w[i]) + " heading to the depot";
                logger->write_entry_negotiation_result(log_str);
                RCLCPP_INFO(this->get_logger(), "Robot %d with w %f heading to the depot", i, w[i]);
                send_depot_msg(i);
                w[i] = -2.0;
                previous_step_w[i] = -2.0;
            }
        }
    }
}

void Supervisor::pick_AP(const int16_t &robot_id)
{
    if (requested_help_[robot_id] != 3)
        return;
    requested_help_[robot_id] = 5;
    // INFO: Check who wants to help me
    int assigned_robot = -1;
    float wu = -100.0;
    // INFO: the AP checks all the answers it got and picks the robot with highest willingness+utility
    started_negotiation_[robot_id] = 0;
    for (int j = 0; j < n_robots_; j++)
    {
        if (!(robot_id == j) && !(indexAP == j)) // INFO: I won't get help from myself or AP
        {
            if (giving_help_[robot_id * n_robots_ + j] > 0.0000001)
            {
                // RCLCPP_INFO(this->get_logger(), "Robot %d wants to help, with wu %.5f", j, giving_help_[robot_id * n_robots_ + j]);
                if (giving_help_[robot_id * n_robots_ + j] > wu)
                {
                    wu = giving_help_[robot_id * n_robots_ + j];
                    assigned_robot = j;
                }
            }
        }
    }
    if (assigned_robot != -1)
    {
        RCLCPP_INFO(this->get_logger(), "Robot %d is assigned to replace %d with wu %.2f",
                    assigned_robot, robot2help, wu);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No candidate was found to replace %d",
                    robot2help);
    } // INFO: clear the responses given to robot robot_id
    for (int jj = 0; jj < n_robots_; jj++)
    {
        giving_help_[robot_id * n_robots_ + jj] = -1.0;
    }

    for (int j = 0; j < n_robots_; j++)
    {
        if (tasks_assigned_[robot2help * n_robots_ + j] == -1)
        {
            if (assigned_robot == -1) // INFO: if noone is assigned this tasks remains marked as -1.
            {
                tasks_assigned_[robot2help * n_robots_ + j] = -1; // INFO: this means that the task is not assigned, this is what the AP handles.
            }
            else
            {
                // TODO: TRAFFIC CALC: AP sends one message for assigning a robot to task. AP sends this message to robot with id: assigned_robot
                // INFO: SendAdHoc accumulates traffic statistics each time it is called
                if (RobotLoggerList_[assigned_robot]->get_currentRole() != ROLE_DEAD) /* INFO: JCMG Probably this check is not necessary, but as I am not sure... */
                {                                                                     // Only send information to those nodes that are not broken and are not the AP
                    msg_tx.assign(n_robots_, false);
                    send_AdHoc(indexAP, indexAP, assigned_robot, ONLY_IF_CLOSER);
                }

                tasks_assigned_[robot2help * n_robots_ + j] = 0; // INFO: agent that had this task, drops it.
                for (int cancel = 0; cancel < n_robots_; cancel++)
                {
                    tasks_assigned_[assigned_robot * n_robots_ + cancel] = 0; // INFO: assigned agent initially drops previous locations
                }
                tasks_assigned_[assigned_robot * n_robots_ + j] = 1;
                xpos_[assigned_robot * n_robots_ + j] = xpos_[robot2help * n_robots_ + j];
                ypos_[assigned_robot * n_robots_ + j] = ypos_[robot2help * n_robots_ + j];

                msgs_exchanged_[robot_id]++; // Esto va aqui?
            }
            negotiation_concluded++; // Esto va aqui?
        }
    }
    if (assigned_robot > 0)
    {
        auto coord = RobotLoggerList_[robot2help]->get_currentPosition();
        send_replace_msg(robot2help, assigned_robot, coord.x, coord.y);
    }
}

int Supervisor::pick_robot(const int16_t &robot_id)
{
    std::string log_str = "Robot " + std::to_string(robot_id) + " will conclude negotiation";
    logger->write_entry_negotiation_result(log_str);
    RCLCPP_INFO(this->get_logger(), "Robot %d will conclude negotiation", robot_id);
    requested_help_[robot_id] = 0;
    // INFO: Check who wants to help me
    int assigned_robot = -1;
    float wu = -100.0;
    // INFO: the AP checks all the answers it got and picks the robot with highest willingness+utility
    started_negotiation_[robot_id] = 0;

    for (int j = 0; j < n_robots_; j++)
    {
        if (!(robot_id == j) && !(indexAP == j)) // INFO: I won't get help from myself or AP
        {
            if (giving_help_[robot_id * n_robots_ + j] > 0.0000001)
            {
                // printf("Robot %d wants to help, with wu %.5f,\n", j, giving_help_[robot_id * n_robots_ + j]);
                if (giving_help_[robot_id * n_robots_ + j] > wu)
                {
                    wu = giving_help_[robot_id * n_robots_ + j];
                    assigned_robot = j;
                }
            }
        }
    }

    if (assigned_robot != -1)
    {
        RCLCPP_INFO(this->get_logger(), "Robot %d is assigned to replace %d with wu %.2f",
                    assigned_robot, robot_id, wu);
        std::string log_str;
        log_str += "Robot " + std::to_string(assigned_robot) + " is assigned to replace ";
        log_str += std::to_string(robot_id) + " with wu " + std::to_string(wu);
        logger->write_entry_negotiation_result(log_str);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No candidate was found to replace %d",
                    robot_id);
        std::string log_str;
        log_str += "No candidate was found to replace " + std::to_string(robot_id);
        logger->write_entry_negotiation_result(log_str);
    }
    /* Clear the responses given to robot robot_id. */
    for (int jj = 0; jj < n_robots_; jj++)
    {
        giving_help_[robot_id * n_robots_ + jj] = -1.0;
    }
    // Clear giving_help from assigned_robot only if picked one
    if (assigned_robot != -1)
    {
        for (int jj = 0; jj < n_robots_; jj++)
        {
            giving_help_[jj * n_robots_ + assigned_robot] = -1.0;
        }
        // w[assigned_robot] = -1.0;
    }
    negotiation_concluded++;     // Esto no tengo muy claro si va aqui
    msgs_exchanged_[robot_id]++; // idem

    if (assigned_robot >= 0)
    {
        auto coord = RobotLoggerList_[robot_id]->get_currentPosition();
        send_replace_msg(robot_id, assigned_robot, coord.x, coord.y);
    }
    return assigned_robot;
}

std::vector<std::pair<float, int>> Supervisor::sort_willingness(const bool &sort_urgent)
{
    std::vector<std::pair<float, int>> vp;
    /* Insert willingness + indexes pairs. */
    for (int i = 0; i < n_robots_; i++)
    {
        vp.push_back(std::make_pair(w[i], i));
    }
    /* Sorting pairs */
    /* Sorts in ascending order, returns robots who need help. */
    std::sort(vp.begin(), vp.end());
    if (sort_urgent)
    {
        /* Erases elements that either:
        //     a) Have positive willingness (do not need for help) or have already asked for help.
        //     b) Have negative willingness but it is higher than wh2. */
        vp.erase(std::remove_if(vp.begin(), vp.end(),
                                [this](const std::pair<float, int> &p)
                                {
                                    return (p.first == -2.0 || p.first > 0.0 ||
                                            (p.first < 0.0 && p.first > w_h2[p.second]));
                                }),
                 vp.end());
    }
    /* Sorts in descending order, returns robots who are willing to help. */
    else
    {
        /* Reverses vector. */
        std::reverse(vp.begin(), vp.end());
        /* Erases elements with w < 0 */
        vp.erase(std::remove_if(vp.begin(), vp.end(), [this](const std::pair<float, int> &p)
                                { return (p.first < 0.0 || p.second == indexAP); }),
                 vp.end());
    }
    return vp;
}
