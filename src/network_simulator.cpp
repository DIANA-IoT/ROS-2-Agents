// Project Title: ROS2_AGENTS
// File: src/network_simulator.cpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#include <algorithm>
#include <fenv.h>
#include <limits>
#include "ros2_agents/spf_utilities.hpp"
#include "ros2_agents/supervisor.hpp"
#include "ros2_agents/topic_utilities.hpp"

using namespace std::placeholders;

double Supervisor::calculate_coverage(const std::vector<int> &connected)
{
    srand(time(NULL));

    int grid_size = round(MAP_SIZE / GRID_CELD);

    int **coverage_grid = create_grid(MAP_SIZE, connected);

    // Recorro la rejilla para calcular la cobertura
    int x, y, sum = 0;
    for (y = 0; y < grid_size; y++)
    {
        for (x = 0; x < grid_size; x++)
        {
            if ((rand() % 100) < coverage_grid[x][y])
            {

                sum++;
            }
        }
    }

    free(coverage_grid);

    return (sum / pow(grid_size, 2));
}

double Supervisor::calculate_meanPower(const int &my_robot)
{
    int j, received = 0, num_received = 0;
    double tx_power, mean_power = 0;

    for (j = 0; j < n_robots_; j++)
    {
        tx_power = 0;
        if ((my_robot != j) && (RobotLoggerList_[j]->get_currentRole() != ROLE_DEAD))
        {
            msg_tx.assign(n_robots_, false);
            received = 0;
            calculate_potAdHoc(my_robot, my_robot, j, &received, &tx_power);
            if (received)
            {
                mean_power += tx_power;
                num_received++;
            }
        }
    }

    if (num_received != 0)
    {
        mean_power = mean_power / num_received;
    }

    return mean_power;
}

void Supervisor::calculate_potAdHoc(int my_robot, int sender, int receiver, int *fin, double *tx_power)
{
    int i;
    // double k = 5 * pow((double)10.0, -8);
    float radio_coverage = RADIO_COVERAGE_SPF;

    if (receiver != my_robot)
    {
        for (i = 0; i < n_robots_ && !(*fin); i++)
        {
            if ((i != my_robot) && (RobotLoggerList_[i]->get_currentRole() != ROLE_DEAD))
            {
                if (spf_util::compute_point_distances(
                        RobotLoggerList_[my_robot]->get_currentPosition(),
                        RobotLoggerList_[i]->get_currentPosition()) <= radio_coverage)
                {
                    if ((spf_util::compute_point_distances(RobotLoggerList_[i]->get_currentPosition(),
                                                           RobotLoggerList_[receiver]->get_currentPosition()) <
                         spf_util::compute_point_distances(RobotLoggerList_[my_robot]->get_currentPosition(),
                                                           RobotLoggerList_[receiver]->get_currentPosition())))
                    {
                        while (msg_tx[i] < (NUM_MAX_RETX)) // TODO: msg_tx should be cleared? Right now it is a boolean
                        {
                            (*tx_power) += COCIENTE_PRXMIN_K * pow(spf_util::compute_point_distances(
                                                                       RobotLoggerList_[my_robot]->get_currentPosition(),
                                                                       RobotLoggerList_[i]->get_currentPosition()),
                                                                   2);
                            msg_tx[my_robot] = 1;
                            calculate_potAdHoc(i, sender, receiver, fin, tx_power);
                            if ((((double)rand() / (double)RAND_MAX)) > PROB_LOSS)
                                break;
                        }
                    }
                }
            }
        }
    }
    else
    {
        *fin = 1;
    }
}

double Supervisor::calculate_uniformity(const std::vector<int> &connected)
{
    double uniformity = 0, net_uniformity = 0, d_media, d;
    double unbroken_robots = 0.0;
    int i, j, k;

    for (i = 0; i < n_robots_; i++)
    {
        k = 0;
        d_media = 0;
        if (connected[i])
        {
            /* Calculation of total nearby nodes and average distance to them */
            for (j = 0; j < n_robots_; j++)
            {
                d = spf_util::compute_point_distances(RobotLoggerList_[i]->get_currentPosition(),
                                                      RobotLoggerList_[j]->get_currentPosition());
                if (d <= MIN_WIFI_DISTANCE)
                {
                    k++;
                    d_media += d;
                }
            }
            d_media = d_media / k;

            /* Calculation i uniformity */
            for (j = 0; j < n_robots_; j++)
            {
                if (RobotLoggerList_[j]->get_currentRole() != ROLE_DEAD)
                {
                    d = spf_util::compute_point_distances(RobotLoggerList_[i]->get_currentPosition(),
                                                          RobotLoggerList_[j]->get_currentPosition());
                    if (d <= MIN_WIFI_DISTANCE)
                    {
                        uniformity += pow(d - d_media, 2);
                    }
                }
            }
            uniformity = sqrt(uniformity / k);

            net_uniformity += uniformity;

            unbroken_robots += 1.0;
        }
    }
    net_uniformity = net_uniformity / unbroken_robots;

    return net_uniformity;
}

int **Supervisor::create_grid(int map_size, const std::vector<int> &connected)
{
    int i, gx, gy, x, y;

    int grid_size = round(map_size / GRID_CELD);

    // Create and initialize grid
    int **coverage_grid = (int **)malloc(sizeof(int *) * grid_size);
    for (x = 0; x < grid_size; x++)
    {
        coverage_grid[x] = (int *)malloc(sizeof(int) * grid_size);
    }

    for (x = 0; x < grid_size; x++)
    {
        for (y = 0; y < grid_size; y++)
        {
            coverage_grid[x][y] = 0;
        }
    }

    fesetround(FE_UPWARD); // rounding towards positive infinity
    for (i = 0; i < n_robots_; i++)
    {
        if (connected[i])
        {
            // Extract the grid from its position
            gx = (int)rint(fabs(-map_size / 2 - RobotLoggerList_[i]->get_currentPosition().x) / GRID_CELD);
            gy = (int)rint(fabs(-map_size / 2 - RobotLoggerList_[i]->get_currentPosition().y) / GRID_CELD);

            // Loop the grid
            for (y = 0; y < grid_size; y++)
            {
                for (x = 0; x < grid_size; x++)
                {
                    // In the nearby squares up to 0.6 m (3 squares) the probability is 100%
                    if (sqrt(pow(gx - x, 2) + pow(gy - y, 2)) <= (int)rint(ROBOT_COVERAGE / 2 / GRID_CELD))
                    {
                        coverage_grid[x][y] += 100;
                    }
                    else
                    {
                        // In boxes from 0.6 to 1.2 the probability is 60%
                        if (sqrt(pow(gx - x, 2) + pow(gy - y, 2)) <= (int)rint(ROBOT_COVERAGE * 3 / 4 / GRID_CELD))
                        {
                            coverage_grid[x][y] += 60;
                        }
                        // In the 1.2 to 1.6 m frames the probability is 20%
                        else if (sqrt(pow(gx - x, 2) + pow(gy - y, 2)) <= (int)rint(ROBOT_COVERAGE / GRID_CELD))
                        {
                            coverage_grid[x][y] += 20;
                        }
                    }
                }
            }
        }
    }
    return coverage_grid;
}

void Supervisor::detect_connectedAdHoc(int my_robot, std::vector<int> &connected)
{
    int i = 0;
    float radio_coverage = RADIO_COVERAGE_SPF;
    int NewDetected[n_robots_];

    // INFO: First pass: mark as connected those not broken in the area of "my_robot"
    //       and also it's important to detect which one is a new discovery, in order not to keep searching for ever
    for (i = 0; i < n_robots_; i++)
    {
        NewDetected[i] = 0; // INFO: 0 by default. Will be one if first time detected (see bellow)
        if ((RobotLoggerList_[i]->get_currentRole() != ROLE_DEAD) &&
            (spf_util::compute_point_distances(RobotLoggerList_[my_robot]->get_currentPosition(),
                                               RobotLoggerList_[i]->get_currentPosition()) <= radio_coverage))
        {
            if (connected[i] == 0)
            {
                NewDetected[i] = 1; // INFO: A node not already discovered
            }
            connected[i]++; // connected[my_robot] will be increased too
        }
    }

    // INFO: SECOND PASS ... find the nodes with connectivity to those recently discovered
    //      (Already discovered nodes have been o will be explored by the nodes who discovered them first)
    for (i = 0; i < n_robots_; i++)
    {
        if ((NewDetected[i]) && (RobotLoggerList_[i]->get_currentRole() != ROLE_DEAD) && (i != my_robot))
        {
            // Search robots connected to the ones with connection with my_robot that have not been explored yet...
            detect_connectedAdHoc(i, connected);
        }
    }
}

void Supervisor::network_test(void)
{
    initialize_performaceVectors();

    for (int16_t i = 0; i < n_robots_; i++)
    {
        if (RobotLoggerList_[i]->get_currentRole() != ROLE_DEAD)
        {
            msg_tx.assign(n_robots_, false);
            if (i != indexAP)
            { // AP does not send test data.
                send_AdHoc(i, i, indexAP, ONLY_IF_CLOSER);
            }
        }
    }

    double mean_packets_sent = 0;
    double mean_packets_rx = 0;
    double st_dev_packets_sent = 0;
    double st_dev_packets_rx = 0;

    double mean_packets_sent_noAP = 0;
    double st_dev_packets_sent_noAP = 0;

    double no_rotos = 0.0;

    double min_packets_sent = 1e200;
    double max_packets_sent = 0;
    double min_packets_sent_noAP = 1e200;
    double max_packets_sent_noAP = 0;

    /* Calcula media y desviacion tipica, maximo y minimo */
    for (int16_t i = 0; i < n_robots_; i++)
    {
        if (RobotLoggerList_[i]->get_currentRole() != ROLE_DEAD)
        {
            mean_packets_sent += packets_sent_[i];
            if (min_packets_sent > packets_sent_[i])
                min_packets_sent = packets_sent_[i];
            if (max_packets_sent < packets_sent_[i])
                max_packets_sent = packets_sent_[i];
            mean_packets_rx += packets_rx_[i];
            no_rotos += 1.0;
            if (i != indexAP)
            {
                if (min_packets_sent_noAP > packets_sent_[i])
                    min_packets_sent_noAP = packets_sent_[i];
                if (max_packets_sent_noAP < packets_sent_[i])
                    max_packets_sent_noAP = packets_sent_[i];
                mean_packets_sent_noAP += packets_rx_[i];
            }
        }
    }
    mean_packets_sent = mean_packets_sent / no_rotos;
    mean_packets_sent_noAP = mean_packets_sent_noAP / (no_rotos - 1.0);
    mean_packets_rx = mean_packets_rx / no_rotos;

    for (int16_t i = 0; i < n_robots_; i++)
    {
        if (RobotLoggerList_[i]->get_currentRole() != ROLE_DEAD)
        {
            st_dev_packets_sent += pow(packets_sent_[i] - mean_packets_sent, 2);
            st_dev_packets_rx += pow(packets_rx_[i] - mean_packets_rx, 2);

            if (i != indexAP)
            {
                st_dev_packets_sent_noAP += pow(packets_sent_[i] - mean_packets_sent, 2);
            }
        }
    }
    st_dev_packets_sent = sqrt(st_dev_packets_sent) / no_rotos;
    st_dev_packets_sent_noAP = sqrt(st_dev_packets_sent_noAP) / (no_rotos - 1.0);
    st_dev_packets_rx = sqrt(st_dev_packets_rx) / no_rotos;

    logger->write_entry_packets(mean_packets_sent, mean_packets_rx, st_dev_packets_sent,
                                st_dev_packets_rx, min_packets_sent, max_packets_sent);
    logger->write_entry_packetsAP(mean_packets_sent_noAP, st_dev_packets_sent_noAP,
                                  min_packets_sent_noAP, max_packets_sent_noAP);
}

void Supervisor::save_performance(void)
{
    std::vector<int> connected;
    int num_alive_connected = 0;
    connected.resize(n_robots_);
    connected.assign(n_robots_, 0);
    detect_connectedAdHoc(indexAP, connected);

    int16_t i = 0;
    // INFO: Now updates the "connected" flag of the robots, so that the statistic computing functions use it.
    //		Also we compute number of alive & connected here...
    //  Skip this?
    for (i = 0; i < n_robots_; i++)
    {
        if (RobotLoggerList_[i]->get_currentRole() != ROLE_DEAD &&
            connected[i] != 0)
        {
            // INFO: since DetectConnecredAdHoc do not mark dead robots as connected, we didn't need to check the "broken" flag
            //       but just doublechecking...
            //   d_data->robot_data[i].connected=connected[i];
            num_alive_connected++;
        }
        else
        {
            // d_data->robot_data[i].connected=0;
        }
    }

    double relative_coverage = calculate_coverage(connected);
    double total_coverage = relative_coverage * MAP_SIZE;
    double uniform = calculate_uniformity(connected);
    double mean_dist = 0.0;
    double robots_alive = 0.0;
    double mean_power = 0.0;

    double total_acc_packet_negotiation = 0.0;
    double total_acc_packet_total = 0.0;

    for (i = 0; i < n_robots_; i++)
    {

        if (RobotLoggerList_[i]->get_currentRole() != ROLE_DEAD)
        {
            mean_dist += RobotLoggerList_[i]->get_distanceWalked();
            mean_power += calculate_meanPower(i);
            robots_alive += 1.0;
        }
        // FIXME: Should we skip the AP?
        total_acc_packet_negotiation += acc_packet_negotiation_[i];
        total_acc_packet_total += acc_packet_total_[i];
    }
    mean_dist = mean_dist / robots_alive;
    mean_power = mean_power / robots_alive;

    initialize_performaceVectors();
    logger->write_entry_alive(simulation_time_, spf_util::count_dead(RobotDeadList_));
    logger->write_entry_coverage(relative_coverage, total_coverage, uniform,
                                 mean_dist, robots_alive, mean_power);
    network_test();

    logger->write_entry_negotiation(total_acc_packet_negotiation, total_acc_packet_total, num_alive_connected);
}

void Supervisor::send_AdHoc(int my_robot, int sender, int receiver, int strategy)
{
    float radio_coverage = RADIO_COVERAGE_SPF;
    if (RobotLoggerList_[my_robot]->get_currentRole() == ROLE_DEAD)
    {
        std::string temp = "Attempted to send packet from node " + std::to_string(my_robot) + " which is dead";
        logger->write_entry_negotiation_result(temp);
        return;
    }
    packets_sent_[my_robot]++;
    // INFO: For AdHoc TX the transceiver should transmit with apropiate power to reach a range of radio_coverage,
    //  so we compute the extra battery charge consumption to reach that level...

    double wasted = wasted_energy_[my_robot];
    wasted += ROBOT_WASTE_PER_PACKET * (1.0 + ROBOT_WASTE_INCREASE_DISTANCE * radio_coverage);
    wasted_energy_[my_robot] = wasted;

    if (sender == my_robot)
    {
        // printf("Robot %d envía a %d:\n",my_robot+1,receiver+1);
        own_packets_sent_[my_robot]++;
    }
    // printf("Robot %d en send_AdHoc: posx %f posy %f \n",my_robot+1,d_data->p_data->positions[my_robot].px,d_data->p_data->positions[my_robot].py);
    msg_tx[my_robot] = 1;

    for (int16_t i = 0; i < n_robots_; i++)
    {
        if ((i != my_robot) && (ROLE_DEAD != RobotLoggerList_[i]->get_currentRole()))
        {
            if (spf_util::compute_point_distances(RobotLoggerList_[my_robot]->get_currentPosition(),
                                                  RobotLoggerList_[i]->get_currentPosition()) <= radio_coverage)
            {
                // printf("Robot %d reenvía a %d el mensaje de %d para %d\n",my_robot+1,i+1,sender+1,receiver+1);
                // d_data->p_data->packets_rx[i]++;
                packets_rx_[i]++;
                if (receiver == i)
                {
                    // _data->p_data->own_packets_rx[i]++;
                    own_packets_rx_[i]++;
                }
                else if (strategy == 0 && msg_tx[i] == 0) // Estrategia rtx una vez todos.
                {
                    send_AdHoc(i, sender, receiver, strategy);
                }
                else if ((spf_util::compute_point_distances(RobotLoggerList_[i]->get_currentPosition(),
                                                            RobotLoggerList_[receiver]->get_currentPosition()) <
                          spf_util::compute_point_distances(RobotLoggerList_[my_robot]->get_currentPosition(),
                                                            RobotLoggerList_[receiver]->get_currentPosition())) &&
                         msg_tx[i] == 0) // Estrategia rtx si mas cerca
                {
                    send_AdHoc(i, sender, receiver, strategy);
                }
            }
        }
    }
}

void Supervisor::network_simulation(void)
{
    int num_rotos = 0;
    // Think about it
    double battery_sim_time_slice = 0.0;
    // static double accum_cycles = 0.0;
    RCLCPP_INFO(this->get_logger(), "Network simulation taking place");
    double min_sim_cycles = BATTERY_CAPACITY_MAH / ROBOT_WASTE_PER_PACKET;
    RobotLogger *ptr_to_robot = nullptr;
    /* Firstly, clear performance vectors so that data is valid. */
    initialize_performaceVectors();
    for (int16_t i = 0; i < n_robots_; i++)
    {
        auto isdead = (RobotLoggerList_[i]->get_currentRole() == ROLE_DEAD);
        if (isdead != RobotDeadList_[i])
        {
            std::string temp = "Mismatch at isdead for robot " + std::to_string(i);
            logger->write_entry_negotiation_result(temp);
        }
    }

    /* Robots tx data to AP. */
    for (int16_t i = 0; i < n_robots_; i++)
    {
        ptr_to_robot = RobotLoggerList_.at(i);
        if (ptr_to_robot->get_currentRole() != ROLE_DEAD)
        {
            msg_tx.assign(n_robots_, false);
            // if (d_data->robot_data[i].level != 1) // TODO: ?
            // {
            send_AdHoc(i, i, indexAP, ONLY_IF_CLOSER);
            // }
        }
    }
    /* Compute how many sim cycles remain before the first robot depletes. */
    std::vector<double> remaining_cycles;
    remaining_cycles.resize(n_robots_);

    for (int16_t i = 0; i < n_robots_; i++)
    {
        ptr_to_robot = RobotLoggerList_.at(i);
        /* Do not account for AP nor dead robots. */
        if (i == indexAP ||
            ptr_to_robot->get_currentRole() != ROLE_DEAD)
        {
            /* estimated_drain shows how much battery is drained in a simulation cycle. (mAh) */
            double estimated_drain = (wasted_energy_[i] + ROBOT_DUTYCYCLE_WASTE / ROBOT_PACKET_RATE_HOUR);
            if (supervisor_type_ == SUPERVISOR_AAGENTS)
            {
                /* distance to depot or collecting point. */
                geometry_msgs::msg::Point depot;
                depot.x = 0.0;
                depot.y = 0.0;
                double distance = spf_util::compute_point_distances(depot,
                                                                    ptr_to_robot->get_currentPosition());
                /* Remaining percentage until the robot reaches the critical battery level. */
                double to_critical = ptr_to_robot->get_batteryPercentage() - spf_util::compute_critical_level(distance);
                remaining_cycles[i] = (0.01 * to_critical * BATTERY_CAPACITY_MAH) / estimated_drain;
            }
            else if (supervisor_type_ == SUPERVISOR_SPF)
            {
                remaining_cycles[i] = (0.01 * ptr_to_robot->get_batteryPercentage() * BATTERY_CAPACITY_MAH) / estimated_drain;
            }
            if (min_sim_cycles > remaining_cycles[i])
                min_sim_cycles = remaining_cycles[i];
        }
    }
    if (min_sim_cycles < 0.0)
        min_sim_cycles = 0.0; // INFO. If this happens is because a node has dead while moving.

    RCLCPP_INFO(this->get_logger(), "Sim cycles: %f", min_sim_cycles);
    /* Advance simulation time. */
    // INFO: we use now this variable to hold the time slice. It remains constant until the next movement
    // battery_sim_time_slice = min_sim_cycles / SIMULATION_NUM_ITERATIONS;
    battery_sim_time_slice = min_sim_cycles;
    if (battery_sim_time_slice < SIMULATION_MIN_INTERVAL)
        battery_sim_time_slice = SIMULATION_MIN_INTERVAL;
    // INFO: Updates the "global number of cycles" since the beginning of the simulation
    simulation_time_ += battery_sim_time_slice;
    RCLCPP_INFO(this->get_logger(),
                "Simulation goes %f cycles ahead, to %f",
                battery_sim_time_slice,
                simulation_time_);

    // INFO: this code updates the battery levels of the robot after one time slice has passed
    //       and mark as broken all the broken robots.
    std::vector<double> battery_to_substract;
    battery_to_substract.resize(n_robots_);
    double remaining_battery = 0.0;
    /* Update and notify robots with the new battery levels. */
    // INFO: I'll use this loop also for copying traffic statistics to packets_per_agent array
    for (int16_t i = 0; i < n_robots_; i++)
    {
        ptr_to_robot = RobotLoggerList_.at(i);
        // We dont account for AP nor dead robots
        if (ptr_to_robot->get_currentRole() == RoleTypes::ROLE_AP ||
            ptr_to_robot->get_currentRole() == RoleTypes::ROLE_DEAD)
        {
            packets_per_agent_[i] = 0;
            continue;
        }

        /* battery to substract in mAh*/
        battery_to_substract[i] = (wasted_energy_[i] + ROBOT_DUTYCYCLE_WASTE / ROBOT_PACKET_RATE_HOUR) * battery_sim_time_slice;
        /* Convert it to percentage */
        battery_to_substract[i] = 100.0 * (battery_to_substract[i] / BATTERY_CAPACITY_MAH);
        remaining_battery = ptr_to_robot->get_batteryPercentage() - battery_to_substract[i];
        // INFO: Copy traffic statistics to packets_per_agent
        /* TODO: I am not sure if this parameter is correct.
            Should be it be set according to this simulation round or accourding to the
            accumulate? */
        packets_per_agent_[i] = packets_sent_[i];                          // INFO: Per cycle
        acc_packet_total_[i] += packets_sent_[i] * battery_sim_time_slice; // Total packets in the time_slice
        if (remaining_battery < BATTERY_CRITICAL_PER)
        {
            // All nodes with less than 5% of battery are considered to die now. Should we change this?
            // El AP no entra en la cuenta...
            if (i != indexAP)
            {
                //    printf("Robot %i has dead\n", i);
                // rotos[i] = 1;
                num_rotos++; // cuenta cuántos llevamos
            }
        }
    }

    for (int16_t i = 0; i < n_robots_; i++)
    {
        ptr_to_robot = RobotLoggerList_.at(i);
        // We dont account for AP nor dead robots
        if (ptr_to_robot->get_currentRole() == RoleTypes::ROLE_AP ||
            ptr_to_robot->get_currentRole() == RoleTypes::ROLE_DEAD)
            continue;
        MessageRadioUpdateParameter temp;
        temp.percentage = battery_to_substract[i];
        ros2_mwsn_msgs::msg::Supervisor2Controller msg;
        msg.destination = i;             // This field is redundant if the namespace is added
        msg.type = MESSAGE_RADIO_UPDATE; // This field is redundant for dedicated topics
        msg.data.resize(sizeof(double));
        memcpy(&msg.data[0], &temp, sizeof(MessageRadioUpdateParameter));
        battery_pub_[i]->publish(std::move(msg));
        ros2_mwsn_msgs::msg::Controller2Supervisor fake_msg;
        fake_msg.origin = i;
        fake_msg.type = MESSAGE_RADIO_UPDATE;
        fake_msg.data.resize(sizeof(double));
        memcpy(&fake_msg.data[0], &remaining_battery, sizeof(double));
        robot_callback(fake_msg);
    }
}
// INFO: End of the simulation of battery depletion due to communications.