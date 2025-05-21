// Project Title: ROS2_AGENTS
// File: include/ros2_agents/logger.hpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#ifndef __LOGGER_HPP__
#define __LOGGER_HPP__

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "ros2_agents/defs.hpp"

/// @brief Class used to store parameters of interest using simple text files and IO.
/**
 * This class logs the robot-related metrics (positions, roles, battery, etc.) and
 * network metrics into two different files.
 * 
 * With respect to network files, the next order should be following to keep formatting:
 * write_entry_alive -> write_entry_coverage -> write_entry_packets -> write_entry_packetsAP ->
 * write_entry_negotiation
 */
class LoggerClass
{
private:
    std::string bag_path;
    std::ofstream file_handler_robotics_, file_handler_network_, file_handler_negotiation_;
public:
    //! Explicit parametrized constructor.
    /*!
        The constructor creates a text file in the path marked by bag_path.
        \param log_path: Absolute system path where the stats folders/files will be stored.
    */
    explicit LoggerClass(const std::string &log_path)
    {
        std::string robots_file = log_path + "_robots.txt";
        std::string network_file = log_path + "_network.txt";
        std::string negotiation_file = log_path + "_negotiation.txt";
        /* Attempts to open file for robotic metrics */
        file_handler_robotics_.open(robots_file, std::ios::out | std::ios::trunc);
        assert(file_handler_robotics_.is_open());
        /* Attempts to open file for network metrics. */
        file_handler_network_.open(network_file, std::ios::out | std::ios::trunc);
        assert(file_handler_network_.is_open());
        /* Attempts to open file for negotiation logs. */
        file_handler_negotiation_.open(negotiation_file, std::ios::out | std::ios::trunc);
        assert(file_handler_negotiation_.is_open());
        /* Succeed, so create the first line. */
        file_handler_robotics_<< "Id " << "X " << "Y " << "T " << "BAT " << "ROLE " << "TS\n";
        /* Create the first line. */
        file_handler_network_ << "ciclos\t" << "rotos\t" << "rel_cove\t" <<	 "cov\t" <<	"uniform\t"	<<  "mean_dist\t";
        file_handler_network_ << "alive\t" << "mean_power\t" << "mean_snd\t" << "mean_rx\t" << "st_dev_sent\t"; 
        file_handler_network_<< "st_dev_rx\t" << "min_sent\t" << "max_sent\t" << "meantx_noAP\t" << "devtx_noAP\t";
        file_handler_network_ << "min_sent_noAP\t" << "max_sent_noAP" << "acc_negotiation\t" << "acc_total\t" << "connected\n";
        file_handler_network_.flush();
        file_handler_network_ << setiosflags(std::ofstream::fixed);
        file_handler_network_.precision(8);
    }
    //! Closes the logging file and synchronizes the changes to hard disk.
    virtual ~LoggerClass(){
        file_handler_robotics_.flush();
        file_handler_robotics_.close();
        file_handler_network_.flush();
        file_handler_network_.close();
    }

    //! Add new entry to the logging file.
    /*!
        This function writes a new entry to the robotics logging file.
        @param id: robot id
        @param x: floating point robot position (X coordenate)
        @param y: floating point robot position (Y coordenate)
        @param t: floating point robot orientation (yaw)
        @param bat: double-precision float with current battery level (0.0-100.0%)
        @param role: Enumeration of the RoleTypes
        @param ts: Simulation timestamp
    */
    void write_entry_robots(const int16_t &id, const float &x, const float &y, const float &t, const float &bat, const RoleTypes &role, const time_t &ts)
    {
        assert(file_handler_robotics_.is_open());
        file_handler_robotics_ << id << " " << x  << " " << y << " " << t << " " << bat << " " << role << " " << ts << "\n";
        file_handler_robotics_.flush();
    }
    /// @brief Write entry to network logging file. This should be the first call.
    /// @param accum_cycles 
    /// @param num_rotos 
    void write_entry_alive(const double &accum_cycles, const int &num_rotos)
    {
        assert(file_handler_network_.is_open());
        file_handler_network_ << accum_cycles << "\t" << num_rotos << "\t";
        file_handler_network_.flush();
    }
    /// @brief Write entry to network logging file. It must be called after write_entry_alive
    /// @param relative_coverage 
    /// @param total_coverage 
    /// @param uniform 
    /// @param mean_dist 
    /// @param robots_alive 
    /// @param mean_power 
    void write_entry_coverage(const double &relative_coverage, const double &total_coverage, const double &uniform, const double &mean_dist, const double &robots_alive, const double &mean_power)
    {
        assert(file_handler_network_.is_open());
        file_handler_network_ << relative_coverage << "\t" << total_coverage << "\t" << uniform << "\t";
        file_handler_network_ << mean_dist << "\t" << robots_alive << "\t" << mean_power << "\t";
        file_handler_network_.flush();
    }
    /// @brief Writes the number of packets sent and received by each agent individually to the negotiation logging file.
    /// @param packets_per_agent 
    void write_entry_individual_packets(const std::vector<int> &packets_per_agent, const double &timestamp)
    {
        assert(file_handler_negotiation_.is_open());
        file_handler_negotiation_ << "Timestamp: " << timestamp << " & packets" << std::endl;
        for (size_t i = 0; i < packets_per_agent.size(); i++)
        {
            file_handler_negotiation_ << packets_per_agent[i] << "\t";
        }
        file_handler_negotiation_ << std::endl;
        file_handler_negotiation_.flush();
    }
    /// @brief Write entry to network logging file. It must be called after write_entry_coverage
    /// @param mean_packets_sent 
    /// @param mean_packets_rx 
    /// @param st_dev_packets_sent 
    /// @param st_dev_packets_rx 
    /// @param min_packets_sent 
    /// @param max_packets_sent 
    void write_entry_packets(const double &mean_packets_sent, const double &mean_packets_rx, const double &st_dev_packets_sent, const double &st_dev_packets_rx, const double &min_packets_sent, const double &max_packets_sent)
    {
        assert(file_handler_network_.is_open());
        file_handler_network_<< mean_packets_sent << "\t" << mean_packets_rx << "\t" << st_dev_packets_sent << "\t";
        file_handler_network_ << st_dev_packets_rx << "\t" << min_packets_sent << "\t" << max_packets_sent << "\t";
        file_handler_network_.flush();
    }
    /// @brief Write entry to network logging file. It must be called after write_entry_packetsAP
    /// @param mean_packets_sent_noAP 
    /// @param st_dev_packets_sent_noAP 
    /// @param min_packets_sent_noAP 
    /// @param max_packets_sent_noAP 
    void write_entry_packetsAP(const double &mean_packets_sent_noAP, const double &st_dev_packets_sent_noAP, const double &min_packets_sent_noAP, const double max_packets_sent_noAP)
    {
        assert(file_handler_network_.is_open());
        file_handler_network_ << mean_packets_sent_noAP << "\t" << st_dev_packets_sent_noAP << "\t";
        file_handler_network_ << min_packets_sent_noAP << "\t" << max_packets_sent_noAP << "\t";
        file_handler_network_.flush();
    }

    /// @brief Write entry to network logging file, for negotiation metrics. This should be the last entry call.
    /// @param total_acc_packet_negotiation 
    /// @param total_acc_packet_total 
    /// @param num_alive_connected 
    void write_entry_negotiation(const double &total_acc_packet_negotiation, const double &total_acc_packet_total, const double &num_alive_connected)
    {
        assert(file_handler_network_.is_open());
        file_handler_network_ << total_acc_packet_negotiation << "\t" << total_acc_packet_total << "\t";
        file_handler_network_ << num_alive_connected << "\t" << std::endl;
        file_handler_network_.flush();
    }

    /// @brief Write entry to the negotiation to log the negotiation result
    /// @param s Text string to write to.
    void write_entry_negotiation_result(const std::string &s)
    {
        assert(file_handler_negotiation_.is_open());
        file_handler_negotiation_ << s << std::endl;
        file_handler_negotiation_.flush();
    }

    /// @brief Write entry containing the willingness for each robot (indistinctly whether they are alive or not), one column for robot
    /// @param timestamp timestamp for this dump
    /// @param w Willingness vector
    void write_entry_willingness(const std::vector<double> &w, const double &timestamp)
    {
        assert(file_handler_negotiation_.is_open());
        file_handler_negotiation_ << "Timestamp: " << timestamp << " & w" << std::endl;
        for (size_t i = 0; i < w.size(); i++)
        {
            file_handler_negotiation_ << w[i] << " ";
        }
        file_handler_negotiation_ << std::endl;
        file_handler_negotiation_.flush();
    }

};

#endif
