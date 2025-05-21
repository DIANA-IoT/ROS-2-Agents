// Project Title: ROS2_AGENTS
// File: include/ros2_agents/supervisor.hpp
// Author: José-Borja Castillo-Sánchez, DIANA Group UMA
// Date: 2025
// (c) Copyright by Universidad de Málaga
// License This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#ifndef __Supervisor_HPP__
#define __Supervisor_HPP__

#include <condition_variable>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
/* POSIX semaphore */
#include <semaphore.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "ros2_agents/defs.hpp"
#include "ros2_mwsn_msgs/msg/controller2_supervisor.hpp"
#include "ros2_mwsn_msgs/msg/intra_network.hpp"
#include "ros2_mwsn_msgs/msg/position.hpp"
#include "ros2_mwsn_msgs/msg/supervisor2_controller.hpp"
#include "ros2_agents/logger.hpp"
#include "ros2_agents/negotiation_class.hpp"
#include "ros2_agents/robot_controller.hpp"
#include "ros2_agents/robot_logger.hpp"
#include "ros2_agents/visualization_class.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

class RobotLogger;

typedef rclcpp::Clock::SharedPtr clock_ptr;

/// @brief Supervisor class. An unique node in the network, mainly for logging and orchestring negotiations.
/*!
    The Supervisor node is intended to be unique in the network, as it is the one who decides when to start
    the deployment, to control negotiations, and perphaps, most important, logs everything (almost) that happens inside the network.
*/
class Supervisor : public rclcpp::Node, public rclcpp::Context
{
public:
    //! Supervisor default constructor
    explicit Supervisor(const rclcpp::NodeOptions &node_options =
                            rclcpp::NodeOptions());
    virtual ~Supervisor();

private:
    typedef rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_type;
    typedef rclcpp::Subscription<ros2_mwsn_msgs::msg::Controller2Supervisor>::SharedPtr robotFeedback_sub_type;
    /* Callbacks. */
    /// @brief callback for IntraNetwork meesages
    /// @param msg received message.
    void intra_msg_callback(const ros2_mwsn_msgs::msg::IntraNetwork &msg);
    // @brief Position subscription callback, common to all robots.
    void position_callback(const ros2_mwsn_msgs::msg::Position &msg);
    /// @brief Callback to attend the timeout of each negotiation class.
    void negotiation_timeout_callback(const int16_t &id);
    /// @brief Callback to handle Controller2Supervisor messages on topic /robot_feedback
    /// @param msg The message that contains the status of each robot.
    void robot_callback(const ros2_mwsn_msgs::msg::Controller2Supervisor &msg);
    /// @brief Periodic callback to check the MWSAN status
    /*!
        This callback is responsible to check number of alive nodes, so the simulation / control
        round will be finished when a certain amount of nodes have died (> 70 %).
        In this callback, it is also checked the number of stationary nodes, alongside the current
        status of the simulation.
        This allows to implement a FSM-alike behaviour that could ease the management of the three
        phases of the network:
            1. Deployment.
            2. Maintenance.
            3. Retrieving.
        TODO: Update this
    */
    void sim_control_callback(void);
    /// @brief Periodic callback to simulate a report of the battery consumption due to transceiver operations.
    /*!
        This callbacks simulates a report of battery consumption modelled as constant battery substraction.
        As of future work, this callback should be either removed or send more realistic data depending on the
        network topology.
    */
    void update_radio_callback(void);
    /// @brief Periodic callback to update RViz visualization data.
    /*!
        Periodic callback to update RViz visualization data. This function should not be called too frequently as it can overload with use
        the system (due to copy/iterations through the RMW) or the network (published messages can be heavy).
    */
    void visualization_timeout_callback(void);
    /* End: Callbacks. */
    /* Functions: */
    /// @brief Function a robot asks to be replaced.
    /// @param robot_id The robot who needs help.
    void ask_for_help(const int16_t &robot_id);
    /// @brief Auxiliary function to assign a random battery level to every node at system startup.
    void assign_battery_levels(void);
    /// @brief 
    /// @param  
    /// @return 
    double calculate_coverage(const std::vector<int> &connected);
    /// @brief Calculate mean power to transmit from one robot to the others
    /// @param my_robot Robot id who is sending packets.
    /// @return The mean power
    double calculate_meanPower(const int &my_robot);
    /// @brief 
    /// @param my_robot 
    /// @param sender 
    /// @param receiver 
    /// @param fin 
    /// @param tx_power 
    void calculate_potAdHoc(int my_robot, int sender, int receiver, int* fin, double* tx_power);
    /// @brief 
    /// @param  
    /// @return 
    double calculate_uniformity(const std::vector<int> &connected);
    /// @brief This function checks if the treshold of dead nodes has been trespassed, so that control / simulation is ended.
    /// @param dead_nodes The number of dead nodes in the network.
    /*!
        Checks if simulation/control needs to be ended. It does so by comparing the argument dead_nodes with
        a predefined threshoold. If that is true, an special message on sim_status topic its published.
    */
    bool check_end(int16_t dead_nodes);
    /// @brief Clear agent negotiation tasks
    /// @param None
    void clear_tasks(void);
    /// @brief Counts the number of nodes who have marked themselves as stationary.
    /// @return Int16 with the number of stationary nodes.
    int16_t count_stationary(void);
    /// @brief 
    /// @param map_size 
    /// @return 
    int **create_grid(int map_size, const std::vector<int> &connected);
    /// @brief Compute connected robots according to the AdHoc parameters.
    /// @param my_robot
    /// @param connected
    void detect_connectedAdHoc(int my_robot, std::vector<int> &connected);
    /// @brief 
    /// @param robot_id 
    /// @param robots_w robots with positive willingness
    void handle_helpRequest(const int16_t &robot_id, std::vector<std::pair<float, int>> &robots_w);
    /// @brief Initialize performance vector for the Network Simulator. Vectors should be already allocated.
    /// @param None
    void initialize_performaceVectors(void);
    /// @brief In this function, every robot subscription is created, apart from other ROS 2 API utilities.
    void initialize_supervisor(void);
    /// @brief Returns true if more than 80 % of robots are stationary.
    bool is_simulation_stationary(void);
    /// @brief Kills a robot in Stage simulator.
    /// @param id The robot id (/robot{id}) to kill
    void kill_stage_robot(const int16_t &id);
    /// @brief Function to write desired robot metrics into the logging file, if enabled.
    /*!
        This function shall solely be called after the events simulation.
        Plus, ROS 2 time must be replaced by the simulation time.
    */
    void log_robot_metrics(void);
    /// @brief 
    /// @param  
    void negotiation_AP(const int16_t &robot_id);
    /// @brief 
    /// @param robot_id 
    void negotiation_robot(const int16_t &robot_id);
    /// @brief Function where network and battery simulation takes place.
    /// @param None
    void network_simulation(void);
    /// @brief 
    /// @param  
    void network_test(void);
    /// @brief Perform the negotiation when using Agents.
    /// @param None
    void perform_negotiation(void);
    /// @brief 
    /// @param robot_id 
    void pick_AP(const int16_t &robot_id);
    /// @brief 
    /// @param robot_id 
    /// @return the chosen robot
    int pick_robot(const int16_t &robot_id);
    /// @brief Simulation only. Commands stage simulator to run as fast as possible.
    /// @param None
    void run_fast(void);
    /// @brief Simulation only. Commands stage simulator to run in real time.
    /// @param None
    void run_realtime(void);
    /// @brief Save network-related performance in a text file
    /// @param None
    void save_performance(void);
    /// @brief Sort the list of robots2help so corrected utility is maximized in the network
    /// @param robots2help vector of robots (w, id) to be helped.
    /// @return The pair of robot2help, robotReplacing to notify the candidates. 
    //std::vector<std::pair<int,int>> sort_maximum(const std::vector<std::pair<float, int>> &robots2help);
    /// @brief Sort the willingness vector in ascending/descending order.
    /*!
        By sorting the willingness in ascending order, numerically lower indexes
        represent the robots with higher urgency to negotiate. Conversely, 
        descending order is used to sort robots with higher disposition to help others.
    */
    /// @param const bool &sort_urgent
    /// @return Vector with willingness (float) plus the robot index (int).
    std::vector<std::pair<float, int>> sort_willingness(const bool &sort_urgent);
    /// @brief Thread function where simulation takes place
    /// @param None
    void simulation_thread(void);
    /// @brief Sets the closest robot to the origin as AP
    /// @param None
    void set_AP(void);
    /// @brief Send an AdHoc message and update statistics.
    /// @param my_robot TODO:
    /// @param sender TODO:
    /// @param receiver TODO:
    /// @param msj_tx TODO: 
    /// @param strategy TODO:
    void send_AdHoc(int my_robot, int sender, int receiver, int strategy);
    /// @brief Auxiliary function to send a broadcast message that commmands robots to deploy if they have enough battery.
    void send_deploy_msg(void);
    /// @brief Command to robot_id to retrieve to the depot location.
    /// @param robot_id robot identifier who shall retrieve.
    void send_depot_msg(const int16_t &robot_id);
    /// @brief Message to finish negotiation.
    void send_end_negotiation_msg(void);
    /// @brief Sends a message to a specific so it replaces a robot in need.
    /// @param origin The robot who needs help
    /// @param destination The identifier to whom the message its directed.
    /// @param x X-position to cover
    /// @param y Y-position to cover
    void send_replace_msg(const int16_t &origin, const int16_t &destination, const float &x, const float &y);
    /// @brief Function that sends a broadcast message to make all robots stationary.
    void send_static_msg(void);
    /// @brief Function that assigns battery levels and then, deploys the robots.
    void start_simulation(void);
    /* End: Functions. */

    sem_t simulation_sem_;
    /*
        Condition variables to advance simulation state.
        Unlocks simulation_thread as the arrival of asynchronous events.
    */
    std::mutex *m;
    std::condition_variable *cv;
    /*
        Data mutexes.
        Provide mutual exclusion to data.
    */
    std::mutex *data_mutex_;

    rclcpp::TimerBase::SharedPtr logger_write_timer_;
    rclcpp::TimerBase::SharedPtr update_radio_timer_;
    rclcpp::TimerBase::SharedPtr visualization_timer_;
    robotFeedback_sub_type robotFeedbackSub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr killRobot_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<ros2_mwsn_msgs::msg::Position>::SharedPtr position_sub_;
    rclcpp::Publisher<ros2_mwsn_msgs::msg::Supervisor2Controller>::SharedPtr robotCommander_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sim_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sim_control_pub_;

    std::chrono::steady_clock::time_point start_time_;

    std::thread *simulation_thread_;
    
    std::vector<int> packets_sent_;
    std::vector<double> wasted_energy_;
    std::vector<int> own_packets_sent_;
    std::vector<int> packets_rx_;
    std::vector<int> own_packets_rx_;
    std::vector<double> net_mean_power_;
    std::vector<int> acc_packet_negotiation_;
    std::vector<int> acc_packet_total_;

    std::vector<bool> msg_tx;
    std::vector<int> msgs_exchanged_;
    std::vector<bool> RobotDeadList_;
    std::vector<int> packets_per_agent_;
    std::vector<double> giving_help_;
    std::vector<int> requested_help_;
    std::vector<int> started_negotiation_;
    std::vector<int> tasks_assigned_;
    std::vector<float> xpos_, ypos_;
    // Battery publication, indicates how much battery needs to be substracted to current level
    // Represented by: /roboti/battery_drain
    std::vector<rclcpp::Publisher<ros2_mwsn_msgs::msg::Supervisor2Controller>::SharedPtr> battery_pub_;
    /// /roboti/battery subscription
    std::vector<rclcpp::Subscription<ros2_mwsn_msgs::msg::Controller2Supervisor>::SharedPtr> RobotBatSub_;
    std::vector<RobotLogger *> RobotLoggerList_;
    // std::vector<NegotiationAP *>RobotNegotiationList_;
    std::vector<initial_pose_sub_type> robotIPSubList_;

    std::vector<double> w, w_h, w_h2;
    std::vector<double> previous_step_w;
    enum class NetworkStates
    {
        DEPLOYMENT,
        IDLE,
        NEGOTIATION,
        RELOCATE
    };
    Visualization *visualization_;

    /* Parameters */
    int algorithm_;
    std::string bag_path;
    bool use_logger_;
    int n_robots_;
    /* Internal variables */
    int negotiation_concluded = 0;
    int total_negotiations = 0;
    int total_negotiations_AP = 0;
    int unfinished_negotiations = 0;
    int unfinished_negotiations_AP = 0;
    int16_t alive_remaining_;
    int16_t indexAP;
    int16_t robot2help;
    LoggerClass *logger = nullptr;
    NetworkStates current_state_;
    double simulation_time_; /**< Simulation time, different to wall or ROS time. This is the one to be included in the logging file.*/
    SupervisorTypes supervisor_type_;
};

#endif