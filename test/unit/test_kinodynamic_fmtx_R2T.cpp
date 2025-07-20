// Copyright 2025 Soheil E.nia

// Includes
#include "motion_planning/planners/planner_factory.hpp"
#include "motion_planning/state_space/rdt_statespace.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/utils/parse_sdf.hpp"
#include "motion_planning/utils/ros2_manager_r2t.hpp" // Use your new manager
#include "motion_planning/utils/rviz_visualization.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>
#include <iostream>
#include <thread>
#include <valgrind/callgrind.h>



struct LogEntry {
    double elapsed_s = 0.0;
    double duration_ms = 0.0;
    double time_to_goal = 0.0;
    double path_cost = 0.0;
    int obstacle_checks = 0;
    long long rewire_neighbor_searches = 0;
    int orphaned_nodes = 0;
};

// Define a global running flag for signal handling
std::atomic<bool> g_running{true};

// Signal handler to gracefully shut down
void sigint_handler(int sig)
{
    g_running = false;
}

// Gazebo simulation control function (remains unchanged)

void resetAndPlaySimulation()
{
    gz::transport::Node node;
    {
        gz::msgs::WorldControl reset_req;
        reset_req.mutable_reset()->set_all(true);
        gz::msgs::Boolean reset_res;
        bool result;
        unsigned int timeout = 3000; // ms
        
        bool executed = node.Request("/world/default/control", 
                                   reset_req,
                                   timeout,
                                   reset_res,
                                   result);
        
        if (!executed || !result || !reset_res.data()) {
            std::cerr << "Failed to reset world" << std::endl;
            return;
        }
        std::cout << "World reset successfully" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    {
        gz::msgs::WorldControl play_req;
        play_req.set_pause(false);
        
        gz::msgs::Boolean play_res;
        bool result;
        
        bool executed = node.Request("/world/default/control",
                                   play_req,
                                   3000,
                                   play_res,
                                   result);
        
        if (!executed || !result || !play_res.data()) {
            std::cerr << "Failed to play simulation" << std::endl;
            return;
        }
        std::cout << "Simulation playing successfully" << std::endl;
    }
}


int main(int argc, char** argv)
{
    // --- 1. Initial Setup ---
    rclcpp::init(argc, argv);
    signal(SIGINT, sigint_handler);


    // 1) Parse your flags
    int num_samples = 3000;
    double factor = 2.0;
    unsigned int seed = 42;
    int run_secs = 30;


    for(int i = 1; i < argc; ++i) {
        std::string s{argv[i]};
        if(s == "--samples" && i+1 < argc) {
        num_samples = std::stoi(argv[++i]);
        }
        else if(s == "--factor" && i+1 < argc) {
        factor = std::stod(argv[++i]);
        }
        else if(s == "--seed" && i+1 < argc) {
        seed = std::stoi(argv[++i]);
        }
        else if(s == "--duration" && i+1 < argc) {
        run_secs = std::stoi(argv[++i]);
        }
        else if(s == "--help") {
        std::cout << "Usage: " << argv[0]
                    << " [--samples N] [--factor F] [--seed S] [--duration T]\n";
        return 0;
        }
    }


    // --- 2. Parameter Setup ---
    // Encapsulate parameters for better organization
    Params manager_params;
    manager_params.setParam("use_sim_time", true);
    manager_params.setParam("sim_time_step", -0.04); // Time-to-go consumed per sim step
    manager_params.setParam("sim_frequency_hz", 50);  // Smoothness of arrow
    manager_params.setParam("vis_frequency_hz", 10);  // Obstacle visualization rate
    manager_params.setParam("follow_path", true);

    Params gazebo_params;
    gazebo_params.setParam("robot_model_name", "tugbot");
    gazebo_params.setParam("default_robot_x", 48.0); // in case you want to test the planner without running gz sim
    gazebo_params.setParam("default_robot_y", 48.0);
    gazebo_params.setParam("world_name", "default");
    gazebo_params.setParam("use_range", false); // use_range and partial_update and use_heuristic are related! --> take care of this later!
    gazebo_params.setParam("sensor_range", 20.0);
    gazebo_params.setParam("estimation", true);
    gazebo_params.setParam("kf_model_type", "cv");
    gazebo_params.setParam("fcl", false);

    // gazebo_params.setParam("inflation", 0.0); //2.0 meters --> this will be added to obstalce radius when obstalce checking --> minimum should be D-ball containing the robot
    // This value is CRITICAL. If it's 0.0, your robot has no size.
    // Set it to a value representing your robot's radius + a safety margin.
    // For example, if your robot is 1 meter wide, a radius of 0.5m + a buffer of 1m = 1.5.
    gazebo_params.setParam("inflation", 0.5); // <-- VERIFY THIS IS A REASONABLE, NON-ZERO VALUE
    gazebo_params.setParam("persistent_static_obstacles", false);



    Params planner_params;
    planner_params.setParam("num_of_samples", num_samples);
    planner_params.setParam("factor", factor);
    planner_params.setParam("use_kdtree", true);
    planner_params.setParam("kdtree_type", "NanoFlann");
    planner_params.setParam("partial_update", true);
    planner_params.setParam("static_obs_presence", false);
    planner_params.setParam("obs_cache", false);
    planner_params.setParam("partial_plot", false);
    planner_params.setParam("use_heuristic", false);
    planner_params.setParam("ignore_sample", false);
    planner_params.setParam("prune", false);
    planner_params.setParam("kd_dim", 3); // 2 or 3 only for R2T

    planner_params.setParam("precache_neighbors", true);
    // --- 3. Object Initialization ---
    // A single node is shared for visualization purposes
    auto vis_node = std::make_shared<rclcpp::Node>("fmtx_visualizer",
        rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
    
    auto visualization = std::make_shared<RVizVisualization>(vis_node);
    auto sim_clock = vis_node->get_clock();

    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_4_obs.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_many.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_many_constant_acc.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_many_constant_acc_uncrowded.sdf");
    auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_straight_box.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_straight.sdf");
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(sim_clock, gazebo_params, obstacle_info);


    // --- 4. Planner and Problem Definition ---
    const int dim = 3;
    const int spatial_dim = 2;
    auto problem_def = std::make_shared<ProblemDefinition>(dim);
    
    Eigen::VectorXd tree_root_state(3);
    tree_root_state << -48.0, -48.0, 0.0; // Destination: x, y, time-to-go
    problem_def->setStart(tree_root_state); // "Start" of the backward search

    Eigen::VectorXd robot_initial_state(3);
    robot_initial_state << 48.0, 48.0, 30.0; // Initial robot state: x, y, total time budget
    problem_def->setGoal(robot_initial_state); // "Goal" of the backward search

    Eigen::VectorXd lower_bounds(3), upper_bounds(3);
    lower_bounds << -50.0, -50.0, 0.0;
    upper_bounds << 50.0, 50.0, 30.0; // Max time-to-go for any sample
    problem_def->setBounds(lower_bounds, upper_bounds);


    double min_velocity = 0.0;
    double max_velocity = 15.0;
    double robot_velocity = 10.0;
    // Create the single, consolidated R2TROSManager
    auto ros_manager = std::make_shared<R2TROS2Manager>(obstacle_checker, visualization, manager_params,robot_velocity, robot_initial_state);
    auto statespace = std::make_shared<RDTStateSpace>(spatial_dim, min_velocity , max_velocity , robot_velocity, 30000, seed);
    auto planner = PlannerFactory::getInstance().createPlanner(PlannerType::KinodynamicFMTX, statespace, problem_def, obstacle_checker);
    
    auto kinodynamic_planner = dynamic_cast<KinodynamicFMTX*>(planner.get());
    kinodynamic_planner->setClock(sim_clock);
    planner->setup(planner_params, visualization);


    std::vector<Eigen::VectorXd> current_executable_path;

    // --- 5. Perform the INITIAL Plan ---
    RCLCPP_INFO(vis_node->get_logger(), "Running initial plan...");
    // obstacle_checker->getAtomicSnapshot();
    auto start = std::chrono::steady_clock::now();

    planner->plan();
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    if (duration.count() > 0) {
        std::cout << "time taken for the initial plan : " << duration.count() 
                << " milliseconds\n";
    }

    // kinodynamic_planner->printCacheStatus();


    // Anchor the robot to the initial plan
    kinodynamic_planner->setRobotState(robot_initial_state);
    
    // --- FIX: Assign to the variable, don't re-declare it with 'auto' ---
    current_executable_path = kinodynamic_planner->getPathPositions();
    if (!current_executable_path.empty()) {
        ros_manager->setPath(current_executable_path);
    }
    RCLCPP_INFO(vis_node->get_logger(), "Initial plan complete. Executing...");



    // ================== NEW CODE: CREATE A DEDICATED VISUALIZATION TIMER ==================
    const int tree_visualization_hz = 10; // Visualize the tree only 2 times per second.
    auto tree_vis_timer = vis_node->create_wall_timer(
        std::chrono::milliseconds(1000 / tree_visualization_hz),
        [&kinodynamic_planner]() { // Use a lambda to call the visualizeTree function
            if (kinodynamic_planner) {
                kinodynamic_planner->visualizeTree();
            }
        });



    // --- 6. Set Up Executor (Unchanged) ---
    // rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::executors::StaticSingleThreadedExecutor executor; // +++ ADD THIS

    executor.add_node(ros_manager);
    executor.add_node(vis_node); // **IMPORTANT**: Add the vis_node to the executor so its timer runs!

    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    // --- 7. Main Execution and Replanning Loop ---
    resetAndPlaySimulation();
    RCLCPP_INFO(vis_node->get_logger(), "Starting execution and monitoring loop. Press Ctrl+C to exit.");
    const double goal_tolerance = 0.5; // How close to (0,0) counts as "reached", in meters.
    std::vector<double> sim_durations;
    std::vector<std::tuple<double, double>> sim_duration_2;


    bool limited = true; 
    if (manager_params.getParam<bool>("follow_path"))
        limited = false;
    auto start_time = std::chrono::steady_clock::now();
    auto time_limit = std::chrono::seconds(run_secs);


    std::vector<LogEntry> log_data;
    auto global_start = std::chrono::steady_clock::now();
    rclcpp::Rate loop_rate(20); // Frequency to check for replan triggers
    // Start profiling
    int counter = 0;

    CALLGRIND_START_INSTRUMENTATION;
    while (g_running && rclcpp::ok())
    {
        // if (counter > 200)
        //     break;
        // counter++;
        /////////////
        if (limited) {
            auto now = std::chrono::steady_clock::now();
            if (now - start_time > time_limit) {
                std::cout << "[INFO] time_limit seconds have passed. Exiting loop.\n";
                // break;  // exit the loop
            }
        }
        /////////////

        bool needs_replan = false;
        
        // Get the robot's current state ONCE per cycle.
        Eigen::VectorXd current_sim_state = ros_manager->getCurrentSimulatedState();
        kinodynamic_planner->setRobotState(current_sim_state);
        if (current_sim_state.size() == 0) { // Wait for the simulation to initialize
             loop_rate.sleep();
             continue;
        }
        // Calculate the 2D distance to the goal using the tree_root_state variable.
        double distance_to_goal = (current_sim_state.head<2>() - tree_root_state.head<2>()).norm();

        if (distance_to_goal < goal_tolerance) {
            RCLCPP_INFO(vis_node->get_logger(), "Goal Reached! Mission Accomplished.");
            g_running = false; // Set the flag to cleanly exit the loop.
            continue;          // Skip the rest of this loop iteration.
        }

        const auto& snapshot = obstacle_checker->getAtomicSnapshot();

        // --- TRIGGER 1 (Reactive): Is my current path predictively safe? ---
        // Pass both the path and the robot's current state to the validator.
        // if (!kinodynamic_planner->isPathStillValid(current_executable_path, current_sim_state)) {
        //     RCLCPP_INFO(vis_node->get_logger(), "Current path is no longer predictively valid! Triggering replan.");
        //     needs_replan = true;
        // }

        // --- TRIGGER 2 (Proactive): Have obstacles changed in a significant way? ---
        // if (!needs_replan && kinodynamic_planner->updateObstacleSamples(snapshot.obstacles)) {
        //     RCLCPP_INFO(vis_node->get_logger(), "Obstacle change detected! Proactively replanning...");
        //     needs_replan = true;
        // }
        auto start = std::chrono::steady_clock::now();
        kinodynamic_planner->updateObstacleSamples(snapshot.obstacles);
        // if (needs_replan) {
            // Get the robot's current state to plan FROM.
            // current_sim_state = ros_manager->getCurrentSimulatedState();
            // kinodynamic_planner->setRobotState(current_sim_state);
            planner->plan();
            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            if (duration.count() > 0) {
                std::cout << "time taken for the update : " << duration.count() 
                        << " milliseconds\n";
            }
            sim_durations.push_back(duration.count());
            double elapsed_s = std::chrono::duration<double>(start - global_start).count();
            double duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
            sim_duration_2.emplace_back(elapsed_s, duration_ms);

            ///////---------
            LogEntry entry;
            entry.elapsed_s = std::chrono::duration<double>(start - global_start).count();
            entry.duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
            //////---------
  

            // Re-run the full planning pipeline
            current_sim_state = ros_manager->getCurrentSimulatedState();
            kinodynamic_planner->setRobotState(current_sim_state);
            auto new_executable_path = kinodynamic_planner->getPathPositions();
            
            ////----------
            // Get the complete metrics struct AFTER plan and setRobotState are done
            const auto& metrics = kinodynamic_planner->getLastReplanMetrics();

            // Populate the log entry
            entry.obstacle_checks = metrics.obstacle_checks;
            entry.rewire_neighbor_searches = metrics.rewire_neighbor_searches;
            entry.orphaned_nodes = metrics.orphaned_nodes;
            entry.path_cost = metrics.path_cost; // <-- Get cost directly from metrics
            entry.time_to_goal = kinodynamic_planner->getRobotTimeToGo(); // This is still separate and correct

            log_data.push_back(entry);
            ////-----------



            // *** CORRECTED LOGIC TO HANDLE FAILURE ***
            if (new_executable_path.empty()) {
                // FAILURE CASE: The planner could not find a valid path from the robot's current state.
                RCLCPP_ERROR(vis_node->get_logger(), "Replanning failed! Commanding robot to STOP.");
                
                // Create a "stop" path containing only the robot's current state.
                std::vector<Eigen::VectorXd> stop_path;
                stop_path.push_back(current_sim_state);
                
                // Update the current path and send it to the manager to halt execution.
                current_executable_path = stop_path;
                ros_manager->setPath(current_executable_path);

            } else {
                // SUCCESS CASE: A new path was found.
                // Check if the newly generated path is actually different from the one we're already on.
                if (kinodynamic_planner->arePathsSimilar(current_executable_path, new_executable_path, 0.1)) { // Increased tolerance
                    // RCLCPP_INFO(vis_node->get_logger(), "Replanning resulted in a similar path. No update needed.");
                } else {
                    // RCLCPP_INFO(vis_node->get_logger(), "New optimal path found. Updating trajectory.");
                    // If the path is meaningfully new, update our stored path and send it to the manager.
                    current_executable_path = new_executable_path;
                    ros_manager->setPath(current_executable_path);
                }
            }
        // }

        kinodynamic_planner->visualizePath(current_executable_path);
        // We visualize the tree in every frame, regardless of replanning.
        // kinodynamic_planner->visualizeTree();
        loop_rate.sleep();
    }
    // Stop profiling
    CALLGRIND_STOP_INSTRUMENTATION;

    // while (g_running && rclcpp::ok())
    // {
    //     bool needs_replan = false;
        
    //     // Get the robot's current state ONCE per cycle.
    //     Eigen::VectorXd current_sim_state = ros_manager->getCurrentSimulatedState();
    //     if (current_sim_state.size() == 0) { // Wait for the simulation to initialize
    //          loop_rate.sleep();
    //          continue;
    //     }

    //     // Calculate the 2D distance to the goal using the tree_root_state variable.
    //     double distance_to_goal = (current_sim_state.head<2>() - tree_root_state.head<2>()).norm();

    //     if (distance_to_goal < goal_tolerance) {
    //         RCLCPP_INFO(vis_node->get_logger(), "Goal Reached! Mission Accomplished.");
    //         g_running = false; // Set the flag to cleanly exit the loop.
    //         continue;          // Skip the rest of this loop iteration.
    //     }



    //     auto snapshot = obstacle_checker->getAtomicSnapshot();

    //     // --- TRIGGER 1 (Reactive): Is my current path predictively safe? ---
    //     if (!kinodynamic_planner->isPathStillValid(current_executable_path, current_sim_state)) {
    //         // RCLCPP_INFO(vis_node->get_logger(), "Current path is no longer predictively valid! Triggering replan.");
    //         needs_replan = true;
    //     }

    //     auto start = std::chrono::steady_clock::now();


    //     // --- TRIGGER 2 (Proactive): Have obstacles changed in a significant way? ---
    //     if (!needs_replan && kinodynamic_planner->updateObstacleSamples(snapshot.obstacles)) {
    //         // RCLCPP_INFO(vis_node->get_logger(), "Obstacle change detected! Proactively replanning...");
    //         needs_replan = true;
    //     }

    //     if (needs_replan) {
            
    //         // RCLCPP_INFO(vis_node->get_logger(), "Replanning triggered. Finding new optimal path...");
            
    //         // --- CRITICAL SECTION ---
    //         // Ensure the planner's state is updated with the LATEST robot position before planning.
    //         // We use the `current_sim_state` variable we fetched at the start of this loop.
    //         kinodynamic_planner->setRobotState(current_sim_state);

    //         // Now, run the planner and get the new path.
    //         // getPathPositions will correctly use the state we just set.
    //         planner->plan();
    //         auto new_executable_path = kinodynamic_planner->getPathPositions();
            
    //         // Path comparison and update logic...
    //         if (!new_executable_path.empty()) {
    //             // --- HYSTERESIS CHECK ---
    //             // Only switch to the new path if it's significantly different from the old one.
    //             // A tolerance of 0.1 means waypoints must differ by > 10cm. Tune as needed.
    //             if (!kinodynamic_planner->arePathsSimilar(current_executable_path, new_executable_path, 0.1)) {
    //                 // RCLCPP_INFO(this->get_logger(), "New, different path found. Updating trajectory.");
    //                 current_executable_path = new_executable_path;
    //                 ros_manager->setPath(current_executable_path);
    //                 // kinodynamic_planner->visualizePath(current_executable_path);
    //             } else {
    //                 // Path is essentially the same. Do nothing and continue on the current path for another cycle.
    //                 // RCLCPP_INFO(this->get_logger(), "Replanning resulted in a similar path. Ignoring update.");
    //             }

    //         } else {
    //             // FAILURE CASE: Robot is stranded. Implement evasive maneuver.
    //             // RCLCPP_ERROR(vis_node->get_logger(), "Robot is stranded! Attempting evasive maneuver...");

    //             // --- "SAFETY BUBBLE" LOGIC ---
    //             const int num_escape_directions = 16;
    //             const double probe_distance = 1.0; // How far to look for a safe spot
    //             double max_safe_distance = -1.0;
    //             Eigen::Vector2d best_escape_vector = Eigen::Vector2d::Zero();

    //             for (int i = 0; i < num_escape_directions; ++i) {
    //                 double angle = 2.0 * M_PI * i / num_escape_directions;
    //                 Eigen::Vector2d escape_vector(cos(angle), sin(angle));
                    
    //                 // Calculate a probe point in this direction
    //                 Eigen::Vector2d probe_point = current_sim_state.head<2>() + escape_vector * probe_distance;

    //                 // Find how far this probe point is from any obstacle
    //                 double safe_distance = obstacle_checker->findNearestObstacleDistance(probe_point);

    //                 if (safe_distance > max_safe_distance) {
    //                     max_safe_distance = safe_distance;
    //                     best_escape_vector = escape_vector;
    //                 }
    //             }

    //             if (max_safe_distance > 0.25) { // Only move if we found a reasonably safe direction
    //                 // RCLCPP_WARN(vis_node->get_logger(), "Found safe direction. Executing short evasive move.");
                    
    //                 // Create a short, 2-point path for the maneuver
    //                 std::vector<Eigen::VectorXd> escape_path;
    //                 escape_path.push_back(current_sim_state); // Start from current state

    //                 Eigen::VectorXd escape_goal_state = current_sim_state;
    //                 // Move 0.5 meters in the safest direction
    //                 escape_goal_state.head<2>() += best_escape_vector * 0.5; 
    //                 // Decrease time-to-go slightly to make it a valid forward-in-time segment
    //                 escape_goal_state(2) -= 0.5; // Assume 1 m/s for time calculation
    //                 escape_path.push_back(escape_goal_state);

    //                 current_executable_path = escape_path;
    //                 ros_manager->setPath(current_executable_path);
    //                 // kinodynamic_planner->visualizePath(current_executable_path);

    //             } else {
    //                 // FINAL FALLBACK: If completely trapped, command a stop.
    //                 // RCLCPP_ERROR(vis_node->get_logger(), "No safe escape direction found! Commanding robot to STOP.");
    //                 std::vector<Eigen::VectorXd> stop_path;
    //                 stop_path.push_back(current_sim_state);
    //                 current_executable_path = stop_path;
    //                 ros_manager->setPath(current_executable_path);
    //             }
    //         }
    //     }
    //     auto end = std::chrono::steady_clock::now();
    //     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //     if (duration.count() > 0) {
    //         std::cout << "time taken for the update : " << duration.count() 
    //                 << " milliseconds\n";
    //     }

    //     // Visualization and sleep remain the same.
    //     // kinodynamic_planner->visualizeTree();
    //     loop_rate.sleep();
    // }



    // Get timestamp for a unique filename
    std::time_t now_time = std::time(nullptr);
    std::tm* local_tm = std::localtime(&now_time);
    char time_buf[80];
    strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", local_tm);

    // Create filename
    int num_of_samples_val = planner_params.getParam<int>("num_of_samples");
    std::string filename = "sim_fmtx_" + std::to_string(num_of_samples_val) + 
                           "samples_" + time_buf + "_metrics.csv";

    std::cout << "Writing replan metrics to: " << filename << std::endl;

    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cerr << "Error: failed to open " << filename << std::endl;
        return 1;
    }

    // Write CSV header
    out << "elapsed_s,duration_ms,time_to_goal,path_cost,obstacle_checks,rewire_neighbor_searches,orphaned_nodes\n";
    
    // Write log data
    for (const auto& entry : log_data) {
        out << entry.elapsed_s << ","
            << entry.duration_ms << ","
            << entry.time_to_goal << ","
            << entry.path_cost << ","
            << entry.obstacle_checks << ","
            << entry.rewire_neighbor_searches << ","
            << entry.orphaned_nodes << "\n";
    }
    out.close();

    std::cout << "Done writing CSV.\n";
    





    // --- 8. Graceful Shutdown ---
    RCLCPP_INFO(vis_node->get_logger(), "Shutting down.");
    executor.cancel();
    if (executor_thread.joinable())
    {
        executor_thread.join();
    }
    rclcpp::shutdown();
    return 0;
}