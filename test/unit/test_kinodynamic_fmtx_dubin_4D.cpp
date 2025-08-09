// Copyright 2025 Soheil E.nia

#include "motion_planning/planners/planner_factory.hpp"
#include "motion_planning/state_space/dubins_time_statespace.hpp" // Changed
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/utils/parse_sdf.hpp"
#include "motion_planning/utils/ros2_manager_dubin.hpp" // Changed
#include "motion_planning/utils/rviz_visualization.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>
#include <iostream>
#include <thread>
#include <valgrind/callgrind.h>

struct LogEntry {
    double elapsed_s = 0.0;
    double duration_ms = 0.0;
    double time_to_goal = 0.0; // The time-to-go of the robot's path
    double path_cost = 0.0;
    int obstacle_checks = 0;
    long long rewire_neighbor_searches = 0;
    int orphaned_nodes = 0;
    int collision_count = 0;
};


std::atomic<bool> g_running{true};

void sigint_handler(int sig) {
    g_running = false;
}


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

int main(int argc, char** argv) {
    // --- Initial Setup ---
    rclcpp::init(argc, argv);
    signal(SIGINT, sigint_handler);


    int num_samples = 5000;
    double factor = 1.5;
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

    // --- Parameter Setup ---
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
    gazebo_params.setParam("bullet", false);

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
    planner_params.setParam("precache_neighbors", true);
    planner_params.setParam("kd_dim", 4); // 2 or 3 or 4 only dubin
    planner_params.setParam("use_knn", false);
    planner_params.setParam("mode", 1); //1: prune false 2: prune true

    // --- Object Initialization ---
    auto vis_node = std::make_shared<rclcpp::Node>("fmtx_dubins_visualizer",
        rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
    auto visualization = std::make_shared<RVizVisualization>(vis_node);
    auto sim_clock = vis_node->get_clock();

    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_many_constant_acc_uncrowded.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_many_constant_acc.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_straight_box.sdf");
    auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_straight_box_circle.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_straight.sdf");
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(sim_clock, gazebo_params, obstacle_info);

    // --- Planner and Problem Definition (4D Dubins) ---
    const int dim = 4;
    auto problem_def = std::make_shared<ProblemDefinition>(dim);

    Eigen::VectorXd tree_root_state(4);
    tree_root_state << -48.0, -48.0, -3 * M_PI / 4.0, 0.0; // Goal: x, y, theta, time-to-go
    problem_def->setStart(tree_root_state);

    Eigen::VectorXd robot_initial_state(4);
    robot_initial_state << 48.0, 48.0, M_PI / 4.0, 40.0; // Start: x, y, theta, time budget
    problem_def->setGoal(robot_initial_state);

    Eigen::VectorXd lower_bounds(4), upper_bounds(4);
    lower_bounds << -50.0, -50.0, -M_PI, 0.0;
    upper_bounds << 50.0, 50.0, M_PI, 40.0;
    problem_def->setBounds(lower_bounds, upper_bounds);

    double min_turning_radius = 2.0;
    double min_velocity = 1.0;
    double max_velocity = 10.0;
    auto statespace = std::make_shared<DubinsTimeStateSpace>(min_turning_radius, min_velocity, max_velocity, seed);

    auto ros_manager = std::make_shared<DubinsROS2Manager>(obstacle_checker, visualization, manager_params, robot_initial_state);
    auto planner = PlannerFactory::getInstance().createPlanner(PlannerType::KinodynamicFMTX, statespace, problem_def, obstacle_checker);
    auto kinodynamic_planner = dynamic_cast<KinodynamicFMTX*>(planner.get());
    kinodynamic_planner->setClock(sim_clock);
    planner->setup(planner_params, visualization);

    // --- Initial Plan ---
    std::vector<Eigen::VectorXd> current_executable_path;
    RCLCPP_INFO(vis_node->get_logger(), "Running initial plan...");
    // obstacle_checker->getAtomicSnapshot();
    planner->plan();
    kinodynamic_planner->setRobotState(robot_initial_state);
    current_executable_path = kinodynamic_planner->getPathPositions();
    if (!current_executable_path.empty()) {
        ros_manager->setPath(current_executable_path);
    }
    RCLCPP_INFO(vis_node->get_logger(), "Initial plan complete. Executing...");

    auto tree_vis_timer = vis_node->create_wall_timer(
        std::chrono::milliseconds(100),
        [&]() { if (kinodynamic_planner) kinodynamic_planner->visualizeTree(); });

    // --- Executor Setup ---
    // rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::executors::StaticSingleThreadedExecutor executor; // +++ ADD THIS

    executor.add_node(ros_manager);
    executor.add_node(vis_node); // for dubin i do not plot the edges based on trajecotry because thats too demanding. i just connected the parent to child via simple edge so you might see soem edges going through obstalce but in reality the dubin is going around them so dont be alarm!

    std::thread executor_thread([&executor]() { executor.spin(); });

    // --- Main Execution and Replanning Loop ---
    resetAndPlaySimulation();
    RCLCPP_INFO(vis_node->get_logger(), "Starting execution and monitoring loop. Press Ctrl+C to exit.");
    const double goal_tolerance = 3.0;
    rclcpp::Rate loop_rate(20);

    std::vector<double> sim_durations;
    std::vector<std::tuple<double, double>> sim_duration_2;


    bool limited = true; 
    if (manager_params.getParam<bool>("follow_path"))
        limited = false;
    

    int counter = 0 ;
    std::vector<LogEntry> log_data;

    auto start_time = std::chrono::steady_clock::now();
    auto time_limit = std::chrono::seconds(run_secs);

    auto global_start = std::chrono::steady_clock::now();


    // Start profiling
    CALLGRIND_START_INSTRUMENTATION;
    while (g_running && rclcpp::ok()) {
        // if (counter > 200)
        //     break;
        // counter++;
        /////////////
        if (limited) {
            auto now = std::chrono::steady_clock::now();
            if (now - start_time > time_limit) {
                std::cout << "[INFO] time_limit seconds have passed. Exiting loop.\n";
                break;  // exit the loop
            }
        }
        /////////////


        // --- Read robot state once ---
        Eigen::VectorXd current_state = ros_manager->getCurrentSimulatedState();
        kinodynamic_planner->setRobotState(current_state);

        if (current_state.size() == 0) {
            loop_rate.sleep();
            continue;
        }

        // --- Check goal reached ---
        double dist_to_goal = (current_state.head<2>() - tree_root_state.head<2>()).norm();
        if (dist_to_goal < goal_tolerance) {
            RCLCPP_INFO(vis_node->get_logger(), "Goal Reached! Mission Accomplished.");
            g_running = false;
            break;
        }

        // --- Update obstacle snapshot ---
        auto snapshot = obstacle_checker->getAtomicSnapshot();
        auto start = std::chrono::steady_clock::now();

        bool obstacles_changed = kinodynamic_planner->updateObstacleSamples(snapshot.obstacles);

        current_state = ros_manager->getCurrentSimulatedState();
        kinodynamic_planner->setRobotState(current_state);
        // --- Reactive safety check ---
        bool path_invalid = !kinodynamic_planner->isPathStillValid(current_executable_path, current_state);

        // --- Decide whether to replan ---
        if (path_invalid || obstacles_changed) {
            RCLCPP_INFO(vis_node->get_logger(), "Triggering replan (invalid path: %s, obstacles changed: %s)",
                        path_invalid ? "yes" : "no",
                        obstacles_changed ? "yes" : "no");

            current_state = ros_manager->getCurrentSimulatedState();
            kinodynamic_planner->setRobotState(current_state);
            // Execute replanning
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

  
            Eigen::VectorXd fresh_robot_state = ros_manager->getCurrentSimulatedState();
            kinodynamic_planner->setRobotState(fresh_robot_state);
            auto new_path = kinodynamic_planner->getPathPositions();


            ////----------
            const auto& metrics = kinodynamic_planner->getLastReplanMetrics();

            entry.obstacle_checks = metrics.obstacle_checks;
            entry.rewire_neighbor_searches = metrics.rewire_neighbor_searches;
            entry.orphaned_nodes = metrics.orphaned_nodes;
            entry.path_cost = metrics.path_cost; // <-- Get cost directly from metrics
            entry.time_to_goal = kinodynamic_planner->getRobotTimeToGo(); // This is still separate and correct

            log_data.push_back(entry);
            ////-----------

            //////////////////////////
            if (new_path.empty()) {
                // Failure: stop in place
                RCLCPP_ERROR(vis_node->get_logger(), "Replanning failed. Engaging E-STOP.");
                current_executable_path = { current_state };
            } else {
                // Success: only update if meaningfully different
                if (!kinodynamic_planner->arePathsSimilar(current_executable_path, new_path, 0.1)) {
                    RCLCPP_INFO(vis_node->get_logger(), "Updating to new optimal path.");
                    current_executable_path = new_path;
                } else {
                    RCLCPP_INFO(vis_node->get_logger(), "New path similar to current. Keeping existing.");
                }
            }
            ros_manager->setPath(current_executable_path);
        }

        kinodynamic_planner->visualizePath(current_executable_path);

        loop_rate.sleep();
    }
    // Stop profiling
    CALLGRIND_STOP_INSTRUMENTATION;


    const int final_collision_count = ros_manager->getCollisionCount();
    RCLCPP_FATAL(vis_node->get_logger(), "SIMULATION COMPLETE. TOTAL DETECTED COLLISIONS: %d", final_collision_count);
    for (auto& entry : log_data) {
        entry.collision_count = final_collision_count;
    }

    std::time_t now_time = std::time(nullptr);
    std::tm* local_tm = std::localtime(&now_time);
    char time_buf[80];
    strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", local_tm);

    int num_of_samples_val = planner_params.getParam<int>("num_of_samples");
    std::string filename = "sim_fmtx_" + std::to_string(num_of_samples_val) + 
                           "samples_" + time_buf + "_metrics.csv";

    std::cout << "Writing replan metrics to: " << filename << std::endl;

    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cerr << "Error: failed to open " << filename << std::endl;
        return 1;
    }

    out << "elapsed_s,duration_ms,time_to_goal,path_cost,obstacle_checks,rewire_neighbor_searches,orphaned_nodes,collision_count\n";
    
    for (const auto& entry : log_data) {
        out << entry.elapsed_s << ","
            << entry.duration_ms << ","
            << entry.time_to_goal << ","
            << entry.path_cost << ","
            << entry.obstacle_checks << ","
            << entry.rewire_neighbor_searches << ","
            << entry.orphaned_nodes << ","
            << entry.collision_count << "\n";
    }
    out.close();

    std::cout << "Done writing CSV.\n";
    




    // --- Graceful Shutdown ---
    RCLCPP_INFO(vis_node->get_logger(), "Shutting down.");
    g_running = false;
    executor.cancel();
    if (executor_thread.joinable()) {
        executor_thread.join();
    }
    rclcpp::shutdown();
    return 0;
}
