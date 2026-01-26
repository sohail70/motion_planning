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




void resetAndPauseSimulation() {
    gz::transport::Node node;
    gz::msgs::WorldControl reset_req;
    reset_req.mutable_reset()->set_all(true);
    reset_req.set_pause(true); // <--- THIS IS THE KEY: Reset but stay frozen
    
    gz::msgs::Boolean res;
    bool result;
    if (node.Request("/world/default/control", reset_req, 3000, res, result) && result && res.data()) {
        std::cout << "[SYNC] World Reset and Paused at T=0.\n";
    }
}

void unpauseSimulation() {
    gz::transport::Node node;
    gz::msgs::WorldControl play_req;
    play_req.set_pause(false); // <--- Pull the trigger
    
    gz::msgs::Boolean res;
    bool result;
    node.Request("/world/default/control", play_req, 3000, res, result);
    std::cout << "[SYNC] Simulation Started!\n";
}

// Add this near your other helper functions in main.cpp
void stepGazebo(unsigned int steps) {
    gz::transport::Node node;
    gz::msgs::WorldControl step_req;
    
    step_req.set_pause(true);       // IMPORTANT: Stay paused after stepping
    step_req.set_multi_step(steps); // Advance physics by 'steps' iterations
    
    gz::msgs::Boolean res;
    bool result;
    
    // Send request to the default world control topic
    node.Request("/world/default/control", step_req, 1000, res, result);
}

int main(int argc, char** argv) {
    // --- Initial Setup ---
    rclcpp::init(argc, argv);
    signal(SIGINT, sigint_handler);


    int num_samples = 100;
    double factor = 3.0;
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
    manager_params.setParam("vis_frequency_hz", 30);  // Obstacle visualization rate
    manager_params.setParam("follow_path", true);


    double time_budget_ = 40.0;
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
    gazebo_params.setParam("initial_budget_time", time_budget_);
    manager_params.setParam("inflation", gazebo_params.getParam<double>("inflation"));  // Obstacle visualization rate



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

    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_1_obs.sdf");
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
    robot_initial_state << 48.0, 48.0, M_PI / 4.0, time_budget_; // Start: x, y, theta, time budget
    problem_def->setGoal(robot_initial_state);

    Eigen::VectorXd lower_bounds(4), upper_bounds(4);
    lower_bounds << -50.0, -50.0, -M_PI, 0.0;
    upper_bounds << 50.0, 50.0, M_PI, time_budget_;
    problem_def->setBounds(lower_bounds, upper_bounds);

    double min_turning_radius = 2.0;
    double min_velocity = 2.0;
    double max_velocity = 20.0;

    auto ros_manager = std::make_shared<DubinsROS2Manager>(obstacle_checker, visualization, manager_params, robot_initial_state, time_budget_);
    auto statespace = std::make_shared<DubinsTimeStateSpace>(min_turning_radius, min_velocity, max_velocity, seed);
    auto planner = PlannerFactory::getInstance().createPlanner(PlannerType::KinodynamicFMTX, statespace, problem_def, obstacle_checker);

    auto kinodynamic_planner = dynamic_cast<KinodynamicFMTX*>(planner.get());
    kinodynamic_planner->setClock(sim_clock);
    planner->setup(planner_params, visualization);


    kinodynamic_planner->resetMetrics(); 

    // --- Initial Plan ---
    std::vector<Eigen::VectorXd> current_executable_path;
    RCLCPP_INFO(vis_node->get_logger(), "Running initial plan...");

    auto start = std::chrono::steady_clock::now();
    // obstacle_checker->getAtomicSnapshot();
    planner->plan();
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    if (duration.count() > 0) {
        std::cout << "time taken for the initial plan : " << duration.count() 
                << " milliseconds\n";
    }
    kinodynamic_planner->setRobotState(robot_initial_state);
    current_executable_path = kinodynamic_planner->getPathPositions();
    if (!current_executable_path.empty()) {
        ros_manager->setPath(current_executable_path);
    }
    RCLCPP_INFO(vis_node->get_logger(), "Initial plan complete. Executing...");

    // ======================================================
    std::vector<LogEntry> log_data;
    LogEntry initial_entry;
    initial_entry.elapsed_s = 0;
    initial_entry.duration_ms = duration.count();
    
    // Get the metrics accumulated during planner->plan()
    const auto& initial_metrics = kinodynamic_planner->getLastReplanMetrics();
    // At first there is no obstalce we can put zero below! 
    initial_entry.obstacle_checks = initial_metrics.obstacle_checks;
    initial_entry.rewire_neighbor_searches = initial_metrics.rewire_neighbor_searches;
    initial_entry.orphaned_nodes = initial_metrics.orphaned_nodes;
    initial_entry.path_cost = initial_metrics.path_cost;
    
    // For time_to_goal, we use the initial robot state
    initial_entry.time_to_goal = robot_initial_state(robot_initial_state.size() - 1);
    
    // Push this entry to your log vector
    log_data.push_back(initial_entry);
    // ======================================================




    const int tree_visualization_hz = 10; // Visualize the tree only 2 times per second.
    auto tree_vis_timer = vis_node->create_wall_timer(
        std::chrono::milliseconds(1000 / tree_visualization_hz),
        [&kinodynamic_planner]() { // Use a lambda to call the visualizeTree function
            if (kinodynamic_planner) {
                kinodynamic_planner->visualizeTree();
            }
        });


    // --- Main Execution and Replanning Loop ---
    // resetAndPauseSimulation();
    RCLCPP_INFO(vis_node->get_logger(), "Starting execution and monitoring loop. Press Ctrl+C to exit.");
    const double goal_tolerance = 3.0;

    std::vector<double> sim_durations;
    std::vector<std::tuple<double, double>> sim_duration_2;


    bool limited = true; 
    if (manager_params.getParam<bool>("follow_path"))
        limited = false;
    
    auto start_time = std::chrono::steady_clock::now();
    auto time_limit = std::chrono::seconds(run_secs);

    auto global_start = std::chrono::steady_clock::now();

    int counter = 0 ;


    rclcpp::Rate loop_rate(20);


    // 1. Cast the shared_ptr to the specific Gazebo class
    auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker);
    // 3. Initialize the RRTx timers ONCE
    // STEP 3: Grab the actual "Frozen" positions from Gazebo
    if (gazebo_checker) {
        gazebo_checker->processLatestPoseInfo(0);
        
        // Initialize timers based on the exact Budget (e.g., 25.0)
        double initial_T = robot_initial_state(2); 
        gazebo_checker->initializeDynamicObstacles(initial_T);
        std::cout << "[SYNC] FMTx Timers initialized against frozen physics.\n";
    }


    // --- Executor Setup ---
    // rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::executors::StaticSingleThreadedExecutor executor; // +++ ADD THIS

    executor.add_node(ros_manager);
    executor.add_node(vis_node); // for dubin i do not plot the edges based on trajecotry because thats too demanding. i just connected the parent to child via simple edge so you might see soem edges going through obstalce but in reality the dubin is going around them so dont be alarm!

    // std::thread executor_thread([&executor]() { executor.spin(); });

    // // Start profiling
    // CALLGRIND_START_INSTRUMENTATION;
    // while (g_running && rclcpp::ok()) {
    //     // if (counter > 200)
    //     //     break;
    //     // counter++;
    //     /////////////
    //     if (limited) {
    //         auto now = std::chrono::steady_clock::now();
    //         if (now - start_time > time_limit) {
    //             std::cout << "[INFO] time_limit seconds have passed. Exiting loop.\n";
    //             break;  // exit the loop
    //         }
    //     }
    //     /////////////


    //     // --- Read robot state once ---
    //     Eigen::VectorXd current_state = ros_manager->getCurrentSimulatedState();
    //     kinodynamic_planner->setRobotState(current_state);

    //     if (current_state.size() == 0) {
    //         loop_rate.sleep();
    //         continue;
    //     }

    //     // --- Check goal reached ---
    //     double dist_to_goal = (current_state.head<2>() - tree_root_state.head<2>()).norm();
    //     if (dist_to_goal < goal_tolerance) {
    //         RCLCPP_INFO(vis_node->get_logger(), "Goal Reached! Mission Accomplished.");
    //         g_running = false;
    //         break;
    //     }

    //     // --- Update obstacle snapshot ---
    //     auto snapshot = obstacle_checker->getAtomicSnapshot();
    //     auto start = std::chrono::steady_clock::now();

    //     bool obstacles_changed = kinodynamic_planner->updateObstacleSamples(snapshot.obstacles);

    //     current_state = ros_manager->getCurrentSimulatedState();
    //     kinodynamic_planner->setRobotState(current_state);
    //     // --- Reactive safety check ---
    //     bool path_invalid = !kinodynamic_planner->isPathStillValid(current_executable_path, current_state);

    //     // --- Decide whether to replan ---
    //     if (path_invalid || obstacles_changed) {
    //         RCLCPP_INFO(vis_node->get_logger(), "Triggering replan (invalid path: %s, obstacles changed: %s)",
    //                     path_invalid ? "yes" : "no",
    //                     obstacles_changed ? "yes" : "no");

    //         current_state = ros_manager->getCurrentSimulatedState();
    //         kinodynamic_planner->setRobotState(current_state);
    //         // Execute replanning
    //         planner->plan();

    //         auto end = std::chrono::steady_clock::now();
    //         auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //         if (duration.count() > 0) {
    //             std::cout << "time taken for the update : " << duration.count() 
    //                     << " milliseconds\n";
    //         }
    //         sim_durations.push_back(duration.count());
    //         double elapsed_s = std::chrono::duration<double>(start - global_start).count();
    //         double duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
    //         sim_duration_2.emplace_back(elapsed_s, duration_ms);

    //         ///////---------
    //         LogEntry entry;
    //         entry.elapsed_s = std::chrono::duration<double>(start - global_start).count();
    //         entry.duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
    //         //////---------

  
    //         Eigen::VectorXd fresh_robot_state = ros_manager->getCurrentSimulatedState();
    //         kinodynamic_planner->setRobotState(fresh_robot_state);
    //         auto new_path = kinodynamic_planner->getPathPositions();


    //         ////----------
    //         const auto& metrics = kinodynamic_planner->getLastReplanMetrics();

    //         entry.obstacle_checks = metrics.obstacle_checks;
    //         entry.rewire_neighbor_searches = metrics.rewire_neighbor_searches;
    //         entry.orphaned_nodes = metrics.orphaned_nodes;
    //         entry.path_cost = metrics.path_cost; // <-- Get cost directly from metrics
    //         entry.time_to_goal = kinodynamic_planner->getRobotTimeToGo(); // This is still separate and correct

    //         log_data.push_back(entry);
    //         ////-----------

    //         //////////////////////////
    //         if (new_path.empty()) {
    //             // Failure: stop in place
    //             RCLCPP_ERROR(vis_node->get_logger(), "Replanning failed. Engaging E-STOP.");
    //             current_executable_path = { current_state };
    //         } else {
    //             // Success: only update if meaningfully different
    //             if (!kinodynamic_planner->arePathsSimilar(current_executable_path, new_path, 0.1)) {
    //                 RCLCPP_INFO(vis_node->get_logger(), "Updating to new optimal path.");
    //                 current_executable_path = new_path;
    //             } else {
    //                 RCLCPP_INFO(vis_node->get_logger(), "New path similar to current. Keeping existing.");
    //             }
    //         }
    //         ros_manager->setPath(current_executable_path);
    //     }

    //     kinodynamic_planner->visualizePath(current_executable_path);

    //     loop_rate.sleep();
    // }
    // // Stop profiling
    // CALLGRIND_STOP_INSTRUMENTATION;

    // Start profiling
    CALLGRIND_START_INSTRUMENTATION;


    // std::vector<TurnaroundEvent> current_run_events;

    // --- CONFIGURATION ---
    const double slice_time = 0.02;       // The robot moves this amount every slice (e.g., 0.05s)
    // const double gazebo_max_step = 0.001; // Physics engine step size
    
    // std::cout << "[CONFIG] Slice Time: " << slice_time << "s | Gazebo Step: " << gazebo_max_step << "s" << std::endl;
    std::vector<Eigen::VectorXd> current_viz_path; 
    
    // Variables to track time within the current slice
    double time_accumulated_in_slice = 0.0;
    auto slice_start_time = std::chrono::steady_clock::now();
    bool stopMechanism = false;

    while (g_running && rclcpp::ok()) 
    {
        executor.spin_some(); 

        // --- 1. Time Limit Check ---
        if (limited) {
            auto now = std::chrono::steady_clock::now();
            if (now - start_time > time_limit) {
                std::cout << "[INFO] time_limit seconds have passed. Exiting loop.\n";
                break;
            }
        }

        // --- 2. GET STATE ---
        Eigen::VectorXd current_sim_state = ros_manager->getCurrentSimulatedState();
        if (current_sim_state.size() == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        // Update Planner
        kinodynamic_planner->setRobotState(current_sim_state); 
        double T_robot = current_sim_state(current_sim_state.size()-1);

        // 2. Convert to Forward Simulation Time
        // If Budget started at 20.0, and is now 18.5, then Sim Time is 1.5
        double sim_time = time_budget_ - T_robot; 


        // --- 3. DETECT EVENTS & PLAN ---
        gazebo_checker->processLatestPoseInfo(sim_time);

        ObstacleVector turned_obs = gazebo_checker->checkAndRepairObstacles(T_robot);

        // // --- CAPTURE LOGS FROM RETURNED VECTOR : THIS IS FOR TURNAROUND EVENT DETERMINISM VERIFICATION ---
        // if (!turned_obs.empty()) {
        //     for (const auto& ob : turned_obs) {
        //         // We only log if it's a dynamic obstacle that actually turned
        //         // (The function returns initialized obstacles too, but they have nextDirectionChangeTime set)
        //         if (ob.is_dynamic) {
        //             current_run_events.push_back(obstacleToLogEvent(ob, T_robot));
                    
        //             // // Optional: Print to console manually since we removed RCLCPP_WARN
        //             // std::cout << "[TURNAROUND] " << ob.name 
        //             //         << " | T_robot: " << T_robot 
        //             //         << " | Expected: " << ob.nextDirectionChangeTime
        //             //         << " | Vel: (" << ob.velocity.x() << ", " << ob.velocity.y() << ")" << std::endl;
        //         }
        //     }
        // }
        // // ---------------------------------------


        auto calc_start = std::chrono::steady_clock::now();

        if (!turned_obs.empty()) {
            auto start_update = std::chrono::steady_clock::now();
            kinodynamic_planner->updateObstacleSamples(turned_obs);
            auto end_update = std::chrono::steady_clock::now();
            double duration_ms = std::chrono::duration<double, std::milli>(end_update - start_update).count();
            
            // Log
            std::string obs_names;
            for (size_t i = 0; i < turned_obs.size(); ++i) {
                obs_names += turned_obs[i].name;
                if (i < turned_obs.size() - 1) obs_names += ", ";
            }
            RCLCPP_INFO(rclcpp::get_logger("FMTx_Timing"), 
                "updateObstacleSamples took: %.2f ms | Processed: [ %s ]", 
                duration_ms, obs_names.c_str());

            // ======================================================
            LogEntry entry;
            const auto& metrics = kinodynamic_planner->getLastReplanMetrics();
            
            // Time since the start of the simulation
            entry.elapsed_s = std::chrono::duration<double>(slice_start_time - global_start).count();
            
            // Duration of the calculation (updateObstacleSamples + internal plan)
            entry.duration_ms = std::chrono::duration<double, std::milli>(end_update - start_update).count(); 
            
            entry.obstacle_checks = metrics.obstacle_checks;
            entry.orphaned_nodes = metrics.orphaned_nodes;
            entry.path_cost = metrics.path_cost;
            entry.time_to_goal = T_robot;
            entry.rewire_neighbor_searches = metrics.rewire_neighbor_searches;
            
            log_data.push_back(entry);
            // ======================================================


        }

        auto calc_end = std::chrono::steady_clock::now();

        // --- 4. UPDATE VISUALIZATION & THREATS ---
        std::vector<Obstacle> culprits = obstacle_checker->getAndClearCulprits();
        ros_manager->updateThreats(culprits);

        auto new_executable_path = kinodynamic_planner->getPathPositions();
        if (!new_executable_path.empty()) {
            ros_manager->setPath(new_executable_path);
            current_viz_path = new_executable_path;
        }

        if (!current_viz_path.empty()) {
            kinodynamic_planner->visualizePath(current_viz_path);
        }

        // --- 6. SLICE MANAGEMENT (FIXED) ---
        
        // Calculate how much "wall clock" time passed during this iteration
        auto now = std::chrono::steady_clock::now();
        double dt_wall = std::chrono::duration<double>(now - slice_start_time).count();
        
        // Add this time to our accumulator
        time_accumulated_in_slice += dt_wall;

        // Check if we have filled the slice
        if (time_accumulated_in_slice >= slice_time) {
            
            // --- SLICE COMPLETE ---
            
            // // 1. Step Physics by exactly 'slice_time'
            // int steps_needed = static_cast<int>(slice_time / gazebo_max_step);
            // stepGazebo(steps_needed);
            
            // // 2. Step Robot by exactly 'slice_time'
            // ros_manager->stepSimulation(slice_time);

            // ======================================================
            // NEW: SAFETY CHECK BEFORE MOVING
            // ======================================================
            if (!stopMechanism ||  kinodynamic_planner->isRobotSafe()) {
                // 2. Step Robot by exactly 'slice_time'
                ros_manager->stepSimulation(slice_time);
            } else {
                // ROBOT IS LOST / TRAPPED
                ros_manager->stepStationary(slice_time);

            }

            // 3. Reset Accumulator
            time_accumulated_in_slice = 0.0;
            slice_start_time = std::chrono::steady_clock::now();
            // ======================================================
            // MOVE PRINT HERE
            // ======================================================
            Eigen::VectorXd updated_state = ros_manager->getCurrentSimulatedState();
            T_robot = updated_state(updated_state.size()-1);
            if (stopMechanism && T_robot <= 0) {
                std::cout << "T_ROBOT: " << T_robot << std::endl; // <--- MOVE HERE
                std::cout << "LOST DEADLINE! \n";
                break;
            }
            // ======================================================




            // 4. Check Goal (Only check after moving)
            // Eigen::VectorXd updated_state = ros_manager->getCurrentSimulatedState();
            double distance_to_goal = (updated_state.head<2>() - tree_root_state.head<2>()).norm();
            if (distance_to_goal < goal_tolerance) {
                RCLCPP_INFO(vis_node->get_logger(), "Goal Reached!");
                ros_manager->updateThreats({});
                g_running = false;
            }
        } else {
            // --- CRITICAL FIX: SLEEP THE LOOP ---
            // If we haven't filled the slice yet, sleep for the remaining time.
            // This prevents the loop from spinning thousands of times per second.
            double remaining_time = slice_time - time_accumulated_in_slice;
            if (remaining_time > 0.0) {
                std::this_thread::sleep_for(std::chrono::duration<double>(remaining_time));
            }
        }
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
    rclcpp::shutdown();
    return 0;
}
