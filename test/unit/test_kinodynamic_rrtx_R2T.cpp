// Copyright 2025 Soheil E.nia

#include "motion_planning/planners/planner_factory.hpp"
#include "motion_planning/state_space/rdt_statespace.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/utils/parse_sdf.hpp"
#include "motion_planning/utils/ros2_manager_r2t.hpp"
#include "motion_planning/utils/logs.hpp"
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



// Helper to convert Obstacle to TurnaroundEvent for logging
TurnaroundEvent obstacleToLogEvent(const Obstacle& ob, double current_T_robot) {
    TurnaroundEvent event;
    event.name = ob.name;
    event.t_robot = current_T_robot;
    event.expected_t = ob.nextDirectionChangeTime; // This was set right before the return
    event.vel_x = ob.velocity.x();
    event.vel_y = ob.velocity.y();
    return event;
}

struct LogEntry {
    double elapsed_s = 0.0;
    double duration_ms = 0.0;
    double time_to_goal = 0.0;
    double path_cost = 0.0;
    int obstacle_checks = 0;
    long long rewire_neighbor_searches = 0;
    int orphaned_nodes = 0;
    int collision_count = 0;
    
};

std::atomic<bool> g_running{true};

void sigint_handler(int sig)
{
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

int main(int argc, char** argv)
{
    // --- Initial Setup ---
    rclcpp::init(argc, argv);
    signal(SIGINT, sigint_handler);


    int num_samples = 300;
    double factor = 1.0;
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
    // Encapsulate parameters for better organization
    Params manager_params;
    manager_params.setParam("use_sim_time", true);
    manager_params.setParam("sim_time_step", -0.04); // Time-to-go consumed per sim step
    manager_params.setParam("sim_frequency_hz", 50);  // Smoothness of arrow
    manager_params.setParam("vis_frequency_hz", 30);  // Obstacle visualization rate
    manager_params.setParam("follow_path", true);

    double time_budget_ = 20.0;
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
    gazebo_params.setParam("inflation", 0.75); // <-- VERIFY THIS IS A REASONABLE, NON-ZERO VALUE
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
    planner_params.setParam("kd_dim", 3); // 2 or 3 only for R2T
    // planner_params.setParam("mode", 1); // 1: full node centric | 2: full obstalce centric | 3: node centric plus a map to obstalce check against speicific obstalces
    planner_params.setParam("delta", 15); // 1: full node centric | 2: full obstalce centric | 3: node centric plus a map to obstalce check against speicific obstalces

    // --- Object Initialization ---
    // A single node is shared for visualization purposes
    auto vis_node = std::make_shared<rclcpp::Node>("rrtx_visualizer",
        rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
    
    auto visualization = std::make_shared<RVizVisualization>(vis_node);
    auto sim_clock = vis_node->get_clock();

    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_4_obs.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_many.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_many_constant_acc.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_many_constant_acc_uncrowded.sdf");
    auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_straight_box_circle.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_straight_box_circle_10.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_straight.sdf");
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(sim_clock, gazebo_params, obstacle_info);


    // --- Planner and Problem Definition ---
    const int dim = 3;
    const int spatial_dim = 2;
    auto problem_def = std::make_shared<ProblemDefinition>(dim);
    Eigen::VectorXd tree_root_state(3);
    tree_root_state << -48.0, -48.0, 0.0; // Destination: x, y, time-to-go
    problem_def->setStart(tree_root_state); // "Start" of the backward search

    Eigen::VectorXd robot_initial_state(3);
    robot_initial_state << 48.0, 48.0, time_budget_; // Initial robot state: x, y, total time budget
    problem_def->setGoal(robot_initial_state); // "Goal" of the backward search

    Eigen::VectorXd lower_bounds(3), upper_bounds(3);
    lower_bounds << -50.0, -50.0, 0.0;
    upper_bounds << 50.0, 50.0, time_budget_; // Max time-to-go for any sample
    problem_def->setBounds(lower_bounds, upper_bounds);


    double min_velocity = 0.0;
    double max_velocity = 20.0;
    double robot_velocity = 10.0;
    // Create the single, consolidated R2TROSManager
    auto ros_manager = std::make_shared<R2TROS2Manager>(obstacle_checker, visualization, manager_params,robot_velocity, robot_initial_state,time_budget_);
    auto statespace = std::make_shared<RDTStateSpace>(spatial_dim, min_velocity , max_velocity , robot_velocity, 30000, seed);
    auto planner = PlannerFactory::getInstance().createPlanner(PlannerType::KinodynamicRRTX, statespace, problem_def, obstacle_checker);
    
    auto kinodynamic_planner = dynamic_cast<KinodynamicRRTX*>(planner.get());
    kinodynamic_planner->setClock(sim_clock);
    planner->setup(planner_params, visualization);

    std::vector<Eigen::VectorXd> current_executable_path;

    // --- Perform the INITIAL Plan ---
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

    kinodynamic_planner->dumpTreeToCSV("rrtx_tree_nodes.csv");


    // Anchor the robot to the initial plan
    kinodynamic_planner->setRobotState(robot_initial_state);
    
    current_executable_path = kinodynamic_planner->getPathPositions();
    if (!current_executable_path.empty()) {
        ros_manager->setPath(current_executable_path);
    }
    RCLCPP_INFO(vis_node->get_logger(), "Initial plan complete. Executing...");



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
    // std::this_thread::sleep_for(std::chrono::milliseconds(800));


    const double goal_tolerance = 3.0;

    std::vector<double> sim_durations;
    std::vector<std::tuple<double, double>> sim_duration_2;

    bool limited = true; 
    if (manager_params.getParam<bool>("follow_path"))
        limited = false;
    auto start_time = std::chrono::steady_clock::now();
    auto time_limit = std::chrono::seconds(run_secs);

    int counter = 0;

    std::vector<LogEntry> log_data;

    auto global_start = std::chrono::steady_clock::now();
    // rclcpp::Rate loop_rate(20); // Frequency to check for replan triggers


    // 1. Cast the shared_ptr to the specific Gazebo class
    auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker);
    // 3. Initialize the RRTx timers ONCE
    // STEP 3: Grab the actual "Frozen" positions from Gazebo
    if (gazebo_checker) {
        gazebo_checker->processLatestPoseInfo(0);
        
        // Initialize timers based on the exact Budget (e.g., 25.0)
        double initial_T = robot_initial_state(2); 
        gazebo_checker->initializeDynamicObstacles(initial_T);
        std::cout << "[SYNC] RRTx Timers initialized against frozen physics.\n";
    }

    // --- Set Up Executor (Unchanged) ---
    // rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::executors::StaticSingleThreadedExecutor executor; // +++ ADD THIS

    executor.add_node(ros_manager);
    executor.add_node(vis_node); // Add the vis_node to the executor so its timer runs!

    // std::thread executor_thread([&executor]() {
    //     executor.spin();
    // });
    // STEP 5: Start the Physics and the Main Loop simultaneously
    // unpauseSimulation();

    RCLCPP_INFO(vis_node->get_logger(), "Starting execution and monitoring loop.");


    // Start profiling
    // while (g_running && rclcpp::ok())
    // {
    //     // if (counter > 300)  
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
        
    //     bool needs_replan = false;
        
    //     // Get the robot's current state ONCE per cycle.
    //     Eigen::VectorXd current_sim_state = ros_manager->getCurrentSimulatedState();



    //     kinodynamic_planner->setRobotState(current_sim_state);
    //     if (current_sim_state.size() == 0) { // Wait for the simulation to initialize
    //          loop_rate.sleep();
    //          continue;
    //     }
    //     // Calculate the 2D distance to the goal using the tree_root_state variable.
    //     double distance_to_goal = (current_sim_state.head<2>() - tree_root_state.head<2>()).norm();

    //     if (distance_to_goal < goal_tolerance) {
    //         RCLCPP_INFO(vis_node->get_logger(), "Goal Reached! Mission Accomplished.");
    //         ros_manager->updateThreats({}); 
    //         g_running = false; // Set the flag to cleanly exit the loop.
    //         continue;          // Skip the rest of this loop iteration.
    //     }

    //     const auto& snapshot = obstacle_checker->getAtomicSnapshot();

    //     auto start = std::chrono::steady_clock::now();

    //     kinodynamic_planner->updateObstacleSamples(snapshot.obstacles);

    //     auto end = std::chrono::steady_clock::now();

    //     // --- [NEW] 6. SYNC THREATS ---
    //     // 1. Get the obstacles that caused collisions during this specific plan
    //     std::vector<Obstacle> culprits = obstacle_checker->getAndClearCulprits();
        
    //     // 2. Send them to the R2T Manager. 
    //     // The manager's background timer will see these and draw them as RED.
    //     ros_manager->updateThreats(culprits);

    //     // -----------------------------

    //     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //     if (duration.count() > 0) {
    //         std::cout << "time taken for the update : " << duration.count() 
    //                 << " milliseconds\n";
    //     }
    //     sim_durations.push_back(duration.count());
    //     double elapsed_s = std::chrono::duration<double>(start - global_start).count();
    //     double duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
    //     sim_duration_2.emplace_back(elapsed_s, duration_ms);

    //     /////-----
    //     LogEntry entry;
    //     entry.elapsed_s = std::chrono::duration<double>(start - global_start).count();
    //     entry.duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
    //     /////-----


    //     Eigen::VectorXd fresh_robot_state = ros_manager->getCurrentSimulatedState();
    //     kinodynamic_planner->setRobotState(fresh_robot_state);
    //     auto new_executable_path = kinodynamic_planner->getPathPositions();

    //     ////--------
    //     const auto& metrics = kinodynamic_planner->getLastReplanMetrics();
    //     entry.obstacle_checks = metrics.obstacle_checks;
    //     entry.rewire_neighbor_searches = metrics.rewire_neighbor_searches;
    //     entry.orphaned_nodes = metrics.orphaned_nodes;
    //     entry.path_cost = metrics.path_cost;
    //     entry.time_to_goal = kinodynamic_planner->getRobotTimeToGo();
        
    //     log_data.push_back(entry);
    //     /////--------
        
    //     if (new_executable_path.empty()) {
    //         // FAILURE CASE: The planner could not find a valid path from the robot's current state.
    //         RCLCPP_ERROR(vis_node->get_logger(), "Replanning failed! Commanding robot to STOP.");
            
    //         // Create a "stop" path containing only the robot's current state.
    //         std::vector<Eigen::VectorXd> stop_path;
    //         stop_path.push_back(fresh_robot_state);
            
    //         // Update the current path and send it to the manager to halt execution.
    //         current_executable_path = stop_path;
    //         ros_manager->setPath(current_executable_path);

    //     } else {
    //         // SUCCESS CASE: A new path was found.
    //         // Check if the newly generated path is actually different from the one we're already on.
    //         if (kinodynamic_planner->arePathsSimilar(current_executable_path, new_executable_path, 0.1)) { // Increased tolerance
    //             RCLCPP_INFO(vis_node->get_logger(), "Replanning resulted in a similar path. No update needed.");
    //         } else {
    //             RCLCPP_INFO(vis_node->get_logger(), "New optimal path found. Updating trajectory.");
    //             // If the path is meaningfully new, update our stored path and send it to the manager.
    //             current_executable_path = new_executable_path;
    //             ros_manager->setPath(current_executable_path);
    //         }
    //     }
    //     // }

    //     kinodynamic_planner->visualizePath(current_executable_path);
    //     // We visualize the tree in every frame, regardless of replanning.
    //     // kinodynamic_planner->visualizeTree();
    //     loop_rate.sleep();
    // }

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
        // // --- DEBUG: LOG TIME BEFORE PLANNING ---
        // Eigen::VectorXd state_before_plan = ros_manager->getCurrentSimulatedState();
        // double t_before = state_before_plan(state_before_plan.size() - 1);
        // double sim_t_before = time_budget_ - t_before;
        // RCLCPP_INFO(rclcpp::get_logger("MainLoop_Time"), 
        //     ">>> START PLANNING | Robot T_Goal: %.2f | Sim_Time: %.2f", t_before, sim_t_before);


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
            RCLCPP_INFO(rclcpp::get_logger("RRTx_Timing"), 
                "updateObstacleSamples took: %.2f ms | Processed: [ %s ]", 
                duration_ms, obs_names.c_str());
        }

        auto calc_end = std::chrono::steady_clock::now();

        // // =========================================================================
        // // CRITICAL FIX: RE-SYNC PLANNER WITH ROBOT AFTER LONG CALCULATIONS
        // // =========================================================================
        // // The planner took ~6 seconds. The robot moved from T=5.46 to T=13.56.
        // // We MUST update the planner's internal robot state to the FUTURE time (13.56)
        // // so it generates a valid path for where the robot IS, not where it WAS.
        
        // Eigen::VectorXd fresh_robot_state = ros_manager->getCurrentSimulatedState();
        // kinodynamic_planner->setRobotState(fresh_robot_state);
        
        // // Debug Log to verify sync
        // double fresh_t = fresh_robot_state(fresh_robot_state.size()-1);
        // double fresh_sim_t = time_budget_ - fresh_t;
        // RCLCPP_INFO(rclcpp::get_logger("MainLoop_Sync"), 
        //     "Post-Plan Sync: Robot T_Goal=%.2f | Sim_Time=%.2f", fresh_t, fresh_sim_t);
        // // =========================================================================



        // // --- DEBUG: LOG TIME AFTER PLANNING ---
        // Eigen::VectorXd state_after_plan = ros_manager->getCurrentSimulatedState();
        // double t_after = state_after_plan(state_after_plan.size() - 1);
        // double sim_t_after = time_budget_ - t_after;
        // RCLCPP_INFO(rclcpp::get_logger("MainLoop_Time"), 
        //     "<<< END PLANNING   | Robot T_Goal: %.2f | Sim_Time: %.2f | Time_Drift: %.2f s", 
        //     t_after, sim_t_after, (sim_t_after - sim_t_before));


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

        // --- 5. LOGGING ---
        LogEntry entry;
        const auto& metrics = kinodynamic_planner->getLastReplanMetrics();
        entry.elapsed_s = std::chrono::duration<double>(slice_start_time - global_start).count();
        entry.duration_ms = std::chrono::duration<double, std::milli>(calc_end - calc_start).count(); 
        entry.obstacle_checks = metrics.obstacle_checks;
        entry.orphaned_nodes = metrics.orphaned_nodes;
        entry.path_cost = metrics.path_cost;
        entry.time_to_goal = T_robot;
        entry.rewire_neighbor_searches = metrics.rewire_neighbor_searches;
        log_data.push_back(entry);

        // --- 6. SLICE MANAGEMENT (FIXED) ---
        
        // Calculate how much "wall clock" time passed during this iteration
        auto now = std::chrono::steady_clock::now();
        double dt_wall = std::chrono::duration<double>(now - slice_start_time).count();
        
        // Add this time to our accumulator
        time_accumulated_in_slice += dt_wall;

        // Check if we have filled the slice
        if (time_accumulated_in_slice >= slice_time) {
            
            // --- SLICE COMPLETE ---
            
            // 1. Step Physics by exactly 'slice_time'
            // int steps_needed = static_cast<int>(slice_time / gazebo_max_step);
            // stepGazebo(steps_needed);
            
            // // 2. Step Robot by exactly 'slice_time'
            // ros_manager->stepSimulation(slice_time);


            // ======================================================
            // NEW: SAFETY CHECK BEFORE MOVING
            // ======================================================
            if (!stopMechanism || kinodynamic_planner->isRobotSafe()) {
                // 2. Step Robot by exactly 'slice_time'
                ros_manager->stepSimulation(slice_time);
            } else {
                // ROBOT IS LOST / TRAPPED
                ros_manager->stepStationary(slice_time);

            }
            // ======================================================

            // 3. Reset Accumulator
            time_accumulated_in_slice = 0.0;
            slice_start_time = std::chrono::steady_clock::now();

            // ======================================================
            // MOVE PRINT HERE
            // ======================================================
            Eigen::VectorXd updated_state = ros_manager->getCurrentSimulatedState();
            T_robot = updated_state(updated_state.size()-1);
            

            if (stopMechanism && T_robot <= 0) {
                std::cout << "T_ROBOT: " << T_robot << std::endl; 
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


    // verifyDeterminism(current_run_events);

    const int final_collision_count = ros_manager->getCollisionCount();
    RCLCPP_FATAL(vis_node->get_logger(), "SIMULATION COMPLETE. TOTAL DETECTED COLLISIONS: %d", final_collision_count);

    for (auto& entry : log_data) {
        entry.collision_count = final_collision_count;
    }

    int num_of_samples_val = planner_params.getParam<int>("num_of_samples");
    std::time_t now_time = std::time(nullptr);
    std::tm* local_tm = std::localtime(&now_time);
    char time_buf[80];
    strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", local_tm);
    
    std::string filename = "sim_rrtx_" + std::to_string(num_of_samples_val) +
                           "samples_" + time_buf + "_metrics.csv";
    
    std::cout << "Writing replan metrics to: " << filename << std::endl;
    
    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cerr << "Error: failed to open " << filename << std::endl;
        return 1;
    }
    
    out << "elapsed_s,duration_ms,time_to_goal,path_cost,obstacle_checks,rewire_neighbor_searches,orphaned_nodes,collision_count\n";
    
    for (const auto& log_item : log_data) {
        out << log_item.elapsed_s << ","
            << log_item.duration_ms << ","
            << log_item.time_to_goal << ","
            << log_item.path_cost << ","
            << log_item.obstacle_checks << ","
            << log_item.rewire_neighbor_searches << ","
            << log_item.orphaned_nodes << ","
            << log_item.collision_count << "\n";
    }
    out.close();
    std::cout << "Done writing CSV.\n";



    // --- Graceful Shutdown ---
    RCLCPP_INFO(vis_node->get_logger(), "Shutting down.");
    executor.cancel();
    // if (executor_thread.joinable())
    // {
    //     executor_thread.join();
    // }
    rclcpp::shutdown();
    return 0;
}