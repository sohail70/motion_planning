// Copyright 2025 Soheil E.nia

#include "motion_planning/planners/planner_factory.hpp"
#include "motion_planning/state_space/thruster_statespace.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/utils/parse_sdf.hpp"
#include "motion_planning/utils/ros2_manager_thruster.hpp" 
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
    double time_to_goal = 0.0;
    double path_cost = 0.0;
    int obstacle_checks = 0;
    long long rewire_neighbor_searches = 0;
    int orphaned_nodes = 0;
    int collision_count = 0;
};

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








std::atomic<bool> g_running{true};


void sigint_handler(int sig) {
    g_running = false;
    rclcpp::shutdown();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    // Set up SIGINT handler
    struct sigaction sa;
    sa.sa_handler = sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGINT, &sa, nullptr) < 0) {
        std::cerr << "Failed to set signal handler: " << strerror(errno) << std::endl;
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }


    int num_samples = 10;
    double factor = 1.5;
    unsigned int seed = 42;
    int run_secs = 20;


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

    std::srand(seed);
    std::cout << "[INFO] seed=" << seed
                << ", samples=" << num_samples
                << ", factor=" << factor
                << ", duration=" << run_secs << "s\n";




    // Create Params for ROS2Manager
    Params manager_params;
    manager_params.setParam("use_sim_time", true);
    manager_params.setParam("simulation_time_step", -0.04); // 50 Hz simulation loop
    manager_params.setParam("thruster_state_dimension", 5);
    manager_params.setParam("sim_frequency_hz", 50);  // Smoothness of arrow
    manager_params.setParam("vis_frequency_hz", 30);  // Obstacle visualization rate
    manager_params.setParam("follow_path", true);

    double time_budget_ = 25.0;
    Params gazebo_params;
    gazebo_params.setParam("robot_model_name", "tugbot");
    gazebo_params.setParam("default_robot_x", 48.0); 
    gazebo_params.setParam("default_robot_y", 48.0);
    gazebo_params.setParam("world_name", "default");
    gazebo_params.setParam("use_range", false); 
    gazebo_params.setParam("sensor_range", 20.0);
    gazebo_params.setParam("estimation", true);
    gazebo_params.setParam("inflation", 0.75); // A larger inflation makes the robot fatter to the planner, which might prevent it from finding paths through narrow gaps.
    gazebo_params.setParam("persistent_static_obstacles", false);
    gazebo_params.setParam("fcl", false);
    gazebo_params.setParam("bullet", true);
    gazebo_params.setParam("initial_budget_time", time_budget_);
    manager_params.setParam("inflation", gazebo_params.getParam<double>("inflation"));  // Obstacle visualization rate

    Params planner_params;
    planner_params.setParam("num_of_samples", num_samples);
    planner_params.setParam("factor", factor);
    planner_params.setParam("use_kdtree", true); // for now the false is not impelmented! maybe i should make it default! can't think of a case of not using it but i just wanted to see the performance without it for low sample cases.
    planner_params.setParam("kdtree_type", "NanoFlann");
    planner_params.setParam("partial_update", true); //update till the robot's costToInit
    planner_params.setParam("static_obs_presence", false); // to not process static obstalces twice because obstacle checker keeps sending all the obstalces! i geuss the persisten_static_obstalces needs to be true always
    planner_params.setParam("obs_cache", false); 
    planner_params.setParam("partial_plot", false);
    planner_params.setParam("use_heuristic", false); // TODO: I need to verify if its legit workingor not.
    planner_params.setParam("precache_neighbors", true);
    planner_params.setParam("kd_dim", 3); // 2 or 3 for only 2nd order thruster and 4 incase you do 3rd order [x, y, z, vx, vy, vz, time]
    planner_params.setParam("use_knn", false);
    planner_params.setParam("mode", 1); //1: prune false 2: prune true

    // Object Initialization
    auto vis_node = std::make_shared<rclcpp::Node>("fmtx_thruster_visualizer",
        rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
    auto visualization = std::make_shared<RVizVisualization>(vis_node);
    auto sim_clock = vis_node->get_clock();

    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_many_constant_acc_uncrowded.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_many_constant_acc.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_straight_box.sdf");
    auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_straight_box_circle_10_slow.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_straight.sdf");
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(sim_clock, gazebo_params, obstacle_info);

    // --- Planner and Problem Definition (5D Thruster) ---
    const int dim = 5; // STATE: [x, y, vx, vy, time]
    auto problem_def = std::make_shared<ProblemDefinition>(dim);

    // The GOAL for the robot is the ROOT of the search tree.
    // Time-to-go is 0 at the goal. Velocity is zero.
    Eigen::VectorXd tree_root_state(dim);
    tree_root_state << -48.0, -48.0, 0.0, 0.0, 0.0;
    problem_def->setStart(tree_root_state);

    // The robot's INITIAL state is the GOAL of the search tree.
    // It starts with a full time budget.
    Eigen::VectorXd robot_initial_state(dim);
    robot_initial_state << 48.0, 48.0, 0.0, 0.0, time_budget_;
    problem_def->setGoal(robot_initial_state);

    // Define 5D bounds, including velocity limits.
    Eigen::VectorXd lower_bounds(dim), upper_bounds(dim);
    double max_vel = 15.0; // Max velocity in any direction
    lower_bounds << -50.0, -50.0, -max_vel, -max_vel, 0.0;
    upper_bounds << 50.0, 50.0,  max_vel,  max_vel, time_budget_;
    problem_def->setBounds(lower_bounds, upper_bounds);

    // Create the thruster state space with max acceleration.
    double max_acceleration = 5.0; // m/s^2
    auto ros_manager = std::make_shared<ThrusterROS2Manager>(obstacle_checker, visualization, manager_params, robot_initial_state ,time_budget_);
    auto statespace = std::make_shared<ThrusterSteerStateSpace>(dim, max_acceleration, max_vel, seed);
    auto planner = PlannerFactory::getInstance().createPlanner(PlannerType::KinodynamicFMTX, statespace, problem_def, obstacle_checker);
    
    auto kinodynamic_planner = dynamic_cast<KinodynamicFMTX*>(planner.get());
    kinodynamic_planner->setClock(sim_clock);
    planner->setup(planner_params, visualization);

    // --- Initial Plan ---
    RCLCPP_INFO(vis_node->get_logger(), "Running initial plan...");
    planner->plan();
    kinodynamic_planner->setRobotState(robot_initial_state);
    auto path = kinodynamic_planner->getPathPositions();
    if (!path.empty()) {
        ros_manager->setPlannedThrusterTrajectory(path);
        // Set the robot's starting state in the manager to initialize its clock.
        ros_manager->setInitialState(robot_initial_state);
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
    // resetAndPlaySimulation();
    const double goal_tolerance = 3.0;
    // rclcpp::Rate loop_rate(20);
    std::vector<double> sim_durations;
    std::vector<std::tuple<double, double>> sim_duration_2;

    bool limited = true; 
    if (manager_params.getParam<bool>("follow_path"))
        limited = false;
    auto start_time = std::chrono::steady_clock::now();
    auto time_limit = std::chrono::seconds(run_secs);


    std::vector<LogEntry> log_data;
    auto global_start = std::chrono::steady_clock::now();
    
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
    executor.add_node(vis_node); // Don't mind the straight line connection which passes through static obstacles! i didnt want to spent time visualizing correct traj but just wanted to check if the graph can reach the robot or not!
    // std::thread executor_thread([&executor]() { executor.spin(); });
    RCLCPP_INFO(vis_node->get_logger(), "Starting execution and monitoring loop.");


    // Start profiling
    CALLGRIND_START_INSTRUMENTATION;
        // --- CONFIGURATION ---
    const double slice_time = 0.02;       // The robot moves this amount every slice (e.g., 0.05s)
    // const double gazebo_max_step = 0.001; // Physics engine step size
    
    // std::cout << "[CONFIG] Slice Time: " << slice_time << "s | Gazebo Step: " << gazebo_max_step << "s" << std::endl;
    std::vector<Eigen::VectorXd> current_viz_path; 
    
    // Variables to track time within the current slice
    double time_accumulated_in_slice = 0.0;
    auto slice_start_time = std::chrono::steady_clock::now();
    bool stopMechanism = false;

    while (g_running && rclcpp::ok()) {
        // --- 1. Time Limit Check (Restored) ---
        executor.spin_some();
        if (limited) {
            auto now = std::chrono::steady_clock::now();
            if (now - start_time > time_limit) {
                std::cout << "[INFO] time_limit seconds have passed. Exiting loop.\n";
                break;
            }
        }

        auto loop_start_time = std::chrono::steady_clock::now();

        // --- 2. Get Robot State & Correct Order ---
        // [CRITICAL ORDER FIX]: Update Planner State FIRST so it knows where the robot IS.
        Eigen::VectorXd current_sim_state = ros_manager->getCurrentKinodynamicState();
        if (current_sim_state.size() == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        // Update the planner's internal robot node immediately
        kinodynamic_planner->setRobotState(current_sim_state); 
        double T_robot = current_sim_state(current_sim_state.size()-1);

        // 2. Convert to Forward Simulation Time
        // If Budget started at 20.0, and is now 18.5, then Sim Time is 1.5
        double sim_time = time_budget_ - T_robot; 

        // --- 3. Detect Events ---
        // Grab the frozen physics state
        gazebo_checker->processLatestPoseInfo(sim_time);
        
        // Calculate which obstacles have flipped relative to the Robot's T
        ObstacleVector turned_obs = gazebo_checker->checkAndRepairObstacles(T_robot);

        // Measure computation time for this specific update step
        auto calc_start = std::chrono::steady_clock::now();

        // // --- 4. Update Planner ---
        // // Only burn CPU if something actually changed
        // if (!turned_obs.empty()) {
        //     kinodynamic_planner->updateObstacleSamples(turned_obs);
        // }

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


        // --- 4. UPDATE VISUALIZATION & THREATS ---
        std::vector<Obstacle> culprits = obstacle_checker->getAndClearCulprits();
        ros_manager->updateThreats(culprits);

        auto new_executable_path = kinodynamic_planner->getPathPositions();
        if (!new_executable_path.empty()) {
            ros_manager->setPlannedThrusterTrajectory(new_executable_path);
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
            Eigen::VectorXd updated_state = ros_manager->getCurrentKinodynamicState();
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



// // --- Main Execution and Replanning Loop ---
//     double adaptive_prediction_dt = 0.005; 

//     CALLGRIND_START_INSTRUMENTATION;
//     while (g_running && rclcpp::ok()) {
//         if (limited) {
//             auto now = std::chrono::steady_clock::now();
//             if (now - start_time > time_limit) break;
//         }

//         // --- 1. ADAPTIVE PREDICTION ---
//         // Project robot state forward based on the last known plan duration.
//         // This compensates for the time it takes to compute the next path.
//         Eigen::VectorXd current_state = ros_manager->getCurrentKinodynamicState();
//         Eigen::VectorXd future_state = current_state;

//         if (current_state.size() >= 4) {
//              future_state(0) += current_state(2) * adaptive_prediction_dt; 
//              future_state(1) += current_state(3) * adaptive_prediction_dt; 
//              future_state(4) -= adaptive_prediction_dt; 
//         }

//         // Check Goal Proximity
//         double dist_to_goal = (current_state.head<2>() - tree_root_state.head<2>()).norm();
//         if (dist_to_goal < goal_tolerance) {
//             RCLCPP_INFO(vis_node->get_logger(), "Goal Reached!");
//             g_running = false;
//             break;
//         }

//         // --- 2. PLAN ---
//         kinodynamic_planner->setRobotState(future_state);
//         auto snapshot = obstacle_checker->getAtomicSnapshot();
        
//         auto start_plan = std::chrono::steady_clock::now();
        
//         // Update the planner with the latest moving obstacle positions
//         kinodynamic_planner->updateObstacleSamples(snapshot.obstacles);
        
//         // Run FMT* Plan (The ObstacleChecker's isTrajectorySafe calls will fill the cache here)
//         planner->plan();

//         auto end_plan = std::chrono::steady_clock::now();
//         double plan_duration_sec = std::chrono::duration<double>(end_plan - start_plan).count();

//         // --- 3. SYNC THREATS & VISUALIZE ---
//         // 3.1 Extract all obstacles that caused a collision during the search phase
//         std::vector<Obstacle> culprits = obstacle_checker->getAndClearCulprits();

//         // 3.2 SEND TO MANAGER: This causes the specific obstacles to turn RED in Gazebo/RViz
//         ros_manager->updateThreats(culprits);


//         // --- 4. UPDATE ADAPTIVE PREDICTION ---
//         // Set the projection for the NEXT loop: Actual computation time + 20% safety buffer
//         adaptive_prediction_dt = std::max(0.001, plan_duration_sec * 1.2);

//         // --- 5. EXECUTION & VISUALIZATION ---
//         auto new_path = kinodynamic_planner->getPathPositions();

//         if (!new_path.empty()) {
//             // Update the simulated robot's trajectory
//             ros_manager->setPlannedThrusterTrajectory(new_path);
//             kinodynamic_planner->visualizePath(new_path);
//         } else {
//             RCLCPP_WARN(vis_node->get_logger(), "Replanning failed. Path is blocked.");
//         }
        
//         // --- 6. METRICS LOGGING ---
//         LogEntry entry;
//         entry.elapsed_s = std::chrono::duration<double>(start_plan - global_start).count();
//         entry.duration_ms = plan_duration_sec * 1000.0;
        
//         const auto& metrics = kinodynamic_planner->getLastReplanMetrics();
//         entry.obstacle_checks = metrics.obstacle_checks;
//         entry.rewire_neighbor_searches = metrics.rewire_neighbor_searches;
//         entry.orphaned_nodes = metrics.orphaned_nodes;
//         entry.path_cost = metrics.path_cost;
//         entry.time_to_goal = kinodynamic_planner->getRobotTimeToGo();
        
//         log_data.push_back(entry);

//         // Output real-time stats
//         std::cout << "Plan: " << std::fixed << std::setprecision(2) << entry.duration_ms 
//                   << "ms | Rejections: " << culprits.size() 
//                   << " | Adaptive Pred: " << adaptive_prediction_dt * 1000 << "ms\n";

//         loop_rate.sleep();
//     }
//     CALLGRIND_STOP_INSTRUMENTATION;



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
    




    // Graceful Shutdown
    RCLCPP_INFO(vis_node->get_logger(), "Shutting down.");
    g_running = false;
    executor.cancel();
    // if (executor_thread.joinable()) {
    //     executor_thread.join();
    // }
    rclcpp::shutdown();
    return 0;
}



