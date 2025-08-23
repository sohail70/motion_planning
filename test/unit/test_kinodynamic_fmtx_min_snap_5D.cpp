// Copyright 2025 Soheil E.nia

#include "motion_planning/planners/planner_factory.hpp"
#include "motion_planning/state_space/min_snap_statespace.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/utils/parse_sdf.hpp"
#include "motion_planning/utils/ros2_manager_min_snap.hpp" 
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


    int num_samples = 1;
    double factor = 3.0;
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
    manager_params.setParam("vis_frequency_hz", 10);  // Obstacle visualization rate
    manager_params.setParam("follow_path", true);

    Params gazebo_params;
    gazebo_params.setParam("robot_model_name", "tugbot");
    gazebo_params.setParam("default_robot_x", 48.0); 
    gazebo_params.setParam("default_robot_y", 48.0);
    gazebo_params.setParam("world_name", "default");
    gazebo_params.setParam("use_range", false); 
    gazebo_params.setParam("sensor_range", 20.0);
    gazebo_params.setParam("estimation", true);
    gazebo_params.setParam("inflation", 0.5); // A larger inflation makes the robot fatter to the planner, which might prevent it from finding paths through narrow gaps.
    gazebo_params.setParam("persistent_static_obstacles", false);
    gazebo_params.setParam("fcl", false); //TODO: Implement this for 3D min-snap too!
    gazebo_params.setParam("bullet", true);
    gazebo_params.setParam("spatial_dim", 3);

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
    planner_params.setParam("precache_neighbors", false); // Cant do that in min-snap
    planner_params.setParam("kd_dim", 5); //
    planner_params.setParam("use_knn", false);
    planner_params.setParam("mode", 1); //1: prune false 2: prune true

    // --- Object Initialization ---
    auto vis_node = std::make_shared<rclcpp::Node>("fmtx_thruster_visualizer",
        rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
    auto visualization = std::make_shared<RVizVisualization>(vis_node);
    auto sim_clock = vis_node->get_clock();

    auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_straight_box_circle_10_3D.sdf");
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(sim_clock, gazebo_params, obstacle_info);

    // --- Planner and Problem Definition (5D MinSnap) ---
    const int dim = 5; // STATE: [x, y, z, yaw, time]
    auto problem_def = std::make_shared<ProblemDefinition>(dim);

    // GOAL for the robot is the ROOT of the search tree.
    Eigen::VectorXd tree_root_state(dim);
    tree_root_state << -48.0, -48.0, -10.0, M_PI / 2.0, 0.0;
    problem_def->setStart(tree_root_state);

    // Robot's INITIAL state is the GOAL of the search tree.
    Eigen::VectorXd robot_initial_state(dim);
    // Increased time budget for the larger travel distance.
    double time_budget = 40.0;
    robot_initial_state << 48.0, 48.0, 48.0, -M_PI / 2.0, time_budget;
    problem_def->setGoal(robot_initial_state);

    // Define the new, larger state space bounds.
    Eigen::VectorXd lower_bounds(dim), upper_bounds(dim);
    lower_bounds << -48.0, -48.0, 0.0, -M_PI, 0.0;
    upper_bounds << 48.0, 48.0, 48.0, M_PI, time_budget;
    problem_def->setBounds(lower_bounds, upper_bounds);

    // Create the MinSnap state space
    // Increased dynamic limits to make trajectories in the larger space more feasible.
    double v_max = 20.0; // m/s
    double a_max = 7.0;  // m/s^2
    const double w_vel = 0.1;
    const double w_acc = 0.5;
    const double w_snap = 1.0;
    auto statespace = std::make_shared<MinSnapStateSpace>(5, v_max, a_max, w_vel, w_acc, w_snap, seed);
    // Use the new MinSnap ROS manager
    auto ros_manager = std::make_shared<MinSnapROS2Manager>(obstacle_checker, visualization, manager_params);
    
    auto planner = PlannerFactory::getInstance().createPlanner(PlannerType::KinodynamicFMTX, statespace, problem_def, obstacle_checker);
    auto kinodynamic_planner = dynamic_cast<KinodynamicFMTX*>(planner.get());
    kinodynamic_planner->setClock(sim_clock);
    planner->setup(planner_params, visualization);

    // --- Initial Plan ---
    RCLCPP_INFO(vis_node->get_logger(), "Running initial plan...");
    planner->plan();

    // [MODIFICATION] The first call to setRobotState uses the initial position and assumes rest (zero vel/accel).
    Eigen::VectorXd initial_vel = Eigen::VectorXd::Zero(4); // 4 axes: x, y, z, yaw
    Eigen::VectorXd initial_accel = Eigen::VectorXd::Zero(4);
    kinodynamic_planner->setRobotState(robot_initial_state, initial_vel, initial_accel);

    // [MODIFICATION] Get the full trajectory segments, not just sampled points.
    auto path_segments = kinodynamic_planner->getPathAsTrajectories();
    if (!path_segments.empty()) {
        // [MODIFICATION] Pass the vector of Trajectory objects to the manager.
        ros_manager->setPlannedTrajectory(path_segments);
        // Set the robot's starting state in the manager to initialize its clock.
        ros_manager->setInitialState(robot_initial_state);
    }
    
    RCLCPP_INFO(vis_node->get_logger(), "Initial plan complete. Executing...");

    auto tree_vis_timer = vis_node->create_wall_timer(
        std::chrono::milliseconds(200),
        [&]() { if (kinodynamic_planner) kinodynamic_planner->visualizeTreeReal(); });

    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(ros_manager);
    executor.add_node(vis_node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    // --- Main Execution and Replanning Loop ---
    resetAndPlaySimulation();
    const double goal_tolerance = 3.0;
    rclcpp::Rate loop_rate(20);
    auto time_limit = std::chrono::seconds(run_secs);
    auto global_start = std::chrono::steady_clock::now();

    std::vector<LogEntry> log_data;
 
    CALLGRIND_START_INSTRUMENTATION;
    while (g_running && rclcpp::ok()) {
        if (std::chrono::steady_clock::now() - global_start > time_limit) {
            std::cout << "[INFO] Time limit reached. Exiting loop.\n";
            break;
        }

        // [MODIFICATION] Get the robot's full kinodynamic state from the simulator.
        Eigen::VectorXd current_pos = ros_manager->getCurrentPosition();
        Eigen::VectorXd current_vel = ros_manager->getCurrentVelocity();
        Eigen::VectorXd current_accel = ros_manager->getCurrentAcceleration();

        // [MODIFICATION] Pass the full state to the planner to find the best anchor node.
        kinodynamic_planner->setRobotState(current_pos, current_vel, current_accel);

        // Check if the goal has been reached.
        double dist_to_goal = (current_pos.head<2>() - tree_root_state.head<2>()).norm();
        if (dist_to_goal < goal_tolerance) {
            RCLCPP_INFO(vis_node->get_logger(), "Goal Reached! Mission Accomplished.");
            g_running = false;
            break;
        }
        
        // This loop now performs replanning on every cycle to test performance.
        // In a real application, you would wrap this in a condition like `if(!is_path_still_safe)`.
        {
            auto snapshot = obstacle_checker->getAtomicSnapshot();
            auto start = std::chrono::steady_clock::now();
            kinodynamic_planner->updateObstacleSamples(snapshot.obstacles); 
            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            std::cout << "Time taken for update: " << duration.count() << " ms\n";
            // Run the core planning algorithm to repair/update the tree.
            start = std::chrono::steady_clock::now();
            planner->plan();

            end = std::chrono::steady_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            std::cout << "Time taken for plan: " << duration.count() << " ms\n";
            
            // ... logging logic ...

            // [MODIFICATION] After planning, set the state again to find the best anchor in the *updated* tree.
            kinodynamic_planner->setRobotState(ros_manager->getCurrentPosition(), ros_manager->getCurrentVelocity(), ros_manager->getCurrentAcceleration());

            // [MODIFICATION] Get the new path as a vector of Trajectory objects.
            auto new_path_segments = kinodynamic_planner->getPathAsTrajectories();

            if (!new_path_segments.empty()) {
                // std::reverse(new_path_segments.begin(), new_path_segments.end());
                // [MODIFICATION] Pass the new trajectory segments to the manager.
                ros_manager->setPlannedTrajectory(new_path_segments);
                
                // // For visualization, extract the sampled points from the segments.
                // std::vector<Eigen::VectorXd> new_path_points;
                // for (const auto& segment : new_path_segments) {
                //     new_path_points.insert(new_path_points.end(), segment.path_points.begin(), segment.path_points.end());
                // }
                kinodynamic_planner->visualizePath(new_path_segments);
            } else {
                RCLCPP_ERROR(vis_node->get_logger(), "Replanning failed to find a new path!");
                ros_manager->setPlannedTrajectory({}); 
            }
        }
        
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



