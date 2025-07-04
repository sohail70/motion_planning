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


// NEW: Define the states for our state machine
enum class RobotState {
    EXECUTING_MAIN_PATH,
    HOVERING_AND_REPLANNING,
    CRITICAL_FAILURE // A state for when no safe moves are possible
};


// Global running flag for signal handling
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
    // --- 1. Initial Setup ---
    rclcpp::init(argc, argv);
    signal(SIGINT, sigint_handler);


        // 1) Parse your flags
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

    // --- 2. Parameter Setup ---
    // Encapsulate parameters for better organization
    Params manager_params;
    manager_params.setParam("use_sim_time", true);
    manager_params.setParam("sim_time_step", -0.04); // Time-to-go consumed per sim step
    manager_params.setParam("sim_frequency_hz", 50);  // Smoothness of arrow
    manager_params.setParam("vis_frequency_hz", 10);  // Obstacle visualization rate

    Params gazebo_params;
    gazebo_params.setParam("robot_model_name", "tugbot");
    gazebo_params.setParam("default_robot_x", 48.0); // in case you want to test the planner without running gz sim
    gazebo_params.setParam("default_robot_y", 48.0);
    gazebo_params.setParam("world_name", "default");
    gazebo_params.setParam("use_range", false); // use_range and partial_update and use_heuristic are related! --> take care of this later!
    gazebo_params.setParam("sensor_range", 20.0);
    gazebo_params.setParam("estimation", true);
    gazebo_params.setParam("kf_model_type", "cv");

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
    planner_params.setParam("partial_update", false);
    planner_params.setParam("static_obs_presence", false);
    planner_params.setParam("obs_cache", false);
    planner_params.setParam("partial_plot", false);
    planner_params.setParam("use_heuristic", false);
    planner_params.setParam("ignore_sample", false);
    planner_params.setParam("prune", false);

    // --- 3. Object Initialization ---
    auto vis_node = std::make_shared<rclcpp::Node>("fmtx_dubins_visualizer",
        rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
    auto visualization = std::make_shared<RVizVisualization>(vis_node);
    auto sim_clock = vis_node->get_clock();

    auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_many_constant_acc_uncrowded.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_many_constant_acc.sdf");
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(sim_clock, gazebo_params, obstacle_info);

    // --- 4. Planner and Problem Definition (4D Dubins) ---
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
    auto statespace = std::make_shared<DubinsTimeStateSpace>(min_turning_radius, min_velocity, max_velocity);

    auto ros_manager = std::make_shared<DubinsROS2Manager>(obstacle_checker, visualization, manager_params, robot_initial_state);
    auto planner = PlannerFactory::getInstance().createPlanner(PlannerType::KinodynamicFMTX, statespace, problem_def, obstacle_checker);
    auto kinodynamic_planner = dynamic_cast<KinodynamicFMTX*>(planner.get());
    kinodynamic_planner->setClock(sim_clock);
    planner->setup(planner_params, visualization);

    // --- 5. Initial Plan ---
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

    // Visualization timer for the search tree
    auto tree_vis_timer = vis_node->create_wall_timer(
        std::chrono::milliseconds(100),
        [&]() { if (kinodynamic_planner) kinodynamic_planner->visualizeTree(); });

    // --- 6. Executor Setup ---
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(ros_manager);
    // executor.add_node(vis_node); // for dubin i do not plot the edges based on trajecotry because thats too demanding. i just connected the parent to child via simple edge so you might see soem edges going through obstalce but in reality the dubin is going around them so dont be alarm!

    std::thread executor_thread([&executor]() { executor.spin(); });

    // --- 7. Main Execution and Replanning Loop ---
    resetAndPlaySimulation();
    RCLCPP_INFO(vis_node->get_logger(), "Starting execution and monitoring loop. Press Ctrl+C to exit.");
    const double goal_tolerance = 2.0;
    rclcpp::Rate loop_rate(20);




    while (g_running && rclcpp::ok()) {
        // --- 1. Read robot state once ---
        Eigen::VectorXd current_state = ros_manager->getCurrentSimulatedState();
        kinodynamic_planner->setRobotState(current_state);

        if (current_state.size() == 0) {
            loop_rate.sleep();
            continue;
        }

        // --- 2. Check goal reached ---
        double dist_to_goal = (current_state.head<2>() - tree_root_state.head<2>()).norm();
        if (dist_to_goal < goal_tolerance) {
            RCLCPP_INFO(vis_node->get_logger(), "Goal Reached! Mission Accomplished.");
            g_running = false;
            break;
        }

        // --- 3. Update obstacle snapshot ---
        auto snapshot = obstacle_checker->getAtomicSnapshot();
        bool obstacles_changed = kinodynamic_planner->updateObstacleSamples(snapshot.obstacles);

        current_state = ros_manager->getCurrentSimulatedState();
        kinodynamic_planner->setRobotState(current_state);

        // --- 4. Reactive safety check ---
        bool path_invalid = !kinodynamic_planner->isPathStillValid(current_executable_path, current_state);

        // --- 5. Decide whether to replan ---
        if (path_invalid || obstacles_changed) {
            RCLCPP_INFO(vis_node->get_logger(), "Triggering replan (invalid path: %s, obstacles changed: %s)",
                        path_invalid ? "yes" : "no",
                        obstacles_changed ? "yes" : "no");

            current_state = ros_manager->getCurrentSimulatedState();
            kinodynamic_planner->setRobotState(current_state);
            // 6. Execute replanning
            planner->plan();

            Eigen::VectorXd fresh_robot_state = ros_manager->getCurrentSimulatedState();
            kinodynamic_planner->setRobotState(fresh_robot_state);
            auto new_path = kinodynamic_planner->getPathPositions();
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
            //////////////////////////
            // if (new_path.empty()) {
            //     // Failure: Attempt to generate a safe hover path.
            //     RCLCPP_WARN(vis_node->get_logger(), "Replanning failed. Searching for a safe hover maneuver.");

            //     auto dubins_time_ss = std::dynamic_pointer_cast<DubinsTimeStateSpace>(statespace);
            //     bool hover_path_found = false;

            //     if (dubins_time_ss) {
            //         const double hover_duration = 3.0; // seconds
            //         const double current_time = sim_clock->now().seconds();

            //         // 1. First, try to generate a hover path to the RIGHT.
            //         // FIX: Remove the class name prefix from the enum.
            //         auto hover_traj_right = dubins_time_ss->createHoverPath(
            //             fresh_robot_state, hover_duration, HoverDirection::RIGHT);

            //         // 2. Check if this path is safe.
            //         if (obstacle_checker->isTrajectorySafe(hover_traj_right, current_time)) {
            //             RCLCPP_INFO(vis_node->get_logger(), "Safe hover path found (RIGHT). Executing.");
            //             current_executable_path = hover_traj_right.path_points;
            //             hover_path_found = true;
            //         } else {
            //             // 3. If right is not safe, try to generate a path to the LEFT.
            //             RCLCPP_WARN(vis_node->get_logger(), "Right hover is unsafe. Trying left.");
            //             // FIX: Remove the class name prefix from the enum.
            //             auto hover_traj_left = dubins_time_ss->createHoverPath(
            //                 fresh_robot_state, hover_duration, HoverDirection::LEFT);

            //             // 4. Check if the left path is safe.
            //             if (obstacle_checker->isTrajectorySafe(hover_traj_left, current_time)) {
            //                 RCLCPP_INFO(vis_node->get_logger(), "Safe hover path found (LEFT). Executing.");
            //                 current_executable_path = hover_traj_left.path_points;
            //                 hover_path_found = true;
            //             }
            //         }
            //     }

            //     // 5. If neither hover path is safe, stop as a last resort.
            //     if (!hover_path_found) {
            //         RCLCPP_ERROR(vis_node->get_logger(), "No safe hover maneuver possible. Robot is trapped. Engaging E-STOP.");
            //         current_executable_path = { fresh_robot_state };
            //     }
            // } else {
            //     // Success case remains the same
            //     if (!kinodynamic_planner->arePathsSimilar(current_executable_path, new_path, 0.1)) {
            //         RCLCPP_INFO(vis_node->get_logger(), "Updating to new optimal path.");
            //         current_executable_path = new_path;
            //     } else {
            //         RCLCPP_INFO(vis_node->get_logger(), "New path similar to current. Keeping existing.");
            //     }
            // }



            //////////////////////////
            // Send updated (or stop) path to robot
            ros_manager->setPath(current_executable_path);
        }

        // --- 7. Always visualize current plan ---
        kinodynamic_planner->visualizePath(current_executable_path);

        // --- 8. Control layer can fetch and follow 'current_executable_path' at high rate elsewhere ---

        loop_rate.sleep();
    }

    // --- 8. Graceful Shutdown ---
    RCLCPP_INFO(vis_node->get_logger(), "Shutting down.");
    g_running = false;
    executor.cancel();
    if (executor_thread.joinable()) {
        executor_thread.join();
    }
    rclcpp::shutdown();
    return 0;
}
