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








std::atomic<bool> g_running{true}; // Flag to control the infinite loop


void sigint_handler(int sig) {
    g_running = false; // Stop the main loop
    rclcpp::shutdown();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    //////////////////////////////////////////////////////////////////////////////////////////////////
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


    //////////////////////////////////////////////////////////////////////////////////////////////////
    // ─────────────────────────────────────────────────────────────────────────────
    // 1) Parse your flags
    int num_samples = 10;
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

    // 2) Seed RNG
    std::srand(seed);
    std::cout << "[INFO] seed=" << seed
                << ", samples=" << num_samples
                << ", factor=" << factor
                << ", duration=" << run_secs << "s\n";

    // ─────────────────────────────────────────────────────────────────────────────



    // Create Params for ROS2Manager
    Params manager_params;
    manager_params.setParam("use_sim_time", true);
    manager_params.setParam("simulation_time_step", -0.05); // 50 Hz simulation loop
    manager_params.setParam("thruster_state_dimension", 5);

    Params gazebo_params;
    gazebo_params.setParam("robot_model_name", "tugbot");
    gazebo_params.setParam("default_robot_x", 48.0); 
    gazebo_params.setParam("default_robot_y", 48.0);
    gazebo_params.setParam("world_name", "default");
    gazebo_params.setParam("use_range", false); 
    gazebo_params.setParam("sensor_range", 20.0);
    gazebo_params.setParam("estimation", true);
    gazebo_params.setParam("inflation", 0.0); 
    gazebo_params.setParam("persistent_static_obstacles", false);

    Params planner_params;
    planner_params.setParam("num_of_samples", num_samples);
    planner_params.setParam("factor", factor);
    planner_params.setParam("use_kdtree", true); // for now the false is not impelmented! maybe i should make it default! can't think of a case of not using it but i just wanted to see the performance without it for low sample cases.
    planner_params.setParam("kdtree_type", "NanoFlann");
    planner_params.setParam("partial_update", false); //update till the robot's costToInit
    planner_params.setParam("static_obs_presence", false); // to not process static obstalces twice because obstacle checker keeps sending all the obstalces! i geuss the persisten_static_obstalces needs to be true always
    /*
        we can cache and its useful because near obstacle there comes a time that too many z indices came up with the same best neighbor node for specific x index
        and since there is an obs in between then we end up needing to re check the same obstacle check between nodes
        I reckon if i use blocked_neighbor variable then we won't need this but that blocked_neighbor variable introduces it own overhead that only worth to use
        if we are using fmta 

        another thing i notices is this doesnt help much with performance in my 2D case. but im gonna leave it in case i have time to test it in more dimensions
    
    */
    planner_params.setParam("obs_cache", false); // TODO: later i should implement it in the fmt node (edge info) it self but right now im using a hash map in fmtx it self which i dont think is efficient but at the same time since im in replanning mode then restarting the obs status of edges is also maybe a challenge but i think its doable
    planner_params.setParam("partial_plot", false);
    planner_params.setParam("use_heuristic", false); // TODO: I need to verify if its legit workingor not.
    planner_params.setParam("ignore_sample", false); // false: no explicit obstalce check  -  true: explicit obstalce check in dynamic update --> when ignore_sample true the prune is not happening anymore so doesnt matter what you put there
    /*
        right now ignore samples is being used with specific way of finding the samples and also the collision check also happens in fmt expand
        later maybe i could combine it with obstale aware distance and no collision checks and see what happens
    */
    planner_params.setParam("prune", false); // prune == true means do an obstalce check in handlAdd/Remove and set the neighbor cost to inf and DO NOT  obstalce check in plan , prune==false means do not do an obstalce check in handleAdd/Remove and delay it in plan --> the delayed part makes it more expensive in case of high obstalce but in case of low obstalce its faster! (also for high number of samples the delayed part is slower)--> prune true overall is faster i guess
    /*
        IMPORTANT NOTE: prune vs plan? in prune we do obstacle check in local vicinity of obstalce and set cost to neighbor to inf in add obstalce and reset in remove obstalce
                        and since we invalidated the edges between those nodes on obstalce and their neighbor, we don't need to do an obstacle check in plan function 
                        but i wonder what if we do not do an obstacle check in add/remove obstalce and postpone the check to plan ? this is in line with fmt philosophy but
                        the thing is then we have to do lots of obstacle checks for all the orphaned edges! as opposed to do only local obstacle checks so the question boild down
                        to number of obstacle checks in local vicnity of the obstalce with ALL their neighbors and the number of delayed obstacle checks in plan function where only the query
                        current edge-> best_neighbor edge. in my tests the prune version where we do not delay checks works faster but maybe at high dimensional space its better to use the delayed version!
                        thats another topic of research i'll do later!

                        all in all prune is like doing the rrtx approach in setting the distance to inf based on explicit obstacle check between a node in obstalce and its neighbors

        NEW UPDATE: in 30 obstalce case testing with the whole environment visible doing the fmt style was much better!--> maybe i should put two variants!!! --> Need to decide !
    */

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // --- 3. Object Initialization ---
    auto vis_node = std::make_shared<rclcpp::Node>("fmtx_thruster_visualizer",
        rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
    auto visualization = std::make_shared<RVizVisualization>(vis_node);
    auto sim_clock = vis_node->get_clock();

    auto obstacle_info = parseSdfObstacles("your_world.sdf");
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(sim_clock, gazebo_params, obstacle_info);

    // --- 4. Planner and Problem Definition (5D Thruster) ---
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
    double time_budget = 40.0;
    robot_initial_state << 48.0, 48.0, 0.0, 0.0, time_budget;
    problem_def->setGoal(robot_initial_state);

    // Define 5D bounds, including velocity limits.
    Eigen::VectorXd lower_bounds(dim), upper_bounds(dim);
    double max_vel = 10.0; // Max velocity in any direction
    lower_bounds << -50.0, -50.0, -max_vel, -max_vel, 0.0;
    upper_bounds << 50.0, 50.0,  max_vel,  max_vel, time_budget;
    problem_def->setBounds(lower_bounds, upper_bounds);

    // Create the thruster state space with max acceleration.
    double max_acceleration = 2.0; // m/s^2
    auto statespace = std::make_shared<ThrusterSteerStateSpace>(dim, max_acceleration);
    
    // The ROS manager now uses the thruster-specific version.
    auto ros_manager = std::make_shared<ROS2Manager>(obstacle_checker, visualization, manager_params);
    
    auto planner = PlannerFactory::getInstance().createPlanner(PlannerType::KinodynamicFMTX, statespace, problem_def, obstacle_checker);
    auto kinodynamic_planner = dynamic_cast<KinodynamicFMTX*>(planner.get());
    kinodynamic_planner->setClock(sim_clock);
    planner->setup(planner_params, visualization);

    // --- 5. Initial Plan ---
    RCLCPP_INFO(vis_node->get_logger(), "Running initial plan...");
    planner->plan();

    // To get the path, we need a function that stitches the ExecutionTrajectory data.
    // (See Section 3 for the implementation of this new function).
    kinodynamic_planner->setRobotState(robot_initial_state);
    auto path = kinodynamic_planner->getPathPositions(); // Returns the correct format directly
    if (!path.empty()) {
        ros_manager->setPlannedThrusterTrajectory(path);
                // Set the robot's starting state in the manager to initialize its clock.
        ros_manager->setInitialState(robot_initial_state);
    }
    
    RCLCPP_INFO(vis_node->get_logger(), "Initial plan complete. Executing...");

    // Visualization timer for the search tree
    auto tree_vis_timer = vis_node->create_wall_timer(
        std::chrono::milliseconds(200),
        [&]() { if (kinodynamic_planner) kinodynamic_planner->visualizeTree(); });

    // --- 6. Executor Setup ---
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(ros_manager);
    // executor.add_node(vis_node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    // --- 7. Main Execution and Replanning Loop ---
    resetAndPlaySimulation();
    const double goal_tolerance = 2.0;
    rclcpp::Rate loop_rate(20); // Replanning loop can run slower

    while (g_running && rclcpp::ok()) {
        // Get the robot's current 5D state from the simulator.
        Eigen::VectorXd current_state = ros_manager->getCurrentKinodynamicState();
        kinodynamic_planner->setRobotState(current_state);

        // Check for goal arrival.
        double dist_to_goal = (current_state.head<2>() - tree_root_state.head<2>()).norm();
        if (dist_to_goal < goal_tolerance) {
            RCLCPP_INFO(vis_node->get_logger(), "Goal Reached! Mission Accomplished.");
            g_running = false;
            break;
        }

        // Update obstacles and check if replanning is needed.
        auto snapshot = obstacle_checker->getAtomicSnapshot();
        bool obstacles_changed = kinodynamic_planner->updateObstacleSamples(snapshot.obstacles);

        // Here, you would also add a call to a path validator like `isPathStillValid`.
        // For simplicity, we replan only on obstacle changes.
        if (obstacles_changed) {
            RCLCPP_INFO(vis_node->get_logger(), "Obstacles changed. Triggering replan...");
            
            // Update the planner's knowledge of the robot's state right before planning.
            kinodynamic_planner->setRobotState(ros_manager->getCurrentKinodynamicState());
            
            planner->plan();

            // Get the new execution trajectory.
            auto path = kinodynamic_planner->getPathPositions(); // Returns the correct format directly
            if (!path.empty()) {
                ros_manager->setPlannedThrusterTrajectory(path);
                kinodynamic_planner->visualizePath(path);
            }
        }
        

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



