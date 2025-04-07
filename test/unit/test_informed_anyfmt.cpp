// Copyright 2025 Soheil E.nia

#include "motion_planning/state_space/euclidean_statespace.hpp"
#include "motion_planning/planners/planner_factory.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/occupancygrid_obstacle_checker.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/utils/ros2_manager.hpp"
#include "motion_planning/utils/parse_sdf.hpp"
#include <valgrind/callgrind.h>

#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <thread>
#include <atomic>

std::atomic<bool> running{true}; // Flag to control the infinite loop
pid_t child_pid = -1;

void runRosGzBridge() {
    // // Launch parameter_bridge as a child process with suppressed logs
    // child_pid = fork();
    // if (child_pid == 0) {
    //     // Child process: Execute the ros_gz_bridge command with suppressed logs
    //     execlp("ros2", "ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", 
    //            "-p", "config_file:=/home/sohail/jazzy_ws/src/simple_robot/params/ros_gz_bridge.yaml",
    //            "--log-level", "error", // Suppress logs below "error" severity
    //            (char *)NULL);
    //     // If execlp fails, print an error and exit
    //     std::cerr << "Failed to launch parameter_bridge: " << strerror(errno) << std::endl;
    //     exit(EXIT_FAILURE);
    // } else if (child_pid < 0) {
    //     // Fork failed
    //     std::cerr << "Failed to fork process: " << strerror(errno) << std::endl;
    //     return;
    // }

    // // Parent process: Wait for the child process to finish
    // int status;
    // waitpid(child_pid, &status, 0);
    // if (WIFEXITED(status)) {
    //     std::cout << "ros_gz_bridge exited with status: " << WEXITSTATUS(status) << std::endl;
    // } else {
    //     std::cerr << "ros_gz_bridge terminated abnormally" << std::endl;
    // }
}

void sigint_handler(int sig) {
    if (child_pid > 0) {
        // Terminate the child process
        kill(child_pid, SIGTERM);
        waitpid(child_pid, nullptr, 0);
        child_pid = -1;
    }
    running = false; // Stop the main loop
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

    // Launch ros_gz_bridge in a separate thread
    std::thread ros_gz_bridge_thread(runRosGzBridge);
    ros_gz_bridge_thread.detach(); // Detach the thread to run independently


    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Create Params for Pure-Pursuit Controller
    Params controller_params;
    controller_params.setParam("kp_angular", 1.2);
    controller_params.setParam("ki_angular", 0.05);
    controller_params.setParam("kd_angular", 0.2);
    controller_params.setParam("cross_track_gain", 0.8);
    controller_params.setParam("max_angular_speed", 0.5);
    controller_params.setParam("lookahead_distance", 1.5);
    controller_params.setParam("control_loop_dt", 0.005);

    // Create Params for Nav2Controller
    Params nav2_controller_params;
    nav2_controller_params.setParam("follow_path_topic", "/follow_path");
    nav2_controller_params.setParam("max_speed", 2.0);


    Params DWA;
    // ========== Core motion limits ==========
    DWA.setParam("max_speed",         3.0);   // Robot can go up to 3 m/s
    DWA.setParam("min_speed",        -1.0);   // Allow reversing if needed
    DWA.setParam("max_yawrate",       0.8);   // Turn rate up to 1.5 rad/s
    DWA.setParam("max_accel",         3.0);   // Accelerate up to 2 m/s^2
    DWA.setParam("max_decel",         3.0);   // Decelerate up to 2 m/s^2
    DWA.setParam("max_dyawrate",      2.0);   // Angular acceleration limit
    DWA.setParam("robot_radius",      0.3);

    // ========== Sampling and horizon ==========
    DWA.setParam("dt",               0.1);    // Simulation time step in DWA
    DWA.setParam("predict_time",     5.0);    // 2s horizon for quick re-planning
    int sim_steps_ = (int)(DWA.getParam<double>("predict_time") / DWA.getParam<double>("dt"));
    // -> 2.0 / 0.1 = 20 steps
    DWA.setParam("sim_steps", sim_steps_);

    DWA.setParam("speed_resolution",  0.1);
    DWA.setParam("yawrate_resolution",0.1);

    // ========== Cost weights ==========
    DWA.setParam("obstacle_cost_gain",  5.0); // Higher => more aggressive obstacle avoidance
    DWA.setParam("speed_cost_gain",     0.3); // Medium => encourages higher speed, but not crazy
    DWA.setParam("goal_cost_gain",      3.0); // Balanced
    DWA.setParam("path_cost_gain",      0.3); // Enough to stay near path, but not too strict

    

    // Create Params for ROS2Manager
    Params ros2_manager_params;
    ros2_manager_params.setParam("use_sim_time", true);
    ros2_manager_params.setParam("follow_path", false);
    ros2_manager_params.setParam("controller", "pure_pursuit");
    // ros2_manager_params.setParam("controller", "dwa");

    Params gazebo_params;
    gazebo_params.setParam("robot_model_name", "tugbot");
    gazebo_params.setParam("default_robot_x", 48.0); // in case you want to test the planner without running gz sim
    gazebo_params.setParam("default_robot_y", 48.0);
    gazebo_params.setParam("world_name", "default");
    gazebo_params.setParam("use_range", false); // use_range and partial_update and use_heuristic are related! --> take care of this later!
    gazebo_params.setParam("sensor_range", 20.0);
    /*
        When you use ignore_sample == true i don't think would need a inflation specially in low sample case --> math can be proved i guess.
    */
    gazebo_params.setParam("inflation", 0.0); //2.0 meters --> this will be added to obstalce radius when obstalce checking --> minimum should be D-ball containing the robot
    gazebo_params.setParam("persistent_static_obstacles", true);

    Params planner_params;
    planner_params.setParam("num_of_samples", 50);
    planner_params.setParam("num_batch", 100); // Adding samples (any time!)
    planner_params.setParam("use_kdtree", true); // for now the false is not impelmented! maybe i should make it default! can't think of a case of not using it but i just wanted to see the performance without it for low sample cases.
    planner_params.setParam("kdtree_type", "NanoFlann");
    planner_params.setParam("obs_cache", true);
    planner_params.setParam("partial_plot", false);
    /////////////////////////////////////////////////////////0////////////////////////////////////////

    // Create ROS node
    auto node = std::make_shared<rclcpp::Node>("informed_anyfmt_visualizer");
    auto visualization = std::make_shared<RVizVisualization>(node);

    auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/static_world.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/static_world2.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/static_removable_world.sdf");
    for (const auto& [name, info] : obstacle_info) {
        std::cout << name << ": " << info << "\n";
    }
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(gazebo_params, obstacle_info);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Create Controller and Nav2Controller objects
    auto controller = std::make_shared<Controller>(controller_params);
    auto nav2_controller = std::make_shared<Nav2Controller>(nav2_controller_params); // TODO: later i should omit this from ros2 manager by creating a highe level abstraction for controllers!
    auto dwa_controller =  std::make_shared<DWAPlanner>(DWA,obstacle_checker);

    /////////////////////////////////////////////////////////////////////////////////////////////////

    auto ros2_manager = std::make_shared<ROS2Manager>(obstacle_checker, visualization, controller, nav2_controller,dwa_controller, ros2_manager_params);


    int dim = 2;
    Eigen::VectorXd start_position = ros2_manager->getStartPosition(); // I put a cv in here so i needed the above thread so it wouldn't stop the ros2 callbacks! --> also if use_rviz_goal==false no worries because the default value for this func is 0,0


    auto problem_def = std::make_shared<ProblemDefinition>(dim);
    auto snapshot = obstacle_checker->getAtomicSnapshot(); //This triggers to save the current obstalce positions
    problem_def->setStart(start_position); //Root of the tree
    // problem_def->setStart(Eigen::VectorXd::Zero(dim));
    problem_def->setGoal(Eigen::VectorXd::Ones(dim) * 50); // where the robot starts!
    problem_def->setBounds(-50, 50);



    std::unique_ptr<StateSpace> statespace = std::make_unique<EuclideanStateSpace>(dim, 30000);
    std::unique_ptr<Planner> planner = PlannerFactory::getInstance().createPlanner(PlannerType::InformedANYFMT, std::move(statespace),problem_def, obstacle_checker);
    planner->setup(planner_params, visualization);

    auto start = std::chrono::high_resolution_clock::now();
    planner->plan();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken for the update : " << duration.count() 
                << " milliseconds\n";



    rclcpp::Rate loop_rate(3000);

    int counter = 0;

    CALLGRIND_START_INSTRUMENTATION;
    // The main loop
    while (running && rclcpp::ok()) {
        // if(counter>100)
        //     break;
        counter++;

        if (ros2_manager->hasNewGoal()) {
            start_position = ros2_manager->getStartPosition(); 
            auto snapshot = obstacle_checker->getAtomicSnapshot(); // In case i changed the robot/obstalce positions manually for the new plan
            problem_def->setStart(start_position);
            problem_def->setGoal(snapshot.robot_position);
            planner->setup(planner_params, visualization);
            auto start = std::chrono::high_resolution_clock::now();
            planner->plan();
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            std::cout << "Time taken for the update : " << duration.count() 
                        << " milliseconds\n";

        }

        auto start = std::chrono::high_resolution_clock::now();
        planner->plan();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "time taken for the update : " << duration.count() << " milliseconds\n";

        // std::vector<Eigen::VectorXd> shortest_path_ = dynamic_cast<InformedANYFMT*>(planner.get())->getSmoothedPathPositions(5, 2);
        // ros2_manager->followPath(shortest_path_);

        // dynamic_cast<InformedANYFMT*>(planner.get())->visualizeSmoothedPath(shortest_path_);
        dynamic_cast<InformedANYFMT*>(planner.get())->visualizePath(dynamic_cast<InformedANYFMT*>(planner.get())->getPathIndex());

        // dynamic_cast<InformedANYFMT*>(planner.get())->visualizeHeapAndUnvisited();
        dynamic_cast<InformedANYFMT*>(planner.get())->visualizeTree();
        rclcpp::spin_some(ros2_manager);
        loop_rate.sleep();
    }
    CALLGRIND_STOP_INSTRUMENTATION;

    
    // Cleanup
    if (child_pid > 0) {
        kill(child_pid, SIGTERM);
        waitpid(child_pid, nullptr, 0);
    }
    rclcpp::shutdown();
    return 0;
}







