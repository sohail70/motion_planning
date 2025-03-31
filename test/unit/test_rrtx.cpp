// Copyright 2025 Soheil E.nia
/**
 * TODO: cullNeighbor makes the tree to have sub-optimal connections! 
 * TODO: the only difference in my versrion is using samples_in_obstalce_ in the removeObstalce and etc. also i didin't create  the statespace after checking the parent in extent function (because weirdly the algorithm demands an lmc beofre deciding to have the sample as a tree node or not!)
 */

#include "motion_planning/state_space/euclidean_statespace.hpp"
#include "motion_planning/planners/planner_factory.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/occupancygrid_obstacle_checker.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/utils/ros2_manager.hpp"
#include "motion_planning/utils/parse_sdf.hpp"


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

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
    DWA.setParam("min_speed",        -2.0);   // Allow reversing if needed
    DWA.setParam("max_yawrate",       0.8);   // Turn rate up to 1.5 rad/s
    DWA.setParam("max_accel",         2.0);   // Accelerate up to 2 m/s^2
    DWA.setParam("max_decel",         2.0);   // Decelerate up to 2 m/s^2
    DWA.setParam("max_dyawrate",      1.0);   // Angular acceleration limit
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
    DWA.setParam("obstacle_cost_gain",  3.0); // Higher => more aggressive obstacle avoidance
    DWA.setParam("speed_cost_gain",     1.0); // Medium => encourages higher speed, but not crazy
    DWA.setParam("goal_cost_gain",      1.0); // Balanced
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
    gazebo_params.setParam("persistent_static_obstacles", true);

    Params planner_params;
    planner_params.setParam("num_of_samples", 5000);
    planner_params.setParam("use_kdtree", true); // for now the false is not impelmented! maybe i should make it default! can't think of a case of not using it but i just wanted to see the performance without it for low sample cases.
    planner_params.setParam("kdtree_type", "NanoFlann");



    /////////////////////////////////////////////////////////////////////////////////////////////////
    // Create ROS node
    auto node = std::make_shared<rclcpp::Node>("rrtx_visualizer");
    auto visualization = std::make_shared<RVizVisualization>(node);

    auto obstacle_radii = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world.sdf");
    // auto obstacle_radii = parseSdfForObstacleRadii("/home/sohail/gazeb/GAZEBO_MOV/static_world.sdf");
    // auto obstacle_radii = parseSdfForObstacleRadii("/home/sohail/gazeb/GAZEBO_MOV/static_removable_world.sdf");
for (const auto& [name, info] : obstacle_radii) {
    std::cout << name << ": " << info << "\n";
}
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(gazebo_params, obstacle_radii);

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
    problem_def->setStart(start_position); //Root of the tree
    // problem_def->setStart(Eigen::VectorXd::Zero(dim));
    problem_def->setGoal(Eigen::VectorXd::Ones(dim) * 50); // where the robot starts!
    problem_def->setBounds(-50, 50);



    std::unique_ptr<StateSpace> statespace = std::make_unique<EuclideanStateSpace>(dim, 5000);
    std::unique_ptr<Planner> planner = PlannerFactory::getInstance().createPlanner(PlannerType::RRTX, std::move(statespace),problem_def, obstacle_checker);
    planner->setup(planner_params, visualization);


    // Plan the static one!
    planner->plan();
    while (rclcpp::ok()) {
        if (ros2_manager->hasNewGoal()) {
            start_position = ros2_manager->getStartPosition(); 
            problem_def->setStart(start_position);
            problem_def->setGoal(obstacle_checker->getRobotPosition());
            planner->setup(planner_params, visualization);
            planner->plan();   // For rrtx is different because we want to have the RRTX to cap to the max num samples and give us a tree first. for fair comparison with FMTX
        }


        auto obstacles = obstacle_checker->getObstaclePositions();
        auto robot = obstacle_checker->getRobotPosition();
        // if (robot(0) != 0.0 && robot(1) != 0.0 && use_robot==true) // Else it will only use the setGoal to set the vbot
            dynamic_cast<RRTX*>(planner.get())->setRobotIndex(robot);
        ////////// PLAN //////////
        auto start = std::chrono::high_resolution_clock::now();
        dynamic_cast<RRTX*>(planner.get())->updateObstacleSamples(obstacles);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (duration.count()>0)
            std::cout << "Time taken by update loop: " << duration.count() << " milliseconds\n";
        

        std::vector<Eigen::VectorXd> shortest_path_ = dynamic_cast<RRTX*>(planner.get())->getSmoothedPathPositions(5, 2);
        ros2_manager->followPath(shortest_path_);

        ////////// VISUALIZE /////
        // dynamic_cast<RRTX*>(planner.get())->visualizePath(dynamic_cast<RRTX*>(planner.get())->getPathIndex());
        dynamic_cast<RRTX*>(planner.get())->visualizeSmoothedPath(shortest_path_);
        dynamic_cast<RRTX*>(planner.get())->visualizeTree();

        rclcpp::spin_some(ros2_manager);
    }

    // // Start the timer
    // auto start_time = std::chrono::high_resolution_clock::now();

    // // Run the loop for 20 seconds
    // while (std::chrono::duration_cast<std::chrono::seconds>(
    //         std::chrono::high_resolution_clock::now() - start_time).count() < 20) {
    //     auto obstacles = obstacle_checker->getObstaclePositions();
    //     auto robot = obstacle_checker->getRobotPosition();
    //     if (robot(0) != 0.0 && robot(1) != 0.0 && use_robot==true) // Else it will only use the setGoal to set the vbot
    //         dynamic_cast<RRTX*>(planner.get())->setRobotIndex(robot);
    //     ////////// PLAN //////////
    //     auto start = std::chrono::high_resolution_clock::now();
    //     dynamic_cast<RRTX*>(planner.get())->updateObstacleSamples(obstacles);
    //     auto end = std::chrono::high_resolution_clock::now();
    //     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //     if (duration.count()>0)
    //         std::cout << "Time taken by update loop: " << duration.count() << " milliseconds\n";
        
    //     ////////// VISUALIZE /////
    //     // dynamic_cast<RRTX*>(planner.get())->visualizePath(dynamic_cast<RRTX*>(planner.get())->getPathIndex());
    //     // dynamic_cast<RRTX*>(planner.get())->visualizeTree();

    //     rclcpp::spin_some(ros2_manager);
    // }
    rclcpp::shutdown();

}
