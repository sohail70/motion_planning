#include "motion_planning/utils/ros2_manager.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/parse_sdf.hpp"

int main(int argc, char** argv) {
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
    gazebo_params.setParam("estimation", true);
    /*
        When you use ignore_sample == true i don't think would need a inflation specially in low sample case --> math can be proved i guess.
    */
    gazebo_params.setParam("inflation", 0.0); //2.0 meters --> this will be added to obstalce radius when obstalce checking --> minimum should be D-ball containing the robot
    gazebo_params.setParam("persistent_static_obstacles", true);


    /////////////////////////////////////////////////////////////////////////////////////////////////
    auto node = std::make_shared<rclcpp::Node>("visualizer", rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("use_sim_time", ros2_manager_params.getParam<bool>("use_sim_time"))
    }));

    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world.sdf");
    auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_4_obs.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/static_world2.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/static_removable_world.sdf");

    for (const auto& [name, info] : obstacle_info) {
        std::cout << name << ": " << info << "\n";
    }
    // GET THE CLOCK FROM THE NODE. This will be a sim clock.
    auto sim_clock = node->get_clock();
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(sim_clock, gazebo_params, obstacle_info); // Robot model name and obstacle radius

    auto visualizer = std::make_shared<RVizVisualization>(node);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Create Controller and Nav2Controller objects
    auto controller = std::make_shared<Controller>(controller_params);
    auto nav2_controller = std::make_shared<Nav2Controller>(nav2_controller_params); // TODO: later i should omit this from ros2 manager by creating a highe level abstraction for controllers!
    auto dwa_controller =  std::make_shared<DWAPlanner>(DWA,obstacle_checker);

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // auto ros2_manager = std::make_shared<ROS2Manager>(obstacle_checker, visualizer);
    auto ros2_manager = std::make_shared<ROS2Manager>(obstacle_checker, visualizer, controller, nav2_controller,dwa_controller, ros2_manager_params);

    rclcpp::spin(ros2_manager);
    rclcpp::shutdown();
    return 0;
}
