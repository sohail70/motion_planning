// Copyright 2025 Soheil E.nia


#include "motion_planning/state_space/euclidean_statespace.hpp"
#include "motion_planning/planners/planner_factory.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/occupancygrid_obstacle_checker.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/utils/ros2_manager.hpp"
#include "motion_planning/utils/parse_sdf.hpp"
#include <valgrind/callgrind.h>



#include <gz/transport/Node.hh>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/server_control.pb.h>  // Often contains Boolean definition
#include <gz/msgs/boolean.pb.h>  // For Boolean response

// 2. Corrected simulation control function
void resetAndPlaySimulation()
{
    // Create Gazebo transport node
    gz::transport::Node node;
    
    // Reset the world
    {
        gz::msgs::WorldControl reset_req;
        reset_req.mutable_reset()->set_all(true);
        
        // Boolean response type is now properly included
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

    // Brief pause to ensure reset completes
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Play the simulation
    {
        gz::msgs::WorldControl play_req;
        play_req.set_pause(false);  // Unpause to play
        
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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    //////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // ─────────────────────────────────────────────────────────────────────────────
    // 1) Parse your flags
    int num_samples = 10000;
    double factor = 2.0;
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

    // 2) Seed RNG
    std::srand(seed);
    std::cout << "[INFO] seed=" << seed
                << ", samples=" << num_samples
                << ", factor=" << factor
                << ", duration=" << run_secs << "s\n";

    // ─────────────────────────────────────────────────────────────────────────────




    //////////////////////////////////////////////////////////////////////////////////////////////////

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
    gazebo_params.setParam("inflation", 0.0); // inflation added to obstalce radius virtually for the planner
    gazebo_params.setParam("persistent_static_obstacles", false);
    gazebo_params.setParam("estimation", true);

    Params planner_params;
    planner_params.setParam("num_of_samples", num_samples);
    planner_params.setParam("factor", factor);
    planner_params.setParam("use_kdtree", true); // for now the false is not impelmented! maybe i should make it default! can't think of a case of not using it but i just wanted to see the performance without it for low sample cases.
    planner_params.setParam("kdtree_type", "NanoFlann");
    planner_params.setParam("partial_update", true); // update the tree cost of the robot or not
    planner_params.setParam("ignore_sample", false); // false: no explicit obstalce check  -  true: explicit obstalce check in dynamic update
    planner_params.setParam("static_obs_presence", false); // to not process static obstalces twice because obstacle checker keeps sending all the obstalces! i geuss the persisten_static_obstalces needs to be true always
    planner_params.setParam("mode", 1); // 1: full node centric | 2: full obstalce centric | 3: node centric plus a map to obstalce check against speicific obstalces



    /////////////////////////////////////////////////////////////////////////////////////////////////

    // Create ROS node
    // auto node = std::make_shared<rclcpp::Node>("rrtx_visualizer");
    auto node = std::make_shared<rclcpp::Node>("rrtx_visualizer", rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("use_sim_time", ros2_manager_params.getParam<bool>("use_sim_time"))
    }));
    auto visualization = std::make_shared<RVizVisualization>(node);

    auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/static_world.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/static_removable_world.sdf");
    for (const auto& [name, info] : obstacle_info) {
        std::cout << name << ": " << info << "\n";
    }
    // GET THE CLOCK FROM THE NODE. This will be a sim clock.
    auto sim_clock = node->get_clock();
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(sim_clock, gazebo_params, obstacle_info);

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
    // problem_def->setStart(Eigen::VectorXd::Ones(dim) * -50);
    problem_def->setGoal(Eigen::VectorXd::Ones(dim) * 50); // where the robot starts!
    problem_def->setBounds(-50, 50);



    std::shared_ptr<StateSpace> statespace = std::make_shared<EuclideanStateSpace>(dim, 20000, seed);
    std::shared_ptr<Planner> planner = PlannerFactory::getInstance().createPlanner(PlannerType::RRTX, statespace,problem_def, obstacle_checker);
    planner->setup(planner_params, visualization);
    // Plan the static one!
    planner->plan();
    dynamic_cast<RRTX*>(planner.get())->dumpTreeToCSV("tree_run_rrtx.csv");



    // //----------- Waiting for the Sim Clock to start ------------ //
    // bool simulation_is_paused = true;
    // auto node_clock = ros2_manager->get_clock();
    // // We'll store the initial sim time
    // rclcpp::Time last_time = node_clock->now();
    // std::cout << "[DEBUG] Initially, last_time = " << last_time.seconds() 
    //         << " (sim seconds)\n";
    // std::cout << "[INFO] Waiting for gz-sim to unpause...\n";

    // while (rclcpp::ok() && simulation_is_paused)
    // {
    //     // 1) Spin to process any incoming clock messages
    //     rclcpp::spin_some(ros2_manager);

    //     // 2) Get current sim time
    //     rclcpp::Time current_time = node_clock->now();
    //     double dt = (current_time - last_time).seconds();

    //     // // 3) Print debug
    //     // std::cout << "[DEBUG] last_time=" << last_time.seconds() 
    //     //         << ", current_time=" << current_time.seconds() 
    //     //         << ", dt=" << dt << "\n";

    //     // 4) Check if it’s advanced
    //     if (current_time > last_time) {
    //         std::cout << "[DEBUG] => current_time is strictly greater than last_time, so sim is unpaused.\n";
    //         simulation_is_paused = false;
    //         std::cout << "[INFO] Simulation unpaused; starting to log data.\n";
    //     }
    //     else {
    //         // If we land here, sim time hasn't advanced since last check
    //         // std::cout << "[DEBUG] => Simulation still paused, waiting...\n";
    //         rclcpp::sleep_for(std::chrono::milliseconds(1));
    //     }

    //     last_time = current_time;
    // }
    // //----------------------------------------------------------- //


    resetAndPlaySimulation();

    
    // rclcpp::Rate loop_rate(2); // 2 Hz (500ms per loop)
    rclcpp::Rate loop_rate(30); // 10 Hz (100ms per loop)
    auto global_start = std::chrono::steady_clock::now();

    // Suppose you have a boolean that decides if we want a 20s limit
    bool limited = true;  // or read from params, or pass as an argument

    // Capture the "start" time if we plan to limit the loop
    auto start_time = std::chrono::steady_clock::now();
    auto time_limit = std::chrono::seconds(run_secs);

    std::vector<double> sim_durations;
    std::vector<std::tuple<double, double>> sim_duration_2;

    // Start profiling
    CALLGRIND_START_INSTRUMENTATION;


    while (rclcpp::ok()) {

        // 1) If we are limiting to 20s, check if we've exceeded that
        if (limited) {
            auto now = std::chrono::steady_clock::now();
            if (now - start_time > time_limit) {
                std::cout << "[INFO] 20 seconds have passed. Exiting loop.\n";
                break;  // exit the loop
            }
        }



        if (ros2_manager->hasNewGoal()) {
            start_position = ros2_manager->getStartPosition(); 
            problem_def->setStart(start_position);
            problem_def->setGoal(obstacle_checker->getRobotPosition());
            planner->setup(planner_params, visualization);
            planner->plan();   // For rrtx is different because we want to have the RRTX to cap to the max num samples and give us a tree first. for fair comparison with FMTX
        }


        // auto obstacles = obstacle_checker->getObstaclePositions();
        // auto robot = obstacle_checker->getRobotPosition();

        auto snapshot = obstacle_checker->getAtomicSnapshot();
        auto& obstacles = snapshot.obstacles;
        auto& robot = snapshot.robot_position;
        dynamic_cast<RRTX*>(planner.get())->setRobotIndex(robot); // UNCOMMENT THIS LATER!

        ////////// PLAN //////////
        auto start = std::chrono::steady_clock::now();
        dynamic_cast<RRTX*>(planner.get())->updateObstacleSamples(obstacles);
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (duration.count()>0)
            std::cout << "Time taken by update loop: " << duration.count() << " milliseconds\n";
        sim_durations.push_back(duration.count());

        // New metrics (elapsed_s, duration_ms)
        double elapsed_s = std::chrono::duration<double>(start - global_start).count();
        double duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
        sim_duration_2.emplace_back(elapsed_s, duration_ms);



        // std::vector<Eigen::VectorXd> shortest_path_ = dynamic_cast<RRTX*>(planner.get())->getSmoothedPathPositions(5, 2);
        // ros2_manager->followPath(shortest_path_);

        ////////// VISUALIZE /////
        // dynamic_cast<RRTX*>(planner.get())->visualizePath(dynamic_cast<RRTX*>(planner.get())->getPathIndex());
        // dynamic_cast<RRTX*>(planner.get())->visualizeSmoothedPath(shortest_path_);
        dynamic_cast<RRTX*>(planner.get())->visualizeTree();

        rclcpp::spin_some(ros2_manager);
        loop_rate.sleep();

    }
    // Stop profiling
    CALLGRIND_STOP_INSTRUMENTATION;

    const bool SAVE_TIMED_DATA = true; // Set to false to save raw durations
    int num_of_samples_ = planner_params.getParam<int>("num_of_samples");


    if (limited) {
        std::time_t now_time = std::time(nullptr); 
        std::tm* local_tm = std::localtime(&now_time);

        int day    = local_tm->tm_mday;
        int month  = local_tm->tm_mon + 1;
        int year   = local_tm->tm_year + 1900;
        int hour   = local_tm->tm_hour;
        int minute = local_tm->tm_min;
        int second = local_tm->tm_sec;

        // Create base filename with timestamp
        std::string planner_type = "rrtx";
        std::string base_filename = "sim_" + planner_type + "_" +
            std::to_string(num_of_samples_) + "samples_" + 
            std::to_string(day) + "_" +
            std::to_string(month) + "_" +
            std::to_string(year) + "_" +
            std::to_string(hour) + "_" +
            std::to_string(minute) + "_" +
            std::to_string(second);

        if (SAVE_TIMED_DATA) {
            // Save timed data (elapsed_s, duration_ms)
            std::string filename = base_filename + "_timed.csv";
            std::cout << "Writing timed durations to: " << filename << std::endl;

            std::ofstream out(filename);
            if (!out.is_open()) {
                std::cerr << "Error: failed to open " << filename << std::endl;
                return 1;
            }

            out << "elapsed_s,duration_ms\n"; // CSV header
            for (const auto& [elapsed, duration] : sim_duration_2) {
                out << elapsed << "," << duration << "\n";
            }
            out.close();
        } else {
            // Save raw durations (legacy format)
            std::string filename = base_filename + "_raw.csv";
            std::cout << "Writing raw durations to: " << filename << std::endl;

            std::ofstream out(filename);
            if (!out.is_open()) {
                std::cerr << "Error: failed to open " << filename << std::endl;
                return 1;
            }

            for (const auto& d : sim_durations) {
                out << d << "\n";
            }
            out.close();
        }

        std::cout << "Done writing CSV.\n";
    }




  
    rclcpp::shutdown();

}
