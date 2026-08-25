// Copyright 2025 Soheil E.nia
#include "motion_planning/planners/planner_factory.hpp"
#include "motion_planning/state_space/rdt_statespace.hpp"
#include "motion_planning/state_space/dubins_time_statespace.hpp"
#include "motion_planning/state_space/thruster_statespace.hpp"
#include "motion_planning/utils/deterministic_obstacle_checker.hpp"
#include "motion_planning/utils/parse_sdf.hpp"
#include "motion_planning/utils/ros2_manager_base.hpp" 
#include "motion_planning/utils/ros2_manager_r2t.hpp"
#include "motion_planning/utils/ros2_manager_dubin.hpp"
#include "motion_planning/utils/ros2_manager_thruster.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <valgrind/callgrind.h>

#define SCREEN_SHOT 0
#define USE_METRIC 0

/*
    USE_REALTIME: Real-time viability on this hardware
*/
#define USE_REALTIME 0 // Penalizing Long plan time and making the simulator to catch up (and we test collisions in stepOnce in manager classes to benchmark)
#define USE_1_X 0 // Just for smooth visualization purposes


struct ExperimentConfig {
    std::string name;
    std::string planner_type;
    std::string state_space_type;
    std::string manager_type;
    
    double time_budget;
    int duration_limit;
    double slice_time;
    unsigned int seed;
    std::vector<double> start_state;
    std::vector<double> goal_state;
    int dimensions;
    std::vector<double> bounds_min;
    std::vector<double> bounds_max;
    double min_velocity = 0.0;
    double max_velocity = 20.0;
    double min_turning_radius = 2.0;
    double max_acceleration = 5.0;
    std::string sdf_path;
    int num_pillar_nodes = 50; 
    double goal_radius = 0.5;
    double vis_frequency_hz = 30;
    std::map<std::string, std::string> planner_params_str;
    std::map<std::string, std::string> manager_params_str;
    std::map<std::string, std::string> gazebo_params_str;

    bool heuristic = 0;


    bool is_geometric_mode() const {
        // Use the explicit planner flag as primary source
        if (planner_params_str.count("is_geometric_mode")) {
            std::string val = planner_params_str.at("is_geometric_mode");
            return val == "true";
        }
        return false;
    }

    double time_extent() const {              // single source for time dimension
        return is_geometric_mode() ? 0.0 : time_budget;
    }

    double get_inflation() const {
        // Prefer gazebo → manager → default (minimal logic)
        if (gazebo_params_str.count("inflation"))
            return std::stod(gazebo_params_str.at("inflation"));
        if (manager_params_str.count("inflation"))
            return std::stod(manager_params_str.at("inflation"));
        return 0.75;
    }

};
ExperimentConfig loadConfig(const std::string& filepath) {
    YAML::Node config = YAML::LoadFile(filepath);
    ExperimentConfig c;

    // Experiment block (required)
    c.name            = config["experiment"]["name"].as<std::string>();
    c.planner_type    = config["experiment"]["planner_type"].as<std::string>();
    c.state_space_type = config["experiment"]["state_space_type"].as<std::string>();
    c.manager_type    = config["experiment"]["manager_type"].as<std::string>();

    // Simulation block (time_budget optional for geometric)
    if (config["simulation"]["time_budget"]) {
        c.time_budget = config["simulation"]["time_budget"].as<double>();
    }
    if (config["simulation"]["duration_limit"]) {
        c.duration_limit = config["simulation"]["duration_limit"].as<int>();
    } 
    c.slice_time = config["simulation"]["slice_time"].as<double>();
    c.seed       = config["simulation"]["seed"].as<unsigned int>();

    // Robot block (spatial parts always present, dynamics optional)
    c.start_state    = config["robot"]["start_state"].as<std::vector<double>>();
    c.goal_state     = config["robot"]["goal_state"].as<std::vector<double>>();
    c.dimensions     = config["robot"]["dimensions"].as<int>();
    c.bounds_min     = config["robot"]["bounds_min"].as<std::vector<double>>();
    c.bounds_max     = config["robot"]["bounds_max"].as<std::vector<double>>();

    // vis
    c.vis_frequency_hz = config["manager_params"]["vis_frequency_hz"].as<double>();

    // Dynamics — only set if present (no crash if missing)
    if (config["robot"]["min_velocity"]) {
        c.min_velocity = config["robot"]["min_velocity"].as<double>();
    }
    if (config["robot"]["max_velocity"]) {
        c.max_velocity = config["robot"]["max_velocity"].as<double>();
    }
    if (config["robot"]["min_turning_radius"]) {
        c.min_turning_radius = config["robot"]["min_turning_radius"].as<double>();
    }
    if (config["robot"]["max_acceleration"]) {
        c.max_acceleration = config["robot"]["max_acceleration"].as<double>();
    }

    // SDF path (required for Gazebo)
    c.sdf_path = config["gazebo_params"]["sdf_path"].as<std::string>();

    // Param maps (optional sections)
    auto load_map = [](YAML::Node node, std::map<std::string, std::string>& map) {
        if (!node) return;
        for (const auto& kv : node) {
            map[kv.first.as<std::string>()] = kv.second.as<std::string>();
        }
    };
    if (config["planner_params"]) load_map(config["planner_params"], c.planner_params_str);
    if (config["manager_params"]) load_map(config["manager_params"], c.manager_params_str);
    if (config["gazebo_params"]) load_map(config["gazebo_params"], c.gazebo_params_str);


    if (c.planner_params_str.count("num_pillar_nodes")) {
        c.num_pillar_nodes = std::stoi(c.planner_params_str.at("num_pillar_nodes"));
    }
    if (c.planner_params_str.count("goal_radius")) {
        c.goal_radius = std::stod(c.planner_params_str.at("goal_radius"));
    }
    if (c.planner_params_str.count("heuristic")) {
        std::string val = c.planner_params_str.at("heuristic");
        c.heuristic = (val == "true" || val == "1" || val == "True" || val == "yes");
    }

    return c;
}

void populateParams(Params& p, const std::map<std::string, std::string>& map) {
    for (const auto& kv : map) {
        try {
            if (kv.second == "true" || kv.second == "false") {
                p.setParam(kv.first, kv.second == "true");
            } 
            // V-- ADDED 'e' AND 'E' TO DETECT SCIENTIFIC NOTATION
            else if (kv.second.find('.') != std::string::npos || 
                     kv.second.find('e') != std::string::npos || 
                     kv.second.find('E') != std::string::npos) {
                p.setParam(kv.first, std::stod(kv.second));
            } else {
                p.setParam(kv.first, std::stoi(kv.second));
            }
        } catch (...) {
            p.setParam(kv.first, kv.second);
        }
    }
}

#if USE_METRIC
struct LogEntry {
    int row_id = 0;                  // monotonically increasing
    double elapsed_s = 0.0;          // wall-clock since global_start
    double sim_time = 0.0;           // cfg.time_budget - T_robot
    double time_to_goal = 0.0;       // T_robot

    std::string event_type;          // "set_state", "update", "plan", "slice_end", "goal_reached", ...
    
    double setup_ms = std::numeric_limits<double>::quiet_NaN();
    double update_ms = std::numeric_limits<double>::quiet_NaN();
    double plan_ms = std::numeric_limits<double>::quiet_NaN();
    double total_latency_ms = std::numeric_limits<double>::quiet_NaN();

    double path_cost = std::numeric_limits<double>::quiet_NaN();
    double obstacle_checks = std::numeric_limits<double>::quiet_NaN();
    double collision_count = std::numeric_limits<double>::quiet_NaN();
    

    double tree_size = std::numeric_limits<double>::quiet_NaN();
    double isolated_nodes = std::numeric_limits<double>::quiet_NaN();
    double avg_deg_out = std::numeric_limits<double>::quiet_NaN();
    double avg_deg_in = std::numeric_limits<double>::quiet_NaN();
    double neighborhood_radius = std::numeric_limits<double>::quiet_NaN();


    double setrobotstate_ms = std::numeric_limits<double>::quiet_NaN();
    // FOR REAL TIME!
    double decision_latency_ms = std::numeric_limits<double>::quiet_NaN(); // real compute that drove the sim step
    double applied_step_s      = std::numeric_limits<double>::quiet_NaN(); // actual simulated advance this iteration




    // executed solution quality — filled only on terminal rows
    bool   reached_goal = false;
    double exec_length  = std::numeric_limits<double>::quiet_NaN(); // L    [m]
    double exec_time    = std::numeric_limits<double>::quiet_NaN(); // T    [s]
    double exec_turn    = std::numeric_limits<double>::quiet_NaN(); // turn [rad]   (Dubins)
    double exec_effort  = std::numeric_limits<double>::quiet_NaN(); // int|a|dt     (Thruster)

};
#endif

std::atomic<bool> g_running{true};
void sigint_handler(int sig) { g_running = false; }




std::shared_ptr<StateSpace> createStateSpace(const ExperimentConfig& cfg) {
    if (cfg.state_space_type == "RDT") {
        return std::make_shared<RDTStateSpace>(cfg.dimensions, cfg.min_velocity,
                                                cfg.max_velocity, cfg.seed,
                                                cfg.is_geometric_mode());
    } else if (cfg.state_space_type == "Dubins") {
        return std::make_shared<DubinsTimeStateSpace>(cfg.min_turning_radius,
                                                       cfg.min_velocity,
                                                       cfg.max_velocity, cfg.seed);
    } else if (cfg.state_space_type == "Thruster") {
        return std::make_shared<ThrusterSteerStateSpace>(cfg.dimensions,
                                                          cfg.max_acceleration,
                                                          cfg.max_velocity, cfg.seed);
    }
    throw std::runtime_error("Unknown state space type");
}



void runRotatingTubeExperiment(const ExperimentConfig& cfg) {
    // Parse SDF and FORCE Artificial Kinematics
    auto obstacle_info = parseSdfObstacles(cfg.sdf_path);

    std::cout << "[AO Setup] Kinodynamic Rotation Mode: Converting static obstacles to spinning tubes.\n";
    for (auto& [name, info] : obstacle_info) {
        // Force the obstacles to be dynamic so generatePrediction works
        info.is_dynamic = true;
        
        // Give it an artificial speed. Higher speed = longer space-time tube.
        info.speed = 1.5; 
        
        // Infinite amplitude so the checker never triggers a turnaround event on its own.
        info.amplitude = 99999.0;
        
        // Start pointing along the positive X-axis
        info.direction = Eigen::Vector3d(1.0, 0.0, 0.0);
    }

    Params gazebo_params;
    populateParams(gazebo_params, cfg.gazebo_params_str);
    // Force Kinodynamic Mode
    gazebo_params.setParam("is_geometric_mode", false); 
    gazebo_params.setParam("initial_budget_time", cfg.time_budget);

    auto dummy_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    auto obstacle_checker = std::make_shared<DeterministicObstacleChecker>(
        dummy_clock, gazebo_params, obstacle_info);

    // Initialize at T=0
    obstacle_checker->processLatestPoseInfo(0.0);
    ObstacleVector environment_obs = obstacle_checker->getObstacles();

    std::cout << "Rotation Test: Loaded " << environment_obs.size() << " spinning obstacles\n";

    // Build start/goal/bounds exactly like main()
    const int dim = cfg.dimensions;

    if (cfg.dimensions < static_cast<int>(cfg.start_state.size())) {
        throw std::runtime_error("dimensions in YAML is smaller than start_state size");
    }
    if (cfg.start_state.size() != cfg.goal_state.size() ||
        cfg.start_state.size() != cfg.bounds_min.size() ||
        cfg.start_state.size() != cfg.bounds_max.size()) {
        throw std::runtime_error("start/goal/bounds size mismatch in YAML");
    }

    Eigen::VectorXd start_spatial(cfg.start_state.size());
    Eigen::VectorXd goal_spatial(cfg.goal_state.size());
    Eigen::VectorXd lower_spatial(cfg.bounds_min.size());
    Eigen::VectorXd upper_spatial(cfg.bounds_max.size());

    for (size_t i = 0; i < cfg.start_state.size(); ++i) start_spatial(i) = cfg.start_state[i];
    for (size_t i = 0; i < cfg.goal_state.size();  ++i) goal_spatial(i)  = cfg.goal_state[i];
    for (size_t i = 0; i < cfg.bounds_min.size();  ++i) lower_spatial(i) = cfg.bounds_min[i];
    for (size_t i = 0; i < cfg.bounds_max.size();  ++i) upper_spatial(i) = cfg.bounds_max[i];

    Eigen::VectorXd start_vec = start_spatial;
    Eigen::VectorXd goal_vec  = goal_spatial;
    Eigen::VectorXd lower_vec = lower_spatial;
    Eigen::VectorXd upper_vec = upper_spatial;

    const bool needs_time_dimension = (cfg.dimensions > static_cast<int>(cfg.start_state.size()));
    const int time_idx = cfg.dimensions - 1;
    if (needs_time_dimension) {
        start_vec.conservativeResize(cfg.dimensions);
        goal_vec.conservativeResize(cfg.dimensions);
        lower_vec.conservativeResize(cfg.dimensions);
        upper_vec.conservativeResize(cfg.dimensions);

        start_vec(time_idx) = cfg.time_extent();
        goal_vec(time_idx)  = 0.0;
        lower_vec(time_idx) = 0.0;
        upper_vec(time_idx) = cfg.time_extent();
    }

    // Visualization Setup
    const bool visualize = true;
    rclcpp::Node::SharedPtr vis_node;
    std::shared_ptr<RVizVisualization> visualization;

    if (visualize) {
        vis_node = std::make_shared<rclcpp::Node>(
            cfg.name + "_rotation_visualizer",
            rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));

        visualization = std::make_shared<RVizVisualization>(vis_node);
        visualization->clearMarkers("");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        Eigen::VectorXd axes_lower(2), axes_upper(2);
        axes_lower << lower_spatial(0), lower_spatial(1);
        axes_upper << upper_spatial(0), upper_spatial(1);
        visualization->visualizeAxes(axes_lower, axes_upper, 5.0, "map");

        std::vector<Eigen::VectorXd> safe_cyl_pos, threat_cyl_pos;
        std::vector<double> safe_cyl_radii, threat_cyl_radii;
        std::vector<std::tuple<Eigen::Vector2d, double, double, double>> safe_boxes, threat_boxes;
        std::vector<Eigen::Vector2d> safe_vel_pos, safe_vel_val;
        std::vector<Eigen::Vector2d> threat_vel_pos, threat_vel_val;

        for (const auto& ob : environment_obs) {
            if (ob.type == Obstacle::CIRCLE) {
                Eigen::VectorXd pos(2);
                pos << ob.position.x(), ob.position.y();
                safe_cyl_pos.push_back(pos);
                safe_cyl_radii.push_back(ob.dimensions.radius);
            } else if (ob.type == Obstacle::BOX) {
                safe_boxes.emplace_back(
                    ob.position,
                    ob.dimensions.width,
                    ob.dimensions.height,
                    ob.dimensions.rotation
                );
            }
        }

        Eigen::Vector3d robot_pos(start_vec(0), start_vec(1), 0.0); // Safe because start_vec has >= 2 elements
        Eigen::VectorXd orientation_quat(4);
        orientation_quat << 0, 0, 0, 1;
        std::vector<float> robot_color = {0.0f, 0.0f, 1.0f};
        std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> empty_trace;

        visualization->publishObstacleFrame(
            safe_cyl_pos, safe_cyl_radii,
            threat_cyl_pos, threat_cyl_radii,
            safe_boxes, threat_boxes,
            safe_vel_pos, safe_vel_val,
            threat_vel_pos, threat_vel_val,
            empty_trace,
            robot_pos,
            orientation_quat,
            robot_color,
            0,
            "",
            {},
            {},
            "map"
        );
        visualization->triggerPublish();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Setup Planner
    auto statespace = createStateSpace(cfg);
    auto problem_def = std::make_shared<ProblemDefinition>(dim);
    problem_def->setStart(goal_vec);   // Backward search
    problem_def->setGoal(start_vec);
    problem_def->setBounds(lower_vec, upper_vec);

    auto planner = std::make_shared<KinodynamicANYFMTX>(statespace, problem_def, obstacle_checker);

    Params planner_params;
    populateParams(planner_params, cfg.planner_params_str);
    planner_params.setParam("is_geometric_mode", false);
    planner_params.setParam("partial_update", false);
    planner_params.setParam("use_knn", false);

    planner->setup(planner_params, visualization);


    // auto warmup_duration = std::chrono::milliseconds(500); 
    // auto warmup_start = std::chrono::steady_clock::now();
    // while (std::chrono::steady_clock::now() - warmup_start < warmup_duration) {
    //     planner->plan(); 
    // }

    planner->setRobotState(start_vec, false);

    // Initial Empty/Clear step
    obstacle_checker->clearActiveObstacles();

    // Main Rotation Loop
    auto time_limit = std::chrono::seconds(cfg.duration_limit);
    auto start_time = std::chrono::steady_clock::now();

    // 10 degrees per tick
    double current_angle = 0.0;
    const double angle_step = 10.0 * (M_PI / 180.0); 

    std::ofstream csv_log("revisits_rotation_test.csv");
    csv_log << "angle_deg,tree_size,path_cost,revisits\n";

    while (g_running && rclcpp::ok()) {
        auto now = std::chrono::steady_clock::now();
        if (now - start_time > time_limit) {
            std::cout << "[INFO] Time limit reached." << std::endl;
            break;
        }

        // Artificially Spin the velocity vector!
        std::vector<Eigen::Vector2d> obs_positions;
        std::vector<Eigen::Vector2d> obs_velocities;

        int obs_index = 0; // Keep track of which obstacle we are on
        for (auto& ob : environment_obs) {
            // Determine direction: 1.0 for CCW, -1.0 for CW
            double direction_multiplier = (obs_index % 2 == 0) ? 1.0 : -1.0;
            
            // Apply the multiplier to the current_angle for this specific obstacle
            double specific_angle = current_angle * direction_multiplier;

            // Calculate new heading based on the specific angle
            double vx = std::cos(specific_angle) * ob.speed_scalar;
            double vy = std::sin(specific_angle) * ob.speed_scalar;
            ob.velocity = Eigen::Vector2d(vx, vy);
            
            double scale = 2.0;
            // Save for visualization
            obs_positions.push_back(ob.position);
            obs_velocities.push_back(Eigen::Vector2d(scale*vx, scale*vy));
            
            obs_index++; // Increment the index for the next obstacle
        }

        // Trigger FMTX to Sever and Repair the Graph
        // This will call removeObstacle(old_tube) and addNewObstacle(new_tube)
        planner->updateObstacles(environment_obs);

        // Let FMTX add new samples and heal the graph
        planner->plan();

        // Log the metrics
        const auto& metrics = planner->getLastReplanMetrics();
        csv_log << (current_angle * 180.0 / M_PI) << "," 
                << planner->getTreeSize() << "," 
                << metrics.path_cost << "," 
                << metrics.revisits << "\n";

        // Visualize Tree, Tubes, and Velocity Vectors
        if (visualize) {
            planner->visualizeTree();
            visualization->visualizeSweptTubes(environment_obs, start_vec(time_idx), "map");
            visualization->visualizeVelocityVectors(obs_positions, obs_velocities, "map", {0.0f, 1.0f, 0.0f}, "velocities");
            visualization->triggerPublish();
            
            // Slow down slightly so you can see it spinning like a radar in RViz!
            // std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
        }

        // Step Angle
        current_angle += angle_step;
        if (current_angle >= 2.0 * M_PI) {
            current_angle -= 2.0 * M_PI;
        }
    }

    csv_log.close();
    std::cout << "TREE SIZE AT THE END: " << planner->getTreeSize() << "\n";
}


void runAOExperiment(const ExperimentConfig& cfg) {
    // Parse SDF and FORCE a frozen static snapshot BEFORE checker construction
    auto obstacle_info = parseSdfObstacles(cfg.sdf_path);

    for (auto& [name, info] : obstacle_info) {
        info.is_dynamic = false;
        info.speed = 0.0;
        info.amplitude = 0.0;
        info.direction.setZero();
    }

    Params gazebo_params;
    populateParams(gazebo_params, cfg.gazebo_params_str);
    gazebo_params.setParam("is_geometric_mode", cfg.is_geometric_mode());
    gazebo_params.setParam("initial_budget_time", cfg.is_geometric_mode() ? 0.0 : cfg.time_budget);

    auto dummy_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    auto obstacle_checker = std::make_shared<DeterministicObstacleChecker>(
        dummy_clock, gazebo_params, obstacle_info);

    // no initializeDynamicObstacles()
    obstacle_checker->processLatestPoseInfo(0.0);
    ObstacleVector static_obs = obstacle_checker->getObstacles();

    std::cout << "AO Test: Loaded " << static_obs.size() << " frozen obstacles\n";
    for (const auto& ob : static_obs) {
        std::cout << "  " << ob.name << " at ("
                  << ob.position.x() << ", " << ob.position.y() << ")\n";
    }

    // Build start/goal/bounds exactly like main()
    const int dim = cfg.dimensions;

    if (cfg.dimensions < static_cast<int>(cfg.start_state.size())) {
        throw std::runtime_error("dimensions in YAML is smaller than start_state size");
    }
    if (cfg.start_state.size() != cfg.goal_state.size() ||
        cfg.start_state.size() != cfg.bounds_min.size() ||
        cfg.start_state.size() != cfg.bounds_max.size()) {
        throw std::runtime_error("start/goal/bounds size mismatch in YAML");
    }

    Eigen::VectorXd start_spatial(cfg.start_state.size());
    Eigen::VectorXd goal_spatial(cfg.goal_state.size());
    Eigen::VectorXd lower_spatial(cfg.bounds_min.size());
    Eigen::VectorXd upper_spatial(cfg.bounds_max.size());

    for (size_t i = 0; i < cfg.start_state.size(); ++i) start_spatial(i) = cfg.start_state[i];
    for (size_t i = 0; i < cfg.goal_state.size();  ++i) goal_spatial(i)  = cfg.goal_state[i];
    for (size_t i = 0; i < cfg.bounds_min.size();  ++i) lower_spatial(i) = cfg.bounds_min[i];
    for (size_t i = 0; i < cfg.bounds_max.size();  ++i) upper_spatial(i) = cfg.bounds_max[i];

    Eigen::VectorXd start_vec = start_spatial;
    Eigen::VectorXd goal_vec  = goal_spatial;
    Eigen::VectorXd lower_vec = lower_spatial;
    Eigen::VectorXd upper_vec = upper_spatial;

    const bool needs_time_dimension = (cfg.dimensions > static_cast<int>(cfg.start_state.size()));
    if (needs_time_dimension) {
        const int time_idx = cfg.dimensions - 1;

        start_vec.conservativeResize(cfg.dimensions);
        goal_vec.conservativeResize(cfg.dimensions);
        lower_vec.conservativeResize(cfg.dimensions);
        upper_vec.conservativeResize(cfg.dimensions);

        start_vec(time_idx) = cfg.time_extent();
        goal_vec(time_idx)  = 0.0;
        lower_vec(time_idx) = 0.0;
        upper_vec(time_idx) = cfg.time_extent();
    }

    // Visualization: do NOT call rclcpp::init() here
    const bool visualize = true;
    rclcpp::Node::SharedPtr vis_node;
    std::shared_ptr<RVizVisualization> visualization;

    if (visualize) {
        vis_node = std::make_shared<rclcpp::Node>(
            cfg.name + "_ao_visualizer",
            rclcpp::NodeOptions().parameter_overrides(
                {rclcpp::Parameter("use_sim_time", true)}));

        visualization = std::make_shared<RVizVisualization>(vis_node);

        visualization->clearMarkers("");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        Eigen::VectorXd axes_lower(2), axes_upper(2);
        axes_lower << lower_spatial(0), lower_spatial(1);
        axes_upper << upper_spatial(0), upper_spatial(1);
        visualization->visualizeAxes(axes_lower, axes_upper, 5.0, "map");

        std::vector<Eigen::VectorXd> safe_cyl_pos, threat_cyl_pos;
        std::vector<double> safe_cyl_radii, threat_cyl_radii;
        std::vector<std::tuple<Eigen::Vector2d, double, double, double>> safe_boxes, threat_boxes;
        std::vector<Eigen::Vector2d> safe_vel_pos, safe_vel_val;
        std::vector<Eigen::Vector2d> threat_vel_pos, threat_vel_val;

        for (const auto& ob : static_obs) {
            if (ob.type == Obstacle::CIRCLE) {
                Eigen::VectorXd pos(2);
                pos << ob.position.x(), ob.position.y();
                safe_cyl_pos.push_back(pos);
                safe_cyl_radii.push_back(ob.dimensions.radius);
            } else if (ob.type == Obstacle::BOX) {
                safe_boxes.emplace_back(
                    ob.position,
                    ob.dimensions.width,
                    ob.dimensions.height,
                    ob.dimensions.rotation
                );
            }
        }

        Eigen::Vector3d robot_pos(start_vec(0), start_vec(1), 0.0);
        Eigen::VectorXd orientation_quat(4);
        orientation_quat << 0, 0, 0, 1;
        std::vector<float> robot_color = {0.0f, 0.0f, 1.0f};
        std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> empty_trace;
        visualization->publishObstacleFrame(
            safe_cyl_pos, safe_cyl_radii,
            threat_cyl_pos, threat_cyl_radii,
            safe_boxes, threat_boxes,
            safe_vel_pos, safe_vel_val,
            threat_vel_pos, threat_vel_val,
            empty_trace,
            robot_pos,
            orientation_quat,
            robot_color,
            0,
            "",
            {},
            {},
            "map"
        );
        visualization->triggerPublish();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Main AO loop - fixed snapshot, varying n only

    auto statespace = createStateSpace(cfg);
    auto problem_def = std::make_shared<ProblemDefinition>(dim);
    problem_def->setStart(goal_vec);   // Backward search
    problem_def->setGoal(start_vec);
    problem_def->setBounds(lower_vec, upper_vec);

    auto planner = std::make_shared<KinodynamicANYFMTX>(statespace, problem_def, obstacle_checker);

    Params planner_params;
    populateParams(planner_params, cfg.planner_params_str);
    planner_params.setParam("is_geometric_mode", cfg.is_geometric_mode());
    planner_params.setParam("partial_update", false);
    planner_params.setParam("use_knn", false);


    planner->setup(planner_params, visualization);

    obstacle_checker->clearActiveObstacles();
    // planner->addStaticObstacles(static_obs);
    // planner->plan();
    planner->setRobotState(start_vec, false);
    obstacle_checker->processLatestPoseInfo(0.0);
    auto time_limit = std::chrono::seconds(cfg.duration_limit);
    auto start_time = std::chrono::steady_clock::now();
    while (g_running && rclcpp::ok()) {
        if (std::chrono::steady_clock::now() - start_time > time_limit) {
            std::cout << "[INFO] Time limit reached." << std::endl;
            break;
        }
        planner->addStaticObstacles(static_obs);
        planner->plan();
        if (visualize) {
            planner->visualizeTree();
            visualization->triggerPublish();
            // std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        planner->removeStaticObstacles(static_obs);
    }
    std::cout<<"TREE SIZE AT THE END: "<<planner->getTreeSize()<<"\n";

}








int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, sigint_handler);
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config_file.yaml>" << std::endl;
        return 1;
    }
    std::string config_path = argv[1];
    std::cout << "[CONFIG] Loading: " << config_path << std::endl;
    ExperimentConfig cfg = loadConfig(config_path);


    if (cfg.name == "AO_test") {
        runAOExperiment(cfg);
        return 0;
    }
    if (cfg.name == "RTube_test") {
        runRotatingTubeExperiment(cfg);
        return 0;
    }



    // --- 1. Parameters ---
    Params manager_params, gazebo_params, planner_params;
    populateParams(manager_params, cfg.manager_params_str);
    populateParams(gazebo_params, cfg.gazebo_params_str);
    populateParams(planner_params, cfg.planner_params_str);

    gazebo_params.setParam("initial_budget_time", cfg.time_budget);
    bool is_geometric_mode = cfg.is_geometric_mode();
    gazebo_params.setParam("is_geometric_mode", is_geometric_mode);
    planner_params.setParam("is_geometric_mode", is_geometric_mode);
    double infl = cfg.get_inflation();
    gazebo_params.setParam("inflation", infl);
    manager_params.setParam("inflation", infl);
    manager_params.setParam("slice_time", cfg.slice_time);

    bool should_vis = (cfg.vis_frequency_hz > 0);

    // --- 2. ROS & Vis ---
    auto vis_node = std::make_shared<rclcpp::Node>(cfg.name + "_visualizer",
        rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
    auto visualization = std::make_shared<RVizVisualization>(vis_node);
    auto sim_clock = vis_node->get_clock();

    // --- 3. Obstacles ---
    auto obstacle_info = parseSdfObstacles(cfg.sdf_path);
    auto obstacle_checker = std::make_shared<DeterministicObstacleChecker>(sim_clock, gazebo_params, obstacle_info);

    // --- 4. Problem Definition ---
    auto problem_def = std::make_shared<ProblemDefinition>(cfg.dimensions);

    Eigen::VectorXd start_vec(cfg.dimensions);
    Eigen::VectorXd goal_vec(cfg.dimensions);
    Eigen::VectorXd lower_vec(cfg.dimensions);
    Eigen::VectorXd upper_vec(cfg.dimensions);

    // These vectors now have size = number of spatial dimensions only
    Eigen::VectorXd start_spatial(cfg.start_state.size());
    Eigen::VectorXd goal_spatial (cfg.goal_state.size());
    Eigen::VectorXd lower_spatial(cfg.bounds_min.size());
    Eigen::VectorXd upper_spatial(cfg.bounds_max.size());

    for (size_t i = 0; i < cfg.start_state.size(); ++i)
        start_spatial(i) = cfg.start_state[i];
    for (size_t i = 0; i < cfg.goal_state.size(); ++i)
        goal_spatial(i)  = cfg.goal_state[i];
    for (size_t i = 0; i < cfg.bounds_min.size(); ++i)
        lower_spatial(i) = cfg.bounds_min[i];
    for (size_t i = 0; i < cfg.bounds_max.size(); ++i)
        upper_spatial(i) = cfg.bounds_max[i];

    // ── 2. Decide if we need to add a time dimension ──
    bool needs_time_dimension = (cfg.dimensions > static_cast<int>(cfg.start_state.size()));

    // Safety check (optional but very useful during development)
    if (cfg.dimensions < static_cast<int>(cfg.start_state.size())) {
        throw std::runtime_error("dimensions in YAML is smaller than start_state size!");
    }
    if (cfg.start_state.size() != cfg.goal_state.size() ||
        cfg.start_state.size() != cfg.bounds_min.size() ||
        cfg.start_state.size() != cfg.bounds_max.size()) {
        throw std::runtime_error("start/goal/bounds size mismatch in YAML!");
    }

    // ── 3. Build final vectors ──
    start_vec = start_spatial;
    goal_vec  = goal_spatial;
    lower_vec = lower_spatial;
    upper_vec = upper_spatial;

    if (needs_time_dimension) {
        // Append time slot — single source of truth
        int time_idx = cfg.dimensions - 1;

        start_vec.conservativeResize(cfg.dimensions);
        goal_vec.conservativeResize(cfg.dimensions);
        lower_vec.conservativeResize(cfg.dimensions);
        upper_vec.conservativeResize(cfg.dimensions);

        start_vec(time_idx) = cfg.time_extent();   // 0.0 or time_budget
        goal_vec(time_idx)  = 0.0;                 // goal time is always 0
        lower_vec(time_idx) = 0.0;
        upper_vec(time_idx) = cfg.time_extent();   // time budget is max time
    }
    
    // Planner Setup: Start=Goal, Goal=Start (Backward Search)
    problem_def->setStart(goal_vec);  
    problem_def->setGoal(start_vec);  
    problem_def->setBounds(lower_vec, upper_vec);

    // --- 5. State Space Factory ---
    std::shared_ptr<StateSpace> statespace;
    if (cfg.state_space_type == "RDT") {
        // Pass the boolean flag directly to the constructor
        statespace = std::make_shared<RDTStateSpace>(cfg.dimensions, cfg.min_velocity, cfg.max_velocity, cfg.seed, is_geometric_mode);
    } else if (cfg.state_space_type == "Dubins") {
        statespace = std::make_shared<DubinsTimeStateSpace>(cfg.min_turning_radius, cfg.min_velocity, cfg.max_velocity, cfg.seed);
    } else if (cfg.state_space_type == "Thruster") {
        statespace = std::make_shared<ThrusterSteerStateSpace>(cfg.dimensions, cfg.max_acceleration, cfg.max_velocity, cfg.seed);
    } else {
        std::cerr << "Unknown State Space: " << cfg.state_space_type << std::endl;
        return 1;
    }

    // --- 6. Manager Factory ---
    std::shared_ptr<ROS2ManagerBase> ros_manager;
    if (is_geometric_mode) {
        ros_manager = nullptr;
        std::cout << "[MODE] Geometric (R2) detected. Disabling ROS2 Manager." << std::endl;
    } else {
        if (cfg.manager_type == "R2T") {
            ros_manager = std::make_shared<R2TROS2Manager>(obstacle_checker, visualization, manager_params, start_vec, cfg.time_budget);
        } else if (cfg.manager_type == "Dubins") {
            ros_manager = std::make_shared<DubinsROS2Manager>(obstacle_checker, visualization, manager_params, start_vec, cfg.time_budget);
        } else if (cfg.manager_type == "Thruster") {
            ros_manager = std::make_shared<ThrusterROS2Manager>(obstacle_checker, visualization, manager_params, start_vec, cfg.time_budget);
        } else {
            std::cerr << "Unknown Manager: " << cfg.manager_type << std::endl;
            return 1;
        }
    }

    // ros_manager = std::dynamic_pointer_cast<R2TROS2Manager>(ros_manager); 
    // --- 7. Planner Factory ---
    PlannerType p_type = PlannerType::KinodynamicFMTX; 
    bool is_anytime = false;
    if (cfg.planner_type == "KinodynamicFMTX") p_type = PlannerType::KinodynamicFMTX;
    else if (cfg.planner_type == "KinodynamicRRTX") p_type = PlannerType::KinodynamicRRTX;
    else if (cfg.planner_type == "KinodynamicANYFMTX") { p_type = PlannerType::KinodynamicANYFMTX; is_anytime = true; }
    else if (cfg.planner_type == "KinodynamicANYRRTX") { p_type = PlannerType::KinodynamicANYRRTX; is_anytime = true; }
    else if (cfg.planner_type == "KinodynamicPRMStarDStarLite") p_type = PlannerType::KinodynamicPRMStarDStarLite;
    else if (cfg.planner_type == "KinodynamicLLPTStar") { p_type = PlannerType::KinodynamicLLPTStar; is_anytime = true;}
    else {
        std::cerr << "Unknown Planner: " << cfg.planner_type << std::endl;
        return 1;
    }
    auto planner = PlannerFactory::getInstance().createPlanner(p_type, statespace, problem_def, obstacle_checker);

    auto setup_start = std::chrono::steady_clock::now();
    planner->setup(planner_params, visualization);
    auto setup_end = std::chrono::steady_clock::now();
    double setup_time_ms = std::chrono::duration<double, std::milli>(setup_end - setup_start).count();    


    auto kinodynamic_planner = std::dynamic_pointer_cast<Planner>(planner); 
    // kinodynamic_planner = std::dynamic_pointer_cast<KinodynamicFMTX>(planner); 
    // if (kinodynamic_planner) kinodynamic_planner->setClock(sim_clock);

    kinodynamic_planner->setCurrentRobotTime(cfg.time_budget);

    // just in case heuristic is on!
    KinodynamicPRMStarDStarLite* dlite = nullptr;
    KinodynamicLLPTStar* llpt = nullptr;
    if (cfg.heuristic) {
        dlite = dynamic_cast<KinodynamicPRMStarDStarLite*>(kinodynamic_planner.get());
        llpt = dynamic_cast<KinodynamicLLPTStar*>(kinodynamic_planner.get());
    }

  


    // --- 8. Initial Plan ---
    RCLCPP_INFO(vis_node->get_logger(), "Running initial plan...");
    auto start_t = std::chrono::steady_clock::now();
    
    bool fixed_sample = true; // Always use deterministic loops for scientific runs or else the intial tree size would be different for the time based preplan in each run even with the same seed!
    if (is_anytime) {
        if(!fixed_sample){
            auto warmup_duration = std::chrono::milliseconds(500); 
            auto warmup_start = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - warmup_start < warmup_duration) {
                planner->plan(); 
            }
        }
        else{
            const int TARGET_SAMPLES = 500;
            while (kinodynamic_planner->getTreeSize() < TARGET_SAMPLES) {
                planner->plan();
            }
        }
    } else {
        planner->plan();
    }

    std::cout<<"INITIAL TREE SIZE: "<<planner->getTreeSize()<<"\n";
    
    auto end_t = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t);
    std::cout << "Initial plan took: " << duration.count() << " ms" << std::endl;
    
    // --- 8.5 Seed Obstacles & Cut Graph ---
    // Now that the free-space graph exists, we drop the Day 0 obstacles onto it.
    auto gazebo_checker = std::dynamic_pointer_cast<DeterministicObstacleChecker>(obstacle_checker);
    if (gazebo_checker) {
        double initial_T = is_geometric_mode ? 0.0 : start_vec(start_vec.size()-1);
        
        Eigen::Vector2d robot_pos = start_vec.head<2>(); // (x,y)
        gazebo_checker->processLatestPoseInfo(0.0); 
        ObstacleVector initial_obs = gazebo_checker->checkAndRepairObstacles(initial_T, robot_pos);
        
        // This physically severs edges and triggers D* Lite to recompute the shortest path
        if (!initial_obs.empty() && kinodynamic_planner) {
            kinodynamic_planner->updateObstacles(initial_obs);
            RCLCPP_INFO(vis_node->get_logger(), "Day 0 Obstacles Seeded. Graph edges cut and repaired!");
        }
    }
    
    kinodynamic_planner->setRobotState(start_vec, false); 
    
    auto path = kinodynamic_planner->getPathPositions();
    if (!path.empty() && ros_manager) {
        ros_manager->setPath(path);
    }

    // if (cfg.planner_type == "KinodynamicRRTX"){
    //     auto rrtx_planner = dynamic_cast<KinodynamicRRTX*>(planner.get());
    //     rrtx_planner->dumpTreeToCSV("rrtx_tree_nodes.csv");
    // }
    // if (cfg.planner_type == "KinodynamicANYRRTX"){
    //     auto anyrrtx_planner = dynamic_cast<KinodynamicANYRRTX*>(planner.get());
    //     anyrrtx_planner->dumpTreeToCSV("anyrrtx_tree_nodes.csv");
    // }
    // if (cfg.planner_type == "KinodynamicANYFMTX"){
    //     auto fmtx_planner = dynamic_cast<KinodynamicANYFMTX*>(planner.get());
    //     fmtx_planner->dumpTreeToCSV("anyfmtx_tree_nodes.csv");
    // }

#if USE_METRIC
    // STORE THE INITIAL PLAN METRICS FOR THE CSV
    std::vector<LogEntry> log_data; 
    int next_row_id = 1; // GLOBAL ROW COUNTER
    
    LogEntry init_entry;
    const auto& init_metrics = kinodynamic_planner->getLastReplanMetrics();
    
    init_entry.row_id = next_row_id++;
    init_entry.event_type = "initial_plan";
    init_entry.elapsed_s = 0.0; 
    init_entry.sim_time = 0.0;
    init_entry.time_to_goal = is_geometric_mode ? 0.0 : start_vec(start_vec.size() - 1);
    
    init_entry.setup_ms = setup_time_ms;
    init_entry.plan_ms = duration.count(); 
    init_entry.total_latency_ms = duration.count();
    
    init_entry.path_cost = init_metrics.path_cost;
    init_entry.obstacle_checks = init_metrics.obstacle_checks;
    init_entry.tree_size = kinodynamic_planner->getTreeSize();
    init_entry.isolated_nodes = kinodynamic_planner->getIsolatedNodeCount(); 
    init_entry.avg_deg_out = kinodynamic_planner->getAvgOutDegree();
    init_entry.avg_deg_in = kinodynamic_planner->getAvgInDegree();
    init_entry.neighborhood_radius = kinodynamic_planner->getNeighborhoodRadius();
    
    log_data.push_back(init_entry);



    // SAVE GRAPH
    std::ofstream graph_log_file;
    int graph_cycle_count = 0;
    
    bool should_log_graph = (!is_geometric_mode && !is_anytime);
    should_log_graph = false; // SOHEIL: Dont need it for now!

    if (should_log_graph) {
        std::time_t now_time = std::time(nullptr);
        std::tm* local_tm = std::localtime(&now_time);
        char time_buf[80];
        strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", local_tm);
        
        std::string graph_filename = "graph_" + cfg.name + "_" + time_buf + ".csv";
        graph_log_file.open(graph_filename);
        
        if (graph_log_file.is_open()) {
            graph_log_file << "cycle_id,node_id,x,y,g,lmc,parent_id,is_robot_anchor,robot_anchor_id,robot_g,robot_lmc,bridge_cost,robot_total_cost,robot_time_to_goal\n";
            
            kinodynamic_planner->logGraphState(graph_log_file, graph_cycle_count);
            graph_cycle_count++;
            
            std::cout << "[INFO] Graph logging enabled. Initial graph saved to: " << graph_filename << std::endl;
        } else {
            std::cerr << "[ERROR] Could not open graph log file!" << std::endl;
        }
    }

#endif

    // --- 9. Executor Setup ---
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(vis_node);
    if (ros_manager) {
        executor.add_node(ros_manager);
    }

    // --- DRAW RVIZ AXES ONCE ---
    // Extract only the spatial (X, Y) bounds to draw the 2D grid numbers
    Eigen::VectorXd axes_lower(2);
    axes_lower << lower_spatial(0), lower_spatial(1);
    
    Eigen::VectorXd axes_upper(2);
    axes_upper << upper_spatial(0), upper_spatial(1);

    visualization->clearMarkers(""); // Empty string means "Delete literally everything"
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Give RViz 100ms to process the delete command

    // Call it exactly once before the simulation loop begins.
    // 5.0 places a number marker every 5 meters.
    visualization->visualizeAxes(axes_lower, axes_upper, 5.0, "map");




#if SCREEN_SHOT
    RCLCPP_INFO(vis_node->get_logger(), "Simulation starting in 3 seconds...");
    RCLCPP_INFO(vis_node->get_logger(), "--> CLICK ON THE RVIZ WINDOW NOW TO MAKE IT ACTIVE <--");
    
    // Give yourself 3 seconds to click the RViz window
    for (int countdown = 3; countdown > 0; --countdown) {
        std::cout << countdown << "..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "GO!" << std::endl;
    bool captured_start = false;
    double next_screenshot_time = cfg.time_budget - 5.0; // Schedule first 
#endif



    // auto gazebo_checker = std::dynamic_pointer_cast<DeterministicObstacleChecker>(obstacle_checker);
    // if (gazebo_checker) {
    //     // Initialize with T=0 for geometric, or time_budget for kinodynamic
    //     double initial_T = is_geometric_mode ? 0.0 : start_vec(start_vec.size()-1);
    //     gazebo_checker->processLatestPoseInfo(0); // Sync physics at T=0
    //     gazebo_checker->initializeDynamicObstacles(initial_T);
    // }

    auto global_start = std::chrono::steady_clock::now();
    auto time_limit = std::chrono::seconds(cfg.duration_limit);
    auto start_time = std::chrono::steady_clock::now();
    
    CALLGRIND_START_INSTRUMENTATION;
    // BRANCHING LOGIC
    if (is_geometric_mode) {
        // --- GEOMETRIC MODE (R2) ---
        RCLCPP_INFO(vis_node->get_logger(), "Starting Geometric Planning Loop (R2)...");

        double pending_anchor_repair_ms = 0.0; // Specific to LLPT
        // const auto loop_duration = std::chrono::duration<double>(slice_dt);
        const double slice_dt   = cfg.slice_time;
        const int    num_slices = static_cast<int>(std::lround(cfg.duration_limit / slice_dt));
        double sim_time = 0.0;

        std::cout<<"NUM_SLICES: "<<num_slices<<"\n";
        // while (g_running && rclcpp::ok()) {
        for (int slice = 0; slice < num_slices; ++slice) {

            auto loop_start_time = std::chrono::steady_clock::now();
            executor.spin_some();

            if (cfg.heuristic) { 
                if(cfg.planner_type == "KinodynamicPRMStarDStarLite"){
                    double repair = dlite->getLastAnchorRepairMs();
                    pending_anchor_repair_ms += repair;
                    dlite->resetLastAnchorRepairMs();
                }
                else if(cfg.planner_type == "KinodynamicLLPTStar"){
                    double repair = llpt->getLastAnchorRepairMs();
                    pending_anchor_repair_ms += repair;
                    llpt->resetLastAnchorRepairMs();

                }
            }

            gazebo_checker->processLatestPoseInfo(sim_time);
            ObstacleVector all_obs = gazebo_checker->getObstacles();
            for (auto& ob : all_obs){
                ob.predicted_path = gazebo_checker->generatePrediction(ob, 0);
            }

            // 1. UPDATE EVENT
            kinodynamic_planner->resetMetrics();
            auto t_update_1 = std::chrono::steady_clock::now();
            kinodynamic_planner->updateObstacles(all_obs);
            auto t_update_2 = std::chrono::steady_clock::now();
            double update_time_ms = std::chrono::duration<double, std::milli>(t_update_2 - t_update_1).count();

            if (cfg.heuristic){ //Specific to DLITE
                update_time_ms += pending_anchor_repair_ms;
                // std::cout << "PENDING ANCHOR REPAIR: " << std::fixed << std::setprecision(6) << pending_anchor_repair_ms << "\n";
                pending_anchor_repair_ms = 0.0;
            }

            // if (update_time_ms > 1.0) {
                RCLCPP_INFO( rclcpp::get_logger("Planner_Timing"), "updateObstacles took: %.2f ms", update_time_ms);
            // }

#if USE_METRIC
            {
                LogEntry update_entry;
                const auto& update_metrics = kinodynamic_planner->getLastReplanMetrics();

                update_entry.row_id = next_row_id++;
                update_entry.event_type = "update";
                update_entry.elapsed_s =
                    std::chrono::duration<double>(t_update_2 - global_start).count();
                update_entry.sim_time = sim_time;

                update_entry.update_ms = update_time_ms;
                update_entry.obstacle_checks = update_metrics.obstacle_checks;
                update_entry.tree_size = kinodynamic_planner->getTreeSize();
                update_entry.isolated_nodes = kinodynamic_planner->getIsolatedNodeCount();

                log_data.push_back(update_entry);
            }
#endif

#if USE_METRIC
            if (should_log_graph && graph_log_file.is_open()) {
                kinodynamic_planner->logGraphState(graph_log_file, graph_cycle_count);
                graph_cycle_count++;
            }
#endif

            // 2. PLAN EVENT
            if (is_anytime) {
                kinodynamic_planner->resetMetrics();
                auto t_plan_1 = std::chrono::steady_clock::now();
                kinodynamic_planner->plan();
                auto t_plan_2 = std::chrono::steady_clock::now();
                double plan_time_ms = std::chrono::duration<double, std::milli>(t_plan_2 - t_plan_1).count();

                RCLCPP_INFO( rclcpp::get_logger("Planner_Timing"), "plan took: %.2f ms", plan_time_ms);

#if USE_METRIC
                {
                    LogEntry plan_entry;
                    const auto& plan_metrics = kinodynamic_planner->getLastReplanMetrics();

                    plan_entry.row_id = next_row_id++;
                    plan_entry.event_type = "plan";
                    plan_entry.elapsed_s = std::chrono::duration<double>(t_plan_2 - global_start).count();
                    plan_entry.sim_time = sim_time;

                    plan_entry.plan_ms = plan_time_ms;
                    plan_entry.obstacle_checks = plan_metrics.obstacle_checks;
                    plan_entry.tree_size = kinodynamic_planner->getTreeSize();
                    plan_entry.isolated_nodes = kinodynamic_planner->getIsolatedNodeCount();

                    log_data.push_back(plan_entry);
                }
#endif
            }

            // --- VISUALIZATION ---
            std::vector<Eigen::VectorXd> safe_cyl_pos, threat_cyl_pos;
            std::vector<double> safe_cyl_radii, threat_cyl_radii;
            std::vector<std::tuple<Eigen::Vector2d, double, double, double>> safe_boxes, threat_boxes;
            std::vector<Eigen::Vector2d> safe_vel_pos, safe_vel_val;
            std::vector<Eigen::Vector2d> threat_vel_pos, threat_vel_val;

            for (const auto& obstacle : all_obs) {
                bool is_threat = false;
                if (obstacle.type == Obstacle::CIRCLE) {
                    Eigen::VectorXd pos(2);
                    pos << obstacle.position.x(), obstacle.position.y();
                    if (is_threat) {
                        threat_cyl_pos.push_back(pos);
                        threat_cyl_radii.push_back(obstacle.dimensions.radius);
                    } else {
                        safe_cyl_pos.push_back(pos);
                        safe_cyl_radii.push_back(obstacle.dimensions.radius);
                    }
                } else if (obstacle.type == Obstacle::BOX) {
                    auto box_tuple = std::make_tuple(
                        obstacle.position,
                        obstacle.dimensions.width,
                        obstacle.dimensions.height,
                        obstacle.dimensions.rotation
                    );
                    if (is_threat) threat_boxes.push_back(box_tuple);
                    else safe_boxes.push_back(box_tuple);
                }

                if (obstacle.is_dynamic && obstacle.velocity.norm() > 0.01) {
                    Eigen::Vector2d vel_pos(obstacle.position.x(), obstacle.position.y());
                    Eigen::Vector2d scaled_velocity = obstacle.velocity * 0.2;
                    if (is_threat) {
                        threat_vel_pos.push_back(vel_pos);
                        threat_vel_val.push_back(scaled_velocity);
                    } else {
                        safe_vel_pos.push_back(vel_pos);
                        safe_vel_val.push_back(scaled_velocity);
                    }
                }
            }


            // In the geometric loop, after updateObstacles / plan, before extracting path:
            Eigen::VectorXd static_robot_state = start_vec; // your static start state
            // If your state space includes time in kinodynamic mode, but here it's R2,
            // ensure the state dimension matches what the planner expects (e.g., just [x,y]).
            // If you're using the same KinodynamicLLPTStar with kd_dim_=2, it's fine.
            // kinodynamic_planner->setRobotState(static_robot_state);
            auto t1 = std::chrono::steady_clock::now();
            kinodynamic_planner->setRobotState(static_robot_state , false);
            auto t2 = std::chrono::steady_clock::now();
            double ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
            RCLCPP_INFO(rclcpp::get_logger("SETROBOTSTATE"), " took: %.6f ms", ms);


            if(should_vis){
                Eigen::Vector3d robot_pos(start_vec(0), start_vec(1), 0.0);
                Eigen::VectorXd orientation_quat(4);
                orientation_quat << 0, 0, 0, 1;
                std::vector<float> robot_color = {0.0f, 0.0f, 1.0f};
                std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> empty_trace;

                visualization->publishObstacleFrame(
                    safe_cyl_pos, safe_cyl_radii, threat_cyl_pos, threat_cyl_radii,
                    safe_boxes, threat_boxes, safe_vel_pos, safe_vel_val, threat_vel_pos, threat_vel_val,
                    empty_trace, robot_pos, orientation_quat, robot_color, 0, "", {}, {}, "map"
                );

                kinodynamic_planner->visualizeTree();


                auto current_path = kinodynamic_planner->getPathPositions();
                if (!current_path.empty()) {
                    kinodynamic_planner->visualizePathGradient(current_path);
                } else {
                    // path is empty → planner is trapped
                    RCLCPP_WARN(vis_node->get_logger(), "No path found!");
                }


                visualization->triggerPublish();
            }


            
            sim_time += slice_dt;
            auto loop_end_time = std::chrono::steady_clock::now();
            double loop_wall_ms = std::chrono::duration<double, std::milli>(loop_end_time - loop_start_time).count();

#if USE_METRIC
            {
                // // LOOP-END MARKER
                // double loop_wall_ms = std::chrono::duration<double, std::milli>(loop_end_time - loop_start_time).count();

                LogEntry slice_entry;
                slice_entry.row_id = next_row_id++;
                slice_entry.event_type = "slice_end";
                slice_entry.elapsed_s = std::chrono::duration<double>(loop_end_time - global_start).count();
                slice_entry.sim_time = sim_time;
                slice_entry.total_latency_ms = loop_wall_ms;
                log_data.push_back(slice_entry);
            }
#endif


#if USE_1_X
            // Cosmetic pacing — visualization only, no effect on benchmarks
            if (should_vis) {
                double target_ms = slice_dt * 1000.0;
                if (loop_wall_ms < target_ms) {
                    std::this_thread::sleep_for(
                        std::chrono::duration<double, std::milli>(target_ms - loop_wall_ms));
                }
            }
#endif


        }
#if USE_METRIC
        {
            auto t_final = std::chrono::steady_clock::now();
            LogEntry final;
            final.row_id = next_row_id++;
            final.event_type = "sim_end";   // renamed from "time_limit"
            final.elapsed_s = std::chrono::duration<double>(t_final - global_start).count();
            final.sim_time = sim_time;
            final.time_to_goal = 0.0;
            final.tree_size = kinodynamic_planner->getTreeSize();
            final.isolated_nodes = kinodynamic_planner->getIsolatedNodeCount();
            final.avg_deg_out = kinodynamic_planner->getAvgOutDegree();
            final.avg_deg_in = kinodynamic_planner->getAvgInDegree();
            final.neighborhood_radius = kinodynamic_planner->getNeighborhoodRadius();
            log_data.push_back(final);
        }
#endif



    }
    else {


#if USE_REALTIME
        // ============================================================================
        // REAL-TIME KINODYNAMIC MODE (PURE WALL-CLOCK, SAME-SLICE ACCOUNTING)
        // ============================================================================
        RCLCPP_INFO(vis_node->get_logger(), "Starting Kinodynamic Planning Loop (REAL-TIME)...");
        auto last_step_wall = std::chrono::steady_clock::now();   // Tracks wall clock for real-time pacing
        bool first_path_ready = false;   // ensures screenshot waits for a path
        double pending_anchor_repair_ms = 0.0; // Accumulated for Python update_ms logging

        // Even if compute were instant, the world still moves forward one control period,
        // because the controller physically cannot issue commands faster than its rate.
        const double control_dt = cfg.slice_time; // control rate
        double decision_latency_ms = 0.0; // ALL compute charged to the current slice (diagnostic only)

        while (g_running && rclcpp::ok()) {
            executor.spin_some();

            Eigen::VectorXd current_sim_state = ros_manager->getCurrentSimulatedState();
            if (current_sim_state.size() == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // READ REPAIR FROM PREVIOUS ITERATION'S setRobotState
            if (cfg.heuristic) {
                if (cfg.planner_type == "KinodynamicPRMStarDStarLite") {
                    pending_anchor_repair_ms += dlite->getLastAnchorRepairMs();
                    dlite->resetLastAnchorRepairMs();
                } else if (cfg.planner_type == "KinodynamicLLPTStar") {
                    pending_anchor_repair_ms += llpt->getLastAnchorRepairMs();
                    llpt->resetLastAnchorRepairMs();
                }
            }

            double T_robot = current_sim_state(current_sim_state.size() - 1);
            Eigen::Vector2d robot_pos = current_sim_state.head<2>();
            double sim_time = cfg.time_budget - T_robot;

            if (T_robot <= 0.0) {
                g_running = false;
                break;
            }

            kinodynamic_planner->setCurrentRobotTime(T_robot);

#if SCREEN_SHOT
            if (!captured_start && first_path_ready) {
                visualization->takeScreenshot(T_robot, false);
                captured_start = true;
            }

            if (T_robot <= next_screenshot_time && T_robot > 0.5) {
                visualization->takeScreenshot(T_robot, false);
                next_screenshot_time -= 4.2;
            }
#endif

            // =========================================================================
            // ANYTIME WORK ABOVE THE GATE
            // Only true anytime refinement lives here.
            // =========================================================================
            if (is_anytime) {
                kinodynamic_planner->resetMetrics(); // reset for plan event
                auto t1 = std::chrono::steady_clock::now();
                kinodynamic_planner->plan();
                auto t2 = std::chrono::steady_clock::now();
                double plan_time_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
                decision_latency_ms += plan_time_ms;

#if USE_METRIC
                {
                    LogEntry plan_entry;
                    const auto& plan_metrics = kinodynamic_planner->getLastReplanMetrics();

                    plan_entry.row_id = next_row_id++;
                    plan_entry.event_type = "plan";
                    plan_entry.elapsed_s = std::chrono::duration<double>(
                        t2 - global_start).count();
                    plan_entry.sim_time = sim_time;
                    plan_entry.time_to_goal = T_robot;

                    plan_entry.plan_ms = plan_time_ms;
                    plan_entry.path_cost = plan_metrics.path_cost;
                    plan_entry.obstacle_checks = plan_metrics.obstacle_checks;
                    plan_entry.tree_size = kinodynamic_planner->getTreeSize();
                    plan_entry.isolated_nodes = kinodynamic_planner->getIsolatedNodeCount();

                    log_data.push_back(plan_entry);
                }
#endif
            }

            if (should_vis) {
                visualization->visualizeTimeToGoal(T_robot, -42.5, 45.0);
                kinodynamic_planner->visualizeTree();

                auto live_path = kinodynamic_planner->getLivePathPositions(current_sim_state);
                if (!live_path.empty()) {
                    kinodynamic_planner->visualizePathGradient(live_path);
                }
            }

            // =========================================================================
            // WALL-CLOCK STEP GATE
            // =========================================================================
            double wall_since_step_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - last_step_wall).count();

            if (wall_since_step_ms < control_dt * 1000.0) {
                continue;
            }

            // -------------------------------------------------------------------------
            // CURRENT-SLICE REACTION WORK BEFORE THE STEP
            // -------------------------------------------------------------------------
            gazebo_checker->processLatestPoseInfo(sim_time);
            ObstacleVector turned_obs = gazebo_checker->checkAndRepairObstacles(T_robot, robot_pos);

            if (!turned_obs.empty()) {
                kinodynamic_planner->resetMetrics(); // reset for updateObstacle event

                auto t1 = std::chrono::steady_clock::now();
                kinodynamic_planner->updateObstacles(turned_obs);
                auto t2 = std::chrono::steady_clock::now();
                double update_time_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();

                // FOLD PENDING REPAIR INTO UPDATE (For Python script)
                if (cfg.heuristic) {
                    update_time_ms += pending_anchor_repair_ms;
                    pending_anchor_repair_ms = 0.0;
                }
                decision_latency_ms += update_time_ms;

                RCLCPP_INFO(rclcpp::get_logger("Planner_Timing"),
                            "updateObstacles: %.2f ms", update_time_ms);

#if USE_METRIC
                {
                    LogEntry update_entry;
                    const auto& update_metrics = kinodynamic_planner->getLastReplanMetrics();

                    update_entry.row_id = next_row_id++;
                    update_entry.event_type = "update";
                    update_entry.elapsed_s = std::chrono::duration<double>(
                        t2 - global_start).count();
                    update_entry.sim_time = sim_time;
                    update_entry.time_to_goal = T_robot;

                    update_entry.update_ms = update_time_ms; // Contains the folded repair cost
                    update_entry.path_cost = update_metrics.path_cost;
                    update_entry.obstacle_checks = update_metrics.obstacle_checks;
                    update_entry.tree_size = kinodynamic_planner->getTreeSize();
                    update_entry.isolated_nodes = kinodynamic_planner->getIsolatedNodeCount();

                    log_data.push_back(update_entry);
                }

                if (should_log_graph && graph_log_file.is_open()) {
                    kinodynamic_planner->logGraphState(graph_log_file, graph_cycle_count);
                    graph_cycle_count++;
                }
#endif
            }

            // Keeping track of bridge safety + anchor reach on the current sampled state
            auto tq1 = std::chrono::steady_clock::now();

            bool opportunity_found = false;

            bool edge_destroyed = false;
            const auto& all_current_obs = gazebo_checker->getObstacles();
            edge_destroyed = !kinodynamic_planner->isCurrentBridgeSafe(all_current_obs);

            bool reached_end_of_edge = kinodynamic_planner->hasReachedAnchor(current_sim_state);

            auto tq2 = std::chrono::steady_clock::now();
            decision_latency_ms += std::chrono::duration<double, std::milli>(tq2 - tq1).count();

            if (edge_destroyed || reached_end_of_edge || !first_path_ready || opportunity_found) {
                kinodynamic_planner->resetMetrics();

                auto t1 = std::chrono::steady_clock::now();
                kinodynamic_planner->setRobotState(current_sim_state, reached_end_of_edge); // WILL BE CHARGED AT THE TOP OF NEXT LOOP
                auto t2 = std::chrono::steady_clock::now();
                double ms = std::chrono::duration<double, std::milli>(t2 - t1).count();

                RCLCPP_INFO(rclcpp::get_logger("SETROBOTSTATE"), " took: %.6f ms", ms);
                decision_latency_ms += ms;

#if USE_METRIC
                {
                    auto t_after_set = std::chrono::steady_clock::now();

                    LogEntry state_entry;
                    const auto& state_metrics = kinodynamic_planner->getLastReplanMetrics();

                    state_entry.row_id = next_row_id++;
                    state_entry.event_type = "set_state";
                    state_entry.elapsed_s = std::chrono::duration<double>(
                        t_after_set - global_start).count();
                    state_entry.sim_time = sim_time;
                    state_entry.time_to_goal = T_robot;

                    state_entry.path_cost = state_metrics.path_cost;
                    state_entry.obstacle_checks = state_metrics.obstacle_checks;
                    state_entry.tree_size = kinodynamic_planner->getTreeSize();
                    state_entry.isolated_nodes = kinodynamic_planner->getIsolatedNodeCount();
                    state_entry.setrobotstate_ms = ms;
                    log_data.push_back(state_entry);
                }
#endif

                auto new_path = kinodynamic_planner->getPathPositions();
                if (!new_path.empty()) {
                    ros_manager->setPath(new_path);
                    first_path_ready = true;
                } else {
                    ros_manager->setPath({});
#if USE_METRIC
                    LogEntry trapped_entry;
                    trapped_entry.row_id = next_row_id++;
                    trapped_entry.event_type = "planner_trapped";
                    trapped_entry.elapsed_s = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - global_start).count();
                    trapped_entry.sim_time = sim_time;
                    trapped_entry.time_to_goal = T_robot;
                    const auto& metrics = kinodynamic_planner->getLastReplanMetrics();
                    trapped_entry.path_cost = metrics.path_cost;
                    trapped_entry.obstacle_checks = metrics.obstacle_checks;
                    trapped_entry.tree_size = kinodynamic_planner->getTreeSize();
                    trapped_entry.isolated_nodes = kinodynamic_planner->getIsolatedNodeCount();
                    log_data.push_back(trapped_entry);
#endif
                    g_running = false;
                    break;
                }
            }

            // -------------------------------------------------------------------------
            // IMMEDIATE CATCH-UP
            // -------------------------------------------------------------------------
            auto t_step = std::chrono::steady_clock::now();
            double slice_wall_ms = std::chrono::duration<double, std::milli>(
                t_step - last_step_wall).count();
            // double applied_step_s = slice_wall_ms / 1000.0;
            double applied_step_s = std::max(slice_wall_ms / 1000.0, control_dt); // If one planner is fast we cant step the simulator just by a micro second because in reality we cant be faster than control rate!


            ros_manager->stepSimulation(applied_step_s);
            last_step_wall = std::chrono::steady_clock::now();

            Eigen::VectorXd updated_state = ros_manager->getCurrentSimulatedState();
            if (updated_state.size() == 0) {
                decision_latency_ms = 0.0;
                continue;
            }

            double updated_T_robot = updated_state(updated_state.size() - 1);
            double updated_sim_time = cfg.time_budget - updated_T_robot;

#if USE_METRIC
            {
                LogEntry slice_entry;
                slice_entry.row_id = next_row_id++;
                slice_entry.event_type = "slice_end";
                slice_entry.elapsed_s = std::chrono::duration<double>(
                    t_step - global_start).count();
                slice_entry.sim_time = updated_sim_time;
                slice_entry.time_to_goal = updated_T_robot;
                slice_entry.total_latency_ms = slice_wall_ms;
                slice_entry.decision_latency_ms = decision_latency_ms;
                slice_entry.applied_step_s = applied_step_s;
                log_data.push_back(slice_entry);
            }
#endif

            decision_latency_ms = 0.0;

            // GOAL CHECK ON POST-STEP STATE
            double dist_to_goal = (updated_state.head<2>() - goal_vec.head<2>()).norm();
            if (dist_to_goal < cfg.goal_radius) {
                ExecutedMetrics em = ros_manager->getExecutedMetrics();
                ros_manager->notifyGoalReached();

#if SCREEN_SHOT
                double final_t_robot = updated_state(updated_state.size() - 1);
                visualization->takeScreenshot(final_t_robot, true);
#endif
#if USE_METRIC
                {
                    kinodynamic_planner->resetMetrics();
                    auto t_goal_state = std::chrono::steady_clock::now();
                    RCLCPP_INFO(vis_node->get_logger(), "Goal Reached!");

                    LogEntry final_entry;
                    const auto& final_metrics = kinodynamic_planner->getLastReplanMetrics();

                    final_entry.row_id = next_row_id++;
                    final_entry.event_type = "goal_reached";
                    final_entry.elapsed_s = std::chrono::duration<double>(
                        t_goal_state - global_start).count();
                    final_entry.sim_time = cfg.time_budget - updated_state(updated_state.size() - 1);
                    final_entry.time_to_goal = updated_state(updated_state.size() - 1);

                    final_entry.path_cost = 0.0;
                    final_entry.tree_size = kinodynamic_planner->getTreeSize();
                    final_entry.isolated_nodes = kinodynamic_planner->getIsolatedNodeCount();
                    final_entry.avg_deg_out = kinodynamic_planner->getAvgOutDegree();
                    final_entry.avg_deg_in = kinodynamic_planner->getAvgInDegree();
                    final_entry.neighborhood_radius = kinodynamic_planner->getNeighborhoodRadius();

                    final_entry.reached_goal = true;
                    final_entry.exec_length = em.path_length;
                    final_entry.exec_time = em.arrival_time;
                    final_entry.exec_turn = em.heading_change;
                    final_entry.exec_effort = em.control_effort;

                    // FLUSH DROPPED-TAIL REPAIR
                    if (cfg.heuristic && pending_anchor_repair_ms > 0.0) {
                        if (std::isnan(final_entry.update_ms))
                            final_entry.update_ms = 0.0;
                        final_entry.update_ms += pending_anchor_repair_ms;
                        pending_anchor_repair_ms = 0.0;
                    }

                    log_data.push_back(final_entry);
                }
#endif
                g_running = false;
            }

            if (should_vis) {
                visualization->triggerPublish();
            }
        }


#else
        // --- KINODYNAMIC MODE (R2T, Dubins, Thruster) ---
        RCLCPP_INFO(vis_node->get_logger(), "Starting Kinodynamic Planning Loop...");
        auto slice_start_time = std::chrono::steady_clock::now(); // CHANGED: keep only slice landmark, removed accumulators
        bool first_path_ready = false;   // ensures screenshot waits for a path
        double pending_anchor_repair_ms = 0.0; // Specific to DLITE
        double last_shortcut_check_time = 0.0; 
// #if USE_REALTIME
//         //even if compute were instant, the world still moves forward one control period, because the controller physically cannot issue commands faster than its rate.
//         const double control_dt = 0.02; // control rate
// #endif
        double decision_latency_ms = 0.0; // real decision compute only (for real time latency)




        const double perception_ms = 0.0; // realistic sensor+detection cost when modeling deployment


        while (g_running && rclcpp::ok()) {
            executor.spin_some();


            Eigen::VectorXd current_sim_state = ros_manager->getCurrentSimulatedState();
            if (current_sim_state.size() == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // kinodynamic_planner->resetMetrics(); // Reset for setRobotState even loggin


            // auto t1 = std::chrono::steady_clock::now();
            // kinodynamic_planner->setRobotState(current_sim_state);
            // auto t2 = std::chrono::steady_clock::now();
            // double ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
            // RCLCPP_INFO(rclcpp::get_logger("SETROBOTSTATE"), " took: %.6f ms", ms);
            

            
            if (cfg.heuristic) { // Specific to DLITE
                if(cfg.planner_type == "KinodynamicPRMStarDStarLite"){
                    double repair = dlite->getLastAnchorRepairMs();
                    pending_anchor_repair_ms += repair;
                    dlite->resetLastAnchorRepairMs();
                }
                else if(cfg.planner_type == "KinodynamicLLPTStar"){
                    double repair = llpt->getLastAnchorRepairMs();
                    pending_anchor_repair_ms += repair;
                    llpt->resetLastAnchorRepairMs();

                }
            }


            auto t_set_state = std::chrono::steady_clock::now();  // setRobotState event completion time

            double T_robot = current_sim_state(current_sim_state.size() - 1);
            Eigen::Vector2d robot_pos = current_sim_state.head<2>(); // (x,y)

            double sim_time = cfg.time_budget - T_robot;
            if (T_robot <= 0.0) {
                // Log a time-limit event if desired
                g_running = false;
                break;
            }
            kinodynamic_planner->setCurrentRobotTime(T_robot);

#if SCREEN_SHOT
            if (!captured_start && first_path_ready) {
                visualization->takeScreenshot(T_robot, false);
                captured_start = true;
            }

            if (T_robot <= next_screenshot_time && T_robot > 0.5) {
                visualization->takeScreenshot(T_robot, false);
                next_screenshot_time -= 10.0;
            }
#endif


            gazebo_checker->processLatestPoseInfo(sim_time);
            ObstacleVector turned_obs = gazebo_checker->checkAndRepairObstacles(T_robot, robot_pos);
            int count = 0;
            // 1. MEASURE UPDATE TIME
            if (!turned_obs.empty()) {
                kinodynamic_planner->resetMetrics(); // reset for updateObstacle event

                auto t1 = std::chrono::steady_clock::now();
                kinodynamic_planner->updateObstacles(turned_obs);
                auto t2 = std::chrono::steady_clock::now();
                double update_time_ms = std::chrono::duration<double, std::milli>(t2 - t1).count(); // CHANGED
                decision_latency_ms += update_time_ms;


                if (cfg.heuristic){ //Specific to DLITE
                    update_time_ms += pending_anchor_repair_ms;
                    // std::cout << "PENDING ANCHOR REPAIR: " << std::fixed << std::setprecision(6) << pending_anchor_repair_ms << "\n";
                    pending_anchor_repair_ms = 0.0;
                }

                RCLCPP_INFO( rclcpp::get_logger("Planner_Timing"), "updateObstacles: %.2f ms", update_time_ms);

#if USE_METRIC
                {   // CHANGED: log update as its own event row
                    LogEntry update_entry;
                    const auto& update_metrics = kinodynamic_planner->getLastReplanMetrics();

                    update_entry.row_id = next_row_id++;
                    update_entry.event_type = "update";
                    update_entry.elapsed_s = std::chrono::duration<double>(
                        t2 - global_start).count();
                    update_entry.sim_time = sim_time;
                    update_entry.time_to_goal = T_robot;

                    update_entry.update_ms = update_time_ms;

                    update_entry.path_cost = update_metrics.path_cost;
                    update_entry.obstacle_checks = update_metrics.obstacle_checks;
                    update_entry.tree_size = kinodynamic_planner->getTreeSize();
                    update_entry.isolated_nodes = kinodynamic_planner->getIsolatedNodeCount();

                    log_data.push_back(update_entry);
                }

                if (should_log_graph && graph_log_file.is_open()) {
                    kinodynamic_planner->logGraphState(graph_log_file, graph_cycle_count);
                    graph_cycle_count++;
                }
#endif
            count = kinodynamic_planner->getLastReplanMetrics().obstacle_checks;
            // std::cout<<"UpdateCollisionChecks: "<<kinodynamic_planner->getLastReplanMetrics().obstacle_checks<<"\n";
            }

            // 2. MEASURE PLAN TIME
            if (is_anytime) {
                kinodynamic_planner->resetMetrics(); // reset for plan event
                auto t1 = std::chrono::steady_clock::now();
                kinodynamic_planner->plan();
                auto t2 = std::chrono::steady_clock::now();
                double plan_time_ms = std::chrono::duration<double, std::milli>(t2 - t1).count(); // CHANGED
                decision_latency_ms += plan_time_ms;

                if (!turned_obs.empty()) {
                    RCLCPP_INFO( rclcpp::get_logger("Planner_Timing"), "plan took: %.2f ms", plan_time_ms);
                }

#if USE_METRIC
                {   // CHANGED: log anytime plan as its own event row
                    LogEntry plan_entry;
                    const auto& plan_metrics = kinodynamic_planner->getLastReplanMetrics();

                    plan_entry.row_id = next_row_id++;
                    plan_entry.event_type = "plan";
                    plan_entry.elapsed_s = std::chrono::duration<double>(
                        t2 - global_start).count();
                    plan_entry.sim_time = sim_time;
                    plan_entry.time_to_goal = T_robot;

                    plan_entry.plan_ms = plan_time_ms;

                    plan_entry.path_cost = plan_metrics.path_cost;
                    plan_entry.obstacle_checks = plan_metrics.obstacle_checks;
                    plan_entry.tree_size = kinodynamic_planner->getTreeSize();
                    plan_entry.isolated_nodes = kinodynamic_planner->getIsolatedNodeCount();

                    log_data.push_back(plan_entry);
                }
#endif
            }
            // if(!turned_obs.empty()){
            //     if (cfg.planner_type == "KinodynamicANYRRTX")
            //         std::cout<<"RepairCollisionChecks: "<<count<<"\n";

            //     if (cfg.planner_type == "KinodynamicANYFMTX")
            //         std::cout<<"RepairCollisionChecks: "<<count + kinodynamic_planner->getLastReplanMetrics().obstacle_checks<<"\n";
            // }

            // Keeping track of hasShortcut + isCurrentBridgeSafe + hasReachedAnchor
            auto tq1 = std::chrono::steady_clock::now();

            /*
                hasShortcut function is dangerous for benchmarking when LLPT is involved! because using resolvePathLazy inside actually steps the algorithm away
                from being lazy and it acts like a semi-eager approach! so for benchmarking we ommit the hasShortcut from all!
                or i make the hasShortcut of LLPT , readonly!
            */
            // OPPORTUNISTIC SHORTCUT TIMER
            // static double last_shortcut_check_time = 0.0; 
            const double SHORTCUT_CHECK_INTERVAL = 0.5; // Peek at KD-Tree every 0.5s of sim time
            
            bool opportunity_found = false;
            // if ((sim_time - last_shortcut_check_time) >= SHORTCUT_CHECK_INTERVAL) {
            //     if (first_path_ready) {
            //         opportunity_found = kinodynamic_planner->hasShortcut(current_sim_state, 0.01); //0.01 --> 1 percent improvement!
            //     }
            //     last_shortcut_check_time = sim_time;
            // }
            // // ------------------------------------


            // 1. Did the latest updateObstacles destroy our current edge?
            bool edge_destroyed = false;
            
            // Check against ALL obstacles
            // The dynamic obstacles are always moving, so we must check if their 
            // continuous sweep has intersected our current bridge trajectory.
            const auto& all_current_obs = gazebo_checker->getObstacles(); 
            edge_destroyed = !kinodynamic_planner->isCurrentBridgeSafe(all_current_obs);
            // ----------------------------------------------------------------

            // 2. Have we physically reached the end of the edge we are tracking? (or Did the anchor become inf?)
            bool reached_end_of_edge = kinodynamic_planner->hasReachedAnchor(current_sim_state);

            auto tq2 = std::chrono::steady_clock::now();
            decision_latency_ms += std::chrono::duration<double, std::milli>(tq2 - tq1).count();

    
            

           if (edge_destroyed || reached_end_of_edge || !first_path_ready || opportunity_found) {
                // if (opportunity_found) std::cout << "\n[SHORTCUT] >= 15% Improvement Opportunity Seized!\n";
                std::cout<<"Edge Destroyed: "<<edge_destroyed<<"|"<<"Reached End of Edge: "<<reached_end_of_edge<<"|"<<"Oppurtunity Found: "<<opportunity_found<<"\n";

                kinodynamic_planner->resetMetrics(); // Reset for setRobotState even loggin
                // std::cout<<edge_destroyed<<", "<<reached_end_of_edge<<" , "<<first_path_ready<<"\n";
                auto t1 = std::chrono::steady_clock::now();
                kinodynamic_planner->setRobotState(current_sim_state, reached_end_of_edge);
                auto t2 = std::chrono::steady_clock::now();
                double ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
                RCLCPP_INFO(rclcpp::get_logger("SETROBOTSTATE"), " took: %.6f ms", ms);
                decision_latency_ms += ms;


#if USE_METRIC
                {   
                    auto t_after_set = std::chrono::steady_clock::now();  

                    // log setRobotState snapshot as its own event row
                    LogEntry state_entry;
                    const auto& state_metrics = kinodynamic_planner->getLastReplanMetrics();

                    state_entry.row_id = next_row_id++; 
                    state_entry.event_type = "set_state";
                    state_entry.elapsed_s = std::chrono::duration<double>(
                        t_after_set - global_start).count();
                    state_entry.sim_time = sim_time;
                    state_entry.time_to_goal = T_robot;

                    state_entry.path_cost = state_metrics.path_cost;
                    state_entry.obstacle_checks = state_metrics.obstacle_checks;
                    state_entry.tree_size = kinodynamic_planner->getTreeSize();
                    state_entry.isolated_nodes = kinodynamic_planner->getIsolatedNodeCount();
                    state_entry.setrobotstate_ms = ms;
                    log_data.push_back(state_entry);
                }
#endif
                
                // Fetch the new path resulting from the new anchor
                /*
                    planner emits a time-parameterized path (time is the last element of each waypoint),
                    so the trajectory is the command (There’s no control law turning a plan into actuation)
                */
                auto new_path = kinodynamic_planner->getPathPositions();
                if (!new_path.empty()) {
                    ros_manager->setPath(new_path);
                    first_path_ready = true;
                } else {
                    // Planner is Trapped
                    ros_manager->setPath({}); 
#if USE_METRIC 
                    LogEntry trapped_entry;
                    trapped_entry.row_id = next_row_id++;
                    trapped_entry.event_type = "planner_trapped";
                    trapped_entry.elapsed_s = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - global_start).count();
                    trapped_entry.sim_time = sim_time;
                    trapped_entry.time_to_goal = T_robot;
                    const auto& metrics = kinodynamic_planner->getLastReplanMetrics();
                    trapped_entry.path_cost = metrics.path_cost;
                    trapped_entry.obstacle_checks = metrics.obstacle_checks;
                    trapped_entry.tree_size = kinodynamic_planner->getTreeSize();
                    trapped_entry.isolated_nodes = kinodynamic_planner->getIsolatedNodeCount();
                    log_data.push_back(trapped_entry);
#endif
                    g_running = false;
                    break;   // exit simulation
                }
            }


            if (should_vis){
                // VISUALIZATION
                visualization->visualizeTimeToGoal(T_robot, -42.5, 45.0);
                kinodynamic_planner->visualizeTree();
                // kinodynamic_planner->visualizeTreeReal();
                // kinodynamic_planner->visualizeTreeGradient();

                auto live_path = kinodynamic_planner->getLivePathPositions(current_sim_state);
                if (!live_path.empty()) {
                    kinodynamic_planner->visualizePathGradient(live_path);
                }



            }

            auto t_slice_end = std::chrono::steady_clock::now();
            double slice_wall_ms = std::chrono::duration<double, std::milli>(t_slice_end - slice_start_time).count();

            double applied_step_s = 0.0;
            double decision_latency_ms_logged = 0.0;
            {
// #if USE_REALTIME
//                 decision_latency_ms += perception_ms;
//                 decision_latency_ms_logged = decision_latency_ms;
//                 double decision_latency_s = decision_latency_ms / 1000.0;
//                 /*
//                     you cannot apply a new plan faster than the controller ticks. 
//                     Suppose the planner finishes in 0.5 ms but your control loop runs at 100 Hz (control_dt=10 ms).
//                     The actuators only accept a new reference at the next control edge. 
//                     Finishing early buys you nothing — the fresh plan still waits until the loop ticks
// ;
//                     One honest caveat:
//                     This collapses two asynchronous clocks into one. In a real robot the controller ticks at control_dt
//                     continuously while the planner updates the reference whenever it finishes, asynchronously. 
//                     A fully faithful model runs those on separate threads. 
//                     The max is the single-threaded simplification of that, and for my purpose it’s the conservative, defensible choice: 
//                     it never understates how long the robot ran on a stale plan. 
//                     So for a latency stress test it’s not just legitimate, it’s the right reduction.
//                 */
//                 applied_step_s = std::max(decision_latency_s, control_dt);
//                 // std::cout<<"APPLIED_STEP_S: "<<applied_step_s<<"\n";
//                 ros_manager->stepSimulation(applied_step_s);
// #else
                decision_latency_ms_logged = decision_latency_ms;          // bonus diagnostic in slice mode
                applied_step_s = cfg.slice_time;
                ros_manager->stepSimulation(cfg.slice_time);
// #endif
                decision_latency_ms = 0.0; // Clear for next iteration


#if USE_METRIC
                {
                    Eigen::VectorXd updated_state = ros_manager->getCurrentSimulatedState();
                    double updated_T_robot = updated_state(updated_state.size() - 1);
                    double updated_sim_time = cfg.time_budget - updated_T_robot;

                    LogEntry slice_entry;
                    slice_entry.row_id = next_row_id++;
                    slice_entry.event_type = "slice_end";
                    slice_entry.elapsed_s = std::chrono::duration<double>(
                        t_slice_end - global_start).count();
                    slice_entry.sim_time = updated_sim_time;
                    slice_entry.time_to_goal = updated_T_robot;
                    slice_entry.total_latency_ms = slice_wall_ms;
// #if USE_REALTIME
                    slice_entry.decision_latency_ms = decision_latency_ms_logged; // drove the step in realtime
                    slice_entry.applied_step_s      = applied_step_s;             // actual sim advance
// #endif
                    log_data.push_back(slice_entry);
                }
#endif

                // GOAL CHECK
                Eigen::VectorXd updated_state = ros_manager->getCurrentSimulatedState();
                // double dist_to_goal = (updated_state.head<2>() - goal_vec.head<2>()).norm();
                double dist_to_goal = (updated_state.head<2>() - goal_vec.head<2>()).norm();
                if (dist_to_goal < cfg.goal_radius) {
                    ExecutedMetrics em = ros_manager->getExecutedMetrics(); // SOLUTION QUALITY!

                    ros_manager->notifyGoalReached(); // Artifically makes everything zero for the bar vis. The more accurate way is to consider the root's boundary condtion

#if SCREEN_SHOT
                    double final_t_robot = updated_state(updated_state.size() - 1);
                    visualization->takeScreenshot(final_t_robot, true);
#endif
#if USE_METRIC
                    {   
                        // Final path cost of the robot when it reached the boundary of the Goal Region
                        kinodynamic_planner->resetMetrics();
                        // kinodynamic_planner->setRobotState(updated_state);
                        auto t_goal_state = std::chrono::steady_clock::now();
                        RCLCPP_INFO(vis_node->get_logger(), "Goal Reached!");
                        // goal reached is its own terminal event row
                        LogEntry final_entry;
                        const auto& final_metrics = kinodynamic_planner->getLastReplanMetrics();

                        final_entry.row_id = next_row_id++; // CHANGED
                        final_entry.event_type = "goal_reached"; // CHANGED
                        final_entry.elapsed_s = std::chrono::duration<double>(
                            t_goal_state - global_start).count(); // CHANGED
                        final_entry.sim_time = cfg.time_budget - updated_state(updated_state.size() - 1); // CHANGED
                        final_entry.time_to_goal = updated_state(updated_state.size() - 1);

                        final_entry.path_cost = 0.0;
                        final_entry.tree_size = kinodynamic_planner->getTreeSize();
                        final_entry.isolated_nodes = kinodynamic_planner->getIsolatedNodeCount();
                        final_entry.avg_deg_out = kinodynamic_planner->getAvgOutDegree();
                        final_entry.avg_deg_in = kinodynamic_planner->getAvgInDegree();
                        final_entry.neighborhood_radius = kinodynamic_planner->getNeighborhoodRadius();


                        final_entry.reached_goal = true;
                        final_entry.exec_length  = em.path_length;
                        final_entry.exec_time    = em.arrival_time;   // == cfg.time_budget - updated_T_robot
                        final_entry.exec_turn    = em.heading_change;
                        final_entry.exec_effort  = em.control_effort;

                        if (cfg.heuristic && pending_anchor_repair_ms > 0.0) {
                            if (std::isnan(final_entry.update_ms))
                                final_entry.update_ms = 0.0;
                            final_entry.update_ms += pending_anchor_repair_ms;
                            pending_anchor_repair_ms = 0.0;
                        }
                        log_data.push_back(final_entry);
                    }
#endif
                    g_running = false;
                }
            }

            if(should_vis){
                visualization->triggerPublish();
            }


            // RViz does NOT play back at 1x by itself. It just renders whatever pose was
            // last published. Nothing forces a published frame to linger on screen for the
            // amount of *sim time* that frame represents. So perceived speed = how fast
            // this loop iterates, NOT how much sim time each iteration advances.
            //
            // Three things make sim-clock outrun the wall-clock here:
            //
            //   1. perception_ms = 100.0 is a MODELED cost. We ADD it to decision_latency_ms
            //      to advance the sim clock, but we never actually *spend* 100 ms of CPU on
            //      it. So every iteration jumps the sim forward >=100 ms while the real loop
            //      body only burns a few ms. Result: sim runs ~20-50x faster than real life.
            //
            //   2. USE_REALTIME ties applied_step_s to COMPUTE time, not WALL time. Those are
            //      not the same clock. "The world moved 50 ms while I thought for 50 ms" is
            //      causally correct for the *model*, but it says nothing about how long the
            //      iteration takes on your screen.
            //
            //   3. When the control_dt floor kicks in (compute < 20 ms), the sim still jumps
            //      a full 20 ms while the iteration finishes in microseconds -> faster again.
            //
            // FIX: pad each iteration with a real sleep so the wall-clock time spent equals
            // applied_step_s. This stretches each frame to its intended real duration, so
            // your eyes finally see it at 1x. This is COSMETIC ONLY — every metric was
            // already captured above, so sleeping here changes no logged number. It is
            // gated on should_vis so benchmark runs stay at full speed.
            // ============================================================================
#if USE_1_X
            if (should_vis) {
                auto iter_now = std::chrono::steady_clock::now();
                double iter_wall_ms = std::chrono::duration<double, std::milli>(
                    iter_now - slice_start_time).count();
                double target_ms = applied_step_s * 1000.0;   // real time this frame should occupy
                double sleep_ms  = target_ms - iter_wall_ms;   // remaining real time to burn
                if (sleep_ms > 0.0) {
                    std::this_thread::sleep_for(
                        std::chrono::duration<double, std::milli>(sleep_ms));
                }
                // If sleep_ms < 0 the compute was slower than applied_step_s, so we're
                // already behind real time and simply don't sleep — the sim falls behind
                // gracefully instead of speeding up. State stays correct either way.
            }
#endif



            slice_start_time = std::chrono::steady_clock::now();

        }
#endif 
    }
    std::cout<<"OUT: "<<kinodynamic_planner->getAvgOutDegree()<<", IN: "<<kinodynamic_planner->getAvgInDegree()<<"\n";
    std::cout<<"FINAL TREE SIZE: "<<planner->getTreeSize()<<"\n";
    CALLGRIND_STOP_INSTRUMENTATION;



    if (cfg.planner_type == "KinodynamicRRTX"){
        auto rrtx_planner = dynamic_cast<KinodynamicRRTX*>(planner.get());
        rrtx_planner->dumpTreeToCSV("rrtx_tree_nodes.csv");
    }
    if (cfg.planner_type == "KinodynamicANYRRTX"){
        auto anyrrtx_planner = dynamic_cast<KinodynamicANYRRTX*>(planner.get());
        anyrrtx_planner->dumpTreeToCSV("anyrrtx_tree_nodes.csv");
    }
    if (cfg.planner_type == "KinodynamicANYFMTX"){
        auto fmtx_planner = dynamic_cast<KinodynamicANYFMTX*>(planner.get());
        fmtx_planner->dumpTreeToCSV("anyfmtx_tree_nodes.csv");
    }



#if USE_METRIC
    // --- 10. Save Metrics ---
    int final_collision_count = (is_geometric_mode) ? 0 : ros_manager->getCollisionCount();
    for (auto& entry : log_data) {
        entry.collision_count = final_collision_count;
    }

    std::time_t now_time = std::time(nullptr);
    std::tm* local_tm = std::localtime(&now_time);
    char time_buf[80];
    strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", local_tm);

    std::string filename = "sim_" + cfg.name + "_" + time_buf + "_metrics.csv";
    std::cout << "Writing metrics to: " << filename << std::endl;

    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cerr << "Error: failed to open " << filename << std::endl;
        return 1;
    }

    out << "row_id,event_type,elapsed_s,sim_time,setup_ms,total_latency_ms,update_ms,plan_ms,"
           "time_to_goal,path_cost,obstacle_checks,collision_count,tree_size,isolated_nodes,"
           "avg_deg_out,avg_deg_in,neighborhood_radius,"
           "setrobotstate_ms,decision_latency_ms,applied_step_s,"
           "reached_goal,exec_length,exec_time,exec_turn,exec_effort\n";

    for (const auto& entry : log_data) {
        out << entry.row_id << ","
            << entry.event_type << ","
            << entry.elapsed_s << ","
            << entry.sim_time << ","
            << entry.setup_ms << ","
            << entry.total_latency_ms << ","
            << entry.update_ms << ","
            << entry.plan_ms << ","
            << entry.time_to_goal << ","
            << entry.path_cost << ","
            << entry.obstacle_checks << ","
            << entry.collision_count << ","
            << entry.tree_size << ","
            << entry.isolated_nodes << ","
            << entry.avg_deg_out << ","
            << entry.avg_deg_in << ","
            << entry.neighborhood_radius << ","
            << entry.setrobotstate_ms << ","
            << entry.decision_latency_ms << ","
            << entry.applied_step_s << ","
            << entry.reached_goal << ","
            << entry.exec_length << ","
            << entry.exec_time << ","
            << entry.exec_turn << ","
            << entry.exec_effort << "\n";
    }

    out.close();

    if (graph_log_file.is_open()) {
        graph_log_file.close();
        std::cout << "Graph log file closed successfully. Total cycles saved: "
                  << graph_cycle_count << std::endl;
    }
#endif

    rclcpp::shutdown();
    return 0;
}