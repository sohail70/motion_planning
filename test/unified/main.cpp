// Copyright 2025 Soheil E.nia
#include "motion_planning/planners/planner_factory.hpp"
#include "motion_planning/state_space/rdt_statespace.hpp"
#include "motion_planning/state_space/dubins_time_statespace.hpp"
#include "motion_planning/state_space/thruster_statespace.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
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

// --- Config Struct ---
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
    int spatial_dim;
    std::vector<double> bounds_min;
    std::vector<double> bounds_max;
    double min_velocity = 0.0;
    double max_velocity = 20.0;
    double robot_velocity = 10.0;
    double min_turning_radius = 2.0;
    double max_acceleration = 5.0;
    std::string sdf_path;
    std::map<std::string, std::string> planner_params_str;
    std::map<std::string, std::string> manager_params_str;
    std::map<std::string, std::string> gazebo_params_str;
};

ExperimentConfig loadConfig(const std::string& filepath) {
    YAML::Node config = YAML::LoadFile(filepath);
    ExperimentConfig c;
    c.name = config["experiment"]["name"].as<std::string>();
    c.planner_type = config["experiment"]["planner_type"].as<std::string>();
    c.state_space_type = config["experiment"]["state_space_type"].as<std::string>();
    c.manager_type = config["experiment"]["manager_type"].as<std::string>();
    c.time_budget = config["simulation"]["time_budget"].as<double>();
    c.duration_limit = config["simulation"]["duration_limit"].as<int>();
    c.slice_time = config["simulation"]["slice_time"].as<double>();
    c.seed = config["simulation"]["seed"].as<unsigned int>();
    c.start_state = config["robot"]["start_state"].as<std::vector<double>>();
    c.goal_state = config["robot"]["goal_state"].as<std::vector<double>>();
    c.dimensions = config["robot"]["dimensions"].as<int>();
    c.spatial_dim = config["robot"]["spatial_dim"].as<int>();
    c.bounds_min = config["robot"]["bounds_min"].as<std::vector<double>>();
    c.bounds_max = config["robot"]["bounds_max"].as<std::vector<double>>();
    if (config["robot"]["min_velocity"]) c.min_velocity = config["robot"]["min_velocity"].as<double>();
    if (config["robot"]["max_velocity"]) c.max_velocity = config["robot"]["max_velocity"].as<double>();
    if (config["robot"]["robot_velocity"]) c.robot_velocity = config["robot"]["robot_velocity"].as<double>();
    if (config["robot"]["min_turning_radius"]) c.min_turning_radius = config["robot"]["min_turning_radius"].as<double>();
    if (config["robot"]["max_acceleration"]) c.max_acceleration = config["robot"]["max_acceleration"].as<double>();
    c.sdf_path = config["gazebo_params"]["sdf_path"].as<std::string>();
    
    auto load_map = [](YAML::Node node, std::map<std::string, std::string>& map) {
        for (const auto& kv : node) {
            map[kv.first.as<std::string>()] = kv.second.as<std::string>();
        }
    };
    if (config["planner_params"]) load_map(config["planner_params"], c.planner_params_str);
    if (config["manager_params"]) load_map(config["manager_params"], c.manager_params_str);
    if (config["gazebo_params"]) load_map(config["gazebo_params"], c.gazebo_params_str);
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

struct LogEntry {
    double elapsed_s = 0.0;
    // double duration_ms = 0.0;
    
    double total_latency_ms = 0.0; // The sum (Control Loop Delay)
    double update_ms = 0.0;        // Time spent in updateObstacleSamples
    double plan_ms = 0.0;          // Time spent in plan()

    double time_to_goal = 0.0;
    double path_cost = 0.0;
    int obstacle_checks = 0;
    long long rewire_neighbor_searches = 0;
    int orphaned_nodes = 0;
    int collision_count = 0;
    int tree_size = 0;
    double avg_deg_out = 0.0;
    double avg_deg_in = 0.0;
    double neighborhood_radius = 0.0;
};

std::atomic<bool> g_running{true};
void sigint_handler(int sig) { g_running = false; }

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

    // --- 1. Parameters ---
    Params manager_params, gazebo_params, planner_params;
    populateParams(manager_params, cfg.manager_params_str);
    populateParams(gazebo_params, cfg.gazebo_params_str);
    populateParams(planner_params, cfg.planner_params_str);

    // CRITICAL: Propagate shared parameters to ensure consistency
    gazebo_params.setParam("initial_budget_time", cfg.time_budget);
    manager_params.setParam("inflation", gazebo_params.getParam<double>("inflation"));
    
    // Determine mode globally
    bool is_geometric_mode = (cfg.time_budget == 0.0);
    
    // FORCE SYNC: Ensure is_geometric_mode is set in both Gazebo and Planner params
    gazebo_params.setParam("is_geometric_mode", is_geometric_mode);
    planner_params.setParam("is_geometric_mode", is_geometric_mode);

    // --- 2. ROS & Vis ---
    auto vis_node = std::make_shared<rclcpp::Node>(cfg.name + "_visualizer",
        rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
    auto visualization = std::make_shared<RVizVisualization>(vis_node);
    auto sim_clock = vis_node->get_clock();

    // --- 3. Obstacles ---
    auto obstacle_info = parseSdfObstacles(cfg.sdf_path);
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(sim_clock, gazebo_params, obstacle_info);

    // --- 4. Problem Definition ---
    auto problem_def = std::make_shared<ProblemDefinition>(cfg.dimensions);
    Eigen::VectorXd start_vec(cfg.dimensions);
    Eigen::VectorXd goal_vec(cfg.dimensions);
    Eigen::VectorXd lower_vec(cfg.dimensions);
    Eigen::VectorXd upper_vec(cfg.dimensions);
    for(int i=0; i<cfg.dimensions; ++i) start_vec(i) = cfg.start_state[i];
    for(int i=0; i<cfg.dimensions; ++i) goal_vec(i) = cfg.goal_state[i];
    for(int i=0; i<cfg.dimensions; ++i) lower_vec(i) = cfg.bounds_min[i];
    for(int i=0; i<cfg.dimensions; ++i) upper_vec(i) = cfg.bounds_max[i];
    
    // Planner Setup: Start=Goal, Goal=Start (Backward Search)
    problem_def->setStart(goal_vec);  
    problem_def->setGoal(start_vec);  
    problem_def->setBounds(lower_vec, upper_vec);

    // --- 5. State Space Factory ---
    std::shared_ptr<StateSpace> statespace;
    if (cfg.state_space_type == "RDT") {
        // Pass the boolean flag directly to the constructor
        statespace = std::make_shared<RDTStateSpace>(cfg.spatial_dim, cfg.min_velocity, cfg.max_velocity, cfg.robot_velocity, 30000, cfg.seed, is_geometric_mode);
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
            Eigen::VectorXd r2t_initial_state(3);
            r2t_initial_state(0) = start_vec(0); 
            r2t_initial_state(1) = start_vec(1); 
            r2t_initial_state(2) = cfg.time_budget; 
            ros_manager = std::make_shared<R2TROS2Manager>(obstacle_checker, visualization, manager_params, cfg.robot_velocity, r2t_initial_state, cfg.time_budget);
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
    else {
        std::cerr << "Unknown Planner: " << cfg.planner_type << std::endl;
        return 1;
    }
    auto planner = PlannerFactory::getInstance().createPlanner(p_type, statespace, problem_def, obstacle_checker);
    planner->setup(planner_params, visualization);
    
    auto kinodynamic_planner = std::dynamic_pointer_cast<Planner>(planner); 
    // kinodynamic_planner = std::dynamic_pointer_cast<KinodynamicFMTX>(planner); 
    // if (kinodynamic_planner) kinodynamic_planner->setClock(sim_clock);

    // --- 8. Initial Plan ---
    RCLCPP_INFO(vis_node->get_logger(), "Running initial plan...");
    auto start_t = std::chrono::steady_clock::now();
    
    bool fixed_sample = true;
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
    
    auto end_t = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t);
    std::cout << "Initial plan took: " << duration.count() << " ms" << std::endl;
    
    kinodynamic_planner->setRobotState(start_vec); 
    
    auto path = kinodynamic_planner->getPathPositions();
    if (!path.empty() && ros_manager) {
        ros_manager->setPath(path);
    }

    if (cfg.planner_type == "KinodynamicRRTX"){
        auto rrtx_planner = dynamic_cast<KinodynamicRRTX*>(planner.get());
        rrtx_planner->dumpTreeToCSV("rrtx_tree_nodes.csv");
    }


    // --- 9. Executor Setup ---
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(vis_node);
    if (ros_manager) {
        executor.add_node(ros_manager);
    }

    auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker);
    if (gazebo_checker) {
        // Initialize with T=0 for geometric, or time_budget for kinodynamic
        double initial_T = is_geometric_mode ? 0.0 : start_vec(start_vec.size()-1);
        gazebo_checker->processLatestPoseInfo(0); // Sync physics at T=0
        gazebo_checker->initializeDynamicObstacles(initial_T);
    }

    std::vector<LogEntry> log_data;
    auto global_start = std::chrono::steady_clock::now();
    auto time_limit = std::chrono::seconds(cfg.duration_limit);
    auto start_time = std::chrono::steady_clock::now();
    bool limited = !manager_params.getParam<bool>("follow_path");
    
    CALLGRIND_START_INSTRUMENTATION;

    // // ============================================================
    // // BRANCHING LOGIC
    // // ============================================================
    // if (is_geometric_mode) {
    //     // --- GEOMETRIC MODE (R2) ---
    //     RCLCPP_INFO(vis_node->get_logger(), "Starting Geometric Planning Loop (R2)...");
        
    //     const double loop_rate_hz = 20.0; 
    //     const auto loop_duration = std::chrono::duration<double>(1.0 / loop_rate_hz);
    //     double sim_time = 0.0; 
        
    //     while (g_running && rclcpp::ok()) {
    //         auto loop_start_time = std::chrono::steady_clock::now();
    //         executor.spin_some();

    //         // RESET METRICS AT START OF SLICE
    //         kinodynamic_planner->resetMetrics();
            
    //         if (limited) {
    //             if (std::chrono::steady_clock::now() - start_time > time_limit) {
    //                 std::cout << "[INFO] Time limit reached." << std::endl;
    //                 break;
    //             }
    //         }

    //         // Eigen::VectorXd current_sim_state = start_vec;
    //         // // Resize the vector to have one extra element (preserving old values)
    //         // current_sim_state.conservativeResize(start_vec.size() + 1);
            
    //         // // Set the new last element to zero
    //         // current_sim_state(start_vec.size()) = 0.0;
    //         // kinodynamic_planner->setRobotState(current_sim_state);
    //         sim_time += 1.0 / loop_rate_hz;
    //         gazebo_checker->processLatestPoseInfo(sim_time);
    //         ObstacleVector all_obs = gazebo_checker->getObstacles();

    //         if (!all_obs.empty()) {
    //             auto start_update = std::chrono::steady_clock::now();
    //             kinodynamic_planner->updateObstacleSamples(all_obs);
    //             auto end_update = std::chrono::steady_clock::now();
    //             double duration_ms = std::chrono::duration<double, std::milli>(end_update - start_update).count();

    //             RCLCPP_INFO(rclcpp::get_logger("Planner_Timing"), 
    //                 "updateObstacleSamples took: %.2f ms", duration_ms);

    //             LogEntry entry;
    //             const auto& metrics = kinodynamic_planner->getLastReplanMetrics();
    //             entry.elapsed_s = std::chrono::duration<double>(loop_start_time - global_start).count();
    //             entry.duration_ms = std::chrono::duration<double, std::milli>(end_update - start_update).count();
    //             entry.time_to_goal = 0.0; 
    //             entry.obstacle_checks = metrics.obstacle_checks;
    //             entry.orphaned_nodes = metrics.orphaned_nodes;
    //             entry.path_cost = metrics.path_cost;
    //             entry.rewire_neighbor_searches = metrics.rewire_neighbor_searches;
    //             entry.tree_size = kinodynamic_planner->getTreeSize();
    //             entry.avg_degree = kinodynamic_planner->getAvgNodeDegree();
    //             entry.neighborhood_radius = kinodynamic_planner->getNeighborhoodRadius();
    //             log_data.push_back(entry);
    //         }

    //         if (is_anytime) {
    //             // if (kinodynamic_planner->getTreeSize() < 700) {
    //                 auto start_plan = std::chrono::steady_clock::now();
    //                 kinodynamic_planner->plan();
    //                 auto end_plan = std::chrono::steady_clock::now();
    //                 double plan_ms = std::chrono::duration<double, std::milli>(end_plan - start_plan).count();
    //                 RCLCPP_INFO(rclcpp::get_logger("Planner_Timing"), "plan took: %.2f ms", plan_ms);
    //             // }
    //         }


    //         // --- VISUALIZATION (Fixed) ---
    //         std::vector<Eigen::VectorXd> safe_cyl_pos, threat_cyl_pos; 
    //         std::vector<double> safe_cyl_radii, threat_cyl_radii;
    //         std::vector<std::tuple<Eigen::Vector2d, double, double, double>> safe_boxes, threat_boxes;
    //         std::vector<Eigen::Vector2d> safe_vel_pos, safe_vel_val;
    //         std::vector<Eigen::Vector2d> threat_vel_pos, threat_vel_val;

    //         for (const auto& obstacle : all_obs) {
    //             bool is_threat = false; 
    //             if (obstacle.type == Obstacle::CIRCLE) {
    //                 Eigen::VectorXd pos(2); pos << obstacle.position.x(), obstacle.position.y();
    //                 if (is_threat) { threat_cyl_pos.push_back(pos); threat_cyl_radii.push_back(obstacle.dimensions.radius); }
    //                 else { safe_cyl_pos.push_back(pos); safe_cyl_radii.push_back(obstacle.dimensions.radius); }
    //             } else if (obstacle.type == Obstacle::BOX) {
    //                 auto box_tuple = std::make_tuple(obstacle.position, obstacle.dimensions.width, obstacle.dimensions.height, obstacle.dimensions.rotation);
    //                 if (is_threat) threat_boxes.push_back(box_tuple);
    //                 else safe_boxes.push_back(box_tuple);
    //             }
    //             if (obstacle.is_dynamic && obstacle.velocity.norm() > 0.01) {
    //                 Eigen::Vector2d vel_pos(obstacle.position.x(), obstacle.position.y());
    //                 if (is_threat) { threat_vel_pos.push_back(vel_pos); threat_vel_val.push_back(obstacle.velocity); }
    //                 else { safe_vel_pos.push_back(vel_pos); safe_vel_val.push_back(obstacle.velocity); }
    //             }
    //         }

    //         Eigen::Vector3d robot_pos(start_vec(0), start_vec(1), 0.0); 
    //         Eigen::VectorXd orientation_quat(4); orientation_quat << 0,0,0,1;
    //         std::vector<float> robot_color = {0.0f, 0.0f, 1.0f};
    //         std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> empty_trace;

    //         visualization->publishObstacleFrame(
    //             safe_cyl_pos, safe_cyl_radii, threat_cyl_pos, threat_cyl_radii,
    //             safe_boxes, threat_boxes, safe_vel_pos, safe_vel_val, threat_vel_pos, threat_vel_val,
    //             empty_trace, robot_pos, orientation_quat, robot_color, 0, "map"
    //         );

    //         // VISUALIZE PATH AND TREE (This was missing)
    //         auto new_executable_path = kinodynamic_planner->getPathPositions();
    //         if (!new_executable_path.empty()) {
    //             // kinodynamic_planner->visualizePath(new_executable_path);
    //         }
    //         kinodynamic_planner->visualizeTree();

    //         auto loop_end_time = std::chrono::steady_clock::now();
    //         auto elapsed = std::chrono::duration<double>(loop_end_time - loop_start_time);
    //         if (elapsed < loop_duration) {
    //             std::this_thread::sleep_for(loop_duration - elapsed);
    //         }
    //     }
    // } else {
    //     // --- KINODYNAMIC MODE (R2T, Dubins, Thruster) ---
    //     RCLCPP_INFO(vis_node->get_logger(), "Starting Kinodynamic Planning Loop...");
    //     double time_accumulated_in_slice = 0.0;
    //     auto slice_start_time = std::chrono::steady_clock::now();
        
    //     while (g_running && rclcpp::ok()) {
    //         executor.spin_some();

    //         // RESET METRICS AT START OF SLICE
    //         kinodynamic_planner->resetMetrics();

    //         Eigen::VectorXd current_sim_state = ros_manager->getCurrentSimulatedState();
    //         if (current_sim_state.size() == 0) {
    //             std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //             continue;
    //         }
    //         kinodynamic_planner->setRobotState(current_sim_state);
    //         double T_robot = current_sim_state(current_sim_state.size()-1);
    //         double sim_time = cfg.time_budget - T_robot;
            
    //         gazebo_checker->processLatestPoseInfo(sim_time);
    //         ObstacleVector turned_obs = gazebo_checker->checkAndRepairObstacles(T_robot);
            
    //         auto calc_start = std::chrono::steady_clock::now();
    //         if (!turned_obs.empty()) {
    //             auto start_update = std::chrono::steady_clock::now();
    //             kinodynamic_planner->updateObstacleSamples(turned_obs);
    //             auto end_update = std::chrono::steady_clock::now();
    //             double duration_ms = std::chrono::duration<double, std::milli>(end_update - start_update).count();
    //             RCLCPP_INFO(rclcpp::get_logger("Planner_Timing"), 
    //                 "updateObstacleSamples took: %.2f ms", duration_ms);
    //         }
            
    //         if (is_anytime) {
    //             // if (kinodynamic_planner->getTreeSize() < 700) {
    //                 auto start_update = std::chrono::steady_clock::now();
    //                 kinodynamic_planner->plan();
    //                 auto end_update = std::chrono::steady_clock::now();
    //                 double duration_ms = std::chrono::duration<double, std::milli>(end_update - start_update).count();
    //                 // if(!turned_obs.empty())
    //                     RCLCPP_INFO(rclcpp::get_logger("Planner_Timing"), 
    //                         "plan took: %.2f ms", duration_ms);
    //             // }
                
    //         }
    //         auto calc_end = std::chrono::steady_clock::now();

    //         if (!turned_obs.empty() || is_anytime) {
    //             LogEntry entry;
    //             const auto& metrics = kinodynamic_planner->getLastReplanMetrics();
    //             entry.elapsed_s = std::chrono::duration<double>(slice_start_time - global_start).count();
    //             entry.duration_ms = std::chrono::duration<double, std::milli>(calc_end - calc_start).count();
    //             entry.obstacle_checks = metrics.obstacle_checks;
    //             entry.orphaned_nodes = metrics.orphaned_nodes;
    //             entry.path_cost = metrics.path_cost;
    //             entry.time_to_goal = T_robot;
    //             entry.rewire_neighbor_searches = metrics.rewire_neighbor_searches;
    //             entry.tree_size = kinodynamic_planner->getTreeSize();
    //             entry.avg_degree = kinodynamic_planner->getAvgNodeDegree();
    //             entry.neighborhood_radius = kinodynamic_planner->getNeighborhoodRadius();
    //             log_data.push_back(entry);
    //         }

    //         std::vector<Obstacle> culprits = obstacle_checker->getAndClearCulprits();
    //         ros_manager->updateThreats(culprits);
            
    //         auto new_path = kinodynamic_planner->getPathPositions();
    //         if (!new_path.empty()) {
    //             ros_manager->setPath(new_path);
    //             kinodynamic_planner->visualizePath(new_path);
    //         }
    //         kinodynamic_planner->visualizeTree();
    //         // kinodynamic_planner->visualizeTreeReal();

    //         auto now = std::chrono::steady_clock::now();
    //         double dt_wall = std::chrono::duration<double>(now - slice_start_time).count();
    //         time_accumulated_in_slice += dt_wall;
            
    //         if (time_accumulated_in_slice >= cfg.slice_time) {
    //             ros_manager->stepSimulation(cfg.slice_time);
    //             time_accumulated_in_slice = 0.0;
    //             slice_start_time = std::chrono::steady_clock::now();
                
    //             Eigen::VectorXd updated_state = ros_manager->getCurrentSimulatedState();
    //             double dist_to_goal = (updated_state.head<2>() - goal_vec.head<2>()).norm();
    //             if (dist_to_goal < 3.0) {
    //                 RCLCPP_INFO(vis_node->get_logger(), "Goal Reached!");
    //                 g_running = false;
    //             }
    //         } else {
    //             double remaining = cfg.slice_time - time_accumulated_in_slice;
    //             if (remaining > 0.0) std::this_thread::sleep_for(std::chrono::duration<double>(remaining));
    //         }
    //     }
    // }

// ============================================================
    // BRANCHING LOGIC
    // ============================================================
    if (is_geometric_mode) {
        // --- GEOMETRIC MODE (R2) ---
        RCLCPP_INFO(vis_node->get_logger(), "Starting Geometric Planning Loop (R2)...");
        
        const double loop_rate_hz = 20.0; 
        const auto loop_duration = std::chrono::duration<double>(1.0 / loop_rate_hz);
        double sim_time = 0.0; 
        
        while (g_running && rclcpp::ok()) {
            auto loop_start_time = std::chrono::steady_clock::now();
            executor.spin_some();

            // RESET METRICS AT START OF SLICE
            kinodynamic_planner->resetMetrics();
            
            // TIMING VARIABLES FOR THIS SLICE
            double current_update_ms = 0.0;
            double current_plan_ms = 0.0;

            if (limited) {
                if (std::chrono::steady_clock::now() - start_time > time_limit) {
                    std::cout << "[INFO] Time limit reached." << std::endl;
                    break;
                }
            }

            sim_time += 1.0 / loop_rate_hz;
            gazebo_checker->processLatestPoseInfo(sim_time);
            ObstacleVector all_obs = gazebo_checker->getObstacles();

            // 1. MEASURE UPDATE TIME
            if (!all_obs.empty()) {
                auto t1 = std::chrono::steady_clock::now();
                kinodynamic_planner->updateObstacleSamples(all_obs);
                auto t2 = std::chrono::steady_clock::now();
                current_update_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
                
                if (current_update_ms > 1.0) {
                    RCLCPP_INFO(rclcpp::get_logger("Planner_Timing"), 
                        "updateObstacleSamples took: %.2f ms", current_update_ms);
                }
            }

            // 2. MEASURE PLAN TIME
            if (is_anytime) {
                auto t1 = std::chrono::steady_clock::now();
                kinodynamic_planner->plan();
                auto t2 = std::chrono::steady_clock::now();
                current_plan_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
                
                // Optional: Log only if significant to avoid spam
                RCLCPP_INFO(rclcpp::get_logger("Planner_Timing"), "plan took: %.2f ms", current_plan_ms);
            }

            // 3. LOG SEPARATED METRICS
            if (!all_obs.empty() || is_anytime) {
                LogEntry entry;
                const auto& metrics = kinodynamic_planner->getLastReplanMetrics();
                
                entry.elapsed_s = std::chrono::duration<double>(loop_start_time - global_start).count();
                
                // SAVE SPLIT TIMES
                entry.update_ms = current_update_ms;
                entry.plan_ms = current_plan_ms;
                entry.total_latency_ms = current_update_ms + current_plan_ms;

                entry.time_to_goal = 0.0; 
                entry.obstacle_checks = metrics.obstacle_checks;
                entry.orphaned_nodes = metrics.orphaned_nodes;
                entry.path_cost = metrics.path_cost;
                entry.rewire_neighbor_searches = metrics.rewire_neighbor_searches;
                entry.tree_size = kinodynamic_planner->getTreeSize();
                entry.avg_deg_out = kinodynamic_planner->getAvgOutDegree();
                entry.avg_deg_in = kinodynamic_planner->getAvgInDegree();
                entry.neighborhood_radius = kinodynamic_planner->getNeighborhoodRadius();
                log_data.push_back(entry);
            }

            // --- VISUALIZATION ---
            std::vector<Eigen::VectorXd> safe_cyl_pos, threat_cyl_pos; 
            std::vector<double> safe_cyl_radii, threat_cyl_radii;
            std::vector<std::tuple<Eigen::Vector2d, double, double, double>> safe_boxes, threat_boxes;
            std::vector<Eigen::Vector2d> safe_vel_pos, safe_vel_val;
            std::vector<Eigen::Vector2d> threat_vel_pos, threat_vel_val;

            for (const auto& obstacle : all_obs) {
                // (Visualization logic remains the same as your code...)
                bool is_threat = false; 
                if (obstacle.type == Obstacle::CIRCLE) {
                    Eigen::VectorXd pos(2); pos << obstacle.position.x(), obstacle.position.y();
                    if (is_threat) { threat_cyl_pos.push_back(pos); threat_cyl_radii.push_back(obstacle.dimensions.radius); }
                    else { safe_cyl_pos.push_back(pos); safe_cyl_radii.push_back(obstacle.dimensions.radius); }
                } else if (obstacle.type == Obstacle::BOX) {
                    auto box_tuple = std::make_tuple(obstacle.position, obstacle.dimensions.width, obstacle.dimensions.height, obstacle.dimensions.rotation);
                    if (is_threat) threat_boxes.push_back(box_tuple);
                    else safe_boxes.push_back(box_tuple);
                }
                if (obstacle.is_dynamic && obstacle.velocity.norm() > 0.01) {
                    Eigen::Vector2d vel_pos(obstacle.position.x(), obstacle.position.y());
                    if (is_threat) { threat_vel_pos.push_back(vel_pos); threat_vel_val.push_back(obstacle.velocity); }
                    else { safe_vel_pos.push_back(vel_pos); safe_vel_val.push_back(obstacle.velocity); }
                }
            }

            Eigen::Vector3d robot_pos(start_vec(0), start_vec(1), 0.0); 
            Eigen::VectorXd orientation_quat(4); orientation_quat << 0,0,0,1;
            std::vector<float> robot_color = {0.0f, 0.0f, 1.0f};
            std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> empty_trace;

            visualization->publishObstacleFrame(
                safe_cyl_pos, safe_cyl_radii, threat_cyl_pos, threat_cyl_radii,
                safe_boxes, threat_boxes, safe_vel_pos, safe_vel_val, threat_vel_pos, threat_vel_val,
                empty_trace, robot_pos, orientation_quat, robot_color, 0, "map"
            );

            auto new_executable_path = kinodynamic_planner->getPathPositions();
            kinodynamic_planner->visualizeTree();
            // kinodynamic_planner->visualizeTreeReal();

            auto loop_end_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration<double>(loop_end_time - loop_start_time);
            if (elapsed < loop_duration) {
                std::this_thread::sleep_for(loop_duration - elapsed);
            }
        }
    } else {
        // --- KINODYNAMIC MODE (R2T, Dubins, Thruster) ---
        RCLCPP_INFO(vis_node->get_logger(), "Starting Kinodynamic Planning Loop...");
        double time_accumulated_in_slice = 0.0;
        auto slice_start_time = std::chrono::steady_clock::now();
        
        while (g_running && rclcpp::ok()) {
            executor.spin_some();

            // RESET METRICS AT START OF SLICE
            kinodynamic_planner->resetMetrics();
            
            // TIMING VARIABLES FOR THIS SLICE
            double current_update_ms = 0.0;
            double current_plan_ms = 0.0;

            Eigen::VectorXd current_sim_state = ros_manager->getCurrentSimulatedState();
            if (current_sim_state.size() == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            kinodynamic_planner->setRobotState(current_sim_state);
            double T_robot = current_sim_state(current_sim_state.size()-1);
            double sim_time = cfg.time_budget - T_robot;
            
            // gazebo_checker->processLatestPoseInfo(sim_time);
            ObstacleVector turned_obs = gazebo_checker->checkAndRepairObstacles(T_robot);
            
            // sync live snapshot with new tubes
            if (!turned_obs.empty()) {
                gazebo_checker->processLatestPoseInfo(sim_time);  // rebuild vector with updated map (new predicted_path)
            }

            // 1. MEASURE UPDATE TIME
            if (!turned_obs.empty()) {
                auto t1 = std::chrono::steady_clock::now();
                kinodynamic_planner->updateObstacleSamples(turned_obs);
                auto t2 = std::chrono::steady_clock::now();
                current_update_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
                
                RCLCPP_INFO(rclcpp::get_logger("Planner_Timing"), 
                    "updateObstacleSamples: %.2f ms", current_update_ms);
            }
            
            // 2. MEASURE PLAN TIME
            if (is_anytime) {
                auto t1 = std::chrono::steady_clock::now();
                kinodynamic_planner->plan();
                auto t2 = std::chrono::steady_clock::now();
                current_plan_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
                
                // Logging plan time can be spammy, maybe only log if > 10ms
                if (!turned_obs.empty())
                    RCLCPP_INFO(rclcpp::get_logger("Planner_Timing"), "plan took: %.2f ms", current_plan_ms);
            }

            // 3. LOG SEPARATED METRICS
            if (!turned_obs.empty() || is_anytime) {
                LogEntry entry;
                const auto& metrics = kinodynamic_planner->getLastReplanMetrics();
                
                entry.elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - global_start).count();
                
                // SAVE SPLIT TIMES
                entry.update_ms = current_update_ms;
                entry.plan_ms = current_plan_ms;
                entry.total_latency_ms = current_update_ms + current_plan_ms;

                entry.obstacle_checks = metrics.obstacle_checks;
                entry.orphaned_nodes = metrics.orphaned_nodes;
                entry.path_cost = metrics.path_cost;
                entry.time_to_goal = T_robot;
                entry.rewire_neighbor_searches = metrics.rewire_neighbor_searches;
                entry.tree_size = kinodynamic_planner->getTreeSize();
                entry.avg_deg_out = kinodynamic_planner->getAvgOutDegree();
                entry.avg_deg_in = kinodynamic_planner->getAvgInDegree();
                entry.neighborhood_radius = kinodynamic_planner->getNeighborhoodRadius();
                log_data.push_back(entry);
            }

            std::vector<Obstacle> culprits = obstacle_checker->getAndClearCulprits();
            ros_manager->updateThreats(culprits);
            
            auto new_path = kinodynamic_planner->getPathPositions();
            if (!new_path.empty()) {
                ros_manager->setPath(new_path);
                kinodynamic_planner->visualizePath(new_path);
            }
            kinodynamic_planner->visualizeTree();
            // kinodynamic_planner->visualizeTreeReal();

            auto now = std::chrono::steady_clock::now();
            double dt_wall = std::chrono::duration<double>(now - slice_start_time).count();
            time_accumulated_in_slice += dt_wall;
            
            if (time_accumulated_in_slice >= cfg.slice_time) {
                ros_manager->stepSimulation(cfg.slice_time);
                time_accumulated_in_slice = 0.0;
                slice_start_time = std::chrono::steady_clock::now();
                
                Eigen::VectorXd updated_state = ros_manager->getCurrentSimulatedState();
                double dist_to_goal = (updated_state.head<2>() - goal_vec.head<2>()).norm();
                if (dist_to_goal < 3.0) {
                    RCLCPP_INFO(vis_node->get_logger(), "Goal Reached!");
                    g_running = false;
                }
            } else {
                double remaining = cfg.slice_time - time_accumulated_in_slice;
                if (remaining > 0.0) std::this_thread::sleep_for(std::chrono::duration<double>(remaining));
            }
        }
    }
    std::cout<<"FINAL TREE SIZE: "<<planner->getTreeSize()<<"\n";
    CALLGRIND_STOP_INSTRUMENTATION;

    // // --- 10. Save Metrics ---
    // int final_collision_count = (is_geometric_mode) ? 0 : ros_manager->getCollisionCount();
    // for (auto& entry : log_data) entry.collision_count = final_collision_count;
    
    // std::time_t now_time = std::time(nullptr);
    // std::tm* local_tm = std::localtime(&now_time);
    // char time_buf[80];
    // strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", local_tm);
    
    // std::string filename = "sim_" + cfg.name + "_" + time_buf + "_metrics.csv";
    // std::cout << "Writing metrics to: " << filename << std::endl;
    
    // std::ofstream out(filename);
    // if (!out.is_open()) {
    //     std::cerr << "Error: failed to open " << filename << std::endl;
    //     return 1;
    // }
    
    // out << "elapsed_s,duration_ms,time_to_goal,path_cost,obstacle_checks,rewire_neighbor_searches,orphaned_nodes,collision_count,tree_size,avg_degree,radius\n";

    // for (const auto& entry : log_data) {
    //     out << entry.elapsed_s << "," 
    //         << entry.duration_ms << "," 
    //         << entry.time_to_goal << ","
    //         << entry.path_cost << "," 
    //         << entry.obstacle_checks << "," 
    //         << entry.rewire_neighbor_searches << ","
    //         << entry.orphaned_nodes << "," 
    //         << entry.collision_count << ","
    //         << entry.tree_size << ","
    //         << entry.avg_degree << ","
    //         << entry.neighborhood_radius << "\n";
    // }
    // out.close();
    
// --- 10. Save Metrics ---
    int final_collision_count = (is_geometric_mode) ? 0 : ros_manager->getCollisionCount();
    for (auto& entry : log_data) entry.collision_count = final_collision_count;
    
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
    
    // UPDATED HEADER
    out << "elapsed_s,total_latency_ms,update_ms,plan_ms,time_to_goal,path_cost,obstacle_checks,rewire_neighbor_searches,orphaned_nodes,collision_count,tree_size,avg_deg_out,avg_deg_in,radius\n";

    for (const auto& entry : log_data) {
        out << entry.elapsed_s << "," 
            << entry.total_latency_ms << ","
            << entry.update_ms << ","
            << entry.plan_ms << ","
            << entry.time_to_goal << ","
            << entry.path_cost << "," 
            << entry.obstacle_checks << "," 
            << entry.rewire_neighbor_searches << ","
            << entry.orphaned_nodes << "," 
            << entry.collision_count << ","
            << entry.tree_size << ","
            << entry.avg_deg_out << ","
            << entry.avg_deg_in << ","
            << entry.neighborhood_radius << "\n";
    }
    out.close();


    rclcpp::shutdown();
    return 0;
}