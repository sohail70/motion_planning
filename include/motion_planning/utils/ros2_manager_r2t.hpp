// // Copyright 2025 Soheil E.nia

// #pragma once

// #include "rclcpp/rclcpp.hpp"
// #include "motion_planning/utils/obstacle_checker.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"
// #include "motion_planning/utils/deterministic_obstacle_checker.hpp"
// #include "motion_planning/utils/params.hpp"
// #include <Eigen/Dense>
// #include <Eigen/Geometry>
// #include <mutex>
// #include <chrono>
// #include <vector>
// #include <algorithm>

// class R2TROS2Manager : public rclcpp::Node {
// public:
//     R2TROS2Manager(
//         std::shared_ptr<ObstacleChecker> obstacle_checker,
//         std::shared_ptr<RVizVisualization> visualizer,
//         const Params& params,
//         double robot_velocity,
//         const Eigen::VectorXd& initial_sim_state) // Pass in the initial state
//         : Node("r2t_ros_manager", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)})),
//           obstacle_checker_(obstacle_checker),
//           visualizer_(visualizer),
//           is_path_set_(false),
//           last_known_theta_(0.0),
//            robot_velocity_(robot_velocity) 
//     {
//         // --- Initialize the simulation state immediately in the constructor ---
//         if (initial_sim_state.size() != 3) {
//             throw std::runtime_error("R2TROS2Manager: Initial state must be 3D.");
//         }
//         current_interpolated_state_ = initial_sim_state;
        
//         // simulation_time_step_ = params.getParam<double>("sim_time_step", -0.04);
//         int sim_frequency_hz = params.getParam<int>("sim_frequency_hz", 50);
//         int vis_frequency_hz = params.getParam<int>("vis_frequency_hz", 30);
//         // Sim Speed - Frequency (Hz) * Time Step Size (s) 
//         simulation_time_step_ = -1.0 / static_cast<double>(sim_frequency_hz);
        
//         RCLCPP_INFO(this->get_logger(), "Initialized R2TRO2SManager.");

//         vis_timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(1000 / vis_frequency_hz),
//             std::bind(&R2TROS2Manager::visualizationLoop, this));

//         if (params.getParam<bool>("follow_path")){
//             sim_timer_ = this->create_wall_timer(
//                 std::chrono::milliseconds(1000 / sim_frequency_hz),
//                 std::bind(&R2TROS2Manager::simulationLoop, this));
//         }
//     }




//     // void setPath(const std::vector<Eigen::VectorXd>& new_path_from_main) {
//     //     std::lock_guard<std::mutex> lock(path_mutex_);
//     //     if (new_path_from_main.size() < 2) {
//     //         is_path_set_ = false;
//     //         return;
//     //     }
        
//     //     // --- The path from the planner is now trusted to be in the correct forward order. ---
//     //     // --- REMOVE the std::reverse call. ---
//     //     current_path_ = new_path_from_main;
        
//     //     // The simulation starts from the beginning of the new path.
//     //     current_sim_time_ = current_path_.front()(2);

//     //     if (!is_path_set_) {
//     //          robot_spatial_trace_.clear();
//     //     }
//     //     is_path_set_ = true;
//     // }

//     // void setPath(const std::vector<Eigen::VectorXd>& new_path_from_main) {
//     //     // std::lock_guard<std::mutex> lock(path_mutex_);

//     //     if (new_path_from_main.empty()) {
//     //         is_path_set_ = false;
//     //         current_path_.clear();
//     //         return;
//     //     }
        
//     //     // Always update the path to the latest one from the planner.
//     //     current_path_ = new_path_from_main;
        
//     //     // If this is the FIRST time a path is being set,
//     //     // we must initialize the simulation time to the start of that path.
//     //     if (!is_path_set_) {
//     //         // The first point in the path holds the starting state and time-to-go.
//     //         current_sim_time_ = current_path_.front()(2); 
//     //         robot_spatial_trace_.clear();
//     //     }
        
//     //     // Now, we can safely say a path is set. On subsequent calls, the block
//     //     // above will be skipped, preserving current_sim_time_ during replans.
//     //     is_path_set_ = true;
//     // }

//     void setPath(const std::vector<Eigen::VectorXd>& new_path_from_main) {
//         std::lock_guard<std::mutex> lock(path_mutex_);

//         if (new_path_from_main.size() < 2) {
//             is_path_set_ = false;
//             return;
//         }

//         if (!is_path_set_) {
//             // Initial path setup
//             current_path_ = new_path_from_main;
//             current_sim_time_ = current_path_.front()(2);
//             current_interpolated_state_ = current_path_.front();
//             is_path_set_ = true;
//         } else {
//             // --- THE STITCH (The "No-Jump" Fix) ---
//             // 1. Take the new path from the planner
//             std::vector<Eigen::VectorXd> stitched_path = new_path_from_main;

//             // 2. Overwrite the first waypoint with the ACTUAL current state of the robot.
//             // This bridges the gap caused by the 80ms planning latency.
//             stitched_path.front().head<2>() = current_interpolated_state_.head<2>();
            
//             // 3. Ensure the time-to-go for this point matches our current simulation clock.
//             stitched_path.front()(2) = current_sim_time_;

//             // 4. Update the path. The simulation now has a continuous line from 
//             // "Exactly where I am now" to "The first waypoint of the new plan."
//             current_path_ = stitched_path;
//         }
//     }    



//     Eigen::VectorXd getCurrentSimulatedState() {
//         // std::lock_guard<std::mutex> lock(path_mutex_);
//         // It's possible this is called before the first simulation tick,
//         // so ensure the state is initialized.
//         if (current_interpolated_state_.size() == 0) {
//             // Find the initial state from the end of the path (leaf node) if available
//             if (!current_path_.empty()) {
//                 current_interpolated_state_ = current_path_.back();
//             }
//         }
//         return current_interpolated_state_;
//     }

//     int getCollisionCount() const {
//         return collision_count_.load();
//     }

//     void updateThreats(const std::vector<Obstacle>& culprits) {
//         current_threat_names_.clear();
//         for (const auto& obs : culprits) {
//             current_threat_names_.insert(obs.name);
//         }
//     }


// private:
//     std::shared_ptr<ObstacleChecker> obstacle_checker_;
//     std::shared_ptr<RVizVisualization> visualizer_;
//     rclcpp::TimerBase::SharedPtr vis_timer_;
//     rclcpp::TimerBase::SharedPtr sim_timer_;
//     std::mutex path_mutex_;
//     std::vector<Eigen::VectorXd> current_path_;
//     std::vector<Eigen::Vector2d> robot_spatial_trace_;
//     double current_sim_time_;
//     double simulation_time_step_;
//     bool is_path_set_;
//     double last_known_theta_;
//     // NEW MEMBER VARIABLE: to store the latest interpolated state
//     Eigen::VectorXd current_interpolated_state_;
//     double robot_velocity_; 

//     std::atomic<int> collision_count_{0};
//     bool is_in_collision_state_{false};
//     std::set<std::string> current_threat_names_;

//     rclcpp::Time last_clock_time_;
//     double total_budget_;

//     // // Optimized version
//     void visualizationLoop() { if (!obstacle_checker_ || !visualizer_) return; auto gazebo_checker = std::dynamic_pointer_cast<DeterministicObstacleChecker>(obstacle_checker_); if (!gazebo_checker) return; gazebo_checker->processLatestPoseInfo(); const ObstacleVector& all_obstacles = gazebo_checker->getObstaclePositions(); std::vector<Eigen::VectorXd> safe_cyl_pos, threat_cyl_pos; std::vector<double> safe_cyl_radii, threat_cyl_radii;
//             std::vector<std::tuple<Eigen::Vector2d, double, double, double>> safe_boxes, threat_boxes;

//             std::vector<Eigen::Vector2d> safe_vel_pos, safe_vel_val;
//             std::vector<Eigen::Vector2d> threat_vel_pos, threat_vel_val;

//             for (const auto& obstacle : all_obstacles) {
//                 bool is_threat = current_threat_names_.count(obstacle.name);

//                 if (obstacle.type == Obstacle::CIRCLE) {
//                     Eigen::VectorXd pos(2);
//                     pos << obstacle.position.x(), obstacle.position.y();
                    
//                     if (is_threat) {
//                         threat_cyl_pos.push_back(pos);
//                         threat_cyl_radii.push_back(obstacle.dimensions.radius);
//                     } else {
//                         safe_cyl_pos.push_back(pos);
//                         safe_cyl_radii.push_back(obstacle.dimensions.radius);
//                     }
//                 } else if (obstacle.type == Obstacle::BOX) {
//                     auto box_tuple = std::make_tuple(
//                         obstacle.position,
//                         obstacle.dimensions.width,
//                         obstacle.dimensions.height,
//                         obstacle.dimensions.rotation
//                     );
                    
//                     if (is_threat) {
//                         threat_boxes.push_back(box_tuple);
//                     } else {
//                         safe_boxes.push_back(box_tuple);
//                     }
//                 }
                
//                 if (obstacle.is_dynamic && obstacle.velocity.norm() > 0.01) {
//                     if (is_threat) {
//                         threat_vel_pos.push_back(obstacle.position);
//                         threat_vel_val.push_back(obstacle.velocity);
//                     } else {
//                         safe_vel_pos.push_back(obstacle.position);
//                         safe_vel_val.push_back(obstacle.velocity);
//                     }
//                 }
//             }

//             visualizer_->publishObstacleFrame(
//                 safe_cyl_pos, safe_cyl_radii,
//                 threat_cyl_pos, threat_cyl_radii,
//                 safe_boxes, threat_boxes,
//                 safe_vel_pos, safe_vel_val,
//                 threat_vel_pos, threat_vel_val,
//                 "map"
//             );
//         }

// // void visualizationLoop() { 
// //         if (!obstacle_checker_ || !visualizer_) return; 
// //         auto gazebo_checker = std::dynamic_pointer_cast<DeterministicObstacleChecker>(obstacle_checker_); 
// //         if (!gazebo_checker) return; 

// //         gazebo_checker->processLatestPoseInfo(); 
// //         const ObstacleVector& all_obstacles = gazebo_checker->getObstaclePositions(); 
        
// //         // --- Containers for Standard Obstacles ---
// //         std::vector<Eigen::VectorXd> safe_cyl_pos, threat_cyl_pos; 
// //         std::vector<double> safe_cyl_radii, threat_cyl_radii;
// //         std::vector<std::tuple<Eigen::Vector2d, double, double, double>> safe_boxes, threat_boxes;

// //         // --- Containers for Prediction Arrows ---
// //         std::vector<Eigen::Vector2d> safe_vel_pos, safe_vel_val;
// //         std::vector<Eigen::Vector2d> threat_vel_pos, threat_vel_val;

// //         // Hardcode visualization horizon to 5.0s since member var is gone
// //         double vis_horizon = 5.0; 

// //         for (const auto& obstacle : all_obstacles) {
// //             bool is_threat = current_threat_names_.count(obstacle.name);

// //             // 1. Shapes
// //             if (obstacle.type == Obstacle::CIRCLE) {
// //                 Eigen::VectorXd pos(2); pos << obstacle.position.x(), obstacle.position.y();
// //                 if (is_threat) { threat_cyl_pos.push_back(pos); threat_cyl_radii.push_back(obstacle.dimensions.radius); }
// //                 else { safe_cyl_pos.push_back(pos); safe_cyl_radii.push_back(obstacle.dimensions.radius); }
// //             } else if (obstacle.type == Obstacle::BOX) {
// //                 auto box_tuple = std::make_tuple(obstacle.position, obstacle.dimensions.width, obstacle.dimensions.height, obstacle.dimensions.rotation);
// //                 if (is_threat) threat_boxes.push_back(box_tuple);
// //                 else safe_boxes.push_back(box_tuple);
// //             }
            
// //             // 2. Prediction Arrow (Start -> Finish)
// //             if (obstacle.is_dynamic && obstacle.has_ground_truth) {
                
// //                 // Get the FULL path from your smart function
// //                 std::vector<Eigen::Vector3d> raw_prediction = gazebo_checker->generatePrediction(obstacle, vis_horizon);

// //                 if (!raw_prediction.empty()) {
// //                     // Calculate vector from Start to the FINAL predicted point
// //                     Eigen::Vector2d start_pos = obstacle.position;
// //                     Eigen::Vector2d final_pos = raw_prediction.back().head<2>();
                    
// //                     Eigen::Vector2d total_displacement = final_pos - start_pos;

// //                     if (is_threat) {
// //                         threat_vel_pos.push_back(start_pos);
// //                         threat_vel_val.push_back(total_displacement);
// //                     } else {
// //                         safe_vel_pos.push_back(start_pos);
// //                         safe_vel_val.push_back(total_displacement);
// //                     }
// //                 }
// //             }
// //         }

// //         visualizer_->publishObstacleFrame(
// //             safe_cyl_pos, safe_cyl_radii,
// //             threat_cyl_pos, threat_cyl_radii,
// //             safe_boxes, threat_boxes,
// //             safe_vel_pos, safe_vel_val,      
// //             threat_vel_pos, threat_vel_val,  
// //             "map"
// //         );
// //     }


// // With speed logs, fixed Clock-Sync, AND COLLISION CHECKS!
// void simulationLoop() {
//     rclcpp::Time now_sim = this->get_clock()->now();

//     // --- RESET GUARD ---
//     if (last_clock_time_.nanoseconds() == 0 || now_sim < last_clock_time_) {
//         last_clock_time_ = now_sim;
//         // RCLCPP_INFO(this->get_logger(), "CLOCK ANCHORED/RESET: SimTime=%.2f", now_sim.seconds());
//         return; 
//     }

//     double dt_sim = (now_sim - last_clock_time_).seconds();
//     last_clock_time_ = now_sim;

//     if (dt_sim > 0) {
//         current_sim_time_ -= dt_sim;
//     }

//     std::lock_guard<std::mutex> lock(path_mutex_);
    
//     // Safety Clamps
//     if (!current_path_.empty() && current_sim_time_ > current_path_.front()(2)) {
//         current_sim_time_ = current_path_.front()(2);
//     }

//     if (!is_path_set_ || current_path_.size() < 2) {
//         return;
//     }

//     if (current_sim_time_ < current_path_.back()(2)) {
//         current_sim_time_ = current_path_.back()(2);
//     }
    
//     // Find Segment
//     auto it_after = std::lower_bound(current_path_.begin(), current_path_.end(), current_sim_time_,
//         [](const Eigen::VectorXd& point, double time) {
//             return point(2) > time;
//         });

//     if (it_after == current_path_.begin()) it_after++;
//     if (it_after == current_path_.end()) return;

//     auto it_before = std::prev(it_after);
//     const Eigen::VectorXd& state_before = *it_before;
//     const Eigen::VectorXd& state_after = *it_after;
    
//     double segment_duration = state_before(2) - state_after(2);

//     // --- INTERPOLATION ---
//     Eigen::VectorXd current_robot_state(3);
//     if (segment_duration <= 1e-9) {
//         current_robot_state = state_after;
//     } else {
//         double time_into_segment = state_before(2) - current_sim_time_;
//         double interp_factor = time_into_segment / segment_duration;
//         current_robot_state.head<2>() = state_before.head<2>() + interp_factor * (state_after.head<2>() - state_before.head<2>());
//         current_robot_state(2) = current_sim_time_;
//     }
    
//     current_interpolated_state_ = current_robot_state;

//     // Calculate Yaw for Visualization AND Collision
//     Eigen::Vector2d direction_vector = state_after.head<2>() - state_before.head<2>();
//     if (direction_vector.norm() > 1e-6) {
//         last_known_theta_ = atan2(direction_vector.y(), direction_vector.x());
//     }

//     // -------------------------------------------------------------------
//     // 7. [MISSING PART RESTORED] COLLISION COUNTING LOGIC
//     // -------------------------------------------------------------------
//     // We check if the interpolated state is actually valid against current obstacles
//     auto gazebo_checker = std::dynamic_pointer_cast<DeterministicObstacleChecker>(obstacle_checker_);
//     if (gazebo_checker) {
//         Eigen::Vector2d current_pos = current_robot_state.head<2>();
        
//         // This function checks the robot's footprint against the *currently known* // obstacle positions (updated by visualizationLoop or main loop)
//         bool is_colliding_now = gazebo_checker->checkRobotCollision(current_pos, last_known_theta_);

//         if (is_colliding_now && !is_in_collision_state_) {
//             collision_count_++;
//             RCLCPP_FATAL(this->get_logger(), "XXX COLLISION DETECTED XXX At T=%.2f | Total: %d", 
//                          current_sim_time_, collision_count_.load());
            
//             // Optional: You can visualize the collision point here if you want
//         }
//         is_in_collision_state_ = is_colliding_now;
//     }
//     // -------------------------------------------------------------------

//     // --- VISUALIZATION ---
//     Eigen::Vector3d robot_pos_3d(current_robot_state(0), current_robot_state(1), 0.0);
    
//     Eigen::Quaterniond q(Eigen::AngleAxisd(last_known_theta_, Eigen::Vector3d::UnitZ()));
//     Eigen::VectorXd orientation_quat(4);
//     orientation_quat << q.x(), q.y(), q.z(), q.w();
    
//     // Change color to RED if colliding, PURPLE if safe
//     std::vector<float> color = is_in_collision_state_ ? std::vector<float>{1.0f, 0.0f, 0.0f} : std::vector<float>{0.8f, 0.1f, 0.8f};
//     visualizer_->visualizeRobotArrow(robot_pos_3d, orientation_quat, "map", color, "simulated_robot");

//     if(robot_spatial_trace_.empty() || (robot_spatial_trace_.back() - robot_pos_3d.head<2>()).norm() > 0.1) {
//          robot_spatial_trace_.push_back(robot_pos_3d.head<2>());
//     }
    
//     static int trace_pub_throttle = 0;
//     if (++trace_pub_throttle % 10 == 0) {
//         visualizer_->visualizeTrajectories({robot_spatial_trace_}, "map", {1.0f, 1.0f, 0.0f}, "robot_trace");
//     }
// }




//     };

// Copyright 2025 Soheil E.nia
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "motion_planning/utils/obstacle_checker.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/deterministic_obstacle_checker.hpp"
#include "motion_planning/utils/params.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <chrono>
#include <vector>
#include <algorithm>
#include <utility> // for std::pair
#include "motion_planning/utils/ros2_manager_base.hpp"
class R2TROS2Manager : public ROS2ManagerBase {
public:
    R2TROS2Manager(
        std::shared_ptr<ObstacleChecker> obstacle_checker,
        std::shared_ptr<RVizVisualization> visualizer,
        const Params& params,
        const Eigen::VectorXd& initial_sim_state,
        double initial_budget_time) 
        // : Node("r2t_ros_manager", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)})),
        : ROS2ManagerBase("r2t_ros_manager", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)})),
          obstacle_checker_(obstacle_checker),
          visualizer_(visualizer),
          is_path_set_(false),
          last_known_theta_(0.0),
          initial_budget_time_(initial_budget_time)
    {
        inflation = params.getParam<double>("inflation");

        if (initial_sim_state.size() != 3) {
            throw std::runtime_error("R2TROS2Manager: Initial state must be 3D.");
        }
        current_interpolated_state_ = initial_sim_state;
        current_sim_time_ = initial_sim_state(2);

        int vis_frequency_hz = params.getParam<int>("vis_frequency_hz", 30);
        
        // Only start the visualization timer. 
        // Simulation is now driven manually by stepSimulation().
        vis_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / vis_frequency_hz),
            std::bind(&R2TROS2Manager::visualizationLoop, this));
            
        RCLCPP_INFO(this->get_logger(), "Initialized R2TRO2SManager (Manual Sim Mode).");
    }


    // In RosManager or similar
    void stepStationary(double dt) {
        // 1. Update the internal time tracker
        // If you track time explicitly:
        current_sim_time_ -= dt; 
        current_interpolated_state_(2) = current_sim_time_;

    }

    // --- PUBLIC API ---
    /*
        1. Repair/update graph for planning (happens in the main function)
        2. Advance simulation / robot interpolation
        3. Update obstacle positions to current sim time
        4. Crash test at robot’s current position

        so basically since we already have repaird the graph at a time we reach this function we should be safe to move forward in time
        and no collision should happen!
        This is assuming a valid anchor node is found in the update that just happened
    */
    void stepSimulation(double dt) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        
        // 1. Advance Time
        current_sim_time_ -= dt;

        // 2. Safety Clamps
        if (!current_path_.empty()) {
            if (current_sim_time_ > current_path_.front()(2)) {
                current_sim_time_ = current_path_.front()(2);
            }
            if (current_sim_time_ < current_path_.back()(2)) {
                current_sim_time_ = current_path_.back()(2);
            }
        }

        if (!is_path_set_ || current_path_.size() < 2) return;

        // 3. Find Segment & Interpolate
        auto it_after = std::lower_bound(current_path_.begin(), current_path_.end(), current_sim_time_,
            [](const Eigen::VectorXd& point, double time) {
                return point(2) > time;
            });

        if (it_after == current_path_.begin()) it_after++;
        if (it_after == current_path_.end()) return;
        
        auto it_before = std::prev(it_after);
        const Eigen::VectorXd& state_before = *it_before;
        const Eigen::VectorXd& state_after = *it_after;
        
        double segment_duration = state_before(2) - state_after(2);
        
        // Store previous state BEFORE updating current_interpolated_state_
        Eigen::VectorXd prev_state = current_interpolated_state_;

        // Interpolate
        if (segment_duration <= 1e-9) {
            current_interpolated_state_ = state_after;
        } else {
            double time_into_segment = state_before(2) - current_sim_time_;
            double interp_factor = time_into_segment / segment_duration;
            current_interpolated_state_.head<2>() = state_before.head<2>() + interp_factor * (state_after.head<2>() - state_before.head<2>());
            current_interpolated_state_(2) = current_sim_time_;
        }

        // 4. Calculate Orientation
        Eigen::Vector2d direction_vector = state_after.head<2>() - state_before.head<2>();
        if (direction_vector.norm() > 1e-6) {
            last_known_theta_ = atan2(direction_vector.y(), direction_vector.x());
        }

        auto gazebo_checker = std::dynamic_pointer_cast<DeterministicObstacleChecker>(obstacle_checker_);
        double current_planner_time = this->getCurrentSimTime(); 
        double current_sim_time = initial_budget_time_ - current_planner_time;
        gazebo_checker->processLatestPoseInfo(current_sim_time); 
        // 5. Collision Check
        if (gazebo_checker) {
            Eigen::Vector2d current_pos = current_interpolated_state_.head<2>();
            bool is_colliding_now = gazebo_checker->checkRobotCollision(current_pos, last_known_theta_);
            if (is_colliding_now && !is_in_collision_state_) {
                collision_count_++;

                // Collect additional info
                Eigen::Vector2d robot_pos = current_interpolated_state_.head<2>();
                double robot_theta = last_known_theta_;

                // Grab all threats nearby
                auto gazebo_checker = std::dynamic_pointer_cast<DeterministicObstacleChecker>(obstacle_checker_);
                std::vector<std::string> colliding_obstacles;
                if (gazebo_checker) {
                    const auto& all_obstacles = gazebo_checker->getObstaclePositions();
                    for (const auto& obs : all_obstacles) {
                        Eigen::Vector2d obs_pos(obs.position.x(), obs.position.y());
                        double dist = (robot_pos - obs_pos).norm();
                        if (dist < obs.dimensions.radius + inflation) { // optional small epsilon
                            colliding_obstacles.push_back(obs.name);
                        }
                    }
                }

                // Log the enriched crash info
                RCLCPP_FATAL(this->get_logger(),
                            "XXX COLLISION DETECTED XXX At T=%.2f | Total Collisions: %d\n"
                            "Robot Pos: [%.3f, %.3f] | Theta: %.3f rad\n"
                            "Colliding Obstacles: %s",
                            current_sim_time_,
                            collision_count_.load(),
                            robot_pos.x(), robot_pos.y(), robot_theta,
                            colliding_obstacles.empty() ? "NONE" : colliding_obstacles.front().c_str() // could join all names if you want
                );
            }
            is_in_collision_state_ = is_colliding_now;
        }

        // Create a 3D point for start and end (Z=0)
        Eigen::VectorXd start_pt(3); start_pt << prev_state(0), prev_state(1), 0.0; // <--- ADDED SEMICOLON HERE
        Eigen::VectorXd end_pt(3);   end_pt << current_interpolated_state_(0), current_interpolated_state_(1), 0.0;
        
        // Add to edge list
        robot_trace_edges_.push_back({start_pt, end_pt});

            // // 6. VISUALIZATION: Robot Arrow & Trace (EDGES)
        // if (visualizer_) {
        //     // A. Robot Arrow
        //     Eigen::Vector3d robot_pos_3d(current_interpolated_state_(0), current_interpolated_state_(1), 0.0);
        //     Eigen::Quaterniond q(Eigen::AngleAxisd(last_known_theta_, Eigen::Vector3d::UnitZ()));
        //     Eigen::VectorXd orientation_quat(4); orientation_quat << q.x(), q.y(), q.z(), q.w();
            
        //     std::vector<float> color = is_in_collision_state_ ? std::vector<float>{1.0f, 0.0f, 0.0f} : std::vector<float>{0.8f, 0.1f, 0.8f};
        //     visualizer_->visualizeRobotArrow(robot_pos_3d, orientation_quat, "map", color, "simulated_robot");

        //     // B. Robot Trace (Add Edges)
        //     // FIX: Remove the distance check. Always add the edge.
        //     // This ensures continuity even if the robot moves slowly.
            
        //     // B. Robot Trace (Add Edges)
        //     // FIX: Remove the distance check. Always add the edge.
            
        //     // Create a 3D point for start and end (Z=0)
        //     Eigen::VectorXd start_pt(3); start_pt << prev_state(0), prev_state(1), 0.0; // <--- ADDED SEMICOLON HERE
        //     Eigen::VectorXd end_pt(3);   end_pt << current_interpolated_state_(0), current_interpolated_state_(1), 0.0;
            
        //     // Add to edge list
        //     robot_trace_edges_.push_back({start_pt, end_pt});
            
        //     // C. Publish Trace (Throttled to every 10 steps)
        //     // Keep the throttle here to save network bandwidth, but the list is now complete.
        //     static int trace_pub_throttle = 0;
        //     if (++trace_pub_throttle % 2 == 0) {
        //         visualizer_->visualizeEdges(robot_trace_edges_, "map", "1.0,1.0,0.0", "robot_trace_edges");
        //     }
        // }
    }

    // void setPath(const std::vector<Eigen::VectorXd>& new_path_from_main) {
    //     std::lock_guard<std::mutex> lock(path_mutex_);
    //     if (new_path_from_main.size() < 2) {
    //         is_path_set_ = false;
    //         return;
    //     }
        
    //     if (!is_path_set_) {
    //         current_path_ = new_path_from_main;
    //         current_sim_time_ = current_path_.front()(2); 
    //         current_interpolated_state_ = current_path_.front();
    //         is_path_set_ = true;
    //     } else {
    //         std::vector<Eigen::VectorXd> stitched_path = new_path_from_main;
    //         stitched_path.front().head<2>() = current_interpolated_state_.head<2>();
    //         stitched_path.front()(2) = current_sim_time_;
    //         current_path_ = stitched_path;
    //     }
    // }    

    void setPath(const std::vector<Eigen::VectorXd>& new_path_from_main) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (new_path_from_main.size() < 2) {
            is_path_set_ = false;
            return;
        }
        
        if (!is_path_set_) {
            // First time setup
            current_path_ = new_path_from_main;
            // Sync manager time to the path's start time
            current_sim_time_ = current_path_.front()(2); 
            current_interpolated_state_ = current_path_.front();
            is_path_set_ = true;
        } else {
            // --- THE FIX ---
            // 1. Create a copy of the new path
            std::vector<Eigen::VectorXd> stitched_path = new_path_from_main;
            
            // 2. Stitch the POSITION (x, y) to where the robot actually is right now
            stitched_path.front().head<2>() = current_interpolated_state_.head<2>();
            
            // 3. CRITICAL: Do NOT overwrite the Time!
            // Keep the time from the planner (new_path_from_main).
            // This ensures the path is temporally consistent with the planner's graph.
            // stitched_path.front()(2) = current_sim_time_; // <--- REMOVE THIS LINE
            
            // 4. Update the path
            current_path_ = stitched_path;

            // 5. OPTIONAL: Sync Manager Time?
            // If the planner's time is significantly different from the manager's time,
            // it implies a jump. We should snap the manager to the path to prevent interpolation errors.
            // However, usually, we just let stepSimulation handle the progression.
            // If you want to be safe, you can uncomment the following, but it might cause "teleporting" if the planner is slow:
            // current_sim_time_ = current_path_.front()(2); 
        }
    }



    Eigen::VectorXd getCurrentSimulatedState() {
        std::lock_guard<std::mutex> lock(path_mutex_);
        return current_interpolated_state_;
    }

    int getCollisionCount() const { return collision_count_.load(); }

    void updateThreats(const std::vector<Obstacle>& culprits) {
        current_threat_names_.clear();
        for (const auto& obs : culprits) {
            current_threat_names_.insert(obs.name);
        }
    }
    double getCurrentSimTime() const {
        return current_sim_time_;
    }


    // only for geometric case!
    void setExternalSimTime(double t) {
        current_sim_time_ = t;
    }

private:
std::shared_ptr<ObstacleChecker> obstacle_checker_;
std::shared_ptr<RVizVisualization> visualizer_;
    rclcpp::TimerBase::SharedPtr vis_timer_;
    std::mutex path_mutex_;
    std::vector<Eigen::VectorXd> current_path_;
    
    // CHANGED: Store edges instead of points
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> robot_trace_edges_;
    
    double current_sim_time_;
    bool is_path_set_;
    double last_known_theta_;
    Eigen::VectorXd current_interpolated_state_;
    std::atomic<int> collision_count_{0};
    bool is_in_collision_state_{false};
    std::set<std::string> current_threat_names_;
    double initial_budget_time_;
    double inflation;

    // // Visualization Loop (Background Thread - Obstacles Only)
    // void visualizationLoop() { 
    //     if (!obstacle_checker_ || !visualizer_) return; 
    //     auto gazebo_checker = std::dynamic_pointer_cast<DeterministicObstacleChecker>(obstacle_checker_); 
    //     if (!gazebo_checker) return; 
        
    //     double current_planner_time = this->getCurrentSimTime(); // e.g. 18.0 (decreasing)
        
    //     // FIX: Convert to Forward Simulation Time
    //     // If Budget started at 20.0, and is now 18.0, Sim Time is 2.0
    //     double current_sim_time = initial_budget_time_ - current_planner_time;

    //     // Optional: Debug log to verify the conversion
    //     // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
    //     //     "Vis Loop | Planner T: %.2f | Sim T: %.2f", current_planner_time, current_sim_time);

    //     // NOW pass the CORRECT time (Forward Time)
    //     gazebo_checker->processLatestPoseInfo(current_sim_time); 
        
    //     const ObstacleVector& all_obstacles = gazebo_checker->getObstaclePositions(); 
        
    //     std::vector<Eigen::VectorXd> safe_cyl_pos, threat_cyl_pos; 
    //     std::vector<double> safe_cyl_radii, threat_cyl_radii;
    //     std::vector<std::tuple<Eigen::Vector2d, double, double, double>> safe_boxes, threat_boxes;
    //     std::vector<Eigen::Vector2d> safe_vel_pos, safe_vel_val;
    //     std::vector<Eigen::Vector2d> threat_vel_pos, threat_vel_val;
        
    //     for (const auto& obstacle : all_obstacles) {
    //         bool is_threat = current_threat_names_.count(obstacle.name);
    //         if (obstacle.type == Obstacle::CIRCLE) {
    //             Eigen::VectorXd pos(2); pos << obstacle.position.x(), obstacle.position.y();
    //             if (is_threat) { threat_cyl_pos.push_back(pos); threat_cyl_radii.push_back(obstacle.dimensions.radius); }
    //             else { safe_cyl_pos.push_back(pos); safe_cyl_radii.push_back(obstacle.dimensions.radius); }
    //         } else if (obstacle.type == Obstacle::BOX) {
    //             auto box_tuple = std::make_tuple(obstacle.position, obstacle.dimensions.width, obstacle.dimensions.height, obstacle.dimensions.rotation);
    //             if (is_threat) threat_boxes.push_back(box_tuple);
    //             else safe_boxes.push_back(box_tuple);
    //         }
    //         if (obstacle.is_dynamic && obstacle.velocity.norm() > 0.01) {
    //             if (is_threat) { threat_vel_pos.push_back(obstacle.position); threat_vel_val.push_back(obstacle.velocity); }
    //             else { safe_vel_pos.push_back(obstacle.position); safe_vel_val.push_back(obstacle.velocity); }
    //         }
    //     }
    //     visualizer_->publishObstacleFrame(
    //         safe_cyl_pos, safe_cyl_radii, threat_cyl_pos, threat_cyl_radii,
    //         safe_boxes, threat_boxes, safe_vel_pos, safe_vel_val, threat_vel_pos, threat_vel_val, "map"
    //     );
    // }

    void visualizationLoop() { 
        if (!obstacle_checker_ || !visualizer_) return; 
        
        // --- 1. PREPARE OBSTACLE DATA ---
        auto gazebo_checker = std::dynamic_pointer_cast<DeterministicObstacleChecker>(obstacle_checker_); 
        std::vector<Eigen::VectorXd> safe_cyl_pos, threat_cyl_pos; 
        std::vector<double> safe_cyl_radii, threat_cyl_radii;
        std::vector<std::tuple<Eigen::Vector2d, double, double, double>> safe_boxes, threat_boxes;
        std::vector<Eigen::Vector2d> safe_vel_pos, safe_vel_val;
        std::vector<Eigen::Vector2d> threat_vel_pos, threat_vel_val;

        if (gazebo_checker) {
            double current_planner_time = this->getCurrentSimTime(); 
            double current_sim_time = initial_budget_time_ - current_planner_time;
            gazebo_checker->processLatestPoseInfo(current_sim_time); 
            const ObstacleVector& all_obstacles = gazebo_checker->getObstaclePositions(); 
            
            for (const auto& obstacle : all_obstacles) {
                bool is_threat = current_threat_names_.count(obstacle.name);
                if (obstacle.type == Obstacle::CIRCLE) {
                    Eigen::VectorXd pos(2); pos << obstacle.position.x(), obstacle.position.y();
                    if (is_threat) { threat_cyl_pos.push_back(pos); threat_cyl_radii.push_back(obstacle.dimensions.radius); }
                    else { safe_cyl_pos.push_back(pos); safe_cyl_radii.push_back(obstacle.dimensions.radius); }
                } else if (obstacle.type == Obstacle::BOX) {
                    auto box_tuple = std::make_tuple(obstacle.position, obstacle.dimensions.width, obstacle.dimensions.height, obstacle.dimensions.rotation);
                    if (is_threat) threat_boxes.push_back(box_tuple);
                    else safe_boxes.push_back(box_tuple);
                }
                Eigen::Vector2d scaled_velocity = obstacle.velocity * 0.2;
                if (obstacle.is_dynamic && obstacle.velocity.norm() > 0.01) {
                    if (is_threat) { threat_vel_pos.push_back(obstacle.position); threat_vel_val.push_back(scaled_velocity); }
                    else { safe_vel_pos.push_back(obstacle.position); safe_vel_val.push_back(scaled_velocity); }
                }
            }
        }

        // --- 2. PREPARE ROBOT DATA (Trace & Arrow) ---
        std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> current_trace;
        Eigen::Vector3d robot_pos;
        Eigen::VectorXd orientation_quat(4);
        bool is_colliding = false;

        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            
            // Copy trace for thread safety
            current_trace = robot_trace_edges_;
            
            // Prepare Robot Arrow Data
            robot_pos << current_interpolated_state_(0), current_interpolated_state_(1), 0.0;
            Eigen::Quaterniond q(Eigen::AngleAxisd(last_known_theta_, Eigen::Vector3d::UnitZ()));
            orientation_quat << q.x(), q.y(), q.z(), q.w();
            is_colliding = is_in_collision_state_;
        }

        // --- 3. PUBLISH BATCH ---
        // Note: We still use visualizeRobotArrow separately because it's a single marker 
        // and easier to manage than adding it to the complex obstacle batch logic.
        // But the Trace is now batched with obstacles!
        
        // // Draw Arrow
        // std::vector<float> arrow_color = is_colliding ? std::vector<float>{1.0f, 0.0f, 0.0f} : std::vector<float>{0.8f, 0.1f, 0.8f};
        // visualizer_->visualizeRobotArrow(robot_pos, orientation_quat, "map", arrow_color, "simulated_robot");

        // --- 3. PUBLISH BATCH (UPDATED) ---
        std::vector<float> robot_color = is_colliding ? std::vector<float>{1.0f, 0.0f, 0.0f} : std::vector<float>{0.0f, 0.45f, 0.74f};
        
        // Pass robot data directly to the batch function
        visualizer_->publishObstacleFrame(
            safe_cyl_pos, safe_cyl_radii, threat_cyl_pos, threat_cyl_radii,
            safe_boxes, threat_boxes, safe_vel_pos, safe_vel_val, threat_vel_pos, threat_vel_val,
            current_trace, 
            robot_pos, orientation_quat, robot_color, inflation,
            "map"
        );
    }

};
//////////////////












// ////////////////////

// // Copyright 2025 Soheil E.nia
// #pragma once
// #include "rclcpp/rclcpp.hpp"
// #include "motion_planning/utils/obstacle_checker.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"
// #include "motion_planning/utils/deterministic_obstacle_checker.hpp"
// #include "motion_planning/utils/params.hpp"
// #include <Eigen/Dense>
// #include <Eigen/Geometry>
// #include <mutex>
// #include <chrono>
// #include <vector>
// #include <algorithm>
// #include <utility> 

// class R2TROS2Manager : public rclcpp::Node {
// public:
//     R2TROS2Manager(
//         std::shared_ptr<ObstacleChecker> obstacle_checker,
//         std::shared_ptr<RVizVisualization> visualizer,
//         const Params& params,
//         double robot_velocity,
//         const Eigen::VectorXd& initial_sim_state) 
//         : Node("r2t_ros_manager", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)})),
//           obstacle_checker_(obstacle_checker),
//           visualizer_(visualizer),
//           is_path_set_(false),
//           last_known_theta_(0.0),
//           robot_velocity_(robot_velocity)
//     {
//         if (initial_sim_state.size() != 3) {
//             throw std::runtime_error("R2TROS2Manager: Initial state must be 3D.");
//         }
//         current_interpolated_state_ = initial_sim_state;
//         current_sim_time_ = initial_sim_state(2);
        
//         // Store visualization frequency for manual throttling
//         vis_frequency_hz_ = params.getParam<int>("vis_frequency_hz", 30);
            
//         RCLCPP_INFO(this->get_logger(), "Initialized R2TRO2SManager (Manual Sim Mode).");
//     }

//     void stepSimulation(double dt) {
//         std::lock_guard<std::mutex> lock(path_mutex_);
        
//         // 1. Advance Time
//         current_sim_time_ -= dt;

//         // 2. Safety Clamps
//         if (!current_path_.empty()) {
//             if (current_sim_time_ > current_path_.front()(2)) {
//                 current_sim_time_ = current_path_.front()(2);
//             }
//             if (current_sim_time_ < current_path_.back()(2)) {
//                 current_sim_time_ = current_path_.back()(2);
//             }
//         }

//         if (!is_path_set_ || current_path_.size() < 2) return;

//         // 3. Find Segment & Interpolate
//         auto it_after = std::lower_bound(current_path_.begin(), current_path_.end(), current_sim_time_,
//             [](const Eigen::VectorXd& point, double time) {
//                 return point(2) > time;
//             });

//         if (it_after == current_path_.begin()) it_after++;
//         if (it_after == current_path_.end()) return;
        
//         auto it_before = std::prev(it_after);
//         const Eigen::VectorXd& state_before = *it_before;
//         const Eigen::VectorXd& state_after = *it_after;
        
//         double segment_duration = state_before(2) - state_after(2);
        
//         // Store previous state BEFORE updating current_interpolated_state_
//         Eigen::VectorXd prev_state = current_interpolated_state_;

//         // Interpolate
//         if (segment_duration <= 1e-9) {
//             current_interpolated_state_ = state_after;
//         } else {
//             double time_into_segment = state_before(2) - current_sim_time_;
//             double interp_factor = time_into_segment / segment_duration;
//             current_interpolated_state_.head<2>() = state_before.head<2>() + interp_factor * (state_after.head<2>() - state_before.head<2>());
//             current_interpolated_state_(2) = current_sim_time_;
//         }

//         // 4. Calculate Orientation
//         Eigen::Vector2d direction_vector = state_after.head<2>() - state_before.head<2>();
//         if (direction_vector.norm() > 1e-6) {
//             last_known_theta_ = atan2(direction_vector.y(), direction_vector.x());
//         }

//         // 5. Collision Check
//         auto gazebo_checker = std::dynamic_pointer_cast<DeterministicObstacleChecker>(obstacle_checker_);
//         if (gazebo_checker) {
//             Eigen::Vector2d current_pos = current_interpolated_state_.head<2>();
//             bool is_colliding_now = gazebo_checker->checkRobotCollision(current_pos, last_known_theta_);
//             if (is_colliding_now && !is_in_collision_state_) {
//                 collision_count_++;
//                 RCLCPP_FATAL(this->get_logger(), "XXX COLLISION DETECTED XXX At T=%.2f | Total: %d", 
//                              current_sim_time_, collision_count_.load());
//             }
//             is_in_collision_state_ = is_colliding_now;
//         }

//         // 6. VISUALIZATION: Robot Arrow & Trace (EDGES)
//         if (visualizer_) {
//             // A. Robot Arrow
//             Eigen::Vector3d robot_pos_3d(current_interpolated_state_(0), current_interpolated_state_(1), 0.0);
//             Eigen::Quaterniond q(Eigen::AngleAxisd(last_known_theta_, Eigen::Vector3d::UnitZ()));
//             Eigen::VectorXd orientation_quat(4); orientation_quat << q.x(), q.y(), q.z(), q.w();
            
//             std::vector<float> color = is_in_collision_state_ ? std::vector<float>{1.0f, 0.0f, 0.0f} : std::vector<float>{0.8f, 0.1f, 0.8f};
//             visualizer_->visualizeRobotArrow(robot_pos_3d, orientation_quat, "map", color, "simulated_robot");

//             // B. Robot Trace (Add Edges)
//             double dist = (prev_state.head<2>() - current_interpolated_state_.head<2>()).norm();
//             if (dist > 0.1) {
//                 Eigen::VectorXd start_pt(3); start_pt << prev_state(0), prev_state(1), 0.0;
//                 Eigen::VectorXd end_pt(3);   end_pt << current_interpolated_state_(0), current_interpolated_state_(1), 0.0;
//                 robot_trace_edges_.push_back({start_pt, end_pt});
//             }
            
//             // C. Publish Trace (Throttled to every 10 steps)
//             static int trace_pub_throttle = 0;
//             if (++trace_pub_throttle % 10 == 0) {
//                 visualizer_->visualizeEdges(robot_trace_edges_, "map", "1.0,1.0,0.0", "robot_trace_edges");
//             }
//         }
        
//         // 7. CALL VISUALIZATION LOOP (MANUAL THROTTLE INSIDE)
//         visualizationLoop();
//     }

//     void setPath(const std::vector<Eigen::VectorXd>& new_path_from_main) {
//         std::lock_guard<std::mutex> lock(path_mutex_);
//         if (new_path_from_main.size() < 2) {
//             is_path_set_ = false;
//             return;
//         }
        
//         if (!is_path_set_) {
//             current_path_ = new_path_from_main;
//             current_sim_time_ = current_path_.front()(2); 
//             current_interpolated_state_ = current_path_.front();
//             is_path_set_ = true;
//         } else {
//             std::vector<Eigen::VectorXd> stitched_path = new_path_from_main;
//             stitched_path.front().head<2>() = current_interpolated_state_.head<2>();
//             stitched_path.front()(2) = current_sim_time_;
//             current_path_ = stitched_path;
//         }
//     }    

//     Eigen::VectorXd getCurrentSimulatedState() {
//         return current_interpolated_state_;
//     }
    
//     int getCollisionCount() const { return collision_count_.load(); }
    
//     void updateThreats(const std::vector<Obstacle>& culprits) {
//         current_threat_names_.clear();
//         for (const auto& obs : culprits) {
//             current_threat_names_.insert(obs.name);
//         }
//     }

// private:
//     std::shared_ptr<ObstacleChecker> obstacle_checker_;
//     std::shared_ptr<RVizVisualization> visualizer_;
//     std::mutex path_mutex_;
//     std::vector<Eigen::VectorXd> current_path_;
//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> robot_trace_edges_;
//     double current_sim_time_;
//     bool is_path_set_;
//     double last_known_theta_;
//     Eigen::VectorXd current_interpolated_state_;
//     double robot_velocity_; 
//     std::atomic<int> collision_count_{0};
//     bool is_in_collision_state_{false};
//     std::set<std::string> current_threat_names_;
    
//     // NEW: Store frequency for manual throttling
//     int vis_frequency_hz_;

//     // Visualization Loop (Now called manually)
//     void visualizationLoop() { 
//         // --- MANUAL THROTTLE ---
//         // Only run the heavy obstacle visualization logic at the specified frequency
//         static auto last_vis_time = std::chrono::steady_clock::now();
//         auto now = std::chrono::steady_clock::now();
//         double time_since_last_vis = std::chrono::duration<double>(now - last_vis_time).count();
        
//         if (time_since_last_vis < (1.0 / vis_frequency_hz_)) {
//             return; // Skip this frame to prevent flooding RViz
//         }
//         last_vis_time = now;
//         // ------------------------

//         if (!obstacle_checker_ || !visualizer_) return; 
//         auto gazebo_checker = std::dynamic_pointer_cast<DeterministicObstacleChecker>(obstacle_checker_); 
//         if (!gazebo_checker) return; 
        
//         gazebo_checker->processLatestPoseInfo(); 
//         const ObstacleVector& all_obstacles = gazebo_checker->getObstaclePositions(); 
        
//         std::vector<Eigen::VectorXd> safe_cyl_pos, threat_cyl_pos; 
//         std::vector<double> safe_cyl_radii, threat_cyl_radii;
//         std::vector<std::tuple<Eigen::Vector2d, double, double, double>> safe_boxes, threat_boxes;
//         std::vector<Eigen::Vector2d> safe_vel_pos, safe_vel_val;
//         std::vector<Eigen::Vector2d> threat_vel_pos, threat_vel_val;
        
//         for (const auto& obstacle : all_obstacles) {
//             bool is_threat = current_threat_names_.count(obstacle.name);
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
//                 if (is_threat) { threat_vel_pos.push_back(obstacle.position); threat_vel_val.push_back(obstacle.velocity); }
//                 else { safe_vel_pos.push_back(obstacle.position); safe_vel_val.push_back(obstacle.velocity); }
//             }
//         }
//         visualizer_->publishObstacleFrame(
//             safe_cyl_pos, safe_cyl_radii, threat_cyl_pos, threat_cyl_radii,
//             safe_boxes, threat_boxes, safe_vel_pos, safe_vel_val, threat_vel_pos, threat_vel_val, "map"
//         );
//     }
// };