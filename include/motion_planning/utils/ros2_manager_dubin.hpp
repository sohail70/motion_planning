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

class DubinsROS2Manager : public ROS2ManagerBase {
public:
    DubinsROS2Manager(
        std::shared_ptr<ObstacleChecker> obstacle_checker,
        std::shared_ptr<RVizVisualization> visualizer,
        const Params& params,
        const Eigen::VectorXd& initial_sim_state,
        double initial_budget_time) 
        // : Node("dubins_ros2_manager", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)})),
        : ROS2ManagerBase("dubins_ros2_manager", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)})),
          obstacle_checker_(obstacle_checker),
          visualizer_(visualizer),
          is_path_set_(false),
          initial_budget_time_(initial_budget_time)
    {
        inflation = params.getParam<double>("inflation");
        if (initial_sim_state.size() != 4) {
            throw std::runtime_error("DubinsROS2Manager: Initial state must be 4D (x, y, theta, t).");
        }
        current_interpolated_state_ = initial_sim_state;
        current_sim_time_ = initial_sim_state(3); // Time is at index 3 for Dubins
        
        int vis_frequency_hz = params.getParam<int>("vis_frequency_hz", 30);
        
        // Only start the visualization timer. 
        // Simulation is now driven manually by stepSimulation().
        vis_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / vis_frequency_hz),
            std::bind(&DubinsROS2Manager::visualizationLoop, this));
            
        RCLCPP_INFO(this->get_logger(), "Initialized DubinsROS2Manager (Manual Sim Mode).");
    }

    void stepStationary(double dt) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        current_sim_time_ -= dt; 
        current_interpolated_state_(3) = current_sim_time_;
    }

    // --- PUBLIC API ---
    void stepSimulation(double dt) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        
        // 1. Advance Time
        current_sim_time_ -= dt;
        
        // 2. Safety Clamps
        if (!current_path_.empty()) {
            // For Dubins, time is at index 3
            if (current_sim_time_ > current_path_.front()(3)) {
                current_sim_time_ = current_path_.front()(3);
            }
            if (current_sim_time_ < current_path_.back()(3)) {
                current_sim_time_ = current_path_.back()(3);
            }
        }

        if (!is_path_set_ || current_path_.size() < 2) return;

        // 3. Find Segment & Interpolate
        auto it_after = std::lower_bound(current_path_.begin(), current_path_.end(), current_sim_time_,
            [](const Eigen::VectorXd& point, double time) {
                return point(3) > time; // Compare using index 3
            });

        if (it_after == current_path_.begin()) it_after++;
        if (it_after == current_path_.end()) return;
        
        auto it_before = std::prev(it_after);
        const Eigen::VectorXd& state_before = *it_before;
        const Eigen::VectorXd& state_after = *it_after;
        
        double segment_duration = state_before(3) - state_after(3);
        
        // Store previous state BEFORE updating current_interpolated_state_
        Eigen::VectorXd prev_state = current_interpolated_state_;

        // Interpolate
        if (segment_duration <= 1e-9) {
            current_interpolated_state_ = state_after;
        } else {
            double time_into_segment = state_before(3) - current_sim_time_;
            double interp_factor = time_into_segment / segment_duration;
            
            // Position Interpolation
            current_interpolated_state_.head<2>() = state_before.head<2>() + interp_factor * (state_after.head<2>() - state_before.head<2>());
            
            // --- DUBINS SPECIFIC: Angle Interpolation ---
            double theta_before = state_before(2);
            double theta_after = state_after(2);
            double angle_diff = normalizeAngle(theta_after - theta_before);
            current_interpolated_state_(2) = normalizeAngle(theta_before + interp_factor * angle_diff);
            
            // Time Update
            current_interpolated_state_(3) = current_sim_time_;
        }

        // 4. Collision Check
        auto gazebo_checker = std::dynamic_pointer_cast<DeterministicObstacleChecker>(obstacle_checker_);
        if (gazebo_checker) {
            double current_planner_time = this->getCurrentSimTime(); 
            double current_sim_time = initial_budget_time_ - current_planner_time;
            gazebo_checker->processLatestPoseInfo(current_sim_time); 

            Eigen::Vector2d current_pos = current_interpolated_state_.head<2>();
            double current_yaw = current_interpolated_state_(2); // Get yaw from state
            
            bool is_colliding_now = gazebo_checker->checkRobotCollision(current_pos, current_yaw);
            
            if (is_colliding_now && !is_in_collision_state_) {
                collision_count_++;
                // Collect additional info
                std::vector<std::string> colliding_obstacles;
                const auto& all_obstacles = gazebo_checker->getObstaclePositions();
                for (const auto& obs : all_obstacles) {
                    Eigen::Vector2d obs_pos(obs.position.x(), obs.position.y());
                    double dist = (current_pos - obs_pos).norm();
                    if (dist < obs.dimensions.radius + inflation) {
                        colliding_obstacles.push_back(obs.name);
                    }
                }
                RCLCPP_FATAL(this->get_logger(),
                            "XXX COLLISION DETECTED XXX At T=%.2f | Total Collisions: %d\n"
                            "Robot Pos: [%.3f, %.3f] | Theta: %.3f rad\n"
                            "Colliding Obstacles: %s",
                            current_sim_time_,
                            collision_count_.load(),
                            current_pos.x(), current_pos.y(), current_yaw,
                            colliding_obstacles.empty() ? "NONE" : colliding_obstacles.front().c_str()
                );
            }
            is_in_collision_state_ = is_colliding_now;
        }

        // 5. Update Trace (Edges)
        // Create a 3D point for start and end (Z=0)
        Eigen::VectorXd start_pt(3); start_pt << prev_state(0), prev_state(1), 0.0;
        Eigen::VectorXd end_pt(3);   end_pt << current_interpolated_state_(0), current_interpolated_state_(1), 0.0;
        robot_trace_edges_.push_back({start_pt, end_pt});
    }

    void setPath(const std::vector<Eigen::VectorXd>& new_path_from_main) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (new_path_from_main.size() < 2) {
            is_path_set_ = false;
            return;
        }
        
        if (!is_path_set_) {
            // First time setup
            current_path_ = new_path_from_main;
            // Sync manager time to the path's start time (Index 3)
            current_sim_time_ = current_path_.front()(3); 
            current_interpolated_state_ = current_path_.front();
            is_path_set_ = true;
        } else {
            // --- THE FIX ---
            // 1. Create a copy of the new path
            std::vector<Eigen::VectorXd> stitched_path = new_path_from_main;
            
            // 2. Stitch the POSITION (x, y) to where the robot actually is right now
            stitched_path.front().head<2>() = current_interpolated_state_.head<2>();
            
            // 3. Stitch the THETA to where the robot actually is right now
            stitched_path.front()(2) = current_interpolated_state_(2);
            
            // 4. CRITICAL: Do NOT overwrite the Time!
            // Keep the time from the planner (new_path_from_main).
            // stitched_path.front()(3) = current_sim_time_; // <--- REMOVE THIS LINE
            
            // 5. Update the path
            current_path_ = stitched_path;
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
    Eigen::VectorXd current_interpolated_state_;
    std::atomic<int> collision_count_{0};
    bool is_in_collision_state_{false};
    std::set<std::string> current_threat_names_;
    double initial_budget_time_;
    double inflation;

    double normalizeAngle(double angle) {
        angle = fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0.0) angle += 2.0 * M_PI;
        return angle - M_PI;
    }

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
            // Use the interpolated theta from the state vector
            Eigen::Quaterniond q(Eigen::AngleAxisd(current_interpolated_state_(2), Eigen::Vector3d::UnitZ()));
            orientation_quat << q.x(), q.y(), q.z(), q.w();
            is_colliding = is_in_collision_state_;
        }

        // --- 3. PUBLISH BATCH ---
        std::vector<float> robot_color = is_colliding ? std::vector<float>{1.0f, 0.0f, 0.0f} : std::vector<float>{0.0f, 0.45f, 0.74f};
        
        visualizer_->publishObstacleFrame(
            safe_cyl_pos, safe_cyl_radii, threat_cyl_pos, threat_cyl_radii,
            safe_boxes, threat_boxes, safe_vel_pos, safe_vel_val, threat_vel_pos, threat_vel_val,
            current_trace, 
            robot_pos, orientation_quat, robot_color, inflation, 
            "map"
        );
    }
};