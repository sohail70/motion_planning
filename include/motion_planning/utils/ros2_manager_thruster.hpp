// Copyright 2025 Soheil E.nia

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/ds/edge_info.hpp"
#include "motion_planning/utils/params.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <chrono>
#include <functional>
#include <algorithm>
#include <sstream>
#include <iomanip> // For std::fixed, std::setprecision

// Helper functions to extract spatial parts of a state vector
inline Eigen::VectorXd getSpatialPosition(const Eigen::VectorXd& full_state) {
    // Spatial dimensions are the first D elements, where total dim = 2*D + 1
    int D_spatial_dim = (full_state.size() - 1) / 2;
    return full_state.head(D_spatial_dim);
}
inline Eigen::VectorXd getSpatialVelocity(const Eigen::VectorXd& full_state) {
    // Velocity dimensions follow the spatial dimensions
    int D_spatial_dim = (full_state.size() - 1) / 2;
    return full_state.segment(D_spatial_dim, D_spatial_dim);
}

class ROS2Manager : public rclcpp::Node {
public:
    // Full constructor with obstacle checking
    ROS2Manager(
        std::shared_ptr<ObstacleChecker> obstacle_checker,
        std::shared_ptr<RVizVisualization> visualizer,
        const Params& params)
        : Node("ros2_manager_thruster",
               rclcpp::NodeOptions().parameter_overrides(
                   {rclcpp::Parameter("use_sim_time", params.getParam<bool>("use_sim_time", false))}
               )),
          obstacle_checker_(obstacle_checker),
          visualizer_(visualizer),
          is_path_set_(false) {
        
        int dim = params.getParam<int>("thruster_state_dimension", 5);
        current_interpolated_state_ = Eigen::VectorXd::Zero(dim);

        simulation_time_step_ = params.getParam<double>("simulation_time_step", -0.02);
        int sim_frequency_hz = params.getParam<int>("sim_frequency_hz", 50);
        int vis_frequency_hz = params.getParam<int>("vis_frequency_hz", 30);

        RCLCPP_INFO(this->get_logger(), "Initialized Thruster ROS2Manager (Full Constructor).");

        vis_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / vis_frequency_hz),
            std::bind(&ROS2Manager::visualizationLoop, this));

        if (params.getParam<bool>("follow_path")){
            sim_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000 / sim_frequency_hz),
                std::bind(&ROS2Manager::simulationLoop, this));
        }
    }

    // Simplified constructor (used in the test)
    ROS2Manager(
        std::shared_ptr<RVizVisualization> visualizer,
        const Params& params)
        : Node("ros2_manager_thruster_simple",
               rclcpp::NodeOptions().parameter_overrides(
                   {rclcpp::Parameter("use_sim_time", params.getParam<bool>("use_sim_time", false))}
               )),
          obstacle_checker_(nullptr), // No obstacle checker in this version
          visualizer_(visualizer),
          is_path_set_(false)
    {
        // This implementation was missing, causing the segfault.
        int dim = params.getParam<int>("thruster_state_dimension", 5);
        current_interpolated_state_ = Eigen::VectorXd::Zero(dim);
        simulation_time_step_ = params.getParam<double>("simulation_time_step", -0.02);
        int sim_frequency_hz = params.getParam<int>("sim_frequency_hz", 50);

        RCLCPP_INFO(this->get_logger(), "Initialized Thruster ROS2Manager (Simple Constructor).");

        // The simulation timer must be created.
        sim_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / sim_frequency_hz),
            std::bind(&ROS2Manager::simulationLoop, this));
    }


    void setInitialState(const Eigen::VectorXd& state) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        current_interpolated_state_ = state;
        // The last element of the state vector is the timestamp
        current_sim_time_ = state(state.size() - 1);
        robot_spatial_trace_.clear();
        RCLCPP_INFO(this->get_logger(), "Initial state set. Sim time starting at: %.2f", current_sim_time_);
    }

    // Accepts the planned trajectory.
    void setPlannedThrusterTrajectory(const std::vector<Eigen::VectorXd>& path) {
        std::lock_guard<std::mutex> lock(path_mutex_);

        if (path.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "[SET_PATH] Received invalid or empty trajectory. Path not set.");
            is_path_set_ = false;
            current_path_.clear();
            return;
        }

        current_path_ = path;

        if (!current_path_.empty()) {
            const int dim = current_path_.front().size();
            RCLCPP_INFO(this->get_logger(), "[SET_PATH] New path received. Points: %zu, Time Range: [%.4f -> %.4f]",
                current_path_.size(), current_path_.front()(dim-1), current_path_.back()(dim-1));
        }
        is_path_set_ = true;
    }

    Eigen::VectorXd getCurrentKinodynamicState() const {
        std::lock_guard<std::mutex> lock(path_mutex_);
        return current_interpolated_state_;
    }

    int getCollisionCount() const {
        return collision_count_.load();
    }


private:
    std::shared_ptr<ObstacleChecker> obstacle_checker_;
    std::shared_ptr<RVizVisualization> visualizer_;
    rclcpp::TimerBase::SharedPtr vis_timer_;
    rclcpp::TimerBase::SharedPtr sim_timer_;
    mutable std::mutex path_mutex_;

    std::vector<Eigen::VectorXd> current_path_;
    std::vector<Eigen::VectorXd> robot_spatial_trace_;
    double current_sim_time_;
    double simulation_time_step_; // Negative for backward-in-time simulation
    bool is_path_set_;
    Eigen::VectorXd current_interpolated_state_;

    std::atomic<int> collision_count_{0};
    bool is_in_collision_state_{false};


    void visualizationLoop() {
        if (!obstacle_checker_ || !visualizer_) return;
        
        auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker_);
        if (!gazebo_checker) return;

        gazebo_checker->processLatestPoseInfo();
        const ObstacleVector& all_obstacles = gazebo_checker->getObstaclePositions();
        
        // Prepare containers for visualization data
        std::vector<Eigen::VectorXd> cylinder_positions;
        std::vector<double> cylinder_radii;
        
        // Create the specific data structure your visualizeCube function needs
        std::vector<std::tuple<Eigen::Vector2d, double, double, double>> box_data_for_viz;

        std::vector<Eigen::Vector2d> dynamic_obstacle_positions;
        std::vector<Eigen::Vector2d> dynamic_obstacle_velocities;

        // Process each obstacle and sort it into the correct container
        for (const auto& obstacle : all_obstacles) {
            if (obstacle.type == Obstacle::CIRCLE) {
                Eigen::VectorXd pos(2);
                pos << obstacle.position.x(), obstacle.position.y();
                cylinder_positions.push_back(pos);
                cylinder_radii.push_back(obstacle.dimensions.radius);
            } else if (obstacle.type == Obstacle::BOX) {
                // Populate the vector of tuples directly
                box_data_for_viz.emplace_back(
                    obstacle.position,
                    obstacle.dimensions.width,
                    obstacle.dimensions.height,
                    obstacle.dimensions.rotation
                );
            }
            
            if (obstacle.is_dynamic && obstacle.velocity.norm() > 0.01) {
                dynamic_obstacle_positions.push_back(obstacle.position);
                dynamic_obstacle_velocities.push_back(obstacle.velocity);
            }
        }

        // Send data to the visualizer
        if (!cylinder_positions.empty()) {
            visualizer_->visualizeCylinder(cylinder_positions, cylinder_radii, "map", {0.0f, 0.4f, 1.0f}, "cylinder_obstacles");
        }
        if (!box_data_for_viz.empty()) {
            // Call your actual visualizeCube function with the correct data structure
            visualizer_->visualizeCube(box_data_for_viz, "map", {0.0f, 0.6f, 0.8f}, "box_obstacles");
        }
        if (!dynamic_obstacle_positions.empty()) {
            visualizer_->visualizeVelocityVectors(dynamic_obstacle_positions, dynamic_obstacle_velocities, "map", {1.0f, 0.5f, 0.0f}, "velocity_vectors");
        }
    }

    void simulationLoop() {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (!is_path_set_ || current_path_.size() < 2) {
            return;
        }

        current_sim_time_ += simulation_time_step_;

        const int dim = current_path_.front().size();
        const double path_end_time = current_path_.back()(dim - 1);

        // --- Finish Condition ---
        // Stop simulation if we have passed the final time point.
        if (current_sim_time_ < path_end_time) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Simulation finished. Final time reached.");
            is_path_set_ = false;
            return;
        }

        // --- Find Current Segment ---
        // Find the first point in the path that is "in the past" relative to current_sim_time_.
        // Because time is decreasing, we search for the first element LESS than current_sim_time_.
        size_t after_idx = -1;
        for (size_t i = 0; i < current_path_.size(); ++i) {
            if (current_path_[i](dim - 1) < current_sim_time_) {
                after_idx = i;
                break;
            }
        }

        // --- Interpolation ---
        if (after_idx != (size_t)-1 && after_idx > 0) {
            const auto& state_after = current_path_[after_idx];
            const auto& state_before = current_path_[after_idx - 1];

            double time_before = state_before(dim - 1);
            double time_after = state_after(dim - 1);
            double segment_duration = time_before - time_after;
            
            if (segment_duration <= 1e-9) {
                current_interpolated_state_ = state_after;
            } else {
                double time_into_segment = time_before - current_sim_time_;
                double interp_factor = std::clamp(time_into_segment / segment_duration, 0.0, 1.0);
                
                // Interpolate position and velocity
                Eigen::VectorXd new_state(dim);
                const int D_spatial_dim = (dim - 1) / 2;
                new_state.head(D_spatial_dim) = getSpatialPosition(state_before) + interp_factor * (getSpatialPosition(state_after) - getSpatialPosition(state_before));
                new_state.segment(D_spatial_dim, D_spatial_dim) = getSpatialVelocity(state_before) + interp_factor * (getSpatialVelocity(state_after) - getSpatialVelocity(state_before));
                new_state(dim - 1) = current_sim_time_;
                current_interpolated_state_ = new_state;
            }
        } else {
            // If sim time is outside path bounds, hold the start/end state.
            if (current_sim_time_ >= current_path_.front()(dim-1)) {
                current_interpolated_state_ = current_path_.front();
            } else {
                current_interpolated_state_ = current_path_.back();
            }
        }

        // COLLISION COUNTING LOGIC
        auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker_);
        if (gazebo_checker) {
            // --- Use the correct state variable ---
            Eigen::Vector2d current_pos = getSpatialPosition(current_interpolated_state_);

            // --- Calculate current yaw from the velocity vector ---
            Eigen::VectorXd current_vel = getSpatialVelocity(current_interpolated_state_);
            double current_yaw = 0.0; // Default yaw if stationary
            if (current_vel.norm() > 1e-3) {
                current_yaw = std::atan2(current_vel[1], current_vel[0]);
            }
            
            // Call the unified collision check function
            bool is_colliding_now = gazebo_checker->checkRobotCollision(current_pos, current_yaw);

            if (is_colliding_now && !is_in_collision_state_) {
                collision_count_++;
                RCLCPP_FATAL(this->get_logger(), "COLLISION DETECTED! Total Failures: %d", collision_count_.load());
            }
            is_in_collision_state_ = is_colliding_now;
        }

        
        // --- Visualization ---
        Eigen::VectorXd new_pos = getSpatialPosition(current_interpolated_state_);
        Eigen::VectorXd new_vel = getSpatialVelocity(current_interpolated_state_);
        
        // Determine robot orientation from velocity vector
        Eigen::VectorXd robot_orientation_quat(4);
        if (new_vel.norm() > 1e-3) {
            // The velocity vector in the path points opposite to the direction of travel
            // because we are simulating backward in time. Add PI to the yaw to flip it.
            double yaw = std::atan2(new_vel[1], new_vel[0]);
            // yaw += M_PI; // Add 180 degrees to point the arrow in the direction of motion.
            Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
            robot_orientation_quat << q.x(), q.y(), q.z(), q.w();
        } else {
            robot_orientation_quat << 0, 0, 0, 1; // Default orientation
        }
        
        visualizer_->visualizeRobotArrow(new_pos, robot_orientation_quat, "map", {0.8f, 0.1f, 0.8f}, "simulated_robot");
        
        // // Add to the robot's trace for visualization
        // if (robot_spatial_trace_.empty() || (robot_spatial_trace_.back() - new_pos).norm() > 0.1) {
        //      robot_spatial_trace_.push_back(new_pos);
        // }
        // visualizer_->visualizeTrajectories({robot_spatial_trace_}, "map", {1.0f, 0.5f, 0.0f}, "robot_trace");
    }
};