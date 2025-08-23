// Copyright 2025 Soheil E.nia

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/state_space/min_snap_statespace.hpp" // For the Trajectory struct
#include "motion_planning/utils/params.hpp"
#include <Eigen/Dense>
#include <mutex>
#include <chrono>
#include <vector>
#include <atomic>


// Normalizes an angle to the range [-PI, PI].
inline double normalizeAngle(double angle);


// Computes the basis vector for a polynomial at a given derivative and normalized time tau.
static inline Eigen::RowVectorXd basis(int deriv, double tau, int num_coeffs) {
    Eigen::RowVectorXd r = Eigen::RowVectorXd::Zero(num_coeffs);
    for (int i = deriv; i < num_coeffs; ++i) {
        double coeff = 1.0;
        for (int k = 0; k < deriv; ++k) {
            coeff *= (i - k);
        }
        r(i) = coeff * std::pow(tau, i - deriv);
    }
    return r;
}


class MinSnapROS2Manager : public rclcpp::Node {
public:
    MinSnapROS2Manager(
        std::shared_ptr<ObstacleChecker> obstacle_checker,
        std::shared_ptr<RVizVisualization> visualizer,
        const Params& params)
        : Node("ros2_manager_min_snap",
               rclcpp::NodeOptions().parameter_overrides(
                   {rclcpp::Parameter("use_sim_time", params.getParam<bool>("use_sim_time", false))}
               )),
          obstacle_checker_(obstacle_checker),
          visualizer_(visualizer),
          is_path_set_(false),
          current_segment_idx_(-1) { // Initialize index to invalid
        
        current_pos_ = Eigen::VectorXd::Zero(5);
        previous_pos_ = Eigen::Vector3d::Zero();
        current_vel_ = Eigen::VectorXd::Zero(4);
        current_accel_ = Eigen::VectorXd::Zero(4);

        simulation_time_step_ = params.getParam<double>("simulation_time_step", -0.02);
        int sim_frequency_hz = params.getParam<int>("sim_frequency_hz", 50);
        int vis_frequency_hz = params.getParam<int>("vis_frequency_hz", 30);


        RCLCPP_INFO(this->get_logger(), "Initialized MinSnap ROS2Manager.");

        vis_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / vis_frequency_hz),
            std::bind(&MinSnapROS2Manager::visualizationLoop, this));

        if (params.getParam<bool>("follow_path", true)){
            sim_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000 / sim_frequency_hz),
                std::bind(&MinSnapROS2Manager::simulationLoop, this));
        }
    }

    MinSnapROS2Manager(
        std::shared_ptr<RVizVisualization> visualizer)
        : Node("ros2_manager_min_snap",
               rclcpp::NodeOptions().parameter_overrides(
                   {rclcpp::Parameter("use_sim_time", false)}
               )),
          obstacle_checker_(nullptr),
          visualizer_(visualizer),
          is_path_set_(false),
          current_segment_idx_(-1) { // Initialize index to invalid
        
        current_pos_ = Eigen::VectorXd::Zero(5);
        previous_pos_ = Eigen::Vector3d::Zero();
        current_vel_ = Eigen::VectorXd::Zero(4);
        current_accel_ = Eigen::VectorXd::Zero(4);

        simulation_time_step_ = -0.02;
        int sim_frequency_hz = 50;

        RCLCPP_INFO(this->get_logger(), "Initialized MinSnapROS2Manager (Simple Constructor).");

        sim_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / sim_frequency_hz),
            std::bind(&MinSnapROS2Manager::simulationLoop, this));
    }

    void setInitialState(const Eigen::VectorXd& state) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        current_pos_ = state;
        previous_pos_ = state.head<3>(); // Initialize previous position
        current_vel_.setZero();
        current_accel_.setZero();
        // This is the ONLY place the clock should be initialized.
        current_sim_time_ = state(4);
    }

    void setPlannedTrajectory(const std::vector<Trajectory>& path_segments) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (path_segments.empty()) {
            RCLCPP_WARN(this->get_logger(), "[SET_PATH] Rejected: Received an empty path vector.");
            is_path_set_ = false;
            planned_path_segments_.clear();
            return;
        }

        // Validate each segment before accepting the path
        for (size_t i = 0; i < path_segments.size(); ++i) {
            const auto& segment = path_segments[i];
            if (!segment.is_valid || segment.path_points.empty()) {
                RCLCPP_ERROR(this->get_logger(),
                            "[SET_PATH] Rejected: Path contains an invalid segment at index %zu.", i);
                return;
            }
        }
        
        // If all segments are valid, update the path
        planned_path_segments_ = path_segments;
        current_segment_idx_ = 0;
        
        // Only initialize the simulation clock if a path was NOT already set.
        // This is the stable approach that prevents the time-warp loop on every replan.
        if (!is_path_set_) {
            // Start time is the time of the FIRST point of the FIRST segment.
            const double new_path_start_time = planned_path_segments_.front().path_points.front()(4);
            current_sim_time_ = new_path_start_time;
            // RCLCPP_INFO(this->get_logger(), "First valid path received. Initializing sim time to: %.2f", current_sim_time_);
        }
        
        is_path_set_ = true;
    }

    Eigen::VectorXd getCurrentPosition() const {
        std::lock_guard<std::mutex> lock(path_mutex_);
        return current_pos_;
    }

    Eigen::VectorXd getCurrentVelocity() const {
        std::lock_guard<std::mutex> lock(path_mutex_);
        return current_vel_;
    }

    Eigen::VectorXd getCurrentAcceleration() const {
        std::lock_guard<std::mutex> lock(path_mutex_);
        return current_accel_;
    }

    int getCollisionCount() const {
        return collision_count_.load();
    }

private:
    std::shared_ptr<ObstacleChecker> obstacle_checker_;
    std::shared_ptr<RVizVisualization> visualizer_;
    rclcpp::TimerBase::SharedPtr sim_timer_;
    rclcpp::TimerBase::SharedPtr vis_timer_;
    mutable std::mutex path_mutex_;

    std::vector<Trajectory> planned_path_segments_;
    bool is_path_set_;
    int current_segment_idx_;

    Eigen::VectorXd current_pos_;
    Eigen::Vector3d previous_pos_;
    Eigen::VectorXd current_vel_;
    Eigen::VectorXd current_accel_;
    
    double current_sim_time_;
    double simulation_time_step_;
    std::atomic<int> collision_count_{0};
    bool is_in_collision_state_{false};


    void visualizationLoop() {
        if (!obstacle_checker_ || !visualizer_) return;
        
        auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker_);
        if (!gazebo_checker) return;

        gazebo_checker->processLatestPoseInfo();
        const ObstacleVector& all_obstacles = gazebo_checker->getObstaclePositions();
        
        // Prepare containers for visualization data
        std::vector<Eigen::VectorXd> sphere_positions_for_viz;
        std::vector<double> sphere_radii;
        std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, double>> box_data_for_viz;
        std::vector<Eigen::Vector3d> dynamic_obstacle_positions_3d;
        std::vector<Eigen::Vector2d> dynamic_obstacle_velocities_2d;

        for (const auto& obstacle : all_obstacles) {
            Eigen::Vector3d obs_pos_3d(obstacle.position.x(), obstacle.position.y(), obstacle.z);

            if (obstacle.type == Obstacle::CIRCLE) {
                sphere_positions_for_viz.push_back(obs_pos_3d);
                // CORRECTED: Using physical radius without inflation
                sphere_radii.push_back(obstacle.dimensions.radius);
            } else if (obstacle.type == Obstacle::BOX) {
                // CORRECTED: Using physical dimensions without inflation
                Eigen::Vector3d dims(
                    obstacle.dimensions.width,
                    obstacle.dimensions.height,
                    obstacle.dimensions.height // Assuming depth = height
                );
                box_data_for_viz.emplace_back(obs_pos_3d, dims, obstacle.dimensions.rotation);
            }
            
            if (obstacle.is_dynamic && obstacle.velocity.norm() > 0.01) {
                // Correctly using 3D position for velocity vectors
                dynamic_obstacle_positions_3d.push_back(obs_pos_3d);
                dynamic_obstacle_velocities_2d.push_back(obstacle.velocity);
            }
        }

        // Send data to the visualizer
        if (!sphere_positions_for_viz.empty()) {
            visualizer_->visualizeSpheres(sphere_positions_for_viz, sphere_radii, "map", {0.0f, 0.4f, 1.0f}, "sphere_obstacles");
        }
        if (!box_data_for_viz.empty()) {
            visualizer_->visualizeCube(box_data_for_viz, "map", {0.0f, 0.6f, 0.8f}, "box_obstacles");
        }
        if (!dynamic_obstacle_positions_3d.empty()) {
            visualizer_->visualizeVelocityVectors(dynamic_obstacle_positions_3d, dynamic_obstacle_velocities_2d, "map", {1.0f, 0.5f, 0.0f}, "velocity_vectors");
        }
    }



    void simulationLoop() {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (!is_path_set_ || planned_path_segments_.empty()) return;

        current_sim_time_ += simulation_time_step_;

        // This loop now correctly finds the current segment based on the non-reversed path points.
        while (current_segment_idx_ < (int)planned_path_segments_.size() - 1) {
            const auto& segment_for_check = planned_path_segments_[current_segment_idx_];
            // Parent/end time is at the BACK of the path_points vector
            double segment_parent_time = segment_for_check.path_points.back()(4);

            if (current_sim_time_ < segment_parent_time) {
                current_segment_idx_++;
            } else {
                break;
            }
        }

        const auto& final_segment = planned_path_segments_.back();
        double final_time = final_segment.path_points.back()(4);

        if (current_sim_time_ < final_time) {
             RCLCPP_INFO(this->get_logger(), "Simulation finished: Path complete.");
             is_path_set_ = false;
             current_pos_ = final_segment.path_points.back();
             visualizeRobot();
             return;
        }

        const auto& segment_to_execute = planned_path_segments_[current_segment_idx_];
        const double T = segment_to_execute.time_duration;
        // Segment start time is at the FRONT of the path_points vector
        const double segment_start_time = segment_to_execute.path_points.front()(4);

        // This tau calculation is now correct with the fixed start time.
        const double tau = (T > 1e-6) ? ((segment_start_time - current_sim_time_) / T) : 1.0;
        const double evaluation_tau = std::clamp(tau, 0.0, 1.0);

        const int num_coeffs = segment_to_execute.coeffs_per_axis[0].size();
        const int num_axes = 4;

        for (int axis = 0; axis < num_axes; ++axis) {
            const auto& coeffs = segment_to_execute.coeffs_per_axis[axis];
            current_pos_(axis) = (coeffs.transpose() * basis(0, evaluation_tau, num_coeffs).transpose())(0);
            current_vel_(axis) = (coeffs.transpose() * basis(1, evaluation_tau, num_coeffs).transpose())(0) / T;
            current_accel_(axis) = (coeffs.transpose() * basis(2, evaluation_tau, num_coeffs).transpose())(0) / (T * T);
        }
        current_pos_(3) = normalizeAngle(current_pos_(3));
        current_pos_(4) = current_sim_time_;

        ////////////////////////////////////
        // 3D collision check for counting.
        auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker_);
        if (gazebo_checker) {
            // Get the full 3D position and yaw from the current state.
            Eigen::Vector3d current_pos_3d = current_pos_.head<3>();
            double current_yaw = current_pos_(3);
            
            // Call the NEW, overloaded 3D collision check function.
            bool is_colliding_now = gazebo_checker->checkRobotCollision(current_pos_3d, current_yaw);

            if (is_colliding_now && !is_in_collision_state_) {
                collision_count_++;
                RCLCPP_FATAL(this->get_logger(), "COLLISION DETECTED! Total Failures: %d", collision_count_.load());
            }
            is_in_collision_state_ = is_colliding_now;
        }
        //////////////////////////////////////


        visualizeRobot();
        
        // Update previous_pos_ at the end of the loop to correctly calculate yaw for the next frame.
        previous_pos_ = current_pos_.head<3>();
    }

    void visualizeRobot() {
        Eigen::Vector3d pos_3d = current_pos_.head<3>();
        
        double yaw = 0.0;
        
        Eigen::Vector3d velocity_vector = pos_3d - previous_pos_;

        if (velocity_vector.head<2>().norm() > 0.001) {
            yaw = std::atan2(-velocity_vector.y(), -velocity_vector.x());
        } else {
            yaw = current_pos_(3);
        }
        
        Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        Eigen::VectorXd quat(4);
        quat << q.x(), q.y(), q.z(), q.w();
        
        visualizer_->visualizeQuadcopter(pos_3d, quat, "map", {0.8f, 0.1f, 0.8f}, "simulated_robot");
    }
};