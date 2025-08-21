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

// --- Utility Functions ---

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

// --- Class Definition ---

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

        RCLCPP_INFO(this->get_logger(), "Initialized MinSnap ROS2Manager.");

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

// In MinSnapROS2Manager class
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
    
    // [✓ FINAL FIX] Only initialize the simulation clock if a path was NOT already set.
    // This is the stable approach that prevents the time-warp loop on every replan.
    if (!is_path_set_) {
        // The time of the last point of the first segment is the start time.
        // NOTE: This assumes the path segments are ordered correctly (which they now are).
        const double new_path_start_time = planned_path_segments_.front().path_points.back()(4);
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

    void simulationLoop() {
        std::lock_guard<std::mutex> lock(path_mutex_);

        if (!is_path_set_ || planned_path_segments_.empty()) {
            return;
        }

        // --- DEBUG PRINT: Show time progression ---
        // std::cout << "\n--- [SIM_LOOP] Tick --- Current Sim Time (t_goal): " << current_sim_time_ << "s ---\n";

        // Advance the continuous simulation time
        current_sim_time_ += simulation_time_step_;

        // --- DEBUG PRINT: Show segment finding logic ---
        int initial_segment_idx = current_segment_idx_;
        // This loop fast-forwards to find the correct segment for the current time.
        while (current_segment_idx_ < (int)planned_path_segments_.size() - 1) {
            const Trajectory& segment_for_check = planned_path_segments_[current_segment_idx_];
            // The "parent time" is the time-to-goal of the node *after* the current segment
            double segment_parent_time = segment_for_check.path_points.front()(4);

            // std::cout << "[Segment Check] Comparing t_goal " << current_sim_time_
            //           << " with parent_time " << segment_parent_time
            //           << " for segment " << current_segment_idx_ << ".\n";

            if (current_sim_time_ < segment_parent_time) {
                current_segment_idx_++;
                // std::cout << "    ---> Time is less. Moving to next segment: " << current_segment_idx_ << "\n";
            } else {
                // std::cout << "    ---> Time is greater. Sticking with segment " << current_segment_idx_ << ".\n";
                break;
            }
        }
        if (initial_segment_idx != current_segment_idx_){
            //  std::cout << "[Segment Switch] Changed from segment " << initial_segment_idx << " to " << current_segment_idx_ << ".\n";
        }
        // --- END DEBUG ---

        const Trajectory& final_segment_in_path = planned_path_segments_.back();
        double final_path_end_time = final_segment_in_path.path_points.front()(4);

        if (current_sim_time_ < final_path_end_time) {
             RCLCPP_INFO(this->get_logger(), "Simulation finished: Path complete.");
             is_path_set_ = false;
             current_pos_ = final_segment_in_path.path_points.front();
             visualizeRobot();
             return;
        }

        const Trajectory& segment_to_execute = planned_path_segments_[current_segment_idx_];
        const double T = segment_to_execute.time_duration;
        const double segment_start_time = segment_to_execute.path_points.back()(4);

        // --- DEBUG PRINTS: Expose the tau calculation bug ---
        // This is the original, incorrect calculation
        const double original_tau = (T > 1e-6) ? ((current_sim_time_ - segment_start_time) / T) : 1.0;
        
        // This is the corrected calculation
        const double corrected_tau = (T > 1e-6) ? ((segment_start_time - current_sim_time_) / T) : 1.0;
        double clamped_tau = std::max(0.0, std::min(1.0, corrected_tau));
        const double evaluation_tau = clamped_tau;

        // std::cout << "[Tau Calc] Executing on segment " << current_segment_idx_ << " (Duration: " << T << "s, Start t_goal: " << segment_start_time << "s)\n";
        // std::cout << "    [FIX] Corrected Tau (Time progress): " << clamped_tau << "\n";
        // std::cout << "    >>> Using Evaluation Tau (Position on curve): " << evaluation_tau << "\n";


        const int num_coeffs = segment_to_execute.coeffs_per_axis[0].size();
        const int num_axes = 4;

        // Evaluate the polynomial for each axis to get the current state
        for (int axis = 0; axis < num_axes; ++axis) {
            const auto& coeffs = segment_to_execute.coeffs_per_axis[axis];
            current_pos_(axis) = (coeffs.transpose() * basis(0, evaluation_tau, num_coeffs).transpose())(0);
            current_vel_(axis) = (coeffs.transpose() * basis(1, evaluation_tau, num_coeffs).transpose())(0) / T;
            current_accel_(axis) = (coeffs.transpose() * basis(2, evaluation_tau, num_coeffs).transpose())(0) / (T * T);
        }
        current_pos_(3) = normalizeAngle(current_pos_(3));
        current_pos_(4) = current_sim_time_;

        // --- DEBUG PRINT: Show final calculated state ---
        // std::cout << "[State Update] New Position (x,y,t): " << current_pos_(0) << ", " << current_pos_(1) << ", " << current_pos_(4) << "\n";

        visualizeRobot();
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