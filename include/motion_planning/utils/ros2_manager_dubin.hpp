#pragma once

#include "rclcpp/rclcpp.hpp"
#include "motion_planning/utils/obstacle_checker.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/utils/params.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <chrono>
#include <vector>
#include <algorithm>

class DubinsROS2Manager : public rclcpp::Node {
public:
    /**
     * @brief Constructor for full replanning simulations.
     */
    DubinsROS2Manager(
        std::shared_ptr<ObstacleChecker> obstacle_checker,
        std::shared_ptr<RVizVisualization> visualizer,
        const Params& params,
        const Eigen::VectorXd& initial_sim_state)
        : Node("dubins_ros2_manager", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)})),
          obstacle_checker_(obstacle_checker),
          visualizer_(visualizer),
          is_path_set_(false)
    {
        if (initial_sim_state.size() != 4) {
            throw std::runtime_error("DubinsROS2Manager: Initial state must be 4D.");
        }
        current_interpolated_state_ = initial_sim_state;



        simulation_time_step_ = params.getParam<double>("sim_time_step", -0.02);
        int sim_frequency_hz = params.getParam<int>("sim_frequency_hz", 50);
        int vis_frequency_hz = params.getParam<int>("vis_frequency_hz", 30);

        RCLCPP_INFO(this->get_logger(), "Initialized DubinsROS2Manager (Full Constructor).");

        vis_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / vis_frequency_hz),
            std::bind(&DubinsROS2Manager::visualizationLoop, this));

        if (params.getParam<bool>("follow_path")){
            sim_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000 / sim_frequency_hz),
                std::bind(&DubinsROS2Manager::simulationLoop, this));
        }
    }

    /**
     * @brief Simplified constructor for basic visualization tests.
     */
    DubinsROS2Manager(std::shared_ptr<RVizVisualization> visualizer)
        : Node("dubins_ros2_manager", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)})),
          obstacle_checker_(nullptr), // No obstacle checker in simple tests
          visualizer_(visualizer),
          is_path_set_(false),
          simulation_time_step_(-0.02) // Use a default time step
    {
        RCLCPP_INFO(this->get_logger(), "Initialized DubinsROS2Manager (Simple Constructor).");

        sim_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50Hz
            std::bind(&DubinsROS2Manager::simulationLoop, this));

        current_interpolated_state_.resize(4);
        current_interpolated_state_.setZero();
    }


    // // Main method for setting the path
    // void setPath(const std::vector<Eigen::VectorXd>& new_path_from_main) {
    //     std::lock_guard<std::mutex> lock(path_mutex_);

    //     if (new_path_from_main.empty()) {
    //         is_path_set_ = false;
    //         current_path_.clear();
    //         return;
    //     }
    //     current_path_ = new_path_from_main;
    //     if (!is_path_set_) {
    //         current_sim_time_ = current_path_.front()(3);
    //         robot_spatial_trace_.clear();
    //     }
    //     is_path_set_ = true;
    // }
void setPath(const std::vector<Eigen::VectorXd>& new_path_from_main) {
    std::lock_guard<std::mutex> lock(path_mutex_);

    if (new_path_from_main.empty()) {
        is_path_set_ = false;
        current_path_.clear();
        return;
    }
    
    // Update to the new path
    current_path_ = new_path_from_main;
    
    // --- THE FIX ---
    // ALWAYS re-synchronize the simulation time to the start of the new path.
    // This ensures the simulation starts executing the new trajectory from its beginning.

    // current_sim_time_ = current_path_.front()(3); // The last element is time
    current_sim_time_ = current_path_.front()(current_path_.front().size() - 1);

    
    // It's also good practice to clear the visual trace when the path changes dramatically.
    robot_spatial_trace_.clear();
    
    is_path_set_ = true;
}




    Eigen::VectorXd getCurrentSimulatedState() {
        std::lock_guard<std::mutex> lock(path_mutex_);
        return current_interpolated_state_;
    }

    // (Legacy Support)
    void setPlannedDubinsPath(const std::vector<Eigen::VectorXd>& path) {
        setPath(path);
    }

    // (Legacy Support)
    void setInitialState(const Eigen::VectorXd& state) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (state.size() != 4) return;
        current_interpolated_state_ = state;
        current_sim_time_ = state(3);
        robot_spatial_trace_.clear();
        robot_spatial_trace_.push_back(state.head<3>());
    }



    
private:
    // Member variables...
    std::shared_ptr<ObstacleChecker> obstacle_checker_;
    std::shared_ptr<RVizVisualization> visualizer_;
    rclcpp::TimerBase::SharedPtr vis_timer_;
    rclcpp::TimerBase::SharedPtr sim_timer_;
    std::mutex path_mutex_;
    std::vector<Eigen::VectorXd> current_path_;
    std::vector<Eigen::Vector3d> robot_spatial_trace_;
    double current_sim_time_;
    double simulation_time_step_;
    bool is_path_set_;
    Eigen::VectorXd current_interpolated_state_;

    // Methods...
    double normalizeAngle(double angle) {
        angle = fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0.0) angle += 2.0 * M_PI;
        return angle - M_PI;
    }

    void visualizationLoop() {
        // This loop is only needed for the full manager, but must exist.
        if (!obstacle_checker_ || !visualizer_) return;
        
        // Attempt to cast to the Gazebo-specific checker to get obstacle details
        auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker_);
        if (!gazebo_checker) return;

        gazebo_checker->processLatestPoseInfo();
        // Get the latest obstacle information
        const ObstacleVector& all_obstacles = gazebo_checker->getObstaclePositions();
        
        // Prepare containers for visualization data
        std::vector<Eigen::VectorXd> cylinder_obstacles;
        std::vector<double> cylinder_radii;
        std::vector<Eigen::Vector2d> dynamic_obstacle_positions;
        std::vector<Eigen::Vector2d> dynamic_obstacle_velocities;

        // Process each obstacle to sort it for visualization
        for (const auto& obstacle : all_obstacles) {
            if (obstacle.type == Obstacle::CIRCLE) {
                Eigen::VectorXd vec(2);
                vec << obstacle.position.x(), obstacle.position.y();
                cylinder_obstacles.push_back(vec);
                cylinder_radii.push_back(obstacle.dimensions.radius); // Using base radius for visualization
            }
            
            // Collect data for velocity arrows for dynamic obstacles
            if (obstacle.is_dynamic && obstacle.velocity.norm() > 0.01) {
                dynamic_obstacle_positions.push_back(obstacle.position);
                dynamic_obstacle_velocities.push_back(obstacle.velocity);
            }
        }

        // Send data to the visualizer
        if (!cylinder_obstacles.empty()) {
            visualizer_->visualizeCylinder(cylinder_obstacles, cylinder_radii, "map", {0.0f, 0.4f, 1.0f}, "obstacles");
        }
        if (!dynamic_obstacle_positions.empty()) {
            visualizer_->visualizeVelocityVectors(
                dynamic_obstacle_positions, 
                dynamic_obstacle_velocities, 
                "map", 
                {1.0f, 0.5f, 0.0f}, // Orange color for velocity
                "velocity_vectors");
        }
    }

    void simulationLoop() {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (!is_path_set_ || current_path_.size() < 2) {
            return;
        }

        current_sim_time_ += simulation_time_step_;

        if (current_sim_time_ < current_path_.back()(3)) {
             RCLCPP_INFO_ONCE(this->get_logger(), "Simulation finished.");
             is_path_set_ = false;
             return;
        }

        auto it_after = std::lower_bound(current_path_.begin(), current_path_.end(), current_sim_time_,
            [](const Eigen::VectorXd& point, double time) {
                return point(3) > time;
            });

        if (it_after == current_path_.begin() || it_after == current_path_.end()) {
             is_path_set_ = false;
             return;
        }

        auto it_before = std::prev(it_after);
        const Eigen::VectorXd& state_before = *it_before;
        const Eigen::VectorXd& state_after = *it_after;
        double time_before = state_before(3);
        double time_after = state_after(3);
        double segment_duration = time_before - time_after;

        Eigen::VectorXd new_state(4);
        if (segment_duration <= 1e-9) {
            new_state = state_after;
        } else {
            double time_into_segment = time_before - current_sim_time_;
            double interp_factor = std::max(0.0, std::min(1.0, time_into_segment / segment_duration));

            new_state.head<2>() = state_before.head<2>() + interp_factor * (state_after.head<2>() - state_before.head<2>());
            double theta_before = state_before(2);
            double theta_after = state_after(2);
            double angle_diff = normalizeAngle(theta_after - theta_before);
            new_state(2) = normalizeAngle(theta_before + interp_factor * angle_diff);
            new_state(3) = current_sim_time_;
        }

        current_interpolated_state_ = new_state;
        Eigen::Vector3d robot_pos_3d(new_state(0), new_state(1), 0.0);
        Eigen::Quaterniond q(Eigen::AngleAxisd(new_state(2), Eigen::Vector3d::UnitZ()));
        Eigen::VectorXd orientation_quat(4);
        orientation_quat << q.x(), q.y(), q.z(), q.w();

        visualizer_->visualizeRobotArrow(robot_pos_3d, orientation_quat, "map", {0.8f, 0.1f, 0.8f}, "simulated_robot");
        if (robot_spatial_trace_.empty() || (robot_spatial_trace_.back() - robot_pos_3d).norm() > 0.1) {
             robot_spatial_trace_.push_back(robot_pos_3d);
        }
        // visualizer_->visualizeTrajectories({robot_spatial_trace_}, "map", {1.0f, 0.5f, 0.0f}, "robot_trace");
    }
};

