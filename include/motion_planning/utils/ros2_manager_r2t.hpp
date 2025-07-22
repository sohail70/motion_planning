// r2t_ros_manager.hpp

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

class R2TROS2Manager : public rclcpp::Node {
public:
    R2TROS2Manager(
        std::shared_ptr<ObstacleChecker> obstacle_checker,
        std::shared_ptr<RVizVisualization> visualizer,
        const Params& params,
        double robot_velocity,
        const Eigen::VectorXd& initial_sim_state) // NEW: Pass in the initial state
        : Node("r2t_ros_manager", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)})),
          obstacle_checker_(obstacle_checker),
          visualizer_(visualizer),
          is_path_set_(false),
          last_known_theta_(0.0),
           robot_velocity_(robot_velocity) 
    {
        // --- FIX: Initialize the simulation state immediately in the constructor ---
        if (initial_sim_state.size() != 3) {
            throw std::runtime_error("R2TROS2Manager: Initial state must be 3D.");
        }
        current_interpolated_state_ = initial_sim_state;
        
        simulation_time_step_ = params.getParam<double>("sim_time_step", -0.04);
        int sim_frequency_hz = params.getParam<int>("sim_frequency_hz", 50);
        int vis_frequency_hz = params.getParam<int>("vis_frequency_hz", 30);
        
        RCLCPP_INFO(this->get_logger(), "Initialized R2TRO2SManager.");

        vis_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / vis_frequency_hz),
            std::bind(&R2TROS2Manager::visualizationLoop, this));

        if (params.getParam<bool>("follow_path")){
            sim_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000 / sim_frequency_hz),
                std::bind(&R2TROS2Manager::simulationLoop, this));
        }
    }




    // void setPath(const std::vector<Eigen::VectorXd>& new_path_from_main) {
    //     std::lock_guard<std::mutex> lock(path_mutex_);
    //     if (new_path_from_main.size() < 2) {
    //         is_path_set_ = false;
    //         return;
    //     }
        
    //     // --- FIX: The path from the planner is now trusted to be in the correct forward order. ---
    //     // --- REMOVE the std::reverse call. ---
    //     current_path_ = new_path_from_main;
        
    //     // The simulation starts from the beginning of the new path.
    //     current_sim_time_ = current_path_.front()(2);

    //     if (!is_path_set_) {
    //          robot_spatial_trace_.clear();
    //     }
    //     is_path_set_ = true;
    // }

    void setPath(const std::vector<Eigen::VectorXd>& new_path_from_main) {
        // std::lock_guard<std::mutex> lock(path_mutex_);

        if (new_path_from_main.empty()) {
            is_path_set_ = false;
            current_path_.clear();
            return;
        }
        
        // Always update the path to the latest one from the planner.
        current_path_ = new_path_from_main;
        
        // CRITICAL FIX: If this is the FIRST time a path is being set,
        // we must initialize the simulation time to the start of that path.
        if (!is_path_set_) {
            // The first point in the path holds the starting state and time-to-go.
            current_sim_time_ = current_path_.front()(2); 
            robot_spatial_trace_.clear();
        }
        
        // Now, we can safely say a path is set. On subsequent calls, the block
        // above will be skipped, preserving current_sim_time_ during replans.
        is_path_set_ = true;
    }


    Eigen::VectorXd getCurrentSimulatedState() {
        // std::lock_guard<std::mutex> lock(path_mutex_);
        // It's possible this is called before the first simulation tick,
        // so ensure the state is initialized.
        if (current_interpolated_state_.size() == 0) {
            // Find the initial state from the end of the path (leaf node) if available
            if (!current_path_.empty()) {
                current_interpolated_state_ = current_path_.back();
            }
        }
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
    std::mutex path_mutex_;
    std::vector<Eigen::VectorXd> current_path_;
    std::vector<Eigen::Vector2d> robot_spatial_trace_;
    double current_sim_time_;
    double simulation_time_step_;
    bool is_path_set_;
    double last_known_theta_;
    // NEW MEMBER VARIABLE: to store the latest interpolated state
    Eigen::VectorXd current_interpolated_state_;
    double robot_velocity_; 

    std::atomic<int> collision_count_{0};
    bool is_in_collision_state_{false};

    void visualizationLoop() {
        if (!obstacle_checker_ || !visualizer_) return;
        
        auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker_);
        if (!gazebo_checker) return;

        gazebo_checker->processLatestPoseInfo();
        const ObstacleVector& all_obstacles = gazebo_checker->getObstaclePositions();
        
        // --- MODIFIED SECTION START ---
        
        // Prepare containers for visualization data
        std::vector<Eigen::VectorXd> cylinder_positions;
        std::vector<double> cylinder_radii;
        
        // ✅ Create the specific data structure your visualizeCube function needs
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
                // ✅ Populate the vector of tuples directly
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
            // ✅ Call your actual visualizeCube function with the correct data structure
            visualizer_->visualizeCube(box_data_for_viz, "map", {0.0f, 0.6f, 0.8f}, "box_obstacles");
        }
        if (!dynamic_obstacle_positions.empty()) {
            visualizer_->visualizeVelocityVectors(dynamic_obstacle_positions, dynamic_obstacle_velocities, "map", {1.0f, 0.5f, 0.0f}, "velocity_vectors");
        }
        
        // --- MODIFIED SECTION END ---
    }




    //     void simulationLoop() {
    //     std::lock_guard<std::mutex> lock(path_mutex_);
    //     if (!is_path_set_ || current_path_.size() < 2) {
    //         return; // Nothing to do if no valid path is set.
    //     }

    //     // --- Time Progression ---
    //     // Time counts down from a high "time-to-go" value towards zero.
    //     current_sim_time_ += simulation_time_step_; // simulation_time_step_ is negative.

    //     // --- FIX #1: Correct Clamping at the End of the Path ---
    //     // The simulation should stop when it reaches or passes the time of the LAST point
    //     // in the path (the root node), which has the lowest time-to-go.
    //     if (current_sim_time_ < current_path_.back()(2)) {
    //         current_sim_time_ = current_path_.back()(2);
    //     }
        
    //     // --- FIX #2: Correctly Find the Current Path Segment ---
    //     // The path is sorted by descending time: [T_start, T_mid, ..., T_end] where T_start > T_end.
    //     // We need to find the first waypoint in the path whose time is less than or equal to
    //     // our current simulation time.
    //     auto it_after = std::lower_bound(current_path_.begin(), current_path_.end(), current_sim_time_,
    //         [](const Eigen::VectorXd& point, double time) {
    //             // This custom comparator tells lower_bound to find the first element
    //             // that is NOT greater than 'time'.
    //             return point(2) > time;
    //         });

    //     // Handle the edge case where we are exactly at or before the first waypoint.
    //     if (it_after == current_path_.begin()) {
    //         it_after++;
    //     }
    //     // If the search fails or goes past the end, we can't form a segment.
    //     if (it_after == current_path_.end()) {
    //         return;
    //     }

    //     // The segment for interpolation is between the waypoint we found and the one just before it.
    //     auto it_before = std::prev(it_after);

    //     const Eigen::VectorXd& state_before = *it_before;
    //     const Eigen::VectorXd& state_after = *it_after;
        
    //     double time_before = state_before(2);
    //     double time_after = state_after(2);
    //     double segment_duration = time_before - time_after; // Note: duration is positive

    //     // --- Interpolation (Unchanged, but now using the correct segment) ---
    //     Eigen::VectorXd current_robot_state(3);
    //     if (segment_duration <= 1e-9) {
    //         current_robot_state = state_after; // Snap to the end of the segment
    //     } else {
    //         // Calculate how far along we are in this specific segment.
    //         double time_into_segment = time_before - current_sim_time_;
    //         double interp_factor = time_into_segment / segment_duration;
    //         current_robot_state.head<2>() = state_before.head<2>() + interp_factor * (state_after.head<2>() - state_before.head<2>());
    //         current_robot_state(2) = current_sim_time_;
    //     }
        
    //     // Update the state for the main thread's feedback loop
    //     current_interpolated_state_ = current_robot_state;

    //     // --- Visualization (Unchanged) ---
    //     Eigen::Vector3d robot_pos_3d(current_robot_state(0), current_robot_state(1), 0.0);
        
    //     // The direction of travel is from the "before" state to the "after" state.
    //     Eigen::Vector2d direction_vector = state_after.head<2>() - state_before.head<2>();
    //     if (direction_vector.norm() > 1e-6) {
    //         last_known_theta_ = atan2(direction_vector.y(), direction_vector.x());
    //     }
        
    //     Eigen::Quaterniond q(Eigen::AngleAxisd(last_known_theta_, Eigen::Vector3d::UnitZ()));
    //     Eigen::VectorXd orientation_quat(4);
    //     orientation_quat << q.x(), q.y(), q.z(), q.w();
        
    //     visualizer_->visualizeRobotArrow(robot_pos_3d, orientation_quat, "map", {0.8f, 0.1f, 0.8f}, "simulated_robot");

    //     if(robot_spatial_trace_.empty() || (robot_spatial_trace_.back() - robot_pos_3d.head<2>()).norm() > 0.1) {
    //          robot_spatial_trace_.push_back(robot_pos_3d.head<2>());
    //     }
    //     // visualizer_->visualizeTrajectories({robot_spatial_trace_}, "map", {1.0f, 0.5f, 0.0f}, "robot_trace");
    // }


    // With speed logs!
    void simulationLoop() {
        // std::lock_guard<std::mutex> lock(path_mutex_);
        if (!is_path_set_ || current_path_.size() < 2) {
            return;
        }

        current_sim_time_ += simulation_time_step_;

        if (current_sim_time_ < current_path_.back()(2)) {
            current_sim_time_ = current_path_.back()(2);
        }
        
        auto it_after = std::lower_bound(current_path_.begin(), current_path_.end(), current_sim_time_,
            [](const Eigen::VectorXd& point, double time) {
                return point(2) > time;
            });

        if (it_after == current_path_.begin()) {
            it_after++;
        }
        if (it_after == current_path_.end()) {
            return;
        }

        auto it_before = std::prev(it_after);
        const Eigen::VectorXd& state_before = *it_before;
        const Eigen::VectorXd& state_after = *it_after;
        
        double time_before = state_before(2);
        double time_after = state_after(2);
        double segment_duration = time_before - time_after;

        // // =================================================================
        // // =========== ADD THIS BLOCK TO CHECK THE SPEED ===================
        // // =================================================================
        // { // Use a block to keep variables local
        //     double spatial_distance = (state_after.head<2>() - state_before.head<2>()).norm();
        //     double segment_speed = 0.0;
        //     if (segment_duration > 1e-9) {
        //         segment_speed = spatial_distance / segment_duration;
        //     }
        //     // Use an RCLCPP_INFO logger to print, which is better than std::cout in ROS 2
        //     // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Current Segment Speed: %.2f m/s", segment_speed);
        //     std::cout<<"Current segment speed: "<<segment_speed<<"\n";
        // }
        // // =================================================================

        Eigen::VectorXd current_robot_state(3);
        if (segment_duration <= 1e-9) {
            current_robot_state = state_after;
        } else {
            double time_into_segment = time_before - current_sim_time_;
            double interp_factor = time_into_segment / segment_duration;
            current_robot_state.head<2>() = state_before.head<2>() + interp_factor * (state_after.head<2>() - state_before.head<2>());
            current_robot_state(2) = current_sim_time_;
        }
        
        current_interpolated_state_ = current_robot_state;
        // =================================================================
        // =========== SIMPLIFIED: COLLISION COUNTING LOGIC ================
        // =================================================================
        auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker_);
        if (gazebo_checker) {
            Eigen::Vector2d current_pos = current_robot_state.head<2>();
            double current_yaw = last_known_theta_;
            
            // Call the single, unified collision check function
            bool is_colliding_now = gazebo_checker->checkRobotCollision(current_pos, current_yaw);

            if (is_colliding_now && !is_in_collision_state_) {
                collision_count_++;
                RCLCPP_FATAL(this->get_logger(), "COLLISION DETECTED! Total Failures: %d", collision_count_.load());
            }
            is_in_collision_state_ = is_colliding_now;
        }
        ////////////////////////////////////////////////////////////////////


        Eigen::Vector3d robot_pos_3d(current_robot_state(0), current_robot_state(1), 0.0);
        
        Eigen::Vector2d direction_vector = state_after.head<2>() - state_before.head<2>();
        if (direction_vector.norm() > 1e-6) {
            last_known_theta_ = atan2(direction_vector.y(), direction_vector.x());
        }
        
        Eigen::Quaterniond q(Eigen::AngleAxisd(last_known_theta_, Eigen::Vector3d::UnitZ()));
        Eigen::VectorXd orientation_quat(4);
        orientation_quat << q.x(), q.y(), q.z(), q.w();
        
        visualizer_->visualizeRobotArrow(robot_pos_3d, orientation_quat, "map", {0.8f, 0.1f, 0.8f}, "simulated_robot");

        if(robot_spatial_trace_.empty() || (robot_spatial_trace_.back() - robot_pos_3d.head<2>()).norm() > 0.1) {
             robot_spatial_trace_.push_back(robot_pos_3d.head<2>());
        }
    }


//     /////////////////////////////////////////////////////////
//     // for constant speed!
//     void simulationLoop() {
//     std::lock_guard<std::mutex> lock(path_mutex_);
//     if (!is_path_set_ || current_path_.size() < 2) {
//         return;
//     }

//     // Time progression (counts down)
//     current_sim_time_ += simulation_time_step_;

//     if (current_sim_time_ < current_path_.back()(2)) {
//         current_sim_time_ = current_path_.back()(2);
//     }
    
//     // Find the current path segment (this logic is correct)
//     auto it_after = std::lower_bound(current_path_.begin(), current_path_.end(), current_sim_time_,
//         [](const Eigen::VectorXd& point, double time) {
//             return point(2) > time;
//         });

//     if (it_after == current_path_.begin()) {
//         it_after++;
//     }
//     if (it_after == current_path_.end()) {
//         // We've reached the end of the path. Hold the final position.
//         current_interpolated_state_ = current_path_.back();
//         return;
//     }

//     auto it_before = std::prev(it_after);
//     const Eigen::VectorXd& state_before = *it_before;
//     const Eigen::VectorXd& state_after = *it_after;
    
//     //
//     // ---> START OF CRITICAL FIX <---
//     //
    
//     // The time-to-go values from the waypoints define the *total time allotted* for this segment.
//     double time_before = state_before(2);
//     double time_after = state_after(2);
//     double time_allotted_for_segment = time_before - time_after;

//     // Calculate the *actual travel time* for this segment based on the constant velocity model.
//     double spatial_distance = (state_after.head<2>() - state_before.head<2>()).norm();
//     double actual_travel_time = (robot_velocity_ > 1e-6)
//                               ? (spatial_distance / robot_velocity_)
//                               : std::numeric_limits<double>::infinity();
    
//     // The travel phase for this segment ends at this time-to-go value.
//     // The robot then "waits" at state_after until the time-to-go reaches time_after.
//     double travel_end_time = time_before - actual_travel_time;
    
//     Eigen::VectorXd current_robot_state(3);
    
//     // Determine if the simulation is currently in the "travel" or "wait" phase of the segment.
//     if (current_sim_time_ >= travel_end_time) {
//         // We are currently moving. Interpolate based on the *actual_travel_time*.
//         if (actual_travel_time <= 1e-9) {
//             // Zero duration, snap to start
//             current_robot_state.head<2>() = state_before.head<2>();
//         } else {
//             // Calculate interpolation factor based on progress through the travel phase
//             double time_into_travel = time_before - current_sim_time_;
//             double interp_factor = time_into_travel / actual_travel_time;
//             interp_factor = std::max(0.0, std::min(1.0, interp_factor)); // Clamp for safety
            
//             current_robot_state.head<2>() = state_before.head<2>() + interp_factor * (state_after.head<2>() - state_before.head<2>());
//         }
//     } else {
//         // The travel phase is over. The robot is now waiting at the segment's end point.
//         current_robot_state.head<2>() = state_after.head<2>();
//     }
    
//     // The time component of the state is always the current simulation time
//     current_robot_state(2) = current_sim_time_;
    
//     //
//     // ---> END OF CRITICAL FIX <---
//     //
    
//     current_interpolated_state_ = current_robot_state;

//     // Visualization logic (remains the same)...
//     Eigen::Vector3d robot_pos_3d(current_robot_state(0), current_robot_state(1), 0.0);
//     Eigen::Vector2d direction_vector = state_after.head<2>() - state_before.head<2>();
//     if (direction_vector.norm() > 1e-6) {
//         last_known_theta_ = atan2(direction_vector.y(), direction_vector.x());
//     }
//     Eigen::Quaterniond q(Eigen::AngleAxisd(last_known_theta_, Eigen::Vector3d::UnitZ()));
//     Eigen::VectorXd orientation_quat(4);
//     orientation_quat << q.x(), q.y(), q.z(), q.w();
//     visualizer_->visualizeRobotArrow(robot_pos_3d, orientation_quat, "map", {0.8f, 0.1f, 0.8f}, "simulated_robot");
//     if(robot_spatial_trace_.empty() || (robot_spatial_trace_.back() - robot_pos_3d.head<2>()).norm() > 0.1) {
//          robot_spatial_trace_.push_back(robot_pos_3d.head<2>());
//     }
// }

    
};