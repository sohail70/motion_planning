#pragma once

#include "rclcpp/rclcpp.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry> // For Eigen::AngleAxisd, Eigen::Quaterniond
#include <mutex>
#include <chrono>
#include <functional>
#include <vector>
#include <algorithm> // For std::sort, std::lower_bound, std::unique
#include <sstream>   // For logging Eigen vectors

/**
 * @brief Helper function to convert an Eigen vector to a string for easy logging.
 * @param vec The vector to print.
 * @return A string representation, e.g., "[x, y, z]".
 */
inline std::string printEigenVec(const Eigen::VectorXd& vec) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "[" << vec.transpose() << "]";
    return ss.str();
}

/**
 * @brief Manages the simulation and visualization of a Dubins path in ROS2.
 *
 * This class is responsible for taking a complete, time-parameterized Dubins path
 * and simulating a robot's movement along it. It runs a control loop that
 * interpolates the robot's state (x, y, theta) at each time step and publishes
 * visualization markers to RViz, including the robot's current pose and its traveled trail.
 * The simulation is configured to run backward in time.
 */
class DubinsROS2Manager : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Dubins ROS2 Manager object.
     * @param visualizer A shared pointer to the RViz visualization utility.
     */
    DubinsROS2Manager(std::shared_ptr<RVizVisualization> visualizer)
        : Node("dubins_ros2_manager"),
          visualizer_(visualizer),
          simulation_time_step_(-0.02) // Negative step for 50Hz reverse simulation (t=60 -> t=0)
    {
        RCLCPP_INFO(this->get_logger(), "Initialized DubinsROS2Manager.");

        // Timer for the main control/simulation loop, running at 50Hz.
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&DubinsROS2Manager::dubinsControlLoop, this));
        
        // Initialize the state vector for a 4D Dubins state: [x, y, theta, time]
        current_dubins_state_.resize(4);
        current_dubins_state_.setZero();
    }

    /**
     * @brief Sets the initial state of the robot for the simulation.
     * This should typically be the state at the end of the path (e.g., at t=60s).
     * @param state The initial 4D state vector [x, y, theta, time].
     */
    void setInitialState(const Eigen::VectorXd& state) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (state.size() != 4) {
            RCLCPP_ERROR(this->get_logger(), "Initial state must be 4D for Dubins model.");
            return;
        }
        current_dubins_state_ = state;
        RCLCPP_INFO(this->get_logger(), "Set initial robot state to: %s", printEigenVec(current_dubins_state_).c_str());
        
        // Clear any previous trail and add the starting point.
        robot_spatial_trace_.clear();
        robot_spatial_trace_.push_back(current_dubins_state_.head<3>());
    }

    /**
     * @brief Provides the manager with the full, time-parameterized path to follow.
     * The provided path points are sorted by time to ensure correct interpolation.
     * @param path A vector of 4D state vectors representing the path.
     */
    void setPlannedDubinsPath(const std::vector<Eigen::VectorXd>& path) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (path.empty()) {
            RCLCPP_WARN(this->get_logger(), "Attempted to set an empty path.");
            full_planned_path_.clear();
            is_path_set_ = false;
            return;
        }

        full_planned_path_ = path;

        // Sort the path by the time component (4th element) to ensure it's chronological.
        std::sort(full_planned_path_.begin(), full_planned_path_.end(), 
                  [](const Eigen::VectorXd& a, const Eigen::VectorXd& b){
                      return a(3) < b(3); // Sort by time (index 3)
                  });
        
        // Remove duplicate points that might result from concatenating segments.
        auto last = std::unique(full_planned_path_.begin(), full_planned_path_.end(),
                        [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
                            // Consider points duplicates if their time is very close.
                            return std::abs(a(3) - b(3)) < 1e-6;
                        });
        full_planned_path_.erase(last, full_planned_path_.end());

        is_path_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Set new Dubins path with %zu unique points.", full_planned_path_.size());
    }

private:
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::shared_ptr<RVizVisualization> visualizer_;

    // State & Path Data
    Eigen::VectorXd current_dubins_state_;        // Current [x, y, theta, time] of the simulated robot
    std::vector<Eigen::VectorXd> full_planned_path_; // The complete, sorted trajectory from the planner
    std::vector<Eigen::VectorXd> robot_spatial_trace_; // History of the robot's 3D pose [x,y,theta] for drawing its trail

    // Control Flow
    std::mutex path_mutex_;
    bool is_path_set_ = false;
    const double simulation_time_step_; // Time step for each loop (negative for reverse)

    /**
     * @brief Normalizes an angle to the range [-PI, PI].
     */
    double normalizeAngle(double angle) {
        angle = fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0.0)
            angle += 2.0 * M_PI;
        return angle - M_PI;
    }

    /**
     * @brief The main simulation loop, executed by the wall timer.
     */
    void dubinsControlLoop() {
        std::lock_guard<std::mutex> lock(path_mutex_);

        if (!is_path_set_ || full_planned_path_.size() < 2) {
            return; // Nothing to do if no valid path is set.
        }

        // Update the simulation time by stepping it backward.
        double new_sim_time = current_dubins_state_(3) + simulation_time_step_;
        
        // Stop condition: if we've gone past the beginning of the path's time.
        if (new_sim_time < full_planned_path_.front()(3)) {
             RCLCPP_INFO(this->get_logger(), "Simulation finished. Reached start of the path.");
             is_path_set_ = false; // Stop the simulation.
             return;
        }
        
        // Find the segment of the path corresponding to the new simulation time.
        // We use lower_bound to find the first point with a time >= new_sim_time.
        auto it_after = std::lower_bound(full_planned_path_.begin(), full_planned_path_.end(), new_sim_time,
            [](const Eigen::VectorXd& point, double time) {
                return point(3) < time;
            });
            
        // Ensure we have a valid segment [P_before, P_after].
        if (it_after == full_planned_path_.begin() || it_after == full_planned_path_.end()) {
             RCLCPP_WARN(this->get_logger(), "Simulation time is outside path data range. Halting.");
             is_path_set_ = false;
             return;
        }
        
        auto it_before = std::prev(it_after);
        
        const Eigen::VectorXd& state_before = *it_before;
        const Eigen::VectorXd& state_after = *it_after;
        
        double time_before = state_before(3);
        double time_after = state_after(3);
        double segment_duration = time_after - time_before;

        // Interpolate the robot's state.
        if (segment_duration <= 1e-9) {
            // If the time gap is negligible, just snap to the earlier state.
            current_dubins_state_ = state_before;
        } else {
            // Linearly interpolate between the two states based on time.
            double interp_factor = (new_sim_time - time_before) / segment_duration;
            
            // Interpolate x, y positions.
            current_dubins_state_.head<2>() = state_before.head<2>() + interp_factor * (state_after.head<2>() - state_before.head<2>());
            
            // Interpolate theta (heading) carefully to handle angle wrapping.
            double theta_before = state_before(2);
            double theta_after = state_after(2);
            double angle_diff = normalizeAngle(theta_after - theta_before);
            current_dubins_state_(2) = normalizeAngle(theta_before + interp_factor * angle_diff);
            
            // Update the time component of the state.
            current_dubins_state_(3) = new_sim_time;
        }

        // --- Visualization ---
        Eigen::Vector3d robot_pos_3d(current_dubins_state_(0), current_dubins_state_(1), 0.0);
        double robot_theta = current_dubins_state_(2);

        // Convert heading angle (theta) to a quaternion for RViz's arrow marker.
        Eigen::Quaterniond robot_q(Eigen::AngleAxisd(robot_theta, Eigen::Vector3d::UnitZ()));
        Eigen::VectorXd robot_orientation_quat(4);
        robot_orientation_quat << robot_q.x(), robot_q.y(), robot_q.z(), robot_q.w();
        
        // Visualize the robot's current pose as a purple arrow.
        visualizer_->visualizeRobotArrow(robot_pos_3d, robot_orientation_quat, "map", {0.8f, 0.1f, 0.8f}, "dubins_robot_marker");

        // Update and visualize the robot's traveled trail in orange.
        robot_spatial_trace_.push_back(robot_pos_3d);
        visualizer_->visualizeTrajectories({robot_spatial_trace_}, "map", {1.0f, 0.5f, 0.0f}, "robot_trace");
    }
};


//////////////////////////
// /////////////////////////////








// #include "rclcpp/rclcpp.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"
// #include <Eigen/Dense>
// #include <Eigen/Geometry> // For Eigen::AngleAxisd, Eigen::Quaterniond
// #include <mutex>
// #include <chrono>
// #include <functional>
// #include <vector>
// #include <algorithm> // For std::sort, std::lower_bound, std::unique
// #include <sstream>   // For logging Eigen vectors

// /**
//  * @brief Helper function to convert an Eigen vector to a string for easy logging.
//  * @param vec The vector to print.
//  * @return A string representation, e.g., "[x, y, z]".
//  */
// inline std::string printEigenVec(const Eigen::VectorXd& vec) {
//     std::stringstream ss;
//     ss << std::fixed << std::setprecision(2);
//     ss << "[" << vec.transpose() << "]";
//     return ss.str();
// }

// /**
//  * @brief Manages the simulation and visualization of a Dubins path in ROS2.
//  *
//  * This class is responsible for taking a complete, time-parameterized Dubins path
//  * and simulating a robot's movement along it. It runs a control loop that
//  * interpolates the robot's state (x, y, theta) at each time step and publishes
//  * visualization markers to RViz, including the robot's current pose and its traveled trail.
//  * The simulation is configured to run backward in time.
//  */
// class DubinsROS2Manager : public rclcpp::Node {
// public:
//     /**
//      * @brief Construct a new Dubins ROS2 Manager object.
//      * @param visualizer A shared pointer to the RViz visualization utility.
//      */
//     DubinsROS2Manager(std::shared_ptr<RVizVisualization> visualizer)
//         : Node("dubins_ros2_manager"),
//           visualizer_(visualizer),
//           simulation_time_step_(-0.02) // Negative step for 50Hz reverse simulation (t=60 -> t=0)
//     {
//         RCLCPP_INFO(this->get_logger(), "Initialized DubinsROS2Manager.");

//         // Timer for the main control/simulation loop, running at 50Hz.
//         control_timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(20),
//             std::bind(&DubinsROS2Manager::dubinsControlLoop, this));
        
//         // Initialize the state vector for a 4D Dubins state: [x, y, theta, time]
//         current_dubins_state_.resize(4);
//         current_dubins_state_.setZero();
//     }

//     /**
//      * @brief Sets the initial state of the robot for the simulation.
//      * This should typically be the state at the end of the path (e.g., at t=60s).
//      * @param state The initial 4D state vector [x, y, theta, time].
//      */
//     void setInitialState(const Eigen::VectorXd& state) {
//         std::lock_guard<std::mutex> lock(path_mutex_);
//         if (state.size() != 4) {
//             RCLCPP_ERROR(this->get_logger(), "Initial state must be 4D for Dubins model.");
//             return;
//         }
//         current_dubins_state_ = state;
//         RCLCPP_INFO(this->get_logger(), "Set initial robot state to: %s", printEigenVec(current_dubins_state_).c_str());
        
//         // Clear any previous trail and add the starting point.
//         robot_spatial_trace_.clear();
//         robot_spatial_trace_.push_back(current_dubins_state_.head<3>());
//     }

//     /**
//      * @brief Provides the manager with the full, time-parameterized path to follow.
//      * The provided path points are sorted by time to ensure correct interpolation.
//      * @param path A vector of 4D state vectors representing the path.
//      */
//     void setPlannedDubinsPath(const std::vector<Eigen::VectorXd>& path) {
//         std::lock_guard<std::mutex> lock(path_mutex_);
//         if (path.empty()) {
//             RCLCPP_WARN(this->get_logger(), "Attempted to set an empty path.");
//             full_planned_path_.clear();
//             is_path_set_ = false;
//             return;
//         }

//         full_planned_path_ = path;

//         // Sort the path by the time component (4th element) to ensure it's chronological.
//         std::sort(full_planned_path_.begin(), full_planned_path_.end(), 
//                   [](const Eigen::VectorXd& a, const Eigen::VectorXd& b){
//                       return a(3) < b(3); // Sort by time (index 3)
//                   });
        
//         is_path_set_ = true;
//         RCLCPP_INFO(this->get_logger(), "Set new Dubins path with %zu unique points.", full_planned_path_.size());
//     }

// private:
//     rclcpp::TimerBase::SharedPtr control_timer_;
//     std::shared_ptr<RVizVisualization> visualizer_;

//     // State & Path Data
//     Eigen::VectorXd current_dubins_state_;        // Current [x, y, theta, time] of the simulated robot
//     std::vector<Eigen::VectorXd> full_planned_path_; // The complete, sorted trajectory from the planner
//     std::vector<Eigen::VectorXd> robot_spatial_trace_; // History of the robot's 3D pose [x,y,theta] for drawing its trail

//     // Control Flow
//     std::mutex path_mutex_;
//     bool is_path_set_ = false;
//     const double simulation_time_step_; // Time step for each loop (negative for reverse)

//     /**
//      * @brief The main simulation loop, executed by the wall timer.
//      */
//     void dubinsControlLoop() {
//         std::lock_guard<std::mutex> lock(path_mutex_);

//         if (!is_path_set_ || full_planned_path_.size() < 2) {
//             return; // Nothing to do if no valid path is set.
//         }

//         double new_sim_time = current_dubins_state_(3) + simulation_time_step_;
        
//         if (new_sim_time < full_planned_path_.front()(3)) {
//              RCLCPP_INFO(this->get_logger(), "Simulation finished. Reached start of the path.");
//              is_path_set_ = false; // Stop the simulation.
//              return;
//         }
        
//         auto it_after = std::lower_bound(full_planned_path_.begin(), full_planned_path_.end(), new_sim_time,
//             [](const Eigen::VectorXd& point, double time) {
//                 return point(3) < time;
//             });
            
//         if (it_after == full_planned_path_.begin() || it_after == full_planned_path_.end()) {
//              RCLCPP_WARN(this->get_logger(), "Simulation time is outside path data range. Halting.");
//              is_path_set_ = false;
//              return;
//         }
        
//         auto it_before = std::prev(it_after);
//         const Eigen::VectorXd& state_before = *it_before;
//         const Eigen::VectorXd& state_after = *it_after;
        
//         // --- POSITION INTERPOLATION ---
//         double time_before = state_before(3);
//         double time_after = state_after(3);
//         double segment_duration = time_after - time_before;

//         if (segment_duration > 1e-9) {
//             double interp_factor = (new_sim_time - time_before) / segment_duration;
//             current_dubins_state_.head<2>() = state_before.head<2>() + interp_factor * (state_after.head<2>() - state_before.head<2>());
//         } else {
//             current_dubins_state_.head<2>() = state_before.head<2>();
//         }
//         current_dubins_state_(3) = new_sim_time;

//         // --- CORRECTED ORIENTATION CALCULATION ---
//         // Since the robot moves from state_after to state_before (backward in time),
//         // the direction vector must be (state_before - state_after).
//         double dx = state_before(0) - state_after(0);
//         double dy = state_before(1) - state_after(1);
        
//         // Only update heading if there's noticeable movement. Otherwise, keep the last heading.
//         if (std::sqrt(dx*dx + dy*dy) > 1e-6) {
//             current_dubins_state_(2) = std::atan2(dy, dx);
//         } else {
//             // If there's no movement, use the heading from the path data.
//             current_dubins_state_(2) = state_before(2);
//         }

//         // --- Visualization ---
//         Eigen::Vector3d robot_pos_3d(current_dubins_state_(0), current_dubins_state_(1), 0.0);
//         double robot_theta = current_dubins_state_(2);

//         Eigen::Quaterniond robot_q(Eigen::AngleAxisd(robot_theta, Eigen::Vector3d::UnitZ()));
//         Eigen::VectorXd robot_orientation_quat(4);
//         robot_orientation_quat << robot_q.x(), robot_q.y(), robot_q.z(), robot_q.w();
        
//         visualizer_->visualizeRobotArrow(robot_pos_3d, robot_orientation_quat, "map", {0.8f, 0.1f, 0.8f}, "dubins_robot_marker");

//         robot_spatial_trace_.push_back(robot_pos_3d);
//         visualizer_->visualizeTrajectories({robot_spatial_trace_}, "map", {1.0f, 0.5f, 0.0f}, "robot_trace");
//     }
// };



/////////////////////
// //////////////////////
// #pragma once

// #include "rclcpp/rclcpp.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"
// #include <Eigen/Dense>
// #include <Eigen/Geometry> // For Eigen::AngleAxisd, Eigen::Quaterniond
// #include <mutex>
// #include <chrono>
// #include <functional>
// #include <vector>
// #include <algorithm> // For std::sort, std::lower_bound, std::unique
// #include <sstream>   // For logging Eigen vectors

// /**
//  * @brief Helper function to convert an Eigen vector to a string for easy logging.
//  * @param vec The vector to print.
//  * @return A string representation, e.g., "[x, y, z]".
//  */
// inline std::string printEigenVec(const Eigen::VectorXd& vec) {
//     std::stringstream ss;
//     ss << std::fixed << std::setprecision(2);
//     ss << "[" << vec.transpose() << "]";
//     return ss.str();
// }

// /**
//  * @brief Manages the simulation and visualization of a Dubins path in ROS2.
//  *
//  * This class is responsible for taking a complete, time-parameterized Dubins path
//  * and simulating a robot's movement along it. It runs a control loop that
//  * interpolates the robot's state (x, y, theta) at each time step and publishes
//  * visualization markers to RViz, including the robot's current pose and its traveled trail.
//  * The simulation is configured to run backward in time.
//  */
// class DubinsROS2Manager : public rclcpp::Node {
// public:
//     /**
//      * @brief Construct a new Dubins ROS2 Manager object.
//      * @param visualizer A shared pointer to the RViz visualization utility.
//      */
//     DubinsROS2Manager(std::shared_ptr<RVizVisualization> visualizer)
//         : Node("dubins_ros2_manager"),
//           visualizer_(visualizer),
//           simulation_time_step_(-0.02) // Negative step for 50Hz reverse simulation (t=60 -> t=0)
//     {
//         RCLCPP_INFO(this->get_logger(), "Initialized DubinsROS2Manager.");

//         // Timer for the main control/simulation loop, running at 50Hz.
//         control_timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(20),
//             std::bind(&DubinsROS2Manager::dubinsControlLoop, this));
        
//         // Initialize the state vector for a 4D Dubins state: [x, y, theta, time]
//         current_dubins_state_.resize(4);
//         current_dubins_state_.setZero();
//     }

//     /**
//      * @brief Sets the initial state of the robot for the simulation.
//      * This should typically be the state at the end of the path (e.g., at t=60s).
//      * @param state The initial 4D state vector [x, y, theta, time].
//      */
//     void setInitialState(const Eigen::VectorXd& state) {
//         std::lock_guard<std::mutex> lock(path_mutex_);
//         if (state.size() != 4) {
//             RCLCPP_ERROR(this->get_logger(), "Initial state must be 4D for Dubins model.");
//             return;
//         }
//         current_dubins_state_ = state;
//         RCLCPP_INFO(this->get_logger(), "Set initial robot state to: %s", printEigenVec(current_dubins_state_).c_str());
        
//         // Clear any previous trail and add the starting point.
//         robot_spatial_trace_.clear();
//         robot_spatial_trace_.push_back(current_dubins_state_.head<3>());
//     }

//     /**
//      * @brief Provides the manager with the full, time-parameterized path to follow.
//      * The provided path points are sorted by time to ensure correct interpolation.
//      * @param path A vector of 4D state vectors representing the path.
//      */
//     void setPlannedDubinsPath(const std::vector<Eigen::VectorXd>& path) {
//         std::lock_guard<std::mutex> lock(path_mutex_);
//         if (path.empty()) {
//             RCLCPP_WARN(this->get_logger(), "Attempted to set an empty path.");
//             full_planned_path_.clear();
//             is_path_set_ = false;
//             return;
//         }

//         full_planned_path_ = path;

//         // Sort the path by the time component (4th element) to ensure it's chronological.
//         std::sort(full_planned_path_.begin(), full_planned_path_.end(), 
//                   [](const Eigen::VectorXd& a, const Eigen::VectorXd& b){
//                       return a(3) < b(3); // Sort by time (index 3)
//                   });
        
//         is_path_set_ = true;
//         RCLCPP_INFO(this->get_logger(), "Set new Dubins path with %zu unique points.", full_planned_path_.size());
//     }

// private:
//     rclcpp::TimerBase::SharedPtr control_timer_;
//     std::shared_ptr<RVizVisualization> visualizer_;

//     // State & Path Data
//     Eigen::VectorXd current_dubins_state_;        // Current [x, y, theta, time] of the simulated robot
//     std::vector<Eigen::VectorXd> full_planned_path_; // The complete, sorted trajectory from the planner
//     std::vector<Eigen::VectorXd> robot_spatial_trace_; // History of the robot's 3D pose [x,y,theta] for drawing its trail

//     // Control Flow
//     std::mutex path_mutex_;
//     bool is_path_set_ = false;
//     const double simulation_time_step_; // Time step for each loop (negative for reverse)

//     /**
//      * @brief Normalizes an angle to the range [-PI, PI].
//      */
//     double normalizeAngle(double angle) {
//         angle = fmod(angle + M_PI, 2.0 * M_PI);
//         if (angle < 0.0)
//             angle += 2.0 * M_PI;
//         return angle - M_PI;
//     }


//     /**
//      * @brief The main simulation loop, executed by the wall timer.
//      */
//     void dubinsControlLoop() {
//         std::lock_guard<std::mutex> lock(path_mutex_);

//         if (!is_path_set_ || full_planned_path_.size() < 2) {
//             return; // Nothing to do if no valid path is set.
//         }

//         double new_sim_time = current_dubins_state_(3) + simulation_time_step_;
        
//         if (new_sim_time < full_planned_path_.front()(3)) {
//              RCLCPP_INFO(this->get_logger(), "Simulation finished. Reached start of the path.");
//              is_path_set_ = false; // Stop the simulation.
//              return;
//         }
        
//         auto it_after = std::lower_bound(full_planned_path_.begin(), full_planned_path_.end(), new_sim_time,
//             [](const Eigen::VectorXd& point, double time) {
//                 return point(3) < time;
//             });
            
//         if (it_after == full_planned_path_.begin() || it_after == full_planned_path_.end()) {
//              RCLCPP_WARN(this->get_logger(), "Simulation time is outside path data range. Halting.");
//              is_path_set_ = false;
//              return;
//         }
        
//         auto it_before = std::prev(it_after);
//         const Eigen::VectorXd& state_before = *it_before;
//         const Eigen::VectorXd& state_after = *it_after;
        
//         // --- POSITION INTERPOLATION ---
//         double time_before = state_before(3);
//         double time_after = state_after(3);
//         double segment_duration = time_after - time_before;
//         double interp_factor = (segment_duration > 1e-9) ? (new_sim_time - time_before) / segment_duration : 0.0;

//         current_dubins_state_.head<2>() = state_before.head<2>() + interp_factor * (state_after.head<2>() - state_before.head<2>());
//         current_dubins_state_(3) = new_sim_time;


//         // --- DEFINITIVE HYBRID ORIENTATION CALCULATION ---
//         double dx = state_before(0) - state_after(0);
//         double dy = state_before(1) - state_after(1);
//         double segment_dist = std::sqrt(dx*dx + dy*dy);
        
//         // Threshold to distinguish between sparse straight lines and dense arcs.
//         // A value slightly larger than the planner's discretization step is a good choice.
//         const double straight_line_threshold = 1.0; 

//         if (segment_dist > straight_line_threshold) {
//             // LONG SEGMENT: Assumed to be a sparse straight line.
//             // Use robust atan2 calculation for a constant, correct heading.
//             current_dubins_state_(2) = std::atan2(dy, dx);
//         } else {
//             // SHORT SEGMENT: Assumed to be a dense point on an arc.
//             // Interpolate theta from the path data for smooth rotation.
//             double theta_before = state_before(2);
//             double theta_after = state_after(2);
//             // Handle angle wrapping for correct interpolation
//             double angle_diff = normalizeAngle(theta_after - theta_before);
//             current_dubins_state_(2) = normalizeAngle(theta_before + interp_factor * angle_diff);
//         }


//         // --- Visualization ---
//         Eigen::Vector3d robot_pos_3d(current_dubins_state_(0), current_dubins_state_(1), 0.0);
//         double robot_theta = current_dubins_state_(2);

//         Eigen::Quaterniond robot_q(Eigen::AngleAxisd(robot_theta, Eigen::Vector3d::UnitZ()));
//         Eigen::VectorXd robot_orientation_quat(4);
//         robot_orientation_quat << robot_q.x(), robot_q.y(), robot_q.z(), robot_q.w();
        
//         visualizer_->visualizeRobotArrow(robot_pos_3d, robot_orientation_quat, "map", {0.8f, 0.1f, 0.8f}, "dubins_robot_marker");

//         robot_spatial_trace_.push_back(robot_pos_3d);
//         visualizer_->visualizeTrajectories({robot_spatial_trace_}, "map", {1.0f, 0.5f, 0.0f}, "robot_trace");
//     }
// };
