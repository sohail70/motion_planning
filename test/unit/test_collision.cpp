// Copyright 2025 Soheil E.nia


// // // ////////////////////////////////////R2T StateSpace Collision Test//////////////////////////////////////////////////////////////
// #include "rclcpp/rclcpp.hpp"
// #include "motion_planning/utils/gazebo_obstacle_checker.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"
// #include "motion_planning/utils/params.hpp"
// #include "motion_planning/ds/edge_info.hpp"
// #include "motion_planning/utils/parse_sdf.hpp"

// #include <gz/transport/Node.hh>
// #include <gz/msgs/world_control.pb.h>
// #include <gz/msgs/boolean.pb.h>

// // Helper function to reset and play the Gazebo simulation
// void resetAndPlaySimulation() {
//     gz::transport::Node node;
//     {
//         gz::msgs::WorldControl reset_req;
//         reset_req.mutable_reset()->set_all(true);
//         gz::msgs::Boolean reset_res;
//         bool result;
//         node.Request("/world/default/control", reset_req, 3000, reset_res, result);
//     }
//     std::this_thread::sleep_for(std::chrono::milliseconds(500));
//     {
//         gz::msgs::WorldControl play_req;
//         play_req.set_pause(false);
//         gz::msgs::Boolean play_res;
//         bool result;
//         node.Request("/world/default/control", play_req, 3000, play_res, result);
//     }
// }

// class RobotSimulator {
// public:
//     RobotSimulator(const Eigen::VectorXd& start, const Eigen::VectorXd& goal, double speed)
//         : start_state_(start), goal_state_(goal), speed_(speed), elapsed_time_(0.0) {
        
//         current_state_ = start_state_;
//         total_distance_ = (goal_state_.head<2>() - start_state_.head<2>()).norm();
//         total_duration_ = total_distance_ / speed_;
//     }

//     // Move the robot forward in time by a small step 'dt'
//     void update(double dt) {
//         if (isFinished()) {
//             return;
//         }
//         elapsed_time_ += dt;
        
//         // Calculate interpolation factor (clamp between 0 and 1)
//         double interp_factor = std::max(0.0, std::min(1.0, elapsed_time_ / total_duration_));

//         // Linearly interpolate the spatial position
//         current_state_.head<2>() = (1.0 - interp_factor) * start_state_.head<2>() + interp_factor * goal_state_.head<2>();
        
//         // The third element, time-to-go, is the remaining duration
//         current_state_(2) = total_duration_ - elapsed_time_;
//     }

//     Eigen::VectorXd getCurrentState() const {
//         return current_state_;
//     }

//     bool isFinished() const {
//         return elapsed_time_ >= total_duration_;
//     }

// private:
//     Eigen::VectorXd start_state_;
//     Eigen::VectorXd goal_state_;
//     Eigen::VectorXd current_state_;
//     double speed_;
//     double total_distance_;
//     double total_duration_;
//     double elapsed_time_;
// };



// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);

//     // --- Basic Setup ---
//     auto node = std::make_shared<rclcpp::Node>("predictive_test_node", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
//     auto visualizer = std::make_shared<RVizVisualization>(node);
    
//     Params gazebo_params;
//     gazebo_params.setParam("robot_model_name", "tugbot");
//     gazebo_params.setParam("default_robot_x", 48.0);
//     gazebo_params.setParam("default_robot_y", 48.0);
//     gazebo_params.setParam("world_name", "default");
//     gazebo_params.setParam("use_range", false);
//     gazebo_params.setParam("sensor_range", 20.0);
//     gazebo_params.setParam("inflation", 0.5); 
//     gazebo_params.setParam("persistent_static_obstacles", true);
//     gazebo_params.setParam("estimation", true);
//     gazebo_params.setParam("kf_model_type", "cv");
//     gazebo_params.setParam("fcl", false);
//     gazebo_params.setParam("bullet", false);
//     auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_4_obs.sdf");
//     auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(node->get_clock(), gazebo_params, obstacle_info);

//     // --- Define Robot's Path & Validate Kinematics ---
//     const double MAX_ROBOT_SPEED = 5.0; // Set a max speed limit in m/s

//     // Define start and goal with a time dimension (time-to-go).
//     // The goal must have a lower time than the start.
//     Eigen::VectorXd start_node(3);
//     start_node << 15.0, 15.0, 10.0; // Start at (15,15) with 12 seconds to reach the goal.

//     Eigen::VectorXd goal_node(3);
//     goal_node << -15.0, -15.0, 0.0; // Goal is at (-15,-15) at the final moment (t=0).

//     // Calculate the required velocity to meet the time constraints.
//     double spatial_distance = (goal_node.head<2>() - start_node.head<2>()).norm();
//     double time_duration = start_node(2) - goal_node(2);

//     if (time_duration <= 0) {
//         throw std::runtime_error("Time duration must be positive. Goal time must be less than start time.");
//     }
//     double required_speed = spatial_distance / time_duration;

//     RCLCPP_INFO(node->get_logger(), "Path distance: %.2f m, Time duration: %.2f s, Required speed: %.2f m/s",
//         spatial_distance, time_duration, required_speed);

//     // Validate against the robot's maximum speed.
//     if (required_speed > MAX_ROBOT_SPEED) {
//         RCLCPP_ERROR(node->get_logger(), "Impossible trajectory! Required speed (%.2f m/s) exceeds max speed (%.2f m/s).",
//             required_speed, MAX_ROBOT_SPEED);
//         throw std::runtime_error("Required speed exceeds max speed.");
//     }
    
//     // Create the simulator with the validated, required speed.
//     RobotSimulator simulator(start_node, goal_node, required_speed);

//     // --- Run Simulation and Prediction Loop ---
//     resetAndPlaySimulation();
//     RCLCPP_INFO(node->get_logger(), "Starting simulation and prediction loop...");
    


//     // Wait for the first Gazebo message to arrive to ensure we have obstacles.
//     RCLCPP_INFO(node->get_logger(), "Waiting for initial obstacle data from Gazebo...");
//     size_t obstacle_count = 0;
//     for (int i = 0; i < 50; ++i) { // Try for up to 5 seconds
//         rclcpp::spin_some(node);
//         auto snapshot = obstacle_checker->getAtomicSnapshot();
//         obstacle_count = snapshot.obstacles.size();
//         if (obstacle_count > 0) {
//             RCLCPP_INFO(node->get_logger(), "Received data for %zu obstacles. Starting test.", obstacle_count);
//             break;
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }

//     if (obstacle_count == 0) {
//         RCLCPP_ERROR(node->get_logger(), "Timed out waiting for obstacle data from Gazebo. Aborting test.");
//         rclcpp::shutdown();
//         return EXIT_FAILURE;
//     }


//     rclcpp::Rate loop_rate(20); 
//     rclcpp::Clock::SharedPtr clock = node->get_clock();
//     rclcpp::Time last_update_time = clock->now();

//     while (rclcpp::ok() && !simulator.isFinished()) {
//         rclcpp::spin_some(node);

//         rclcpp::Time current_time = clock->now();
//         double dt = (current_time - last_update_time).seconds();
//         last_update_time = current_time;

//         simulator.update(dt);
//         Eigen::VectorXd current_robot_state = simulator.getCurrentState();

//         Trajectory remaining_trajectory;
//         remaining_trajectory.path_points.push_back(current_robot_state);
//         remaining_trajectory.path_points.push_back(goal_node);
//         remaining_trajectory.time_duration = current_robot_state(2); 

//         auto snapshot = obstacle_checker->getAtomicSnapshot();
//         bool is_safe = obstacle_checker->isTrajectorySafe(remaining_trajectory, current_time.seconds());

//         // --- Visualize the result ---
//         Eigen::Vector3d robot_pos_3d(current_robot_state(0), current_robot_state(1), 0.0);
//         Eigen::Vector2d direction_vector = goal_node.head<2>() - current_robot_state.head<2>();
//         double robot_yaw = atan2(direction_vector.y(), direction_vector.x());
//         Eigen::Quaterniond q(Eigen::AngleAxisd(robot_yaw, Eigen::Vector3d::UnitZ()));
//         Eigen::VectorXd orientation_quat(4);
//         orientation_quat << q.x(), q.y(), q.z(), q.w();

//         // Visualize robot arrow
//         std::vector<float> robot_color = is_safe ? std::vector<float>{0.1f, 0.8f, 0.1f, 1.0f}
//                                                  : std::vector<float>{1.0f, 0.1f, 0.1f, 1.0f};
//         visualizer->visualizeRobotArrow(robot_pos_3d, orientation_quat, "map", robot_color, "simulated_robot");
        
//         // Visualize the trajectory line
//         std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> remaining_path_edge;
//         remaining_path_edge.push_back({current_robot_state, goal_node});
//         std::string trajectory_color_str = is_safe ? "0.1,0.8,0.1" : "1.0,0.1,0.1";
//         visualizer->visualizeEdges(remaining_path_edge, "map", trajectory_color_str, "remaining_trajectory");

//         // Prepare separate containers for each shape type
//         std::vector<Eigen::VectorXd> cylinder_positions;
//         std::vector<double> cylinder_radii;
//         std::vector<std::tuple<Eigen::Vector2d, double, double, double>> box_data_for_viz;

//         // Sort obstacles by type
//         for (const auto& obs : snapshot.obstacles) {
//             if (obs.type == Obstacle::CIRCLE) {
//                 cylinder_positions.push_back(obs.position);
//                 cylinder_radii.push_back(obs.dimensions.radius + obs.inflation);
//             } else if (obs.type == Obstacle::BOX) {
//                 box_data_for_viz.emplace_back(
//                     obs.position,
//                     obs.dimensions.width + 2 * obs.inflation,
//                     obs.dimensions.height + 2 * obs.inflation,
//                     obs.dimensions.rotation
//                 );
//             }
//         }

//         // Call the correct visualization function for each shape
//         if (!cylinder_positions.empty()) {
//             visualizer->visualizeCylinder(cylinder_positions, cylinder_radii, "map", {0.0f, 0.4f, 1.0f}, "cylinder_obstacles");
//         }
//         if (!box_data_for_viz.empty()) {
//             visualizer->visualizeCube(box_data_for_viz, "map", {0.0f, 0.6f, 0.8f}, "box_obstacles");
//         }

//         loop_rate.sleep();
//     }
    
//     RCLCPP_INFO(node->get_logger(), "Simulation finished.");
//     rclcpp::shutdown();
//     return 0;
// }





// //////////////////////////////////Dubin Time-StateSpace Collision Test /////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////////

// //  Dubins Time-State Space Predictive Test

// //  This test simulates a robot following a kinematically feasible Dubins path
// //  and continuously checks the remainder of its trajectory for collisions with
// //  dynamic obstacles.

// ////////////////////////////////////////////////////////////////////////////////////////////////

// #include "rclcpp/rclcpp.hpp"
// #include "motion_planning/utils/gazebo_obstacle_checker.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"
// #include "motion_planning/utils/params.hpp"
// #include "motion_planning/ds/edge_info.hpp"
// #include "motion_planning/utils/parse_sdf.hpp"
// #include "motion_planning/state_space/dubins_time_statespace.hpp"
// #include <gz/transport/Node.hh>
// #include <gz/msgs/world_control.pb.h>
// #include <gz/msgs/boolean.pb.h>

// void resetAndPlaySimulation() {
//     gz::transport::Node node;
//     // Reset the world
//     {
//         gz::msgs::WorldControl reset_req;
//         reset_req.mutable_reset()->set_all(true);
//         gz::msgs::Boolean reset_res;
//         bool result;
//         node.Request("/world/default/control", reset_req, 3000, reset_res, result);
//     }
//     std::this_thread::sleep_for(std::chrono::milliseconds(500));
//     // Unpause the simulation
//     {
//         gz::msgs::WorldControl play_req;
//         play_req.set_pause(false);
//         gz::msgs::Boolean play_res;
//         bool result;
//         node.Request("/world/default/control", play_req, 3000, play_res, result);
//     }
// }


// class DubinsRobotSimulator {
// public:
//     // The simulator is initialized with the full trajectory
//     DubinsRobotSimulator(const Trajectory& trajectory)
//         : full_trajectory_(trajectory), elapsed_time_(0.0), current_segment_index_(0) {
        
//         if (!trajectory.is_valid || trajectory.path_points.empty()) {
//             throw std::runtime_error("Cannot simulate an invalid or empty trajectory.");
//         }
//         current_state_ = trajectory.path_points.front();
//     }

//     // Move the robot forward in time by a small step 'dt'
//     void update(double dt) {
//         if (isFinished()) {
//             return;
//         }
//         elapsed_time_ += dt;

//         // Ensure elapsed time doesn't exceed the total duration
//         elapsed_time_ = std::min(elapsed_time_, full_trajectory_.time_duration);

//         // Find which segment of the trajectory the robot is currently on
//         double time_at_segment_start = 0.0;
//         for (size_t i = 0; i < full_trajectory_.path_points.size() - 1; ++i) {
//             const auto& p1 = full_trajectory_.path_points[i];
//             const auto& p2 = full_trajectory_.path_points[i+1];
            
//             double segment_duration = p1(3) - p2(3); // Time flows backwards from start to goal
//             double time_at_segment_end = time_at_segment_start + segment_duration;

//             if (elapsed_time_ >= time_at_segment_start && elapsed_time_ <= time_at_segment_end) {
//                 current_segment_index_ = i;
//                 // Interpolate within this segment
//                 double segment_elapsed_time = elapsed_time_ - time_at_segment_start;
//                 double interp_factor = segment_elapsed_time / segment_duration;

//                 // Interpolate x, y, and theta
//                 current_state_.head<3>() = (1.0 - interp_factor) * p1.head<3>() + interp_factor * p2.head<3>();
//                 // The angle needs careful normalization
//                 double start_angle = p1(2);
//                 double end_angle = p2(2);
//                 double delta_angle = normalizeAngle(end_angle - start_angle);
//                 current_state_(2) = normalizeAngle(start_angle + interp_factor * delta_angle);

//                 // Interpolate the time-to-go
//                 current_state_(3) = p1(3) - segment_elapsed_time;
//                 return;
//             }
//             time_at_segment_start = time_at_segment_end;
//         }

//         // If loop finishes, we are at the goal
//         current_state_ = full_trajectory_.path_points.back();
//     }

//     Eigen::VectorXd getCurrentState() const {
//         return current_state_;
//     }
    
//     // Returns the remaining part of the trajectory from the robot's current state
//     Trajectory getRemainingTrajectory() const {
//         Trajectory remaining;
//         if (isFinished()) {
//             remaining.is_valid = false;
//             return remaining;
//         }

//         remaining.is_valid = true;
//         // The first point is the robot's current interpolated state
//         remaining.path_points.push_back(current_state_);

//         // Add all subsequent waypoints from the original path
//         for (size_t i = current_segment_index_ + 1; i < full_trajectory_.path_points.size(); ++i) {
//             remaining.path_points.push_back(full_trajectory_.path_points[i]);
//         }
        
//         // The remaining duration is simply the time-to-go from the current state
//         remaining.time_duration = current_state_(3);

//         return remaining;
//     }


//     bool isFinished() const {
//         return elapsed_time_ >= full_trajectory_.time_duration;
//     }

// private:
//     Trajectory full_trajectory_;
//     Eigen::VectorXd current_state_;
//     double elapsed_time_;
//     size_t current_segment_index_;

//     // Helper to normalize angles to [-PI, PI]
//     static double normalizeAngle(double angle) {
//         while (angle > M_PI) angle -= 2.0 * M_PI;
//         while (angle <= -M_PI) angle += 2.0 * M_PI;
//         return angle;
//     }
// };


// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);

//     // --- Basic Setup ---
//     auto node = std::make_shared<rclcpp::Node>("predictive_dubin_test_node", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
//     auto visualizer = std::make_shared<RVizVisualization>(node);
    
//     Params gazebo_params;
//     gazebo_params.setParam("robot_model_name", "tugbot");
//     gazebo_params.setParam("default_robot_x", 48.0);
//     gazebo_params.setParam("default_robot_y", 48.0);
//     gazebo_params.setParam("world_name", "default");
//     gazebo_params.setParam("use_range", false);
//     gazebo_params.setParam("sensor_range", 20.0);
//     gazebo_params.setParam("inflation", 0.5); 
//     gazebo_params.setParam("persistent_static_obstacles", true);
//     gazebo_params.setParam("estimation", true);
//     gazebo_params.setParam("kf_model_type", "cv");
//     gazebo_params.setParam("fcl", false);
//     gazebo_params.setParam("bullet", false);

//     auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world_4_obs.sdf");
//     auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(node->get_clock(), gazebo_params, obstacle_info);

//     // --- Define Robot's Path using DubinsTimeStateSpace ---
//     const double MIN_TURNING_RADIUS = 5.0; // meters
//     const double MIN_ROBOT_SPEED = 2.0;    // m/s
//     const double MAX_ROBOT_SPEED = 8.0;    // m/s
    
//     // Instantiate the Dubins Time-based state space
//     DubinsTimeStateSpace dubin_ss(MIN_TURNING_RADIUS, MIN_ROBOT_SPEED, MAX_ROBOT_SPEED);

//     // Define start and goal with a 4D state: (x, y, theta, time-to-go).
//     Eigen::VectorXd start_node(4);
//     start_node << 0.0,  0.0,  0.0, 17.0; // Start at (15,15), facing 45deg, with 15s to reach goal.

//     Eigen::VectorXd goal_node(4);
//     goal_node << 2.5 , 0.0,  M_PI,  0.0; // Goal at (-15,-15), facing -90deg, at the final moment (t=0).


//     // Generate the full, kinematically-aware trajectory
//     Trajectory full_trajectory = dubin_ss.steer(start_node, goal_node);

//     if (!full_trajectory.is_valid) {
//         RCLCPP_ERROR(node->get_logger(), "Failed to generate a valid Dubins trajectory!");
//         throw std::runtime_error("Could not steer between the provided start and goal states.");
//     }
    
//     double required_speed = full_trajectory.geometric_distance / full_trajectory.time_duration;
//     RCLCPP_INFO(node->get_logger(), "Dubins Path Generated! Distance: %.2f m, Time: %.2f s, Avg Speed: %.2f m/s",
//         full_trajectory.geometric_distance, full_trajectory.time_duration, required_speed);

//     // Create the simulator with the generated trajectory.
//     DubinsRobotSimulator simulator(full_trajectory);

//     // --- Run Simulation and Prediction Loop ---
//     resetAndPlaySimulation();
//     RCLCPP_INFO(node->get_logger(), "Starting simulation and prediction loop...");
    
//     rclcpp::Rate loop_rate(20); 
//     rclcpp::Clock::SharedPtr clock = node->get_clock();
//     rclcpp::Time last_update_time = clock->now();

//     while (rclcpp::ok() && !simulator.isFinished()) {
//         rclcpp::spin_some(node);

//         rclcpp::Time current_time = clock->now();
//         double dt = (current_time - last_update_time).seconds();
//         if (dt <= 0) { // Avoid issues on simulation reset
//             loop_rate.sleep();
//             continue;
//         }
//         last_update_time = current_time;

//         // Move the robot along the Dubins path
//         simulator.update(dt);
//         Eigen::VectorXd current_robot_state = simulator.getCurrentState();
        
//         // Get the remaining part of the path for collision checking
//         Trajectory remaining_trajectory = simulator.getRemainingTrajectory();
//         auto snapshot = obstacle_checker->getAtomicSnapshot();

//         // Check if the rest of the path is safe
//         bool is_safe = obstacle_checker->isTrajectorySafe(remaining_trajectory, current_time.seconds());

//         // --- Visualize the result ---
//         Eigen::Vector3d robot_pos_3d(current_robot_state(0), current_robot_state(1), 0.0);
//         double robot_yaw = current_robot_state(2); // Use theta from the state
//         Eigen::Quaterniond q(Eigen::AngleAxisd(robot_yaw, Eigen::Vector3d::UnitZ()));
//         Eigen::VectorXd orientation_quat(4);
//         orientation_quat << q.x(), q.y(), q.z(), q.w();

//         // Visualize robot arrow (green for safe, red for collision)
//         std::vector<float> robot_color = is_safe ? std::vector<float>{0.1f, 0.8f, 0.1f, 1.0f}
//                                                  : std::vector<float>{1.0f, 0.1f, 0.1f, 1.0f};
//         visualizer->visualizeRobotArrow(robot_pos_3d, orientation_quat, "map", robot_color, "simulated_robot");
        
//         // Visualize the remaining trajectory line strip
//         std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> remaining_path_edges;
//         for (size_t i = 0; i < remaining_trajectory.path_points.size() - 1; ++i) {
//             remaining_path_edges.push_back({remaining_trajectory.path_points[i], remaining_trajectory.path_points[i+1]});
//         }
//         std::string trajectory_color_str = is_safe ? "0.1,0.8,0.1" : "1.0,0.1,0.1";
//         visualizer->visualizeEdges(remaining_path_edges, "map", trajectory_color_str, "remaining_trajectory");

//         // Prepare separate containers for each shape type
//         std::vector<Eigen::VectorXd> cylinder_positions;
//         std::vector<double> cylinder_radii;
//         std::vector<std::tuple<Eigen::Vector2d, double, double, double>> box_data_for_viz;

//         // Sort obstacles by type
//         for (const auto& obs : snapshot.obstacles) {
//             if (obs.type == Obstacle::CIRCLE) {
//                 cylinder_positions.push_back(obs.position);
//                 cylinder_radii.push_back(obs.dimensions.radius + obs.inflation);
//             } else if (obs.type == Obstacle::BOX) {
//                 box_data_for_viz.emplace_back(
//                     obs.position,
//                     obs.dimensions.width + 2 * obs.inflation,
//                     obs.dimensions.height + 2 * obs.inflation,
//                     obs.dimensions.rotation
//                 );
//             }
//         }

//         // Call the correct visualization function for each shape
//         if (!cylinder_positions.empty()) {
//             visualizer->visualizeCylinder(cylinder_positions, cylinder_radii, "map", {0.0f, 0.4f, 1.0f}, "cylinder_obstacles");
//         }
//         if (!box_data_for_viz.empty()) {
//             visualizer->visualizeCube(box_data_for_viz, "map", {0.0f, 0.6f, 0.8f}, "box_obstacles");
//         }

//         loop_rate.sleep();
//     }
    
//     RCLCPP_INFO(node->get_logger(), "Simulation finished.");
//     rclcpp::shutdown();
//     return 0;
// }




// ////////////////////////////////////////////////////////////////////////////////////////////////
// //
// //  5D Thruster Predictive Collision Test
// //
// //  This test simulates a robot following a physically accurate second-order 
// //  trajectory and continuously checks the remainder of its path for
// //  collisions with dynamic obstacles from a Gazebo simulation.
// //
// //  It serves as a validation for the GazeboObstacleChecker's ability to correctly
// //  predict collisions for non-linear paths.
// //
// ////////////////////////////////////////////////////////////////////////////////////////////////

// #include "rclcpp/rclcpp.hpp"
// #include "motion_planning/utils/gazebo_obstacle_checker.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"
// #include "motion_planning/utils/params.hpp"
// #include "motion_planning/ds/edge_info.hpp"
// #include "motion_planning/utils/parse_sdf.hpp"
// #include "motion_planning/state_space/thruster_statespace.hpp"

// #include <gz/transport/Node.hh>
// #include <gz/msgs/world_control.pb.h>
// #include <gz/msgs/boolean.pb.h>
// #include <cmath>
// #include <sstream>
// #include <iomanip>

// // Helper function to reset and play the Gazebo simulation
// void resetAndPlaySimulation() {
//     gz::transport::Node node;
//     // Reset the world
//     {
//         gz::msgs::WorldControl reset_req;
//         reset_req.mutable_reset()->set_all(true);
//         gz::msgs::Boolean reset_res;
//         bool result;
//         node.Request("/world/default/control", reset_req, 3000, reset_res, result);
//     }
//     std::this_thread::sleep_for(std::chrono::milliseconds(500));
//     // Unpause the simulation
//     {
//         gz::msgs::WorldControl play_req;
//         play_req.set_pause(false);
//         gz::msgs::Boolean play_res;
//         bool result;
//         node.Request("/world/default/control", play_req, 3000, play_res, result);
//     }
// }


// class ThrusterRobotSimulator {
// public:
//     // The simulator is initialized with the full, fine-grained trajectory
//     ThrusterRobotSimulator(const Trajectory& trajectory)
//         : full_trajectory_(trajectory), elapsed_time_(0.0) {
        
//         if (!trajectory.is_valid || trajectory.path_points.empty()) {
//             throw std::runtime_error("Cannot simulate an invalid or empty trajectory.");
//         }
//         current_state_ = trajectory.path_points.front();
//         total_duration_ = trajectory.time_duration;
//     }

//     // Move the robot forward in time by a small step 'dt'
//     void update(double dt) {
//         if (isFinished()) {
//             return;
//         }
//         elapsed_time_ += dt;
//         elapsed_time_ = std::min(elapsed_time_, total_duration_);

//         // Find the correct segment in the high-resolution path
//         // The path is ordered from high time-to-go to low time-to-go
//         double target_time_to_go = total_duration_ - elapsed_time_;

//         // Find the first point in the path with time <= target_time_to_go
//         auto it = std::lower_bound(full_trajectory_.path_points.begin(), full_trajectory_.path_points.end(), target_time_to_go,
//             [](const Eigen::VectorXd& point, double time_val) {
//                 return point(4) > time_val; // Compare based on time-to-go (last element)
//             });

//         if (it == full_trajectory_.path_points.begin()) {
//             current_state_ = full_trajectory_.path_points.front();
//             return;
//         }
//         if (it == full_trajectory_.path_points.end()) {
//             current_state_ = full_trajectory_.path_points.back();
//             return;
//         }
        
//         // Linear interpolation between the two closest points in the fine-grained path
//         const Eigen::VectorXd& p_after = *it;
//         const Eigen::VectorXd& p_before = *(it - 1);

//         double time_before = p_before(4);
//         double time_after = p_after(4);
//         double segment_duration = time_before - time_after;
        
//         if (segment_duration <= 1e-9) {
//             current_state_ = p_after;
//         } else {
//             double time_into_segment = time_before - target_time_to_go;
//             double interp_factor = std::clamp(time_into_segment / segment_duration, 0.0, 1.0);
//             current_state_ = (1.0 - interp_factor) * p_before + interp_factor * p_after;
//         }
//     }

//     Eigen::VectorXd getCurrentState() const {
//         return current_state_;
//     }

//     // Returns the remaining part of the trajectory from the robot's current state
//     Trajectory getRemainingTrajectory() const {
//         Trajectory remaining;
//         if (isFinished()) {
//             remaining.is_valid = false;
//             return remaining;
//         }

//         remaining.is_valid = true;
//         remaining.path_points.push_back(current_state_); // Start with the current interpolated state

//         // Find the first point in the original path that is "in the future" of our current state
//         auto it = std::lower_bound(full_trajectory_.path_points.begin(), full_trajectory_.path_points.end(), current_state_(4),
//             [](const Eigen::VectorXd& point, double time_val) {
//                 return point(4) > time_val;
//             });
        
//         // Add all subsequent waypoints
//         remaining.path_points.insert(remaining.path_points.end(), it, full_trajectory_.path_points.end());
        
//         remaining.time_duration = current_state_(4); // Remaining duration is the current time-to-go
//         return remaining;
//     }

//     bool isFinished() const {
//         return elapsed_time_ >= total_duration_;
//     }

// private:
//     Trajectory full_trajectory_;
//     Eigen::VectorXd current_state_;
//     double elapsed_time_;
//     double total_duration_;
// };


// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);

//     // --- Basic Setup ---
//     auto node = std::make_shared<rclcpp::Node>("predictive_thruster_test_node", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
//     auto visualizer = std::make_shared<RVizVisualization>(node);
    
//     Params gazebo_params;
//     gazebo_params.setParam("robot_model_name", "tugbot");
//     gazebo_params.setParam("default_robot_x", 48.0);
//     gazebo_params.setParam("default_robot_y", 48.0);
//     gazebo_params.setParam("world_name", "default");
//     gazebo_params.setParam("use_range", false);
//     gazebo_params.setParam("sensor_range", 20.0);
//     gazebo_params.setParam("inflation", 0.5); 
//     gazebo_params.setParam("persistent_static_obstacles", true);
//     gazebo_params.setParam("estimation", true);
//     gazebo_params.setParam("kf_model_type", "cv");
//     gazebo_params.setParam("fcl", false);
//     gazebo_params.setParam("bullet", true);
    
//     auto obstacle_info = parseSdfObstacles("dynamic_world_1_obs.sdf");
//     auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(node->get_clock(), gazebo_params, obstacle_info);

//     // --- 2. Define Robot's Path using ThrusterSteerStateSpace ---
//     const int dim = 5;
//     const double max_acceleration = 2.0; // m/s^2
//     auto thruster_ss = std::make_shared<ThrusterSteerStateSpace>(dim, max_acceleration);

//     std::vector<Eigen::VectorXd> waypoints;
//     waypoints.push_back((Eigen::VectorXd(dim) << 15.0, 15.0, 0.0, 0.0, 15.0).finished());
//     waypoints.push_back((Eigen::VectorXd(dim) << -15.0, -15.0, 5.0, -5.0, 0.0).finished());
    
//     // waypoints.push_back((Eigen::VectorXd(dim) << -20.0, 0.0, 10.0, 2.0, 20.0).finished());
//     // waypoints.push_back((Eigen::VectorXd(dim) << 0.0, -10.0, 2.0, -8.0, 0.0).finished());


//     Trajectory full_trajectory;
//     full_trajectory.is_valid = true;
//     std::vector<Trajectory> segments;

//     for (size_t i = 0; i < waypoints.size() - 1; ++i) {
//         RCLCPP_INFO(node->get_logger(), "--------------------------------------------------");
//         RCLCPP_INFO(node->get_logger(), "Attempting to steer segment %zu:", i);
//         RCLCPP_INFO(node->get_logger(), "  FROM state: [%.2f, %.2f, %.2f, %.2f, %.2f]",
//             waypoints[i](0), waypoints[i](1), waypoints[i](2), waypoints[i](3), waypoints[i](4));
//         RCLCPP_INFO(node->get_logger(), "  TO   state: [%.2f, %.2f, %.2f, %.2f, %.2f]",
//             waypoints[i+1](0), waypoints[i+1](1), waypoints[i+1](2), waypoints[i+1](3), waypoints[i+1](4));

//         Trajectory segment_traj = thruster_ss->steer(waypoints[i], waypoints[i+1]);

//         if (!segment_traj.is_valid) {
//             RCLCPP_ERROR(node->get_logger(), "  RESULT: FAILED to steer segment %zu. Aborting.", i);
//             full_trajectory.is_valid = false;
//             break;
//         }
        
//         RCLCPP_INFO(node->get_logger(), "  RESULT: SUCCESS. Segment %zu generated with %zu points.", i, segment_traj.path_points.size());
//         segments.push_back(segment_traj);
//     }
    
//     // Stitch the valid segments into a single trajectory
//     if (full_trajectory.is_valid) {
//         for (size_t i = 0; i < segments.size(); ++i) {
//             const auto& seg = segments[i];
//             long start_j = (i == 0) ? 0 : 1; // Skip first point on subsequent segments
//             for (long j = start_j; j < seg.path_points.size(); ++j) {
//                 full_trajectory.path_points.push_back(seg.path_points[j]);
//             }
//         }
//         full_trajectory.time_duration = waypoints.front()(dim-1) - waypoints.back()(dim-1);
//     }

//     if (!full_trajectory.is_valid || !std::isfinite(full_trajectory.time_duration)) {
//         RCLCPP_ERROR(node->get_logger(), "Failed to generate a valid Thruster trajectory with finite time! Exiting test.");
//         rclcpp::shutdown();
//         return EXIT_FAILURE;
//     }
    
//     RCLCPP_INFO(node->get_logger(), "--------------------------------------------------");
//     RCLCPP_INFO(node->get_logger(), "Final Thruster Path Generated! Total Time: %.2f s, Total Points: %zu", 
//                 full_trajectory.time_duration, full_trajectory.path_points.size());
    
//     // Detailed printout and validation of the final stitched path
//     RCLCPP_INFO(node->get_logger(), "--- Verifying Final Stitched Trajectory ---");
//     for (size_t i = 0; i < full_trajectory.path_points.size(); ++i) {
//         const auto& p = full_trajectory.path_points[i];
//         RCLCPP_INFO(node->get_logger(), "  Point %3zu: P=[%8.3f, %8.3f], V=[%8.3f, %8.3f], T=[%8.3f]",
//             i, p(0), p(1), p(2), p(3), p(4));
//         if (i > 0) {
//             const auto& p_prev = full_trajectory.path_points[i-1];
//             if (p(4) >= p_prev(4)) {
//                 RCLCPP_WARN(node->get_logger(), "    [WARNING] Time is not strictly decreasing! Prev T: %.3f, Curr T: %.3f", p_prev(4), p(4));
//             }
//         }
//     }
//     RCLCPP_INFO(node->get_logger(), "--- End of Trajectory Verification ---");


//     // Create the simulator with the generated trajectory.
//     ThrusterRobotSimulator simulator(full_trajectory);

//     // --- Run Simulation and Prediction Loop ---
//     resetAndPlaySimulation();
//     RCLCPP_INFO(node->get_logger(), "Starting simulation and prediction loop...");
    
//     rclcpp::Rate loop_rate(20); 
//     rclcpp::Clock::SharedPtr clock = node->get_clock();
//     rclcpp::Time last_update_time = clock->now();

//     while (rclcpp::ok() && !simulator.isFinished()) {
//         rclcpp::spin_some(node);

//         rclcpp::Time current_time = clock->now();
//         double dt = (current_time - last_update_time).seconds();
//         if (dt <= 0) { // Avoid issues on simulation reset
//             loop_rate.sleep();
//             continue;
//         }
//         last_update_time = current_time;

//         // Move the robot along the curved path
//         simulator.update(dt);
//         Eigen::VectorXd current_robot_state = simulator.getCurrentState();
        
//         // Get the remaining part of the path for collision checking
//         Trajectory remaining_trajectory = simulator.getRemainingTrajectory();

//         auto snapshot = obstacle_checker->getAtomicSnapshot();
//         // Check if the rest of the path is safe
//         bool is_safe = obstacle_checker->isTrajectorySafe(remaining_trajectory, current_time.seconds());

//         // --- Visualization ---
//         Eigen::Vector3d robot_pos_3d(current_robot_state(0), current_robot_state(1), 0.0);
//         Eigen::Vector2d robot_vel_2d(current_robot_state(2), current_robot_state(3));
        
//         Eigen::Vector2d forward_velocity = robot_vel_2d;

//         double robot_yaw = 0.0; 
        
//         if (forward_velocity.norm() > 1e-4) {
//             robot_yaw = atan2(forward_velocity.y(), forward_velocity.x());
//         } else if (!remaining_trajectory.path_points.empty() && remaining_trajectory.path_points.size() > 1) {
//             Eigen::Vector2d next_pos(remaining_trajectory.path_points[1](0), remaining_trajectory.path_points[1](1));
//             Eigen::Vector2d current_pos(current_robot_state(0), current_robot_state(1));
//             Eigen::Vector2d direction = next_pos - current_pos;
//             if (direction.norm() > 1e-4) {
//                 robot_yaw = atan2(direction.y(), direction.x());
//             }
//         }
        
//         Eigen::Quaterniond q(Eigen::AngleAxisd(robot_yaw, Eigen::Vector3d::UnitZ()));
//         Eigen::VectorXd orientation_quat(4);
//         orientation_quat << q.x(), q.y(), q.z(), q.w();

//         // Visualize robot arrow (green for safe, red for collision)
//         std::vector<float> robot_color = is_safe ? std::vector<float>{0.1f, 0.8f, 0.1f, 1.0f}
//                                                  : std::vector<float>{1.0f, 0.1f, 0.1f, 1.0f};
//         visualizer->visualizeRobotArrow(robot_pos_3d, orientation_quat, "map", robot_color, "simulated_robot");
        
//         // Visualize the remaining trajectory line strip
//         std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> remaining_path_edges;
//         for (size_t i = 0; i < remaining_trajectory.path_points.size() - 1; ++i) {
//             remaining_path_edges.push_back({remaining_trajectory.path_points[i], remaining_trajectory.path_points[i+1]});
//         }
//         std::string trajectory_color_str = is_safe ? "0.1,0.8,0.1" : "1.0,0.1,0.1";
//         visualizer->visualizeEdges(remaining_path_edges, "map", trajectory_color_str, "remaining_trajectory");

//         // Prepare separate containers for each shape type
//         std::vector<Eigen::VectorXd> cylinder_positions;
//         std::vector<double> cylinder_radii;
//         std::vector<std::tuple<Eigen::Vector2d, double, double, double>> box_data_for_viz;

//         // Sort obstacles by type
//         for (const auto& obs : snapshot.obstacles) {
//             if (obs.type == Obstacle::CIRCLE) {
//                 cylinder_positions.push_back(obs.position);
//                 cylinder_radii.push_back(obs.dimensions.radius + obs.inflation);
//             } else if (obs.type == Obstacle::BOX) {
//                 box_data_for_viz.emplace_back(
//                     obs.position,
//                     obs.dimensions.width + 2 * obs.inflation,
//                     obs.dimensions.height + 2 * obs.inflation,
//                     obs.dimensions.rotation
//                 );
//             }
//         }

//         // Call the correct visualization function for each shape
//         if (!cylinder_positions.empty()) {
//             visualizer->visualizeCylinder(cylinder_positions, cylinder_radii, "map", {0.0f, 0.4f, 1.0f}, "cylinder_obstacles");
//         }
//         if (!box_data_for_viz.empty()) {
//             visualizer->visualizeCube(box_data_for_viz, "map", {0.0f, 0.6f, 0.8f}, "box_obstacles");
//         }
        
//         loop_rate.sleep();
//     }
    
//     RCLCPP_INFO(node->get_logger(), "Simulation finished.");
//     rclcpp::shutdown();
//     return 0;
// }



////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
//
//  5D Min-Snap Predictive Collision Test (Corrected)
//
//  This test simulates a robot following a physically accurate minimum-snap
//  trajectory in 3D space. It continuously checks the remainder of its path for
//  collisions with dynamic obstacles from a Gazebo simulation.
//
////////////////////////////////////////////////////////////////////////////////////////////////

#include "rclcpp/rclcpp.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/params.hpp"
#include "motion_planning/ds/edge_info.hpp"
#include "motion_planning/utils/parse_sdf.hpp"
#include "motion_planning/state_space/min_snap_statespace.hpp"

#include <gz/transport/Node.hh>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <cmath>
#include <sstream>
#include <iomanip>

// Helper function to reset and play the Gazebo simulation
void resetAndPlaySimulation() {
    gz::transport::Node node;
    {
        gz::msgs::WorldControl reset_req;
        reset_req.mutable_reset()->set_all(true);
        gz::msgs::Boolean reset_res;
        bool result;
        node.Request("/world/default/control", reset_req, 3000, reset_res, result);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    {
        gz::msgs::WorldControl play_req;
        play_req.set_pause(false);
        gz::msgs::Boolean play_res;
        bool result;
        node.Request("/world/default/control", play_req, 3000, play_res, result);
    }
}

// Helper to evaluate polynomial basis functions, needed for the new simulator
static inline Eigen::RowVectorXd basis(int deriv, double tau, int num_coeffs) {
    Eigen::RowVectorXd r = Eigen::RowVectorXd::Zero(num_coeffs);
    for (int i = deriv; i < num_coeffs; ++i) {
        double coeff = 1.0;
        for (int k = 0; k < deriv; ++k) coeff *= (i - k);
        r(i) = coeff * std::pow(tau, i - deriv);
    }
    return r;
}

// NEW, ROBUST SIMULATOR CLASS
class MinSnapRobotSimulator {
public:
    MinSnapRobotSimulator(const Trajectory& trajectory)
        : full_trajectory_(trajectory), elapsed_time_(0.0) {
        
        if (!trajectory.is_valid || trajectory.coeffs_per_axis.empty()) {
            throw std::runtime_error("Cannot simulate an invalid trajectory or one without coefficients.");
        }
        total_duration_ = trajectory.time_duration;
        // Evaluate the start state at tau=0
        updateStateAt(0.0);
    }

    void update(double dt) {
        if (isFinished()) return;
        elapsed_time_ += dt;
        elapsed_time_ = std::min(elapsed_time_, total_duration_);
        
        // Calculate normalized time `tau` from 0 (start) to 1 (end)
        double tau = elapsed_time_ / total_duration_;
        updateStateAt(tau);
    }

    Eigen::VectorXd getCurrentState() const {
        return current_state_;
    }

    // This function is no longer needed as we check the full original trajectory
    // from the robot's current time onwards, which is more accurate.
    
    bool isFinished() const {
        return elapsed_time_ >= total_duration_;
    }

private:
    void updateStateAt(double tau) {
        const int num_axes = 4; // x, y, z, yaw
        const int num_coeffs = full_trajectory_.coeffs_per_axis[0].size();
        const double T = total_duration_;

        current_state_.resize(5);
        for (int axis = 0; axis < num_axes; ++axis) {
            const auto& coeffs = full_trajectory_.coeffs_per_axis[axis];
            current_state_(axis) = (coeffs.transpose() * basis(0, tau, num_coeffs).transpose())(0);
        }
        current_state_(3) = fmod(current_state_(3), 2.0 * M_PI); // Normalize yaw
        current_state_(4) = total_duration_ - elapsed_time_; // Current time-to-go
    }

    Trajectory full_trajectory_;
    Eigen::VectorXd current_state_;
    double elapsed_time_;
    double total_duration_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("predictive_minsnap_test_node", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
    auto visualizer = std::make_shared<RVizVisualization>(node);
    
    Params gazebo_params;
    gazebo_params.setParam("robot_model_name", "tugbot");
    gazebo_params.setParam("default_robot_x", 48.0);
    gazebo_params.setParam("default_robot_y", 48.0);
    gazebo_params.setParam("world_name", "default");
    gazebo_params.setParam("use_range", false);
    gazebo_params.setParam("sensor_range", 20.0);
    gazebo_params.setParam("inflation", 0.5); 
    gazebo_params.setParam("persistent_static_obstacles", true);
    gazebo_params.setParam("estimation", true);
    gazebo_params.setParam("kf_model_type", "cv");
    gazebo_params.setParam("fcl", false);
    gazebo_params.setParam("bullet", true);
    gazebo_params.setParam("spatial_dim", 3);
    
    auto obstacle_info = parseSdfObstacles("dynamic_world_1_obs_3D.sdf");
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(node->get_clock(), gazebo_params, obstacle_info);

    auto min_snap_ss = std::make_shared<MinSnapStateSpace>(5, 20.0, 7.0, 0.1, 0.5, 1.0, 42);

    Eigen::VectorXd start_node(5);
    start_node << 15.0, 15.0, 2.0, 0.0, 20.0;
    Eigen::VectorXd goal_node(5);
    goal_node << -15.0, -15.0, 10.0, M_PI, 0.0;

    // --- DEFINE INITIAL AND FINAL DERIVATIVES FOR A CURVED PATH ---
    Eigen::VectorXd v_start(4); v_start << -2.0, -5.0, 3.0, 0.1; // Initial velocity (vx, vy, vz, vyaw)
    Eigen::VectorXd a_start = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd v_goal = Eigen::VectorXd::Zero(4); // Come to a stop at the goal
    Eigen::VectorXd a_goal = Eigen::VectorXd::Zero(4);

    // Call the steer function that generates a curved trajectory
    Trajectory full_trajectory = min_snap_ss->steer(start_node, goal_node, v_start, a_start, v_goal, a_goal);

    if (!full_trajectory.is_valid) {
        RCLCPP_ERROR(node->get_logger(), "Failed to generate a valid Min-Snap trajectory!");
        return EXIT_FAILURE;
    }
    
    RCLCPP_INFO(node->get_logger(), "Final Min-Snap Path Generated! Total Time: %.2f s, Points: %zu", 
                full_trajectory.time_duration, full_trajectory.path_points.size());

    MinSnapRobotSimulator simulator(full_trajectory);

    resetAndPlaySimulation();
    RCLCPP_INFO(node->get_logger(), "Starting simulation and prediction loop...");
    
    rclcpp::Rate loop_rate(20); 
    rclcpp::Clock::SharedPtr clock = node->get_clock();
    rclcpp::Time last_update_time = clock->now();

    while (rclcpp::ok() && !simulator.isFinished()) {
        rclcpp::spin_some(node);

        rclcpp::Time current_time = clock->now();
        double dt = (current_time - last_update_time).seconds();
        if (dt <= 0) { loop_rate.sleep(); continue; }
        last_update_time = current_time;

        simulator.update(dt);
        Eigen::VectorXd current_robot_state = simulator.getCurrentState();
        
        Trajectory remaining_trajectory;
        remaining_trajectory.is_valid = true;
        
        // The first point is the robot's current interpolated state.
        remaining_trajectory.path_points.push_back(current_robot_state);

        // Find the first point in the original path that is "in the future" of our current state.
        auto it = std::lower_bound(full_trajectory.path_points.begin(), full_trajectory.path_points.end(), current_robot_state(4),
            [](const Eigen::VectorXd& point, double time_val) {
                return point(4) > time_val;
            });
        
        // Add all subsequent waypoints from the original path.
        remaining_trajectory.path_points.insert(remaining_trajectory.path_points.end(), it, full_trajectory.path_points.end());
        
        // Set the duration for the remaining path.
        remaining_trajectory.time_duration = current_robot_state(4);
        
        // Now, check only the relevant future part of the path.
        auto snapshot = obstacle_checker->getAtomicSnapshot();
        bool is_safe = obstacle_checker->isTrajectorySafe(remaining_trajectory, current_time.seconds());

        
        // --- Visualization ---
        Eigen::Vector3d robot_pos_3d(current_robot_state(0), current_robot_state(1), current_robot_state(2));
        double robot_yaw = current_robot_state(3);
        
        Eigen::Quaterniond q(Eigen::AngleAxisd(robot_yaw, Eigen::Vector3d::UnitZ()));
        Eigen::VectorXd orientation_quat(4);
        orientation_quat << q.x(), q.y(), q.z(), q.w();

        std::vector<float> robot_color = is_safe ? std::vector<float>{0.1f, 0.8f, 0.1f, 1.0f}
                                                 : std::vector<float>{1.0f, 0.1f, 0.1f, 1.0f};
        visualizer->visualizeQuadcopter(robot_pos_3d, orientation_quat, "map", robot_color, "simulated_robot");
        
        // Visualize the full trajectory path
        std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> path_edges;
        for (size_t i = 0; i < full_trajectory.path_points.size() - 1; ++i) {
            path_edges.push_back({full_trajectory.path_points[i], full_trajectory.path_points[i+1]});
        }
        std::string trajectory_color_str = is_safe ? "0.1,0.8,0.1" : "1.0,0.1,0.1";
        visualizer->visualizeEdges(path_edges, "map", trajectory_color_str, "full_trajectory");

        // Visualize obstacles as spheres
        std::vector<Eigen::VectorXd> sphere_positions;
        std::vector<double> sphere_radii;
        for (const auto& obs : snapshot.obstacles) {
            if (obs.type == Obstacle::CIRCLE) {
                sphere_positions.push_back((Eigen::Vector3d() << obs.position.x(), obs.position.y(), obs.z).finished());
                sphere_radii.push_back(obs.dimensions.radius + obs.inflation);
            }
        }
        if (!sphere_positions.empty()) {
            visualizer->visualizeSpheres(sphere_positions, sphere_radii, "map", {0.0f, 0.4f, 1.0f}, "sphere_obstacles");
        }
        
        loop_rate.sleep();
    }
    
    RCLCPP_INFO(node->get_logger(), "Simulation finished.");
    rclcpp::shutdown();
    return 0;
}