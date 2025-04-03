// Copyright 2025 Soheil E.nia
#pragma once

#include "motion_planning/utils/obstacle_checker.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/occupancygrid_obstacle_checker.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "geometry_msgs/msg/twist.hpp"  // Include the Twist message type
#include "geometry_msgs/msg/pose_stamped.hpp"


#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/utils.h>  // Add this line


struct RobotPose {
    Eigen::Vector2d position;  // 2D position (x, y)
    Eigen::VectorXd orientation;  // Orientation as a quaternion (x, y, z, w)
};


class ROS2Manager : public rclcpp::Node {
public:
    ROS2Manager(
        std::shared_ptr<ObstacleChecker> obstacle_checker, // Use the base class
        std::shared_ptr<RVizVisualization> visualizer)
        : Node("ros2_manager"),
          obstacle_checker_(obstacle_checker),
          visualizer_(visualizer),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tf_listener_(*tf_buffer_)
    {
        // Only subscribe to the map topic if the obstacle checker uses it
        if (std::dynamic_pointer_cast<OccupancyGridObstacleChecker>(obstacle_checker_)) {
            map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", 10, std::bind(&ROS2Manager::mapCallback, this, std::placeholders::_1));
        }

        // Timer for periodic updates (e.g., for Gazebo obstacle checker)
        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), // 10 Hz
            std::bind(&ROS2Manager::updateCallback, this));


        // Create a publisher for cmd_vel
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Subscribe to the goal pose topic
        goal_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&ROS2Manager::goalPoseCallback, this, std::placeholders::_1));



        control_loop_dt_ = std::chrono::milliseconds(50);
        if (follow_path == true)
        {
            if(use_nav == true) {
                // Initialize FollowPath action client
                follow_path_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(
                    this, "follow_path");
            }
            else if (use_pure_pursuit ==true){
                // Create a timer for the control loop
                control_timer_ = this->create_wall_timer(
                    control_loop_dt_,  // 10 Hz
                    std::bind(&ROS2Manager::controlLoop, this));
            }



        }

    }

    // Callback to store the latest goal pose
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(goal_pose_mutex_);
        latest_goal_pose_ = *msg;
        goal_received_ = true; // Set the flag to true
        goal_cv_.notify_all(); // Notify all waiting threads
    }

    // Function to get the latest goal pose as a start position
    Eigen::VectorXd getStartPosition() {
        std::cout<<"WIAT \n";
        std::unique_lock<std::mutex> lock(goal_pose_mutex_);
        // Wait until the goal pose is received
        goal_cv_.wait(lock, [this]() { return goal_received_; });
        std::cout<<" NOT WIAT \n";
        Eigen::VectorXd start_position(2);
        start_position << latest_goal_pose_.pose.position.x, latest_goal_pose_.pose.position.y;
        return start_position;
    }


    // void triggerReplanning() {
    //     // Lock the start position mutex
    //     std::lock_guard<std::mutex> lock(start_position_mutex_);

    //     // Update the problem definition with the new start position
    //     problem_def_->setStart(start_position_);

    //     // Replan
    //     planner_->plan();
    // }


    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // Update the grid in the obstacle checker (if it uses the grid)
        dynamic_cast<OccupancyGridObstacleChecker*>(obstacle_checker_.get())->updateGrid(msg);

        // Get robot pose from TF
        updateRobotPose();
    }

    void updateCallback() {
        // Get robot pose from TF
        updateRobotPose();
    }

    // Function to set linear and angular velocity and publish to cmd_vel topic
    void setCmdVel(double linear_x, double angular_z) {
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = linear_x;
        twist_msg->angular.z = angular_z;

        // Publish the Twist message to the cmd_vel topic
        cmd_vel_publisher_->publish(std::move(twist_msg));
    }

    /////////////////FOR nav2 ///// you have to have a map
    nav_msgs::msg::Path createPathFromNodes(const std::vector<Eigen::VectorXd>& nodes) {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = now();

        for (const auto& node : nodes) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = node[0];
            pose.pose.position.y = node[1];
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        return path;
    }

    bool isPathSignificantlyDifferent(
        const std::vector<Eigen::VectorXd>& new_path,
        const std::vector<Eigen::VectorXd>& old_path,
        double distance_threshold = 0.5) 
    {
        if (new_path.size() != old_path.size()) return true;
    
        for (size_t i = 0; i < new_path.size(); ++i) {
            double dx = new_path[i][0] - old_path[i][0];
            double dy = new_path[i][1] - old_path[i][1];
            if (std::sqrt(dx * dx + dy * dy) > distance_threshold) {
                return true;
            }
        }
        return false;
    }


    void sendPathToNav2(const std::vector<Eigen::VectorXd>& new_path) {
        static std::vector<Eigen::VectorXd> last_sent_path;
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr goal_handle;

        // Check if the new path is significantly different
        if (!isPathSignificantlyDifferent(new_path, last_sent_path)) {
            return; // No need to update Nav2
        }

        // Cancel the current goal if it exists
        if (goal_handle) {
            follow_path_client_->async_cancel_goal(goal_handle);
            RCLCPP_INFO(get_logger(), "Canceling current Nav2 goal due to path changes");
        }

        // Send the new path
        auto path = createPathFromNodes(new_path);
        auto goal_msg = nav2_msgs::action::FollowPath::Goal();
        goal_msg.path = path;
        goal_msg.controller_id = "";  // Use default controller

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            [this, &goal_handle](auto future) {
                auto goal_handle_ptr = future.get();  // Get the raw pointer
                if (!goal_handle_ptr) {
                    RCLCPP_ERROR(get_logger(), "Goal rejected");
                } else {
                    // Wrap the raw pointer in a shared_ptr
                    goal_handle = std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>>(goal_handle_ptr);
                    RCLCPP_INFO(get_logger(), "Nav2 is following the updated path");
                }
            };

        send_goal_options.result_callback = 
            [this](const auto& result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(get_logger(), "Path execution complete!");
                } else {
                    RCLCPP_WARN(get_logger(), "Path execution failed or was canceled");
                }
            };

        follow_path_client_->async_send_goal(goal_msg, send_goal_options);
        last_sent_path = new_path; // Update the last sent path
    }


    //////////////////// for simple pure pursuit ////
    // void followPath(const std::vector<Eigen::VectorXd>& path) {
    //     current_path_ = path;
    // }
    void followPath(const std::vector<Eigen::VectorXd>& path) {
        if (path.empty()) {
            current_path_.clear();
            return;
        }
        current_path_ = path;
        // // Get the robot's current position
        // RobotPose robot_pose = getRobotPose();
        // Eigen::Vector2d robot_pos = robot_pose.position;

        // // Find the first point beyond a minimum distance (e.g., 0.5 meters)
        // const double min_truncate_distance = 1.0; // Tune this value
        // size_t start_index = 0;
        // for (size_t i = 0; i < path.size(); ++i) {
        //     double dx = path[i][0] - robot_pos[0];
        //     double dy = path[i][1] - robot_pos[1];
        //     double dist = std::sqrt(dx * dx + dy * dy);
        //     if (dist >= min_truncate_distance) {
        //         start_index = i;
        //         break;
        //     }
        // }

        // // Assign the truncated path
        // current_path_.assign(path.begin() + start_index, path.end());

        progress_index_ = 0; // Reset progress on new path
    }

    // double calculateCurvature(const Eigen::Vector2d& robot_position, 
    //     const Eigen::Vector2d& lookahead_point) {
    //     double dx = lookahead_point[0] - robot_position[0];
    //     double dy = lookahead_point[1] - robot_position[1];
    //     double distance = std::hypot(dx, dy);
    //     double curvature = (distance == 0) ? 0.0 : 2.0 * std::abs(dy) / (distance * distance);
    //     return curvature;
    // }

    double calculatePathCurvature(size_t lookahead_index) {
        const int num_points = 3;
        if (lookahead_index < num_points) return 0.0;

        std::vector<Eigen::Vector2d> points;
        for (int i = 0; i < num_points; ++i) {
            points.push_back(current_path_[lookahead_index - i].head<2>());
        }

        // Calculate curvature using three consecutive points
        double dx1 = points[1].x() - points[0].x();
        double dy1 = points[1].y() - points[0].y();
        double dx2 = points[2].x() - points[1].x();
        double dy2 = points[2].y() - points[1].y();
        
        double cross = dx1*dy2 - dy1*dx2;
        double norm1 = std::hypot(dx1, dy1);
        double norm2 = std::hypot(dx2, dy2);
        
        return std::abs(cross) / (norm1 * norm2 * norm2); // Approximate curvature
    }

    RobotPose getRobotPose() {
        RobotPose pose;

        // Get the robot's pose from Gazebo or TF
        if (auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker_)) {
            pose.position = gazebo_checker->getRobotPosition();  // Get 2D position
            pose.orientation = gazebo_checker->getRobotOrientation();  // Get orientation as quaternion
        } else {
            // Fallback to TF (if using a real robot)
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

                // Extract position
                pose.position = Eigen::Vector2d(transform.transform.translation.x, transform.transform.translation.y);

                // Extract orientation (quaternion)
                pose.orientation = Eigen::VectorXd(4);
                pose.orientation << transform.transform.rotation.x,
                                    transform.transform.rotation.y,
                                    transform.transform.rotation.z,
                                    transform.transform.rotation.w;
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
                pose.position = Eigen::Vector2d(0.0, 0.0);
                pose.orientation = Eigen::VectorXd::Zero(4);  // Default orientation (identity quaternion)
            }
        }

        return pose;
    }



    size_t findClosestPoint(const Eigen::Vector2d& robot_position) {
        size_t closest_index = 0;
        double min_distance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < current_path_.size(); ++i) {
            double dx = current_path_[i][0] - robot_position[0];
            double dy = current_path_[i][1] - robot_position[1];
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance < min_distance) {
                min_distance = distance;
                closest_index = i;
            }
        }

        return closest_index;
    }

    size_t findLookaheadPoint(const Eigen::Vector2d& robot_position, size_t closest_index) {
        /*
            The findLookaheadPoint logic may fail if the closest point is behind the robot. Search from the start of the truncated path instead of closest_index:
        */

        // for (size_t i = closest_index; i < current_path_.size(); ++i) {
        //     double dx = current_path_[i][0] - robot_position[0];
        //     double dy = current_path_[i][1] - robot_position[1];
        //     double distance = std::sqrt(dx * dx + dy * dy);

        //     if (distance >= lookahead_distance_) {
        //         return i;
        //     }
        // }
        // return current_path_.size() - 1;



        for (int i = static_cast<int>(closest_index); i >= 0; --i) {
            size_t idx = static_cast<size_t>(i);
            double dx = current_path_[idx][0] - robot_position[0];
            double dy = current_path_[idx][1] - robot_position[1];
            double distance = std::sqrt(dx * dx + dy * dy);
            if (distance >= lookahead_distance_) {
                // std::vector<Eigen::VectorXd> look_position_;
                // Eigen::VectorXd vec(2);
                // // vec << obstacle_pos.x(), obstacle_pos.y();
                // vec << current_path_[idx];
                // look_position_.push_back(vec);
                // visualizer_->visualizeNodes(look_position_, "map");
                // RCLCPP_INFO(this->get_logger(), "look index 1: %li   ", i);
                return idx;
            }
        }

        // int idx = 0
        // std::vector<Eigen::VectorXd> look_position_;
        // Eigen::VectorXd vec(2);
        // // vec << obstacle_pos.x(), obstacle_pos.y();
        // vec << current_path_[idx];
        // look_position_.push_back(vec);
        // visualizer_->visualizeNodes(look_position_, "map");
        // RCLCPP_INFO(this->get_logger(), "look index 2: %li   ", 0);

        return 0; // Return the first point if all are within lookahead distance

    }


    double calculateSteeringAngle(const RobotPose& robot_pose, const Eigen::Vector2d& lookahead_point) {
        // Calculate target angle (global frame)
        double dx = lookahead_point[0] - robot_pose.position[0];
        double dy = lookahead_point[1] - robot_pose.position[1];
        double target_angle = std::atan2(dy, dx);

        // Get the robot's yaw (orientation) from Gazebo or TF
        double robot_yaw;
        if (auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker_)) {
            // Convert quaternion to yaw
            Eigen::VectorXd euler = gazebo_checker->quaternionToEuler(robot_pose.orientation);
            robot_yaw = euler[2];  // yaw is the third element
        } else {
            // For TF, extract yaw directly from quaternion
            tf2::Quaternion q(
                robot_pose.orientation[0],
                robot_pose.orientation[1],
                robot_pose.orientation[2],
                robot_pose.orientation[3]
            );
            robot_yaw = tf2::getYaw(q);
        }



        // Compute angle error (normalize to [-π, π])
        double angle_error = target_angle - robot_yaw;
        // RCLCPP_INFO(this->get_logger(), "ROBOT_YAW: %.2f, Target ANGLE: %.2f   ", robot_yaw , target_angle);
        angle_error = std::fmod(angle_error + M_PI, 2 * M_PI) - M_PI;
        angle_error = std::remainder(angle_error, 2 * M_PI); // Normalizes to [-π, π]

        // If error is exactly ±π, choose clockwise rotation
        if (std::abs(angle_error) == M_PI) {
            angle_error = -M_PI; // or M_PI, depending on desired direction
        }

        return angle_error;

        // Apply smoothing
        // double smoothed_angle = smoothing_factor * angle_error + (1 - smoothing_factor) * prev_steering_angle;
        // prev_steering_angle = smoothed_angle;
        // return smoothed_angle;



    }

    void controlLoop() {
            if (current_path_.empty() || current_path_.size()==1) {
                auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
                twist_msg->linear.x = 0.0;
                twist_msg->angular.z = 0.0;
                cmd_vel_publisher_->publish(std::move(twist_msg));
                return;
            }



            // Get the robot's pose (now includes orientation)
            RobotPose robot_pose = getRobotPose();


            // if (current_path_.size() == 1) {
            //     // Get the last point in the path
            //     Eigen::VectorXd& last_point = current_path_[0];

            //     // Ensure the last point has at least 2 dimensions (x, y)
            //     if (last_point.size() >= 2) {
            //         // Calculate the Euclidean distance between the robot's position and the last point
            //         double dx = last_point(0) - robot_pose.position(0); // x difference
            //         double dy = last_point(1) - robot_pose.position(1); // y difference
            //         double distance = std::sqrt(dx * dx + dy * dy); // Euclidean distance
            //         // If the distance is less than 0.2 meters, stop the robot
            //         if (distance < 0.1) {
            //             auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
            //             twist_msg->linear.x = 0.0;
            //             twist_msg->angular.z = 0.0;
            //             cmd_vel_publisher_->publish(std::move(twist_msg));
            //         }
            //     }
            //     return;
            // }

            // // Find closest point and lookahead point
            size_t closest_index = findClosestPoint(robot_pose.position);
            size_t lookahead_index = findLookaheadPoint(robot_pose.position, closest_index);

            // size_t closest_index = findClosestPoint(robot_pose.position);
            // progress_index_ = std::max(progress_index_, closest_index); // Track furthest point reached
            // size_t lookahead_index = findLookaheadPoint(robot_pose.position, progress_index_);


            Eigen::Vector2d lookahead_point = current_path_[lookahead_index].head<2>();
        






            // // Visualize lookahead point --> the next point the shortest path that the robot is trying to reach
            // std::vector<Eigen::VectorXd> look_position_;
            // Eigen::VectorXd vec(2);
            // // vec << obstacle_pos.x(), obstacle_pos.y();
            // vec << current_path_[lookahead_index];
            // look_position_.push_back(vec);
            // std::string color_str = "0.0,1.0,1.0"; // Blue color
 
            // visualizer_->visualizeNodes(look_position_, "map",color_str);




            // Calculate curvature and steering angle
            double curvature = calculatePathCurvature(lookahead_index);
            double steering_angle = calculateSteeringAngle(robot_pose, lookahead_point);

            // Dynamic speed limits based on curvature
            double max_speed = 4.0 * (1.0 - std::tanh(2.0 * curvature)); // Tune 5.0 for sensitivity
            double min_speed = 0.5; // Maintain minimum speed for control authority

            // Aggressive steering-based scaling
            double steering_penalty = 1.0 - (std::abs(steering_angle) / (0.8 * M_PI)); // 0-1 scaling
            steering_penalty = std::clamp(steering_penalty, 0.2, 1.0);

            // Combined velocity calculation
            double target_velocity = std::clamp(max_speed * steering_penalty, min_speed, 4.0);

            // Asymmetric acceleration control
            double dt = std::chrono::duration<double>(control_loop_dt_).count();
            double accel = (target_velocity > current_linear_velocity_) ? 1.0 : 4.0; // Fast deceleration
            current_linear_velocity_ += accel * (target_velocity - current_linear_velocity_) * dt;

            // Angular velocity boosting for sharp turns
            double angular_boost = 1.0 + 2.0 * curvature; // Boost angular response in curves
            double angular_z = kp_angular_ * steering_angle * angular_boost;
            angular_z = std::clamp(angular_z, -max_angular_speed_, max_angular_speed_);




            // Publish cmd_vel
            auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
            twist_msg->linear.x = current_linear_velocity_;
            // twist_msg->linear.x = linear_velocity;
            twist_msg->angular.z = angular_z; // From previous code
            cmd_vel_publisher_->publish(std::move(twist_msg));



            // // Adjust the lookahead_distance_ based on linear velocity to anticipate turns earlier at high speeds
            // lookahead_distance_ = linear_velocity * 2.0; // Tune multiplier (e.g., 2.0 seconds)
            // lookahead_distance_ = std::clamp(lookahead_distance_, 0.5, 1.0); // Min/Max

            // Dynamic lookahead with higher multiplier and wider range
            lookahead_distance_ = current_linear_velocity_ * 3.0;  // Increased from 2.0
            lookahead_distance_ = std::clamp(lookahead_distance_, 1.5, 5.0);  // Wider range



        }



private:
    std::shared_ptr<ObstacleChecker> obstacle_checker_; // Base class pointer
    std::shared_ptr<RVizVisualization> visualizer_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;  // Publisher for cmd_vel
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::vector<Eigen::VectorXd> current_path_;

    double kp_angular_ = 1.0;          // Proportional gain for angular velocity
    double max_angular_speed_ = 1.5;   // Maximum allowed angular speed (rad/s)
    double lookahead_distance_ = 1.5;  // Increase for smoother turns
    double linear_velocity_ = 0.2;
    double current_linear_velocity_ = 0.0;
    std::chrono::milliseconds control_loop_dt_;


    // Low-pass filter to the steering angle to reduce abrupt changes
    double prev_steering_angle = 0.0;
    const double smoothing_factor = 0.2; // Tune for desired smoothness


    size_t progress_index_ = 0;
    bool follow_path  = true;
    bool use_nav = false;
    bool use_pure_pursuit = true;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;
    // Mutex and condition variable for goal pose
    std::mutex goal_pose_mutex_;
    std::condition_variable goal_cv_;
    geometry_msgs::msg::PoseStamped latest_goal_pose_;
    bool goal_received_ = false; // Flag to indicate if a goal pose has been received

    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr follow_path_client_;


    void updateRobotPose() {
        double robot_x = 0.0;
        double robot_y = 0.0;

        // Check if the obstacle checker is of type GazeboObstacleChecker
        if (auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker_)) {
            // Get robot position directly from Gazebo
            Eigen::Vector2d robot_pos = gazebo_checker->getRobotPosition();
            robot_x = robot_pos.x();
            robot_y = robot_pos.y();
        }
        // Otherwise, get robot position from TF
        else {
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
                robot_x = transform.transform.translation.x;
                robot_y = transform.transform.translation.y;
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
                return;
            }
        }

        // Get obstacles in range and visualize them
        visualizeObstacles(robot_x, robot_y);
    }

    void visualizeObstacles(double robot_x, double robot_y) {
        // Get obstacles in range (in world coordinates)
        std::vector<Eigen::VectorXd> obstacles;
        std::vector<double> radii;
        // Check if the obstacle checker is of type OccupancyGridObstacleChecker
        if (auto grid_checker = std::dynamic_pointer_cast<OccupancyGridObstacleChecker>(obstacle_checker_)) {
            obstacles = grid_checker->getObstaclesInRange(robot_x, robot_y);
        }
        // Check if the obstacle checker is of type GazeboObstacleChecker
        else if (auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker_)) {
            for (const auto& obstacle_pos : gazebo_checker->getObstaclePositions()) {
                Eigen::VectorXd vec(2);
                // vec << obstacle_pos.x(), obstacle_pos.y();
                vec << obstacle_pos.position;
                obstacles.push_back(vec);
                radii.push_back(obstacle_pos.radius);
            }

            Eigen::Vector2d robot_pos = gazebo_checker->getRobotPosition();
            obstacles.push_back(robot_pos); //Visualizing robot!
            radii.push_back(1.0); // a radius for robot!
        }

        // Visualize obstacles in RViz
        // visualizer_->visualizeNodes(obstacles, "map");

        // Visualize obstacles in RViz by drawing a circle for each obstacle
        visualizer_->visualizeCylinder(obstacles, radii, "map");
    }
};