// Copyright 2025 Soheil E.nia
#pragma once


#include "motion_planning/utils/obstacle_checker.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/occupancygrid_obstacle_checker.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"


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
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&ROS2Manager::updateCallback, this));
    }

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

private:
    std::shared_ptr<ObstacleChecker> obstacle_checker_; // Base class pointer
    std::shared_ptr<RVizVisualization> visualizer_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

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
            // Eigen::Vector2d robot_pos = gazebo_checker->getRobotPosition();
            // obstacles.push_back(robot_pos); //Visualizing robot!
            
        }


        // Visualize obstacles in RViz
        visualizer_->visualizeNodes(obstacles, "map");

        // Visualize obstacles in RViz by drawing a circle for each obstacle
        visualizer_->visualizeCylinder(obstacles, radii, "map");



        
    }
};