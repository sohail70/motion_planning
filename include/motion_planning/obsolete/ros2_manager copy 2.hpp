// Copyright 2025 Soheil E.nia
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "motion_planning/utils/occupancygrid_obstacle_checker.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"

class ROS2Manager : public rclcpp::Node {
public:
    ROS2Manager(
        std::shared_ptr<OccupancyGridObstacleChecker> obstacle_checker,
        std::shared_ptr<RVizVisualization> visualizer)
        : Node("ros2_manager"),
          obstacle_checker_(obstacle_checker),
          visualizer_(visualizer),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tf_listener_(*tf_buffer_)
    {
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&ROS2Manager::mapCallback, this, std::placeholders::_1));
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // Update the grid in the obstacle checker
        obstacle_checker_->updateGrid(msg);

        // Get robot pose from TF
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }

        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;

        // Get obstacles in range and visualize them
        visualizeObstacles(robot_x, robot_y);
    }

private:
    std::shared_ptr<OccupancyGridObstacleChecker> obstacle_checker_;
    std::shared_ptr<RVizVisualization> visualizer_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void visualizeObstacles(double robot_x, double robot_y) {
        // Get obstacles in range (in world coordinates)
        std::vector<Eigen::VectorXd> obstacles = obstacle_checker_->getObstaclesInRange(robot_x, robot_y);

        // Visualize obstacles in RViz
        visualizer_->visualizeNodes(obstacles, "map");
    }
};