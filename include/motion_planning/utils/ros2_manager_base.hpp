// Copyright 2025 Soheil E.nia
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include "motion_planning/utils/obstacle_checker.hpp"

// Abstract Base Class for ROS2 Managers
class ROS2ManagerBase : public rclcpp::Node {
public:
    // Constructor must match rclcpp::Node to allow derived classes to initialize it
    ROS2ManagerBase(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node(node_name, options) {}

    virtual ~ROS2ManagerBase() = default;

    // --- Public API used in main.cpp ---
    
    // Advance the simulation by dt
    virtual void stepSimulation(double dt) = 0;

    // Update the internal path the robot follows
    virtual void setPath(const std::vector<Eigen::VectorXd>& new_path) = 0;

    // Get the current state of the robot (x, y, theta/v, t)
    virtual Eigen::VectorXd getCurrentSimulatedState() = 0;

    // Update which obstacles are considered "threats" for visualization
    virtual void updateThreats(const std::vector<Obstacle>& culprits) = 0;

    // Get the total number of collisions detected
    virtual int getCollisionCount() const = 0;
    
    // Optional: Get current simulation time if needed directly
    virtual double getCurrentSimTime() const = 0;
};