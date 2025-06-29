// Copyright 2025 Soheil E.nia



#pragma once

#include <memory>
#include "motion_planning/pch.hpp"
#include "motion_planning/ds/edge_info.hpp"


#include "rclcpp/rclcpp.hpp" 

struct Obstacle {
    enum Type { CIRCLE, BOX };
    Type type;
    Eigen::Vector2d position;
    Eigen::Vector2d velocity;
    Eigen::Vector2d acceleration;

    // std::chrono::steady_clock::time_point last_update_time;
    // TO: Use the ROS 2 Time object
    rclcpp::Time last_update_time;
    union {
        struct {
            double radius;
        } circle;
        struct {
            double width;
            double height;
            double rotation;
        } box;
    } dimensions;
    double inflation;
    bool is_dynamic = false;  // true if this obstacle moves
    // bool is_seen = false;  // true once samples-near have been processed

    // Add default constructor
    Obstacle() : type(CIRCLE), position(Eigen::Vector2d::Zero()), inflation(0.0) {
        dimensions.circle.radius = 0.0;
    }

    // Circle constructor
    Obstacle(Eigen::Vector2d pos, double rad, double infl, bool dynamic=false)
        : type(CIRCLE), position(pos), inflation(infl),is_dynamic(dynamic) {
        dimensions.circle.radius = rad;
        // last_update_time = std::chrono::steady_clock::now();
    }

    // Box constructor
    Obstacle(Eigen::Vector2d pos, double w, double h, double rot, double infl, bool dynamic=false)
        : type(BOX), position(pos), inflation(infl), is_dynamic(dynamic) {
        dimensions.box.width = w;
        dimensions.box.height = h;
        dimensions.box.rotation = rot;
        // last_update_time = std::chrono::steady_clock::now();
        // last_update_time is now set externally
    }

    bool operator==(const Obstacle& other) const {
        // Update equality check for new structure
        if (type != other.type || position != other.position || inflation != other.inflation)
            return false;
        
        if (type == CIRCLE) {
            return dimensions.circle.radius == other.dimensions.circle.radius;
        }
        return dimensions.box.width == other.dimensions.box.width &&
               dimensions.box.height == other.dimensions.box.height &&
               dimensions.box.rotation == other.dimensions.box.rotation;
    }
};

struct ObstacleInfo {
    enum Type { CYLINDER, BOX };
    Type type;
    double radius = 0.0;    // For cylinders (initialize!)
    double width = 0.0;     // For boxes (initialize!)
    double height = 0.0;    // For boxes (initialize!)
};

// Operator<< definition OUTSIDE the struct
inline std::ostream& operator<<(std::ostream& os, const ObstacleInfo& info) {
    if (info.type == ObstacleInfo::CYLINDER) {
        os << "Cylinder(r=" << info.radius << ")";
    } else {
        os << "Box(w=" << info.width << ", h=" << info.height << ")";
    }
    return os;
}

class ObstacleChecker {
public:
    virtual ~ObstacleChecker() = default;

    // Pure virtual method for obstacle checking
    virtual bool isObstacleFree(const Eigen::VectorXd& start, const Eigen::VectorXd& end) const = 0;
    virtual bool isObstacleFree(const Eigen::VectorXd& point) const = 0;
    // New virtual function to check a whole path
    virtual bool isObstacleFree(const std::vector<Eigen::VectorXd>& path) const = 0;
    virtual std::optional<Obstacle> getCollidingObstacle( const Trajectory& trajectory, double start_node_cost) const = 0;
    virtual bool isTrajectorySafe( const Trajectory& trajectory, double start_node_time) const = 0;

    // virtual void updateGrid(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid) = 0;
    virtual std::vector<Obstacle> getObstacles() const = 0;
    virtual bool checkFootprintCollision(const Eigen::Vector2d& position,
                                       double yaw,
                                       const std::vector<Eigen::Vector2d>& footprint) const = 0;
                                       
    virtual double distanceToNearestObstacle(const Eigen::Vector2d& position) const = 0;

};





// // simple_obstacle_checker.hpp
// #ifndef SIMPLE_OBSTACLE_CHECKER_HPP
// #define SIMPLE_OBSTACLE_CHECKER_HPP

// #include "obstacle_checker.hpp"
// #include <vector>
// #include <Eigen/Dense>

// class SimpleObstacleChecker : public ObstacleChecker {
// public:
//     SimpleObstacleChecker(const std::vector<Eigen::Vector2d>& obstacles, double obstacle_radius)
//         : obstacles_(obstacles), obstacle_radius_(obstacle_radius) {}

//     bool isObstacleFree(const Eigen::VectorXd& start, const Eigen::VectorXd& end) const override {
//         for (const auto& obstacle : obstacles_) {
//             if (isLineIntersectingCircle(start, end, obstacle, obstacle_radius_)) {
//                 return false;
//             }
//         }
//         return true;
//     }

// private:
//     std::vector<Eigen::Vector2d> obstacles_;
//     double obstacle_radius_;

//     bool isLineIntersectingCircle(const Eigen::VectorXd& start, const Eigen::VectorXd& end,
//                                   const Eigen::Vector2d& center, double radius) const {
//         // Implement line-circle intersection logic
//         // (This is a placeholder; replace with actual collision detection logic)
//         return false;
//     }
// };

// #endif // SIMPLE_OBSTACLE_CHECKER_HPP





// // grid_obstacle_checker.hpp
// #ifndef GRID_OBSTACLE_CHECKER_HPP
// #define GRID_OBSTACLE_CHECKER_HPP

// #include "obstacle_checker.hpp"
// #include <Eigen/Dense>
// #include <vector>

// class GridObstacleChecker : public ObstacleChecker {
// public:
//     GridObstacleChecker(const std::vector<std::vector<bool>>& grid, double resolution)
//         : grid_(grid), resolution_(resolution) {}

//     bool isObstacleFree(const Eigen::VectorXd& start, const Eigen::VectorXd& end) const override {
//         // Implement grid-based obstacle checking
//         // (This is a placeholder; replace with actual grid traversal logic)
//         return true;
//     }

// private:
//     std::vector<std::vector<bool>> grid_;
//     double resolution_;
// };

// #endif // GRID_OBSTACLE_CHECKER_HPP
