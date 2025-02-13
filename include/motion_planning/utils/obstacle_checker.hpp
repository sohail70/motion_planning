// obstacle_checker.hpp
#ifndef OBSTACLE_CHECKER_HPP
#define OBSTACLE_CHECKER_HPP

#include <memory>
#include <Eigen/Dense>

class ObstacleChecker {
public:
    virtual ~ObstacleChecker() = default;

    // Pure virtual method for obstacle checking
    virtual bool isObstacleFree(const Eigen::VectorXd& start, const Eigen::VectorXd& end) const = 0;
};

#endif // OBSTACLE_CHECKER_HPP




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
