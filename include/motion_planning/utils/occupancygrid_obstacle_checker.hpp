// occupancy_grid_obstacle_checker.hpp
#ifndef OCCUPANCY_GRID_OBSTACLE_CHECKER_HPP
#define OCCUPANCY_GRID_OBSTACLE_CHECKER_HPP

#include "obstacle_checker.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

class OccupancyGridObstacleChecker : public ObstacleChecker {
public:
    OccupancyGridObstacleChecker(std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid)
        : grid_(grid) {}

    bool isObstacleFree(const Eigen::VectorXd& start, const Eigen::VectorXd& end) const override {
        // Convert start and end points to grid coordinates
        int start_x = worldToGrid(start.x(), grid_->info.origin.position.x, grid_->info.resolution);
        int start_y = worldToGrid(start.y(), grid_->info.origin.position.y, grid_->info.resolution);
        int end_x = worldToGrid(end.x(), grid_->info.origin.position.x, grid_->info.resolution);
        int end_y = worldToGrid(end.y(), grid_->info.origin.position.y, grid_->info.resolution);

        // Perform Bresenham's line algorithm to check for obstacles along the path
        return isLineObstacleFree(start_x, start_y, end_x, end_y);
    }

private:
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid_;

    // Convert world coordinates to grid coordinates
    int worldToGrid(double world_coord, double origin, double resolution) const {
        return static_cast<int>((world_coord - origin) / resolution);
    }

    // Check if a line between two grid points is obstacle-free
    bool isLineObstacleFree(int x0, int y0, int x1, int y1) const {
        int dx = abs(x1 - x0);
        int dy = -abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        while (true) {
            // Check if the current cell is occupied
            if (isCellOccupied(x0, y0)) {
                return false;
            }

            if (x0 == x1 && y0 == y1) {
                break;
            }

            int e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }

        return true;
    }

    // Check if a grid cell is occupied
    bool isCellOccupied(int x, int y) const {
        if (x < 0 || x >= grid_->info.width || y < 0 || y >= grid_->info.height) {
            return true; // Treat out-of-bounds as occupied
        }

        int index = y * grid_->info.width + x;
        return grid_->data[index] > 50; // Occupied if value > 50
    }
};

#endif // OCCUPANCY_GRID_OBSTACLE_CHECKER_HPP
