// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/utils/obstacle_checker.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

class OccupancyGridObstacleChecker : public ObstacleChecker {
public:
    OccupancyGridObstacleChecker(double lidar_range)
        : lidar_range_(lidar_range) {}

    bool isObstacleFree(const Eigen::VectorXd& start, const Eigen::VectorXd& end) const override {
        if (!grid_) return true; // No grid data yet

        int start_x = worldToGrid(start.x(), grid_->info.origin.position.x, grid_->info.resolution);
        int start_y = worldToGrid(start.y(), grid_->info.origin.position.y, grid_->info.resolution);
        int end_x = worldToGrid(end.x(), grid_->info.origin.position.x, grid_->info.resolution);
        int end_y = worldToGrid(end.y(), grid_->info.origin.position.y, grid_->info.resolution);

        return isLineObstacleFree(start_x, start_y, end_x, end_y);
    }

    void updateGrid(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid) override {
        grid_ = grid;
    }

    // Modify to return a list of obstacle coordinates in range
    std::vector<Eigen::VectorXd> getObstaclesInRange(double robot_x, double robot_y) const {
        std::vector<Eigen::VectorXd> obstacles;

        if (!grid_) return obstacles; // No grid data yet

        // int min_x = worldToGrid(robot_x - lidar_range_, grid_->info.origin.position.x, grid_->info.resolution);
        // int max_x = worldToGrid(robot_x + lidar_range_, grid_->info.origin.position.x, grid_->info.resolution);
        // int min_y = worldToGrid(robot_y - lidar_range_, grid_->info.origin.position.y, grid_->info.resolution);
        // int max_y = worldToGrid(robot_y + lidar_range_, grid_->info.origin.position.y, grid_->info.resolution);


        int min_x = std::max(0, static_cast<int>(worldToGrid(robot_x - lidar_range_, grid_->info.origin.position.x, grid_->info.resolution)));
        int max_x = std::min(static_cast<unsigned int>(grid_->info.width - 1), 
                            static_cast<unsigned int>(worldToGrid(robot_x + lidar_range_, grid_->info.origin.position.x, grid_->info.resolution)));
        int min_y = std::max(0, static_cast<int>(worldToGrid(robot_y - lidar_range_, grid_->info.origin.position.y, grid_->info.resolution)));
        int max_y = std::min(static_cast<unsigned int>(grid_->info.height - 1), 
                            static_cast<unsigned int>(worldToGrid(robot_y + lidar_range_, grid_->info.origin.position.y, grid_->info.resolution)));

        // Iterate through the grid and check for obstacles within the bounding box
        for (int x = min_x; x <= max_x; ++x) {
            for (int y = min_y; y <= max_y; ++y) {
                double dx = (x * grid_->info.resolution + grid_->info.origin.position.x) - robot_x;
                double dy = (y * grid_->info.resolution + grid_->info.origin.position.y) - robot_y;
                if (dx * dx + dy * dy <= lidar_range_ * lidar_range_ && isCellOccupied(x, y)) {
                    Eigen::VectorXd obstacle(2);
                    obstacle << (x * grid_->info.resolution + grid_->info.origin.position.x), 
                                (y * grid_->info.resolution + grid_->info.origin.position.y);
                    obstacles.push_back(obstacle);
                }
            }
        }

        return obstacles;
    }

    Eigen::VectorXd gridToWorld(int grid_x, int grid_y) const {
        double world_x = grid_x * grid_->info.resolution + grid_->info.origin.position.x;
        double world_y = grid_y * grid_->info.resolution + grid_->info.origin.position.y;
        Eigen::VectorXd world_coords(2);
        world_coords << world_x, world_y;
        return world_coords;
    }

    // Convert grid (x, y) to row-major index
    int getRowMajorIndex(int x, int y) const {
        return y * grid_->info.width + x;
    }

    // Convert row-major index to grid (x, y)
    std::pair<int, int> getGridFromIndex(int index) const {
        int x = index % grid_->info.width;
        int y = index / grid_->info.width;
        return {x, y};
    }



private:
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid_;
    double lidar_range_;

    int worldToGrid(double world_coord, double origin, double resolution) const {
        return static_cast<int>((world_coord - origin) / resolution);
    }

    bool isLineObstacleFree(int x0, int y0, int x1, int y1) const {
        int dx = abs(x1 - x0);
        int dy = -abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        while (true) {
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

    bool isCellOccupied(int x, int y) const {
        if (x < 0 || x >= grid_->info.width || y < 0 || y >= grid_->info.height) {
            // return true; // Treat out-of-bounds as occupied
            return false; // Treat out-of-bounds as not occupied
        }
        int index = y * grid_->info.width + x;
        return grid_->data[index] > 50; // Occupied if value > 50
    }
};

/*

--> x
|
y

*/