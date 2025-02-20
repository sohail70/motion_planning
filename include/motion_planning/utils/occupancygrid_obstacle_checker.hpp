// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/utils/obstacle_checker.hpp"



// Custom hash function for std::pair<double, double>
struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto hash1 = std::hash<T1>{}(p.first);
        auto hash2 = std::hash<T2>{}(p.second);
        return hash1 ^ (hash2 << 1); // Combine the two hash values
    }
};

// Custom equality function for std::pair<double, double>
struct PairEqual {
    template <typename T1, typename T2>
    bool operator()(const std::pair<T1, T2>& lhs, const std::pair<T1, T2>& rhs) const {
        return lhs.first == rhs.first && lhs.second == rhs.second;
    }
};

class OccupancyGridObstacleChecker : public ObstacleChecker {
public:
    OccupancyGridObstacleChecker(double lidar_range, double partition_size = 5.0)
        : lidar_range_(lidar_range), partition_size_(partition_size) {}

    bool isObstacleFree(const Eigen::VectorXd& start, const Eigen::VectorXd& end) const override {
        if (!grid_) return true; // No grid data yet

        int start_x = worldToGrid(start.x(), grid_->info.origin.position.x, grid_->info.resolution);
        int start_y = worldToGrid(start.y(), grid_->info.origin.position.y, grid_->info.resolution);
        int end_x = worldToGrid(end.x(), grid_->info.origin.position.x, grid_->info.resolution);
        int end_y = worldToGrid(end.y(), grid_->info.origin.position.y, grid_->info.resolution);

        return isLineObstacleFree(start_x, start_y, end_x, end_y);
    }

    void updateGrid(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid) {
        grid_ = grid;
    }

    // Modify to return a list of obstacle coordinates in range
    std::vector<Eigen::VectorXd> getObstaclesInRange(double robot_x, double robot_y) {
        std::vector<Eigen::VectorXd> obstacles;

        if (!grid_) return obstacles; // No grid data yet

        // Calculate the bounding box of lidar range in grid coordinates
        int min_x = std::max(0, static_cast<int>(worldToGrid(robot_x - lidar_range_, grid_->info.origin.position.x, grid_->info.resolution)));
        int max_x = std::min(static_cast<int>(grid_->info.width - 1), 
                            static_cast<int>(worldToGrid(robot_x + lidar_range_, grid_->info.origin.position.x, grid_->info.resolution)));
        int min_y = std::max(0, static_cast<int>(worldToGrid(robot_y - lidar_range_, grid_->info.origin.position.y, grid_->info.resolution)));
        int max_y = std::min(static_cast<int>(grid_->info.height - 1), 
                            static_cast<int>(worldToGrid(robot_y + lidar_range_, grid_->info.origin.position.y, grid_->info.resolution)));

        // Temporary set to store obstacles detected in this iteration (in world coordinates)
        std::unordered_set<std::pair<double, double>, PairHash, PairEqual> current_obstacles;

        // Iterate through the grid and check for obstacles within the bounding box
        for (int x = min_x; x <= max_x; ++x) {
            for (int y = min_y; y <= max_y; ++y) {
                double dx = (x * grid_->info.resolution + grid_->info.origin.position.x) - robot_x;
                double dy = (y * grid_->info.resolution + grid_->info.origin.position.y) - robot_y;
                if (dx * dx + dy * dy <= lidar_range_ * lidar_range_ && isCellOccupied(x, y)) {
                    // Convert grid coordinates to world coordinates
                    double world_x = x * grid_->info.resolution + grid_->info.origin.position.x;
                    double world_y = y * grid_->info.resolution + grid_->info.origin.position.y;
                    current_obstacles.insert({world_x, world_y}); // Add to current obstacles
                }
            }
        }

        // Update the detected_obstacles_ set with current obstacles
        for (const auto& obstacle : current_obstacles) {
            detected_obstacles_.insert(obstacle); // Add new obstacles
        }

        // Remove outdated obstacles from detected_obstacles_ only if they are in the current range
        for (auto it = detected_obstacles_.begin(); it != detected_obstacles_.end(); ) {
            double world_x = it->first;
            double world_y = it->second;

            // Convert world coordinates to grid coordinates
            int x = worldToGrid(world_x, grid_->info.origin.position.x, grid_->info.resolution);
            int y = worldToGrid(world_y, grid_->info.origin.position.y, grid_->info.resolution);

            // Check if the obstacle is within the current range
            double dx = world_x - robot_x;
            double dy = world_y - robot_y;
            if (dx * dx + dy * dy <= lidar_range_ * lidar_range_) {
                // Only update obstacles within the current range
                if (!isCellOccupied(x, y)) {
                    // Remove obstacle if it is no longer occupied
                    it = detected_obstacles_.erase(it);
                } else {
                    ++it;
                }
            } else {
                // Keep obstacles outside the current range
                ++it;
            }
        }

        // Convert detected_obstacles_ to a vector of Eigen::VectorXd
        for (const auto& obstacle : detected_obstacles_) {
            Eigen::VectorXd obstacle_vec(2);
            obstacle_vec << obstacle.first, obstacle.second;
            obstacles.push_back(obstacle_vec);
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
    double partition_size_;

    // Use custom hash and equality functions for std::pair<double, double>
    std::unordered_set<std::pair<double, double>, PairHash, PairEqual> detected_obstacles_;

    // Use custom hash and equality functions for std::pair<int, int>
    std::unordered_map<std::pair<int, int>, std::unordered_set<std::pair<double, double>, PairHash, PairEqual>, PairHash, PairEqual> partitioned_obstacles_;

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
            return false; // Treat out-of-bounds as not occupied
        }
        int index = y * grid_->info.width + x;
        return grid_->data[index] > 50; // Occupied if value > 50
    }
};