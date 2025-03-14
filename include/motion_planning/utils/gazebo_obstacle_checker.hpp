#pragma once


#include "motion_planning/utils/obstacle_checker.hpp"
#include "motion_planning/utils/visualization.hpp"
#include "motion_planning/utils/params.hpp"
class GazeboObstacleChecker : public ObstacleChecker {
public:

    GazeboObstacleChecker(const Params& params,
                        const std::unordered_map<std::string, double>& obstacle_radii);

    ~GazeboObstacleChecker();

    bool isObstacleFree(const Eigen::VectorXd& start, 
                       const Eigen::VectorXd& end) const override;
    
    bool isObstacleFree(const Eigen::VectorXd& point)const override;

    Eigen::Vector2d getRobotPosition() const;
    Eigen::VectorXd getRobotOrientation() const;

    std::vector<Obstacle> getObstaclePositions() const;
    void robotPoseCallback(const gz::msgs::Pose_V& msg);


    Eigen::VectorXd quaternionToEuler(const Eigen::VectorXd& quaternion) const;
    Eigen::VectorXd getRobotEulerAngles() const;
    double calculateYawFromQuaternion(const Eigen::VectorXd& quaternion);

    void publishPath(const std::vector<Eigen::VectorXd>& waypoints);

std::vector<Obstacle> getObstacles() const override;

    // Add these method overrides
    bool checkFootprintCollision(const Eigen::Vector2d& position,
                               double yaw,
                               const std::vector<Eigen::Vector2d>& footprint) const override {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // Rotate footprint points to world orientation
        const Eigen::Rotation2Dd rot(yaw);
        for(const auto& local_point : footprint) {
            Eigen::Vector2d world_point = position + rot * local_point;
            
            // Check collision with all obstacles
            for(const auto& obstacle : obstacle_positions_) {
                if((world_point - obstacle.position).norm() <= obstacle.radius) {
                    return true;
                }
            }
        }
        return false;
    }

    double distanceToNearestObstacle(const Eigen::Vector2d& position) const override {
        std::lock_guard<std::mutex> lock(data_mutex_);
        double min_distance = std::numeric_limits<double>::max();
        
        for(const auto& obstacle : obstacle_positions_) {
            double dist = (position - obstacle.position).norm() - obstacle.radius;
            min_distance = std::min(min_distance, dist);
        }
        
        return min_distance;
    }

    // Add these helper methods
    Eigen::Vector2d rotatePoint(const Eigen::Vector2d& point, double yaw) const {
        return Eigen::Vector2d(
            point.x() * cos(yaw) - point.y() * sin(yaw),
            point.x() * sin(yaw) + point.y() * cos(yaw)
        );
    }





private:
    void poseInfoCallback(const gz::msgs::Pose_V& msg);
    static bool lineIntersectsCircle(const Eigen::Vector2d& start,
                                    const Eigen::Vector2d& end,
                                    const Eigen::Vector2d& center,
                                    double radius);
    static bool pointIntersectsCircle(const Eigen::Vector2d& point,
                                                  const Eigen::Vector2d& center,
                                                  double radius);
    std::string robot_model_name_;
    std::string world_name_;
    double obstacle_radius_;
    mutable std::mutex data_mutex_; // mutable allows locking in const methods
    Eigen::Vector2d robot_position_;
    Eigen::VectorXd robot_orientation_;
    
    // std::vector<Eigen::Vector2d> obstacle_positions_;
    gz::transport::Node gz_node_;
    gz::transport::Node::Publisher path_pub_;  // Publisher for the path

    bool use_range;
    double sensor_range;
    bool persistent_static_obstacles;


    std::unordered_map<std::string, Obstacle> static_obstacle_positions_;
    




    // Change this member variable
    std::vector<Obstacle> obstacle_positions_;
    
    // Add this member to store radii
    std::unordered_map<std::string, double> obstacle_radii_;


};
