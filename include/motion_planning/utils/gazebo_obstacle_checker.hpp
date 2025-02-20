#pragma once


#include "motion_planning/utils/obstacle_checker.hpp"

class GazeboObstacleChecker : public ObstacleChecker {
public:
    GazeboObstacleChecker(const std::string& robot_model_name, 
                         double obstacle_radius,
                         const std::string& world_name = "default");
    ~GazeboObstacleChecker();

    bool isObstacleFree(const Eigen::VectorXd& start, 
                       const Eigen::VectorXd& end) const override;

    Eigen::Vector2d getRobotPosition() const;
    std::vector<Eigen::Vector2d> getObstaclePositions() const;

private:
    void poseInfoCallback(const gz::msgs::Pose_V& msg);
    static bool lineIntersectsCircle(const Eigen::Vector2d& start,
                                    const Eigen::Vector2d& end,
                                    const Eigen::Vector2d& center,
                                    double radius);

    std::string robot_model_name_;
    double obstacle_radius_;
    mutable std::mutex data_mutex_;
    Eigen::Vector2d robot_position_;
    std::vector<Eigen::Vector2d> obstacle_positions_;
    gz::transport::Node gz_node_;
};
