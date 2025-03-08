#pragma once


#include "motion_planning/utils/obstacle_checker.hpp"
#include "motion_planning/utils/visualization.hpp"

class GazeboObstacleChecker : public ObstacleChecker {
public:

    GazeboObstacleChecker(const std::string& robot_model_name,
                        const std::unordered_map<std::string, double>& obstacle_radii,
                        const std::string& world_name = "default");

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
    double obstacle_radius_;
    mutable std::mutex data_mutex_; // mutable allows locking in const methods
    Eigen::Vector2d robot_position_;
    Eigen::VectorXd robot_orientation_;
    
    // std::vector<Eigen::Vector2d> obstacle_positions_;
    gz::transport::Node gz_node_;
    gz::transport::Node::Publisher path_pub_;  // Publisher for the path

    bool use_range = true;
    double sensor_range = 20.0;
    
    bool persistent_static_obstacles = true;
    std::unordered_map<std::string, Obstacle> static_obstacle_positions_;
    




    // Change this member variable
    std::vector<Obstacle> obstacle_positions_;
    
    // Add this member to store radii
    std::unordered_map<std::string, double> obstacle_radii_;


};
