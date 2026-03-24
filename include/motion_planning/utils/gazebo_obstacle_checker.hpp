// Copyright 2025 Soheil E.nia

#pragma once


#include "motion_planning/utils/obstacle_checker.hpp"
#include "motion_planning/utils/visualization.hpp"
#include "motion_planning/utils/params.hpp"
#include <unsupported/Eigen/Polynomials>  // for PolynomialSolver


class GazeboObstacleChecker : public ObstacleChecker {
public:


    GazeboObstacleChecker(rclcpp::Clock::SharedPtr clock,
                        const Params& params,
                        const std::unordered_map<std::string, ObstacleInfo>& obstacle_info);

    ~GazeboObstacleChecker();

    std::vector<Obstacle> getAndClearCulprits() const {
        std::vector<Obstacle> temp = culprit_cache_;
        culprit_cache_.clear();
        culprit_names_.clear();
        return temp;
    }

    bool isObstacleFree(const Eigen::VectorXd& start, 
                       const Eigen::VectorXd& end) const override;
    
    bool isObstacleFree(const Eigen::VectorXd& point)const override;

    bool isObstacleFree(const std::vector<Eigen::VectorXd>& path) const override;

    bool isObstacleFreeAgainstSingleObstacle(const Eigen::VectorXd& start, const Eigen::VectorXd& end, const Obstacle& obs) const override;
    bool isObstacleFreeAgainstSingleObstacle(const Eigen::VectorXd& point, const Obstacle& obs) const override;




    bool isTrajectorySafeAgainstSingleObstacle(const Trajectory& trajectory, const Obstacle& obstacle) const override;
    bool isTrajectorySafe( const Trajectory& trajectory) const override;
    Eigen::Vector2d getRobotPosition() const;
    Eigen::VectorXd getRobotOrientation() const;

    const ObstacleVector& getObstaclePositions() const;


    Eigen::VectorXd quaternionToEuler(const Eigen::VectorXd& quaternion) const;
    Eigen::VectorXd getRobotEulerAngles() const;
    double calculateYawFromQuaternion(const Eigen::VectorXd& quaternion);


    const ObstacleVector& getObstacles() const override;
    int getObstaclesSize() const override;



    // Calculates the squared distance from a point 'p' to a line segment defined by 'a' and 'b'.
    double distanceSqrdPointToSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b) const {
        const Eigen::Vector2d ab = b - a;
        const Eigen::Vector2d ap = p - a;

        // Get the squared length of the segment.
        const double ab_len_sq = ab.squaredNorm();

        // --- HARDENING: Handle zero-length segments ---
        if (ab_len_sq < 1e-9) {
            // If the segment is just a point, return the squared distance to that point.
            return ap.squaredNorm();
        }
        // ---------------------------------------------

        const double t = ap.dot(ab) / ab_len_sq;

        if (t < 0.0) {
            return ap.squaredNorm(); // Closest point is 'a'
        }
        if (t > 1.0) {
            return (p - b).squaredNorm(); // Closest point is 'b'
        }

        const Eigen::Vector2d closest_point = a + t * ab;
        return (p - closest_point).squaredNorm();
    }

    double distanceSqrdPointToSegment3D(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b) const{
        const Eigen::Vector3d ab = b - a;
        const Eigen::Vector3d ap = p - a;
        const double ab_len_sq = ab.squaredNorm();
        if (ab_len_sq < 1e-9) { return ap.squaredNorm(); }
        const double t = std::max(0.0, std::min(1.0, ap.dot(ab) / ab_len_sq));
        return (p - (a + t * ab)).squaredNorm();
    }


    inline double normalizeAngle(double angle) const {
        // Use fmod to bring the angle into the [-2*PI, 2*PI] range
        angle = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0.0) {
            angle += 2.0 * M_PI;
        }
        return angle - M_PI;
    }

    void processLatestPoseInfo();
    void processLatestPoseInfo(double current_robot_time); 
    Eigen::Vector2d calculateDeterministicPosition(const Obstacle& ob, double current_robot_time) const;


    bool checkRobotCollision(const Eigen::Vector2d& position, double yaw) const;
    bool checkRobotCollision(const Eigen::Vector3d& position, double yaw) const;


    double calculateNextFlip(const Obstacle& ob, double currentRobotTime);
    ObstacleVector checkAndRepairObstacles(double T_robot);
    std::vector<Eigen::Vector3d> generatePrediction(const Obstacle& ob, double current_time) const override;
    void initializeDynamicObstacles(double currentRobotTime);
    void updateObstaclesMathematically(double current_robot_time);
    
    bool isNodeInObstacleTube(const Eigen::VectorXd& node_state, const Obstacle& ob, double max_edge_length) const;

    Eigen::Vector2d getObstacleAnalyticalPosition(const Obstacle& ob, double t_robot) const;
    

private:





    static bool lineIntersectsCircle(const Eigen::Vector2d& start,
                                    const Eigen::Vector2d& end,
                                    const Eigen::Vector2d& center,
                                    double radius);
    static bool pointIntersectsCircle(const Eigen::Vector2d& point,
                                                  const Eigen::Vector2d& center,
                                                  double radius);

    static bool lineIntersectsRectangle(const Eigen::Vector2d& start,
                                       const Eigen::Vector2d& end,
                                       const Eigen::Vector2d& center,
                                       double width, double height,
                                       double rotation);
                                       
    static bool pointIntersectsRectangle(const Eigen::Vector2d& point,
                                        const Eigen::Vector2d& center,
                                        double width, double height,
                                        double rotation);



    mutable std::mutex data_mutex_; // mutable allows locking in const methods
    Eigen::Vector2d robot_position_;
    Eigen::VectorXd robot_orientation_;


    bool use_range;
    double sensor_range;
    double inflation;
    bool persistent_static_obstacles;
    int spatial_dim_;

    /////////////////////////////////////////////////////////////////////////

    std::string footprint_type_;
    double robot_radius_; // Only used for circular footprint
    std::vector<Eigen::Vector2d> rectangular_footprint_; // Only used for rectangular footprint

    bool checkCircularCollisionHelper(const Eigen::Vector2d& robot_position, double robot_radius) const;
    bool checkCircularCollisionHelper3D(const Eigen::Vector3d& robot_position, double robot_radius, double robot_height) const;

    
    // Change this member variable
    ObstacleVector obstacle_positions_;
    
    // Add this member to store radii
    // std::unordered_map<std::string, double> obstacle_radii_;
    std::unordered_map<std::string, ObstacleInfo> obstacle_info_;

    rclcpp::Clock::SharedPtr clock_;

    // Inside the private section:
    mutable std::vector<Obstacle> culprit_cache_;
    mutable std::set<std::string> culprit_names_;
    void recordCulprit(const Obstacle& obs) const;
    std::map<std::string, Obstacle> obstacle_positions_map_;
    // Determinism Debugging
    bool use_deterministic_override_ = true; // Set this to TRUE to force math model

    double initial_budget_time_ ;
    bool is_geometric_mode_;

};
