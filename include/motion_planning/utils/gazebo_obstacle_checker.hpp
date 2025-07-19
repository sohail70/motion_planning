#pragma once


#include "motion_planning/utils/obstacle_checker.hpp"
#include "motion_planning/utils/visualization.hpp"
#include "motion_planning/utils/params.hpp"
#include <unsupported/Eigen/Polynomials>  // for PolynomialSolver
#include "motion_planning/utils/kalman_filter.hpp"
#include "motion_planning/utils/kalman_filter_factory.hpp"
#include <fcl/fcl.h>

class GazeboObstacleChecker : public ObstacleChecker {
public:

    GazeboObstacleChecker(rclcpp::Clock::SharedPtr clock,
                        const Params& params,
                        const std::unordered_map<std::string, ObstacleInfo>& obstacle_info);

    ~GazeboObstacleChecker();

    bool isObstacleFree(const Eigen::VectorXd& start, 
                       const Eigen::VectorXd& end) const override;
    
    bool isObstacleFree(const Eigen::VectorXd& point)const override;

    bool isObstacleFree(const std::vector<Eigen::VectorXd>& path) const override;

bool isTrajectorySafeAgainstSingleObstacle(const Trajectory& trajectory, 
                                           double global_start_time, 
                                           const Obstacle& obstacle) const;


    bool isInVelocityObstacle(
        const Eigen::Vector2d& robot_velocity,
        const Eigen::Vector2d& obs_velocity,
        const Eigen::Vector2d& pos_robot_to_obs,
        double combined_radius
    ) const;


    /**
     * @brief [THE NEW, CORRECTED FUNCTION] Performs a full time-aware collision check.
     * @param trajectory The kinodynamically-feasible trajectory to check.
     * @param start_node_cost The global time at which the robot begins this trajectory.
     * @return An std::optional containing the colliding obstacle if a collision is
     * predicted. Returns std::nullopt if the path is clear.
     */
    std::optional<Obstacle> getCollidingObstacle(const Trajectory& trajectory, double start_node_cost) const override;
    std::optional<Obstacle> getCollidingObstacleFCL(const Trajectory& trajectory, double start_node_cost) const override;
    
    /**
     * @brief [NEW & CORRECTED] Performs a full time-aware collision check for a trajectory.
     * @param trajectory The kinodynamically-feasible trajectory to check.
     * @param start_node_time The global time at which the robot begins this trajectory.
     * @return TRUE if the path is clear, FALSE if a collision is predicted.
     */
    bool isTrajectorySafe( const Trajectory& trajectory, double start_node_time) const override;

    bool check_arc_line_collision(
        // Arc properties
        const Eigen::Vector2d& p_r0_start,
        const Eigen::Vector2d& center,
        double radius,
        double angular_velocity,
        // Obstacle properties
        const Eigen::Vector2d& p_o0_start,
        const Eigen::Vector2d& v_o,
        // Time and collision properties
        double T_segment,
        double R_sq // Combined radius squared
    ) const;

    Eigen::Vector2d getRobotPosition() const;
    Eigen::VectorXd getRobotOrientation() const;

    const ObstacleVector& getObstaclePositions() const;
    void robotPoseCallback(const gz::msgs::Pose_V& msg);


    Eigen::VectorXd quaternionToEuler(const Eigen::VectorXd& quaternion) const;
    Eigen::VectorXd getRobotEulerAngles() const;
    double calculateYawFromQuaternion(const Eigen::VectorXd& quaternion);

    void publishPath(const std::vector<Eigen::VectorXd>& waypoints);

ObstacleVector getObstacles() const override;

bool checkFootprintCollision(const Eigen::Vector2d& position,
                               double yaw,
                               const std::vector<Eigen::Vector2d>& footprint) const override {
    // std::lock_guard<std::mutex> lock(data_mutex_);
    
    const Eigen::Rotation2Dd rot(yaw);
    for(const auto& local_point : footprint) {
        Eigen::Vector2d world_point = position + rot * local_point;
        
        for(const auto& obstacle : obstacle_positions_) {
            if(obstacle.type == Obstacle::CIRCLE) {
                double total_radius = obstacle.dimensions.radius + obstacle.inflation;
                if((world_point - obstacle.position).norm() <= total_radius) {
                    return true;
                }
            } else {
                double width = obstacle.dimensions.width + 2*obstacle.inflation;
                double height = obstacle.dimensions.height + 2*obstacle.inflation;
                if(pointIntersectsRectangle(world_point, obstacle.position,
                                           width, height, obstacle.dimensions.rotation)) {
                    return true;
                }
            }
        }
    }
    return false;
}

double distanceToNearestObstacle(const Eigen::Vector2d& position) const override {
    // std::lock_guard<std::mutex> lock(data_mutex_);
    double min_distance = std::numeric_limits<double>::max();
    
    for(const auto& obstacle : obstacle_positions_) {
        double dist;
        if(obstacle.type == Obstacle::CIRCLE) {
            dist = (position - obstacle.position).norm() - 
                  (obstacle.dimensions.radius + obstacle.inflation);
        } else {
            // Calculate distance to expanded rectangle
            Eigen::Rotation2Dd rot(-obstacle.dimensions.rotation);
            Eigen::Vector2d local_pos = rot * (position - obstacle.position);
            double expanded_width = obstacle.dimensions.width + 2*obstacle.inflation;
            double expanded_height = obstacle.dimensions.height + 2*obstacle.inflation;
            
            double dx = std::max(std::abs(local_pos.x()) - expanded_width/2, 0.0);
            double dy = std::max(std::abs(local_pos.y()) - expanded_height/2, 0.0);
            dist = std::sqrt(dx*dx + dy*dy);
        }
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



    struct Snapshot {
        Eigen::Vector2d robot_position;
        ObstacleVector obstacles;
    };

    Snapshot getAtomicSnapshot() {
        // 1. First, process any new pose message that has arrived since the last call.
        //    This updates the internal state (Kalman filters, obstacle_positions_, etc.).
        processLatestPoseInfo();
        /*
            The snapshot_mutex_ guarantees that the Gazebo callback cannot change obstacle_positions_ 
            while getAtomicSnapshot is in the middle of copying it.
        */
        // std::lock_guard<std::mutex> lock(snapshot_mutex_);
        obstacle_snapshot_ = obstacle_positions_;  // Atomic copy --> obstacle snapshot is gonna be used in isObstacleFree or isTrajectorySafe, because obstalce_positions_ is live updating while you are in a plan() function
        return {robot_position_, obstacle_snapshot_};
    }


    // --- NEW HELPER FUNCTION (based on the Julia code's logic) ---
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


inline double normalizeAngle(double angle) const {
    // Use fmod to bring the angle into the [-2*PI, 2*PI] range
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0) {
        angle += 2.0 * M_PI;
    }
    return angle - M_PI;
}
// Calculates the squared distance from a point to a circular arc segment.
double distanceSqrdPointToArc(
    const Eigen::Vector2d& p,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& end,
    const Eigen::Vector2d& center,
    double radius,
    bool is_clockwise) const
{
    // 1. Vector from point to circle center
    Eigen::Vector2d to_center = p - center;
    double dist_to_center_sq = to_center.squaredNorm();

    // 2. Find the closest point on the full circle to p
    Eigen::Vector2d closest_point_on_circle = center + radius * to_center.normalized();

    // 3. Check if this closest point lies within the arc segment
    double start_angle = atan2(start.y() - center.y(), start.x() - center.x());
    double end_angle = atan2(end.y() - center.y(), end.x() - center.x());
    double point_angle = atan2(closest_point_on_circle.y() - center.y(), closest_point_on_circle.x() - center.x());

    // Normalize angles to be relative to the start angle
    double relative_end_angle = normalizeAngle(end_angle - start_angle);
    double relative_point_angle = normalizeAngle(point_angle - start_angle);

    if (is_clockwise && relative_end_angle > 0) relative_end_angle -= 2 * M_PI;
    if (!is_clockwise && relative_end_angle < 0) relative_end_angle += 2 * M_PI;

    bool is_within_arc = is_clockwise
        ? (relative_point_angle <= 0 && relative_point_angle >= relative_end_angle)
        : (relative_point_angle >= 0 && relative_point_angle <= relative_end_angle);

    if (is_within_arc) {
        // The closest point is on the arc itself.
        return std::pow(std::sqrt(dist_to_center_sq) - radius, 2);
    } else {
        // The closest point is one of the endpoints.
        return std::min((p - start).squaredNorm(), (p - end).squaredNorm());
    }
}



    double findNearestObstacleDistance(const Eigen::Vector2d& point) const {
        // std::lock_guard<std::mutex> lock(snapshot_mutex_);
        double min_distance = std::numeric_limits<double>::infinity();

        if (obstacle_snapshot_.empty()) {
            return min_distance;
        }

        for (const auto& obstacle : obstacle_snapshot_) {
            double current_distance = 0.0;
            const Eigen::Vector2d& center = obstacle.position;
            
            if (obstacle.type == Obstacle::CIRCLE) {
                // Distance from point to circle's edge
                double dist_to_center = (point - center).norm();
                current_distance = std::max(0.0, dist_to_center - (obstacle.dimensions.radius + inflation));
            } else { // BOX
                // For a box, we can approximate by checking distance to its bounding circle
                double half_diagonal = std::hypot(obstacle.dimensions.width, obstacle.dimensions.height) / 2.0;
                double dist_to_center = (point - center).norm();
                current_distance = std::max(0.0, dist_to_center - (half_diagonal + inflation));
            }

            if (current_distance < min_distance) {
                min_distance = current_distance;
            }
        }
        return min_distance;
    }


    void processLatestPoseInfo();
private:
    void poseInfoCallback(const gz::msgs::Pose_V& msg);
    bool new_pose_msg_available_;

    gz::msgs::Pose_V latest_pose_msg_;

    void lightweightPoseCallback(const gz::msgs::Pose_V& msg);


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
    double inflation;
    bool persistent_static_obstacles;
    bool estimation;
    bool use_fcl;


    std::unordered_map<std::string, Obstacle> static_obstacle_positions_;

    ///////////////////////////Predicting Future Positions of Obstacles//////////////////////////////////////////////
    /*
        Even with perfect current_pos and prev_pos values, this calculation creates two major problems:
        Velocity is a Step-Function: The calculated velocity is a single value that represents the average speed over the interval dt. 
        It is not the instantaneous velocity. So your velocity estimate is not a smooth cosine wave, but a series of flat, constant steps.
        Acceleration is an Impulse Function: When you differentiate this step-function of velocity, you get an acceleration that is zero everywhere except at the exact moment of the update, where it becomes an infinitely large spike (an impulse). 
        In code, this results in extremely large, erratic, and non-physical acceleration values.

        So its better to use KF:
        Finite-Difference Method: Is like trying to figure out a car's acceleration by only looking at two photos of it, taken a fraction of a second apart. The result will be wildly inaccurate.
        Kalman Filter Method: Is like having a physics model of the car (its state, including its current velocity and acceleration). When you get a new photo (a new position measurement), 
        you don't throw away your old understanding. Instead, you use the new photo to correct and refine your ongoing estimate of the car's state.
    */

    std::unordered_map<std::string, Obstacle> previous_obstacle_states_; //if you are using finite difference method --> Which introduces noise by the mathematical process of differentiation on discrete time samples
    //if you are using kalman filter to predict obstalce
    // This map stores a Kalman Filter instance for each dynamic obstacle.
    std::unordered_map<std::string, KalmanFilter> obstacle_filters_;
    
    // This map stores the last update time for each filter to calculate 'dt'.
    std::unordered_map<std::string, rclcpp::Time> obstacle_filters_times_;

    std::string kf_model_type_;
    /////////////////////////////////////////////////////////////////////////


    

    

    mutable ObstacleVector obstacle_snapshot_;
    mutable std::mutex snapshot_mutex_;



    // Change this member variable
    ObstacleVector obstacle_positions_;
    
    // Add this member to store radii
    // std::unordered_map<std::string, double> obstacle_radii_;
    std::unordered_map<std::string, ObstacleInfo> obstacle_info_;

    rclcpp::Clock::SharedPtr clock_;


    // Maps an obstacle's unique name (from Gazebo) to its FCL representation.
    mutable std::unordered_map<std::string, fcl::CollisionObjectd> fcl_cache_;

    // /**
    //  * @brief Creates an FCL collision object from our custom Obstacle struct.
    //  * @param obstacle The obstacle to convert.
    //  * @return A shared pointer to an FCL CollisionObject.
    //  */
    // fcl::CollisionObjectd createFCLObject(const Obstacle& obstacle) const;


};
