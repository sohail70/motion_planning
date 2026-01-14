// Copyright 2025 Soheil E.nia

#pragma once


#include "motion_planning/utils/obstacle_checker.hpp"
#include "motion_planning/utils/visualization.hpp"
#include "motion_planning/utils/params.hpp"
#include <unsupported/Eigen/Polynomials>  // for PolynomialSolver
#include "motion_planning/utils/kalman_filter.hpp"
#include "motion_planning/utils/kalman_filter_factory.hpp"
#include <fcl/fcl.h>




#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"

#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btConvexCast.h"
#include <gz/msgs/boolean.pb.h>

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




    bool isTrajectorySafeAgainstSingleObstacle(const Trajectory& trajectory, 
                                           double global_start_time, 
                                           const Obstacle& obstacle) const override;

    


    bool isPointInOrientedBox(
        const Eigen::Vector2d& point, 
        const Eigen::Vector2d& box_center, 
        double w, double h, double rotation
    ) const {
        // 1. Translate point relative to box center
        Eigen::Vector2d d = point - box_center;

        // 2. Rotate point into the box's local axis
        double cos_r = std::cos(-rotation);
        double sin_r = std::sin(-rotation);
        double local_x = d.x() * cos_r - d.y() * sin_r;
        double local_y = d.x() * sin_r + d.y() * cos_r;

        // 3. Simple AABB check in local coordinates
        return (std::abs(local_x) <= w / 2.0) && (std::abs(local_y) <= h / 2.0);
    }


    bool isInVelocityObstacle(
        const Eigen::Vector2d& robot_velocity,
        const Eigen::Vector2d& obs_velocity,
        const Eigen::Vector2d& pos_robot_to_obs,
        double combined_radius
    ) const;


    /**
     * Performs a full time-aware collision check.
     * return An std::optional containing the colliding obstacle if a collision is
     * predicted. Returns std::nullopt if the path is clear.
     */
    std::optional<Obstacle> getCollidingObstacle(const Trajectory& trajectory, double start_node_cost) const override;
    std::optional<Obstacle> getCollidingObstacleFCL(const Trajectory& trajectory, double start_node_cost) const override;
    std::optional<Obstacle> getCollidingObstacleBullet(const Trajectory& trajectory, double start_node_cost) const override;

    bool sweptBoxIntersection(const Eigen::Vector2d& p_r0, const Eigen::Vector2d& v_r, const Eigen::Vector2d& p_o0, const Eigen::Vector2d& v_o, double w, double h, double T_segment, double rotation, bool consider_rotation) const;
    bool sweptBoxIntersection3D(const Eigen::Vector3d& p_r0, const Eigen::Vector3d& v_r, const Eigen::Vector3d& p_o0, const Eigen::Vector3d& v_o, double w, double h, double d, double T, double rot) const;
 
    /**
     * return TRUE if the path is clear, FALSE if a collision is predicted.
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
        processLatestPoseInfo();
        
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        ObstacleVector snapshot_vec;
        for (auto const& [name, ob] : obstacle_positions_map_) {
            snapshot_vec.push_back(ob);
        }
        
        return {robot_position_, snapshot_vec};
    }

    const ObstacleVector& getLatestSnapshot() const {
        return obstacle_snapshot_;
    }
    ObstacleVector getCurrentObstacles() {
        processLatestPoseInfo();
        return obstacle_positions_;
    }


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
    // Calculates the squared distance from a point to a circular arc segment.
    double distanceSqrdPointToArc(
        const Eigen::Vector2d& p,
        const Eigen::Vector2d& start,
        const Eigen::Vector2d& end,
        const Eigen::Vector2d& center,
        double radius,
        bool is_clockwise) const
    {
        // Vector from point to circle center
        Eigen::Vector2d to_center = p - center;
        double dist_to_center_sq = to_center.squaredNorm();

        // Find the closest point on the full circle to p
        Eigen::Vector2d closest_point_on_circle = center + radius * to_center.normalized();

        // Check if this closest point lies within the arc segment
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

    bool checkRobotCollision(const Eigen::Vector2d& position, double yaw) const;
    bool checkRobotCollision(const Eigen::Vector3d& position, double yaw) const;


    double calculateNextFlip(const Obstacle& ob, double currentRobotTime) {
        auto info_it = obstacle_info_.find(ob.name);
        if (info_it == obstacle_info_.end() || !info_it->second.is_dynamic) return -1.0;
        const auto& info = info_it->second;

        // 1. Define turnaround boundaries
        Eigen::Vector2d pointA = info.initial_pose.head<2>();
        Eigen::Vector2d pointB = pointA + (info.direction.head<2>() * info.amplitude);

        // 2. CHATTERING FIX: Target the FURTHEST point
        // When we trigger a flip, we are by definition very close to one boundary.
        // We want to calculate the time to reach the OTHER boundary.
        double distA = (pointA - ob.position).norm();
        double distB = (pointB - ob.position).norm();

        // If we are closer to B, we just hit B, so our next target is A.
        // If we are closer to A, our next target is B.
        Eigen::Vector2d nextTarget = (distA < distB) ? pointB : pointA;
        std::string targetName = (distA < distB) ? "Point B (Limit)" : "Point A (Start)";

        // 3. Calculate time to reach the FAR boundary
        double distance = (nextTarget - ob.position).norm();
        double time_to_reach = distance / std::max(info.speed, 0.001);

        double nextChangeTime = currentRobotTime - time_to_reach;

        // RCLCPP_INFO(rclcpp::get_logger("ObstacleLogic"), 
        //     "[%s] New Target: %s | Dist: %.2f | T_now: %.2f | Next Flip at T: %.2f", 
        //     ob.name.c_str(), targetName.c_str(), distance, currentRobotTime, nextChangeTime);

        return nextChangeTime;
    }



    std::vector<std::string> detectTurnaroundEvents(double currentRobotTime) {
        std::vector<std::string> triggered_obstacles;
        
        for (auto& [name, ob] : obstacle_positions_map_) {
            if (!ob.is_dynamic) continue;

            // Trigger if current time budget T has dropped below predicted turn time
            if (ob.nextDirectionChangeTime > currentRobotTime) {
                
                // --- CHATTERING FIX: Velocity Direction Hysteresis ---
                // Only trigger if the turnaround has actually started or 
                // if we haven't updated the timer yet. 
                // We check the dot product of current velocity and the direction 
                // it WAS supposed to be going.
                auto info_it = obstacle_info_.find(name);
                const auto& info = info_it->second;

                // Define the vector of the current prediction tube
                // (You can store 'current_direction_sign' in the Obstacle struct for better accuracy)
                double current_dir_sign = (ob.velocity.dot(info.direction.head<2>()) > 0) ? 1.0 : -1.0;

                // If the predicted turn time is reached, we trigger the event.
                // The planner will then call calculateNextFlip which we will fix below.
                RCLCPP_WARN(rclcpp::get_logger("Obs Checker"), 
                    "EVENT TRIGGERED for [%s]: Turnaround expected at T=%.2f, Actual T=%.2f", 
                    name.c_str(), ob.nextDirectionChangeTime, currentRobotTime);
                    
                triggered_obstacles.push_back(name);
            }
        }
        return triggered_obstacles;
    }

// // NOTE: No "GazeboObstacleChecker::" prefix here because this is inside the .hpp class
// ObstacleVector checkAndRepairObstacles(double T_robot) {
//     ObstacleVector triggered_obs;
    
//     // Threshold to detect if velocity changed (e.g. hit a wall)
//     const double VELOCITY_CHANGE_THRESHOLD = 0.1; 

//     for (auto& [name, ob] : obstacle_positions_map_) {
//         if (!ob.is_dynamic) continue;

//         // --- 1. INITIALIZATION CHECK (Keep this, it is good!) ---
//         if (!ob.is_initialized_in_graph) {
//             ob.is_initialized_in_graph = true;
            
//             // Generate fresh tube from T_robot -> 0
//             ob.predicted_path = this->generatePrediction(ob, T_robot);
            
//             // Save current velocity so we can detect changes later
//             last_velocities_[name] = ob.velocity; 

//             triggered_obs.push_back(ob);
//             RCLCPP_INFO(rclcpp::get_logger("Obs Checker"), 
//                 "First-time activation for [%s] at T=%.2f", name.c_str(), T_robot);
//             continue; 
//         }

//         // --- 2. VELOCITY FLIP CHECK (The Robust Fix) ---
//         // Instead of a timer, we check if the velocity changed direction physically.
        
//         bool direction_changed = false;
        
//         // Check if we have history for this object
//         if (last_velocities_.find(name) != last_velocities_.end()) {
//             Eigen::Vector2d old_v = last_velocities_[name];
//             Eigen::Vector2d current_v = ob.velocity;
            
//             // If the difference is large, it hit a wall
//             if ((current_v - old_v).norm() > VELOCITY_CHANGE_THRESHOLD) {
//                 direction_changed = true;
                
//                 // RCLCPP_WARN(rclcpp::get_logger("RRTx_Trigger"), 
//                 //     "BOUNCE DETECTED for [%s]! Vel changed from [%.1f, %.1f] to [%.1f, %.1f]",
//                 //     name.c_str(), old_v.x(), old_v.y(), current_v.x(), current_v.y());
//             }
//         }

//         // Update history for next iteration
//         last_velocities_[name] = ob.velocity;

//         // --- 3. TRIGGER UPDATE IF FLIPPED ---
//         if (direction_changed) {
//             // Regenerate the tube starting exactly at T_robot
//             ob.predicted_path = this->generatePrediction(ob, T_robot);
            
//             // Note: We also reset the nextDirectionChangeTime just in case you use it elsewhere, 
//             // though we don't rely on it for detection anymore.
//             ob.nextDirectionChangeTime = this->calculateNextFlip(ob, T_robot);

//             triggered_obs.push_back(ob);
//         }
//     }
//     return triggered_obs;
// }
ObstacleVector checkAndRepairObstacles(double T_robot) {
    ObstacleVector triggered_obs;
    
    for (auto& [name, ob] : obstacle_positions_map_) {
        if (!ob.is_dynamic) continue;

        // --- 1. INITIALIZATION CHECK ---
        if (!ob.is_initialized_in_graph) {
            ob.is_initialized_in_graph = true;
            ob.predicted_path = this->generatePrediction(ob, T_robot);
            last_velocities_[name] = ob.velocity; 
            triggered_obs.push_back(ob);
            continue; 
        }

        // --- 2. CHECK GAZEBO SIGNAL ---
        bool did_turn = false;
        if (obstacle_turnaround_flags_[name]) {
            did_turn = true;
            obstacle_turnaround_flags_[name] = false;
            
            // --- ROBUST VELOCITY FLIP ---
            // We don't just do ob.velocity = -ob.velocity.
            // Instead, we project the velocity onto the Motion Axis, flip the sign of the projection,
            // and reconstruct the vector. This ensures we stay exactly on the line defined by the SDF.
            
            // 1. Calculate how much velocity we have along the motion axis
            double projection = ob.velocity.dot(ob.motion_axis);
            
            // 2. Flip the projection (Reverse direction)
            double flipped_projection = -projection;
            
            // 3. Reconstruct the velocity vector
            // New Velocity = (Axis * Flipped_Projection)
            // This preserves the exact speed scalar from the SDF but reverses direction.
            ob.velocity = ob.motion_axis * flipped_projection;
            
            RCLCPP_WARN(rclcpp::get_logger("Obs_Debug"), 
                "!!! TURNAROUND [%s] !!! | Velocity Corrected to (%.2f, %.2f)", 
                name.c_str(), ob.velocity.x(), ob.velocity.y());
        }

        // --- 3. UPDATE PREDICTION ---
        ob.predicted_path = this->generatePrediction(ob, T_robot);

        // --- 4. TRIGGER UPDATE ---
        if (did_turn) {
            triggered_obs.push_back(ob);
        }
        
        last_velocities_[name] = ob.velocity;
    }
    return triggered_obs;
}


std::vector<Eigen::Vector3d> generatePrediction(const Obstacle& ob, double current_time) const override;

    // // In gazebo_obstacle_checker.cpp
    // std::vector<Eigen::Vector3d> generatePrediction(
    //     const Obstacle& ob, 
    //     double currentTime) 
    // {
    //     std::vector<Eigen::Vector3d> path;
        
    //     // This is the temporal resolution (S.pathTimeStep in Julia).
    //     // A value of 1.0 to 3.0 is usually good for a 25s-40s horizon.
    //     const double dt_step = 2.0; 

    //     // Start at current time (e.g., 25.0) and step down to 0.0
    //     // We use a small epsilon (-1e-9) to ensure we include the T=0 point
    //     for (double t = currentTime; t >= -1e-9; t -= dt_step) {
            
    //         // How much time has passed since the robot "saw" the obstacle at 'ob.position'
    //         double elapsed = currentTime - t; 

    //         // Project position: p_future = p_now + (v * delta_t)
    //         Eigen::Vector2d future_pos = ob.position + (ob.velocity * elapsed);

    //         // Store as (X, Y, Time)
    //         path.emplace_back(future_pos.x(), future_pos.y(), t);
    //     }

    //     RCLCPP_DEBUG(rclcpp::get_logger("Predictor"), 
    //         "Generated prediction for %s: %zu points from T=%.2f to 0.0", 
    //         ob.name.c_str(), path.size(), currentTime);

    //     return path;
    // }

    void initializeDynamicObstacles(double currentRobotTime) {
        // 1. Ensure we have the latest positions from Gazebo
        this->processLatestPoseInfo();

        RCLCPP_INFO(rclcpp::get_logger("ObstacleChecker"), 
                    "Initializing Dynamic Obstacles at T=%.2f", currentRobotTime);

        for (auto& [name, ob] : obstacle_positions_map_) {
            if (ob.is_dynamic) {
                // Calculate the very first turnaround time
                ob.nextDirectionChangeTime = this->calculateNextFlip(ob, currentRobotTime);
                
                RCLCPP_INFO(rclcpp::get_logger("ObstacleChecker"), 
                    "-> [%s] logic initialized. First turnaround at T=%.2f", 
                    name.c_str(), ob.nextDirectionChangeTime);
            }
        }
    }


private:
    Eigen::Vector2d getObstaclePositionAtTime(const Obstacle& ob, double query_time) const;

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

    bool lineIntersectsBox3D(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& center, double w, double h, double d, double rot) const;


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
    bool use_bullet;
    int spatial_dim_;

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

    std::string footprint_type_;
    double robot_radius_; // Only used for circular footprint
    std::vector<Eigen::Vector2d> rectangular_footprint_; // Only used for rectangular footprint

    bool checkCircularCollisionHelper(const Eigen::Vector2d& robot_position, double robot_radius) const;
    bool checkRectangularCollisionHelper(const Eigen::Vector2d& position, double yaw) const;
    bool checkCircularCollisionHelper3D(const Eigen::Vector3d& robot_position, double robot_radius, double robot_height) const;
    bool checkRectangularCollisionHelper3D(const Eigen::Vector3d& position, double yaw) const;

    
    bool fastLineAABBIntersection(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& box_center, const Eigen::Vector3d& box_half_sizes) const;

    


    

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
    //  * Creates an FCL collision object from our custom Obstacle struct.
    //  * obstacle The obstacle to convert.
    //  * A shared pointer to an FCL CollisionObject.
    //  */
    // fcl::CollisionObjectd createFCLObject(const Obstacle& obstacle) const;


    // --- Bullet Physics Members ---
    // These are mutable to allow for lazy initialization inside a const method.
    mutable std::unique_ptr<btDefaultCollisionConfiguration> bullet_collision_config_;
    mutable std::unique_ptr<btCollisionDispatcher> bullet_dispatcher_;
    mutable std::unique_ptr<btBroadphaseInterface> bullet_broadphase_;
    mutable std::unique_ptr<btCollisionWorld> bullet_world_;

    mutable std::unordered_map<std::string, std::unique_ptr<btConvexShape>> bullet_shape_cache_;




        // Inside the private section:
    mutable std::vector<Obstacle> culprit_cache_;
    mutable std::set<std::string> culprit_names_;


    void recordCulprit(const Obstacle& obs) const;



    std::unordered_map<std::string, Eigen::Vector2d> last_velocities_;

    std::map<std::string, Obstacle> obstacle_positions_map_;



    // Map to store the turnaround flags received from Gazebo
    std::unordered_map<std::string, bool> obstacle_turnaround_flags_;



};
