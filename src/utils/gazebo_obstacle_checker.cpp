#include "motion_planning/utils/gazebo_obstacle_checker.hpp"

GazeboObstacleChecker::GazeboObstacleChecker(rclcpp::Clock::SharedPtr clock,
                                            const Params& params,
                                            const std::unordered_map<std::string, ObstacleInfo>& obstacle_info)
        : clock_(clock),
          obstacle_info_(obstacle_info),
          robot_position_(Eigen::Vector2d::Zero()),
          robot_orientation_(Eigen::VectorXd(4)) {  // Initialize orientation as a 4D vector for quaternion

    robot_model_name_ = params.getParam<std::string>("robot_model_name");
    world_name_ = params.getParam<std::string>("world_name");
    use_range = params.getParam<bool>("use_range");
    sensor_range = params.getParam<double>("sensor_range");
    inflation = params.getParam<double>("inflation");
    persistent_static_obstacles = params.getParam<bool>("persistent_static_obstacles");
    robot_position_ << params.getParam<double>("default_robot_x"), params.getParam<double>("default_robot_y");
    estimation = params.getParam<bool>("estimation");
    // Subscribe to the robot pose topic
    std::string robot_pose_topic = "/model/" + robot_model_name_ + "/tf";
    if (!gz_node_.Subscribe(robot_pose_topic, &GazeboObstacleChecker::robotPoseCallback, this)) {
            std::cerr << "Failed to subscribe to robot pose topic: " << robot_pose_topic << std::endl;
        } else {
            std::cout << "Successfully subscribed to robot pose topic: " << robot_pose_topic << std::endl;
    }
    

    std::string topic = "/world/" + world_name_ + "/pose/info";
    if (!gz_node_.Subscribe(topic, &GazeboObstacleChecker::poseInfoCallback, this)) {
        std::cerr << "Failed to subscribe to Gazebo topic: " << topic << std::endl;
    }


    path_pub_ = gz_node_.Advertise<gz::msgs::Pose_V>("/path");
    if (!path_pub_) {
        std::cerr << "Failed to advertise /path topic." << std::endl;
    } else {
        std::cout << "Successfully advertised /path topic." << std::endl;
    }



}

GazeboObstacleChecker::~GazeboObstacleChecker() = default;

// Method to publish a path
void GazeboObstacleChecker::publishPath(const std::vector<Eigen::VectorXd>& waypoints) {
    gz::msgs::Pose_V path_msg;

    for (const auto& waypoint : waypoints) {
        gz::msgs::Pose* pose = path_msg.add_pose();
        pose->mutable_position()->set_x(waypoint.x());
        pose->mutable_position()->set_y(waypoint.y());
        pose->mutable_position()->set_z(0.0);  // Assuming 2D path
    }

    // Publish the path
    if (!path_pub_.Publish(path_msg)) {
        std::cerr << "Failed to publish path to /path topic." << std::endl;
    }
}



bool GazeboObstacleChecker::isObstacleFree(const Eigen::VectorXd& start, const Eigen::VectorXd& end) const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    Eigen::Vector2d start2d = start.head<2>();
    Eigen::Vector2d end2d = end.head<2>();

    for (const auto& obstacle : obstacle_snapshot_) {
        const double inflated = obstacle.inflation;
        const Eigen::Vector2d& center = obstacle.position;
        
        if (obstacle.type == Obstacle::CIRCLE) {
            const double radius = obstacle.dimensions.circle.radius + inflated;
            if (lineIntersectsCircle(start2d, end2d, center, radius)) {
                return false;
            }
        } else {
            const double width = obstacle.dimensions.box.width + 2*inflated;
            const double height = obstacle.dimensions.box.height + 2*inflated;
            const double rotation = obstacle.dimensions.box.rotation;
            if (lineIntersectsRectangle(start2d, end2d, center, width, height, rotation)) {
                return false;
            }
        }
    }
    return true;
}

bool GazeboObstacleChecker::isObstacleFree(const Eigen::VectorXd& point) const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    Eigen::Vector2d point2d = point.head<2>();

    for (const auto& obstacle : obstacle_snapshot_) {
        const double inflated = obstacle.inflation;
        const Eigen::Vector2d& center = obstacle.position;
        
        if (obstacle.type == Obstacle::CIRCLE) {
            const double radius = obstacle.dimensions.circle.radius + inflated;
            if (pointIntersectsCircle(point2d, center, radius)) {
                return false;
            }
        } else {
            const double width = obstacle.dimensions.box.width + 2*inflated;
            const double height = obstacle.dimensions.box.height + 2*inflated;
            const double rotation = obstacle.dimensions.box.rotation;
            if (pointIntersectsRectangle(point2d, center, width, height, rotation)) {
                return false;
            }
        }
    }
    return true;
}


// Implementation of the new trajectory checking method
bool GazeboObstacleChecker::isObstacleFree(const std::vector<Eigen::VectorXd>& path) const {
    if (path.size() < 2) {
        return true; // A single point or empty path is considered collision-free
    }
    for (size_t i = 0; i < path.size() - 1; ++i) {
        // Use your existing line segment checker for each segment of the path
        if (!isObstacleFree(path[i], path[i + 1])) {
            return false; // If any segment is in collision, the whole path is
        }
    }
    return true; // All segments are clear
}


bool GazeboObstacleChecker::isTrajectorySafe(
    const Trajectory& trajectory,
    double global_edge_start_time
) const {
    // A nullopt from getCollidingObstacle means the path is safe.
    return !getCollidingObstacle(trajectory, global_edge_start_time).has_value();
}

// std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacle(
//     const Trajectory& trajectory,
//     double global_edge_start_time
// ) const {
//     std::lock_guard<std::mutex> lock(snapshot_mutex_);

//     if (trajectory.path_points.size() < 2) {
//         return std::nullopt;
//     }

//     // First, check against all static obstacles.
//     for (const auto& obstacle : obstacle_snapshot_) {
//         if (!obstacle.is_dynamic) {
//             for (size_t i = 0; i < trajectory.path_points.size() - 1; ++i) {
//                 const Eigen::Vector2d& seg_start = trajectory.path_points[i].head<2>();
//                 const Eigen::Vector2d& seg_end = trajectory.path_points[i+1].head<2>();

//                 if (obstacle.type == Obstacle::CIRCLE) {
//                     if (lineIntersectsCircle(seg_start, seg_end, obstacle.position, obstacle.dimensions.circle.radius + inflation)) {
//                         return obstacle;
//                     }
//                 } else { // BOX
//                      if (lineIntersectsRectangle(seg_start, seg_end, obstacle.position, obstacle.dimensions.box.width + 2*inflation, obstacle.dimensions.box.height + 2*inflation, obstacle.dimensions.box.rotation)) {
//                         return obstacle;
//                     }
//                 }
//             }
//         }
//     }

//     // --- Part 2: Check against DYNAMIC obstacles ---
//     const Eigen::VectorXd& start_state = trajectory.path_points.front();
//     // const double robot_duration = trajectory.cost; //in case i only use time as a cost in steer methods
//     const double robot_duration = trajectory.time_duration;
    
//     if (robot_duration < 1e-9) return std::nullopt;

//     const Eigen::Vector2d robot_vel = (trajectory.path_points.back().head<2>() - start_state.head<2>()) / robot_duration;

//     for (const auto& obstacle : obstacle_snapshot_) {
//         if (obstacle.is_dynamic) {
//             double obs_radius = (obstacle.type == Obstacle::CIRCLE)
//                               ? obstacle.dimensions.circle.radius
//                               : std::hypot(obstacle.dimensions.box.width, obstacle.dimensions.box.height) / 2.0;
            
//             const double combined_radius = obs_radius + inflation;

//             // Define the global time intervals for the check
//             const double t0 = global_edge_start_time; // Global time robot STARTS the edge.
//             const double T = robot_duration;   // Duration of the traversal.

//             // Get the time the obstacle snapshot was taken
//             // double t_snapshot = std::chrono::duration<double>(obstacle.last_update_time.time_since_epoch()).count();
//             double t_snapshot = obstacle.last_update_time.seconds(); // <-- MUCH SIMPLER


//             // Set up and solve the quadratic equation for collision time (τ)
//             const Eigen::Vector2d rel_vel = robot_vel - obstacle.velocity;
            
//             // Extrapolate both positions back to a common global reference time (τ=0)
//             const Eigen::Vector2d p_robot_at_tau_zero = start_state.head<2>() - robot_vel * t0;
//             const Eigen::Vector2d p_obs_at_tau_zero = obstacle.position - obstacle.velocity * t_snapshot;
//             const Eigen::Vector2d rel_pos_at_tau_zero = p_robot_at_tau_zero - p_obs_at_tau_zero;

//             double A = rel_vel.dot(rel_vel);
//             double B = 2 * rel_pos_at_tau_zero.dot(rel_vel);
//             double C = rel_pos_at_tau_zero.dot(rel_pos_at_tau_zero) - (combined_radius * combined_radius);

//             if (std::abs(A) < 1e-9) { // Parallel movement
//                 if (C <= 0) return obstacle; // Already overlapping
//                 continue;
//             }

//             double discriminant = B * B - 4 * A * C;
//             if (discriminant < 0) continue; // No intersection

//             double sqrt_disc = std::sqrt(discriminant);
//             double tau1 = (-B - sqrt_disc) / (2 * A);
//             double tau2 = (-B + sqrt_disc) / (2 * A);

//             // Check if the collision interval [min(τ1,τ2), max(τ1,τ2)]
//             // overlaps with the robot's traversal interval [t0, t0 + T].
//             if (std::max(t0, std::min(tau1, tau2)) <= std::min(t0 + T, std::max(tau1, tau2))) {
//                 return obstacle; // Collision is predicted to occur.
//             }
//         }
//     }

//     return std::nullopt; // No collisions found.
// }

// // WITH DEBUG 
// std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacle(
//     const Trajectory& trajectory,
//     double global_edge_start_time
// ) const {
//     std::lock_guard<std::mutex> lock(snapshot_mutex_);

//     if (trajectory.path_points.size() < 2) {
//         return std::nullopt;
//     }

//     // --- Stage 1: Static Obstacle Check (No changes needed here) ---
//     for (const auto& obstacle : obstacle_snapshot_) {
//         if (!obstacle.is_dynamic) {
//             for (size_t i = 0; i < trajectory.path_points.size() - 1; ++i) {
//                 const Eigen::Vector2d& seg_start = trajectory.path_points[i].head<2>();
//                 const Eigen::Vector2d& seg_end = trajectory.path_points[i+1].head<2>();

//                 if (obstacle.type == Obstacle::CIRCLE) {
//                     if (lineIntersectsCircle(seg_start, seg_end, obstacle.position, obstacle.dimensions.circle.radius + inflation)) {
//                         return obstacle;
//                     }
//                 } else { // BOX
//                      if (lineIntersectsRectangle(seg_start, seg_end, obstacle.position, obstacle.dimensions.box.width + 2*inflation, obstacle.dimensions.box.height + 2*inflation, obstacle.dimensions.box.rotation)) {
//                         return obstacle;
//                     }
//                 }
//             }
//         }
//     }

//     // --- Stage 2: Dynamic Obstacle Prediction ---
//     const Eigen::VectorXd& start_state = trajectory.path_points.front();
//     const double robot_duration = trajectory.time_duration;
    
//     if (robot_duration < 1e-9) return std::nullopt;

//     const Eigen::Vector2d robot_vel = (trajectory.path_points.back().head<2>() - start_state.head<2>()) / robot_duration;

//     // Check against every dynamic obstacle
//     for (const auto& obstacle : obstacle_snapshot_) {
//         if (obstacle.is_dynamic) {

//             // =================================== DEBUG BLOCK ===================================
//             // This block will print all the data used for the prediction for each dynamic obstacle.
            
//             // Set precision for cleaner output
//             std::cout << std::fixed << std::setprecision(4);

//             std::cout << "\n[Checker] Checking against dynamic obstacle...\n"
//                       << "  > Robot Traversal Starts (t0):    " << global_edge_start_time << "s\n"
//                       << "  > Robot Traversal Duration (T):   " << robot_duration << "s\n"
//                       << "  > Robot Traversal Ends:           " << global_edge_start_time + robot_duration << "s\n"
//                       << "  > Robot Velocity (vx, vy):        (" << robot_vel.x() << ", " << robot_vel.y() << ")\n"
//                       << "--------------------------------------------------\n"
//                       << "  > Obstacle Position (x, y):       (" << obstacle.position.x() << ", " << obstacle.position.y() << ")\n"
//                       << "  > Obstacle Velocity (vx, vy):     (" << obstacle.velocity.x() << ", " << obstacle.velocity.y() << ")\n"
//                       << "  > Obstacle Snapshot Time:         " << obstacle.last_update_time.seconds() << "s\n";
//             // =================================================================================

//             // --- LIKELY POINT OF FAILURE ---
//             // If you run your code and see that "Obstacle Velocity" is always (0.0000, 0.0000),
//             // it means your velocity estimation in `poseInfoCallback` is not working correctly.
//             // The predictive check CANNOT work with zero velocity.

//             double obs_radius = (obstacle.type == Obstacle::CIRCLE)
//                               ? obstacle.dimensions.circle.radius
//                               : std::hypot(obstacle.dimensions.box.width, obstacle.dimensions.box.height) / 2.0;
            
//             const double combined_radius = obs_radius + inflation;

//             const double t0 = global_edge_start_time;
//             const double T = robot_duration;
//             double t_snapshot = obstacle.last_update_time.seconds();

//             const Eigen::Vector2d rel_vel = robot_vel - obstacle.velocity;
            
//             const Eigen::Vector2d p_robot_at_tau_zero = start_state.head<2>() - robot_vel * t0;
//             const Eigen::Vector2d p_obs_at_tau_zero = obstacle.position - obstacle.velocity * t_snapshot;
//             const Eigen::Vector2d rel_pos_at_tau_zero = p_robot_at_tau_zero - p_obs_at_tau_zero;

//             double A = rel_vel.dot(rel_vel);
//             double B = 2 * rel_pos_at_tau_zero.dot(rel_vel);
//             double C = rel_pos_at_tau_zero.dot(rel_pos_at_tau_zero) - (combined_radius * combined_radius);

//             if (std::abs(A) < 1e-9) { 
//                 if (C <= 0) {
//                      std::cout << "  > RESULT: Collision (Parallel Overlap)\n";
//                     return obstacle;
//                 }
//                 std::cout << "  > RESULT: No Collision (Moving in parallel, no overlap)\n";
//                 continue;
//             }

//             double discriminant = B * B - 4 * A * C;

//             // =================================== DEBUG BLOCK ===================================
//             std::cout << "  > Quadratic Discriminant:         " << discriminant << "\n";
//             // =================================================================================

//             if (discriminant < 0) {
//                  std::cout << "  > RESULT: No Collision (Paths never cross at this radius)\n";
//                 continue;
//             }

//             double sqrt_disc = std::sqrt(discriminant);
//             double tau1 = (-B - sqrt_disc) / (2 * A);
//             double tau2 = (-B + sqrt_disc) / (2 * A);

//             double collision_start = std::min(tau1, tau2);
//             double collision_end = std::max(tau1, tau2);

//             // =================================== DEBUG BLOCK ===================================
//             std::cout << "  > Robot Interval:                 [" << t0 << ", " << t0 + T << "]\n"
//                       << "  > Collision Interval:             [" << collision_start << ", " << collision_end << "]\n";
//             // =================================================================================

//             if (std::max(t0, collision_start) <= std::min(t0 + T, collision_end)) {
//                  std::cout << "  > RESULT: PREDICTED COLLISION! Intervals overlap.\n";
//                 return obstacle;
//             } else {
//                  std::cout << "  > RESULT: No Collision (Intervals do not overlap)\n";
//             }
//         }
//     }

//     return std::nullopt;
// }



// // WITH DEBUG AND CORRECTED CURRENT POSITION CHECK
// // Will the robot and obstacle be in the same place at the same global time?
// std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacle(
//     const Trajectory& trajectory,
//     double global_edge_start_time
// ) const {
//     std::lock_guard<std::mutex> lock(snapshot_mutex_);

//     if (trajectory.path_points.size() < 2) {
//         return std::nullopt;
//     }

//     // Get the start and end points of the robot's path segment
//     const Eigen::Vector2d& robot_start_pos = trajectory.path_points.front().head<2>();
//     const Eigen::Vector2d& robot_end_pos = trajectory.path_points.back().head<2>();

//     for (const auto& obstacle : obstacle_snapshot_) {
//         // --- STAGE 1: Check against STATIC obstacles ---
//         if (!obstacle.is_dynamic) {
//             if (obstacle.type == Obstacle::CIRCLE) {
//                 if (lineIntersectsCircle(robot_start_pos, robot_end_pos, obstacle.position, obstacle.dimensions.circle.radius + inflation)) {
//                     // Static collision, return immediately
//                     return obstacle;
//                 }
//             } else { // BOX
//                  if (lineIntersectsRectangle(robot_start_pos, robot_end_pos, obstacle.position, obstacle.dimensions.box.width + 2*inflation, obstacle.dimensions.box.height + 2*inflation, obstacle.dimensions.box.rotation)) {
//                     // Static collision, return immediately
//                     return obstacle;
//                 }
//             }
//             // If no collision with this static obstacle, continue to the next one
//             continue;
//         }

//         // --- STAGE 2: Check against DYNAMIC obstacles ---
//         // This now has two parts: an immediate check and a predictive check.

//         // --- Part A: IMMEDIATE STATIC CHECK (The Critical Fix) ---
//         // First, check if the edge collides with the obstacle's CURRENT position.
//         // This answers the question: "Is this path blocked RIGHT NOW?"
//         if (obstacle.type == Obstacle::CIRCLE) {
//             if (lineIntersectsCircle(robot_start_pos, robot_end_pos, obstacle.position, obstacle.dimensions.circle.radius + inflation)) {
//                 // std::cout << "\n[Checker] RESULT: IMMEDIATE COLLISION with dynamic obstacle's current position.\n";
//                 return obstacle;
//             }
//         } else { // BOX
//             if (lineIntersectsRectangle(robot_start_pos, robot_end_pos, obstacle.position, obstacle.dimensions.box.width + 2*inflation, obstacle.dimensions.box.height + 2*inflation, obstacle.dimensions.box.rotation)) {
//                 // std::cout << "\n[Checker] RESULT: IMMEDIATE COLLISION with dynamic obstacle's current position.\n";
//                 return obstacle;
//             }
//         }

//         // --- Part B: PREDICTIVE CHECK (Your existing, correct logic) ---
//         // If the path is not blocked right now, then check if it will be blocked in the future at the scheduled time.
//         // This answers the question: "Will this path be blocked ON SCHEDULE?"
//         const double robot_duration = trajectory.time_duration;
//         if (robot_duration < 1e-9) continue; // Cannot predict for zero-duration path

//         const Eigen::Vector2d robot_vel = (robot_end_pos - robot_start_pos) / robot_duration;
//         double obs_radius = (obstacle.type == Obstacle::CIRCLE)
//                           ? obstacle.dimensions.circle.radius
//                           : std::hypot(obstacle.dimensions.box.width, obstacle.dimensions.box.height) / 2.0;
//         double robot_radius = 0.0;
//         const double combined_radius = obs_radius + robot_radius;

//         // // =================================== DEBUG BLOCK (Preserved) ===================================
//         // std::cout << std::fixed << std::setprecision(4);
//         // std::cout << "\n[Checker] Checking against dynamic obstacle...\n"
//         //           << "  > Robot Traversal Starts (t0):    " << global_edge_start_time << "s\n"
//         //           << "  > Robot Traversal Duration (T):   " << robot_duration << "s\n"
//         //           << "  > Robot Traversal Ends:           " << global_edge_start_time + robot_duration << "s\n"
//         //           << "  > Robot Velocity (vx, vy):        (" << robot_vel.x() << ", " << robot_vel.y() << ")\n"
//         //           << "--------------------------------------------------\n"
//         //           << "  > Obstacle Position (x, y):       (" << obstacle.position.x() << ", " << obstacle.position.y() << ")\n"
//         //           << "  > Obstacle Velocity (vx, vy):     (" << obstacle.velocity.x() << ", " << obstacle.velocity.y() << ")\n"
//         //           << "  > Obstacle Snapshot Time:         " << obstacle.last_update_time.seconds() << "s\n";
//         // // ==============================================================================================

//         const double t0 = global_edge_start_time;
//         const double T = robot_duration;
//         double t_snapshot = obstacle.last_update_time.seconds();

//         const Eigen::Vector2d rel_vel = robot_vel - obstacle.velocity;
        
//         const Eigen::Vector2d p_robot_at_tau_zero = robot_start_pos - robot_vel * t0;
//         const Eigen::Vector2d p_obs_at_tau_zero = obstacle.position - obstacle.velocity * t_snapshot;
//         const Eigen::Vector2d rel_pos_at_tau_zero = p_robot_at_tau_zero - p_obs_at_tau_zero;

//         double A = rel_vel.dot(rel_vel);
//         double B = 2 * rel_pos_at_tau_zero.dot(rel_vel);
//         double C = rel_pos_at_tau_zero.dot(rel_pos_at_tau_zero) - (combined_radius * combined_radius);

//         if (std::abs(A) < 1e-9) {
//             if (C <= 0) {
//                 //  std::cout << "  > RESULT: PREDICTED COLLISION! (Parallel Overlap)\n";
//                 return obstacle;
//             }
//             // std::cout << "  > RESULT: No Collision (Moving in parallel, no overlap)\n";
//             continue;
//         }

//         double discriminant = B * B - 4 * A * C;
//         // std::cout << "  > Quadratic Discriminant:         " << discriminant << "\n";
        
//         if (discriminant < 0) {
//             //  std::cout << "  > RESULT: No Collision (Paths never cross at this radius)\n";
//             continue;
//         }

//         double sqrt_disc = std::sqrt(discriminant);
//         double tau1 = (-B - sqrt_disc) / (2 * A);
//         double tau2 = (-B + sqrt_disc) / (2 * A);

//         double collision_start = std::min(tau1, tau2);
//         double collision_end = std::max(tau1, tau2);

//         // std::cout << "  > Robot Interval:                 [" << t0 << ", " << t0 + T << "]\n"
//                 //   << "  > Collision Interval:             [" << collision_start << ", " << collision_end << "]\n";

//         if (std::max(t0, collision_start) <= std::min(t0 + T, collision_end)) {
//             //  std::cout << "  > RESULT: PREDICTED COLLISION! Intervals overlap.\n";
//             return obstacle;
//         } else {
//             //  std::cout << "  > RESULT: No Collision (Intervals do not overlap)\n";
//         }
//     }

//     return std::nullopt; // No collisions of any kind were found for any obstacle.
// }




// // --- REWRITTEN getCollidingObstacle with improved logic ---
// std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacle(
//     const Trajectory& trajectory,
//     double global_edge_start_time
// ) const {
//     std::lock_guard<std::mutex> lock(snapshot_mutex_);

//     if (trajectory.path_points.size() < 2) {
//         return std::nullopt;
//     }

//     const Eigen::Vector2d& robot_start_pos = trajectory.path_points.front().head<2>();
//     const Eigen::Vector2d& robot_end_pos = trajectory.path_points.back().head<2>();

//     for (const auto& obstacle : obstacle_snapshot_) {
        
//         // --- Step 1: Define Obstacle Geometry ---
//         double obs_raw_radius = 0.0;
//         if (obstacle.type == Obstacle::CIRCLE) {
//             obs_raw_radius = obstacle.dimensions.circle.radius;
//         } else { // BOX
//             obs_raw_radius = std::hypot(obstacle.dimensions.box.width / 2.0, obstacle.dimensions.box.height / 2.0);
//         }

//         // The 'inflation' parameter IS our robot's effective radius for collision checking.
//         const double robot_radius = inflation;
//         const double combined_radius = obs_raw_radius + robot_radius;

//         // --- Step 2: Broad-Phase Check (from Julia example) ---
//         // First, do a quick check to see if the path segment could even possibly be in collision.
//         double dist_sq = distanceSqrdPointToSegment(obstacle.position, robot_start_pos, robot_end_pos);
//         if (dist_sq > (combined_radius * combined_radius)) {
//             continue; // Path is too far from the obstacle's center. Safe to skip.
//         }

//         // --- Step 3: Narrow-Phase Check (Only if broad-phase passes) ---

//         // A) Immediate Static Collision Check (for ALL obstacle types)
//         bool is_colliding_now = false;
//         if (obstacle.type == Obstacle::CIRCLE) {
//             is_colliding_now = lineIntersectsCircle(robot_start_pos, robot_end_pos, obstacle.position, combined_radius);
//         } else { // BOX - Use full width/height including inflation
//             double inflated_width = obstacle.dimensions.box.width + 2 * robot_radius;
//             double inflated_height = obstacle.dimensions.box.height + 2 * robot_radius;
//             is_colliding_now = lineIntersectsRectangle(robot_start_pos, robot_end_pos, obstacle.position, inflated_width, inflated_height, obstacle.dimensions.box.rotation);
//         }

//         // If it's colliding with the current position, that's enough to reject the path.
//         if (is_colliding_now) {
//             return obstacle;
//         }

//         // B) Predictive Check (ONLY for DYNAMIC obstacles)
//         if (obstacle.is_dynamic) {
//             const double robot_duration = trajectory.time_duration;
//             if (robot_duration < 1e-9) continue;

//             const Eigen::Vector2d robot_vel = (robot_end_pos - robot_start_pos) / robot_duration;
//             const double t0 = global_edge_start_time;
//             const double T = robot_duration;
//             double t_snapshot = obstacle.last_update_time.seconds();
//             const Eigen::Vector2d rel_vel = robot_vel - obstacle.velocity;
//             const Eigen::Vector2d p_robot_at_tau_zero = robot_start_pos - robot_vel * t0;
//             const Eigen::Vector2d p_obs_at_tau_zero = obstacle.position - obstacle.velocity * t_snapshot;
//             const Eigen::Vector2d rel_pos_at_tau_zero = p_robot_at_tau_zero - p_obs_at_tau_zero;

//             double A = rel_vel.dot(rel_vel);
//             double B = 2 * rel_pos_at_tau_zero.dot(rel_vel);
//             double C = rel_pos_at_tau_zero.dot(rel_pos_at_tau_zero) - (combined_radius * combined_radius);

//             if (std::abs(A) < 1e-9) {
//                 if (C <= 0) return obstacle;
//                 continue;
//             }

//             double discriminant = B * B - 4 * A * C;
//             if (discriminant < 0) continue;

//             double sqrt_disc = std::sqrt(discriminant);
//             double tau1 = (-B - sqrt_disc) / (2 * A);
//             double tau2 = (-B + sqrt_disc) / (2 * A);

//             if (std::max(t0, std::min(tau1, tau2)) <= std::min(t0 + T, std::max(tau1, tau2))) {
//                 return obstacle; // Predicted collision
//             }
//         }
//     }

//     return std::nullopt; // No collisions of any kind found
// }



 // withacceleration!
std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacle(
    const Trajectory& trajectory,
    double global_edge_start_time
) const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);

    // Need at least a start and end
    if (trajectory.path_points.size() < 2) {
        return std::nullopt;
    }

    // Extract robot start/end in 2D
    const Eigen::Vector2d  p_r0 = trajectory.path_points.front().head<2>();
    const Eigen::Vector2d  p_rT = trajectory.path_points.back().head<2>();
    const double           T    = trajectory.time_duration;

    // Precompute robot velocity (zero if T≈0)
    Eigen::Vector2d v_r = Eigen::Vector2d::Zero();
    if (T > 1e-9) {
        v_r = (p_rT - p_r0) / T;
    }

    for (auto const& obs : obstacle_snapshot_) {
        // 1) Combined (inflated) radius
        double obs_radius = (obs.type == Obstacle::CIRCLE)
                          ? obs.dimensions.circle.radius
                          : std::hypot(obs.dimensions.box.width/2.0,
                                       obs.dimensions.box.height/2.0);
        double R       = obs_radius + inflation;
        double R_sq    = R*R;

        // 2) Broad-phase: quick segment-to-point distance
        if (distanceSqrdPointToSegment(obs.position, p_r0, p_rT) > R_sq) {
            continue;
        }

        // 3) τ=0 static check (catch immediate collision)
        bool hit0 = false;
        if (obs.type == Obstacle::CIRCLE) {
            hit0 = lineIntersectsCircle(p_r0, p_rT, obs.position, R);
        } else {
            double w = obs.dimensions.box.width  + 2*inflation;
            double h = obs.dimensions.box.height + 2*inflation;
            hit0 = lineIntersectsRectangle(
                     p_r0, p_rT,
                     obs.position, w, h,
                     obs.dimensions.box.rotation);
        }
        if (hit0) {
            return obs;
        }

        // 4) If dynamic, do predictive quartic solve for τ∈(0,T]
        if (obs.is_dynamic && T > 1e-9) {
            // a) Bring obstacle state to τ=0
            double t_snap = obs.last_update_time.seconds();
            Eigen::Vector2d p_o_snap = obs.position;
            Eigen::Vector2d v_o      = obs.velocity;
            Eigen::Vector2d a_o      = obs.acceleration;

            double Δt = global_edge_start_time - t_snap;
            Eigen::Vector2d p_o0 = p_o_snap
                                + v_o * Δt
                                + 0.5 * a_o * (Δt*Δt);
            Eigen::Vector2d v_o0 = v_o + a_o * Δt;

            // b) Relative motion polynomial: r(τ)=p0+v1 τ+v2 τ²
            Eigen::Vector2d p0 = p_r0 - p_o0;
            Eigen::Vector2d v1 = v_r  - v_o0;
            Eigen::Vector2d v2 = -0.5 * a_o;

            // c) Build quartic ‖r(τ)‖² = R² → A4τ⁴+…+A0=0
            double A4 = v2.dot(v2);
            double A3 = 2.0 * v1.dot(v2);
            double A2 = 2.0 * p0.dot(v2) + v1.dot(v1);
            double A1 = 2.0 * p0.dot(v1);
            double A0 = p0.dot(p0) - R_sq;

            Eigen::Matrix<double,5,1> coeffs;
            // IMPORTANT: ascending order τ⁰…τ⁴
            coeffs << A0, A1, A2, A3, A4;

            Eigen::PolynomialSolver<double,4> solver;
            solver.compute(coeffs);

            auto roots = solver.roots();  // std::vector<std::complex<double>>
            for (auto const& ζ : roots) {
                if (std::abs(ζ.imag()) > 1e-6) continue; 
                double τ = ζ.real();
                if (τ > 1e-9 && τ <= T) {
                    // collision at time τ>0
                    return obs;
                }
            }
        }
        // otherwise static and no hit0 → safe
    }

    return std::nullopt;
}

Eigen::Vector2d GazeboObstacleChecker::getRobotPosition() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return robot_position_;
}

Eigen::VectorXd GazeboObstacleChecker::getRobotOrientation() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return robot_orientation_;
}

const std::vector<Obstacle>& GazeboObstacleChecker::getObstaclePositions() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return obstacle_positions_;
}

Eigen::VectorXd GazeboObstacleChecker::getRobotEulerAngles() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return quaternionToEuler(robot_orientation_);
}

Eigen::VectorXd GazeboObstacleChecker::quaternionToEuler(const Eigen::VectorXd& quaternion) const {
    // Ensure the quaternion is of size 4 (x, y, z, w)
    if (quaternion.size() != 4) {
        throw std::invalid_argument("Quaternion must be a 4D vector (x, y, z, w).");
    }

    // Extract quaternion components
    double x = quaternion(0);
    double y = quaternion(1);
    double z = quaternion(2);
    double w = quaternion(3);

    // Convert quaternion to Euler angles (roll, pitch, yaw)
    double roll, pitch, yaw;

    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1) {
        pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    } else {
        pitch = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    // Return Euler angles as a 3D vector
    Eigen::VectorXd euler_angles(3);
    euler_angles << roll, pitch, yaw;
    return euler_angles;
}


// // Pose info callback with finite difference estimation (not good!) --> except maybe for constant velocity model, then its good!
// void GazeboObstacleChecker::poseInfoCallback(const gz::msgs::Pose_V& msg) {
//     std::lock_guard<std::mutex> lock(snapshot_mutex_);
//     obstacle_positions_.clear();
//     std::vector<Obstacle> current_dynamic_obstacles;
//     std::unordered_map<std::string, Obstacle> current_obstacles_map;

//     // Get the current simulation time from the stored clock
//     rclcpp::Time now = clock_->now();
//     // std::cout << "Current sim time: " << now.seconds() << " s" << std::endl;

//     // Update robot position
//     for (int i = 0; i < msg.pose_size(); ++i) {
//         const auto& pose = msg.pose(i);
//         if (pose.name() == robot_model_name_) {
//             robot_position_ = Eigen::Vector2d(pose.position().x(), pose.position().y());
//             break;
//         }
//     }

//     // Process obstacles
//     for (int i = 0; i < msg.pose_size(); ++i) {
//         const auto& pose = msg.pose(i);
//         const std::string name = pose.name();
//         Eigen::Vector2d position(pose.position().x(), pose.position().y());

//         if (name == robot_model_name_) continue;

//         bool is_cylinder = name.find("cylinder") != std::string::npos;
//         bool is_box = name.find("box") != std::string::npos;
//         bool is_static = name.find("static_") != std::string::npos;
//         bool is_moving = name.find("moving_") != std::string::npos;

//         if (!is_cylinder && !is_box) continue;

//         auto info_it = obstacle_info_.find(name);
//         // if (info_it == obstacle_info_.end()) continue;

//         // Create obstacle object
//         Obstacle obstacle;
//         if (is_cylinder) {
//             // obstacle = Obstacle(position, info_it->second.radius, inflation);
//             double radius = (info_it != obstacle_info_.end()) ? info_it->second.radius : 5.0;
//             obstacle = Obstacle(position, radius, inflation, is_moving);
//         } else {
//             double width = (info_it != obstacle_info_.end()) ? info_it->second.width : 10.0;
//             double height = (info_it != obstacle_info_.end()) ? info_it->second.height : 10.0;
            
//             Eigen::Vector4d quat(
//                 pose.orientation().x(),
//                 pose.orientation().y(),
//                 pose.orientation().z(),
//                 pose.orientation().w()
//             );
//             double yaw = calculateYawFromQuaternion(quat);
//             obstacle = Obstacle(position, width, height, yaw, inflation, is_moving);
//         }

//         const bool within_range = !use_range || 
//             (robot_position_ - position).norm() < sensor_range;

//         // Handle static obstacles
//         if (is_static) {
//             if (persistent_static_obstacles) {
//                 auto map_it = static_obstacle_positions_.find(name);
                
//                 if (map_it == static_obstacle_positions_.end()) {
//                     // First detection: only store if in range
//                     if (within_range) {
//                         static_obstacle_positions_[name] = obstacle;
//                     }
//                 } else {
//                     // Update position but don't add yet
//                     map_it->second.position = position;
//                 }
//             }
            
//             // Add to current frame if in range
//             if (within_range) {
//                 obstacle_positions_.push_back(obstacle);
//             }
//         }
//         // Handle dynamic obstacles
//         else if (is_moving && within_range) {

//             obstacle.last_update_time = now;
//             // 1. FIRST, perform velocity estimation on the 'obstacle' object.
//             //    This gives it a non-zero velocity.
//             if(estimation) {
//                 auto prev_it = previous_obstacle_states_.find(name);
//                 if (prev_it != previous_obstacle_states_.end()) {
//                     const auto& prev_obstacle = prev_it->second;
//                     double dt = (now - prev_obstacle.last_update_time).seconds();

//                     if (dt > 1e-6) {
//                         obstacle.velocity = (obstacle.position - prev_obstacle.position) / dt;
//                         obstacle.acceleration = (obstacle.velocity - prev_obstacle.velocity) / dt;
//                     }
//                     else {
//                         // If dt is too small, reuse previous values to avoid instability
//                         obstacle.velocity = prev_obstacle.velocity;
//                         obstacle.acceleration = prev_obstacle.acceleration;
//                     }
//                 } else {
//                     // If we've never seen this obstacle before, assume zero velocity and acceleration
//                     obstacle.velocity.setZero();
//                     obstacle.acceleration.setZero();
//                 }
//             }

//             // 2. NOW that the velocity is calculated, add the fully updated obstacle
//             //    to the list of dynamic obstacles for this frame.
//             current_dynamic_obstacles.push_back(obstacle);
//             current_obstacles_map[name] = obstacle;
//         }


//     }

//     // Add persistent static obstacles (even if currently out of range)
//     if (persistent_static_obstacles) {
//         for (const auto& [name, static_obs] : static_obstacle_positions_) {
//             // Check if not already added from current detection
//             bool exists = std::any_of(
//                 obstacle_positions_.begin(),
//                 obstacle_positions_.end(),
//                 [&](const Obstacle& o) {
//                     return o.position == static_obs.position && 
//                            o.type == static_obs.type;
//                 }
//             );
            
//             if (!exists) {
//                 obstacle_positions_.push_back(static_obs);
//             }
//         }
//     }

//     // Store the current state for the next velocity calculation
//     previous_obstacle_states_ = current_obstacles_map;

//     // Add dynamic obstacles
//     obstacle_positions_.insert(obstacle_positions_.end(),
//                              current_dynamic_obstacles.begin(),
//                              current_dynamic_obstacles.end());

//     // // Debug output
//     // std::cout << "\n===== Current Obstacle Positions =====" << "\n";
//     // for (size_t i = 0; i < obstacle_positions_.size(); ++i) {
//     //     const auto& obs = obstacle_positions_[i];
//     //     std::string type = (obs.type == Obstacle::CIRCLE) ? "Cylinder" : "Box";
//     //     std::string state = (i >= obstacle_positions_.size() - current_dynamic_obstacles.size()) 
//     //                       ? "DYNAMIC" : "STATIC";
//     //     std::cout << "Obstacle " << i+1 << ": " << type
//     //             << " at (" << obs.position.x() << ", " << obs.position.y() << ")"
//     //             << " [" << state << "]" << "\n"
//     // }
//     // std::cout << "=====================================\n" << "\n";


//     // // Debug output using the map 
//     // std::cout << "\n===== Current Obstacle Info =====" << "\n";
//     // for(auto& obs: current_obstacles_map) {
//     //     std::string state = obs.second.is_dynamic ? "DYNAMIC" : "STATIC";
//     //     std::cout<<"Obstalce: "<<obs.first <<" at (" <<obs.second.position.x() <<", "<<obs.second.position.y()<< ")\n"
//     //              << " with vel (" << obs.second.velocity.x() << ", " << obs.second.velocity.y() << ")\n"
//     //              << " with state [" << state << "] \n";
//     // }
// }


// KF Estimation with constant acc model (the next model that can be used is Singer Acceleration Model)
void GazeboObstacleChecker::poseInfoCallback(const gz::msgs::Pose_V& msg) {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    obstacle_positions_.clear();
    std::vector<Obstacle> current_dynamic_obstacles;

    // Get the current simulation time from the stored clock
    rclcpp::Time now = clock_->now();

    // Update robot position
    for (int i = 0; i < msg.pose_size(); ++i) {
        const auto& pose = msg.pose(i);
        if (pose.name() == robot_model_name_) {
            robot_position_ = Eigen::Vector2d(pose.position().x(), pose.position().y());
            break;
        }
    }

    // Process obstacles
    for (int i = 0; i < msg.pose_size(); ++i) {
        const auto& pose = msg.pose(i);
        const std::string name = pose.name();
        Eigen::Vector2d position(pose.position().x(), pose.position().y());

        if (name == robot_model_name_) continue;

        bool is_cylinder = name.find("cylinder") != std::string::npos;
        bool is_box = name.find("box") != std::string::npos;
        bool is_static = name.find("static_") != std::string::npos;
        bool is_moving = name.find("moving_") != std::string::npos;

        if (!is_cylinder && !is_box) continue;

        auto info_it = obstacle_info_.find(name);

        // Create obstacle object
        Obstacle obstacle;
        obstacle.is_dynamic = is_moving;
        if (is_cylinder) {
            obstacle.type = Obstacle::CIRCLE;
            double radius = (info_it != obstacle_info_.end()) ? info_it->second.radius : 5.0;
            obstacle = Obstacle(position, radius, inflation, is_moving);
        } else {
            obstacle.type = Obstacle::BOX;
            double width = (info_it != obstacle_info_.end()) ? info_it->second.width : 10.0;
            double height = (info_it != obstacle_info_.end()) ? info_it->second.height : 10.0;
            
            Eigen::Vector4d quat(
                pose.orientation().x(),
                pose.orientation().y(),
                pose.orientation().z(),
                pose.orientation().w()
            );
            double yaw = calculateYawFromQuaternion(quat);
            obstacle = Obstacle(position, width, height, yaw, inflation, is_moving);
        }

        const bool within_range = !use_range || 
            (robot_position_ - position).norm() < sensor_range;

        // Handle static obstacles
        if (is_static) {
            if (persistent_static_obstacles) {
                auto map_it = static_obstacle_positions_.find(name);
                
                if (map_it == static_obstacle_positions_.end()) {
                    if (within_range) {
                        static_obstacle_positions_[name] = obstacle;
                    }
                } else {
                    map_it->second.position = position;
                }
            }
            
            if (within_range) {
                obstacle_positions_.push_back(obstacle);
            }
        }
        // Handle dynamic obstacles
        else if (is_moving && within_range) {
            // This is the new block that replaces finite difference with a Kalman Filter.
            if(estimation) {
                auto filter_it = obstacle_filters_.find(name);
                if (filter_it == obstacle_filters_.end()) {
                    // First time seeing this obstacle, initialize a new filter.
                    

                    /////////////////Singer model///////////
                    // Define the Singer parameters for this type of obstacle.
                    // These are the values you will tune.
                    double sigma_a = 180.0; // Expected max acceleration (e.g., m/s^2)
                    double alpha = 0.1;   // Maneuver frequency (e.g., 1/s)

                    // Create the filter with the new constructor
                    KalmanFilter new_filter(alpha, sigma_a);
                    
                    Eigen::VectorXd initial_state(6);
                    initial_state << obstacle.position.x(), obstacle.position.y(), 0, 0, 0, 0;
                    new_filter.init(initial_state);
                    
                    filter_it = obstacle_filters_.emplace(name, new_filter).first;
                    // ////////////////Constant acc model////////////
                    // KalmanFilter new_filter;
                    // Eigen::VectorXd initial_state(6);
                    // initial_state << obstacle.position.x(), obstacle.position.y(), 0, 0, 0, 0; // px, py, vx, vy, ax, ay
                    // new_filter.init(initial_state);
                    // filter_it = obstacle_filters_.emplace(name, new_filter).first;
                    // //////////////////////////////

                    
                    // On first sight, velocity and acceleration are zero.
                    obstacle.velocity.setZero();
                    obstacle.acceleration.setZero();

                } else {
                    // This obstacle has an existing filter. Predict and Update.
                    double dt = (now - obstacle_filters_times_[name]).seconds();

                    // Prevent instability from tiny or zero dt
                    if (dt <= 1e-6) {
                        dt = 0.016; // Fallback to a reasonable timestep, e.g., ~60Hz
                    }

                    // 1. Predict step: Estimate where the filter thinks the obstacle should be.
                    filter_it->second.predict(dt);

                    // 2. Update step: Correct the prediction with the new measurement.
                    Eigen::VectorXd measurement(2);
                    measurement << obstacle.position.x(), obstacle.position.y();
                    filter_it->second.update(measurement);

                    // 3. Get the smoothed state from the filter to use in planning.
                    Eigen::VectorXd estimated_state = filter_it->second.getState();
                    obstacle.velocity << estimated_state(2), estimated_state(3);
                    obstacle.acceleration << estimated_state(4), estimated_state(5);
                }
                 // Store the timestamp of this update for the next iteration's dt calculation
                obstacle_filters_times_[name] = now;
            }

            // Store the last update time on the obstacle object itself for the collision checker
            obstacle.last_update_time = now;
            current_dynamic_obstacles.push_back(obstacle);
            
            // // --- DEBUG OUTPUT BLOCK ---
            // std::cout << std::fixed << std::setprecision(3);
            // std::cout << "--- Obstacle [" << name << "] at " << now.seconds() << "s ---\n"
            //           << "  Raw Position:      (" << position.x() << ", " << position.y() << ")\n"
            //           << "  Kalman Velocity:   (" << obstacle.velocity.x() << ", " << obstacle.velocity.y() << ")\n"
            //           << "  Kalman Accel:      (" << obstacle.acceleration.x() << ", " << obstacle.acceleration.y() << ")\n";
        }
    }

    // Add persistent static obstacles (even if currently out of range)
    if (persistent_static_obstacles) {
        for (const auto& [name, static_obs] : static_obstacle_positions_) {
            bool exists = std::any_of(
                obstacle_positions_.begin(),
                obstacle_positions_.end(),
                [&](const Obstacle& o) {
                    return o.position == static_obs.position && 
                           o.type == static_obs.type;
                }
            );
            
            if (!exists) {
                obstacle_positions_.push_back(static_obs);
            }
        }
    }

    // Add all processed dynamic obstacles to the main list
    obstacle_positions_.insert(obstacle_positions_.end(),
                             current_dynamic_obstacles.begin(),
                             current_dynamic_obstacles.end());
}






double GazeboObstacleChecker::calculateYawFromQuaternion(const Eigen::VectorXd& quaternion) {
    // Ensure the quaternion is valid (x, y, z, w)
    if (quaternion.size() != 4) {
        throw std::invalid_argument("Quaternion must be a 4D vector (x, y, z, w).");
    }

    // Extract quaternion components
    double x = quaternion[0];
    double y = quaternion[1];
    double z = quaternion[2];
    double w = quaternion[3];

    // Convert quaternion to yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}



void GazeboObstacleChecker::robotPoseCallback(const gz::msgs::Pose_V& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (msg.pose_size() > 0) {
        const auto& pose = msg.pose(0);

        // Update robot position
        // robot_position_ = Eigen::Vector2d(pose.position().x(), pose.position().y());

        // Update robot orientation (quaternion)
        robot_orientation_ = Eigen::VectorXd(4);
        robot_orientation_ << pose.orientation().x(),
                              pose.orientation().y(),
                              pose.orientation().z(),
                              pose.orientation().w();

        // Calculate yaw from quaternion for debug!
        // double yaw = calculateYawFromQuaternion(robot_orientation_);
        // std::cout<< "ROBOT YAW: " << yaw <<"\n";
    }

}


bool GazeboObstacleChecker::lineIntersectsCircle(const Eigen::Vector2d& start,
                                                 const Eigen::Vector2d& end,
                                                 const Eigen::Vector2d& center,
                                                 double radius) {
    // Check if either endpoint is inside the circle --> because in case BOTH the points lie in the obstalce then not intersection can be detected by the following procedure so its better to put this check!
    if ((start - center).norm() <= radius || (end - center).norm() <= radius) {
        return true;
    }

    const Eigen::Vector2d d = end - start;
    const Eigen::Vector2d f = start - center;
    
    const double a = d.dot(d);
    const double b = 2 * f.dot(d);
    const double c = f.dot(f) - radius * radius;

    double discriminant = b * b - 4 * a * c;
    
    // If the discriminant is negative, no intersection with the circle's boundary
    if (discriminant < 0) return false;

    // Compute the square root of the discriminant
    discriminant = std::sqrt(discriminant);

    // Calculate the parametric intersection points along the line
    const double t1 = (-b - discriminant) / (2 * a);
    const double t2 = (-b + discriminant) / (2 * a);

    // Check if either intersection point lies within the bounds of the segment
    return (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1);
}

bool GazeboObstacleChecker::pointIntersectsCircle(const Eigen::Vector2d& point,
                                                  const Eigen::Vector2d& center,
                                                  double radius) {
    // Calculate the squared distance between the point and the center of the circle
    const double squaredDistance = (point - center).squaredNorm();

    // Check if the squared distance is less than or equal to the squared radius
    return squaredDistance <= (radius * radius);
}




std::vector<Obstacle> GazeboObstacleChecker::getObstacles() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<Obstacle> filtered_obstacles;
    
    for (const auto& obstacle : obstacle_positions_) {
        if (use_range) {
            // Calculate distance from robot to obstacle
            double distance = (robot_position_ - obstacle.position).norm();
            
            // Only include obstacles within sensor range
            if (distance <= sensor_range) {
                filtered_obstacles.push_back(obstacle);
            }
        } else {
            // Include all obstacles if range checking is disabled
            filtered_obstacles.push_back(obstacle);
        }
    }
    
    return filtered_obstacles;
}



// Add rectangle collision detection implementations
bool GazeboObstacleChecker::lineIntersectsRectangle(const Eigen::Vector2d& start,
                                                   const Eigen::Vector2d& end,
                                                   const Eigen::Vector2d& center,
                                                   double width, double height,
                                                   double rotation) {
    // Transform points to rectangle's local coordinate system
    Eigen::Rotation2Dd rot(-rotation);
    Eigen::Vector2d localStart = rot * (start - center);
    Eigen::Vector2d localEnd = rot * (end - center);
    
    // Calculate rectangle bounds
    double halfWidth = width / 2.0;
    double halfHeight = height / 2.0;
    
    // Use Liang-Barsky line clipping algorithm
    double t0 = 0.0;
    double t1 = 1.0;
    double dx = localEnd.x() - localStart.x();
    double dy = localEnd.y() - localStart.y();
    
    double p[4] = {-dx, dx, -dy, dy};
    double q[4] = {localStart.x() + halfWidth, halfWidth - localStart.x(),
                   localStart.y() + halfHeight, halfHeight - localStart.y()};
    
    for(int i = 0; i < 4; i++) {
        if(p[i] == 0) {
            if(q[i] < 0) return false;
        } else {
            double t = q[i] / p[i];
            if(p[i] < 0 && t > t0) t0 = t;
            else if(p[i] > 0 && t < t1) t1 = t;
        }
    }
    
    return t0 < t1 && t0 < 1.0 && t1 > 0.0;
}

bool GazeboObstacleChecker::pointIntersectsRectangle(const Eigen::Vector2d& point,
                                                    const Eigen::Vector2d& center,
                                                    double width, double height,
                                                    double rotation) {
    // Transform point to rectangle's local coordinate system
    Eigen::Rotation2Dd rot(-rotation);
    Eigen::Vector2d local_point = rot * (point - center);
    
    // Check bounds
    return (std::abs(local_point.x()) <= width/2 && 
           std::abs(local_point.y()) <= height/2);
}