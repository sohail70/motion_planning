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
    kf_model_type_ = params.getParam<std::string>("kf_model_type", "cv");
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
            const double radius = obstacle.dimensions.radius + inflated;
            if (lineIntersectsCircle(start2d, end2d, center, radius)) {
                return false;
            }
        } else {
            const double width = obstacle.dimensions.width + 2*inflated;
            const double height = obstacle.dimensions.height + 2*inflated;
            const double rotation = obstacle.dimensions.rotation;
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
            const double radius = obstacle.dimensions.radius + inflated;
            if (pointIntersectsCircle(point2d, center, radius)) {
                return false;
            }
        } else {
            const double width = obstacle.dimensions.width + 2*inflated;
            const double height = obstacle.dimensions.height + 2*inflated;
            const double rotation = obstacle.dimensions.rotation;
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


// This is the new private helper function that performs the core VO check.
bool GazeboObstacleChecker::isInVelocityObstacle(
    const Eigen::Vector2d& robot_velocity,
    const Eigen::Vector2d& obs_velocity,
    const Eigen::Vector2d& pos_robot_to_obs,
    double combined_radius
) const {
    // This function checks if the robot's velocity vector, relative to the obstacle,
    // falls within the "collision cone" projected from the robot's position.

    // 1. Calculate the velocity of the robot relative to the obstacle.
    const Eigen::Vector2d relative_velocity = robot_velocity - obs_velocity;

    // 2. If the obstacle is moving away from the robot, no future collision is possible.
    // A negative dot product means the vectors are pointing in generally opposite directions.
    if (pos_robot_to_obs.dot(relative_velocity) < 0) {
        // A more robust check: check if the distance is increasing.
        // If the squared distance is already increasing, they are moving apart.
        const double relative_speed_sq = relative_velocity.squaredNorm();
        if (relative_speed_sq > 1e-9) {
            // d/dt (dist^2) = 2 * (p_rel . v_rel). If positive, distance is increasing.
            if (pos_robot_to_obs.dot(relative_velocity) > 0) {
                return false;
            }
        }
    }

    // 3. Calculate the geometry of the collision cone.
    const double dist_sq = pos_robot_to_obs.squaredNorm();
    if (dist_sq < 1e-9) return true; // Already colliding

    const double combined_radius_sq = combined_radius * combined_radius;
    // If the robot is already inside the obstacle, it's a collision.
    if (dist_sq <= combined_radius_sq) {
        return true;
    }

    // 4. Perform the geometric check.
    // The angle of the collision cone (lambda in the papers).
    double apex_angle = std::asin(combined_radius / pos_robot_to_obs.norm());

    // The angle between the relative velocity and the line connecting the two agents.
    double vel_angle = std::acos(relative_velocity.dot(pos_robot_to_obs) / (relative_velocity.norm() * pos_robot_to_obs.norm()));

    // If the velocity vector's angle is within the cone's apex angle, it's a collision course.
    return std::abs(vel_angle) < apex_angle;
}



// Finds the time of closest approach (t_cpa) and minimum squared distance
// for an arc-line interaction. Returns true if a collision is found.
bool GazeboObstacleChecker::check_arc_line_collision(
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
) const // Assuming this is a const member function
{

    const double start_angle = std::atan2(p_r0_start.y() - center.y(), p_r0_start.x() - center.x());

    auto dist_sq_func = [&](double t) -> double {
        double angle = start_angle + angular_velocity * t;
        Eigen::Vector2d p_r_t = center + radius * Eigen::Vector2d(std::cos(angle), std::sin(angle));
        Eigen::Vector2d p_o_t = p_o0_start + v_o * t;
        return (p_r_t - p_o_t).squaredNorm();
    };

    auto derivative_func = [&](double t) -> double {
        double angle = start_angle + angular_velocity * t;
        Eigen::Vector2d p_r_t = center + radius * Eigen::Vector2d(std::cos(angle), std::sin(angle));
        Eigen::Vector2d v_r_t = radius * angular_velocity * Eigen::Vector2d(-std::sin(angle), std::cos(angle));
        Eigen::Vector2d p_o_t = p_o0_start + v_o * t;
        return 2.0 * (p_r_t - p_o_t).dot(v_r_t - v_o);
    };
    // --- 1. Boundary Checks ---
    if (dist_sq_func(0.0) <= R_sq) return true;
    if (dist_sq_func(T_segment) <= R_sq) return true;

    // --- 2. Robust Search for Interior Minima ---
    const int search_steps = 40;
    const double derivative_tolerance = 1e-5;
    const int bisection_iterations = 10;
    const double t_step = T_segment / search_steps;

    double last_t = 0.0;
    double last_deriv = derivative_func(last_t);

    for (int i = 1; i <= search_steps; ++i) {
        double current_t = i * t_step;
        double current_deriv = derivative_func(current_t);

        // Tweak: Directly check distance at each sample point's boundary
        if (dist_sq_func(current_t) <= R_sq) return true;

        bool potential_minimum_found = false;
        double bracket_low = -1.0, bracket_high = -1.0;

        if (std::abs(current_deriv) < derivative_tolerance) {
            bracket_low = std::max(0.0, current_t - t_step);
            bracket_high = std::min(T_segment, current_t + t_step);
            potential_minimum_found = true;
        }
        else if (std::copysign(1.0, current_deriv) != std::copysign(1.0, last_deriv)) {
            bracket_low = last_t;
            bracket_high = current_t;
            potential_minimum_found = true;
        }

        if (potential_minimum_found) {
            double t_low = bracket_low, t_high = bracket_high;
            double deriv_low = derivative_func(t_low);

            for (int j = 0; j < bisection_iterations; ++j) {
                double t_mid = (t_low + t_high) / 2.0;
                if (t_mid == t_low || t_mid == t_high) break;

                double deriv_mid = derivative_func(t_mid);
                if (std::copysign(1.0, deriv_mid) == std::copysign(1.0, deriv_low)) {
                    t_low = t_mid;
                    deriv_low = deriv_mid;
                } else {
                    t_high = t_mid;
                }
            }
            if (dist_sq_func((t_low + t_high) / 2.0) <= R_sq) return true;
        }
        
        last_t = current_t;
        last_deriv = current_deriv;
    }

    return false;

}








bool GazeboObstacleChecker::isTrajectorySafe(
    const Trajectory& trajectory,
    double global_edge_start_time
) const {
    // A nullopt from getCollidingObstacle means the path is safe.
    return !getCollidingObstacle(trajectory, global_edge_start_time).has_value();
}

//  With Constant Acc and Constant Vel implmented depending on what the obstalces movement and you KF is!
/*
    Using Eigen to calc exact time of collision
    Broad phase ignore too far away edges based on max velocity of obstalce --> you can comment it if you dont want hardocded stuff
    Reason for static collision check here for dynamic obstalce is to obsolete edges right on obstalce so that replanner doesnt use them
    so that robot wouldnt use them because obstalces can go backward suddenly and if robot is behind obstalce and they are gonna go in the same direction
    it would trick the planner otherwise. also the steer function is noot inteligent in between segment movement

*/
// std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacle(
//     const Trajectory& trajectory,
//     double global_edge_start_time
// ) const {

//     std::lock_guard<std::mutex> lock(snapshot_mutex_);

//     if (trajectory.path_points.size() < 2) {
//         return std::nullopt;
//     }

//     // --- Robot's path and velocity ---
//     const Eigen::Vector2d p_r0 = trajectory.path_points.front().head<2>();
//     const Eigen::Vector2d p_rT = trajectory.path_points.back().head<2>();
//     const double T = trajectory.time_duration;

//     Eigen::Vector2d v_r = Eigen::Vector2d::Zero();
//     if (T > 1e-9) {
//         v_r = (p_rT - p_r0) / T;
//     }

//     for (const auto& obs : obstacle_snapshot_) {
//         // --- 1. Calculate Collision Radii ---
//         double obs_radius = (obs.type == Obstacle::CIRCLE)
//                           ? obs.dimensions.radius
//                           : std::hypot(obs.dimensions.width/2.0, obs.dimensions.height/2.0);
//         double R = obs_radius + inflation;
//         double R_sq = R * R;

//         // --- 2. Broad-Phase Check ---
//         // Quickly discard obstacles that are geometrically too far away.
//         double min_dist_sq_to_path = distanceSqrdPointToSegment(obs.position, p_r0, p_rT);

//         if (obs.is_dynamic) {
//             // For dynamic obstacles, create a "danger zone" based on its max speed.
//             const double v_max_obs = 12.0; // Assume a max possible speed for any obstacle.
//             const double max_travel_dist = v_max_obs * T;
//             const double danger_radius = R + max_travel_dist;
            
//             if (min_dist_sq_to_path > (danger_radius * danger_radius)) {
//                 continue; // Obstacle is too far to reach the path in time.
//             }
//         } else { // This is the static obstacle check.
//             if (min_dist_sq_to_path > R_sq) {
//                 continue; // Static obstacle is not intersecting the path.
//             } else {
//                 // If a static obstacle *is* intersecting, it's a definite collision.
//                 return obs;
//             }
//         }
        
//         // --- 3. Predictive Check for Dynamic Obstacles ---
//         // This only runs for dynamic obstacles that passed the broad-phase check.
//         if (obs.is_dynamic && T > 1e-9) {
//             // Extrapolate obstacle state to the start of the trajectory edge (τ=0)
//             double t_snap = obs.last_update_time.seconds();
//             double delta_t = global_edge_start_time - t_snap;
            
//             Eigen::Vector2d p_o0 = obs.position + obs.velocity * delta_t + 0.5 * obs.acceleration * (delta_t * delta_t);
//             Eigen::Vector2d v_o0 = obs.velocity + obs.acceleration * delta_t;
            
//             Eigen::Vector2d p0 = p_r0 - p_o0; // Relative position at τ=0
//             Eigen::Vector2d v1 = v_r - v_o0;  // Relative velocity at τ=0

//             // --- Case 1: Constant Velocity Model (Zero Acceleration) ---
//             if (obs.acceleration.squaredNorm() < 1e-9) {
//                 double C2 = v1.dot(v1);
//                 double C1 = 2.0 * p0.dot(v1);
//                 double C0 = p0.dot(p0) - R_sq;

//                 if (std::abs(C2) < 1e-9) continue; // No relative velocity, no new collision.

//                 double discriminant = C1 * C1 - 4 * C2 * C0;
//                 if (discriminant < 0) continue; // Paths don't intersect.

//                 double sqrt_disc = std::sqrt(discriminant);
//                 // Check if the collision interval [t1, t2] overlaps with trajectory duration [0, T]
//                 double t1 = (-C1 - sqrt_disc) / (2 * C2); // Time of first contact
//                 double t2 = (-C1 + sqrt_disc) / (2 * C2); // Time of last contact

//                 if (std::max(0.0, t1) <= std::min(T, t2)) {
//                     return obs; // Collision occurs within the trajectory duration.
//                 }
//             } 
//             // --- Case 2: Constant Acceleration Model ---
//             else {
//                 Eigen::Vector2d v2 = -0.5 * obs.acceleration;

//                 double A4 = v2.dot(v2);
//                 double A3 = 2.0 * v1.dot(v2);
//                 double A2 = 2.0 * p0.dot(v2) + v1.dot(v1);
//                 double A1 = 2.0 * p0.dot(v1);
//                 double A0 = p0.dot(p0) - R_sq;

//                 Eigen::Matrix<double,5,1> coeffs;
//                 coeffs << A0, A1, A2, A3, A4;

//                 Eigen::PolynomialSolver<double,4> solver;
//                 solver.compute(coeffs);

//                 for (auto const& root : solver.roots()) {
//                     if (std::abs(root.imag()) > 1e-6) continue; // Ignore complex roots
//                     double tau = root.real();
//                     // Check if a real collision occurs within the trajectory duration
//                     if (tau >= 0 && tau <= T) {
//                         return obs;
//                     }
//                 }
//             }
//         }
//     }

//     return std::nullopt;
// }








// /*
//     Uses constant velocity model and no acceleration
//     Numerical instead of eigen 4th order polynomial
//     No broad phase for far away edges --> implement this but this needs to be implemented for dynamic obstalces carefully by considering their max velocity
//     No static Collision
//     My problem : what if the robot is behind the obstalce and they are moving in the same direction and the obstalce suddenly moves back
//                  or even worse --> the edge that the robot is traveling right now has constant velocity! it doesnt know about obstalce until
//                  we hit the back of the obstalce and then the replanner immeidately want to reroute but it might fail because the robot is in the tip end of the obstalce 
//                  and we get replanner fail!
//                  so a static immeidate check to obsolete the edge might be good --> This is only because my obstalces change direction backward at their end point!
// */
// std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacle(
//     const Trajectory& trajectory,
//     double global_edge_start_time
// ) const {
//     std::lock_guard<std::mutex> lock(snapshot_mutex_);

//     if (trajectory.path_points.size() < 2) {
//         return std::nullopt;
//     }

//     // --- Robot's initial state and velocity ---
//     const Eigen::Vector2d p_r0 = trajectory.path_points.front().head<2>();
//     const Eigen::Vector2d p_rT = trajectory.path_points.back().head<2>();
//     const double T = trajectory.time_duration;
    
//     // CORRECTED: Use if/else to avoid the Eigen expression template type error
//     Eigen::Vector2d v_r;
//     if (T > 1e-9) {
//         v_r = (p_rT - p_r0) / T;
//     } else {
//         v_r = Eigen::Vector2d::Zero();
//     }

//     // --- DISCRETE CHECKING PARAMETERS ---
//     // Check every 25cm of the robot's path, or at least 10 steps.
//     const int num_steps = std::max(10, static_cast<int>(v_r.norm() * T / 0.25));
//     const double time_step = T / num_steps;

//     for (const auto& obs_snapshot : obstacle_snapshot_) {
//         // --- Calculate the combined radius for collision checking ---
//         double obs_radius = (obs_snapshot.type == Obstacle::CIRCLE)
//                           ? obs_snapshot.dimensions.radius
//                           : std::hypot(obs_snapshot.dimensions..width / 2.0,
//                                        obs_snapshot.dimensions.height / 2.0);
//         double combined_radius = obs_radius + inflation;
//         double combined_radius_sq = combined_radius * combined_radius;

//         // --- Extrapolate the obstacle's state to the start of the trajectory (t=0) ---
//         double t_snap = obs_snapshot.last_update_time.seconds();
//         double delta_t_extrapolation = global_edge_start_time - t_snap;
        
//         // Use a simple, robust constant velocity extrapolation
//         Eigen::Vector2d p_o0 = obs_snapshot.position + obs_snapshot.velocity * delta_t_extrapolation;
//         Eigen::Vector2d v_o0 = obs_snapshot.velocity; 

//         // --- DISCRETE CHECKING LOOP ---
//         for (int i = 0; i <= num_steps; ++i) {
//             double current_tau = i * time_step;

//             // 1. Calculate robot's position at this time step
//             Eigen::Vector2d p_robot_at_tau = p_r0 + v_r * current_tau;

//             // 2. Calculate obstacle's position at this time step
//             Eigen::Vector2d p_obs_at_tau = p_o0 + v_o0 * current_tau;
            
//             // 3. Check for collision by comparing squared distance
//             if ((p_robot_at_tau - p_obs_at_tau).squaredNorm() <= combined_radius_sq) {
//                 // Collision detected! Return the obstacle.
//                 return obs_snapshot;
//             }
//         }
//     }

//     // No collision found after checking all obstacles and all time steps
//     return std::nullopt;
// }


/*
    No broad phase for far away edges --> implement this
    Uses constant velocity model and no acceleration
    Numerical instead of eigen by using number of steps instead of exact time of collision
    Has static collision check
*/

// std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacle(
//     const Trajectory& trajectory,
//     double global_edge_start_time
// ) const {

//     std::lock_guard<std::mutex> lock(snapshot_mutex_);

//     if (trajectory.path_points.size() < 2) {
//         return std::nullopt;
//     }

//     // --- Robot's initial state and velocity ---
//     const Eigen::Vector2d p_r0 = trajectory.path_points.front().head<2>();
//     const Eigen::Vector2d p_rT = trajectory.path_points.back().head<2>();
//     const double T = trajectory.time_duration;
    
//     Eigen::Vector2d v_r;
//     if (T > 1e-9) {
//         v_r = (p_rT - p_r0) / T;
//     } else {
//         v_r = Eigen::Vector2d::Zero();
//     }

//     // --- DISCRETE CHECKING PARAMETERS ---
//     const int num_steps = std::max(10, static_cast<int>(v_r.norm() * T / 0.25));
//     const double time_step = T / num_steps;

//     for (const auto& obs_snapshot : obstacle_snapshot_) {
//         // --- Calculate the combined radius for collision checking ---
//         double obs_radius = (obs_snapshot.type == Obstacle::CIRCLE)
//                           ? obs_snapshot.dimensions.radius
//                           : std::hypot(obs_snapshot.dimensions.width / 2.0,
//                                        obs_snapshot.dimensions.height / 2.0);
//         double combined_radius = obs_radius + inflation;
//         double combined_radius_sq = combined_radius * combined_radius;
        
//         // ==================================================================
//         // ✅ --- ADDED: Static Collision Check ---
//         // This is a fast, geometric check that ignores obstacle velocity.
//         // It provides an immediate layer of safety.
//         // ==================================================================
//         // We assume the helper function `distanceSqrdPointToSegment` exists in your class.
//         double min_dist_sq_to_path = distanceSqrdPointToSegment(obs_snapshot.position, p_r0, p_rT);
//         if (min_dist_sq_to_path <= combined_radius_sq) {
//             // The obstacle's current position is already intersecting the future path.
//             return obs_snapshot;
//         }

//         // --- Predictive Check (Only if Static Check Passes) ---
//         // If the obstacle is dynamic, proceed with the predictive check.
//         if (obs_snapshot.is_dynamic) {
//             // Extrapolate the obstacle's state to the start of the trajectory (t=0)
//             double t_snap = obs_snapshot.last_update_time.seconds();
//             double delta_t_extrapolation = global_edge_start_time - t_snap;
            
//             Eigen::Vector2d p_o0 = obs_snapshot.position + obs_snapshot.velocity * delta_t_extrapolation;
//             Eigen::Vector2d v_o0 = obs_snapshot.velocity; 

//             // --- DISCRETE CHECKING LOOP ---
//             for (int i = 0; i <= num_steps; ++i) {
//                 double current_tau = i * time_step;

//                 // 1. Calculate robot's position at this time step
//                 Eigen::Vector2d p_robot_at_tau = p_r0 + v_r * current_tau;

//                 // 2. Calculate obstacle's position at this time step
//                 Eigen::Vector2d p_obs_at_tau = p_o0 + v_o0 * current_tau;
                
//                 // 3. Check for collision by comparing squared distance
//                 if ((p_robot_at_tau - p_obs_at_tau).squaredNorm() <= combined_radius_sq) {
//                     // Collision detected! Return the obstacle.
//                     return obs_snapshot;
//                 }
//             }
//         }
//     }


//     // No collision found after checking all obstacles and all time steps
//     return std::nullopt;
// }



// // works good with R2T
// // Discretize form and checking all the waypoints inside : RTD has only two but dubin has many waypoints (which we assume they are lines since they are small)
// std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacle(
//     const Trajectory& trajectory,
//     double global_start_time
// ) const {
//     std::lock_guard<std::mutex> lock(snapshot_mutex_);

//     if (trajectory.path_points.size() < 2) {
//         return std::nullopt;
//     }

//     auto get_time = [](const Eigen::VectorXd& state) { return state(state.size() - 1); };
    
//     // This log is still useful to see when a check starts
//     // RCLCPP_INFO(rclcpp::get_logger("CollisionChecker"), "\n--- Checking Trajectory (Global Start: %.2f, Segments: %zu) ---", 
//         // global_start_time, trajectory.path_points.size() - 1);

//     for (size_t i = 0; i < trajectory.path_points.size() - 1; ++i) {
//         const Eigen::VectorXd& segment_start_state = trajectory.path_points[i];
//         const Eigen::VectorXd& segment_end_state   = trajectory.path_points[i+1];

//         const Eigen::Vector2d p_r0 = segment_start_state.head<2>();
//         const Eigen::Vector2d p_r1 = segment_end_state.head<2>();
//         const double T_segment = get_time(segment_start_state) - get_time(segment_end_state);

//         if (T_segment <= 1e-9) continue;
        
//         const Eigen::Vector2d v_r = (p_r1 - p_r0) / T_segment;
//         const int num_steps = std::max(2, static_cast<int>(v_r.norm() * T_segment / 0.1));
//         const double time_step = T_segment / num_steps;

//         double time_into_full_trajectory = trajectory.time_duration - get_time(segment_start_state);
//         double global_time_at_segment_start = global_start_time + time_into_full_trajectory;

//         for (const auto& obs_snapshot : obstacle_snapshot_) {
//             double obs_radius = (obs_snapshot.type == Obstacle::CIRCLE)
//                               ? obs_snapshot.dimensions.radius
//                               : std::hypot(obs_snapshot.dimensions.width / 2.0, obs_snapshot.dimensions.height / 2.0);
//             double combined_radius = obs_radius + inflation;
//             double combined_radius_sq = combined_radius * combined_radius;
            
//             if (distanceSqrdPointToSegment(obs_snapshot.position, p_r0, p_r1) <= combined_radius_sq) {
//                 // RCLCPP_WARN(rclcpp::get_logger("CollisionChecker"), "      [!!! STATIC COLLISION !!!] Obstacle at (%.2f, %.2f) is intersecting the path segment.", obs_snapshot.position.x(), obs_snapshot.position.y());
//                 return obs_snapshot;
//             }

//             if (obs_snapshot.is_dynamic) {
//                 // [!!! THE CRITICAL DIAGNOSTIC LOG !!!]
//                 // This prints the raw data the checker is about to use for its prediction.
//                 // RCLCPP_INFO(rclcpp::get_logger("CollisionChecker"), "  [Checking Obstacle] Pos: (%.2f, %.2f) | Vel: (%.2f, %.2f)",
//                     // obs_snapshot.position.x(), obs_snapshot.position.y(),
//                     // obs_snapshot.velocity.x(), obs_snapshot.velocity.y());


//                 double delta_t_extrapolation = global_time_at_segment_start - obs_snapshot.last_update_time.seconds();
//                 delta_t_extrapolation = std::max(0.0, delta_t_extrapolation);

//                 Eigen::Vector2d p_o0 = obs_snapshot.position + obs_snapshot.velocity * delta_t_extrapolation;
//                 const Eigen::Vector2d& v_o0 = obs_snapshot.velocity; 

//                 for (int j = 0; j <= num_steps; ++j) {
//                     double current_tau = j * time_step;
//                     Eigen::Vector2d p_robot_at_tau = p_r0 + v_r * current_tau;
//                     Eigen::Vector2d p_obs_at_tau = p_o0 + v_o0 * current_tau;
                    
//                     double dist_sq = (p_robot_at_tau - p_obs_at_tau).squaredNorm();

//                     if (dist_sq <= combined_radius_sq) {
//                         // RCLCPP_WARN(rclcpp::get_logger("CollisionChecker"), "      [!!! PREDICTIVE COLLISION !!!] at tau=%.2fs into segment.", current_tau);
//                         // RCLCPP_WARN(rclcpp::get_logger("CollisionChecker"), "          - Robot Predicted Pos: (%.2f, %.2f)", p_robot_at_tau.x(), p_robot_at_tau.y());
//                         // RCLCPP_WARN(rclcpp::get_logger("CollisionChecker"), "          - Obstacle Predicted Pos: (%.2f, %.2f)", p_obs_at_tau.x(), p_obs_at_tau.y());
//                         // RCLCPP_WARN(rclcpp::get_logger("CollisionChecker"), "          - Dist^2: %.2f <= Required R^2: %.2f", dist_sq, combined_radius_sq);
//                         return obs_snapshot;
//                     }
//                 }
//             }
//         }
//     }

//     return std::nullopt;
// }




// // ANALYTICAL!


// std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacle(
//     const Trajectory& trajectory,
//     double global_start_time
// ) const {
//     std::lock_guard<std::mutex> lock(snapshot_mutex_);

//     if (trajectory.analytical_segments.empty()) {
//         return std::nullopt;
//     }

//     double cumulative_time = 0.0;

//     // Main loop iterates over the pre-computed analytical segments
//     for (const auto& segment : trajectory.analytical_segments) {
//         const double T_segment = segment.duration;
//         if (T_segment <= 1e-9) continue;

//         const double global_time_at_segment_start = global_start_time + cumulative_time;

//         for (const auto& obs : obstacle_snapshot_) {
//             // --- 1. Define Collision Radii ---
//             const double obs_radius = (obs.type == Obstacle::CIRCLE)
//                                     ? obs.dimensions.radius
//                                     : std::hypot(obs.dimensions.width / 2.0, obs.dimensions.height / 2.0);
//             const double R = obs_radius + inflation;
//             const double R_sq = R * R;
//             const double v_max_obs = 12.0; // A reasonable upper bound on any obstacle's speed

//             // --- 2. Broad-Phase Check ---
//             // Quickly discard obstacles that are geometrically too far away to be a threat.
//             double min_dist_sq_to_path_geom = 0.0;
//             if (segment.type == SegmentType::LINE) {
//                 min_dist_sq_to_path_geom = distanceSqrdPointToSegment(obs.position, segment.start_point, segment.end_point);
//             } else { // ARC
//                 min_dist_sq_to_path_geom = distanceSqrdPointToArc(obs.position, segment.start_point, segment.end_point, segment.center, segment.radius, segment.is_clockwise);
//             }

//             // The maximum distance the obstacle could travel during this segment's duration
//             const double max_obs_travel_dist = v_max_obs * T_segment;
//             // The "danger zone" is the collision radius plus this travel distance
//             const double danger_radius = R + max_obs_travel_dist;

//             if (min_dist_sq_to_path_geom > danger_radius * danger_radius) {
//                 continue; // Obstacle is too far away to reach the path in time.
//             }

//             // --- 3. Static Collision Check (Narrow-Phase part 1) ---
//             // If the obstacle is static, the broad-phase check is sufficient.
//             if (!obs.is_dynamic) {
//                 if (min_dist_sq_to_path_geom <= R_sq) {
//                     return obs; // Definite collision with a static obstacle.
//                 }
//                 continue; // Static obstacle is safe, move to the next obstacle.
//             }

//             // --- 4. Dynamic Predictive Check (Narrow-Phase part 2) ---
//             // This only runs for dynamic obstacles that passed the broad-phase check.
//             const double delta_t_extrapolation = std::max(0.0, global_time_at_segment_start - obs.last_update_time.seconds());
//             const Eigen::Vector2d p_o0 = obs.position + obs.velocity * delta_t_extrapolation;
//             const Eigen::Vector2d& v_o = obs.velocity;

//             // --- SWITCH BASED ON SEGMENT TYPE ---
//             if (segment.type == SegmentType::LINE) {
//                 const Eigen::Vector2d& p_r0 = segment.start_point;
//                 const Eigen::Vector2d v_r = (segment.end_point - p_r0) / T_segment;
//                 const Eigen::Vector2d p0_relative = p_r0 - p_o0;
//                 const Eigen::Vector2d v_relative = v_r - v_o;

//                 const double a = v_relative.dot(v_relative);
//                 const double b = 2.0 * p0_relative.dot(v_relative);
//                 const double c = p0_relative.dot(p0_relative) - R_sq;

//                 if (std::abs(a) < 1e-9) continue;

//                 const double discriminant = b * b - 4 * a * c;
//                 if (discriminant < 0) continue;

//                 const double sqrt_disc = std::sqrt(discriminant);
//                 const double t1 = (-b - sqrt_disc) / (2 * a);
//                 const double t2 = (-b + sqrt_disc) / (2 * a);

//                 if (std::max(0.0, t1) <= std::min(T_segment, t2)) {
//                     return obs; // Collision occurs within the segment's duration.
//                 }

//             } else { // SegmentType::ARC
//                 const double speed = (segment.end_point - segment.start_point).norm() / T_segment;
//                 const double angular_velocity = (segment.is_clockwise ? -1.0 : 1.0) * speed / segment.radius;
                
//                 if (check_arc_line_collision(
//                     segment.start_point, segment.center, segment.radius, angular_velocity,
//                     p_o0, v_o, T_segment, R_sq))
//                 {
//                     return obs;
//                 }
//             }
//         }
//         // Update cumulative time for the next segment
//         cumulative_time += T_segment;
//     }

//     return std::nullopt; // Trajectory is clear
// }



// // // combined by all the above best features! --> works good with R2T and DubinTimeStateSpace!--> fully discrete and no analytical root finding --> this is more robust than root finding in my sim
// std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacle(
//     const Trajectory& trajectory,
//     double global_start_time
// ) const {
//     std::lock_guard<std::mutex> lock(snapshot_mutex_);

//     if (trajectory.path_points.size() < 2) {
//         return std::nullopt;
//     }

//     auto get_xy = [](const Eigen::VectorXd& state) { return state.head<2>(); };
//     auto get_time = [](const Eigen::VectorXd& state) { return state(state.size() - 1); };

//     double time_into_full_trajectory = 0.0;

//     for (size_t i = 0; i < trajectory.path_points.size() - 1; ++i) {
//         const Eigen::VectorXd& segment_start_state = trajectory.path_points[i];
//         const Eigen::VectorXd& segment_end_state   = trajectory.path_points[i + 1];

//         const Eigen::Vector2d p_r0 = get_xy(segment_start_state);
//         const Eigen::Vector2d p_r1 = get_xy(segment_end_state);
//         const double T_segment = get_time(segment_start_state) - get_time(segment_end_state);

//         if (T_segment <= 1e-9) continue;
        
//         const double global_time_at_segment_start = global_start_time + time_into_full_trajectory;
//         const Eigen::Vector2d v_r = (p_r1 - p_r0) / T_segment;

//         for (const auto& obs : obstacle_snapshot_) {
//             const double obs_radius = (obs.type == Obstacle::CIRCLE)
//                                     ? obs.dimensions.radius
//                                     : std::hypot(obs.dimensions.width / 2.0, obs.dimensions.height / 2.0);
//             const double R = obs_radius + inflation;
//             const double R_sq = R * R;

//             if (obs.is_dynamic) {
//                 // --- Narrow-Phase Check for DYNAMIC Obstacles ---
//                 const double delta_t_extrapolation = std::max(0.0, global_time_at_segment_start - obs.last_update_time.seconds());
//                 const Eigen::Vector2d p_o0 = obs.position + obs.velocity * delta_t_extrapolation;
                
//                 const Eigen::Vector2d p_relative_start = p_r0 - p_o0;
//                 const Eigen::Vector2d v_relative = v_r - obs.velocity;
                
//                 const double a = v_relative.dot(v_relative);
//                 const double b = 2.0 * p_relative_start.dot(v_relative);
                
//                 // ✅ CORRECTED LINE:
//                 const double c = p_relative_start.dot(p_relative_start) - R_sq;

//                 if (std::abs(a) < 1e-9) { // Zero relative velocity
//                     if (c <= 0) return obs; // Collision if initially overlapping
//                     else continue;
//                 }

//                 const double discriminant = b * b - 4 * a * c;
//                 if (discriminant >= 0) {
//                     const double sqrt_disc = std::sqrt(discriminant);
//                     const double t1 = (-b - sqrt_disc) / (2.0 * a);
//                     const double t2 = (-b + sqrt_disc) / (2.0 * a);
//                     // Check if collision interval overlaps with the segment's duration
//                     if (std::max(0.0, t1) <= std::min(T_segment, t2)) {
//                         return obs; // Collision found
//                     }
//                 }
//             } else {
//                 // --- Geometric Check for STATIC Obstacles ---
//                 if (distanceSqrdPointToSegment(obs.position, p_r0, p_r1) <= R_sq) {
//                     return obs; // Static collision found
//                 }
//                 // if(lineIntersectsCircle(p_r0, p_r1, obs.position, R))
//                 // {
//                 //     return obs;
//                 // }
//             }
//         }
        
//         time_into_full_trajectory += T_segment;
//     }

//     return std::nullopt; // The trajectory is safe.
// }





std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacle(
    const Trajectory& trajectory,
    double global_start_time
) const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);

    // 1. Initial validation of the trajectory
    if (trajectory.path_points.size() < 2) {
        return std::nullopt;
    }

    // 2. Define helper lambdas to easily access parts of the state vector
    auto get_xy = [](const Eigen::VectorXd& state) { return state.head<2>(); };
    auto get_time = [](const Eigen::VectorXd& state) { return state(state.size() - 1); };
    auto get_vxy = [](const Eigen::VectorXd& state) { return state.segment<2>(2); }; // For 5D states

    double time_into_full_trajectory = 0.0;
    const int state_dim = trajectory.path_points[0].size();

    // 3. Iterate over each segment of the trajectory
    for (size_t i = 0; i < trajectory.path_points.size() - 1; ++i) {
        const Eigen::VectorXd& segment_start_state = trajectory.path_points[i];
        const Eigen::VectorXd& segment_end_state   = trajectory.path_points[i + 1];

        const double T_segment = get_time(segment_start_state) - get_time(segment_end_state);
        if (T_segment <= 1e-9) continue;
        
        const double global_time_at_segment_start = global_start_time + time_into_full_trajectory;

        // 4. Conditionally apply the correct physics model based on state dimension
        if (state_dim == 5) {
            // =======================================================
            // == ACCELERATION MODEL (5D): Subdivide the curved path ==
            // =======================================================
            const int num_subdivisions = 4; // A tunable parameter for accuracy vs. performance
            
            // Calculate the constant acceleration for the entire segment
            const Eigen::Vector2d p_r0_seg = get_xy(segment_start_state);
            const Eigen::Vector2d v_r0_seg = get_vxy(segment_start_state);
            const Eigen::Vector2d v_r1_seg = get_vxy(segment_end_state);
            const Eigen::Vector2d a_r_seg = (v_r1_seg - v_r0_seg) / T_segment;
            
            Eigen::Vector2d p_sub_start = p_r0_seg;
            double t_sub_start = 0.0;

            for (int j = 1; j <= num_subdivisions; ++j) {
                // Calculate the state at the end of the current sub-segment using constant acceleration physics
                double t_sub_end = (static_cast<double>(j) / num_subdivisions) * T_segment;
                Eigen::Vector2d p_sub_end = p_r0_seg + v_r0_seg * t_sub_end + 0.5 * a_r_seg * t_sub_end * t_sub_end;
                
                // Now, perform the linear collision check on this small, accurate sub-segment
                const double T_sub_segment = t_sub_end - t_sub_start;
                const Eigen::Vector2d v_r_sub = (p_sub_end - p_sub_start) / T_sub_segment;

                for (const auto& obs : obstacle_snapshot_) {
                    const double obs_radius = (obs.type == Obstacle::CIRCLE)
                                            ? obs.dimensions.radius
                                            : std::hypot(obs.dimensions.width / 2.0, obs.dimensions.height / 2.0);
                    const double R = obs_radius + inflation;
                    const double R_sq = R * R;

                    if (obs.is_dynamic) {
                        // Dynamic obstacle check for the sub-segment
                        const double global_time_at_sub_segment_start = global_time_at_segment_start + t_sub_start;
                        const double delta_t_extrapolation = std::max(0.0, global_time_at_sub_segment_start - obs.last_update_time.seconds());
                        const Eigen::Vector2d p_o0 = obs.position + obs.velocity * delta_t_extrapolation;
                        
                        const Eigen::Vector2d p_relative_start = p_sub_start - p_o0;
                        const Eigen::Vector2d v_relative = v_r_sub - obs.velocity;
                        
                        const double a = v_relative.dot(v_relative);
                        const double b = 2.0 * p_relative_start.dot(v_relative);
                        const double c = p_relative_start.dot(p_relative_start) - R_sq;

                        if (std::abs(a) < 1e-9) { 
                            if (c <= 0) return obs; // Initially overlapping
                            continue;
                        }

                        const double discriminant = b * b - 4 * a * c;
                        if (discriminant >= 0) {
                            const double sqrt_disc = std::sqrt(discriminant);
                            const double t1 = (-b - sqrt_disc) / (2.0 * a);
                            const double t2 = (-b + sqrt_disc) / (2.0 * a);
                            if (std::max(0.0, t1) <= std::min(T_sub_segment, t2)) {
                                return obs; // Collision found
                            }
                        }
                    } else { 
                        // Static obstacle check for the sub-segment
                        if (distanceSqrdPointToSegment(obs.position, p_sub_start, p_sub_end) <= R_sq) {
                            return obs;
                        }
                    }
                }
                // Prepare for the next sub-segment
                p_sub_start = p_sub_end;
                t_sub_start = t_sub_end;
            }
        } else {
            // =========================================================
            // == CONSTANT VELOCITY MODEL (First-Order / Other Systems) ==
            // =========================================================
            const Eigen::Vector2d p_r0 = get_xy(segment_start_state);
            const Eigen::Vector2d p_r1 = get_xy(segment_end_state);
            const Eigen::Vector2d v_r = (p_r1 - p_r0) / T_segment;

            for (const auto& obs : obstacle_snapshot_) {
                const double obs_radius = (obs.type == Obstacle::CIRCLE)
                                        ? obs.dimensions.radius
                                        : std::hypot(obs.dimensions.width / 2.0, obs.dimensions.height / 2.0);
                const double R = obs_radius + inflation;
                const double R_sq = R * R;

                if (obs.is_dynamic) {
                    // Dynamic obstacle check for the whole linear segment
                    const double delta_t_extrapolation = std::max(0.0, global_time_at_segment_start - obs.last_update_time.seconds());
                    const Eigen::Vector2d p_o0 = obs.position + obs.velocity * delta_t_extrapolation;
                    
                    const Eigen::Vector2d p_relative_start = p_r0 - p_o0;
                    const Eigen::Vector2d v_relative = v_r - obs.velocity;
                    
                    const double a = v_relative.dot(v_relative);
                    const double b = 2.0 * p_relative_start.dot(v_relative);
                    const double c = p_relative_start.dot(p_relative_start) - R_sq;

                    if (std::abs(a) < 1e-9) { 
                        if (c <= 0) return obs;
                        continue;
                    }

                    const double discriminant = b * b - 4 * a * c;
                    if (discriminant >= 0) {
                        const double sqrt_disc = std::sqrt(discriminant);
                        const double t1 = (-b - sqrt_disc) / (2.0 * a);
                        const double t2 = (-b + sqrt_disc) / (2.0 * a);
                        if (std::max(0.0, t1) <= std::min(T_segment, t2)) {
                            return obs;
                        }
                    }
                } else {
                    // Static obstacle check for the whole linear segment
                    if (distanceSqrdPointToSegment(obs.position, p_r0, p_r1) <= R_sq) {
                        return obs;
                    }
                }
            }
        }
        
        // Update the time offset for the next segment
        time_into_full_trajectory += T_segment;
    }

    // 5. If the loop completes, no collisions were found
    return std::nullopt;
}

// // This is the primary function, now rewritten to use the Velocity Obstacle method.
// // This is the new, corrected version of your collision checking function.
// // It implements the segment-by-segment Velocity Obstacle check you proposed.
// std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacle(
//     const Trajectory& trajectory,
//     double global_start_time
// ) const {
//     std::lock_guard<std::mutex> lock(snapshot_mutex_);

//     // A trajectory needs at least two points to form a segment.
//     if (trajectory.path_points.size() < 2) {
//         return std::nullopt;
//     }

//     // Lambdas to easily access components of a state vector.
//     auto get_xy = [](const Eigen::VectorXd& state) { return state.head<2>(); };
//     auto get_time = [](const Eigen::VectorXd& state) { return state(state.size() - 1); };

//     // The robot's radius, used for calculating the combined radius for checks.
//     const double robot_radius = inflation;
    
//     // This will track the time elapsed from the start of the whole trajectory.
//     double cumulative_time = 0.0;

//     // --- Main Loop: Iterate over each segment of the trajectory ---
//     for (size_t i = 0; i < trajectory.path_points.size() - 1; ++i) {
//         const Eigen::VectorXd& qi = trajectory.path_points[i];
//         const Eigen::VectorXd& qj = trajectory.path_points[i + 1];

//         // The duration of this specific segment.
//         const double dt_seg = get_time(qi) - get_time(qj); // Assumes backward time (time-to-go)
//         if (dt_seg <= 1e-9) {
//             // Skip zero-duration "wait" segments, but account for their time.
//             cumulative_time += dt_seg;
//             continue;
//         }

//         // The absolute world time when this segment is predicted to start.
//         const double global_time_at_segment_start = global_start_time + cumulative_time;

//         // --- Check this segment against every obstacle ---
//         for (const auto& obs : obstacle_snapshot_) {
            
//             // Calculate the combined collision radius for this robot-obstacle pair.
//             double obs_radius = (obs.type == Obstacle::CIRCLE)
//                               ? obs.dimensions.radius
//                               : std::hypot(obs.dimensions.width / 2.0, obs.dimensions.height / 2.0);
//             double combined_radius = obs_radius + robot_radius;

//             // --- 1. Static Obstacle Check ---
//             // A simple, fast geometric check for non-moving obstacles.
//             if (!obs.is_dynamic) {
//                 if (distanceSqrdPointToSegment(obs.position, get_xy(qi), get_xy(qj)) <= combined_radius * combined_radius) {
//                     return obs; // Collision with a static obstacle found.
//                 }
//                 continue; // Safe from this static obstacle, check the next one.
//             }

//             // --- 2. Dynamic Obstacle (VO) Check ---
//             // Predict the obstacle's position at the start of THIS segment.
//             double delta_t_extrapolation = global_time_at_segment_start - obs.last_update_time.seconds();
//             Eigen::Vector2d p_o_predicted_start = obs.position + obs.velocity * std::max(0.0, delta_t_extrapolation);
            
//             // Calculate the robot's constant velocity for this segment.
//             const Eigen::Vector2d v_r = (get_xy(qj) - get_xy(qi)) / dt_seg;

//             // The relative position vector from the robot's start to the obstacle's predicted start.
//             const Eigen::Vector2d p_rel = p_o_predicted_start - get_xy(qi);
            
//             // Perform the VO check.
//             if (isInVelocityObstacle(v_r, obs.velocity, p_rel, combined_radius)) {
//                 return obs; // A predictive collision is found.
//             }
//         }

//         // Update the cumulative time for the start of the next segment.
//         cumulative_time += dt_seg;
//     }

//     // If all segments were checked against all obstacles and no collisions were found...
//     return std::nullopt; // ...the trajectory is safe.
// }




Eigen::Vector2d GazeboObstacleChecker::getRobotPosition() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return robot_position_;
}

Eigen::VectorXd GazeboObstacleChecker::getRobotOrientation() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return robot_orientation_;
}

const ObstacleVector& GazeboObstacleChecker::getObstaclePositions() const {
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
//     ObstacleVector current_dynamic_obstacles;
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
    ObstacleVector current_dynamic_obstacles;

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
                    
                    // --- ✅ 1. Use the Factory to create the filter ---
                    KalmanFilter new_filter = KalmanFilterFactory::createFilter(kf_model_type_);

                    // --- ✅ 2. Initialize the state with the correct size ---
                    int state_size = (kf_model_type_ == "cv") ? 4 : 6;
                    Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(state_size);
                    initial_state.head<2>() << obstacle.position.x(), obstacle.position.y();
                    new_filter.init(initial_state);
                    
                    filter_it = obstacle_filters_.emplace(name, new_filter).first;
                    
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

                    if (kf_model_type_ == "cv") {
                        obstacle.acceleration.setZero();
                    } else { // For "ca" and "singer"
                        obstacle.acceleration << estimated_state(4), estimated_state(5);
                    }

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




ObstacleVector GazeboObstacleChecker::getObstacles() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ObstacleVector filtered_obstacles;
    
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