// Copyright 2025 Soheil E.nia

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
    estimation = params.getParam<bool>("estimation", false);
    kf_model_type_ = params.getParam<std::string>("kf_model_type", "cv");
    use_fcl = params.getParam<bool>("fcl", false);
    use_bullet = params.getParam<bool>("bullet", false);
    spatial_dim_ = params.getParam<int>("spatial_dim", 2); // Defaults to 2 for backward compatibility


    initial_budget_time_ = params.getParam<double>("initial_budget_time");


    // =========================================================================
    // CRITICAL FIX: Initialize obstacle_positions_map_ from obstacle_info_
    // =========================================================================
    for (const auto& [name, info] : obstacle_info_) {
        Obstacle ob;
        ob.name = name;
        ob.is_dynamic = info.is_dynamic;
        ob.has_ground_truth = true; // We have ground truth from SDF
        
        // Set Geometry
        if (info.type == ObstacleInfo::CYLINDER) {
            ob.type = Obstacle::CIRCLE;
            ob.dimensions.radius = info.radius;
        } else {
            ob.type = Obstacle::BOX;
            ob.dimensions.width = info.width;
            ob.dimensions.height = info.height;
            ob.dimensions.rotation = 0.0; // Yaw from SDF if needed
        }

        // Set Motion Parameters (Crucial for deterministic math)
        ob.speed_scalar = info.speed;
        ob.motion_limit = info.amplitude;
        ob.initial_origin = info.initial_pose.head<2>();
        
        if (info.direction.head<2>().norm() > 1e-6) {
             ob.motion_axis = info.direction.head<2>().normalized();
        } else {
             ob.motion_axis = Eigen::Vector2d::UnitX();
        }

        // Initialize position to the start point (will be updated in processLatestPoseInfo)
        ob.position = ob.initial_origin;
        ob.velocity = Eigen::Vector2d::Zero();

        // Insert into the map so processLatestPoseInfo finds valid data
        obstacle_positions_map_[name] = ob;
    }
    // =========================================================================


    // Subscribe to the robot pose topic
    std::string robot_pose_topic = "/model/" + robot_model_name_ + "/tf";
    if (!gz_node_.Subscribe(robot_pose_topic, &GazeboObstacleChecker::robotPoseCallback, this)) {
            std::cerr << "Failed to subscribe to robot pose topic: " << robot_pose_topic << std::endl;
        } else {
            std::cout << "Successfully subscribed to robot pose topic: " << robot_pose_topic << std::endl;
    }
    

    footprint_type_ = params.getParam<std::string>("collision_check_footprint", "circular");
    RCLCPP_INFO(rclcpp::get_logger("ObstacleChecker"), "Using '%s' footprint for collision detection.", footprint_type_.c_str());

    if (footprint_type_ == "rectangular") {
        if (params.hasParam("rectangular_footprint_points")) {
            std::vector<double> points = params.getParam<std::vector<double>>("rectangular_footprint_points");
            if (points.size() % 2 != 0) {
                throw std::runtime_error("Rectangular footprint points must be in pairs (x1 y1 x2 y2 ...).");
            }
            // Convert the flat vector of doubles into a vector of 2D points
            for (size_t i = 0; i < points.size(); i += 2) {
                rectangular_footprint_.emplace_back(points[i], points[i+1]);
            }
        } else {
            throw std::runtime_error("Footprint type is 'rectangular' but 'rectangular_footprint_points' parameter was not provided.");
        }
    } else { // "circular" is the default
        // robot_radius_ = params.getParam<double>("inflation", 0.5);
        robot_radius_ = 0.0;
    }




    std::string topic = "/world/" + world_name_ + "/pose/info";
    // if (!gz_node_.Subscribe(topic, &GazeboObstacleChecker::poseInfoCallback, this)) {
    //     std::cerr << "Failed to subscribe to Gazebo topic: " << topic << std::endl;
    // }
    if (!gz_node_.Subscribe(topic, &GazeboObstacleChecker::lightweightPoseCallback, this)) {
        std::cerr << "Failed to subscribe to Gazebo topic: " << topic << std::endl;
    }




    path_pub_ = gz_node_.Advertise<gz::msgs::Pose_V>("/path");
    if (!path_pub_) {
        std::cerr << "Failed to advertise /path topic." << std::endl;
    } else {
        std::cout << "Successfully advertised /path topic." << std::endl;
    }


    // =========================================================================
    // NEW: Subscribe to Turnaround Signals
    // =========================================================================
    for (const auto& [name, info] : obstacle_info_) {
        if (info.is_dynamic) {
            std::string topic = "/model/" + name + "/turnaround";
            
            // Define the callback type explicitly
            using CallbackType = std::function<void(const gz::msgs::Boolean&)>;
            
            // Create the lambda
            CallbackType callback = [this, name](const gz::msgs::Boolean &_msg) {
                // When Gazebo sends a message, this code runs.
                // We set the flag for this specific obstacle to TRUE.
                this->obstacle_turnaround_flags_[name] = true;
            };

            // Subscribe using the std::function
            if (!gz_node_.Subscribe(topic, callback)) {
                std::cerr << "Failed to subscribe to turnaround topic: " << topic << std::endl;
            } else {
                // Initialize the flag to false
                obstacle_turnaround_flags_[name] = false;
            }
        }
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
    // std::lock_guard<std::mutex> lock(snapshot_mutex_);
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

bool GazeboObstacleChecker::isObstacleFreeAgainstSingleObstacle(const Eigen::VectorXd& start, const Eigen::VectorXd& end, const Obstacle& obs) const {
    const Eigen::Vector2d start2d = start.head<2>();
    const Eigen::Vector2d end2d = end.head<2>();
    const double inflated = obs.inflation;
    const Eigen::Vector2d& center = obs.position;
    
    if (obs.type == Obstacle::CIRCLE) {
        const double radius = obs.dimensions.radius + inflated;
        if (lineIntersectsCircle(start2d, end2d, center, radius)) {
            return false; // Collision
        }
    } else { // BOX
        const double width = obs.dimensions.width + 2 * inflated;
        const double height = obs.dimensions.height + 2 * inflated;
        const double rotation = obs.dimensions.rotation;
        if (lineIntersectsRectangle(start2d, end2d, center, width, height, rotation)) {
            return false; // Collision
        }
    }
    
    return true; // No collision with this specific obstacle
}

bool GazeboObstacleChecker::isObstacleFreeAgainstSingleObstacle(const Eigen::VectorXd& point, const Obstacle& obs) const {
    const Eigen::Vector2d point2d = point.head<2>();
    const double inflated = obs.inflation;
    const Eigen::Vector2d& center = obs.position;

    if (obs.type == Obstacle::CIRCLE) {
        const double radius = obs.dimensions.radius + inflated;
        // Re-using the existing point-circle intersection helper
        if (pointIntersectsCircle(point2d, center, radius)) {
            return false; // Collision
        }
    } else { // BOX
        const double width = obs.dimensions.width + 2 * inflated;
        const double height = obs.dimensions.height + 2 * inflated;
        const double rotation = obs.dimensions.rotation;
        // Re-using the existing point-rectangle intersection helper
        if (pointIntersectsRectangle(point2d, center, width, height, rotation)) {
            return false; // Collision
        }
    }

    return true; // No collision with this specific obstacle
}

bool GazeboObstacleChecker::isObstacleFree(const Eigen::VectorXd& point) const {
    // std::lock_guard<std::mutex> lock(snapshot_mutex_);
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

    //  Calculate the velocity of the robot relative to the obstacle.
    const Eigen::Vector2d relative_velocity = robot_velocity - obs_velocity;

    // If the obstacle is moving away from the robot, no future collision is possible.
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

    // Calculate the geometry of the collision cone.
    const double dist_sq = pos_robot_to_obs.squaredNorm();
    if (dist_sq < 1e-9) return true; // Already colliding

    const double combined_radius_sq = combined_radius * combined_radius;
    // If the robot is already inside the obstacle, it's a collision.
    if (dist_sq <= combined_radius_sq) {
        return true;
    }

    // Perform the geometric check.
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
    // --- Boundary Checks ---
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



// // GOOD with prediction horizon
// bool GazeboObstacleChecker::isTrajectorySafeAgainstSingleObstacle(
//     const Trajectory& trajectory,
//     double global_start_time,
//     const Obstacle& obs
// ) const {
//     if (trajectory.path_points.size() < 2) return true;


//     // --- BROAD-PHASE ENVELOPE CHECK ---
//     // If the obstacle center is nowhere near the trajectory spatial area, skip it.
//     // This is the "Filter First" logic from the Julia code.
//     Eigen::Vector2d traj_start = trajectory.path_points.front().head<2>();
//     Eigen::Vector2d traj_end = trajectory.path_points.back().head<2>();
//     Eigen::Vector2d mid_point = (traj_start + traj_end) / 2.0;
//     double half_length = (traj_end - traj_start).norm() / 2.0;
    
//     // Quick distance from obstacle to the entire trajectory "envelope"
//     double dist_to_obs = (obs.position - mid_point).norm();
//     if (dist_to_obs > (half_length + obs.dimensions.radius + inflation + 5.0)) {
//         return true; // Too far away spatially, skip all segments!
//     }
//     // ---------------------------------------


//     const int time_dim_idx = trajectory.path_points[0].size() - 1;
//     double time_into_full_trajectory = 0.0;

//     for (size_t i = 0; i < trajectory.path_points.size() - 1; ++i) {
//         const Eigen::VectorXd& segment_start_state = trajectory.path_points[i];
//         const Eigen::VectorXd& segment_end_state   = trajectory.path_points[i + 1];
//         const double T_segment = segment_start_state(time_dim_idx) - segment_end_state(time_dim_idx);
        
//         if (T_segment <= 1e-9) continue;

//         const double time_at_segment_start = global_start_time + time_into_full_trajectory;
//         const Eigen::Vector2d p_r0 = segment_start_state.head<2>();
//         const Eigen::Vector2d p_r1 = segment_end_state.head<2>();
//         const Eigen::Vector2d v_r = (p_r1 - p_r0) / T_segment;

//         if (obs.type == Obstacle::CIRCLE) {
//             const double R = obs.dimensions.radius + inflation;
//             const double R_sq = R * R;

//             if (obs.is_dynamic) {
//                 const double delta_t = std::max(0.0, time_at_segment_start - obs.last_update_time.seconds());
//                 const Eigen::Vector2d p_o0 = obs.position + obs.velocity * delta_t;
//                 const Eigen::Vector2d p_rel_start = p_r0 - p_o0;
//                 const Eigen::Vector2d v_rel = v_r - obs.velocity;
                
//                 const double a = v_rel.dot(v_rel);
//                 const double b = 2.0 * p_rel_start.dot(v_rel);
//                 const double c = p_rel_start.dot(p_rel_start) - R_sq;

//                 if (std::abs(a) < 1e-9) { 
//                     if (c <= 0) return false;
//                 } else {
//                     const double discriminant = b * b - 4 * a * c;
//                     if (discriminant >= 0) {
//                         const double t1 = (-b - std::sqrt(discriminant)) / (2.0 * a);
//                         const double t2 = (-b + std::sqrt(discriminant)) / (2.0 * a);
//                         if (std::max(0.0, t1) <= std::min(T_segment, t2)) {
//                             return false; // Collision interval overlaps with segment duration
//                         }
//                     }
//                 }
//             } else { // Static Circle
//                 if (distanceSqrdPointToSegment(obs.position, p_r0, p_r1) <= R_sq) {
//                     return false;
//                 }
//             }
//         } else if (obs.type == Obstacle::BOX) {
//             // Apply inflation to width and height for box checks
//             const double w = obs.dimensions.width + 2 * inflation;
//             const double h = obs.dimensions.height + 2 * inflation;

//             if (obs.is_dynamic) {
//                 const double delta_t = std::max(0.0, time_at_segment_start - obs.last_update_time.seconds());
//                 const Eigen::Vector2d p_o0 = obs.position + obs.velocity * delta_t;
                
//                 // Use the swept-volume test for a moving box
//                 if (sweptBoxIntersection(p_r0, v_r, p_o0, obs.velocity, w, h, T_segment, obs.dimensions.rotation, false)) {
//                     return false;
//                 }
//             } else { // Static Box
//                 if (lineIntersectsRectangle(p_r0, p_r1, obs.position, w, h, obs.dimensions.rotation)) {
//                     return false;
//                 }
//             }
//         }
        
//         time_into_full_trajectory += T_segment;
//     }

//     return true; // No collision found
// }

// // FOR ABSOLUTE TIME!
// bool GazeboObstacleChecker::isTrajectorySafeAgainstSingleObstacle(
//     const Trajectory& trajectory, double global_edge_start_time, const Obstacle& obs) const {
    
//     // 1. Time-to-Goal at the very beginning of the edge
//     const int t_idx = trajectory.path_points[0].size() - 1;
//     const double robot_ttg_at_start = trajectory.path_points[0](t_idx);
//     const double combined_rad_sq = std::pow(obs.dimensions.radius + inflation, 2);

//     // 2. Point-by-Point Appointment (Otte Logic)
//     for (const auto& point : trajectory.path_points) {
//         // SimTime when robot hits this specific waypoint = Edge Start SimTime + (TTG difference)
//         double waypoint_sim_time = global_edge_start_time + (robot_ttg_at_start - point(t_idx));
        
//         // Where is the obstacle at that exact SimTime?
//         double dt_since_last_msg = waypoint_sim_time - obs.last_update_time.seconds();
//         Eigen::Vector2d obs_pos = obs.position;
//         if (obs.is_dynamic) {
//             obs_pos += obs.velocity * dt_since_last_msg;
//         }

//         // Check if robot point (x,y) hits obstacle (x,y) at this appointment
//         if ((point.head<2>() - obs_pos).squaredNorm() < combined_rad_sq) return false;
//     }
//     return true;
// }



// // FOR KNOWN DATA ASSOCIATION WITH OBSTACLE PATH
// // ---------------------------------------------------------
// // The Fixed Single-Obstacle Check
// // ---------------------------------------------------------
// // In gazebo_obstacle_checker.cpp

// bool GazeboObstacleChecker::isTrajectorySafeAgainstSingleObstacle(
//     const Trajectory& trajectory, 
//     double global_edge_start_time, 
//     const Obstacle& ob) const 
// {
//     auto logger = rclcpp::get_logger("GazeboChecker");

    
//     // 1. Basic Validity Checks
//     if (!trajectory.is_valid || trajectory.path_points.empty()) return false;
    
//     if (ob.predicted_path.empty()) return true; 


//     // 2. Setup Thresholds
//     double obs_size = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
//                       std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    
//     // Total collision radius (Robot + Obstacle + Inflation)
//     double threshold_dist = robot_radius_ + obs_size + inflation; 
//     double threshold_sq = threshold_dist * threshold_dist;

//     // 3. Define Robot Edge in Time-Space
//     // The robot edge goes from Start (High T) to End (Low T) in RRTx
//     Eigen::Vector2d robot_pos_start = trajectory.path_points.front().head<2>();
//     Eigen::Vector2d robot_pos_end   = trajectory.path_points.back().head<2>();
    
//     double t_robot_start = global_edge_start_time; 
//     double t_robot_end   = t_robot_start - trajectory.time_duration;

//     // // LOG: Check if the time window makes sense
//     // RCLCPP_INFO(logger, "Check Traj vs [%s]: T_Start=%.2f -> T_End=%.2f (Dur=%.2f) | ObsPath: T=%.2f -> T=%.2f",
//     //     ob.name.c_str(), t_robot_start, t_robot_end, trajectory.time_duration, 
//     //     ob.predicted_path.front().z(), ob.predicted_path.back().z());

//     // -----------------------------------------------------------------------
//     // JULIA PORT START: Normalize to "Early" (Past) and "Late" (Future)
//     // The Julia code says: "make life easier by always checking past to future"
//     // -----------------------------------------------------------------------
//     Eigen::Vector2d P_early, P_late;
//     double T_early, T_late;

//     // RRTx usually goes High Time -> Low Time. We swap to make math easier (Low -> High)
//     if (t_robot_end < t_robot_start) {
//         P_early = robot_pos_end;   T_early = t_robot_end;
//         P_late  = robot_pos_start; T_late  = t_robot_start;
//     } else {
//         P_early = robot_pos_start; T_early = t_robot_start;
//         P_late  = robot_pos_end;   T_late  = t_robot_end;
//     }

//     // 4. Optimization: Find Relevant Obstacle Segments (The Julia "findIndexBeforeTime")
//     // We assume the obstacle path is sorted Descending (Future -> Past), matching RRTx.
//     // We want to find the slice of the obstacle path that overlaps [T_early, T_late].

//     // Find first point where Obs Time <= T_late (Since sorted descending, this is the start of valid range)
//     // Actually, simply iterating all segments and checking overlap is robust and fast enough for <100 points.
//     // But let's keep the optimization if the tube is huge.
    
//     // We iterate through the tube to find segments that temporally overlap the robot's movement
//     for (size_t i = 0; i < ob.predicted_path.size() - 1; ++i) {
        
//         Eigen::Vector3d obs_pt1 = ob.predicted_path[i];     // Point A
//         Eigen::Vector3d obs_pt2 = ob.predicted_path[i+1];   // Point B
        
//         // Normalize Obstacle Segment to Past -> Future
//         Eigen::Vector2d O_early, O_late;
//         double OT_early, OT_late;

//         if (obs_pt2.z() < obs_pt1.z()) {
//             O_early = obs_pt2.head<2>(); OT_early = obs_pt2.z();
//             O_late  = obs_pt1.head<2>(); OT_late  = obs_pt1.z();
//         } else {
//             O_early = obs_pt1.head<2>(); OT_early = obs_pt1.z();
//             O_late  = obs_pt2.head<2>(); OT_late  = obs_pt2.z();
//         }

//         // -----------------------------------------------------------------------
//         // CHECK TIME OVERLAP
//         // Does [T_early, T_late] overlap with [OT_early, OT_late]?
//         // -----------------------------------------------------------------------
//         double overlap_min = std::max(T_early, OT_early);
//         double overlap_max = std::min(T_late, OT_late);

//         if (overlap_min > overlap_max) {
//             continue; // No temporal overlap, skip this segment
//         }

//         // -----------------------------------------------------------------------
//         // ANALYTICAL MATH (Solving for T_c)
//         // -----------------------------------------------------------------------
        
//         // Robot Velocity Vector
//         double robot_dt = T_late - T_early;
//         if (robot_dt < 1e-6) robot_dt = 1e-6; // Avoid div by zero
//         Eigen::Vector2d V_robot = (P_late - P_early) / robot_dt;

//         // Obstacle Velocity Vector for this segment
//         double obs_dt = OT_late - OT_early;
//         if (obs_dt < 1e-6) obs_dt = 1e-6;
//         Eigen::Vector2d V_obs = (O_late - O_early) / obs_dt;

//         // Relative Velocity
//         Eigen::Vector2d V_rel = V_robot - V_obs;

//         // Relative Position at t=0 (Virtual origin)
//         // This calculates the offset between the two lines if extended back to time 0
//         Eigen::Vector2d P_rel_0 = (P_early - V_robot * T_early) - (O_early - V_obs * OT_early);

//         // We want to minimize distance squared D^2(t) = |V_rel * t + P_rel_0|^2
//         // D^2(t) = (V_rel.x * t + P_rel_0.x)^2 + ...
//         // Derivative d(D^2)/dt = 2 * (V_rel dot V_rel) * t + 2 * (P_rel_0 dot V_rel) = 0
        
//         double A = V_rel.dot(V_rel);
//         double B = 2.0 * P_rel_0.dot(V_rel);
        
//         double Tc; // Time of closest approach

//         if (std::abs(A) < 1e-9) {
//             // Objects moving parallel at same speed. Distance is constant.
//             // Check any valid time (e.g., start of overlap)
//             Tc = overlap_min;
//         } else {
//             Tc = -B / (2.0 * A);
//         }

//         // Clamp Tc to the valid overlapping time interval
//         if (Tc < overlap_min) Tc = overlap_min;
//         if (Tc > overlap_max) Tc = overlap_max;

//         // -----------------------------------------------------------------------
//         // FINAL CHECK at T_c
//         // -----------------------------------------------------------------------
//         Eigen::Vector2d pos_robot_at_Tc = P_early + V_robot * (Tc - T_early);
//         Eigen::Vector2d pos_obs_at_Tc   = O_early + V_obs * (Tc - OT_early);

//         double dist_sq = (pos_robot_at_Tc - pos_obs_at_Tc).squaredNorm();

//         if (dist_sq < threshold_sq) {
//             double dist = std::sqrt(dist_sq);
//             double thresh = std::sqrt(threshold_sq);
            
//             // RCLCPP_ERROR(logger, "COLLISION DETECTED with [%s]!", ob.name.c_str());
//             // RCLCPP_ERROR(logger, "  > Time: %.2f (Valid Interval: %.2f - %.2f)", Tc, overlap_min, overlap_max);
//             // RCLCPP_ERROR(logger, "  > Robot: (%.2f, %.2f)", pos_robot_at_Tc.x(), pos_robot_at_Tc.y());
//             // RCLCPP_ERROR(logger, "  > Obs:   (%.2f, %.2f)", pos_obs_at_Tc.x(), pos_obs_at_Tc.y());
//             // RCLCPP_ERROR(logger, "  > Dist: %.3f < Threshold: %.3f", dist, thresh);
//             return false;
//         }
//         // else if (dist_sq < threshold_sq * 1.5) {
//         //      RCLCPP_INFO(logger, "Near Miss [%s] at T=%.2f | Dist=%.2f | Thresh=%.2f", 
//         //         ob.name.c_str(), Tc, std::sqrt(dist_sq), std::sqrt(threshold_sq));
//         // }
//     }

//     return true;
// }

// FOR KNOWN DATA ASSOCIATION WITH OBSTACLE PATH
// ---------------------------------------------------------
// The Fixed Single-Obstacle Check
// ---------------------------------------------------------
// In gazebo_obstacle_checker.cpp

// bool GazeboObstacleChecker::isTrajectorySafeAgainstSingleObstacle(
//     const Trajectory& trajectory, 
//     double global_edge_start_time, 
//     const Obstacle& ob) const 
// {
//     auto logger = rclcpp::get_logger("GazeboChecker");

    
//     // 1. Basic Validity Checks
//     if (!trajectory.is_valid || trajectory.path_points.empty()) return false;
    
//     if (ob.predicted_path.empty()) return true; 


//     // 2. Setup Thresholds
//     double obs_size = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
//                       std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    
//     // Total collision radius (Robot + Obstacle + Inflation)
//     double threshold_dist = robot_radius_ + obs_size + inflation; 
//     double threshold_sq = threshold_dist * threshold_dist;

//     // 3. Define Robot Edge in Time-Space
//     // The robot edge goes from Start (High T) to End (Low T) in RRTx
//     Eigen::Vector2d robot_pos_start = trajectory.path_points.front().head<2>();
//     Eigen::Vector2d robot_pos_end   = trajectory.path_points.back().head<2>();
    
//     double t_robot_start = global_edge_start_time; 
//     double t_robot_end   = t_robot_start - trajectory.time_duration;

//     // // LOG: Check if the time window makes sense
//     // RCLCPP_INFO(logger, "Check Traj vs [%s]: T_Start=%.2f -> T_End=%.2f (Dur=%.2f) | ObsPath: T=%.2f -> T=%.2f",
//     //     ob.name.c_str(), t_robot_start, t_robot_end, trajectory.time_duration, 
//     //     ob.predicted_path.front().z(), ob.predicted_path.back().z());

//     // -----------------------------------------------------------------------
//     // JULIA PORT START: Normalize to "Early" (Past) and "Late" (Future)
//     // The Julia code says: "make life easier by always checking past to future"
//     // -----------------------------------------------------------------------
//     Eigen::Vector2d P_early, P_late;
//     double T_early, T_late;

//     // RRTx usually goes High Time -> Low Time. We swap to make math easier (Low -> High)
//     if (t_robot_end < t_robot_start) {
//         P_early = robot_pos_end;   T_early = t_robot_end;
//         P_late  = robot_pos_start; T_late  = t_robot_start;
//     } else {
//         P_early = robot_pos_start; T_early = t_robot_start;
//         P_late  = robot_pos_end;   T_late  = t_robot_end;
//     }

//     // 4. Optimization: Find Relevant Obstacle Segments (The Julia "findIndexBeforeTime")
//     // We assume the obstacle path is sorted Descending (Future -> Past), matching RRTx.
//     // We want to find the slice of the obstacle path that overlaps [T_early, T_late].

//     // Find first point where Obs Time <= T_late (Since sorted descending, this is the start of valid range)
//     // Actually, simply iterating all segments and checking overlap is robust and fast enough for <100 points.
//     // But let's keep the optimization if the tube is huge.
    
//     // We iterate through the tube to find segments that temporally overlap the robot's movement
//     for (size_t i = 0; i < ob.predicted_path.size() - 1; ++i) {
        
//         Eigen::Vector3d obs_pt1 = ob.predicted_path[i];     // Point A
//         Eigen::Vector3d obs_pt2 = ob.predicted_path[i+1];   // Point B
        
//         // Normalize Obstacle Segment to Past -> Future
//         Eigen::Vector2d O_early, O_late;
//         double OT_early, OT_late;

//         if (obs_pt2.z() < obs_pt1.z()) {
//             O_early = obs_pt2.head<2>(); OT_early = obs_pt2.z();
//             O_late  = obs_pt1.head<2>(); OT_late  = obs_pt1.z();
//         } else {
//             O_early = obs_pt1.head<2>(); OT_early = obs_pt1.z();
//             O_late  = obs_pt2.head<2>(); OT_late  = obs_pt2.z();
//         }

//         // -----------------------------------------------------------------------
//         // CHECK TIME OVERLAP
//         // Does [T_early, T_late] overlap with [OT_early, OT_late]?
//         // -----------------------------------------------------------------------
//         double overlap_min = std::max(T_early, OT_early);
//         double overlap_max = std::min(T_late, OT_late);

//         if (overlap_min > overlap_max) {
//             continue; // No temporal overlap, skip this segment
//         }

//         // -----------------------------------------------------------------------
//         // ANALYTICAL MATH (Solving for T_c)
//         // -----------------------------------------------------------------------
        
//         // Robot Velocity Vector
//         double robot_dt = T_late - T_early;
//         if (robot_dt < 1e-6) robot_dt = 1e-6; // Avoid div by zero
//         Eigen::Vector2d V_robot = (P_late - P_early) / robot_dt;

//         // Obstacle Velocity Vector for this segment
//         double obs_dt = OT_late - OT_early;
//         if (obs_dt < 1e-6) obs_dt = 1e-6;
//         Eigen::Vector2d V_obs = (O_late - O_early) / obs_dt;

//         // Relative Velocity
//         Eigen::Vector2d V_rel = V_robot - V_obs;

//         // Relative Position at t=0 (Virtual origin)
//         // This calculates the offset between the two lines if extended back to time 0
//         Eigen::Vector2d P_rel_0 = (P_early - V_robot * T_early) - (O_early - V_obs * OT_early);

//         // We want to minimize distance squared D^2(t) = |V_rel * t + P_rel_0|^2
//         // D^2(t) = (V_rel.x * t + P_rel_0.x)^2 + ...
//         // Derivative d(D^2)/dt = 2 * (V_rel dot V_rel) * t + 2 * (P_rel_0 dot V_rel) = 0
        
//         double A = V_rel.dot(V_rel);
//         double B = 2.0 * P_rel_0.dot(V_rel);
        
//         double Tc; // Time of closest approach

//         if (std::abs(A) < 1e-9) {
//             // Objects moving parallel at same speed. Distance is constant.
//             // Check any valid time (e.g., start of overlap)
//             Tc = overlap_min;
//         } else {
//             Tc = -B / (2.0 * A);
//         }

//         // Clamp Tc to the valid overlapping time interval
//         if (Tc < overlap_min) Tc = overlap_min;
//         if (Tc > overlap_max) Tc = overlap_max;

//         // -----------------------------------------------------------------------
//         // FINAL CHECK at T_c
//         // -----------------------------------------------------------------------
//         Eigen::Vector2d pos_robot_at_Tc = P_early + V_robot * (Tc - T_early);
//         Eigen::Vector2d pos_obs_at_Tc   = O_early + V_obs * (Tc - OT_early);

//         double dist_sq = (pos_robot_at_Tc - pos_obs_at_Tc).squaredNorm();

//         // ======================================================
//         // PINPOINT LOG: 1287 -> 2203 vs moving_cylinder_5
//         // ======================================================
//         if (ob.name == "moving_cylinder_5" && 
//             trajectory.from_node_index == 1287 && 
//             trajectory.to_node_index == 2203) {
            
//             RCLCPP_ERROR(logger, 
//                 "!!! PINPOINT CHECK [1287 -> 2203] vs [Cyl5] !!!\n"
//                 "   Robot Time Window: [%.2f -> %.2f]\n"
//                 "   Obs Time Window:   [%.2f -> %.2f]\n"
//                 "   Overlap:           [%.2f -> %.2f]\n"
//                 "   Closest Approach:  T=%.2f\n"
//                 "   Distance:          %.3f (Threshold: %.3f)\n"
//                 "   RESULT: %s",
//                 T_early, T_late, OT_early, OT_late, overlap_min, overlap_max, Tc,
//                 std::sqrt(dist_sq), threshold_dist,
//                 (dist_sq < threshold_sq) ? "UNSAFE (COLLISION)" : "SAFE");
//         }
//         // ======================================================

//         if (dist_sq < threshold_sq) {
//             double dist = std::sqrt(dist_sq);
//             double thresh = std::sqrt(threshold_sq);
            
//             // RCLCPP_ERROR(logger, "COLLISION DETECTED with [%s]!", ob.name.c_str());
//             // RCLCPP_ERROR(logger, "  > Time: %.2f (Valid Interval: %.2f - %.2f)", Tc, overlap_min, overlap_max);
//             // RCLCPP_ERROR(logger, "  > Robot: (%.2f, %.2f)", pos_robot_at_Tc.x(), pos_robot_at_Tc.y());
//             // RCLCPP_ERROR(logger, "  > Obs:   (%.2f, %.2f)", pos_obs_at_Tc.x(), pos_obs_at_Tc.y());
//             // RCLCPP_ERROR(logger, "  > Dist: %.3f < Threshold: %.3f", dist, thresh);
//             return false;
//         }
//         // else if (dist_sq < threshold_sq * 1.5) {
//         //      RCLCPP_INFO(logger, "Near Miss [%s] at T=%.2f | Dist=%.2f | Thresh=%.2f", 
//         //         ob.name.c_str(), Tc, std::sqrt(dist_sq), std::sqrt(threshold_sq));
//         // }
//     }

//     return true;
// }

bool GazeboObstacleChecker::isTrajectorySafeAgainstSingleObstacle(
    const Trajectory& trajectory, 
    double global_edge_start_time, 
    const Obstacle& ob) const 
{
    auto logger = rclcpp::get_logger("GazeboChecker");

    // 1. Basic Validity Checks
    if (!trajectory.is_valid || trajectory.path_points.empty()) return false;
    if (ob.predicted_path.empty()) return true; 

    // 2. Setup Thresholds
    double obs_size = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                      std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    
    double threshold_dist = robot_radius_ + obs_size + inflation; 
    double threshold_sq = threshold_dist * threshold_dist;

    // 3. Define Robot Edge in Time-Space
    Eigen::Vector2d robot_pos_start = trajectory.path_points.front().head<2>();
    Eigen::Vector2d robot_pos_end   = trajectory.path_points.back().head<2>();
    
    double t_robot_start = global_edge_start_time; 
    double t_robot_end   = t_robot_start - trajectory.time_duration;

    // Normalize Robot Time (Early -> Late)
    Eigen::Vector2d P_early, P_late;
    double T_early, T_late;
    if (t_robot_end < t_robot_start) {
        P_early = robot_pos_end;   T_early = t_robot_end;
        P_late  = robot_pos_start; T_late  = t_robot_start;
    } else {
        P_early = robot_pos_start; T_early = t_robot_start;
        P_late  = robot_pos_end;   T_late  = t_robot_end;
    }

    // Iterate through obstacle path segments
    for (size_t i = 0; i < ob.predicted_path.size() - 1; ++i) {
        Eigen::Vector3d obs_pt1 = ob.predicted_path[i];     // Point A
        Eigen::Vector3d obs_pt2 = ob.predicted_path[i+1];   // Point B
        
        // Normalize Obstacle Time (Early -> Late)
        Eigen::Vector2d O_early, O_late;
        double OT_early, OT_late;
        if (obs_pt2.z() < obs_pt1.z()) {
            O_early = obs_pt2.head<2>(); OT_early = obs_pt2.z();
            O_late  = obs_pt1.head<2>(); OT_late  = obs_pt1.z();
        } else {
            O_early = obs_pt1.head<2>(); OT_early = obs_pt1.z();
            O_late  = obs_pt2.head<2>(); OT_late  = obs_pt2.z();
        }

        // Check Time Overlap
        double overlap_min = std::max(T_early, OT_early);
        double overlap_max = std::min(T_late, OT_late);

        if (overlap_min > overlap_max) {
            continue; // No temporal overlap
        }
        // -----------------------------------------------------------------------
        // ANALYTICAL MATH (Robust Version)
        // -----------------------------------------------------------------------
        double robot_dt = T_late - T_early;
        if (robot_dt < 1e-6) robot_dt = 1e-6;
        Eigen::Vector2d V_robot = (P_late - P_early) / robot_dt;

        double obs_dt = OT_late - OT_early;
        if (obs_dt < 1e-6) obs_dt = 1e-6;
        Eigen::Vector2d V_obs = (O_late - O_early) / obs_dt;
        // if(ob.name=="moving_cylinder_5"){
        //     std::cout<<"vOBS: "<<V_obs<<"\n";
        // }

        Eigen::Vector2d V_rel = V_robot - V_obs;

        Eigen::Vector2d P_robot_at_min = P_early + V_robot * (overlap_min - T_early);
        Eigen::Vector2d P_obs_at_min   = O_early + V_obs   * (overlap_min - OT_early);
        Eigen::Vector2d P_rel_at_min = P_robot_at_min - P_obs_at_min;

        double A = V_rel.dot(V_rel);
        double B = 2.0 * P_rel_at_min.dot(V_rel);
        
        double Tc_offset = (std::abs(A) < 1e-9) ? 0.0 : -B / (2.0 * A);
        double Tc = overlap_min + Tc_offset;

        if (Tc < overlap_min) Tc = overlap_min;
        if (Tc > overlap_max) Tc = overlap_max;
        
        // -----------------------------------------------------------------------
        // ROBUSTNESS FIX: Check Boundaries
        // -----------------------------------------------------------------------
        // std::vector<double> times_to_check = {Tc, overlap_min, overlap_max};
        double eps_check = std::max(1e-4, (overlap_max - overlap_min) * 1e-2);
        std::vector<double> times_to_check = {Tc, overlap_min, overlap_max, Tc - eps_check, Tc + eps_check};


        
        for (double t_current : times_to_check) {
            Eigen::Vector2d pos_robot_at_t = P_early + V_robot * (t_current - T_early);
            Eigen::Vector2d pos_obs_at_t   = O_early + V_obs   * (t_current - OT_early);
            double dist_sq = (pos_robot_at_t - pos_obs_at_t).squaredNorm();
            
            // // After computing dist_sq and threshold_sq; inside the loop for times_to_check

            // const double refine_margin = 0.20; // meters
            // double refine_thresh_sq = (threshold_dist + refine_margin) * (threshold_dist + refine_margin);
            // if (dist_sq > refine_thresh_sq) {
            //     // definitely far, skip heavy refinement
            // } else {
            //     const double refine_max_dist = 0.01; // 5 cm
            //     double obs_speed = V_obs.norm();
            //     double dt_refine = (obs_speed > 1e-6) ? (refine_max_dist / obs_speed) : 0.02;
            //     // keep dt_refine in a reasonable band
            //     dt_refine = std::clamp(dt_refine, 0.001, 0.01);

            //     // cap the number of refine samples
            //     const size_t MAX_REFINE_SAMPLES = 200;
            //     size_t approx_samples = static_cast<size_t>(std::ceil((overlap_max - overlap_min) / dt_refine));
            //     if (approx_samples > MAX_REFINE_SAMPLES) {
            //         dt_refine = (overlap_max - overlap_min) / static_cast<double>(MAX_REFINE_SAMPLES);
            //         if (dt_refine <= 0.0) dt_refine = 1e-3;
            //     }

            //     for (double rt = overlap_min; rt <= overlap_max + 1e-12; rt += dt_refine) {
            //         Eigen::Vector2d pos_robot_rt = P_early + V_robot * (rt - T_early);
            //         Eigen::Vector2d pos_obs_rt   = O_early + V_obs   * (rt - OT_early);
            //         double d2 = (pos_robot_rt - pos_obs_rt).squaredNorm();
            //         if (d2 < threshold_sq) {
            //             return false; // collision found during refined checks
            //         }
            //     }
            //     // final exact check
            //     {
            //         Eigen::Vector2d pr = P_early + V_robot * (overlap_max - T_early);
            //         Eigen::Vector2d po = O_early + V_obs   * (overlap_max - OT_early);
            //         if ((pr - po).squaredNorm() < threshold_sq) return false;
            //     }
            // }

            if (dist_sq < threshold_sq) {
                return false;
            }
        }
    }
    return true;
}
// bool GazeboObstacleChecker::isTrajectorySafeAgainstSingleObstacle(
//     const Trajectory& trajectory, 
//     double global_edge_start_time, 
//     const Obstacle& ob) const 
// {
//     auto logger = rclcpp::get_logger("GazeboChecker");

//     // 1. Basic Validity Checks
//     if (!trajectory.is_valid || trajectory.path_points.empty()) return false;
//     if (ob.predicted_path.empty()) return true; 

//     // 2. Setup Thresholds
//     double obs_size = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
//                       std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    
//     double threshold_dist = robot_radius_ + obs_size + inflation; 
//     double threshold_sq = threshold_dist * threshold_dist;

//     // 3. Define Robot Edge in Time-Space
//     Eigen::Vector2d robot_pos_start = trajectory.path_points.front().head<2>();
//     Eigen::Vector2d robot_pos_end   = trajectory.path_points.back().head<2>();
    
//     double t_robot_start = global_edge_start_time; 
//     double t_robot_end   = t_robot_start - trajectory.time_duration;

//     // Normalize Robot Time (Early -> Late)
//     Eigen::Vector2d P_early, P_late;
//     double T_early, T_late;
//     if (t_robot_end < t_robot_start) {
//         P_early = robot_pos_end;   T_early = t_robot_end;
//         P_late  = robot_pos_start; T_late  = t_robot_start;
//     } else {
//         P_early = robot_pos_start; T_early = t_robot_start;
//         P_late  = robot_pos_end;   T_late  = t_robot_end;
//     }

//     // Iterate through obstacle path segments
//     for (size_t i = 0; i < ob.predicted_path.size() - 1; ++i) {
//         Eigen::Vector3d obs_pt1 = ob.predicted_path[i];     // Point A
//         Eigen::Vector3d obs_pt2 = ob.predicted_path[i+1];   // Point B
        
//         // Normalize Obstacle Time (Early -> Late)
//         Eigen::Vector2d O_early, O_late;
//         double OT_early, OT_late;
//         if (obs_pt2.z() < obs_pt1.z()) {
//             O_early = obs_pt2.head<2>(); OT_early = obs_pt2.z();
//             O_late  = obs_pt1.head<2>(); OT_late  = obs_pt1.z();
//         } else {
//             O_early = obs_pt1.head<2>(); OT_early = obs_pt1.z();
//             O_late  = obs_pt2.head<2>(); OT_late  = obs_pt2.z();
//         }

//         // Check Time Overlap
//         double overlap_min = std::max(T_early, OT_early);
//         double overlap_max = std::min(T_late, OT_late);

//         if (overlap_min > overlap_max) {
//             continue; // No temporal overlap
//         }

//         // ======================================================
//         // ARTIFICIAL CHECK (Keep your debug)
//         // ======================================================
//         if (ob.name == "moving_cylinder_5" &&
//             trajectory.from_node_index == 1287 &&
//             trajectory.to_node_index == 2203)
//         {
//             const double T_probe = 3.14;
//             if (T_probe >= OT_early && T_probe <= OT_late &&
//                 T_probe >= T_early && T_probe <= T_late)
//             {
//                 double obs_dt = OT_late - OT_early;
//                 Eigen::Vector2d V_obs = (O_late - O_early) / (obs_dt < 1e-6 ? 1.0 : obs_dt);
//                 Eigen::Vector2d pos_obs = O_early + V_obs * (T_probe - OT_early);

//                 double robot_dt = T_late - T_early;
//                 Eigen::Vector2d V_robot = (P_late - P_early) / (robot_dt < 1e-6 ? 1.0 : robot_dt);
//                 Eigen::Vector2d pos_robot = P_early + V_robot * (T_probe - T_early);

//                 double dist = (pos_robot - pos_obs).norm();
//                 RCLCPP_ERROR(logger,
//                     "!!! EDGE-PINPOINT CHECK [1287 -> 2203] vs [Cyl5] !!!\n"
//                     "   Obstacle Time:  %.2f\n"
//                     "   Robot Edge:     [%.2f -> %.2f]\n"
//                     "   Robot Pos:      (%.2f, %.2f)\n"
//                     "   Obs Pos:        (%.2f, %.2f)\n"
//                     "   Distance:       %.3f (Threshold: %.3f)\n"
//                     "   RESULT: %s",
//                     T_probe, T_early, T_late,
//                     pos_robot.x(), pos_robot.y(),
//                     pos_obs.x(), pos_obs.y(),
//                     dist, threshold_dist,
//                     (dist < threshold_dist) ? "COLLISION" : "SAFE");
//             }
//         }
//         // ======================================================

//         // -----------------------------------------------------------------------
//         // CONTINUOUS TIME ANALYTICAL MATH (The Fix)
//         // -----------------------------------------------------------------------
        
//         // 1. Calculate Velocities
//         double robot_dt = T_late - T_early;
//         if (robot_dt < 1e-6) robot_dt = 1e-6;
//         Eigen::Vector2d V_robot = (P_late - P_early) / robot_dt;

//         double obs_dt = OT_late - OT_early;
//         if (obs_dt < 1e-6) obs_dt = 1e-6;
//         Eigen::Vector2d V_obs = (O_late - O_early) / obs_dt;

//         // 2. Relative Motion
//         // P_rel(t) = (P_early + V_robot * t) - (O_early + V_obs * t)
//         // P_rel(t) = P_rel_0 + V_rel * t
//         // Note: t here is time relative to overlap_min
//         Eigen::Vector2d V_rel = V_robot - V_obs;
        
//         Eigen::Vector2d P_robot_at_min = P_early + V_robot * (overlap_min - T_early);
//         Eigen::Vector2d P_obs_at_min   = O_early + V_obs   * (overlap_min - OT_early);
//         Eigen::Vector2d P_rel_at_min = P_robot_at_min - P_obs_at_min;

//         // 3. Find Time of Closest Approach (Tc)
//         // Distance^2(t) = (P_rel_at_min + V_rel * t)^2
//         // D(t) = A*t^2 + B*t + C
//         // dD/dt = 2*A*t + B = 0  =>  t = -B / 2A
        
//         double A = V_rel.dot(V_rel);
//         double B = 2.0 * P_rel_at_min.dot(V_rel);
//         // C is not needed for finding the time, only for distance
        
//         double t_critical = 0.0;
//         if (std::abs(A) > 1e-9) {
//             t_critical = -B / (2.0 * A);
//         }

//         // 4. Collect Candidate Times
//         // We must check the critical point AND the boundaries of the overlap window
//         std::vector<double> times_to_check_relative;
        
//         // Add the critical point (clamped to overlap window)
//         if (t_critical >= 0.0 && t_critical <= (overlap_max - overlap_min)) {
//             times_to_check_relative.push_back(t_critical);
//         }
        
//         // Always add boundaries
//         times_to_check_relative.push_back(0.0);
//         times_to_check_relative.push_back(overlap_max - overlap_min);

//         // 5. Check Candidates
//         for (double t_rel : times_to_check_relative) {
//             double t_current = overlap_min + t_rel;

//             Eigen::Vector2d pos_robot_at_t = P_early + V_robot * (t_current - T_early);
//             Eigen::Vector2d pos_obs_at_t   = O_early + V_obs   * (t_current - OT_early);
            
//             double dist_sq = (pos_robot_at_t - pos_obs_at_t).squaredNorm();
            
//             // Logging
//             if (ob.name == "moving_cylinder_5" && 
//                 trajectory.from_node_index == 1287 && 
//                 trajectory.to_node_index == 2203) {
                
//                 RCLCPP_ERROR(logger, 
//                     "!!! PINPOINT CHECK [1287 -> 2203] vs [Cyl5] (FIXED) !!!\n"
//                     "   Checking Time:     %.4f\n"
//                     "   Robot Time Window: [%.2f -> %.2f]\n"
//                     "   Obs Time Window:   [%.2f -> %.2f]\n"
//                     "   Overlap:           [%.2f -> %.2f]\n"
//                     "   Distance:          %.3f (Threshold: %.3f)\n"
//                     "   RESULT: %s",
//                     t_current, T_early, T_late, OT_early, OT_late, overlap_min, overlap_max,
//                     std::sqrt(dist_sq), threshold_dist,
//                     (dist_sq < threshold_sq) ? "UNSAFE (COLLISION)" : "SAFE");
//             }

//             if (dist_sq < threshold_sq) {
//                 return false; // Collision detected
//             }
//         }
//     }
//     return true;
// }



/**
 * Performs a continuous, analytical collision check for a moving point (robot) against a moving box (obstacle).
 * It can handle both non-rotating (AABB) and rotated (OBB) boxes.
 */
bool GazeboObstacleChecker::sweptBoxIntersection(
    const Eigen::Vector2d& p_r0, const Eigen::Vector2d& v_r,
    const Eigen::Vector2d& p_o0, const Eigen::Vector2d& v_o,
    double w, double h, double T_segment,
    double rotation, bool consider_rotation) const
{
    // Calculate relative motion. The problem becomes a moving point vs. a stationary box.
    const Eigen::Vector2d v_rel = v_r - v_o;
    const Eigen::Vector2d p_rel_start = p_r0 - p_o0;

    Eigen::Vector2d p_local_start = p_rel_start;
    Eigen::Vector2d v_local = v_rel;

    // If rotation needs to be considered, transform into the box's local frame.
    if (consider_rotation) {
        Eigen::Rotation2Dd rot(-rotation);
        p_local_start = rot * p_rel_start;
        v_local = rot * v_rel;
    }

    // Perform a slab-based (ray-AABB) intersection test in the box's (potentially rotated) frame.
    double t_near = 0.0;
    double t_far = T_segment;
    const double half_w = w / 2.0;
    const double half_h = h / 2.0;

    for (int i = 0; i < 2; ++i) { // Iterate over x and y axes
        const double slab_min = (i == 0) ? -half_w : -half_h;
        const double slab_max = (i == 0) ?  half_w :  half_h;

        if (std::abs(v_local[i]) < 1e-9) {
            // Ray is parallel to the slab. If it's outside the slab, no collision is possible.
            if (p_local_start[i] < slab_min || p_local_start[i] > slab_max) {
                return false;
            }
        } else {
            // Calculate intersection times with the slab planes.
            double t1 = (slab_min - p_local_start[i]) / v_local[i];
            double t2 = (slab_max - p_local_start[i]) / v_local[i];

            if (t1 > t2) std::swap(t1, t2); // Ensure t1 is the earlier time

            t_near = std::max(t_near, t1);
            t_far = std::min(t_far, t2);

            if (t_near > t_far) {
                return false;
            }
        }
    }
    return t_near <= T_segment;
}






// bool GazeboObstacleChecker::isTrajectorySafe(
//     const Trajectory& trajectory,
//     double global_edge_start_time
// ) const {
//     // A nullopt from getCollidingObstacle means the path is safe.
//     // return isObstacleFree(trajectory.path_points.at(0),trajectory.path_points.at(1));
//     if (use_fcl)
//         return !getCollidingObstacleFCL(trajectory, global_edge_start_time).has_value();
//     else if (use_bullet)
//         return !getCollidingObstacleBullet(trajectory, global_edge_start_time).has_value();
//     else
//         return !getCollidingObstacle(trajectory, global_edge_start_time).has_value();

// }


bool GazeboObstacleChecker::isTrajectorySafe(
    const Trajectory& trajectory,
    double global_edge_start_time // Time-To-Goal (TTG)
) const {
    // 1. Get the current world snapshot
    // (This list already contains the updated positions/velocities)
    ObstacleVector all_obs = getObstacles(); 

    // 2. Loop through EVERY obstacle and check its Tube
    for (const auto& ob : all_obs) {
        // If an edge hits ANY obstacle's predicted path, it's NOT safe
        if (!isTrajectorySafeAgainstSingleObstacle(trajectory, global_edge_start_time, ob)) {
            return false; 
        }
    }

    return true; // Safe against the whole world
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
//         // --- Calculate Collision Radii ---
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
        
//         // ---  Predictive Check for Dynamic Obstacles ---
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

//             // Calculate robot's position at this time step
//             Eigen::Vector2d p_robot_at_tau = p_r0 + v_r * current_tau;

//             // Calculate obstacle's position at this time step
//             Eigen::Vector2d p_obs_at_tau = p_o0 + v_o0 * current_tau;
            
//             // Check for collision by comparing squared distance
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
        
//         // --- Static Collision Check ---
//         // This is a fast, geometric check that ignores obstacle velocity.
//         // It provides an immediate layer of safety.
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

//                 // Calculate robot's position at this time step
//                 Eigen::Vector2d p_robot_at_tau = p_r0 + v_r * current_tau;

//                 // Calculate obstacle's position at this time step
//                 Eigen::Vector2d p_obs_at_tau = p_o0 + v_o0 * current_tau;
                
//                 // Check for collision by comparing squared distance
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
//             // --- Define Collision Radii ---
//             const double obs_radius = (obs.type == Obstacle::CIRCLE)
//                                     ? obs.dimensions.radius
//                                     : std::hypot(obs.dimensions.width / 2.0, obs.dimensions.height / 2.0);
//             const double R = obs_radius + inflation;
//             const double R_sq = R * R;
//             const double v_max_obs = 12.0; // A reasonable upper bound on any obstacle's speed

//             // --- Broad-Phase Check ---
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

//             // --- Static Collision Check (Narrow-Phase part 1) ---
//             // If the obstacle is static, the broad-phase check is sufficient.
//             if (!obs.is_dynamic) {
//                 if (min_dist_sq_to_path_geom <= R_sq) {
//                     return obs; // Definite collision with a static obstacle.
//                 }
//                 continue; // Static obstacle is safe, move to the next obstacle.
//             }

//             // --- Dynamic Predictive Check (Narrow-Phase part 2) ---
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




// // THE LAST PERFECT SOLUTION (WITHOUT FCL and only for circle obstalces)
// std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacle(
//     const Trajectory& trajectory,
//     double global_start_time
// ) const {
//     /*
//         I Guarantee that getAtomicSnapshot() is only called once before plan() so no race between getAtomicSnapshot() and getCollidingObstalce for obstalce_snapshots variable. Hence I can comment the following lock
//         The lock in getAtomicSnapshot() is for the race between getAtomicSnapshot (Reader/Copier runs on my main planner thread.) and poseInfoCallback (Writer runs on the gazebo thread) and thats necessary

//     */
//     // std::lock_guard<std::mutex> lock(snapshot_mutex_);

//     // auto start = std::chrono::steady_clock::now();


//     // Initial validation of the trajectory
//     if (trajectory.path_points.size() < 2) {
//         return std::nullopt;
//     }

//     // Define helper lambdas to easily access parts of the state vector
//     auto get_xy = [](const Eigen::VectorXd& state) { return state.head<2>(); };
//     auto get_time = [](const Eigen::VectorXd& state) { return state(state.size() - 1); };
//     auto get_vxy = [](const Eigen::VectorXd& state) { return state.segment<2>(2); }; // For 5D states

//     double time_into_full_trajectory = 0.0;
//     const int state_dim = trajectory.path_points[0].size();

//     // Iterate over each segment of the trajectory
//     for (size_t i = 0; i < trajectory.path_points.size() - 1; ++i) {
//         const Eigen::VectorXd& segment_start_state = trajectory.path_points[i];
//         const Eigen::VectorXd& segment_end_state   = trajectory.path_points[i + 1];

//         const double T_segment = get_time(segment_start_state) - get_time(segment_end_state);
//         if (T_segment <= 1e-9) continue;
        
//         const double global_time_at_segment_start = global_start_time + time_into_full_trajectory;

//         // Conditionally apply the correct physics model based on state dimension
//         if (state_dim == 5) {
//         // if (false) {
//             // ACCELERATION MODEL (5D): Subdivide the curved path
//             const int num_subdivisions = 1; // A tunable parameter for accuracy vs. performance
            
//             // Calculate the constant acceleration for the entire segment
//             const Eigen::Vector2d p_r0_seg = get_xy(segment_start_state);
//             const Eigen::Vector2d v_r0_seg = get_vxy(segment_start_state);
//             const Eigen::Vector2d v_r1_seg = get_vxy(segment_end_state);
//             const Eigen::Vector2d a_r_seg = (v_r1_seg - v_r0_seg) / T_segment;
            
//             Eigen::Vector2d p_sub_start = p_r0_seg;
//             double t_sub_start = 0.0;

//             for (int j = 1; j <= num_subdivisions; ++j) {
//                 // Calculate the state at the end of the current sub-segment using constant acceleration physics
//                 double t_sub_end = (static_cast<double>(j) / num_subdivisions) * T_segment;
//                 Eigen::Vector2d p_sub_end = p_r0_seg + v_r0_seg * t_sub_end + 0.5 * a_r_seg * t_sub_end * t_sub_end;
                
//                 // Now, perform the linear collision check on this small, accurate sub-segment
//                 const double T_sub_segment = t_sub_end - t_sub_start;
//                 const Eigen::Vector2d v_r_sub = (p_sub_end - p_sub_start) / T_sub_segment;

//                 for (const auto& obs : obstacle_snapshot_) {
//                     const double obs_radius = (obs.type == Obstacle::CIRCLE)
//                                             ? obs.dimensions.radius
//                                             : std::hypot(obs.dimensions.width / 2.0, obs.dimensions.height / 2.0);
//                     const double R = obs_radius + inflation;
//                     const double R_sq = R * R;


//                     // // --- OPTIMIZATION: BROAD-PHASE CHECK --- This Phase gives us Some performance boost but it makes the detection to be delayed a bit because it ignores detection after some distance
//                     // // Calculate the maximum distance the obstacle could travel during this segment
//                     // const double v_max_obs = 30.0; // A reasonable upper bound on any obstacle's speed
//                     // const double max_obs_travel_dist = v_max_obs * T_segment;
                    
//                     // // The "danger zone" is the collision radius plus this travel distance
//                     // const double danger_radius = R + max_obs_travel_dist;
//                     // const double danger_radius_sq = danger_radius * danger_radius;

//                     // // Get the geometric path of the robot segment (line or curve)
//                     // const Eigen::Vector2d p_r0 = get_xy(segment_start_state);
//                     // const Eigen::Vector2d p_r1 = get_xy(segment_end_state);
                    
//                     // // Check if the obstacle is geometrically too far to ever reach the path
//                     // if (distanceSqrdPointToSegment(obs.position, p_r0, p_r1) > danger_radius_sq) {
//                     //     continue; // Skip this obstacle; it's too far away
//                     // }
//                     // // --- END OF OPTIMIZATION ---




//                     if (obs.is_dynamic) {
//                         // Dynamic obstacle check for the sub-segment
//                         const double global_time_at_sub_segment_start = global_time_at_segment_start + t_sub_start;
//                         const double delta_t_extrapolation = std::max(0.0, global_time_at_sub_segment_start - obs.last_update_time.seconds());
//                         const Eigen::Vector2d p_o0 = obs.position + obs.velocity * delta_t_extrapolation;
                        
//                         const Eigen::Vector2d p_relative_start = p_sub_start - p_o0;
//                         const Eigen::Vector2d v_relative = v_r_sub - obs.velocity;
                        
//                         const double a = v_relative.dot(v_relative);
//                         const double b = 2.0 * p_relative_start.dot(v_relative);
//                         const double c = p_relative_start.dot(p_relative_start) - R_sq;

//                         if (std::abs(a) < 1e-9) { 
//                             if (c <= 0) return obs; // Initially overlapping
//                             continue;
//                         }

//                         const double discriminant = b * b - 4 * a * c;
//                         if (discriminant >= 0) {
//                             const double sqrt_disc = std::sqrt(discriminant);
//                             const double t1 = (-b - sqrt_disc) / (2.0 * a);
//                             const double t2 = (-b + sqrt_disc) / (2.0 * a);
//                             if (std::max(0.0, t1) <= std::min(T_sub_segment, t2)) {
//                                 return obs; // Collision found
//                             }
//                         }
//                     } else { 
//                         // Static obstacle check for the sub-segment
//                         if (distanceSqrdPointToSegment(obs.position, p_sub_start, p_sub_end) <= R_sq) {
//                             return obs;
//                         }
//                     }
//                 }
//                 // Prepare for the next sub-segment
//                 p_sub_start = p_sub_end;
//                 t_sub_start = t_sub_end;
//             }
//         } else {
//             // CONSTANT VELOCITY MODEL (First-Order / Other Systems)
//             const Eigen::Vector2d p_r0 = get_xy(segment_start_state);
//             const Eigen::Vector2d p_r1 = get_xy(segment_end_state);
//             const Eigen::Vector2d v_r = (p_r1 - p_r0) / T_segment;

//             for (const auto& obs : obstacle_snapshot_) {
//                 const double obs_radius = (obs.type == Obstacle::CIRCLE)
//                                         ? obs.dimensions.radius
//                                         : std::hypot(obs.dimensions.width / 2.0, obs.dimensions.height / 2.0);
//                 const double R = obs_radius + inflation;
//                 const double R_sq = R * R;

//                 // // --- OPTIMIZATION: BROAD-PHASE CHECK ---
//                 // // Calculate the maximum distance the obstacle could travel during this segment
//                 // const double v_max_obs = 15.0; // A reasonable upper bound on any obstacle's speed
//                 // const double max_obs_travel_dist = v_max_obs * T_segment;
                
//                 // // The "danger zone" is the collision radius plus this travel distance
//                 // const double danger_radius = R + max_obs_travel_dist;
//                 // const double danger_radius_sq = danger_radius * danger_radius;

//                 // // Get the geometric path of the robot segment (line or curve)
//                 // const Eigen::Vector2d p_r0 = get_xy(segment_start_state);
//                 // const Eigen::Vector2d p_r1 = get_xy(segment_end_state);
                
//                 // // Check if the obstacle is geometrically too far to ever reach the path
//                 // if (distanceSqrdPointToSegment(obs.position, p_r0, p_r1) > danger_radius_sq) {
//                 //     continue; // Skip this obstacle; it's too far away
//                 // }
//                 // // --- END OF OPTIMIZATION ---



//                 if (obs.is_dynamic) {
//                     // Dynamic obstacle check for the whole linear segment
//                     const double delta_t_extrapolation = std::max(0.0, global_time_at_segment_start - obs.last_update_time.seconds());
//                     const Eigen::Vector2d p_o0 = obs.position + obs.velocity * delta_t_extrapolation;
                    
//                     const Eigen::Vector2d p_relative_start = p_r0 - p_o0;
//                     const Eigen::Vector2d v_relative = v_r - obs.velocity;
                    
//                     const double a = v_relative.dot(v_relative);
//                     const double b = 2.0 * p_relative_start.dot(v_relative);
//                     const double c = p_relative_start.dot(p_relative_start) - R_sq;

//                     if (std::abs(a) < 1e-9) { 
//                         if (c <= 0) return obs;
//                         continue;
//                     }

//                     const double discriminant = b * b - 4 * a * c;
//                     if (discriminant >= 0) {
//                         const double sqrt_disc = std::sqrt(discriminant);
//                         const double t1 = (-b - sqrt_disc) / (2.0 * a);
//                         const double t2 = (-b + sqrt_disc) / (2.0 * a);
//                         if (std::max(0.0, t1) <= std::min(T_segment, t2)) {
//                             return obs;
//                         }
//                     }
//                 } else {
//                     // Static obstacle check for the whole linear segment
//                     if (distanceSqrdPointToSegment(obs.position, p_r0, p_r1) <= R_sq) {
//                         return obs;
//                     }
//                 }
//             }
//         }
        
//         // Update the time offset for the next segment
//         time_into_full_trajectory += T_segment;
//     }
//     // auto end = std::chrono::steady_clock::now();
//     // auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
//     // std::cout << "time taken for the update : " << duration.count() << " nano-seconds\n";


//     // 5. If the loop completes, no collisions were found
//     return std::nullopt;
// }



// GOOD
std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacle(
    const Trajectory& trajectory,
    double global_start_time
) const {
    // auto start = std::chrono::steady_clock::now();

    auto recordCulprit = [&](const Obstacle& obs) {
            if (culprit_names_.find(obs.name) == culprit_names_.end()) {
                culprit_names_.insert(obs.name);
                culprit_cache_.push_back(obs);
            }
    };

    if (trajectory.path_points.size() < 2) {
        return std::nullopt;
    }
    // bool debug_mode_ = true;
    //  3D spatial Collision check
    if (spatial_dim_ == 3) {
        // if (debug_mode_) {
        //     RCLCPP_INFO(rclcpp::get_logger("CollisionCheck3D"), "\n--- Checking 3D Trajectory (Current Sim Time: %.2f) ---", global_start_time);
        // }

        auto get_xyz = [](const Eigen::VectorXd& state) { return state.head<3>(); };
        auto get_time = [](const Eigen::VectorXd& state) { return state.tail<1>()[0]; };
        double time_into_full_trajectory = 0.0;

        for (size_t i = 0; i < trajectory.path_points.size() - 1; ++i) {
            const auto& start_state = trajectory.path_points[i];
            const auto& end_state = trajectory.path_points[i+1];
            const double T = get_time(start_state) - get_time(end_state);
            if (T <= 1e-9) continue;
            
            const double seg_start_time = global_start_time + time_into_full_trajectory;
            const Eigen::Vector3d p_r0 = get_xyz(start_state);
            const Eigen::Vector3d p_r1 = get_xyz(end_state);
            const Eigen::Vector3d v_r = (p_r1 - p_r0) / T;

            // if (debug_mode_) {
            //     RCLCPP_INFO(rclcpp::get_logger("CollisionCheck3D"), "  [Seg %zu] T=%.2fs, Robot Start: (%.1f, %.1f, %.1f), Vel: (%.1f, %.1f, %.1f)", i, T, p_r0.x(), p_r0.y(), p_r0.z(), v_r.x(), v_r.y(), v_r.z());
            // }

            for (const auto& obs : obstacle_snapshot_) {
                Eigen::Vector3d obs_pos(obs.position.x(), obs.position.y(), obs.z);
                Eigen::Vector3d obs_vel(obs.velocity.x(), obs.velocity.y(), 0.0);

                // if (debug_mode_ && obs.is_dynamic) {
                //     RCLCPP_INFO(rclcpp::get_logger("CollisionCheck3D"), "    -> vs Obstacle '%s'", obs.name.c_str());
                //     double dt_extrapolate = std::max(0.0, seg_start_time - obs.last_update_time.seconds());
                //     Eigen::Vector3d p_o0 = obs_pos + obs_vel * dt_extrapolate;

                //     RCLCPP_INFO(rclcpp::get_logger("CollisionCheck3D"), "       - Snap Pos: (%.1f, %.1f, %.1f) @ t=%.2f", obs_pos.x(), obs_pos.y(), obs_pos.z(), obs.last_update_time.seconds());
                //     RCLCPP_INFO(rclcpp::get_logger("CollisionCheck3D"), "       - Seg Start Time: %.2f -> Extrapolation dt: %.2f", seg_start_time, dt_extrapolate);
                //     RCLCPP_INFO(rclcpp::get_logger("CollisionCheck3D"), "       - Extrapolated Obs Pos: (%.1f, %.1f, %.1f)", p_o0.x(), p_o0.y(), p_o0.z());
                // }

                if (obs.type == Obstacle::CIRCLE) {
                    const double R_sq = std::pow(obs.dimensions.radius + inflation, 2);
                    if (obs.is_dynamic) {
                        const double dt = std::max(0.0, seg_start_time - obs.last_update_time.seconds());
                        const Eigen::Vector3d p_o0 = obs_pos + obs_vel * dt;
                        const Eigen::Vector3d p_rel = p_r0 - p_o0;
                        const Eigen::Vector3d v_rel = v_r - obs_vel;
                        const double a = v_rel.dot(v_rel), b = 2*p_rel.dot(v_rel), c = p_rel.dot(p_rel) - R_sq;
                        const double disc = b*b - 4*a*c;
                        
                        // if (debug_mode_) {
                        //     RCLCPP_INFO(rclcpp::get_logger("CollisionCheck3D"), "       - Sphere Check: a=%.2f, b=%.2f, c=%.2f, disc=%.2f", a, b, c, disc);
                        // }

                        if (disc >= 0) {
                            const double t1 = (-b-sqrt(disc))/(2*a);
                            const double t2 = (-b+sqrt(disc))/(2*a);
                            // if (debug_mode_) {
                            //     RCLCPP_INFO(rclcpp::get_logger("CollisionCheck3D"), "       - Collision interval: [%.2f, %.2f] vs Segment duration: [0.00, %.2f]", t1, t2, T);
                            // }
                            if (std::max(0.0, t1) <= std::min(T, t2)) {
                                // RCLCPP_WARN(rclcpp::get_logger("CollisionCheck3D"), "       - !!!!! COLLISION DETECTED !!!!!");
                                recordCulprit(obs);
                                return obs;
                            }
                        }
                    } else { // Static sphere
                        if (distanceSqrdPointToSegment3D(obs_pos, p_r0, p_r1) <= R_sq) 
                        recordCulprit(obs);
                        return obs;
                    }
                }  else if (obs.type == Obstacle::BOX) {
                    const double w = obs.dimensions.width+2*inflation, h = obs.dimensions.height+2*inflation, d = obs.dimensions.height+2*inflation;
                    const Eigen::Vector3d box_half_sizes(w / 2.0, h / 2.0, d / 2.0);

                    if (obs.is_dynamic) {
                        // Define the robot's size for the Minkowski sum.
                        // For now, we assume a point robot (zero size) to match the original line-vs-box logic.
                        // For a more robust check, replace with your robot's actual bounding box half-sizes.
                        const Eigen::Vector3d robot_half_sizes(0.0, 0.0, 0.0); 

                        // The effective "inflated" obstacle dimensions for the check
                        const Eigen::Vector3d minkowski_half_sizes = box_half_sizes + robot_half_sizes;

                        // Calculate the obstacle's predicted start position for this trajectory segment
                        const double dt = std::max(0.0, seg_start_time - obs.last_update_time.seconds());
                        const Eigen::Vector3d p_o0 = obs_pos + obs_vel * dt;

                        // Define the robot's trajectory segment in the obstacle's relative frame.
                        // This treats the obstacle as a stationary box at the origin.
                        const Eigen::Vector3d p_start_relative = p_r0 - p_o0;
                        const Eigen::Vector3d p_end_relative = p_r1 - (p_o0 + obs_vel * T);



                        // if (sweptBoxIntersection3D(p_r0, v_r, p_o0, obs_vel, w, h, d, T, obs.dimensions.rotation)) return obs;
                        if (fastLineAABBIntersection(p_start_relative, p_end_relative, Eigen::Vector3d::Zero(), minkowski_half_sizes)) {
                            recordCulprit(obs);
                            return obs; // COLLISION!
                        }
                    } else {
                        // if (lineIntersectsBox3D(p_r0, p_r1, obs_pos, w, h, d, obs.dimensions.rotation)) return obs;
                        if (fastLineAABBIntersection(p_r0, p_r1, obs_pos, box_half_sizes)) {
                            recordCulprit(obs);
                            return obs; // Collision detected!
                        }
                    }
                }
            }
            time_into_full_trajectory += T;
        }
        // auto end = std::chrono::steady_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        // std::cout << "Time taken for collision check: " << duration.count() << " micro s\n";
        return std::nullopt;
    }
    else { // The spatial part here is 2D
        auto get_xy = [](const Eigen::VectorXd& state) { return state.head<2>(); };
        auto get_time = [](const Eigen::VectorXd& state) { return state(state.size() - 1); };
        auto get_vxy = [](const Eigen::VectorXd& state) { return state.segment<2>(2); };

        double time_into_full_trajectory = 0.0;
        const int state_dim = trajectory.path_points[0].size();

        for (size_t i = 0; i < trajectory.path_points.size() - 1; ++i) {
            const Eigen::VectorXd& segment_start_state = trajectory.path_points[i];
            const Eigen::VectorXd& segment_end_state   = trajectory.path_points[i + 1];

            const double T_segment = get_time(segment_start_state) - get_time(segment_end_state);
            if (T_segment <= 1e-9) continue;
            
            const double global_time_at_segment_start = global_start_time + time_into_full_trajectory;
            /*
                Because the time duration (T_segment) of this sub-segment is so small, two things are true:
                The tiny curved path is almost indistinguishable from a straight line.
                The change in velocity over this tiny interval is almost zero.
                So maybe its faster to always use the else part!
            */ 
            if (state_dim == 5) {
            // if (false) {
                // ACCELERATION MODEL (5D): Approximate the CURVED path as one line 
                const Eigen::Vector2d p_r0_seg = get_xy(segment_start_state);
                const Eigen::Vector2d v_r0_seg = get_vxy(segment_start_state);
                const Eigen::Vector2d v_r1_seg = get_vxy(segment_end_state);
                const Eigen::Vector2d a_r_seg = (v_r1_seg - v_r0_seg) / T_segment;
                
                // Calculate the end point of the curved segment and the average velocity
                const Eigen::Vector2d p_r1_seg = p_r0_seg + v_r0_seg * T_segment + 0.5 * a_r_seg * T_segment * T_segment;
                const Eigen::Vector2d v_r_seg = (p_r1_seg - p_r0_seg) / T_segment;

                for (const auto& obs : obstacle_snapshot_) {
                    if (obs.type == Obstacle::CIRCLE) {
                        const double R = obs.dimensions.radius + inflation;
                        const double R_sq = R * R;
                        if (obs.is_dynamic) {
                            const double delta_t = std::max(0.0, global_time_at_segment_start - obs.last_update_time.seconds());
                            const Eigen::Vector2d p_o0 = obs.position + obs.velocity * delta_t;
                            const Eigen::Vector2d p_rel_start = p_r0_seg - p_o0;
                            const Eigen::Vector2d v_rel = v_r_seg - obs.velocity;
                            const double a = v_rel.dot(v_rel);
                            const double b = 2.0 * p_rel_start.dot(v_rel);
                            const double c = p_rel_start.dot(p_rel_start) - R_sq;
                            if (std::abs(a) < 1e-9) { 
                                if (c <= 0) { recordCulprit(obs); return obs; } 
                                continue; 
                            }
                            const double disc = b * b - 4 * a * c;
                            if (disc >= 0) {
                                const double t1 = (-b - std::sqrt(disc)) / (2.0 * a);
                                const double t2 = (-b + std::sqrt(disc)) / (2.0 * a);
                                if (std::max(0.0, t1) <= std::min(T_segment, t2)) {
                                    recordCulprit(obs);
                                    return obs;
                                }
                            }
                        } else {
                            if (distanceSqrdPointToSegment(obs.position, p_r0_seg, p_r1_seg) <= R_sq) {
                                recordCulprit(obs);
                                return obs;
                            }
                        }
                    } else if (obs.type == Obstacle::BOX) {
                        const double w = obs.dimensions.width + 2 * inflation;
                        const double h = obs.dimensions.height + 2 * inflation;
                        if (obs.is_dynamic) {
                            const double delta_t = std::max(0.0, global_time_at_segment_start - obs.last_update_time.seconds());
                            const Eigen::Vector2d p_o0 = obs.position + obs.velocity * delta_t;
                            // Since boxes don't rotate, we can call with 'consider_rotation' as false for max performance.
                            if (sweptBoxIntersection(p_r0_seg, v_r_seg, p_o0, obs.velocity, w, h, T_segment, obs.dimensions.rotation, false)) {
                                recordCulprit(obs);
                                return obs;
                            }
                        } else {
                            if (lineIntersectsRectangle(p_r0_seg, p_r1_seg, obs.position, w, h, obs.dimensions.rotation)) {
                                recordCulprit(obs);
                                return obs;
                            }
                        }
                    }
                }
            } else {
                // CONSTANT VELOCITY MODEL (Already a straight line)
                const Eigen::Vector2d p_r0 = get_xy(segment_start_state);
                const Eigen::Vector2d p_r1 = get_xy(segment_end_state);
                const Eigen::Vector2d v_r = (p_r1 - p_r0) / T_segment;

                for (const auto& obs : obstacle_snapshot_) {
                    if (obs.type == Obstacle::CIRCLE) {
                        const double R = obs.dimensions.radius + inflation;
                        const double R_sq = R * R;
                        if (obs.is_dynamic) {
                            const double delta_t = std::max(0.0, global_time_at_segment_start - obs.last_update_time.seconds());
                            const Eigen::Vector2d p_o0 = obs.position + obs.velocity * delta_t;
                            const Eigen::Vector2d p_rel_start = p_r0 - p_o0;
                            const Eigen::Vector2d v_rel = v_r - obs.velocity;
                            const double a = v_rel.dot(v_rel);
                            const double b = 2.0 * p_rel_start.dot(v_rel);
                            const double c = p_rel_start.dot(p_rel_start) - R_sq;
                            if (std::abs(a) < 1e-9) { 
                                if (c <= 0) { recordCulprit(obs); return obs; } 
                                continue; 
                            }
                            const double disc = b * b - 4 * a * c;
                            if (disc >= 0) {
                                const double sqrt_disc = std::sqrt(disc);
                                const double t1 = (-b - sqrt_disc) / (2.0 * a);
                                const double t2 = (-b + sqrt_disc) / (2.0 * a);
                                if (std::max(0.0, t1) <= std::min(T_segment, t2)) {
                                    recordCulprit(obs);
                                    return obs;
                                }
                            }
                        } else {
                            if (distanceSqrdPointToSegment(obs.position, p_r0, p_r1) <= R_sq) {
                                recordCulprit(obs);
                                return obs;
                            }
                        }
                    } else if (obs.type == Obstacle::BOX) {
                        const double w = obs.dimensions.width + 2 * inflation;
                        const double h = obs.dimensions.height + 2 * inflation;
                        if (obs.is_dynamic) {
                            const double delta_t = std::max(0.0, global_time_at_segment_start - obs.last_update_time.seconds());
                            const Eigen::Vector2d p_o0 = obs.position + obs.velocity * delta_t;
                            // Since boxes don't rotate, we can call with 'consider_rotation' as false for max performance.
                            if (sweptBoxIntersection(p_r0, v_r, p_o0, obs.velocity, w, h, T_segment, obs.dimensions.rotation, false)) {
                                recordCulprit(obs);
                                return obs;
                            }
                        } else {
                            if (lineIntersectsRectangle(p_r0, p_r1, obs.position, w, h, obs.dimensions.rotation)) {
                                recordCulprit(obs);
                                return obs;
                            }
                        }
                    }
                }
            }
            
            time_into_full_trajectory += T_segment;
        }

        return std::nullopt;
    }



}

// // For Absolute time
// std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacle(
//     const Trajectory& trajectory,
//     double global_start_time // The SimTime when the robot is at trajectory.path_points[0]
// ) const {
//     // Standard RRTX Helper: Records the threat for your red visualization
//     auto recordCulprit = [&](const Obstacle& obs) {
//         if (culprit_names_.find(obs.name) == culprit_names_.end()) {
//             culprit_names_.insert(obs.name);
//             culprit_cache_.push_back(obs);
//         }
//     };

//     if (trajectory.path_points.empty()) return std::nullopt;

//     const int state_dim = trajectory.path_points[0].size();
//     const int t_idx = state_dim - 1; // Time-To-Goal coordinate index
//     const double robot_ttg_at_start = trajectory.path_points[0](t_idx);

//     for (const auto& obs : obstacle_snapshot_) {
//         // --- 1. BROAD PHASE GATEKEEPER ---
//         double obs_bound = (obs.type == Obstacle::CIRCLE) ? 
//                             obs.dimensions.radius : 
//                             std::hypot(obs.dimensions.width / 2.0, obs.dimensions.height / 2.0);
        
//         double max_obs_move = obs.is_dynamic ? (obs.velocity.norm() * trajectory.total_duration) : 0.0;
//         double dist_to_envelope = (obs.position - trajectory.envelope_center.head<2>()).norm();

//         // Skip the entire obstacle if it can't physically reach the trajectory's spatial volume
//         if (dist_to_envelope > (trajectory.envelope_radius + obs_bound + inflation + max_obs_move + 1.0)) {
//             continue; 
//         }

//         // --- 2. NARROW PHASE: SPACE-TIME APPOINTMENTS ---
//         for (const auto& point : trajectory.path_points) {
//             // A. CALCULATE APPOINTMENT SimTime
//             // Arrival_SimTime = (SimTime at start of edge) + (Time elapsed since start of edge)
//             double abs_sim_time = global_start_time + (robot_ttg_at_start - point(t_idx));

//             // B. PROJECT THE OBSTACLE TO THAT INSTANT
//             Eigen::Vector2d obs_pos = obs.position;
//             if (obs.is_dynamic) {
//                 double dt = abs_sim_time - obs.last_update_time.seconds();
//                 obs_pos += obs.velocity * dt;
//             }

//             // C. POINT-IN-SHAPE CHECK (O(1) Math)
//             bool collision = false;
//             if (obs.type == Obstacle::CIRCLE) {
//                 double d_sq = (point.head<2>() - obs_pos).squaredNorm();
//                 if (d_sq < std::pow(obs.dimensions.radius + inflation, 2)) collision = true;
//             } else { // BOX
//                 collision = isPointInOrientedBox(point.head<2>(), obs_pos, 
//                                                obs.dimensions.width + 2*inflation, 
//                                                obs.dimensions.height + 2*inflation, 
//                                                obs.dimensions.rotation);
//             }

//             if (collision) {
//                 recordCulprit(obs); // Save for red visualization!
//                 return obs; 
//             }
//         }
//     }
//     return std::nullopt;
// }



// FCL IMPLEMENTATION --> for general purpose collision check with any mesh --> its pretty demanding!
std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacleFCL(
    const Trajectory& trajectory,
    double global_start_time
) const {
    // A trajectory needs at least two points to form a segment.
    if (trajectory.path_points.size() < 2) {
        return std::nullopt;
    }

    auto recordCulprit = [&](const Obstacle& obs) {
        if (culprit_names_.find(obs.name) == culprit_names_.end()) {
            culprit_names_.insert(obs.name);
            culprit_cache_.push_back(obs);
        }
    };

    // Create the robot's collision geometry (a sphere representing the safety bubble).
    auto robot_geom = std::make_shared<fcl::Sphered>(inflation);

    // Helper lambdas to get position and time from a state vector.
    auto get_xy = [](const Eigen::VectorXd& state) { return state.head<2>(); };
    auto get_time = [](const Eigen::VectorXd& state) { return state(state.size() - 1); };

    // // --- DEBUG: Print initial conditions for this check ---
    // std::cout << std::fixed << std::setprecision(3);
    // std::cout << "\n--- [FCL CHECK START] ---\n"
    //           << "Robot Radius (inflation): " << inflation 
    //           << ", Global Start Time: " << global_start_time
    //           << ", Trajectory Points: " << trajectory.path_points.size() << std::endl;

    // --- Loop through each LINEARIZED segment from the trajectory ---
    for (size_t i = 0; i < trajectory.path_points.size() - 1; ++i) {
        const Eigen::VectorXd& segment_start_state = trajectory.path_points[i];
        const Eigen::VectorXd& segment_end_state   = trajectory.path_points[i + 1];

        const double T_segment = get_time(segment_start_state) - get_time(segment_end_state);
        if (T_segment <= 1e-9) {
            continue;
        }
        
        const double t_arrival_predicted = global_start_time + get_time(trajectory.path_points.front());
        const double global_time_at_segment_start = t_arrival_predicted - get_time(segment_start_state);

        // // --- DEBUG: Print segment info ---
        // std::cout << "  [Segment " << i << "] T_start: " << global_time_at_segment_start
        //           << ", Duration: " << T_segment << "\n"
        //           << "    Robot Start: (" << get_xy(segment_start_state).x() << ", " << get_xy(segment_start_state).y() << ")\n"
        //           << "    Robot End:   (" << get_xy(segment_end_state).x() << ", " << get_xy(segment_end_state).y() << ")" << std::endl;

        const fcl::Transform3d tf_robot_start(fcl::Translation3d(get_xy(segment_start_state).x(), get_xy(segment_start_state).y(), 0));
        const fcl::Transform3d tf_robot_end(fcl::Translation3d(get_xy(segment_end_state).x(), get_xy(segment_end_state).y(), 0));

        for (const auto& obs : obstacle_snapshot_) {
            auto cache_it = fcl_cache_.find(obs.name);
            if (cache_it == fcl_cache_.end()) {
                continue;
            }

            const fcl::CollisionObjectd& obs_co = cache_it->second;
            auto obs_geom_ptr = obs_co.collisionGeometry();
            
            // --- DEBUG: Print obstacle info ---
            double obs_radius = 0.0;
            if (obs.type == Obstacle::CIRCLE) obs_radius = obs.dimensions.radius;
            else obs_radius = std::hypot(obs.dimensions.width/2.0, obs.dimensions.height/2.0);

            // std::cout << "    -> Checking Obstacle '" << obs.name << "' (Radius: " << obs_radius << ")\n";

            double delta_t = std::max(0.0, global_time_at_segment_start - obs.last_update_time.seconds());
            Eigen::Vector2d obs_pos_start = obs.position + obs.velocity * delta_t;
            Eigen::Vector2d obs_pos_end = obs_pos_start + obs.velocity * T_segment;
            
            // // --- DEBUG: Print predicted positions ---
            // std::cout << "      Obs Snapshot Pos: (" << obs.position.x() << ", " << obs.position.y() << ") at t=" << obs.last_update_time.seconds() << "\n"
            //           << "      delta_t: " << delta_t << "\n"
            //           << "      Obs Predicted Start: (" << obs_pos_start.x() << ", " << obs_pos_start.y() << ")\n"
            //           << "      Obs Predicted End:   (" << obs_pos_end.x() << ", " << obs_pos_end.y() << ")" << std::endl;

            fcl::Transform3d tf_obs_start_mutable = fcl::Transform3d::Identity();
            tf_obs_start_mutable.translation() = fcl::Vector3d(obs_pos_start.x(), obs_pos_start.y(), 0);
            tf_obs_start_mutable.linear() = Eigen::AngleAxisd(obs.dimensions.rotation, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            const fcl::Transform3d tf_obs_start = tf_obs_start_mutable;

            fcl::Transform3d tf_obs_end_mutable = tf_obs_start;
            tf_obs_end_mutable.translation() = fcl::Vector3d(obs_pos_end.x(), obs_pos_end.y(), 0);
            const fcl::Transform3d tf_obs_end = tf_obs_end_mutable;
            
            fcl::ContinuousCollisionRequestd request;
            request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
            fcl::ContinuousCollisionResultd result;
            
            fcl::continuousCollide(robot_geom.get(), tf_robot_start, tf_robot_end,
                                   obs_geom_ptr.get(), tf_obs_start, tf_obs_end, request, result);

            if (result.is_collide) {
                // // --- DEBUG: Announce collision ---
                // std::cout << "      !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
                //           << "      !!!!!! FCL RESULT: COLLISION DETECTED !!!!!!\n"
                //           << "      !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
                recordCulprit(obs);
                return obs;
            } else {
                // // --- DEBUG: Announce no collision ---
                // std::cout << "      FCL RESULT: NO COLLISION" << std::endl;
            }
        }
    }

    // --- DEBUG: Announce end of check ---
    // std::cout << "--- [FCL CHECK END] --- No collisions found.\n" << std::endl;
    return std::nullopt;
}


// // Bullet with world object being process in the function it self!
// std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacleBullet(
//     const Trajectory& trajectory,
//     double global_start_time
// ) const {
//     // --- Initial Setup & Validation ---
//     if (trajectory.path_points.size() < 2) {
//         return std::nullopt;
//     }

//     // Lazy initialization of the Bullet world. This ensures it's only created once.
//     if (!bullet_world_) {
//         bullet_collision_config_ = std::make_unique<btDefaultCollisionConfiguration>();
//         bullet_dispatcher_ = std::make_unique<btCollisionDispatcher>(bullet_collision_config_.get());
//         bullet_broadphase_ = std::make_unique<btDbvtBroadphase>();
//         bullet_world_ = std::make_unique<btCollisionWorld>(
//             bullet_dispatcher_.get(), bullet_broadphase_.get(), bullet_collision_config_.get()
//         );
//     }

//     // Helper lambdas for cleaner code
//     auto get_xy = [](const Eigen::VectorXd& state) { return state.head<2>(); };
//     auto get_time = [](const Eigen::VectorXd& state) { return state(state.size() - 1); };
//     auto get_vxy = [](const Eigen::VectorXd& state) { return state.segment<2>(2); };
//     auto to_btVector3 = [](const Eigen::Vector2d& vec) { return btVector3(vec.x(), vec.y(), 0); };

//     double time_into_full_trajectory = 0.0;
//     const int state_dim = trajectory.path_points[0].size();

//     // The robot is represented as a sphere with the inflation radius for all checks.
//     btSphereShape robot_shape(inflation);

//     // --- Iterate Through Trajectory Segments ---
//     for (size_t i = 0; i < trajectory.path_points.size() - 1; ++i) {
//         const Eigen::VectorXd& start_state = trajectory.path_points[i];
//         const Eigen::VectorXd& end_state = trajectory.path_points[i + 1];

//         const double T_segment = get_time(start_state) - get_time(end_state);
//         if (T_segment <= 1e-9) continue;

//         const double global_time_at_segment_start = global_start_time + time_into_full_trajectory;

//         // Calculate robot's start and end positions for this segment
//         Eigen::Vector2d p_r0, p_r1;
//         if (state_dim == 5) { // 5D state space (x, y, vx, vy, t) - accounts for acceleration
//             const Eigen::Vector2d v_r0 = get_vxy(start_state);
//             const Eigen::Vector2d v_r1 = get_vxy(end_state);
//             const Eigen::Vector2d a_r = (v_r1 - v_r0) / T_segment;
//             p_r0 = get_xy(start_state);
//             p_r1 = p_r0 + v_r0 * T_segment + 0.5 * a_r * T_segment * T_segment;
//         } else { // 3D or 4D state space (constant velocity)
//             p_r0 = get_xy(start_state);
//             p_r1 = get_xy(end_state);
//         }

//         // Define robot's start and end transforms
//         btTransform robot_tf_start, robot_tf_end;
//         robot_tf_start.setIdentity();
//         robot_tf_start.setOrigin(to_btVector3(p_r0));
//         robot_tf_end.setIdentity();
//         robot_tf_end.setOrigin(to_btVector3(p_r1));

//         // --- Check Against Each Obstacle ---
//         for (const auto& obs : obstacle_snapshot_) {
//             // Create the obstacle's collision shape
//             std::unique_ptr<btConvexShape> obs_shape;
//             if (obs.type == Obstacle::BOX) {
//                 obs_shape = std::make_unique<btBoxShape>(btVector3(obs.dimensions.width / 2.0, obs.dimensions.height / 2.0, 1.0));
//             } else { // CIRCLE
//                 obs_shape = std::make_unique<btSphereShape>(obs.dimensions.radius);
//             }

//             // Predict the obstacle's start and end transforms for the segment
//             btTransform obs_tf_start, obs_tf_end;
//             obs_tf_start.setIdentity();
//             obs_tf_end.setIdentity();

//             Eigen::Vector2d obs_pos_start = obs.position;
//             if (obs.is_dynamic) {
//                 double delta_t = std::max(0.0, global_time_at_segment_start - obs.last_update_time.seconds());
//                 obs_pos_start = obs.position + obs.velocity * delta_t;
//                 Eigen::Vector2d obs_pos_end = obs_pos_start + obs.velocity * T_segment;
//                 obs_tf_end.setOrigin(to_btVector3(obs_pos_end));
//             } else {
//                 obs_tf_end.setOrigin(to_btVector3(obs_pos_start)); // Static obstacle doesn't move
//             }
//             obs_tf_start.setOrigin(to_btVector3(obs_pos_start));
            
//             // Apply rotation for boxes
//             if(obs.type == Obstacle::BOX) {
//                 btQuaternion q;
//                 q.setEuler(obs.dimensions.rotation, 0, 0); // Assuming yaw is the rotation axis
//                 obs_tf_start.setRotation(q);
//                 obs_tf_end.setRotation(q); // Assuming non-rotating obstacles
//             }

//             // --- Perform Relative Motion Sweep Test (THE CORE FIX) ---

//             // Create a temporary collision object for the obstacle and add it to the world
//             btCollisionObject obs_co;
//             obs_co.setCollisionShape(obs_shape.get());
//             obs_co.setWorldTransform(obs_tf_start);
//             bullet_world_->addCollisionObject(&obs_co);

//             // Calculate the robot's end transform *relative* to the obstacle's motion
//             btTransform robot_tf_end_relative = robot_tf_end;
//             btVector3 obstacle_displacement = obs_tf_end.getOrigin() - obs_tf_start.getOrigin();
//             robot_tf_end_relative.getOrigin() -= obstacle_displacement;

//             // Setup the callback and perform the sweep test with the correct 5 arguments
//             btCollisionWorld::ClosestConvexResultCallback result_callback(btVector3(0,0,0), btVector3(0,0,0));
            
//             bullet_world_->convexSweepTest(
//                 &robot_shape,
//                 robot_tf_start,
//                 robot_tf_end_relative, // Use the modified relative transform
//                 result_callback,
//                 bullet_world_->getDispatchInfo().m_allowedCcdPenetration
//             );

//             // Clean up by removing the temporary object from the world
//             bullet_world_->removeCollisionObject(&obs_co);

//             if (result_callback.hasHit()) {
//                 return obs; // Collision detected
//             }
//         }
//         time_into_full_trajectory += T_segment;
//     }

//     return std::nullopt; // No collision found
// }

std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacleBullet(
    const Trajectory& trajectory,
    double global_start_time
) const {

    // --- Initial Setup & Validation ---
    if (trajectory.path_points.size() < 2) {
        return std::nullopt;
    }

    // Lazy initialization of the Bullet world.
    if (!bullet_world_) {
        bullet_collision_config_ = std::make_unique<btDefaultCollisionConfiguration>();
        bullet_dispatcher_ = std::make_unique<btCollisionDispatcher>(bullet_collision_config_.get());
        bullet_broadphase_ = std::make_unique<btDbvtBroadphase>();
        bullet_world_ = std::make_unique<btCollisionWorld>(
            bullet_dispatcher_.get(), bullet_broadphase_.get(), bullet_collision_config_.get()
        );
    }



    if(spatial_dim_==3){
        // --- NEW: 3D Helper lambdas ---
        auto get_xyz = [](const Eigen::VectorXd& state) { return state.head<3>(); };
        auto get_time = [](const Eigen::VectorXd& state) { return state.tail<1>()[0]; };
        auto to_btVector3_3d = [](const Eigen::Vector3d& vec) { return btVector3(vec.x(), vec.y(), vec.z()); };

        double time_into_full_trajectory = 0.0;
        btSphereShape robot_shape(inflation); // Robot is a sphere in 3D

        // Re-use the callback object for efficiency
        btCollisionWorld::ClosestConvexResultCallback result_callback(btVector3(0,0,0), btVector3(0,0,0));

        // --- Iterate Through 3D Trajectory Segments ---
        for (size_t i = 0; i < trajectory.path_points.size() - 1; ++i) {
            const Eigen::VectorXd& start_state = trajectory.path_points[i];
            const Eigen::VectorXd& end_state = trajectory.path_points[i + 1];

            const double T_segment = get_time(start_state) - get_time(end_state);
            if (T_segment <= 1e-9) continue;

            const double global_time_at_segment_start = global_start_time + time_into_full_trajectory;

            // Get robot's start and end positions in 3D
            const Eigen::Vector3d p_r0 = get_xyz(start_state);
            const Eigen::Vector3d p_r1 = get_xyz(end_state);

            // Define robot's start and end transforms in 3D
            btTransform robot_tf_start, robot_tf_end;
            robot_tf_start.setIdentity();
            robot_tf_start.setOrigin(to_btVector3_3d(p_r0));
            robot_tf_end.setIdentity();
            robot_tf_end.setOrigin(to_btVector3_3d(p_r1));

            // --- Check Against Each Obstacle in 3D ---
            for (const auto& obs : obstacle_snapshot_) {
                auto shape_it = bullet_shape_cache_.find(obs.name);
                if (shape_it == bullet_shape_cache_.end()) continue; // Shape not found in cache
                btConvexShape* obs_shape = shape_it->second.get();

                // Predict the obstacle's 3D start and end transforms
                btTransform obs_tf_start, obs_tf_end;
                obs_tf_start.setIdentity();
                obs_tf_end.setIdentity();

                Eigen::Vector3d obs_pos(obs.position.x(), obs.position.y(), obs.z);
                Eigen::Vector3d obs_vel(obs.velocity.x(), obs.velocity.y(), 0.0); // Assume 2D obstacle velocity

                Eigen::Vector3d obs_pos_start = obs_pos;
                if (obs.is_dynamic) {
                    double delta_t = std::max(0.0, global_time_at_segment_start - obs.last_update_time.seconds());
                    obs_pos_start = obs_pos + obs_vel * delta_t;
                    Eigen::Vector3d obs_pos_end = obs_pos_start + obs_vel * T_segment;
                    obs_tf_end.setOrigin(to_btVector3_3d(obs_pos_end));
                } else {
                    obs_tf_end.setOrigin(to_btVector3_3d(obs_pos_start)); // Static obstacle does not move
                }
                obs_tf_start.setOrigin(to_btVector3_3d(obs_pos_start));
                
                if(obs.type == Obstacle::BOX) {
                    btQuaternion q;
                    q.setEuler(obs.dimensions.rotation, 0, 0); // Yaw, Pitch, Roll
                    obs_tf_start.setRotation(q);
                    obs_tf_end.setRotation(q);
                }

                btCollisionObject obs_co;
                obs_co.setCollisionShape(obs_shape);
                obs_co.setWorldTransform(obs_tf_start);
                bullet_world_->addCollisionObject(&obs_co);

                // Calculate robot's end transform relative to the obstacle's motion
                btTransform robot_tf_end_relative = robot_tf_end;
                btVector3 obstacle_displacement = obs_tf_end.getOrigin() - obs_tf_start.getOrigin();
                robot_tf_end_relative.getOrigin() -= obstacle_displacement;

                // Reset callback state and perform the sweep test
                result_callback.m_closestHitFraction = 1.0f;
                result_callback.m_hitCollisionObject = nullptr;
                bullet_world_->convexSweepTest(&robot_shape, robot_tf_start, robot_tf_end_relative, result_callback, 0.0f);

                bullet_world_->removeCollisionObject(&obs_co);

                if (result_callback.hasHit()) {
                    // recordCulprit(obs);
                    if (culprit_names_.find(obs.name) == culprit_names_.end()) {
                        culprit_names_.insert(obs.name);
                        culprit_cache_.push_back(obs);
                    }
                    return obs; // Collision detected!
                }
            }
            time_into_full_trajectory += T_segment;
        }

        return std::nullopt; // Trajectory is safe
    }
    else{
        // Helper lambdas for cleaner code
        auto get_xy = [](const Eigen::VectorXd& state) { return state.head<2>(); };
        auto get_time = [](const Eigen::VectorXd& state) { return state(state.size() - 1); };
        auto get_vxy = [](const Eigen::VectorXd& state) { return state.segment<2>(2); };
        auto to_btVector3 = [](const Eigen::Vector2d& vec) { return btVector3(vec.x(), vec.y(), 0); };

        double time_into_full_trajectory = 0.0;
        const int state_dim = trajectory.path_points[0].size();
        btSphereShape robot_shape(inflation);

        // OPTIMIZATION: Create the callback object once and reuse it.
        btCollisionWorld::ClosestConvexResultCallback result_callback(btVector3(0,0,0), btVector3(0,0,0));

        // 2. --- Iterate Through Trajectory Segments ---
        for (size_t i = 0; i < trajectory.path_points.size() - 1; ++i) {
            const Eigen::VectorXd& start_state = trajectory.path_points[i];
            const Eigen::VectorXd& end_state = trajectory.path_points[i + 1];

            const double T_segment = get_time(start_state) - get_time(end_state);
            if (T_segment <= 1e-9) continue;

            const double global_time_at_segment_start = global_start_time + time_into_full_trajectory;

            // Calculate robot's start and end positions for this segment
            Eigen::Vector2d p_r0, p_r1;
            if (state_dim == 5) { // 5D state space
                const Eigen::Vector2d v_r0 = get_vxy(start_state);
                const Eigen::Vector2d v_r1 = get_vxy(end_state);
                const Eigen::Vector2d a_r = (v_r1 - v_r0) / T_segment;
                p_r0 = get_xy(start_state);
                p_r1 = p_r0 + v_r0 * T_segment + 0.5 * a_r * T_segment * T_segment;
            } else { // 3D/4D state space
                p_r0 = get_xy(start_state);
                p_r1 = get_xy(end_state);
            }

            btTransform robot_tf_start, robot_tf_end;
            robot_tf_start.setIdentity();
            robot_tf_start.setOrigin(to_btVector3(p_r0));
            robot_tf_end.setIdentity();
            robot_tf_end.setOrigin(to_btVector3(p_r1));

            // --- Check Against Each Obstacle ---
            for (const auto& obs : obstacle_snapshot_) {
                // OPTIMIZATION: Get the pre-cached shape instead of creating a new one.
                auto shape_it = bullet_shape_cache_.find(obs.name);
                if (shape_it == bullet_shape_cache_.end()) continue; // Shape not found
                btConvexShape* obs_shape = shape_it->second.get();

                // Predict the obstacle's start and end transforms
                btTransform obs_tf_start, obs_tf_end;
                obs_tf_start.setIdentity();
                obs_tf_end.setIdentity();

                Eigen::Vector2d obs_pos_start = obs.position;
                if (obs.is_dynamic) {
                    double delta_t = std::max(0.0, global_time_at_segment_start - obs.last_update_time.seconds());
                    obs_pos_start = obs.position + obs.velocity * delta_t;
                    Eigen::Vector2d obs_pos_end = obs_pos_start + obs.velocity * T_segment;
                    obs_tf_end.setOrigin(to_btVector3(obs_pos_end));
                } else {
                    obs_tf_end.setOrigin(to_btVector3(obs_pos_start));
                }
                obs_tf_start.setOrigin(to_btVector3(obs_pos_start));
                
                if(obs.type == Obstacle::BOX) {
                    btQuaternion q;
                    q.setEuler(obs.dimensions.rotation, 0, 0);
                    obs_tf_start.setRotation(q);
                    obs_tf_end.setRotation(q);
                }

                // Create a temporary collision object on the stack
                btCollisionObject obs_co;
                obs_co.setCollisionShape(obs_shape);
                obs_co.setWorldTransform(obs_tf_start);
                bullet_world_->addCollisionObject(&obs_co);

                btTransform robot_tf_end_relative = robot_tf_end;
                btVector3 obstacle_displacement = obs_tf_end.getOrigin() - obs_tf_start.getOrigin();
                robot_tf_end_relative.getOrigin() -= obstacle_displacement;

                // OPTIMIZATION: Reset the state of the reused callback object.
                result_callback.m_closestHitFraction = 1.0f;
                result_callback.m_hitCollisionObject = nullptr;
                
                bullet_world_->convexSweepTest(&robot_shape, robot_tf_start, robot_tf_end_relative, result_callback, 0.0f);

                bullet_world_->removeCollisionObject(&obs_co);

                if (result_callback.hasHit()) {
                    // recordCulprit(obs);
                    if (culprit_names_.find(obs.name) == culprit_names_.end()) {
                        culprit_names_.insert(obs.name);
                        culprit_cache_.push_back(obs);
                    }
                    return obs;
                }
            }
            time_into_full_trajectory += T_segment;
        }

        return std::nullopt;
    }

}

// std::optional<Obstacle> GazeboObstacleChecker::getCollidingObstacleBullet(
//     const Trajectory& trajectory,
//     double global_start_time
// ) const {
//     // --- Initial Setup & Validation ---
//     if (trajectory.path_points.size() < 2 || trajectory.coeffs_per_axis.empty()) {
//         return std::nullopt; // Not a valid trajectory to check
//     }

//     // Lazy initialization of the Bullet world.
//     if (!bullet_world_) {
//         bullet_collision_config_ = std::make_unique<btDefaultCollisionConfiguration>();
//         bullet_dispatcher_ = std::make_unique<btCollisionDispatcher>(bullet_collision_config_.get());
//         bullet_broadphase_ = std::make_unique<btDbvtBroadphase>();
//         bullet_world_ = std::make_unique<btCollisionWorld>(
//             bullet_dispatcher_.get(), bullet_broadphase_.get(), bullet_collision_config_.get()
//         );
//     }
    
//     // Create the robot's collision shape once with its effective radius.
//     btSphereShape robot_shape(inflation);

//     // Re-use the callback object for all sweep tests to improve efficiency.
//     btCollisionWorld::ClosestConvexResultCallback result_callback(btVector3(0,0,0), btVector3(0,0,0));

//     // --- The "basis" function, copied from MinSnapROS2Manager for evaluating the polynomial ---
//     auto basis = [&](int deriv, double tau, int num_coeffs) -> Eigen::RowVectorXd {
//         Eigen::RowVectorXd r = Eigen::RowVectorXd::Zero(num_coeffs);
//         for (int i = deriv; i < num_coeffs; ++i) {
//             double c = 1.0; for (int k = 0; k < deriv; ++k) c *= (i - k);
//             r(i) = c * std::pow(tau, i - deriv);
//         }
//         return r;
//     };
    
//     // --- Main Loop: Check each segment of the trajectory ---
//     // A full trajectory from the planner is composed of one or more of these segments (e.g., A->S, S->B).
//     const double T_segment = trajectory.time_duration;
//     if (T_segment <= 1e-9) {
//         return std::nullopt; // Cannot check a zero-duration segment
//     }

//     const double global_time_at_segment_start = global_start_time;

//     // --- High-Resolution Sub-division of the Curved Path ---
//     const int num_subdivisions = 10; // Increase for more accuracy, decrease for more speed. 10 is a good start.
//     const int num_coeffs = trajectory.coeffs_per_axis[0].size();
    
//     Eigen::Vector3d p_sub_start_3d = trajectory.path_points.front().head<3>();

//     for (int j = 1; j <= num_subdivisions; ++j) {
//         double tau = static_cast<double>(j) / num_subdivisions;

//         // Evaluate the polynomial at time `tau` to get the true 3D position at the end of the sub-segment.
//         Eigen::Vector3d p_sub_end_3d;
//         p_sub_end_3d.x() = (trajectory.coeffs_per_axis[0].transpose() * basis(0, tau, num_coeffs).transpose())(0);
//         p_sub_end_3d.y() = (trajectory.coeffs_per_axis[1].transpose() * basis(0, tau, num_coeffs).transpose())(0);
//         p_sub_end_3d.z() = (trajectory.coeffs_per_axis[2].transpose() * basis(0, tau, num_coeffs).transpose())(0);

//         // Define the robot's motion for this short, near-linear sub-segment.
//         btTransform robot_tf_start, robot_tf_end;
//         robot_tf_start.setIdentity();
//         robot_tf_start.setOrigin(btVector3(p_sub_start_3d.x(), p_sub_start_3d.y(), p_sub_start_3d.z()));
//         robot_tf_end.setIdentity();
//         robot_tf_end.setOrigin(btVector3(p_sub_end_3d.x(), p_sub_end_3d.y(), p_sub_end_3d.z()));

//         // --- Check this sub-segment against every obstacle ---
//         for (const auto& obs : obstacle_snapshot_) {
//             auto shape_it = bullet_shape_cache_.find(obs.name);
//             if (shape_it == bullet_shape_cache_.end()) continue;
//             btConvexShape* obs_shape = shape_it->second.get();

//             // Predict obstacle motion over the DURATION of the sub-segment.
//             const double T_sub_segment = T_segment / num_subdivisions;
//             const double time_at_sub_segment_start = global_time_at_segment_start + (T_segment * (j - 1) / num_subdivisions);

//             btTransform obs_tf_start, obs_tf_end;
//             obs_tf_start.setIdentity();
//             obs_tf_end.setIdentity();

//             Eigen::Vector3d obs_pos(obs.position.x(), obs.position.y(), obs.z);
//             Eigen::Vector3d obs_vel(obs.velocity.x(), obs.velocity.y(), 0.0); // Assumes constant Z

//             Eigen::Vector3d obs_pos_start = obs_pos;
//             if (obs.is_dynamic) {
//                 double delta_t = std::max(0.0, time_at_sub_segment_start - obs.last_update_time.seconds());
//                 obs_pos_start = obs_pos + obs_vel * delta_t;
//                 Eigen::Vector3d obs_pos_end = obs_pos_start + obs_vel * T_sub_segment;
//                 obs_tf_end.setOrigin(btVector3(obs_pos_end.x(), obs_pos_end.y(), obs_pos_end.z()));
//             } else {
//                 obs_tf_end.setOrigin(btVector3(obs_pos_start.x(), obs_pos_start.y(), obs_pos_start.z()));
//             }
//             obs_tf_start.setOrigin(btVector3(obs_pos_start.x(), obs_pos_start.y(), obs_pos_start.z()));
            
//             // --- Perform the Relative Motion Sweep Test ---
//             btCollisionObject obs_co;
//             obs_co.setCollisionShape(obs_shape);
//             obs_co.setWorldTransform(obs_tf_start);
//             bullet_world_->addCollisionObject(&obs_co);

//             btTransform robot_tf_end_relative = robot_tf_end;
//             btVector3 obstacle_displacement = obs_tf_end.getOrigin() - obs_tf_start.getOrigin();
//             robot_tf_end_relative.getOrigin() -= obstacle_displacement;

//             result_callback.m_closestHitFraction = 1.0f;
//             result_callback.m_hitCollisionObject = nullptr;
            
//             bullet_world_->convexSweepTest(&robot_shape, robot_tf_start, robot_tf_end_relative, result_callback, 0.0f);

//             bullet_world_->removeCollisionObject(&obs_co);

//             if (result_callback.hasHit()) {
//                 return obs; // COLLISION DETECTED!
//             }
//         }
        
//         // The end of this sub-segment is the start of the next one.
//         p_sub_start_3d = p_sub_end_3d;
//     }

//     return std::nullopt; // Trajectory is safe
// }




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

//             // --- Static Obstacle Check ---
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
    // std::lock_guard<std::mutex> lock(data_mutex_);
    return robot_position_;
}

Eigen::VectorXd GazeboObstacleChecker::getRobotOrientation() const {
    // std::lock_guard<std::mutex> lock(data_mutex_);
    return robot_orientation_;
}

const ObstacleVector& GazeboObstacleChecker::getObstaclePositions() const {
    // std::lock_guard<std::mutex> lock(data_mutex_);
    return obstacle_positions_;
}

Eigen::VectorXd GazeboObstacleChecker::getRobotEulerAngles() const {
    // std::lock_guard<std::mutex> lock(data_mutex_);
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
//             // FIRST, perform velocity estimation on the 'obstacle' object.
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
    // std::lock_guard<std::mutex> lock(snapshot_mutex_);
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
        double position_z = pose.position().z();


        if (name == robot_model_name_) continue;

        bool is_cylinder = name.find("cylinder") != std::string::npos;
        bool is_box = name.find("box") != std::string::npos;
        bool is_static = name.find("static_") != std::string::npos;
        bool is_moving = name.find("moving_") != std::string::npos;

        if (!is_cylinder && !is_box) continue;

        auto info_it = obstacle_info_.find(name);

        // Create obstacle object
        Obstacle obstacle;
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
        obstacle.name = name;
        obstacle.is_dynamic = is_moving;

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
                    
                    // --- Use the Factory to create the filter ---
                    KalmanFilter new_filter = KalmanFilterFactory::createFilter(kf_model_type_);

                    // --- Initialize the state with the correct size ---
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

                    // Predict step: Estimate where the filter thinks the obstacle should be.
                    filter_it->second.predict(dt);

                    // Update step: Correct the prediction with the new measurement.
                    Eigen::VectorXd measurement(2);
                    measurement << obstacle.position.x(), obstacle.position.y();
                    filter_it->second.update(measurement);

                    // Get the smoothed state from the filter to use in planning.
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


// The new lightweight callback implementation
void GazeboObstacleChecker::lightweightPoseCallback(const gz::msgs::Pose_V& msg) {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    latest_pose_msg_ = msg; // Overwrite the previous message
    new_pose_msg_available_ = true;
}


// // The new processing function 
// void GazeboObstacleChecker::processLatestPoseInfo() {
//     gz::msgs::Pose_V msg;
//     {
//         std::lock_guard<std::mutex> lock(snapshot_mutex_);
//         if (!new_pose_msg_available_) return;
//         msg = latest_pose_msg_;
//         new_pose_msg_available_ = false;
//     }

//     rclcpp::Time now = clock_->now();
//     obstacle_positions_.clear(); // Clear for Rviz sync

//     for (int i = 0; i < msg.pose_size(); ++i) {
//         const auto& pose = msg.pose(i);
//         const std::string name = pose.name();
        
//         // 1. Handle Robot
//         if (name == robot_model_name_) {
//             robot_position_ = Eigen::Vector2d(pose.position().x(), pose.position().y());
//             continue;
//         }

//         // 2. Find Obstacle Info (SDF Ground Truth)
//         auto info_it = obstacle_info_.find(name);
//         if (info_it == obstacle_info_.end()) continue;

//         const auto& info = info_it->second;
//         Obstacle& ob = obstacle_positions_map_[name]; // Get or create
        
//         Eigen::Vector2d new_pos(pose.position().x(), pose.position().y());
//         ob.previous_position = ob.position; 

//         // 3. Populate Static/Ground Truth Data (Every time, to be safe)
//         ob.name = name;
//         ob.is_dynamic = info.is_dynamic;
//         ob.has_ground_truth = true;
//         ob.speed_scalar = info.speed;        // e.g., 8.0
//         ob.motion_limit = info.amplitude;    // e.g., 40.0
//         ob.initial_origin = info.initial_pose.head<2>();
        
//         // Normalize direction from SDF (e.g., (1,0) or (0,1))
//         if (info.direction.head<2>().norm() > 1e-6) {
//              ob.motion_axis = info.direction.head<2>().normalized();
//         } else {
//              ob.motion_axis = Eigen::Vector2d::UnitX(); // Fallback
//         }

//        // 4. Robust Velocity Calculation
//         if (ob.is_dynamic) {
//             // CASE A: FIRST FRAME (Initialization)
//             // We ignore Gazebo's velocity on the first frame because it is often unstable (0 or jitter).
//             // We trust the SDF Ground Truth (speed_scalar + motion_axis).
//             if (ob.last_update_time.nanoseconds() == 0) {
//                 // Determine direction: 
//                 // If SDF direction is (1,0), we assume positive. If (-1,0), negative.
//                 // We normalize the SDF direction to get the sign.
//                 double sdf_dir_sign = info.direction.head<2>().normalized().x(); 
//                 // Note: For Y-axis movement, this logic still holds because normalized vector preserves sign.
                
//                 // To be safe, we project the SDF direction onto the motion axis (which is normalized)
//                 sdf_dir_sign = info.direction.head<2>().dot(ob.motion_axis);

//                 ob.velocity = ob.motion_axis * (ob.speed_scalar * sdf_dir_sign);
//             }
//             // CASE B: SUBSEQUENT FRAMES (Physics)
//             else {
//                 Eigen::Vector2d displacement = new_pos - ob.position;
                
//                 // Only update direction if we moved enough to be sure (filter jitter)
//                 if (displacement.norm() > 1e-4) {
//                     // Project displacement onto the motion axis
//                     double dot = displacement.dot(ob.motion_axis);
//                     double dir_sign = (dot >= 0.0) ? 1.0 : -1.0;
                    
//                     // FORCE EXACT VELOCITY: Axis * (SDF_Speed * Sign)
//                     ob.velocity = ob.motion_axis * (ob.speed_scalar * dir_sign);
//                 }
//                 // If displacement is tiny, keep previous velocity (Inertia)
//             }
//         } else {
//             ob.velocity = Eigen::Vector2d::Zero();
//         }

//         // 5. Update State
//         ob.position = new_pos;
//         ob.last_update_time = now;

//         // 6. Geometry Sync (Visuals & Collision)
//         if (info.type == ObstacleInfo::CYLINDER) {
//             ob.type = Obstacle::CIRCLE;
//             ob.dimensions.radius = info.radius;
//         } else {
//             ob.type = Obstacle::BOX;
//             ob.dimensions.width = info.width;
//             ob.dimensions.height = info.height;
            
//             // Quaternion to Yaw
//             Eigen::Quaterniond q(
//                 pose.orientation().w(),
//                 pose.orientation().x(),
//                 pose.orientation().y(),
//                 pose.orientation().z()
//             );
//             // Extract Yaw from Eigen Quaternion
//             auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2); // ZYX order, Z is index 0 or 2 depending on convention
//             // Simpler 2D planar projection for Yaw:
//             double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 
//                                     1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
//             ob.dimensions.rotation = yaw;
//         }

//         // // 7. Logging (Throttled)
//         // if (ob.is_dynamic) {
//         //     RCLCPP_INFO_THROTTLE(rclcpp::get_logger("PoseInfo"), *clock_, 1000,
//         //         "Dynamic Obs [%s]: Pos(%.1f, %.1f) | Vel(%.1f, %.1f) | Size: %.1f", 
//         //         name.c_str(), ob.position.x(), ob.position.y(), 
//         //         ob.velocity.x(), ob.velocity.y(), 
//         //         (ob.type == Obstacle::CIRCLE ? ob.dimensions.radius : ob.dimensions.width));
//         // }

//         // Push to vector for Visualization
//         obstacle_positions_.push_back(ob);
//     }
// }

// Helper function implementation
// Eigen::Vector2d GazeboObstacleChecker::calculateDeterministicPosition(const Obstacle& ob, double current_robot_time) const {
    
//     if (!ob.has_ground_truth) return ob.position;

//     double time_elapsed = initial_budget_time_ - current_robot_time;
//     double amplitude = ob.motion_limit; 
//     double speed = ob.speed_scalar;
    
//     if (speed < 1e-6) return ob.position;

//     double total_distance = speed * time_elapsed;
//     double phase = total_distance / amplitude;
//     int cycle = static_cast<int>(phase); 
//     double remainder = phase - static_cast<double>(cycle);

//     // =================================================================
//     // FIX: The SDF 'initial_origin' IS the Start Point.
//     // =================================================================
//     // Based on your SDF:
//     // <pose>-55.0 30.0 0 0 0 0</pose>
//     // <amplitude>110.0</amplitude>
//     // The obstacle starts at -55 and moves to +55.
//     // Therefore, 'initial_origin' is the start point. We do NOT subtract amplitude.
    
//     Eigen::Vector2d math_start_point = ob.initial_origin; 
    
//     Eigen::Vector2d calculated_pos;
//     if (cycle % 2 == 0) {
//         // Moving Forward (Start -> End)
//         calculated_pos = math_start_point + ob.motion_axis * (remainder * amplitude);
//     } else {
//         // Moving Backward (End -> Start)
//         calculated_pos = math_start_point + ob.motion_axis * ((1.0 - remainder) * amplitude);
//     }
//     return calculated_pos;
// }

// // The updated processing function --> deterministic integration!
// void GazeboObstacleChecker::processLatestPoseInfo(double current_robot_time) {
//     gz::msgs::Pose_V msg;
//     {
//         std::lock_guard<std::mutex> lock(snapshot_mutex_);
//         if (!new_pose_msg_available_) return;
//         msg = latest_pose_msg_;
//         new_pose_msg_available_ = false;
//     }
//     rclcpp::Time now = clock_->now();
//     obstacle_positions_.clear(); 
    
//     for (int i = 0; i < msg.pose_size(); ++i) {
//         const auto& pose = msg.pose(i);
//         const std::string name = pose.name();
        
//         if (name == robot_model_name_) {
//             robot_position_ = Eigen::Vector2d(pose.position().x(), pose.position().y());
//             continue;
//         }

//         auto info_it = obstacle_info_.find(name);
//         if (info_it == obstacle_info_.end()) continue;
//         const auto& info = info_it->second;
//         Obstacle& ob = obstacle_positions_map_[name]; 
        
//         // 1. Get Gazebo Position (The "Noisy" Truth)
//         Eigen::Vector2d gazebo_pos(pose.position().x(), pose.position().y());
        
//         // 2. Calculate Deterministic Position (The "Math" Truth)
//         Eigen::Vector2d math_pos = gazebo_pos; // Default fallback
//         if (use_deterministic_override_ && ob.is_dynamic) {
//             math_pos = calculateDeterministicPosition(ob, current_robot_time);
            
//             // =================================================================
//             // ERROR ANALYSIS
//             // =================================================================
//             // COMMENTED OUT: The time comparison is invalid because Gazebo's 
//             // internal clock drifts from the planner's wall-clock time.
//             /*
//             double error = (gazebo_pos - math_pos).norm();
//             if (error > 0.01) {
//                 RCLCPP_WARN_THROTTLE(rclcpp::get_logger("Obs_Error"), *clock_, 2000,
//                     "[NOISE DETECTED] [%s] | T: %.2f | Error: %.4f m | Gazebo: (%.2f, %.2f) | Math: (%.2f, %.2f)",
//                     name.c_str(), current_robot_time, error, 
//                     gazebo_pos.x(), gazebo_pos.y(), 
//                     math_pos.x(), math_pos.y());
//             }
//             */
//             // =================================================================
//         }

//         // 3. Update Obstacle State
//         // If override is ON, we use math_pos. If OFF, we use gazebo_pos.
//         Eigen::Vector2d active_pos = use_deterministic_override_ ? math_pos : gazebo_pos;
        
//         ob.previous_position = ob.position; 
        
//         // Populate Static Data
//         ob.name = name;
//         ob.is_dynamic = info.is_dynamic;
//         ob.has_ground_truth = true;
//         ob.speed_scalar = info.speed;
//         ob.motion_limit = info.amplitude;
//         ob.initial_origin = info.initial_pose.head<2>();
        
//         if (info.direction.head<2>().norm() > 1e-6) {
//              ob.motion_axis = info.direction.head<2>().normalized();
//         } else {
//              ob.motion_axis = Eigen::Vector2d::UnitX();
//         }

//         // 4. Velocity Calculation
//         if (ob.is_dynamic) {
//             if (ob.last_update_time.nanoseconds() == 0) {
//                 double sdf_dir_sign = info.direction.head<2>().dot(ob.motion_axis);
//                 ob.velocity = ob.motion_axis * (ob.speed_scalar * sdf_dir_sign);
//             } else {
//                 // Calculate displacement based on the ACTIVE position (Math or Gazebo)
//                 Eigen::Vector2d displacement = active_pos - ob.position;
                
//                 if (displacement.norm() > 1e-4) {
//                     double dot = displacement.dot(ob.motion_axis);
//                     double dir_sign = (dot >= 0.0) ? 1.0 : -1.0;
//                     ob.velocity = ob.motion_axis * (ob.speed_scalar * dir_sign);
//                 }
//             }
//         } else {
//             ob.velocity = Eigen::Vector2d::Zero();
//         }

//         // 5. Final Update
//         ob.position = active_pos; // Use the calculated position
//         ob.last_update_time = now;

//         // 6. Geometry Sync (Same as before)
//         if (info.type == ObstacleInfo::CYLINDER) {
//             ob.type = Obstacle::CIRCLE;
//             ob.dimensions.radius = info.radius;
//         } else {
//             ob.type = Obstacle::BOX;
//             ob.dimensions.width = info.width;
//             ob.dimensions.height = info.height;
            
//             Eigen::Quaterniond q(
//                 pose.orientation().w(),
//                 pose.orientation().x(),
//                 pose.orientation().y(),
//                 pose.orientation().z()
//             );
//             double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 
//                                     1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
//             ob.dimensions.rotation = yaw;
//         }

//         obstacle_positions_.push_back(ob);
//     }
// }

// void GazeboObstacleChecker::processLatestPoseInfo(double sim_time) {
//     obstacle_positions_.clear(); 
    
//     for (auto& [name, ob] : obstacle_positions_map_) {
//         if (ob.is_dynamic) {
//             // 1. Calculate Position based on Forward Time
//             double phase = (ob.speed_scalar * sim_time) / ob.motion_limit;
//             double cycle_val = std::fmod(phase, 2.0);
//             double remainder = phase - static_cast<int>(phase);

//             Eigen::Vector2d calculated_pos;
//             if (cycle_val < 1.0) {
//                 // Moving Forward (Start -> End)
//                 calculated_pos = ob.initial_origin + ob.motion_axis * (remainder * ob.motion_limit);
//             } else {
//                 // Moving Backward (End -> Start)
//                 calculated_pos = ob.initial_origin + ob.motion_axis * ((1.0 - remainder) * ob.motion_limit);
//             }
//             ob.position = calculated_pos;

//             // 2. Calculate Velocity based on Forward Time
//             // FIX: 
//             // 1. Determine direction along the axis based on the cycle (Forward or Backward)
//             double cycle_direction = (cycle_val < 1.0) ? 1.0 : -1.0;
            
//             // 2. Calculate velocity: Axis * Speed * CycleDirection
//             // motion_axis contains the SDF direction (e.g., 1,0 or 0,-1).
//             // cycle_direction flips it if we are in the return leg.
//             ob.velocity = ob.motion_axis * (ob.speed_scalar * cycle_direction);
            
//         } else {
//             ob.position = ob.initial_origin;
//             ob.velocity = Eigen::Vector2d::Zero();
//         }
        
//         obstacle_positions_.push_back(ob);
//     }
// }

void GazeboObstacleChecker::processLatestPoseInfo(double sim_time) {
    obstacle_positions_.clear(); 
    
    for (auto& [name, ob] : obstacle_positions_map_) {
        if (ob.is_dynamic) {
            // Calculate time for one leg (forward or backward)
            double time_per_leg = ob.motion_limit / ob.speed_scalar;  // 110/28 = 3.92857s
            
            // Total cycle time (forward + backward)
            double cycle_time = 2.0 * time_per_leg;  // 7.85714s
            
            // Where are we in the current cycle?
            double cycle_position = std::fmod(sim_time, cycle_time);
            
            // Determine position and velocity
            if (cycle_position <= time_per_leg) {
                // Forward leg: moving from initial_origin to initial_origin + motion_axis * amplitude
                double progress = cycle_position / time_per_leg;  // 0 to 1
                ob.position = ob.initial_origin + ob.motion_axis * (progress * ob.motion_limit);
                ob.velocity = ob.motion_axis * ob.speed_scalar;  // Positive direction
            } else {
                // Backward leg: moving from end back to start
                double progress = (cycle_position - time_per_leg) / time_per_leg;  // 0 to 1
                ob.position = ob.initial_origin + ob.motion_axis * ((1.0 - progress) * ob.motion_limit);
                ob.velocity = ob.motion_axis * (-ob.speed_scalar);  // Negative direction
            }
        } else {
            ob.position = ob.initial_origin;
            ob.velocity = Eigen::Vector2d::Zero();
        }
        
        obstacle_positions_.push_back(ob);
    }
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
    // std::lock_guard<std::mutex> lock(data_mutex_);

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
    // std::lock_guard<std::mutex> lock(data_mutex_);
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


// fcl::CollisionObjectd GazeboObstacleChecker::createFCLObject(const Obstacle& obstacle) const {
//     std::shared_ptr<fcl::CollisionGeometryd> geom;

//     if (obstacle.type == Obstacle::CIRCLE) {
//         // For circles, we use a cylinder in FCL with a small height.
//         // FCL doesn't have a 2D circle, but a short cylinder is equivalent for 2D checks.
//         geom = std::make_shared<fcl::Cylinderd>(obstacle.dimensions.radius, 1.0);
//     } else { // BOX
//         geom = std::make_shared<fcl::Boxd>(obstacle.dimensions.width, obstacle.dimensions.height, 1.0);
//     }

//     return fcl::CollisionObjectd(geom);
// }



// // Implementation of the unified public function
// bool GazeboObstacleChecker::checkRobotCollision(const Eigen::Vector2d& position, double yaw) const {
//     if (footprint_type_ == "rectangular") {
//         return checkRectangularCollisionHelper(position, yaw);
//     } else { // "circular" or default
//         return checkCircularCollisionHelper(position, robot_radius_);
//     }
// }

bool GazeboObstacleChecker::checkRobotCollision(const Eigen::Vector2d& robot_pos, double yaw) const {
    // 1. Thread Safety Lock (CRITICAL if running in threaded executor)
    // std::lock_guard<std::mutex> lock(obstacles_mutex_); 

    // 2. DIAGNOSTIC: Check if we even know about obstacles
    if (obstacle_positions_.empty()) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("CollisionCheck"), *clock_, 1000, 
            "[CollisionCheck] SKIPPING: 0 obstacles known! (Is ProcessLatestPoseInfo running?)");
        return false;
    }


    // Using inflation as robot radius
    double robot_r = inflation; 

    for (const auto& ob : obstacle_positions_) {
        // Calculate Distance
        double dist = (robot_pos - ob.position).norm();
        
        // Calculate Obstacle Radius (Circle vs Box approximation)
        double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                       std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);

        double collision_threshold = robot_r + obs_r;

        // // 4. LOGGING: Print details if we are getting close (within 3 meters of collision)
        // if (dist < (collision_threshold + 3.0)) {
        //     RCLCPP_INFO_THROTTLE(rclcpp::get_logger("CollisionDebug"), *clock_, 200, 
        //         "NEAR MISS: Robot(%.2f, %.2f) vs [%s](%.2f, %.2f) \n"
        //         "    -> Dist: %.3f | Threshold: %.3f (Rob:%.1f + Obs:%.1f)",
        //         robot_pos.x(), robot_pos.y(), 
        //         ob.name.c_str(), ob.position.x(), ob.position.y(), 
        //         dist, collision_threshold, robot_r, obs_r);
        // }

        // 5. The Check
        if (dist < collision_threshold) {
            RCLCPP_ERROR(rclcpp::get_logger("CollisionCheck"), 
                "!!! CRASH DETECTED !!! Robot vs [%s] | Dist: %.3f < %.3f",
                ob.name.c_str(), dist, collision_threshold);
            return true;
        }
    }
    return false;
}



// Helper for rectangular checks now uses its member variable
bool GazeboObstacleChecker::checkRectangularCollisionHelper(const Eigen::Vector2d& position, double yaw) const {
    const Eigen::Rotation2Dd rot(yaw);
    for(const auto& local_point : rectangular_footprint_) { // Uses member variable
        Eigen::Vector2d world_point = position + rot * local_point;
        for(const auto& obstacle : obstacle_snapshot_) {
            if(obstacle.type == Obstacle::CIRCLE) {
                if((world_point - obstacle.position).norm() <= obstacle.dimensions.radius) return true;
            } else { // BOX
                if(pointIntersectsRectangle(world_point, obstacle.position,
                                           obstacle.dimensions.width, obstacle.dimensions.height, obstacle.dimensions.rotation)) return true;
            }
        }
    }
    return false;
}

// Helper for circular checks (no change in logic, just making it private)
bool GazeboObstacleChecker::checkCircularCollisionHelper(const Eigen::Vector2d& robot_position, double robot_radius) const {
    for (const auto& obstacle : obstacle_snapshot_) {
        if (obstacle.type == Obstacle::CIRCLE) {
            double required_dist = robot_radius + obstacle.dimensions.radius;
            if ((robot_position - obstacle.position).norm() <= required_dist) return true;
        } else { // BOX
            Eigen::Rotation2Dd rot(-obstacle.dimensions.rotation);
            Eigen::Vector2d local_circle_pos = rot * (robot_position - obstacle.position);
            double closest_x = std::max(-obstacle.dimensions.width / 2.0, std::min(local_circle_pos.x(), obstacle.dimensions.width / 2.0));
            double closest_y = std::max(-obstacle.dimensions.height / 2.0, std::min(local_circle_pos.y(), obstacle.dimensions.height / 2.0));
            if ((local_circle_pos - Eigen::Vector2d(closest_x, closest_y)).norm() <= robot_radius) return true;
        }
    }
    return false;
}

bool GazeboObstacleChecker::lineIntersectsBox3D(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& center, double w, double h, double d, double rot) const{
    // 1. Transform the line segment into the box's local coordinate system,
    // where the box is an Axis-Aligned Bounding Box (AABB) centered at the origin.
    Eigen::Vector3d p_start_local = start - center;
    Eigen::Vector3d p_end_local = end - center;
    Eigen::Rotation2Dd rot2d(-rot);
    p_start_local.head<2>() = rot2d * p_start_local.head<2>();
    p_end_local.head<2>() = rot2d * p_end_local.head<2>();

    // 2. Perform the Slab Test intersection algorithm.
    double t_near = 0.0;
    double t_far = 1.0; // The parameter `t` for the segment goes from 0.0 to 1.0
    Eigen::Vector3d delta = p_end_local - p_start_local;
    const double half_dims[3] = {w / 2.0, h / 2.0, d / 2.0};

    for (int i = 0; i < 3; ++i) { // Iterate over x, y, and z axes
        if (std::abs(delta[i]) < 1e-9) {
            // The line segment is parallel to the slab.
            // If it's outside the slab boundaries, there is no collision.
            if (p_start_local[i] < -half_dims[i] || p_start_local[i] > half_dims[i]) {
                return false;
            }
        } else {
            // Calculate intersection times of the line with the two slab planes.
            double t1 = (-half_dims[i] - p_start_local[i]) / delta[i];
            double t2 = ( half_dims[i] - p_start_local[i]) / delta[i];

            if (t1 > t2) std::swap(t1, t2); // Ensure t1 is the entry point

            // Update the overall intersection interval.
            t_near = std::max(t_near, t1);
            t_far = std::min(t_far, t2);

            // If the interval is invalid at any point, the line misses the box.
            if (t_near > t_far) {
                return false;
            }
        }
    }
    
    // If the loop completes, the line segment intersects the AABB.
    return true;
}


bool GazeboObstacleChecker::sweptBoxIntersection3D(const Eigen::Vector3d& p_r0, const Eigen::Vector3d& v_r, const Eigen::Vector3d& p_o0, const Eigen::Vector3d& v_o, double w, double h, double d, double T, double rot) const{
    const Eigen::Vector3d v_rel = v_r - v_o;
    Eigen::Vector3d p_rel_start = p_r0 - p_o0;
    Eigen::Vector3d v_local = v_rel;
    Eigen::Rotation2Dd rot2d(-rot);
    p_rel_start.head<2>() = rot2d * p_rel_start.head<2>();
    v_local.head<2>() = rot2d * v_local.head<2>();
    double t_near = 0.0, t_far = T;
    const double half_dims[3] = {w/2.0, h/2.0, d/2.0};
    for (int i = 0; i < 3; ++i) {
        if (std::abs(v_local[i]) < 1e-9) {
            if (p_rel_start[i] < -half_dims[i] || p_rel_start[i] > half_dims[i]) return false;
        } else {
            double t1 = (-half_dims[i] - p_rel_start[i]) / v_local[i];
            double t2 = ( half_dims[i] - p_rel_start[i]) / v_local[i];
            if (t1 > t2) std::swap(t1, t2);
            t_near = std::max(t_near, t1);
            t_far = std::min(t_far, t2);
            if (t_near > t_far) return false;
        }
    }
    return t_near <= T && t_far >= 0.0;
}


// NEW: Overloaded 3D collision checking function
bool GazeboObstacleChecker::checkRobotCollision(const Eigen::Vector3d& position, double yaw) const {
    // For Min-Snap, the robot is best approximated by a cylinder (a circle with height).
    const double ROBOT_HEIGHT_3D = 0.5;
    const double ROBOT_RADIUS_3D = inflation; 

    // Correctly switch between footprint types, just like the 2D version.
    if (footprint_type_ == "rectangular") {
        return checkRectangularCollisionHelper3D(position, yaw);
    } else { // "circular"
        return checkCircularCollisionHelper3D(position, ROBOT_RADIUS_3D, ROBOT_HEIGHT_3D);
    }
}

// NEW: Helper for 3D circular (cylinder-sphere/cylinder-box) collision
bool GazeboObstacleChecker::checkCircularCollisionHelper3D(const Eigen::Vector3d& robot_pos, double robot_radius, double robot_height) const {
    for (const auto& obs : obstacle_snapshot_) {
        Eigen::Vector3d obs_pos_3d(obs.position.x(), obs.position.y(), obs.z);

        if (obs.type == Obstacle::CIRCLE) { // Obstacle is a Sphere
            double combined_radius = robot_radius + obs.dimensions.radius;
            if ((robot_pos.head<2>() - obs_pos_3d.head<2>()).squaredNorm() < combined_radius * combined_radius) {
                double robot_z_min = robot_pos.z() - robot_height / 2.0;
                double robot_z_max = robot_pos.z() + robot_height / 2.0;
                double obs_z_min = obs_pos_3d.z() - obs.dimensions.radius;
                double obs_z_max = obs_pos_3d.z() + obs.dimensions.radius;

                if (std::max(robot_z_min, obs_z_min) <= std::min(robot_z_max, obs_z_max)) {
                    return true; // Collision
                }
            }
        } else { // Obstacle is a Box
            Eigen::Rotation2Dd rot(-obs.dimensions.rotation);
            Eigen::Vector3d local_robot_pos = robot_pos - obs_pos_3d;
            local_robot_pos.head<2>() = rot * local_robot_pos.head<2>();

            double closest_x = std::max(-obs.dimensions.width / 2.0, std::min(local_robot_pos.x(), obs.dimensions.width / 2.0));
            double closest_y = std::max(-obs.dimensions.height / 2.0, std::min(local_robot_pos.y(), obs.dimensions.height / 2.0));
            double closest_z = std::max(-obs.dimensions.height / 2.0, std::min(local_robot_pos.z(), obs.dimensions.height / 2.0)); // Assume depth = height

            Eigen::Vector3d closest_point_on_box(closest_x, closest_y, closest_z);
            if ((local_robot_pos - closest_point_on_box).squaredNorm() < robot_radius * robot_radius) {
                 return true; // Collision
            }
        }
    }
    return false;
}


// // NEW: Implementation for the 3D rectangular footprint check
// bool GazeboObstacleChecker::checkRectangularCollisionHelper3D(const Eigen::Vector3d& position, double yaw) const {
//     const Eigen::Rotation2Dd rot2d(yaw);

//     // This check is simplified and treats the 3D footprint as a series of vertical lines.
//     // This is a common and effective approximation for quadcopter-like robots.
//     for (const auto& local_footprint_point : rectangular_footprint_) {
//         Eigen::Vector2d world_xy = position.head<2>() + rot2d * local_footprint_point;
        
//         // Define the top and bottom of the vertical line representing a corner of the robot
//         Eigen::Vector3d line_top(world_xy.x(), world_xy.y(), position.z() + 0.25); // Assuming 0.5m height
//         Eigen::Vector3d line_bottom(world_xy.x(), world_xy.y(), position.z() - 0.25);

//         for (const auto& obs : obstacle_snapshot_) {
//              Eigen::Vector3d obs_pos_3d(obs.position.x(), obs.position.y(), obs.z);
//             if (obs.type == Obstacle::CIRCLE) { // Sphere vs Line
//                 double R_sq = std::pow(obs.dimensions.radius, 2);
//                 if (distanceSqrdPointToSegment3D(obs_pos_3d, line_bottom, line_top) <= R_sq) {
//                     return true;
//                 }
//             } else { // Box vs Line
//                 double w = obs.dimensions.width;
//                 double h = obs.dimensions.height;
//                 double d = h; // Assume depth = height
//                 if (lineIntersectsBox3D(line_bottom, line_top, obs_pos_3d, w, h, d, obs.dimensions.rotation)) {
//                     return true;
//                 }
//             }
//         }
//     }
//     return false;
// }

bool GazeboObstacleChecker::checkRectangularCollisionHelper3D(const Eigen::Vector3d& position, double yaw) const {
    const Eigen::Rotation2Dd rot2d(yaw);

    // Define the z-range of the robot's body
    // Assuming the robot is 0.5m tall, centered at 'position.z()'
    const double robot_z_min = position.z() - 0.25;
    const double robot_z_max = position.z() + 0.25;

    for (const auto& local_footprint_point : rectangular_footprint_) {
        // Get the world coordinates of the corner of the robot's footprint
        Eigen::Vector2d world_xy = position.head<2>() + rot2d * local_footprint_point;

        for (const auto& obs : obstacle_snapshot_) {
            if (obs.type == Obstacle::BOX) { // Cube Obstacle
                const double half_width = obs.dimensions.width / 2.0;
                const Eigen::Vector2d obs_pos_2d = obs.position.head<2>();

                // --- Step 1: 2D AABB Check ---
                if (world_xy.x() >= obs_pos_2d.x() - half_width &&
                    world_xy.x() <= obs_pos_2d.x() + half_width &&
                    world_xy.y() >= obs_pos_2d.y() - half_width &&
                    world_xy.y() <= obs_pos_2d.y() + half_width) {
                    
                    // --- Step 2: Z-Axis Overlap Check ---
                    const double obs_z_min = obs.z - half_width;
                    const double obs_z_max = obs.z + half_width;

                    if (robot_z_max > obs_z_min && robot_z_min < obs_z_max) {
                        return true; // Collision!
                    }
                }
            } else { // Spherical Obstacle (the original check is efficient enough)
                const Eigen::Vector3d obs_pos_3d(obs.position.x(), obs.position.y(), obs.z);
                const Eigen::Vector3d line_top(world_xy.x(), world_xy.y(), robot_z_max);
                const Eigen::Vector3d line_bottom(world_xy.x(), world_xy.y(), robot_z_min);
                const double r_sq = obs.dimensions.radius * obs.dimensions.radius;

                if (distanceSqrdPointToSegment3D(obs_pos_3d, line_bottom, line_top) <= r_sq) {
                    return true; // Collision!
                }
            }
        }
    }

    return false; // No collision
}



/**
 * This transforms the problem from "does a moving robot collide with a moving box?" to "does the robot's relative path intersect a stationary box? Using relative motion
 * 
 * @brief Performs a fast intersection test between a 3D line segment and an Axis-Aligned Bounding Box (AABB).
 * This is much faster than a general-purpose Oriented Bounding Box (OBB) test when rotation is zero.
 * @param p0 The start point of the line segment.
 * @param p1 The end point of the line segment.
 * @param box_center The center of the AABB.
 * @param box_half_sizes The half-dimensions (half-width, half-height, half-depth) of the box.
 * @return True if the line segment intersects the box, false otherwise.
 */
bool GazeboObstacleChecker::fastLineAABBIntersection(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
                              const Eigen::Vector3d& box_center, const Eigen::Vector3d& box_half_sizes) const{
    const Eigen::Vector3d box_min = box_center - box_half_sizes;
    const Eigen::Vector3d box_max = box_center + box_half_sizes;
    const Eigen::Vector3d dir = p1 - p0;

    double tmin = 0.0;
    double tmax = 1.0; // We are checking a segment, so t is in [0, 1]

    for (int i = 0; i < 3; ++i) {
        if (std::abs(dir(i)) < 1e-9) { // Ray is parallel to the slab
            // If the ray's origin is not inside the slab, it will never intersect
            if (p0(i) < box_min(i) || p0(i) > box_max(i)) {
                return false;
            }
        } else {
            // Compute intersection t value of ray with near and far plane of slab
            double t1 = (box_min(i) - p0(i)) / dir(i);
            double t2 = (box_max(i) - p0(i)) / dir(i);

            // Make t1 the smaller value
            if (t1 > t2) std::swap(t1, t2);

            // Update the overall intersection interval
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);

            // If the interval is invalid, there is no collision
            if (tmin > tmax) {
                return false;
            }
        }
    }
    // If the interval is valid, a collision occurs
    return true;
}

void GazeboObstacleChecker::recordCulprit(const Obstacle& obs) const {
    // if (collision_culprits_names_.find(obs.name) == collision_culprits_names_.end()) {
    //     collision_culprits_names_.insert(obs.name);
    //     collision_culprits_data_.push_back(obs);
    // }
}



// Eigen::Vector2d GazeboObstacleChecker::getObstaclePositionAtTime(const Obstacle& ob, double query_time) const {
//     const auto& path = ob.predicted_path;
//     if (path.empty()) return ob.position;

//     // Bounds check (Path is stored: High Time -> Low Time)
//     if (query_time >= path.front().z()) return Eigen::Vector2d(path.front().x(), path.front().y());
//     if (query_time <= path.back().z()) return Eigen::Vector2d(path.back().x(), path.back().y());

//     // Linear Search & Interpolate
//     for (size_t i = 0; i < path.size() - 1; ++i) {
//         const auto& p_prev = path[i];     // Higher T
//         const auto& p_next = path[i+1];   // Lower T

//         if (query_time <= p_prev.z() && query_time >= p_next.z()) {
//             double total_dt = p_prev.z() - p_next.z();
//             if (std::abs(total_dt) < 1e-6) return Eigen::Vector2d(p_prev.x(), p_prev.y());

//             double alpha = (query_time - p_next.z()) / total_dt; 
//             double x = p_next.x() + alpha * (p_prev.x() - p_next.x());
//             double y = p_next.y() + alpha * (p_prev.y() - p_next.y());
//             return Eigen::Vector2d(x, y);
//         }
//     }
//     return ob.position;
// }




Eigen::Vector2d GazeboObstacleChecker::getObstaclePositionAtTime(
    const Obstacle& ob, double query_time) const 
{
    auto logger = rclcpp::get_logger("GazeboChecker");

    const auto& path = ob.predicted_path;
    if (path.empty()) {
        RCLCPP_DEBUG(logger, "Obs [%s] has empty path. Returning static pos.", ob.name.c_str());
        return ob.position;
    }

    // Determine direction
    bool is_descending = path.front().z() > path.back().z();
    double start_t = path.front().z();
    double end_t = path.back().z();

    // Check bounds based on direction
    if (is_descending) {
        if (query_time >= start_t) {
             RCLCPP_DEBUG(logger, "Query T=%.2f >= Start T=%.2f. Clamping to Start.", query_time, start_t);
             return Eigen::Vector2d(path.front().x(), path.front().y());
        }
        if (query_time <= end_t) {
             RCLCPP_DEBUG(logger, "Query T=%.2f <= End T=%.2f. Clamping to End.", query_time, end_t);
             return Eigen::Vector2d(path.back().x(), path.back().y());
        }
    } else { // Ascending
        if (query_time <= start_t) return Eigen::Vector2d(path.front().x(), path.front().y());
        if (query_time >= end_t)   return Eigen::Vector2d(path.back().x(), path.back().y());
    }

    // Linear Interpolation
    for (size_t i = 0; i < path.size() - 1; ++i) {
        const auto& p1 = path[i];
        const auto& p2 = path[i+1];

        // Robust check for containing interval regardless of direction
        bool in_interval = (p1.z() >= query_time && query_time >= p2.z()) || 
                           (p1.z() <= query_time && query_time <= p2.z());

        if (in_interval) {
            double total_dt = p2.z() - p1.z();
            if (std::abs(total_dt) < 1e-6) {
                RCLCPP_WARN(logger, "Zero DT detected in path of [%s] at idx %zu", ob.name.c_str(), i);
                return Eigen::Vector2d(p1.x(), p1.y());
            }

            double alpha = (query_time - p1.z()) / total_dt;
            
            Eigen::Vector2d result(
                p1.x() + alpha * (p2.x() - p1.x()),
                p1.y() + alpha * (p2.y() - p1.y())
            );

            // Detailed Math Log (Uncomment if needed)
            RCLCPP_INFO(logger, "Interpolate [%s]: T_query=%.3f | P1(T=%.3f) -> P2(T=%.3f) | Alpha=%.3f | Res=(%.2f, %.2f)",
                ob.name.c_str(), query_time, p1.z(), p2.z(), alpha, result.x(), result.y());
            return result;
        }
    }

    RCLCPP_WARN(logger, "Fallthrough in getObstaclePositionAtTime for [%s] T=%.2f. Returning Pos.", ob.name.c_str(), query_time);
    return ob.position;
}
// // // ITS GOOD BUT IF I HAVE COLLISON ITS BECAUSE IT dt IS NOT SMALL ENOUGH!!!
// std::vector<Eigen::Vector3d> GazeboObstacleChecker::generatePrediction(
//     const Obstacle& ob, 
//     double currentTime) const 
// {
//     std::vector<Eigen::Vector3d> path;
    
//     // Safety check: Needs to be dynamic and have valid physics data
//     if (!ob.is_dynamic || !ob.has_ground_truth) return path;

//     // const double dt_step = 0.05; 
//     const double dt_step = 0.1; // we cant reduce this much because of kd tree queries in addNewObstacle/RemoveObstacle but the isTrajecotrySafe functions is analytical and doesnt need much points!
    
//     // 1. Current State (Trusted from processLatestPoseInfo)
//     Eigen::Vector2d current_v = ob.velocity;
//     Eigen::Vector2d predicted_pos = ob.position;
    
//     // 2. Simulate (Linear Projection only)
//     // We do NOT check for walls or amplitude. 
//     // We assume it keeps going in the same direction forever.
//     for (double t = currentTime; t >= -1e-9; t -= dt_step) {
        
//         // Add point [X, Y, Time]
//         path.emplace_back(predicted_pos.x(), predicted_pos.y(), t);

//         // Move linearly: Pos = Pos + (Vel * dt)
//         predicted_pos = predicted_pos + (current_v * dt_step);
//     }
    
//     // // --- DEBUG LOG ---
//     // // You will now see Y increasing/decreasing monotonically (Straight Line)
//     // // instead of zig-zagging.
//     // // -----------------
//     // std::cout << "\n=== LINEAR PREDICTION TUBE FOR " << ob.name << " ===" << std::endl;
//     // for (size_t i = 0; i < path.size(); i+=5) { // Print every 5th point to save space
//     //      printf("[%zu] X:%.2f  Y:%.2f  T:%.2f\n", 
//     //            i, path[i].x(), path[i].y(), path[i].z());
//     // }
//     // std::cout << "===================================================\n" << std::endl;

//     return path;
// }


std::vector<Eigen::Vector3d> GazeboObstacleChecker::generatePrediction(
    const Obstacle& ob, 
    double currentTime) const 
{
    std::vector<Eigen::Vector3d> path;
    
    if (!ob.is_dynamic || !ob.has_ground_truth) return path;

    // 1. Calculate Effective Radius
    double R_eff = 0.0;
    if (ob.type == Obstacle::CIRCLE) {
        R_eff = ob.dimensions.radius;
    } else {
        // For Box, use the radius of the circumscribed circle (half diagonal)
        // This ensures we cover the corners even if it rotates (though yours is linear)
        R_eff = std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    }

    // 2. Get Speed (Scalar magnitude of velocity vector)
    double speed = ob.velocity.norm();

    // 3. Calculate Adaptive DT
    // We add a small safety factor (e.g., 0.8) to ensure the circles overlap slightly
    // rather than just touching. This prevents "edge cases" (pun intended).
    double dt_step = 0.1; // Default fallback
    
    if (speed > 1e-6) {
        // Formula: dt = (2 * Radius) / Speed
        // We clamp it to a minimum (e.g., 0.05) to prevent infinite loops if speed is huge,
        // and a maximum (e.g., 0.5) to prevent too sparse samples for very slow objects.
        double calculated_dt = (2.0 * R_eff) / speed;
        // Clamp values to keep sanity
        dt_step = std::clamp(calculated_dt, 0.05, 1.0);
    } else {
        // Static obstacle (or very slow)
        dt_step = 0.5; // Don't need many samples if it's not moving
    }

    // 4. Generate Path
    Eigen::Vector2d predicted_pos = ob.position;
    Eigen::Vector2d current_v = ob.velocity;

    for (double t = currentTime; t >= -1e-9; t -= dt_step) {
        path.emplace_back(predicted_pos.x(), predicted_pos.y(), t);
        predicted_pos = predicted_pos + (current_v * dt_step);
    }
    
    return path;
}


// std::vector<Eigen::Vector3d> GazeboObstacleChecker::generatePrediction(
//     const Obstacle& ob, 
//     double currentTime) const 
// {
//     std::vector<Eigen::Vector3d> path;
    
//     if (!ob.is_dynamic || !ob.has_ground_truth) return path;
    
//     // 1. Define the Prediction Horizon (How far into the future do we care?)
//     // If your planner plans 5 seconds ahead, set this to 5.0.
//     const double prediction_horizon = 5.0; 
    
//     // 2. Start Point (Now)
//     path.emplace_back(ob.position.x(), ob.position.y(), currentTime);
    
//     // 3. End Point (Future)
//     // We project linearly: Pos + (Vel * Time)
//     Eigen::Vector2d future_pos = ob.position + (ob.velocity * prediction_horizon);
//     double future_time = currentTime - prediction_horizon; // Assuming your time goes backwards
    
//     path.emplace_back(future_pos.x(), future_pos.y(), future_time);
    
//     return path;
// }


//////////////////////////////////////////////NON DETERMINISTIC//////////////////////////////////////////
void GazeboObstacleChecker::initializeDynamicObstacles(double currentRobotTime) {
    // currentRobotTime is T_robot (e.g., 20.0)
    RCLCPP_INFO(rclcpp::get_logger("ObstacleChecker"), 
                "Initializing Dynamic Obstacles (T_robot Mode).");
                
    for (auto& [name, ob] : obstacle_positions_map_) {
        if (ob.is_dynamic) {
            double time_for_one_leg = ob.motion_limit / ob.speed_scalar;
            
            // Set the timer to the first turnaround in T_robot terms
            // Start (20.0) minus one leg duration
            ob.nextDirectionChangeTime = currentRobotTime - time_for_one_leg; 
            
            RCLCPP_INFO(rclcpp::get_logger("ObstacleChecker"), 
                "-> [%s] logic initialized. First turnaround at T_Robot=%.2f", 
                name.c_str(), ob.nextDirectionChangeTime);
        }
    }
}
// ObstacleVector GazeboObstacleChecker::checkAndRepairObstacles(double T_robot) {
// ObstacleVector triggered_obs;

// for (auto& [name, ob] : obstacle_positions_map_) {
//     if (!ob.is_dynamic) continue;

//     // --- 1. INITIALIZATION CHECK ---
//     if (!ob.is_initialized_in_graph) {
//         ob.is_initialized_in_graph = true;
//         ob.predicted_path = this->generatePrediction(ob, T_robot);
//         last_velocities_[name] = ob.velocity; 
//         triggered_obs.push_back(ob);
//         continue; 
//     }

//     // --- 2. CHECK GAZEBO SIGNAL ---
//     bool did_turn = false;
//     if (obstacle_turnaround_flags_[name]) {
//         did_turn = true;
//         obstacle_turnaround_flags_[name] = false;
        
//         // --- ROBUST VELOCITY FLIP ---
//         // We don't just do ob.velocity = -ob.velocity.
//         // Instead, we project the velocity onto the Motion Axis, flip the sign of the projection,
//         // and reconstruct the vector. This ensures we stay exactly on the line defined by the SDF.
        
//         // 1. Calculate how much velocity we have along the motion axis
//         double projection = ob.velocity.dot(ob.motion_axis);
        
//         // 2. Flip the projection (Reverse direction)
//         double flipped_projection = -projection;
        
//         // 3. Reconstruct the velocity vector
//         // New Velocity = (Axis * Flipped_Projection)
//         // This preserves the exact speed scalar from the SDF but reverses direction.
//         ob.velocity = ob.motion_axis * flipped_projection;
        
//         RCLCPP_WARN(rclcpp::get_logger("Obs_Debug"), 
//             "!!! TURNAROUND [%s] !!! | Velocity Corrected to (%.2f, %.2f)", 
//             name.c_str(), ob.velocity.x(), ob.velocity.y());
//     }

//     // --- 3. UPDATE PREDICTION ---
//     ob.predicted_path = this->generatePrediction(ob, T_robot);

//     // --- 4. TRIGGER UPDATE ---
//     if (did_turn) {
//         triggered_obs.push_back(ob);
//     }
    
//     last_velocities_[name] = ob.velocity;
// }
// return triggered_obs;
// }



std::vector<std::string> GazeboObstacleChecker::detectTurnaroundEvents(double currentRobotTime) {
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




// double GazeboObstacleChecker::calculateNextFlip(const Obstacle& ob, double currentRobotTime) {
//     auto info_it = obstacle_info_.find(ob.name);
//     if (info_it == obstacle_info_.end() || !info_it->second.is_dynamic) return -1.0;
//     const auto& info = info_it->second;

//     // 1. Define turnaround boundaries
//     Eigen::Vector2d pointA = info.initial_pose.head<2>();
//     Eigen::Vector2d pointB = pointA + (info.direction.head<2>() * info.amplitude);

//     // 2. CHATTERING FIX: Target the FURTHEST point
//     // When we trigger a flip, we are by definition very close to one boundary.
//     // We want to calculate the time to reach the OTHER boundary.
//     double distA = (pointA - ob.position).norm();
//     double distB = (pointB - ob.position).norm();

//     // If we are closer to B, we just hit B, so our next target is A.
//     // If we are closer to A, our next target is B.
//     Eigen::Vector2d nextTarget = (distA < distB) ? pointB : pointA;
//     std::string targetName = (distA < distB) ? "Point B (Limit)" : "Point A (Start)";

//     // 3. Calculate time to reach the FAR boundary
//     double distance = (nextTarget - ob.position).norm();
//     double time_to_reach = distance / std::max(info.speed, 0.001);

//     double nextChangeTime = currentRobotTime - time_to_reach;

//     // RCLCPP_INFO(rclcpp::get_logger("ObstacleLogic"), 
//     //     "[%s] New Target: %s | Dist: %.2f | T_now: %.2f | Next Flip at T: %.2f", 
//     //     ob.name.c_str(), targetName.c_str(), distance, currentRobotTime, nextChangeTime);

//     return nextChangeTime;
// }

///////////////////////////////////////////////////////////////////////////////////////////////////
// // Change argument from sim_time to T_robot
// ObstacleVector GazeboObstacleChecker::checkAndRepairObstacles(double T_robot) {
//     ObstacleVector triggered_obs;
    
//     for (auto& [name, ob] : obstacle_positions_map_) {
//         if (!ob.is_dynamic) continue;
        
//         if (!ob.is_initialized_in_graph) {
//             ob.is_initialized_in_graph = true;
//             // Generate prediction starting from current T_robot (e.g., 20.0)
//             // The prediction function will handle the loop down to 0
//             ob.predicted_path = this->generatePrediction(ob, T_robot);
//             triggered_obs.push_back(ob);
            
//             // Initialize the NEXT turnaround time in T_robot terms.
//             // If SimTime turnaround is at 3.67, T_robot turnaround is at 20.0 - 3.67 = 16.33.
//             double time_for_one_leg = ob.motion_limit / ob.speed_scalar;
//             ob.nextDirectionChangeTime = T_robot - time_for_one_leg; 
            
//             continue; 
//         }
        
//         // --- CHECK FOR TURNAROUND (T_robot decreases) ---
//         // We check if T_robot has dropped BELOW the scheduled time
//         if (T_robot <= ob.nextDirectionChangeTime) {
            
//             // Calculate T_robot equivalent for logging (optional)
//             double sim_time = initial_budget_time_ - T_robot; 
            
//             RCLCPP_WARN(rclcpp::get_logger("Obs_Debug"),
//                 "!!! TURNAROUND [%s] !!! | T_Robot: %.2f | Scheduled: %.2f", 
//                 name.c_str(), T_robot, ob.nextDirectionChangeTime);
                
//             // --- UPDATE SCHEDULE ---
//             // Schedule the NEXT turnaround (one leg further into the past)
//             double time_for_one_leg = ob.motion_limit / ob.speed_scalar;
//             ob.nextDirectionChangeTime -= time_for_one_leg;
            
//             // --- UPDATE PREDICTION ---
//             // Generate path starting from current T_robot
//             ob.predicted_path = this->generatePrediction(ob, T_robot);
            
//             triggered_obs.push_back(ob);
//         }
//     }
//     return triggered_obs;
// }

ObstacleVector GazeboObstacleChecker::checkAndRepairObstacles(double T_robot) {
    ObstacleVector triggered_obs;
    
    for (auto& [name, ob] : obstacle_positions_map_) {
        if (!ob.is_dynamic) continue;
        
        if (!ob.is_initialized_in_graph) {
            ob.is_initialized_in_graph = true;
            // Generate prediction starting from current T_robot
            ob.predicted_path = this->generatePrediction(ob, T_robot);
            triggered_obs.push_back(ob);
            
            // Schedule the NEXT turnaround
            double time_for_one_leg = ob.motion_limit / ob.speed_scalar;
            ob.nextDirectionChangeTime = T_robot - time_for_one_leg; 
            
            continue; 
        }
        
        // --- FIX: Check for Turnaround ---
        // We want to trigger if we have PASSED the scheduled time.
        // Since T_robot decreases (e.g., 4.30 -> 4.28), we check if T_robot is less than or equal to the scheduled time.
        // However, to avoid triggering multiple times for the same event, we check if we crossed the boundary.
        
        // Simple robust check: If current time is PAST the scheduled time, update.
        if (T_robot <= ob.nextDirectionChangeTime) {
            
            // Calculate Sim Time for logging
            double sim_time = initial_budget_time_ - T_robot; 
            
            RCLCPP_WARN(rclcpp::get_logger("Obs_Debug"),
                "!!! TURNAROUND [%s] !!! | T_Robot: %.2f | Scheduled: %.2f", 
                name.c_str(), T_robot, ob.nextDirectionChangeTime);
                
            // --- UPDATE SCHEDULE ---
            // Schedule the NEXT turnaround (one leg further into the past)
            double time_for_one_leg = ob.motion_limit / ob.speed_scalar;
            ob.nextDirectionChangeTime -= time_for_one_leg;
            
            // --- UPDATE PREDICTION ---
            // CRITICAL: We must regenerate the prediction based on the NEW direction.
            // generatePrediction uses ob.velocity. We must update ob.velocity FIRST.
            
            // 1. Update the Obstacle's internal state (Position & Velocity) to match the new leg
            // We use the exact same logic as processLatestPoseInfo to find the new velocity
            double cycle_time = 2.0 * time_for_one_leg;
            double cycle_position = std::fmod(sim_time, cycle_time);
            
            if (cycle_position <= time_for_one_leg) {
                // Forward leg
                ob.velocity = ob.motion_axis * ob.speed_scalar;
            } else {
                // Backward leg
                ob.velocity = ob.motion_axis * (-ob.speed_scalar);
            }
            
            // 2. Now generate the prediction using this NEW velocity
            ob.predicted_path = this->generatePrediction(ob, T_robot);
            
            triggered_obs.push_back(ob);
        }
    }
    return triggered_obs;
}


// double GazeboObstacleChecker::calculateNextFlip(const Obstacle& ob, double currentRobotTime) {
//     auto info_it = obstacle_info_.find(ob.name);
//     if (info_it == obstacle_info_.end() || !info_it->second.is_dynamic) return -1.0;
//     const auto& info = info_it->second;

//     double amplitude = info.amplitude;
//     double speed = info.speed; 
    
//     if (speed < 1e-6) return -1.0;
//     double time_for_one_leg = amplitude / speed;

//     double time_elapsed = initial_budget_time_ - currentRobotTime;

//     // =================================================================
//     // FIX: Force calculation to the NEXT leg to prevent chattering
//     // =================================================================
//     // We add a tiny epsilon to time_elapsed. This ensures that if we are 
//     // exactly at the turnaround point (e.g. 4.0000), we are treated as 
//     // being slightly into the NEXT leg (4.00001), so we calculate the 
//     // turnaround for the end of THAT leg, not the current one.
//     double time_elapsed_safe = time_elapsed + 1e-9; 
    
//     int current_leg_index = static_cast<int>(time_elapsed_safe / time_for_one_leg);
    
//     // The next turnaround is at the end of this new leg
//     double time_of_next_turn = (current_leg_index + 1) * time_for_one_leg;
    
//     double nextChangeTime = initial_budget_time_ - time_of_next_turn;

//     return nextChangeTime;
// }
// double GazeboObstacleChecker::calculateNextFlip(const Obstacle& ob, double currentRobotTime) {
//     auto info_it = obstacle_info_.find(ob.name);
//     if (info_it == obstacle_info_.end() || !info_it->second.is_dynamic) return -1.0;
    
//     const auto& info = info_it->second;
//     double amplitude = info.amplitude;
//     double speed = info.speed; 
    
//     if (speed < 1e-6) return -1.0;
    
//     double time_for_one_leg = amplitude / speed;
    
//     // FIX: Use Elapsed Time (increasing)
//     double time_elapsed = initial_budget_time_ - currentRobotTime;
    
//     // FIX: Use fmod to determine the current leg index robustly
//     double phase = (speed * time_elapsed) / amplitude;
//     double cycle_val = std::fmod(phase, 2.0);
    
//     // Determine which leg we are in (0 or 1)
//     int current_leg_index = (cycle_val < 1.0) ? 0 : 1;
    
//     // The next turnaround happens at the end of the *next* leg (current + 1)
//     // We add 1 to ensure we are looking forward in time.
//     int next_leg_index = current_leg_index + 1;
    
//     double time_of_next_turn_elapsed = next_leg_index * time_for_one_leg;
    
//     // Convert back to Robot Time Budget (decreasing)
//     double nextChangeTime = initial_budget_time_ - time_of_next_turn_elapsed;
    
//     return nextChangeTime;
// }