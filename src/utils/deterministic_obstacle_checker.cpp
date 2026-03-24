// Copyright 2025 Soheil E.nia

#include "motion_planning/utils/deterministic_obstacle_checker.hpp"

DeterministicObstacleChecker::DeterministicObstacleChecker(rclcpp::Clock::SharedPtr clock,
                                            const Params& params,
                                            const std::unordered_map<std::string, ObstacleInfo>& obstacle_info)
        : clock_(clock),
          obstacle_info_(obstacle_info){  // Initialize orientation as a 4D vector for quaternion

    inflation = params.getParam<double>("inflation");
    is_geometric_mode_ = params.getParam<bool>("is_geometric_mode", false);
    initial_budget_time_ = params.getParam<double>("initial_budget_time");

    // Initialize obstacle_positions_map_ from obstacle_info_
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



}

DeterministicObstacleChecker::~DeterministicObstacleChecker() = default;



bool DeterministicObstacleChecker::isObstacleFree(const Eigen::VectorXd& start, const Eigen::VectorXd& end) const {
    // std::lock_guard<std::mutex> lock(snapshot_mutex_);
    Eigen::Vector2d start2d = start.head<2>();
    Eigen::Vector2d end2d = end.head<2>();

    for (const auto& obstacle : obstacle_positions_) {
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

bool DeterministicObstacleChecker::isObstacleFreeAgainstSingleObstacle(const Eigen::VectorXd& start, const Eigen::VectorXd& end, const Obstacle& obs) const {
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

bool DeterministicObstacleChecker::isObstacleFreeAgainstSingleObstacle(const Eigen::VectorXd& point, const Obstacle& obs) const {
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

bool DeterministicObstacleChecker::isObstacleFree(const Eigen::VectorXd& point) const {
    // std::lock_guard<std::mutex> lock(snapshot_mutex_);
    Eigen::Vector2d point2d = point.head<2>();
    // IMPORTANT : I CHANGED THE FOLLOWING FROM obstacle_snapshot_ to obstacle_positions_
    for (const auto& obstacle : obstacle_positions_) {
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
bool DeterministicObstacleChecker::isObstacleFree(const std::vector<Eigen::VectorXd>& path) const {
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



// FOR CHECKING THE WHOLE PATH POINTS IN THE TRAJECTORY!

bool DeterministicObstacleChecker::isTrajectorySafeAgainstSingleObstacle(
    const Trajectory& trajectory,
    const Obstacle& ob) const
{
    // auto logger = rclcpp::get_logger("GazeboChecker");
    // 1. Basic Validity Checks
    if (!trajectory.is_valid || trajectory.path_points.empty()) return false;


    // --- GEOMETRIC MODE LOGIC ---
    if (is_geometric_mode_) {
        // In geometric mode, we treat the obstacle as a static circle at its CURRENT position.
        // We check the distance from the trajectory segment to this point.
        
        const Eigen::VectorXd& p1 = trajectory.path_points[0];
        const Eigen::VectorXd& p2 = trajectory.path_points[1];
        
        Eigen::Vector2d r_start = p1.head<2>();
        Eigen::Vector2d r_end   = p2.head<2>();
        
        // Obstacle Position (Static snapshot)
        Eigen::Vector2d obs_pos(ob.position.x(), ob.position.y());
        
        double obs_size = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius :
                          std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
        
        double threshold_dist = robot_radius_ + obs_size + inflation;
        double threshold_sq = threshold_dist * threshold_dist;

        // Use your existing helper function
        double dist_sq = distanceSqrdPointToSegment(obs_pos, r_start, r_end);
        
        return (dist_sq >= threshold_sq);
    }





    if (ob.predicted_path.empty()) return true;

    // 2. Setup Thresholds
    double obs_size = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius :
                      std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    double threshold_dist = robot_radius_ + obs_size + inflation;
    double threshold_sq = threshold_dist * threshold_dist;

    // 3. Iterate through Robot Path Segments
    // We check every segment defined by consecutive points in trajectory.path_points
    for (size_t r_idx = 0; r_idx < trajectory.path_points.size() - 1; ++r_idx) {
        
        // --- Define Robot Segment ---
        const Eigen::VectorXd& p1_robot = trajectory.path_points[r_idx];
        const Eigen::VectorXd& p2_robot = trajectory.path_points[r_idx+1];

        // Extract spatial positions (2D or 3D depending on your setup)
        Eigen::Vector2d P_r_start = p1_robot.head<2>();
        Eigen::Vector2d P_r_end   = p2_robot.head<2>();

        // Extract times
        double t_r_start = p1_robot(p1_robot.size() - 1); // Time-to-go at start of segment
        double t_r_end   = p2_robot(p2_robot.size() - 1); // Time-to-go at end of segment

        // Normalize Robot Time (Early -> Late) for the math logic
        Eigen::Vector2d P_early, P_late;
        double T_early, T_late;
        if (t_r_end < t_r_start) {
            P_early = P_r_end;   T_early = t_r_end;
            P_late  = P_r_start; T_late  = t_r_start;
        } else {
            P_early = P_r_start; T_early = t_r_start;
            P_late  = P_r_end;   T_late  = t_r_end;
        }

        // --- Iterate through Obstacle Path Segments ---
        for (size_t o_idx = 0; o_idx < ob.predicted_path.size() - 1; ++o_idx) {
            Eigen::Vector3d obs_pt1 = ob.predicted_path[o_idx];
            Eigen::Vector3d obs_pt2 = ob.predicted_path[o_idx+1];

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
                continue; // No temporal overlap for this pair of segments
            }

            // -----------------------------------------------------------------------
            // ANALYTICAL MATH (Relative Velocity)
            // -----------------------------------------------------------------------
            double robot_dt = T_late - T_early;
            if (robot_dt < 1e-6) robot_dt = 1e-6;
            Eigen::Vector2d V_robot = (P_late - P_early) / robot_dt;

            double obs_dt = OT_late - OT_early;
            if (obs_dt < 1e-6) obs_dt = 1e-6;
            Eigen::Vector2d V_obs = (O_late - O_early) / obs_dt;

            Eigen::Vector2d V_rel = V_robot - V_obs;
            
            // Position of robot relative to obstacle at the start of overlap
            Eigen::Vector2d P_robot_at_min = P_early + V_robot * (overlap_min - T_early);
            Eigen::Vector2d P_obs_at_min   = O_early + V_obs   * (overlap_min - OT_early);
            Eigen::Vector2d P_rel_at_min = P_robot_at_min - P_obs_at_min;

            // Find time of closest approach within the overlap interval
            double A = V_rel.dot(V_rel);
            double B = 2.0 * P_rel_at_min.dot(V_rel);
            double Tc_offset = (std::abs(A) < 1e-9) ? 0.0 : -B / (2.0 * A);
            double Tc = overlap_min + Tc_offset;

            // Clamp Tc to the overlap interval
            if (Tc < overlap_min) Tc = overlap_min;
            if (Tc > overlap_max) Tc = overlap_max;

            // -----------------------------------------------------------------------
            // ROBUSTNESS FIX: Check Boundaries
            // -----------------------------------------------------------------------
            double eps_check = std::max(1e-4, (overlap_max - overlap_min) * 1e-2);
            std::vector<double> times_to_check = {Tc, overlap_min, overlap_max, Tc - eps_check, Tc + eps_check};

            for (double t_current : times_to_check) {
                Eigen::Vector2d pos_robot_at_t = P_early + V_robot * (t_current - T_early);
                Eigen::Vector2d pos_obs_at_t   = O_early + V_obs   * (t_current - OT_early);
                
                double dist_sq = (pos_robot_at_t - pos_obs_at_t).squaredNorm();
                if (dist_sq < threshold_sq) {
                    return false; // Collision detected
                }
            }
        }
    }
    return true; // No collision found in any segment
}



bool DeterministicObstacleChecker::isTrajectorySafe(
    const Trajectory& trajectory
) const {
    // 1. Get the current world snapshot
    // (This list already contains the updated positions/velocities)
    const ObstacleVector& all_obs = getObstacles(); 

    // 2. Loop through EVERY obstacle and check its Tube
    for (const auto& ob : all_obs) {
        // If an edge hits ANY obstacle's predicted path, it's NOT safe
        if (!isTrajectorySafeAgainstSingleObstacle(trajectory, ob)) {
            return false; 
        }
    }

    return true; // Safe against the whole world
}

const ObstacleVector& DeterministicObstacleChecker::getObstaclePositions() const {
    // std::lock_guard<std::mutex> lock(data_mutex_);
    return obstacle_positions_;
}

Eigen::VectorXd DeterministicObstacleChecker::getRobotEulerAngles() const {
    // std::lock_guard<std::mutex> lock(data_mutex_);
    return quaternionToEuler(robot_orientation_);
}

Eigen::VectorXd DeterministicObstacleChecker::quaternionToEuler(const Eigen::VectorXd& quaternion) const {
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

void DeterministicObstacleChecker::processLatestPoseInfo(double sim_time) {
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

double DeterministicObstacleChecker::calculateYawFromQuaternion(const Eigen::VectorXd& quaternion) {
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




bool DeterministicObstacleChecker::lineIntersectsCircle(const Eigen::Vector2d& start,
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

bool DeterministicObstacleChecker::pointIntersectsCircle(const Eigen::Vector2d& point,
                                                  const Eigen::Vector2d& center,
                                                  double radius) {
    // Calculate the squared distance between the point and the center of the circle
    const double squaredDistance = (point - center).squaredNorm();

    // Check if the squared distance is less than or equal to the squared radius
    return squaredDistance <= (radius * radius);
}




const ObstacleVector& DeterministicObstacleChecker::getObstacles() const {
    // // std::lock_guard<std::mutex> lock(data_mutex_);
    // ObstacleVector filtered_obstacles;
    
    // for (const auto& obstacle : obstacle_positions_) {
    //     if (use_range) {
    //         // Calculate distance from robot to obstacle
    //         double distance = (robot_position_ - obstacle.position).norm();
            
    //         // Only include obstacles within sensor range
    //         if (distance <= sensor_range) {
    //             filtered_obstacles.push_back(obstacle);
    //         }
    //     } else {
    //         // Include all obstacles if range checking is disabled
    //         filtered_obstacles.push_back(obstacle);
    //     }
    // }
    
    // return filtered_obstacles;


    return obstacle_positions_;
}

int DeterministicObstacleChecker::getObstaclesSize() const {
    return obstacle_positions_.size();
}


// Add rectangle collision detection implementations
bool DeterministicObstacleChecker::lineIntersectsRectangle(const Eigen::Vector2d& start,
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

bool DeterministicObstacleChecker::pointIntersectsRectangle(const Eigen::Vector2d& point,
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

bool DeterministicObstacleChecker::checkRobotCollision(const Eigen::Vector2d& robot_pos, double yaw) const {
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



// Helper for circular checks (no change in logic, just making it private)
bool DeterministicObstacleChecker::checkCircularCollisionHelper(const Eigen::Vector2d& robot_position, double robot_radius) const {
    for (const auto& obstacle : obstacle_positions_) {
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

// NEW: Overloaded 3D collision checking function
bool DeterministicObstacleChecker::checkRobotCollision(const Eigen::Vector3d& position, double yaw) const {
    // For Min-Snap, the robot is best approximated by a cylinder (a circle with height).
    const double ROBOT_HEIGHT_3D = 0.5;
    const double ROBOT_RADIUS_3D = inflation; 
    return checkCircularCollisionHelper3D(position, ROBOT_RADIUS_3D, ROBOT_HEIGHT_3D);
}

// NEW: Helper for 3D circular (cylinder-sphere/cylinder-box) collision
bool DeterministicObstacleChecker::checkCircularCollisionHelper3D(const Eigen::Vector3d& robot_pos, double robot_radius, double robot_height) const {
    for (const auto& obs : obstacle_positions_) {
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



void DeterministicObstacleChecker::recordCulprit(const Obstacle& obs) const {
    // if (collision_culprits_names_.find(obs.name) == collision_culprits_names_.end()) {
    //     collision_culprits_names_.insert(obs.name);
    //     collision_culprits_data_.push_back(obs);
    // }
}

std::vector<Eigen::Vector3d> DeterministicObstacleChecker::generatePrediction(
    const Obstacle& ob, 
    double currentTime) const 
{
    std::vector<Eigen::Vector3d> path;

    // --- GEOMETRIC MODE ---
    // We don't care about time or future movement. 
    // We just need the obstacle's current position for the collision checker.
    if (is_geometric_mode_) {
        // We use Z=0.0 for the time component. 
        // The collision checker (isTrajectorySafeAgainstSingleObstacle) will ignore this Z value 
        // and treat the obstacle as a static circle at (X, Y).
        path.emplace_back(ob.position.x(), ob.position.y(), 0.0);

        // THE FIX: Initialize the AABB bounds for a stationary point
        ob.min_x = ob.position.x();
        ob.max_x = ob.position.x();
        ob.min_y = ob.position.y();
        ob.max_y = ob.position.y();


        return path;
    }


    
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
    
    ob.min_x = std::numeric_limits<double>::max();
    ob.max_x = std::numeric_limits<double>::lowest();
    ob.min_y = std::numeric_limits<double>::max();
    ob.max_y = std::numeric_limits<double>::lowest();

    for (const auto& p : path) {
        if (p.x() < ob.min_x) ob.min_x = p.x();
        if (p.x() > ob.max_x) ob.max_x = p.x();
        if (p.y() < ob.min_y) ob.min_y = p.y();
        if (p.y() > ob.max_y) ob.max_y = p.y();
    }


    return path;
}

bool DeterministicObstacleChecker::isNodeInObstacleTube(const Eigen::VectorXd& node_state, 
                                                const Obstacle& ob, 
                                                double max_edge_length) const {
    if (ob.predicted_path.empty()) return false;

    // 1. Calculate the Search Radius (The "Inflation" of the AABB)
    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    
    double search_radius;
    if (is_geometric_mode_) {
        search_radius = obs_r + ob.inflation + max_edge_length;
    } else {
        double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
        search_radius = obs_r + ob.inflation + max_edge_length + gap_coverage_inflation;
    }

    double nx = node_state[0];
    double ny = node_state[1];

    // ========================================================================
    // THE AABB SHORT-CIRCUIT (The "Simple" but massive gain)
    // ========================================================================
    // If the node is outside the bounding box (inflated by search_radius), 
    // it is physically impossible for it to be inside the tube. 
    if (nx < (ob.min_x - search_radius) || nx > (ob.max_x + search_radius) ||
        ny < (ob.min_y - search_radius) || ny > (ob.max_y + search_radius)) {
        return false; // Skip the heavy O(P) loop entirely!
    }

    // 2. Precise Loop (Only runs if the node is "near" the tube)
    double search_radius_sq = search_radius * search_radius;
    for (const auto& path_point : ob.predicted_path) {
        double dx = nx - path_point.x();
        double dy = ny - path_point.y();
        if ((dx*dx + dy*dy) <= search_radius_sq) { 
            return true; 
        }
    }

    return false;
}


void DeterministicObstacleChecker::initializeDynamicObstacles(double currentRobotTime) {
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

ObstacleVector DeterministicObstacleChecker::checkAndRepairObstacles(double T_robot) {
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

