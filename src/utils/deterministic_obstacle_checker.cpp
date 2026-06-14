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
        ob.initially_visible = (ob.name.find("visible") != std::string::npos);
        ob.is_discovered     = ob.initially_visible;   // if known from start, mark as discovered
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

    robot_radius_ = 0.0; // Leave this (use inflation as an integrated variable for both robot radius and obstacle inflation!).
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



// // FOR CHECKING THE WHOLE PATH POINTS IN THE TRAJECTORY!

// bool DeterministicObstacleChecker::isTrajectorySafeAgainstSingleObstacle(
//     const Trajectory& trajectory,
//     const Obstacle& ob) const
// {
//     // auto logger = rclcpp::get_logger("GazeboChecker");
//     // 1. Basic Validity Checks
//     if (!trajectory.is_valid || trajectory.path_points.empty()) return false;


//     // --- GEOMETRIC MODE LOGIC ---
//     if (is_geometric_mode_) {
//         // In geometric mode, we treat the obstacle as a static circle at its CURRENT position.
//         // We check the distance from the trajectory segment to this point.
        
//         const Eigen::VectorXd& p1 = trajectory.path_points[0];
//         const Eigen::VectorXd& p2 = trajectory.path_points[1];
        
//         Eigen::Vector2d r_start = p1.head<2>();
//         Eigen::Vector2d r_end   = p2.head<2>();
        
//         // Obstacle Position (Static snapshot)
//         Eigen::Vector2d obs_pos(ob.position.x(), ob.position.y());
        
//         double obs_size = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius :
//                           std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
        
//         double threshold_dist = robot_radius_ + obs_size + inflation;
//         double threshold_sq = threshold_dist * threshold_dist;

//         // Use your existing helper function
//         double dist_sq = distanceSqrdPointToSegment(obs_pos, r_start, r_end);
        
//         return (dist_sq >= threshold_sq);
//     }





//     if (ob.predicted_path.empty()) return true;

//     // 2. Setup Thresholds
//     double obs_size = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius :
//                       std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
//     double threshold_dist = robot_radius_ + obs_size + inflation;
//     double threshold_sq = threshold_dist * threshold_dist;

//     // 3. Iterate through Robot Path Segments
//     // We check every segment defined by consecutive points in trajectory.path_points
//     for (size_t r_idx = 0; r_idx < trajectory.path_points.size() - 1; ++r_idx) {
        
//         // --- Define Robot Segment ---
//         const Eigen::VectorXd& p1_robot = trajectory.path_points[r_idx];
//         const Eigen::VectorXd& p2_robot = trajectory.path_points[r_idx+1];

//         // Extract spatial positions (2D or 3D depending on your setup)
//         Eigen::Vector2d P_r_start = p1_robot.head<2>();
//         Eigen::Vector2d P_r_end   = p2_robot.head<2>();

//         // Extract times
//         double t_r_start = p1_robot(p1_robot.size() - 1); // Time-to-go at start of segment
//         double t_r_end   = p2_robot(p2_robot.size() - 1); // Time-to-go at end of segment

//         // Normalize Robot Time (Early -> Late) for the math logic
//         Eigen::Vector2d P_early, P_late;
//         double T_early, T_late;
//         if (t_r_end < t_r_start) {
//             P_early = P_r_end;   T_early = t_r_end;
//             P_late  = P_r_start; T_late  = t_r_start;
//         } else {
//             P_early = P_r_start; T_early = t_r_start;
//             P_late  = P_r_end;   T_late  = t_r_end;
//         }

//         // --- Iterate through Obstacle Path Segments ---
//         for (size_t o_idx = 0; o_idx < ob.predicted_path.size() - 1; ++o_idx) {
//             Eigen::Vector3d obs_pt1 = ob.predicted_path[o_idx];
//             Eigen::Vector3d obs_pt2 = ob.predicted_path[o_idx+1];

//             // Normalize Obstacle Time (Early -> Late)
//             Eigen::Vector2d O_early, O_late;
//             double OT_early, OT_late;
//             if (obs_pt2.z() < obs_pt1.z()) {
//                 O_early = obs_pt2.head<2>(); OT_early = obs_pt2.z();
//                 O_late  = obs_pt1.head<2>(); OT_late  = obs_pt1.z();
//             } else {
//                 O_early = obs_pt1.head<2>(); OT_early = obs_pt1.z();
//                 O_late  = obs_pt2.head<2>(); OT_late  = obs_pt2.z();
//             }

//             // Check Time Overlap
//             double overlap_min = std::max(T_early, OT_early);
//             double overlap_max = std::min(T_late, OT_late);
//             if (overlap_min > overlap_max) {
//                 continue; // No temporal overlap for this pair of segments
//             }

//             // -----------------------------------------------------------------------
//             // ANALYTICAL MATH (Relative Velocity)
//             // -----------------------------------------------------------------------
//             double robot_dt = T_late - T_early;
//             if (robot_dt < 1e-6) robot_dt = 1e-6;
//             Eigen::Vector2d V_robot = (P_late - P_early) / robot_dt;

//             double obs_dt = OT_late - OT_early;
//             if (obs_dt < 1e-6) obs_dt = 1e-6;
//             Eigen::Vector2d V_obs = (O_late - O_early) / obs_dt;

//             Eigen::Vector2d V_rel = V_robot - V_obs;
            
//             // Position of robot relative to obstacle at the start of overlap
//             Eigen::Vector2d P_robot_at_min = P_early + V_robot * (overlap_min - T_early);
//             Eigen::Vector2d P_obs_at_min   = O_early + V_obs   * (overlap_min - OT_early);
//             Eigen::Vector2d P_rel_at_min = P_robot_at_min - P_obs_at_min;

//             // Find time of closest approach within the overlap interval
//             double A = V_rel.dot(V_rel);
//             double B = 2.0 * P_rel_at_min.dot(V_rel);
//             double Tc_offset = (std::abs(A) < 1e-9) ? 0.0 : -B / (2.0 * A);
//             double Tc = overlap_min + Tc_offset;

//             // Clamp Tc to the overlap interval
//             if (Tc < overlap_min) Tc = overlap_min;
//             if (Tc > overlap_max) Tc = overlap_max;

//             // -----------------------------------------------------------------------
//             // ROBUSTNESS FIX: Check Boundaries
//             // -----------------------------------------------------------------------
//             double eps_check = std::max(1e-4, (overlap_max - overlap_min) * 1e-2);
//             std::vector<double> times_to_check = {Tc, overlap_min, overlap_max, Tc - eps_check, Tc + eps_check};

//             for (double t_current : times_to_check) {
//                 Eigen::Vector2d pos_robot_at_t = P_early + V_robot * (t_current - T_early);
//                 Eigen::Vector2d pos_obs_at_t   = O_early + V_obs   * (t_current - OT_early);
                
//                 double dist_sq = (pos_robot_at_t - pos_obs_at_t).squaredNorm();
//                 if (dist_sq < threshold_sq) {
//                     return false; // Collision detected
//                 }
//             }
//         }
//     }
//     return true; // No collision found in any segment
// }

/*
 * CONTINUOUS COLLISION DETECTION (CCD) PIPELINE EXPLANATION
 * This function determines if a robot's trajectory (represented as a series of 
 * time-parameterized states) collides with a single obstacle's predicted path.
 * 
 * The pipeline operates in 4 distinct phases:
 * 
 * 1. GEOMETRIC MODE FAST-PATH:
 *    If the planner is running in purely geometric mode (no time dimension), the obstacle 
 *    is treated as static. We calculate the squared distance from the obstacle's current 
 *    center to the robot's spatial line segment. If distance < (Radius + Inflation)^2, 
 *    it's a collision.
 * 
 * 2. BROAD-PHASE REJECTION (AABB / Bounding Sphere):
 *    Before executing expensive continuous math on dense trajectories, we check the 
 *    spatial bounding boxes of the entire robot trajectory against the entire obstacle tube. 
 *    If their bounds do not intersect in space, we instantly return SAFE (O(1) rejection).
 * 
 * 3. THE MATH KERNEL (checkSegmentMath):
 *    For dynamic obstacles, we isolate the continuous collision detection (CCD) physics. 
 *    Given a robot segment and an obstacle segment, we:
 *      a) Find their overlapping time window [overlap_min, overlap_max].
 *      b) Calculate their relative velocity vector (V_rel).
 *      c) Calculate the time of closest approach (T_c) using the derivative of the 
 *         distance-squared function.
 *      d) Check the spatial distance exactly at T_c and at the boundaries of the 
 *         time window to ensure no tunneling occurred.
 * 
 * 4. THE TEMPORAL LOOP ARCHITECTURE:
 *    To feed the right segments into the Math Kernel, we must traverse the arrays.
 *    Because both the robot's state and the obstacle's prediction flow monotonically in time, 
 *    evaluating every pair (O(N*M)) is massively inefficient. 
 *    Instead, we use either a Temporal Sweep (O(N+M)) or a Binary Search (O(N log M)) 
 *    to intelligently skip segments that do not overlap in time, passing only the physically 
 *    relevant segments to the Math Kernel.
 */

bool DeterministicObstacleChecker::isTrajectorySafeAgainstSingleObstacle(
    const Trajectory& trajectory,
    const Obstacle& ob) const
{
    // Basic Validity Checks
    if (!trajectory.is_valid || trajectory.path_points.empty()) return false;

    // STATIC OBSTACLE
    if (!ob.is_dynamic) {
        Eigen::Vector2d obs_pos(ob.position.x(), ob.position.y());
        
        if (ob.type == Obstacle::CIRCLE) {
            double threshold_sq = std::pow(robot_radius_ + ob.dimensions.radius + inflation, 2);
            for (size_t i = 0; i < trajectory.path_points.size() - 1; ++i) {
                Eigen::Vector2d r_start = trajectory.path_points[i].head<2>();
                Eigen::Vector2d r_end   = trajectory.path_points[i + 1].head<2>();
                
                if (distanceSqrdPointToSegment(obs_pos, r_start, r_end) < threshold_sq) {
                    return false;
                }
            }
        } 
        else if (ob.type == Obstacle::BOX) {
            // Inflate box to account for robot radius and inflation
            double total_margin = robot_radius_ + inflation;
            double eff_width  = ob.dimensions.width  + 2.0 * total_margin;
            double eff_height = ob.dimensions.height + 2.0 * total_margin;
            
            for (size_t i = 0; i < trajectory.path_points.size() - 1; ++i) {
                Eigen::Vector2d r_start = trajectory.path_points[i].head<2>();
                Eigen::Vector2d r_end   = trajectory.path_points[i + 1].head<2>();
                
                if (lineIntersectsRectangle(r_start, r_end, obs_pos, eff_width, eff_height, ob.dimensions.rotation)) {
                    return false;
                }
            }
        }
        return true;
    }

    // GEOMETRIC MODE LOGIC
    if (is_geometric_mode_) {
        Eigen::Vector2d r_start = trajectory.path_points[0].head<2>();
        Eigen::Vector2d r_end   = trajectory.path_points[1].head<2>();
        Eigen::Vector2d obs_pos(ob.position.x(), ob.position.y());
        
        if (ob.type == Obstacle::CIRCLE) {
            double threshold_sq = std::pow(robot_radius_ + ob.dimensions.radius + inflation, 2);
            return (distanceSqrdPointToSegment(obs_pos, r_start, r_end) >= threshold_sq);
        } 
        else if (ob.type == Obstacle::BOX) {
            double total_margin = robot_radius_ + inflation;
            double eff_width  = ob.dimensions.width  + 2.0 * total_margin;
            double eff_height = ob.dimensions.height + 2.0 * total_margin;
            
            // Return true if it is SAFE (i.e. does NOT intersect)
            return !lineIntersectsRectangle(r_start, r_end, obs_pos, eff_width, eff_height, ob.dimensions.rotation);
        }
    }


    if (ob.predicted_path.empty()) return true;

    // SETUP THRESHOLDS
    double obs_size = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);

    // TURNAROUND SLICE-TIME SAFETY MARGIN
    // To prevent Collisions where the obstacle turns around suddenly (since the planner assumes constant velocity and straight line movement and step simulation moves the robot and obstacle simultaneously)
    // before the next checkAndRepair cycle catches it, we pad the threshold. --> mind that this threshold can also be considered as inflation but overall it gives a last chance to the planner to find something incase of those rare events!
    // This also helps to benchmark success rate more accuratly (of course to a certain degree because if an obstalce suddenly turnsaround there is a low chance of escape if obstalce is too near!)
    // A buffer of 0.4 - 0.5m is enough --> If slice time is 0.02 and velocity is 50 m/s then the turnaround buffer should be 1m
    const double TURNAROUND_BUFFER = 1.0; // Meters of safety against intra-slice turnarounds
    
    double threshold_dist = robot_radius_ + obs_size + inflation + TURNAROUND_BUFFER; // robot_radius_ is zero by design. inflation take this into account

    double threshold_sq = threshold_dist * threshold_dist;


    // BROAD-PHASE O(1) AABB REJECTION
    double traj_min_x = std::numeric_limits<double>::max();
    double traj_max_x = std::numeric_limits<double>::lowest();
    double traj_min_y = std::numeric_limits<double>::max();
    double traj_max_y = std::numeric_limits<double>::lowest();

    for (const auto& pt : trajectory.path_points) {
        if (pt.x() < traj_min_x) traj_min_x = pt.x();
        if (pt.x() > traj_max_x) traj_max_x = pt.x();
        if (pt.y() < traj_min_y) traj_min_y = pt.y();
        if (pt.y() > traj_max_y) traj_max_y = pt.y();
    }

    bool aabb_would_reject = false;
    // If the bounding boxes don't intersect, it's physically impossible to collide
    if (traj_min_x > (ob.max_x + threshold_dist) || 
        traj_max_x < (ob.min_x - threshold_dist) ||
        traj_min_y > (ob.max_y + threshold_dist) || 
        traj_max_y < (ob.min_y - threshold_dist)) {
        return true; // Skip O(N+M) Sweep Entirely!
        // aabb_would_reject = true;
    }


    // Extract Robot Segment (Actually Meaning the Trajectory. The robot could later be there!)
    auto getRobotSegment = [&](size_t idx, Eigen::Vector2d& P_early, Eigen::Vector2d& P_late, double& T_early, double& T_late) {
        const Eigen::VectorXd& p1 = trajectory.path_points[idx];
        const Eigen::VectorXd& p2 = trajectory.path_points[idx+1];
        double t1 = p1(p1.size() - 1);
        double t2 = p2(p2.size() - 1);
        if (t2 < t1) { P_early = p2.head<2>(); T_early = t2; P_late = p1.head<2>(); T_late = t1; } 
        else         { P_early = p1.head<2>(); T_early = t1; P_late = p2.head<2>(); T_late = t2; }
    };

    // Extract Obstacle Segment
    auto getObsSegment = [&](size_t idx, Eigen::Vector2d& O_early, Eigen::Vector2d& O_late, double& OT_early, double& OT_late) {
        const Eigen::Vector3d& obs_pt1 = ob.predicted_path[idx];
        const Eigen::Vector3d& obs_pt2 = ob.predicted_path[idx+1];
        if (obs_pt2.z() < obs_pt1.z()) { O_early = obs_pt2.head<2>(); OT_early = obs_pt2.z(); O_late = obs_pt1.head<2>(); OT_late = obs_pt1.z(); } 
        else                           { O_early = obs_pt1.head<2>(); OT_early = obs_pt1.z(); O_late = obs_pt2.head<2>(); OT_late = obs_pt2.z(); }
    };

    // // CCD Math (Returns true if SAFE, false if COLLISION)
    // // Computes Time-of-Closest-Approach (Tc) for relative motion quadratic, then samples
    // // critical points within the VALID TEMPORAL OVERLAP [overlap_min, overlap_max] ONLY.
    // auto checkSegmentMath = [&](const Eigen::Vector2d& P_early, const Eigen::Vector2d& P_late, double T_early, double T_late,
    //                             const Eigen::Vector2d& O_early, const Eigen::Vector2d& O_late, double OT_early, double OT_late) -> bool {
        
    //     double overlap_min = std::max(T_early, OT_early);
    //     double overlap_max = std::min(T_late, OT_late);
        
    //     if (overlap_min > overlap_max) return true; // No temporal overlap

    //     // Linear velocities over segment duration (prevent div-by-zero)
    //     double robot_dt = std::max(1e-6, T_late - T_early);
    //     Eigen::Vector2d V_robot = (P_late - P_early) / robot_dt;

    //     double obs_dt = std::max(1e-6, OT_late - OT_early);
    //     Eigen::Vector2d V_obs = (O_late - O_early) / obs_dt;

    //     // Relative motion: Solve quadratic min_t ||P_rel + V_rel*t||^2 = 0
    //     Eigen::Vector2d V_rel = V_robot - V_obs;
    //     Eigen::Vector2d P_rel_at_min = (P_early + V_robot * (overlap_min - T_early)) - 
    //                                 (O_early + V_obs * (overlap_min - OT_early));

    //     double A = V_rel.dot(V_rel);
    //     double B = 2.0 * P_rel_at_min.dot(V_rel);
        
    //     // Time offset to theoretical closest approach (handle parallel motion)
    //     double Tc_offset = (std::abs(A) < 1e-9) ? 0.0 : -B / (2.0 * A);

    //     // Clamp to physical segment overlap (CRITICAL: prevents extrapolation)
    //     double Tc = std::clamp(overlap_min + Tc_offset, overlap_min, overlap_max);
        
    //     // Numerical robustness: Sample endpoints + Tc ±ε (ε=1% window or 0.1mm)
    //     // BUT ONLY IF those times remain within physical bounds!
    //     double eps_check = std::max(1e-4, (overlap_max - overlap_min) * 1e-2);

    //     std::vector<double> times_to_check = {overlap_min, overlap_max, Tc};
        
    //     // PREVENT EXTRAPOLATION BUG: Only add epsilon samples if physically valid
    //     if (Tc - eps_check > overlap_min) times_to_check.push_back(Tc - eps_check);
    //     if (Tc + eps_check < overlap_max) times_to_check.push_back(Tc + eps_check);

    //     // Test exact distance at sampled times (all within valid segment bounds)
    //     for (double t_current : times_to_check) {
    //         Eigen::Vector2d pos_robot_at_t = P_early + V_robot * (t_current - T_early);
    //         Eigen::Vector2d pos_obs_at_t   = O_early + V_obs   * (t_current - OT_early);
    //         if ((pos_robot_at_t - pos_obs_at_t).squaredNorm() < threshold_sq) {
    //             return false; // Collision detected
    //         }
    //     }
    //     return true; // Safe
    // };

    // ---------------------------------------------------------
    // CONTINUOUS COLLISION MATH ---> Compared to Above: added Box collision check explicitly!
    // --- IF BOX: Use dense swept AABB check ---
    /*
        * WHY WE CANNOT USE THE QUADRATIC (CIRCLE) METHOD FOR BOXES:
        * A moving box creates a large, non-uniform swept volume. If we approximate 
        * a box as a circle and only check the exact moment of closest center-to-center 
        * approach (Tc), we will miss the corners! The centers might be far enough 
        * apart at Tc to be considered "safe", but the long edge or corner of the 
        * rectangular box could still sweep right through the robot before or after Tc.
        * Therefore, we MUST slice the temporal overlap (dt_step) and test exact 
        * Axis-Aligned Bounding Box (AABB) intersections to guarantee the box's 
        * corners do not hit the robot between time samples.
        */
    
    // ---------------------------------------------------------
    // double threshold_sq = threshold_dist * threshold_dist;
    auto checkSegmentMath = [&](const Eigen::Vector2d& P_early, const Eigen::Vector2d& P_late, double T_early, double T_late,
                                const Eigen::Vector2d& O_early, const Eigen::Vector2d& O_late, double OT_early, double OT_late) -> bool {
        
        double overlap_min = std::max(T_early, OT_early);
        double overlap_max = std::min(T_late, OT_late);
        if (overlap_min > overlap_max) return true; // No temporal overlap

        double robot_dt = std::max(1e-6, T_late - T_early);
        Eigen::Vector2d V_robot = (P_late - P_early) / robot_dt;

        double obs_dt = std::max(1e-6, OT_late - OT_early);
        Eigen::Vector2d V_obs = (O_late - O_early) / obs_dt;

        // --- IF CIRCLE: Use fast Euclidean math ---
        if (ob.type == Obstacle::CIRCLE) {
            Eigen::Vector2d V_rel = V_robot - V_obs;
            Eigen::Vector2d P_rel_at_min = (P_early + V_robot * (overlap_min - T_early)) - (O_early + V_obs * (overlap_min - OT_early));
            double A = V_rel.dot(V_rel);
            double B = 2.0 * P_rel_at_min.dot(V_rel);
            double Tc_offset = (std::abs(A) < 1e-9) ? 0.0 : -B / (2.0 * A);
            double Tc = std::clamp(overlap_min + Tc_offset, overlap_min, overlap_max);
            double eps_check = std::max(1e-4, (overlap_max - overlap_min) * 1e-2);
            
            std::vector<double> times_to_check = {overlap_min, overlap_max, Tc};
            if (Tc - eps_check > overlap_min) times_to_check.push_back(Tc - eps_check);
            if (Tc + eps_check < overlap_max) times_to_check.push_back(Tc + eps_check);

            for (double t_current : times_to_check) {
                if (((P_early + V_robot * (t_current - T_early)) - (O_early + V_obs * (t_current - OT_early))).squaredNorm() < threshold_sq) return false;
            }
            return true;
        } 
        // --- IF BOX: Use dense swept AABB check ---
        else if (ob.type == Obstacle::BOX) {
            double total_margin = robot_radius_ + inflation + TURNAROUND_BUFFER;
            double eff_width  = ob.dimensions.width  + 2.0 * total_margin;
            double eff_height = ob.dimensions.height + 2.0 * total_margin;
            double half_width = eff_width / 2.0;
            double half_height = eff_height / 2.0;
            double cos_theta = std::cos(-ob.dimensions.rotation);
            double sin_theta = std::sin(-ob.dimensions.rotation);

            // Step through overlap at high frequency (e.g. 0.02s) to catch corners
            double dt_step = 0.02; 
            for (double t_current = overlap_min; t_current <= overlap_max; t_current += dt_step) {
                Eigen::Vector2d pos_robot = P_early + V_robot * (t_current - T_early);
                Eigen::Vector2d pos_obs   = O_early + V_obs   * (t_current - OT_early);
                
                double dx = pos_robot.x() - pos_obs.x();
                double dy = pos_robot.y() - pos_obs.y();
                double local_x = (dx * cos_theta) - (dy * sin_theta);
                double local_y = (dx * sin_theta) + (dy * cos_theta);

                if (std::abs(local_x) < half_width && std::abs(local_y) < half_height) return false; 
            }
            // Check absolute endpoint
            Eigen::Vector2d pos_robot = P_early + V_robot * (overlap_max - T_early);
            Eigen::Vector2d pos_obs   = O_early + V_obs   * (overlap_max - OT_early);
            double dx = pos_robot.x() - pos_obs.x();
            double dy = pos_robot.y() - pos_obs.y();
            if (std::abs((dx * cos_theta) - (dy * sin_theta)) < half_width && 
                std::abs((dx * sin_theta) + (dy * cos_theta)) < half_height) return false;

            return true;
        }
        return true;
    };


    /*
    * CONTINUOUS COLLISION DETECTION (CCD) COMPLEXITY ANALYSIS
    * Notes for the methodology section of the paper justifying the algorithmic choice 
    * for collision checking. We specifically avoid naive O(N*M) nested loops and optimize 
    * beyond standard O(N log M) binary search approaches (like those used in RRTx).
    * 
    * 1. THE MATH & CROSSOVER THRESHOLD:
    *    - Let N = Robot trajectory segments, M = Obstacle predicted segments.
    *    - Binary Search Approach: Searches the obstacle array for *every* robot segment.
    *      Total cost: O(N * log2(M)). 
    *    - Two-Pointer Sweep: Steps through both timelines concurrently in a single pass.
    *      Total cost: O(N + M).
    *    - Crossover point: Binary search is only faster when N is extremely small: 
    *      N < M / (log2(M) - 1).
    * 
    * 2. STATE-SPACE DEPENDENCY:
    *    - Sparse Spaces (Geometric R2 / Simple Dubins): Robot paths have very few segments 
    *      (e.g., N=1). Here, binary search wins because 1 * log2(16) < 1 + 16.
    *    - Dense Spaces (Kinodynamic / Thruster): Numerical integration creates highly dense 
    *      trajectories (e.g., N=100). Even if obstacle predictions are kept short due to 
    *      dynamic uncertainty (e.g., M=16), the temporal sweep scales much better 
    *      (100 + 16 = 116 ops) compared to repeated binary searches (100 * log2(16) = 400 ops).
    * 
    * 3. HARDWARE & MEMORY LOCALITY:
    *    - The sweep method accesses array memory strictly sequentially (idx++). Modern CPUs 
    *      heavily optimize this via L1 cache prefetching, resulting in minimal cache misses.
    *    - std::lower_bound jumps non-sequentially, frequently stalling the CPU pipeline with 
    *      cache misses, adding hidden hardware overhead not captured by pure Big-O notation.
    * 
    * "The efficiency of continuous collision detection depends heavily on the trajectory density 
    * of the state space. For simple geometric spaces where robot trajectories consist of a single 
    * segment (N=1), querying the obstacle's trajectory via temporal binary search O(N log M) is 
    * optimal. However, for complex kinodynamic models like the Thruster, numerical integration 
    * results in dense trajectories (N >> M). In these spaces, binary search overhead multiplies 
    * iteratively, making a synchronized two-pointer temporal sweep O(N+M) vastly superior both 
    * in algorithmic complexity and CPU cache locality."
    */

    size_t R_SIZE = trajectory.path_points.size();
    size_t O_SIZE = ob.predicted_path.size();
    size_t N = R_SIZE - 1; // Robot segments
    size_t M = O_SIZE - 1; // Obstacle segments
    // Compute algorithmic cost. We add a 1.5x multiplier to Binary Search to account 
    // for CPU cache misses caused by non-sequential memory access (std::lower_bound).
    double sweep_cost = static_cast<double>(N + M);
    double binary_cost = static_cast<double>(N) * std::log2(M > 1 ? M : 2) * 1.5;

    // Auto-Select: Method 3 (Binary) for R2/R2T. Method 2 (Sweep) for Dubins/Thruster.
    // (1 = Nested, 2 = Sweep, 3 = Binary Search)
    int CHECK_METHOD = (binary_cost < sweep_cost) ? 3 : 2;

    if (CHECK_METHOD == 1) {
        // O(N*M) Nested Loop (Baseline)
        for (size_t r_idx = 0; r_idx < R_SIZE - 1; ++r_idx) {
            Eigen::Vector2d P_early, P_late; double T_early, T_late;
            getRobotSegment(r_idx, P_early, P_late, T_early, T_late);

            for (size_t o_idx = 0; o_idx < O_SIZE - 1; ++o_idx) {
                Eigen::Vector2d O_early, O_late; double OT_early, OT_late;
                getObsSegment(o_idx, O_early, O_late, OT_early, OT_late);

                if (!checkSegmentMath(P_early, P_late, T_early, T_late, O_early, O_late, OT_early, OT_late)) {
                    return false; 
                }
            }
        }
    } 
    else if (CHECK_METHOD == 2) {
        // O(N+M) Temporal Sweep (DOWNWARD TIME)
        size_t r_idx = 0;
        size_t o_idx = 0;

        while (r_idx < R_SIZE - 1 && o_idx < O_SIZE - 1) {
            Eigen::Vector2d P_early, P_late, O_early, O_late; 
            double T_early, T_late, OT_early, OT_late;
            
            getRobotSegment(r_idx, P_early, P_late, T_early, T_late);
            getObsSegment(o_idx, O_early, O_late, OT_early, OT_late);

            if (!checkSegmentMath(P_early, P_late, T_early, T_late, O_early, O_late, OT_early, OT_late)) {
                // if (aabb_would_reject) std::cout << "\n[FATAL FLAW] AABB said SAFE, but continuous check found COLLISION! (Method 2)\n" << std::endl;
                return false; 
            }

            // ADVANCE POINTER LOGIC (Descending Time)
            // The segment that has the HIGHER bottom bound (early time) finishes its downward sweep first.
            if (T_early > OT_early) {
                r_idx++; 
            } else if (OT_early > T_early) {
                o_idx++; 
            } else {
                r_idx++; o_idx++; 
            }
        }
    }
    else if (CHECK_METHOD == 3) {
        // O(N log M) Binary Search (DESCENDING TIME)
        for (size_t r_idx = 0; r_idx < R_SIZE - 1; ++r_idx) {
            Eigen::Vector2d P_early, P_late; double T_early, T_late;
            getRobotSegment(r_idx, P_early, P_late, T_early, T_late);

            // 1. Binary Search for the start index
            // Because the array is descending, we look for the first point where pt.z() <= T_late
            // We use greater-than (>) as the comparator for descending arrays in lower_bound.
            auto it_start = std::lower_bound(ob.predicted_path.begin(), ob.predicted_path.end(), T_late,
                [](const Eigen::Vector3d& pt, double val) { return pt.z() > val; });
            
            size_t firstObsInd = (it_start == ob.predicted_path.end()) ? O_SIZE - 1 : std::distance(ob.predicted_path.begin(), it_start);
            
            // Step back one index to capture the segment that crosses the T_late boundary
            if (firstObsInd > 0) firstObsInd--; 

            // 2. Loop over the overlapping temporal window
            for (size_t o_idx = firstObsInd; o_idx < O_SIZE - 1; ++o_idx) {
                Eigen::Vector2d O_early, O_late; double OT_early, OT_late;
                getObsSegment(o_idx, O_early, O_late, OT_early, OT_late);

                // Early Break: If the obstacle segment time drops below the robot's window, break immediately!
                if (OT_late < T_early) {
                    break; 
                }

                if (!checkSegmentMath(P_early, P_late, T_early, T_late, O_early, O_late, OT_early, OT_late)) {
                    // if (aabb_would_reject) std::cout << "\n[FATAL FLAW] AABB said SAFE, but continuous check found COLLISION! (Method 3)\n" << std::endl;
                    return false; 
                }
            }
        }
    }

    return true; 

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
        // PURGED: visible obstacle the robot already reached. Never collision-check again.
        if (ob.is_removed) {
            continue;
        }
        // HIDDEN: static obstacle not yet within sensor range. Robot can't see it.
        if (!ob.is_dynamic && !ob.is_discovered) continue;



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

// bool DeterministicObstacleChecker::checkRobotCollision(const Eigen::Vector2d& robot_pos, double yaw) const {
//     // 1. Thread Safety Lock (CRITICAL if running in threaded executor)
//     // std::lock_guard<std::mutex> lock(obstacles_mutex_); 

//     // 2. DIAGNOSTIC: Check if we even know about obstacles
//     if (obstacle_positions_.empty()) {
//         RCLCPP_WARN_THROTTLE(rclcpp::get_logger("CollisionCheck"), *clock_, 1000, 
//             "[CollisionCheck] SKIPPING: 0 obstacles known! (Is ProcessLatestPoseInfo running?)");
//         return false;
//     }


//     // Using inflation as robot radius
//     double robot_r = inflation; 

//     for (const auto& ob : obstacle_positions_) {
//         // Calculate Distance
//         double dist = (robot_pos - ob.position).norm();
        
//         // Calculate Obstacle Radius (Circle vs Box approximation)
//         double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
//                        std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);

//         double collision_threshold = robot_r + obs_r;

//         // // 4. LOGGING: Print details if we are getting close (within 3 meters of collision)
//         // if (dist < (collision_threshold + 3.0)) {
//         //     RCLCPP_INFO_THROTTLE(rclcpp::get_logger("CollisionDebug"), *clock_, 200, 
//         //         "NEAR MISS: Robot(%.2f, %.2f) vs [%s](%.2f, %.2f) \n"
//         //         "    -> Dist: %.3f | Threshold: %.3f (Rob:%.1f + Obs:%.1f)",
//         //         robot_pos.x(), robot_pos.y(), 
//         //         ob.name.c_str(), ob.position.x(), ob.position.y(), 
//         //         dist, collision_threshold, robot_r, obs_r);
//         // }

//         // 5. The Check
//         if (dist < collision_threshold) {
//             RCLCPP_ERROR(rclcpp::get_logger("CollisionCheck"), 
//                 "!!! CRASH DETECTED !!! Robot vs [%s] | Dist: %.3f < %.3f",
//                 ob.name.c_str(), dist, collision_threshold);
//             return true;
//         }
//     }
//     return false;
// }


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
        
        // ---------------------------------------------------------
        // CIRCLE COLLISION CHECK
        // ---------------------------------------------------------
        if (ob.type == Obstacle::CIRCLE) {
            double dist = (robot_pos - ob.position).norm();
            double collision_threshold = robot_r + ob.dimensions.radius;
            
            if (dist < collision_threshold) {
                RCLCPP_ERROR(rclcpp::get_logger("CollisionCheck"), 
                    "!!! CRASH DETECTED !!! Robot vs CIRCLE [%s] | Dist: %.3f < %.3f",
                    ob.name.c_str(), dist, collision_threshold);
                return true;
            }
        }
        // ---------------------------------------------------------
        // ORIENTED BOX (OBB) COLLISION CHECK
        // ---------------------------------------------------------
        else if (ob.type == Obstacle::BOX) {
            // 1. Inflate the box dimensions by the robot's radius
            double eff_width  = ob.dimensions.width  + (2.0 * robot_r);
            double eff_height = ob.dimensions.height + (2.0 * robot_r);

            // 2. Translate robot relative to the box's center
            double dx = robot_pos.x() - ob.position.x();
            double dy = robot_pos.y() - ob.position.y();

            // 3. Rotate the robot's relative position by the INVERSE of the box's rotation.
            // This brings the robot into the local, axis-aligned coordinate frame of the box.
            double cos_theta = std::cos(-ob.dimensions.rotation);
            double sin_theta = std::sin(-ob.dimensions.rotation);
            
            double local_x = (dx * cos_theta) - (dy * sin_theta);
            double local_y = (dx * sin_theta) + (dy * cos_theta);

            // 4. Axis-Aligned Bounding Box (AABB) check in local frame
            double half_width = eff_width / 2.0;
            double half_height = eff_height / 2.0;

            if (std::abs(local_x) < half_width && std::abs(local_y) < half_height) {
                RCLCPP_ERROR(rclcpp::get_logger("CollisionCheck"), 
                    "!!! CRASH DETECTED !!! Robot vs BOX [%s] | Local Pos: (%.2f, %.2f)",
                    ob.name.c_str(), local_x, local_y);
                return true;
            }
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

// std::vector<Eigen::Vector3d> DeterministicObstacleChecker::generatePrediction(
//     const Obstacle& ob, 
//     double currentTime) const 
// {
//     std::vector<Eigen::Vector3d> path;

//     // --- GEOMETRIC MODE ---
//     // We don't care about time or future movement. 
//     // We just need the obstacle's current position for the collision checker.
//     if (is_geometric_mode_) {
//         // We use Z=0.0 for the time component. 
//         // The collision checker (isTrajectorySafeAgainstSingleObstacle) will ignore this Z value 
//         // and treat the obstacle as a static circle at (X, Y).
//         path.emplace_back(ob.position.x(), ob.position.y(), 0.0);

//         // THE FIX: Initialize the AABB bounds for a stationary point
//         ob.min_x = ob.position.x();
//         ob.max_x = ob.position.x();
//         ob.min_y = ob.position.y();
//         ob.max_y = ob.position.y();


//         return path;
//     }


    
//     if (!ob.is_dynamic || !ob.has_ground_truth) return path;

//     // 1. Calculate Effective Radius
//     double R_eff = 0.0;
//     if (ob.type == Obstacle::CIRCLE) {
//         R_eff = ob.dimensions.radius;
//     } else {
//         // For Box, use the radius of the circumscribed circle (half diagonal)
//         // This ensures we cover the corners even if it rotates (though yours is linear)
//         R_eff = std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
//     }

//     // 2. Get Speed (Scalar magnitude of velocity vector)
//     double speed = ob.velocity.norm();

//     // 3. Calculate Adaptive DT
//     // We add a small safety factor (e.g., 0.8) to ensure the circles overlap slightly
//     // rather than just touching. This prevents "edge cases" (pun intended).
//     double dt_step = 0.1; // Default fallback
    
//     if (speed > 1e-6) {
//         // Formula: dt = (2 * Radius) / Speed
//         // We clamp it to a minimum (e.g., 0.05) to prevent infinite loops if speed is huge,
//         // and a maximum (e.g., 0.5) to prevent too sparse samples for very slow objects.
//         double calculated_dt = (2.0 * R_eff) / speed;
//         // Clamp values to keep sanity
//         dt_step = std::clamp(calculated_dt, 0.05, 1.0);
//     } else {
//         // Static obstacle (or very slow)
//         dt_step = 0.5; // Don't need many samples if it's not moving
//     }

//     // 4. Generate Path
//     Eigen::Vector2d predicted_pos = ob.position;
//     Eigen::Vector2d current_v = ob.velocity;

//     for (double t = currentTime; t >= -1e-9; t -= dt_step) {
//         path.emplace_back(predicted_pos.x(), predicted_pos.y(), t);
//         predicted_pos = predicted_pos + (current_v * dt_step);
//     }
    
//     ob.min_x = std::numeric_limits<double>::max();
//     ob.max_x = std::numeric_limits<double>::lowest();
//     ob.min_y = std::numeric_limits<double>::max();
//     ob.max_y = std::numeric_limits<double>::lowest();

//     for (const auto& p : path) {
//         if (p.x() < ob.min_x) ob.min_x = p.x();
//         if (p.x() > ob.max_x) ob.max_x = p.x();
//         if (p.y() < ob.min_y) ob.min_y = p.y();
//         if (p.y() > ob.max_y) ob.max_y = p.y();
//     }



//     return path;
// }

// ObstacleVector DeterministicObstacleChecker::checkAndRepairObstacles(double T_robot) {
//     ObstacleVector triggered_obs;
    
//     for (auto& [name, ob] : obstacle_positions_map_) {
//         if (!ob.is_dynamic) continue;
        
//         if (!ob.is_initialized_in_graph) {
//             ob.is_initialized_in_graph = true;
//             // Generate prediction starting from current T_robot
//             ob.predicted_path = this->generatePrediction(ob, T_robot);
//             triggered_obs.push_back(ob);
            
//             // Schedule the NEXT turnaround
//             double time_for_one_leg = ob.motion_limit / ob.speed_scalar;
//             ob.nextDirectionChangeTime = T_robot - time_for_one_leg; 
            
//             continue; 
//         }
        
//         // --- FIX: Check for Turnaround ---
//         // We want to trigger if we have PASSED the scheduled time.
//         // Since T_robot decreases (e.g., 4.30 -> 4.28), we check if T_robot is less than or equal to the scheduled time.
//         // However, to avoid triggering multiple times for the same event, we check if we crossed the boundary.
        
//         // Simple robust check: If current time is PAST the scheduled time, update.
//         if (T_robot <= ob.nextDirectionChangeTime) {
            
//             // Calculate Sim Time for logging
//             double sim_time = initial_budget_time_ - T_robot; 
            
//             RCLCPP_WARN(rclcpp::get_logger("Obs_Debug"),
//                 "!!! TURNAROUND [%s] !!! | T_Robot: %.2f | Scheduled: %.2f", 
//                 name.c_str(), T_robot, ob.nextDirectionChangeTime);
                
//             // --- UPDATE SCHEDULE ---
//             // Schedule the NEXT turnaround (one leg further into the past)
//             double time_for_one_leg = ob.motion_limit / ob.speed_scalar;
//             ob.nextDirectionChangeTime -= time_for_one_leg;
            
//             // --- UPDATE PREDICTION ---
//             // CRITICAL: We must regenerate the prediction based on the NEW direction.
//             // generatePrediction uses ob.velocity. We must update ob.velocity FIRST.
            
//             // 1. Update the Obstacle's internal state (Position & Velocity) to match the new leg
//             // We use the exact same logic as processLatestPoseInfo to find the new velocity
//             double cycle_time = 2.0 * time_for_one_leg;
//             double cycle_position = std::fmod(sim_time, cycle_time);
            
//             if (cycle_position <= time_for_one_leg) {
//                 // Forward leg
//                 ob.velocity = ob.motion_axis * ob.speed_scalar;
//             } else {
//                 // Backward leg
//                 ob.velocity = ob.motion_axis * (-ob.speed_scalar);
//             }
            
//             // 2. Now generate the prediction using this NEW velocity
//             ob.predicted_path = this->generatePrediction(ob, T_robot);
            
//             triggered_obs.push_back(ob);
//         }
//     }
//     return triggered_obs;
// }


/////////////////////////////////////////////////////////////////////
ObstacleVector DeterministicObstacleChecker::checkAndRepairObstacles(double T_robot, const Eigen::Vector2d& robot_pos) {
    ObstacleVector triggered_obs;
    
    // Identify if this is the absolute first tick of the simulation
    bool is_first_slice = (std::abs(T_robot - initial_budget_time_) < 1e-6);

    for (auto& [name, ob] : obstacle_positions_map_) {
        
        // ==========================================
        // 1. STATIC OBSTACLE LOGIC
        // ==========================================
        // if (!ob.is_dynamic) {
        //     // ONLY initialize static obstacles on the very first simulation tick
        //     if (is_first_slice && !ob.is_initialized_in_graph) {
        //         ob.is_initialized_in_graph = true;
        //         ob.predicted_path = this->generatePrediction(ob, T_robot);
        //         triggered_obs.push_back(ob);
        //     }
        //     continue; // Skip static obstacles forever after Day 0
        // }


        if (ob.is_removed) continue;            // already purged + signaled

        if (!ob.is_dynamic) {
            if (ob.initially_visible) {
                if (!ob.is_initialized_in_graph) {
                    ob.is_initialized_in_graph = true;
                    /*
                        Very important to not get confused! why do we even use generateprediction for static obstalces? dnot we use the spatial
                        geometry for static obstalces in isTrajecotyrSafe func? 
                        dont confuse the isgeometricmode with static obstacles! these are different things! for geometric mode we only care about the spatial position of the obstalce
                        and we explicitly handle this in updateobstalce functions of the the planners (e.g. FMTX) but what if we wanted to use R2T with static obstacles?
                        we need to have a tube for static obstalces too 
                        (with all the predicted pose being the same for all the time duration from initial time budget to zero! I still am not sure why use the current T_robot wont work for static obs there must be a bug somewhere else in istrajectory function maybe ?!)
                        so we fill the tube just for addNewobstalce and removeObstacle KDTREE implmentaiton that uses predicted path!
                        so Eiter i have to explicitly handle late discovered static obstacles differently in addnewobstalce/removeobstalce or use initial_budget_time_ in generate prediction for static obstalces!
                        So basically this is the example that will reveal whats what:
                        If the obstacle is a static wall, you want to orphan every node that sits inside its $(X,Y)$ footprint, 
                        regardless of its time $T_n$.
                        If you use a single query point at $T_{robot}$, and a node exists inside the wall at $T_n = 10.0$ (while $T_{robot} = 40.0$), 
                        the temporal difference $(40.0 - 10.0)^2 = 900$ is added to the distance.
                        The KD-tree thinks this node is $30$ units away, even though it is physically 
                        sitting exactly on top of the obstacle in 2D space! It will not orphan the node.
                        So either do use the intial_budget_time_ for static obs generateprediction or use another KD-TREE that doesnt use time dimension
                        for static obstalces!

                        SO: 

                        Static Obstacles use initial_budget_time_: 
                        Because they never move, their physical footprint is permanently deadly. We erase them from the entire timeline 
                        so the math never gets confused by them.

                        Dynamic Obstacles use T_robot: 
                        Because they move, their footprint is only deadly at specific times in the future. 
                        We project their danger forward (from $T_{robot}$ down to $0$) and leave the past alone, because projecting dynamic movement backward into the past is invalid physics and unnecessary for future collision avoidance.

                    */
                    ob.predicted_path = generatePrediction(ob, initial_budget_time_); //Dont use T robot!
                    triggered_obs.push_back(ob);                 // Day-0 seed
                }
                else {
                    // double dist = (ob.position - robot_pos).norm();
                    // if (dist <= sensor_range_) {
                    //     ob.is_removed = true;                    // purge
                    //     triggered_obs.push_back(ob);             // FINAL signal, carries Day-0 path
                    // }
                }
            }
            else {
                if (!ob.is_discovered) {
                    double dist = (ob.position - robot_pos).norm();
                    if (dist <= sensor_range_) {
                        ob.is_discovered = true;
                        ob.predicted_path = generatePrediction(ob, initial_budget_time_); // IMPORTANT: a static obstacle is static in space, so its pillar must be static across the entire time axis.
                        triggered_obs.push_back(ob);
                    }
                }
            }
            continue;
        }







        // ==========================================
        // 2. DYNAMIC OBSTACLE LOGIC
        // ==========================================
        if (!ob.is_initialized_in_graph) {
            ob.is_initialized_in_graph = true;
            // Generate full horizon prediction starting from current T_robot
            ob.predicted_path = this->generatePrediction(ob, T_robot);
            triggered_obs.push_back(ob);
            
            // Schedule the NEXT turnaround
            double time_for_one_leg = ob.motion_limit / ob.speed_scalar;
            ob.nextDirectionChangeTime = T_robot - time_for_one_leg; 
            
            continue; 
        }
        
        // --- Check for Turnaround Event ---
        if (T_robot <= ob.nextDirectionChangeTime) {
            
            // Calculate Sim Time for logic and logging
            double sim_time = initial_budget_time_ - T_robot; 
            
            RCLCPP_WARN(rclcpp::get_logger("Obs_Debug"),
                "!!! TURNAROUND [%s] !!! | T_Robot: %.2f | Scheduled: %.2f", 
                name.c_str(), T_robot, ob.nextDirectionChangeTime);
                
            // Schedule the NEXT turnaround (one leg further into the past)
            double time_for_one_leg = ob.motion_limit / ob.speed_scalar;
            ob.nextDirectionChangeTime -= time_for_one_leg;
            
            // Update the Obstacle's Velocity based on the new leg
            double cycle_time = 2.0 * time_for_one_leg;
            double cycle_position = std::fmod(sim_time, cycle_time);
            
            if (cycle_position <= time_for_one_leg) {
                ob.velocity = ob.motion_axis * ob.speed_scalar; // Forward
            } else {
                ob.velocity = ob.motion_axis * (-ob.speed_scalar); // Backward
            }
            
            // Now generate the NEW full horizon prediction using this NEW velocity
            ob.predicted_path = this->generatePrediction(ob, T_robot);
            triggered_obs.push_back(ob);
        }
    }
    return triggered_obs;
}


std::vector<Eigen::Vector3d> DeterministicObstacleChecker::generatePrediction(
    const Obstacle& ob, 
    double currentTime) const 
{
    std::vector<Eigen::Vector3d> path;

    // --- GEOMETRIC MODE ---
    if (is_geometric_mode_) {
        // Geometric mode treats all obstacles as 2D static points in the KD-Tree
        path.emplace_back(ob.position.x(), ob.position.y(), 0.0);
        ob.min_x = ob.position.x();
        ob.max_x = ob.position.x();
        ob.min_y = ob.position.y();
        ob.max_y = ob.position.y();
        return path;
    }
    
    if (!ob.has_ground_truth) return path;

    // 1. Calculate Effective Radius
    double R_eff = 0.0;
    if (ob.type == Obstacle::CIRCLE) {
        R_eff = ob.dimensions.radius;
    } else {
        R_eff = std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    }

    // 2. Get Speed (Scalar magnitude of velocity vector)
    double speed = ob.velocity.norm();

    // 3. Calculate Adaptive DT
    double dt_step = 0.1; 
    if (speed > 1e-6) {
        double calculated_dt = (2.0 * R_eff) / speed;
        dt_step = std::clamp(calculated_dt, 0.05, 1.0);
    } else {
        // If speed is 0 (Static Obstacle), take larger steps to build a time pillar efficiently
        dt_step = 0.5; 
    }

    // 4. Generate Path (FULL HORIZON down to 0.0)
    Eigen::Vector2d predicted_pos = ob.position;
    Eigen::Vector2d current_v = ob.velocity;

    // For static obstacles, current_v is (0,0), so this loop just stacks points at the 
    // exact same X,Y position all the way down the time axis, creating a solid pillar.
    for (double t = currentTime; t >= -1e-9; t -= dt_step) {
        path.emplace_back(predicted_pos.x(), predicted_pos.y(), t);
        predicted_pos = predicted_pos + (current_v * dt_step);
    }
    
    // 5. Update Bounding Box
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


// /*

// You are asking exactly the right questions about how Receding Horizon Control (MPC) is supposed to work.

// It might feel redundant, but the UPDATE_INTERVAL (sliding window) is the core mechanism that keeps the robot safe without blowing up your CPU.

// Here is exactly why we cannot regenerate the tube every iteration, and why we cannot wait the full 2.5 seconds to regenerate it.
// The "Constant Replanning" Problem (Why not every iteration?)

// Yes, checkAndRepairObstacles is called in every single simulation slice (e.g., every 0.1 seconds).
// If we generate a new prediction tube in every single slice:

//     FMTX would call removeObstacle and addNewObstacle 10 times a second.

//     FMTX would run the heavy plan() wavefront repair 10 times a second.

//     You would be deleting and recreating almost the exact same space-time tube over and over, wasting massive amounts of CPU and making your latency graphs look terrible.

// The "Blind Spot" Problem (Why not wait the full 2.5s?)

// You suggested: "we just need to check here if 2.5 second horizon has passed or not to regnerate the tube"

// If we wait for the full 2.5 seconds to pass before generating a new tube, watch what happens to the planner's "vision":

//     At T=10.0: We generate a tube down to T=7.5. The planner sees 2.5 seconds into the future. Perfect.

//     At T=8.5: The robot has moved. The tube still only goes down to T=7.5. The planner now only sees 1.0 second into the future.

//     At T=7.6: The planner only sees 0.1 seconds into the future! The robot might plan a path that drives right up to the obstacle, assuming the obstacle magically vanishes at T=7.5.

//     At T=7.5: The horizon expires, we finally generate a new tube. But it's too late—the robot is already on a collision course because it was planning with a "blind spot" for the last second.

// The Solution: The Sliding Window (Receding Horizon)

// To fix this, we need the planner to always see at least 1.5 seconds into the future, but we don't want to run plan() 10 times a second.

// This is why we use an UPDATE_INTERVAL (e.g., 1.0s) that is smaller than the PREDICTION_HORIZON (2.5s).

//     At T=10.0: Generate tube to 7.5. (Vision: 2.5s ahead)

//     At T=9.0: The timer fires! We delete the old tube and generate a new one to 6.5. (Vision was dropping to 1.5s, but now it's back to 2.5s ahead).

//     At T=8.0: The timer fires! Generate new tube to 5.5.

// Why this is elegant for FMTX:

//     Safety: The robot never goes blind. It always has between 1.5s and 2.5s of guaranteed space-time invalidation in front of it.

//     Efficiency: updateObstacles and plan() only run once per second (or instantly if an oracle turnaround event happens). Your CPU stays relaxed, and your baseline average planning time drops drastically.

// This UPDATE_INTERVAL vs PREDICTION_HORIZON ratio is a foundational concept in receding horizon planners. In your paper, you can boast that FMTX uses a "1.0s sliding update with a 2.5s prediction horizon to guarantee minimum look-ahead safety while bounding computational frequency.



// However, the reason you cannot wait the full 2.5 seconds to call updateObstacles is a fatal mathematical flaw called the Prediction Blind Spot.

// Since your plan() is running in every single iteration, it is constantly looking at the graph to find the optimal path. If you only regenerate the tube when the 2.5 seconds expire, you are feeding the planner a shrinking window of the future, which will cause the robot to crash.
// The "Shrinking Headlights" Problem

// Think of the 2.5s prediction tube as the robot's headlights.

// If you update every 2.5 seconds:

//     T_robot = 10.0: You generate a 2.5s tube down to $T=7.5$. Your plan() function runs and safely avoids the obstacle because it sees 2.5 seconds into the future.

//     T_robot = 8.5: You are in the main loop. plan() runs to add new samples. It looks at the graph. The obstacle tube currently in the graph ends at $T=7.5$. That is only 1.0 second in the future! The planner assumes that after $T=7.5$, the obstacle ceases to exist. It will happily route the robot right through the obstacle's future path.

//     T_robot = 7.6: plan() runs. The tube in the graph ends at $T=7.5$. The planner can only see 0.1 seconds into the future. The robot is practically blind and driving at high speed.

//     T_robot = 7.5: The 2.5s timer finally expires. You call updateObstacles and generate a new tube. But it's too late—the robot already committed to a trajectory that crashes into the obstacle because it couldn't see it coming during the last 1.5 seconds.

// Why the 1.0s Sliding Window (Update Interval) is Mandatory

// To keep the robot safe, plan() must never be allowed to look at a graph that has less than a safe braking distance of future prediction.

// By setting UPDATE_INTERVAL = 1.0 and HORIZON = 2.5, you create an overlapping receding horizon.

//     T_robot = 10.0: Tube goes to $T=7.5$ (Vision: 2.5s)

//     T_robot = 9.5: Tube goes to $T=7.5$ (Vision: 2.0s)

//     T_robot = 9.0: The 1.0s interval triggers! You call updateObstacles. The old tube is deleted, and a new 2.5s tube is generated down to $T=6.5$. (Vision jumps back up to 2.5s).

// */



// ObstacleVector DeterministicObstacleChecker::checkAndRepairObstacles(double T_robot) {
//     ObstacleVector triggered_obs;
    
//     // --- FINITE HORIZON PERIODIC TRIGGER ---
//     // We need to periodically slide the short 2.5s prediction tube forward.
//     static double last_periodic_update_T_robot = initial_budget_time_; 
//     const double UPDATE_INTERVAL = 1.0; // Trigger a new tube every 1.0 seconds
    
//     bool global_periodic_trigger = false;
//     if (last_periodic_update_T_robot - T_robot >= UPDATE_INTERVAL) {
//         global_periodic_trigger = true;
//         last_periodic_update_T_robot = T_robot;
//     }

//     // Identify if this is the absolute first tick of the simulation
//     bool is_first_slice = (std::abs(T_robot - initial_budget_time_) < 1e-6);

//     for (auto& [name, ob] : obstacle_positions_map_) {
        
//         // ==========================================
//         // 1. STATIC OBSTACLE LOGIC
//         // ==========================================
//         if (!ob.is_dynamic) {
//             // ONLY initialize static obstacles on the very first simulation tick
//             if (is_first_slice && !ob.is_initialized_in_graph) {
//                 ob.is_initialized_in_graph = true;
//                 ob.predicted_path = this->generatePrediction(ob, T_robot);
//                 triggered_obs.push_back(ob);
//             }
//             continue; // Skip static obstacles forever after Day 0
//         }

//         // ==========================================
//         // 2. DYNAMIC OBSTACLE LOGIC
//         // ==========================================
//         bool turnaround_triggered = false;

//         // A. Initial Graph Setup for Dynamics
//         if (!ob.is_initialized_in_graph) {
//             ob.is_initialized_in_graph = true;
//             ob.predicted_path = this->generatePrediction(ob, T_robot);
//             triggered_obs.push_back(ob);
            
//             // Schedule the NEXT turnaround
//             double time_for_one_leg = ob.motion_limit / ob.speed_scalar;
//             ob.nextDirectionChangeTime = T_robot - time_for_one_leg; 
            
//             continue; 
//         }
        
//         // B. ORACLE EVENT: Check for Turnaround
//         if (T_robot <= ob.nextDirectionChangeTime) {
//             turnaround_triggered = true;
            
//             // Calculate ascending Sim Time for standard modulo math
//             double sim_time = initial_budget_time_ - T_robot; 
            
//             RCLCPP_WARN(rclcpp::get_logger("Obs_Debug"),
//                 "!!! TURNAROUND [%s] !!! | T_Robot: %.2f | Scheduled: %.2f", 
//                 name.c_str(), T_robot, ob.nextDirectionChangeTime);
                
//             // Schedule the NEXT turnaround (one leg further down in T_robot time)
//             double time_for_one_leg = ob.motion_limit / ob.speed_scalar;
//             ob.nextDirectionChangeTime -= time_for_one_leg;
            
//             // Update the Oracle Truth (Flip the velocity)
//             double cycle_time = 2.0 * time_for_one_leg;
//             double cycle_position = std::fmod(sim_time, cycle_time);
            
//             if (cycle_position <= time_for_one_leg) {
//                 ob.velocity = ob.motion_axis * ob.speed_scalar; // Forward
//             } else {
//                 ob.velocity = ob.motion_axis * (-ob.speed_scalar); // Backward
//             }
//         }

//         // C. TUBE GENERATION: Trigger if sliding horizon OR oracle event occurred
//         if (global_periodic_trigger || turnaround_triggered) {
//             // Generate the prediction using the CURRENT velocity (constant velocity assumption)
//             ob.predicted_path = this->generatePrediction(ob, T_robot);
//             triggered_obs.push_back(ob);
//         }
//     }
    
//     return triggered_obs;
// }

// std::vector<Eigen::Vector3d> DeterministicObstacleChecker::generatePrediction(
//     const Obstacle& ob, 
//     double currentTime) const 
// {
//     std::vector<Eigen::Vector3d> path;

//     // --- GEOMETRIC MODE ---
//     if (is_geometric_mode_) {
//         // Geometric mode treats all obstacles as 2D static points in the KD-Tree
//         path.emplace_back(ob.position.x(), ob.position.y(), 0.0);
//         ob.min_x = ob.position.x();
//         ob.max_x = ob.position.x();
//         ob.min_y = ob.position.y();
//         ob.max_y = ob.position.y();
//         return path;
//     }
    
//     // Safety check (Static obstacles use their current initial position)
//     if (!ob.is_dynamic || !ob.has_ground_truth) {
//         path.emplace_back(ob.position.x(), ob.position.y(), currentTime);
//         ob.min_x = ob.position.x();
//         ob.max_x = ob.position.x();
//         ob.min_y = ob.position.y();
//         ob.max_y = ob.position.y();
//         return path;
//     }

//     // 1. Calculate Effective Radius
//     double R_eff = 0.0;
//     if (ob.type == Obstacle::CIRCLE) {
//         R_eff = ob.dimensions.radius;
//     } else {
//         R_eff = std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
//     }

//     // 2. Get Speed
//     double speed = ob.velocity.norm();

//     // 3. Calculate Adaptive DT
//     double dt_step = 0.1; 
//     if (speed > 1e-6) {
//         double calculated_dt = (2.0 * R_eff) / speed;
//         dt_step = std::clamp(calculated_dt, 0.05, 1.0);
//     } else {
//         dt_step = 0.5; 
//     }

//     // 4. Generate Path (FINITE HORIZON + CONSTANT VELOCITY)
//     Eigen::Vector2d predicted_pos = ob.position;
//     Eigen::Vector2d current_v = ob.velocity;

//     // --- HORIZON SETTING ---
//     const double PREDICTION_HORIZON = 2.5; 
//     double end_time = std::max(-1e-9, currentTime - PREDICTION_HORIZON);

//     // Constant Velocity Assumption: Project a straight line
//     for (double t = currentTime; t >= end_time; t -= dt_step) {
//         path.emplace_back(predicted_pos.x(), predicted_pos.y(), t);
//         predicted_pos = predicted_pos + (current_v * dt_step);
//     }
    
//     // 5. Update Bounding Box
//     ob.min_x = std::numeric_limits<double>::max();
//     ob.max_x = std::numeric_limits<double>::lowest();
//     ob.min_y = std::numeric_limits<double>::max();
//     ob.max_y = std::numeric_limits<double>::lowest();

//     for (const auto& p : path) {
//         if (p.x() < ob.min_x) ob.min_x = p.x();
//         if (p.x() > ob.max_x) ob.max_x = p.x();
//         if (p.y() < ob.min_y) ob.min_y = p.y();
//         if (p.y() > ob.max_y) ob.max_y = p.y();
//     }

//     return path;
// }