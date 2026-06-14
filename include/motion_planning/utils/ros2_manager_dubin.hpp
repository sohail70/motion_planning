// Copyright 2025 Soheil E.nia
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "motion_planning/utils/obstacle_checker.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/deterministic_obstacle_checker.hpp"
#include "motion_planning/utils/params.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <chrono>
#include <vector>
#include <algorithm>
#include <utility> // for std::pair
#include "motion_planning/utils/ros2_manager_base.hpp"

class DubinsROS2Manager : public ROS2ManagerBase {
public:
    DubinsROS2Manager(
        std::shared_ptr<ObstacleChecker> obstacle_checker,
        std::shared_ptr<RVizVisualization> visualizer,
        const Params& params,
        const Eigen::VectorXd& initial_sim_state,
        double initial_budget_time) 
        // : Node("dubins_ros2_manager", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)})),
        : ROS2ManagerBase("dubins_ros2_manager", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)})),
          obstacle_checker_(obstacle_checker),
          visualizer_(visualizer),
          is_path_set_(false),
          initial_budget_time_(initial_budget_time)
    {
        inflation = params.getParam<double>("inflation");
        if (initial_sim_state.size() != 4) {
            throw std::runtime_error("DubinsROS2Manager: Initial state must be 4D (x, y, theta, t).");
        }
        current_interpolated_state_ = initial_sim_state;
        current_sim_time_ = initial_sim_state(3); // Time is at index 3 for Dubins
        slice_time = params.getParam<double>("slice_time");
        
        int vis_frequency_hz = params.getParam<int>("vis_frequency_hz", 30);
        
        // Only start the visualization timer. 
        // Simulation is now driven manually by stepSimulation().
        if (vis_frequency_hz!=0){
            vis_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000 / vis_frequency_hz),
                std::bind(&DubinsROS2Manager::visualizationLoop, this));
        }
            
        RCLCPP_INFO(this->get_logger(), "Initialized DubinsROS2Manager (Manual Sim Mode).");
    }

    void stepStationary(double dt) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        current_sim_time_ -= dt; 
        current_interpolated_state_(3) = current_sim_time_;
    }

    // // --- PUBLIC API ---
    // void stepSimulation(double dt) {
    //     std::lock_guard<std::mutex> lock(path_mutex_);
        
    //     // 1. Advance Time
    //     current_sim_time_ -= dt;
        
    //     // 2. Safety Clamps
    //     if (!current_path_.empty()) {
    //         // For Dubins, time is at index 3
    //         if (current_sim_time_ > current_path_.front()(3)) {
    //             current_sim_time_ = current_path_.front()(3);
    //         }
    //         if (current_sim_time_ < current_path_.back()(3)) {
    //             current_sim_time_ = current_path_.back()(3);
    //         }
    //     }

    //     if (!is_path_set_ || current_path_.size() < 2) return;

    //     // 3. Find Segment & Interpolate
    //     auto it_after = std::lower_bound(current_path_.begin(), current_path_.end(), current_sim_time_,
    //         [](const Eigen::VectorXd& point, double time) {
    //             return point(3) > time; // Compare using index 3
    //         });

    //     if (it_after == current_path_.begin()) it_after++;
    //     if (it_after == current_path_.end()) return;
        
    //     auto it_before = std::prev(it_after);
    //     const Eigen::VectorXd& state_before = *it_before;
    //     const Eigen::VectorXd& state_after = *it_after;
        
    //     double segment_duration = state_before(3) - state_after(3);
        
    //     // Store previous state BEFORE updating current_interpolated_state_
    //     Eigen::VectorXd prev_state = current_interpolated_state_;

    //     // Interpolate
    //     if (segment_duration <= 1e-9) {
    //         current_interpolated_state_ = state_after;
    //     } else {
    //         double time_into_segment = state_before(3) - current_sim_time_;
    //         double interp_factor = time_into_segment / segment_duration;
            
    //         // Position Interpolation
    //         current_interpolated_state_.head<2>() = state_before.head<2>() + interp_factor * (state_after.head<2>() - state_before.head<2>());
            
    //         // --- DUBINS SPECIFIC: Angle Interpolation ---
    //         double theta_before = state_before(2);
    //         double theta_after = state_after(2);
    //         double angle_diff = normalizeAngle(theta_after - theta_before);
    //         current_interpolated_state_(2) = normalizeAngle(theta_before + interp_factor * angle_diff);
            
    //         // Time Update
    //         current_interpolated_state_(3) = current_sim_time_;
    //     }

    //     // 4. Collision Check
    //     auto gazebo_checker = std::dynamic_pointer_cast<DeterministicObstacleChecker>(obstacle_checker_);
    //     if (gazebo_checker) {
    //         double current_planner_time = this->getCurrentSimTime(); 
    //         double current_sim_time = initial_budget_time_ - current_planner_time;
    //         gazebo_checker->processLatestPoseInfo(current_sim_time); 

    //         Eigen::Vector2d current_pos = current_interpolated_state_.head<2>();
    //         double current_yaw = current_interpolated_state_(2); // Get yaw from state
            
    //         bool is_colliding_now = gazebo_checker->checkRobotCollision(current_pos, current_yaw);
            
    //         if (is_colliding_now && !is_in_collision_state_) {
    //             collision_count_++;
    //             // Collect additional info
    //             std::vector<std::string> colliding_obstacles;
    //             const auto& all_obstacles = gazebo_checker->getObstaclePositions();
    //             for (const auto& obs : all_obstacles) {
    //                 Eigen::Vector2d obs_pos(obs.position.x(), obs.position.y());
    //                 double dist = (current_pos - obs_pos).norm();
    //                 if (dist < obs.dimensions.radius + inflation) {
    //                     colliding_obstacles.push_back(obs.name);
    //                 }
    //             }
    //             RCLCPP_FATAL(this->get_logger(),
    //                         "XXX COLLISION DETECTED XXX At T=%.2f | Total Collisions: %d\n"
    //                         "Robot Pos: [%.3f, %.3f] | Theta: %.3f rad\n"
    //                         "Colliding Obstacles: %s",
    //                         current_sim_time_,
    //                         collision_count_.load(),
    //                         current_pos.x(), current_pos.y(), current_yaw,
    //                         colliding_obstacles.empty() ? "NONE" : colliding_obstacles.front().c_str()
    //             );
    //         }
    //         is_in_collision_state_ = is_colliding_now;
    //     }

    //     // 5. Update Trace (Edges)
    //     // Create a 3D point for start and end (Z=0)
    //     Eigen::VectorXd start_pt(3); start_pt << prev_state(0), prev_state(1), 0.0;
    //     Eigen::VectorXd end_pt(3);   end_pt << current_interpolated_state_(0), current_interpolated_state_(1), 0.0;
    //     robot_trace_edges_.push_back({start_pt, end_pt});
    // }


    void stepSimulation(double dt) {
        std::lock_guard<std::mutex> lock(path_mutex_);   // lock ONCE, here
        const double max_sub = slice_time;                      // collision-check resolution
        int n = std::max(1, (int)std::ceil(dt / max_sub));
        double sub = dt / n;
        for (int i = 0; i < n; ++i) {
            stepOnce(sub);
        }
    }
    void stepOnce(double dt) {
        // std::lock_guard<std::mutex> lock(path_mutex_);

        const double TIME_EPS = 1e-9;

        // 1. Advance time
        current_sim_time_ -= dt;

        if (!is_path_set_ || current_path_.size() < 2) {
            current_interpolated_state_(3) = current_sim_time_;
            return;
        }

        // 2. Clamp time to path range
        if (current_sim_time_ > current_path_.front()(3)) {
            current_sim_time_ = current_path_.front()(3);
        }
        if (current_sim_time_ < current_path_.back()(3)) {
            current_sim_time_ = current_path_.back()(3);
        }

        Eigen::VectorXd prev_state = current_interpolated_state_;

        // 3. Find segment manually and robustly
        size_t seg_idx = current_path_.size() - 2;
        bool found = false;

        for (size_t i = 0; i + 1 < current_path_.size(); ++i) {
            const double t0 = current_path_[i](3);     // later
            const double t1 = current_path_[i + 1](3); // earlier

            if (t0 + TIME_EPS >= current_sim_time_ &&
                current_sim_time_ + TIME_EPS >= t1) {
                seg_idx = i;
                found = true;
                break;
            }
        }

        if (!found) {
            if (current_sim_time_ >= current_path_.front()(3) - TIME_EPS) {
                current_interpolated_state_ = current_path_.front();
                current_interpolated_state_(3) = current_sim_time_;
            } else {
                current_interpolated_state_ = current_path_.back();
                current_interpolated_state_(3) = current_sim_time_;
            }
        } else {
            const Eigen::VectorXd& state_before = current_path_[seg_idx];
            const Eigen::VectorXd& state_after  = current_path_[seg_idx + 1];

            const double t_before = state_before(3);
            const double t_after  = state_after(3);
            const double segment_duration = t_before - t_after;

            // Compute speed for this segment
            double segment_dist = (state_after.head<2>() - state_before.head<2>()).norm();
            if (segment_duration > TIME_EPS) {
                current_speed_ = segment_dist / segment_duration;
            } else {
                current_speed_ = 0.0;
            }

            // compute angular velocity from heading change
            double dtheta = normalizeAngle(state_after(2) - state_before(2));
            current_angular_speed_ = (segment_duration > TIME_EPS) ? (dtheta / segment_duration) : 0.0;


            

            if (segment_duration <= TIME_EPS) {
                current_interpolated_state_ =
                    (std::abs(current_sim_time_ - t_before) <= std::abs(current_sim_time_ - t_after))
                    ? state_before : state_after;
                current_interpolated_state_(3) = current_sim_time_;
            } else {
                double alpha = (t_before - current_sim_time_) / segment_duration;
                alpha = std::clamp(alpha, 0.0, 1.0);

                const Eigen::Vector2d p0 = state_before.head<2>();
                const Eigen::Vector2d p1 = state_after.head<2>();

                const double th0 = state_before(2);
                const double th1 = state_after(2);
                const double dth = normalizeAngle(th1 - th0);

                current_interpolated_state_ = state_before;
                current_interpolated_state_.head<2>() = p0 + alpha * (p1 - p0);
                current_interpolated_state_(2) = normalizeAngle(th0 + alpha * dth);
                current_interpolated_state_(3) = current_sim_time_;
            }
        }

        // 4. Collision check
        auto gazebo_checker = std::dynamic_pointer_cast<DeterministicObstacleChecker>(obstacle_checker_);
        if (gazebo_checker) {
            double current_planner_time = this->getCurrentSimTime();
            double current_sim_time = initial_budget_time_ - current_planner_time;
            gazebo_checker->processLatestPoseInfo(current_sim_time);

            const Eigen::Vector2d current_pos = current_interpolated_state_.head<2>();
            const double current_yaw = current_interpolated_state_(2);

            bool is_colliding_now = gazebo_checker->checkRobotCollision(current_pos, current_yaw);

            if (is_colliding_now && !is_in_collision_state_) {
                collision_count_++;

                std::vector<std::string> colliding_obstacles;
                const auto& all_obstacles = gazebo_checker->getObstaclePositions();
                for (const auto& obs : all_obstacles) {
                    Eigen::Vector2d obs_pos(obs.position.x(), obs.position.y());
                    double dist = (current_pos - obs_pos).norm();
                    if (dist < obs.dimensions.radius + inflation) {
                        colliding_obstacles.push_back(obs.name);
                    }
                }

                RCLCPP_FATAL(this->get_logger(),
                            "XXX COLLISION DETECTED XXX At T=%.2f | Total Collisions: %d\n"
                            "Robot Pos: [%.3f, %.3f] | Theta: %.3f rad\n"
                            "Colliding Obstacles: %s",
                            current_sim_time_,
                            collision_count_.load(),
                            current_pos.x(), current_pos.y(), current_yaw,
                            colliding_obstacles.empty() ? "NONE" : colliding_obstacles.front().c_str());
            }

            is_in_collision_state_ = is_colliding_now;
        }

        // 5. Update trace on ground
        Eigen::VectorXd start_pt(3);
        start_pt << prev_state(0), prev_state(1), 0.0;

        Eigen::VectorXd end_pt(3);
        end_pt << current_interpolated_state_(0), current_interpolated_state_(1), 0.0;

        robot_trace_edges_.push_back({start_pt, end_pt});


        // ---- Physical solution-quality metrics ----
        const double seg_dt     = prev_state(3) - current_interpolated_state_(3);                       // [s]
        const double seg_dx     = (current_interpolated_state_.head<2>() - prev_state.head<2>()).norm(); // [m]
        const double seg_dtheta = std::abs(normalizeAngle(current_interpolated_state_(2) - prev_state(2))); // [rad]

        executed_path_length_    += seg_dx;
        executed_time_           += seg_dt;
        executed_heading_change_ += seg_dtheta;   // captures curvature/turning that L alone hides

        // std::cout << "[Dubins] L = " << executed_path_length_    << " m"
        //         << " | T = "       << executed_time_           << " s"
        //         << " | turn = "    << executed_heading_change_ << " rad"
        //         << " | v = "       << current_speed_           << " m/s"
        //         << " | w = "       << current_angular_speed_   << " rad/s\n";


    }

    ExecutedMetrics getExecutedMetrics() const override {
        ExecutedMetrics m;
        m.path_length    = executed_path_length_;
        m.arrival_time   = executed_time_;
        m.heading_change = executed_heading_change_;
        return m;   // control_effort left 0.0
    }



void setPath(const std::vector<Eigen::VectorXd>& new_path) {
    std::lock_guard<std::mutex> lock(path_mutex_);

    auto unwrapAngle = [&](double prev, double cur) {
        return prev + normalizeAngle(cur - prev);
    };

    if (new_path.size() < 2) {
        return;
    }

    if (!is_path_set_) {
        current_path_ = new_path;
        current_sim_time_ = current_path_.front()(3);
        current_interpolated_state_ = current_path_.front();
        is_path_set_ = true;
        return;
    }

    const double TIME_EPS = 1e-9;
    const double POS_EPS  = 1e-6;
    const double TH_EPS   = 1e-6;

    // Use executed state time, not already-advanced sim clock
    Eigen::VectorXd robot = current_interpolated_state_;
    const double t_robot = robot(3);  // robot(3) already has correct timestamp

    // Find first point with time < t_robot
    size_t next_idx = new_path.size();
    for (size_t i = 0; i < new_path.size(); ++i) {
        if (new_path[i](3) < t_robot - TIME_EPS) {
            next_idx = i;
            break;
        }
    }

    if (next_idx == new_path.size()) {
        return;
    }

    std::vector<Eigen::VectorXd> stitched;
    stitched.reserve(new_path.size() - next_idx + 1);

    // stitched[0] = robot current state
    stitched.push_back(robot);

    // Add first valid target
    {
        Eigen::VectorXd pt = new_path[next_idx];
        pt(2) = unwrapAngle(stitched.back()(2), pt(2));

        double dt = std::abs(pt(3) - stitched.back()(3));
        double dx = (pt.head<2>() - stitched.back().head<2>()).norm();
        double dth = std::abs(normalizeAngle(pt(2) - stitched.back()(2)));

        if (!(dt < TIME_EPS && dx < POS_EPS && dth < TH_EPS) &&
            pt(3) < stitched.back()(3) - TIME_EPS) {
            stitched.push_back(pt);
        }
    }

    // Append rest, skip duplicates + bad times
    for (size_t i = next_idx + 1; i < new_path.size(); ++i) {
        Eigen::VectorXd pt = new_path[i];

        double dt = std::abs(pt(3) - stitched.back()(3));
        double dx = (pt.head<2>() - stitched.back().head<2>()).norm();
        double dth = std::abs(normalizeAngle(pt(2) - stitched.back()(2)));

        if (dt < TIME_EPS && dx < POS_EPS && dth < TH_EPS) continue;
        if (pt(3) >= stitched.back()(3) - TIME_EPS) continue;

        pt(2) = unwrapAngle(stitched.back()(2), pt(2));
        stitched.push_back(pt);
    }

    if (stitched.size() < 2) {
        return;
    }

    // // Sanity: strictly decreasing times
    // for (size_t i = 1; i < stitched.size(); ++i) {
    //     if (!(stitched[i-1](3) > stitched[i](3) + TIME_EPS)) {
    //         return;
    //     }
    // }

    current_path_ = std::move(stitched);
    is_path_set_ = true;
}




    Eigen::VectorXd getCurrentSimulatedState() {
        std::lock_guard<std::mutex> lock(path_mutex_);
        return current_interpolated_state_;
    }

    int getCollisionCount() const { return collision_count_.load(); }

    void updateThreats(const std::vector<Obstacle>& culprits) {
        current_threat_names_.clear();
        for (const auto& obs : culprits) {
            current_threat_names_.insert(obs.name);
        }
    }

    double getCurrentSimTime() const {
        return current_sim_time_;
    }


    void notifyGoalReached() {
        // Use unique_lock to allow early unlocking
        std::unique_lock<std::mutex> lock(path_mutex_);
        goal_reached_ = true;
        current_speed_ = 0.0;
        current_angular_speed_ = 0.0;
        lock.unlock();   // release the mutex now

        // Safe to call visualizationLoop() – no deadlock
        visualizationLoop();
    }

private:
    std::shared_ptr<ObstacleChecker> obstacle_checker_;
    std::shared_ptr<RVizVisualization> visualizer_;
    rclcpp::TimerBase::SharedPtr vis_timer_;
    std::mutex path_mutex_;
    std::vector<Eigen::VectorXd> current_path_;
    
    // CHANGED: Store edges instead of points
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> robot_trace_edges_;
    
    double current_sim_time_;
    bool is_path_set_;
    Eigen::VectorXd current_interpolated_state_;
    std::atomic<int> collision_count_{0};
    bool is_in_collision_state_{false};
    std::set<std::string> current_threat_names_;
    double initial_budget_time_;
    double inflation;

    double current_speed_ = 0;
    double current_angular_speed_ = 0;
    bool goal_reached_ = false;

    double slice_time;

    // Dubins members
    double executed_path_length_   = 0.0; // [m]
    double executed_time_          = 0.0; // [s]
    double executed_heading_change_= 0.0; // [rad] total absolute turning


    double normalizeAngle(double angle) {
        angle = fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0.0) angle += 2.0 * M_PI;
        return angle - M_PI;
    }

    void visualizationLoop() { 
        if (!obstacle_checker_ || !visualizer_) return; 
        
        // --- 1. PREPARE OBSTACLE DATA ---
        auto gazebo_checker = std::dynamic_pointer_cast<DeterministicObstacleChecker>(obstacle_checker_); 
        std::vector<Eigen::VectorXd> safe_cyl_pos, threat_cyl_pos; 
        std::vector<double> safe_cyl_radii, threat_cyl_radii;
        std::vector<std::tuple<Eigen::Vector2d, double, double, double>> safe_boxes, threat_boxes;
        std::vector<Eigen::Vector2d> safe_vel_pos, safe_vel_val;
        std::vector<Eigen::Vector2d> threat_vel_pos, threat_vel_val;
        
        if (gazebo_checker) {
            double current_planner_time = this->getCurrentSimTime(); 
            double current_sim_time = initial_budget_time_ - current_planner_time;
            gazebo_checker->processLatestPoseInfo(current_sim_time); 
            const ObstacleVector& all_obstacles = gazebo_checker->getObstaclePositions(); 
            
            for (const auto& obstacle : all_obstacles) {
                // SKIPPING STATIC OBSTACLES THAT ARE NOT VISIBLE (EITHER AT FIRST OR NOT IN SENSOR RANGE)
                if(!obstacle.is_dynamic && !obstacle.is_discovered) continue;

                bool is_threat = current_threat_names_.count(obstacle.name);
                if (obstacle.type == Obstacle::CIRCLE) {
                    Eigen::VectorXd pos(2); pos << obstacle.position.x(), obstacle.position.y();
                    if (is_threat) { threat_cyl_pos.push_back(pos); threat_cyl_radii.push_back(obstacle.dimensions.radius); }
                    else { safe_cyl_pos.push_back(pos); safe_cyl_radii.push_back(obstacle.dimensions.radius); }
                } else if (obstacle.type == Obstacle::BOX) {
                    auto box_tuple = std::make_tuple(obstacle.position, obstacle.dimensions.width, obstacle.dimensions.height, obstacle.dimensions.rotation);
                    if (is_threat) threat_boxes.push_back(box_tuple);
                    else safe_boxes.push_back(box_tuple);
                }
                if (obstacle.is_dynamic && obstacle.velocity.norm() > 0.01) {
                    if (is_threat) { threat_vel_pos.push_back(obstacle.position); threat_vel_val.push_back(obstacle.velocity); }
                    else { safe_vel_pos.push_back(obstacle.position); safe_vel_val.push_back(obstacle.velocity); }
                }
            }
        }

        // --- 2. PREPARE ROBOT DATA (Trace & Arrow) ---
        std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> current_trace;
        Eigen::Vector3d robot_pos;
        Eigen::VectorXd orientation_quat(4);
        bool is_colliding = false;
        std::string robot_state_text;
        std::vector<double> bar_vals;
        std::vector<std::string> bar_names = {"v", "w"};

        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            
            // Copy trace for thread safety
            current_trace = robot_trace_edges_;
            
            // Prepare Robot Arrow Data
            robot_pos << current_interpolated_state_(0), current_interpolated_state_(1), 0.0;
            // Use the interpolated theta from the state vector
            Eigen::Quaterniond q(Eigen::AngleAxisd(current_interpolated_state_(2), Eigen::Vector3d::UnitZ()));
            orientation_quat << q.x(), q.y(), q.z(), q.w();
            is_colliding = is_in_collision_state_;

            // Format text using precomputed values
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1) << "v=" << current_speed_ << " m/s";
            oss << " | ω=" << current_angular_speed_ << " rad/s";
            robot_state_text = oss.str();
            bar_vals = {current_speed_, std::abs(current_angular_speed_)};
            if (goal_reached_)
                bar_vals = {0.0, 0.0};
            

        }

        // PUBLISH BATCH
        std::vector<float> robot_color = is_colliding ? std::vector<float>{1.0f, 0.0f, 0.0f} : std::vector<float>{0.0f, 0.45f, 0.74f};
        
        visualizer_->publishObstacleFrame(
            safe_cyl_pos, safe_cyl_radii, threat_cyl_pos, threat_cyl_radii,
            safe_boxes, threat_boxes, safe_vel_pos, safe_vel_val, threat_vel_pos, threat_vel_val,
            current_trace, 
            robot_pos, orientation_quat, robot_color, inflation, 
            robot_state_text,
            bar_vals, bar_names,
            "map"
        );
    }
};