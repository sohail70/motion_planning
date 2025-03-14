
#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <limits>
#include <algorithm>
#include "motion_planning/utils/obstacle_checker.hpp"
#include "motion_planning/utils/params.hpp"
#include <rclcpp/rclcpp.hpp>

struct DWAVisualization {
    std::vector<std::vector<Eigen::Vector3d>> candidate_trajectories;
    std::vector<Eigen::Vector3d> best_trajectory;
    std::vector<std::vector<Eigen::Vector2d>> footprints;
    Eigen::Vector2d best_velocity;
};

class DWAPlanner {
public:
    struct State {
        Eigen::Vector3d pose;      // x, y, theta
        Eigen::Vector2d velocity;  // v, w
    };

    DWAPlanner(const Params& params,
               std::shared_ptr<ObstacleChecker> obstacle_checker)
        : obstacle_checker_(obstacle_checker) {
        loadParams(params);
    }

    Eigen::Vector2d computeVelocityCommand(
        const State& current_state,
        const Eigen::Vector3d& goal,
        const std::vector<Eigen::Vector3d>& global_path,
        DWAVisualization& vis_data) 
    {
        // 1) Early exit if goal is reached
        if (checkReachedGoal(current_state.pose, goal)) {
            std::cout << "[DEBUG] Goal reached. Returning zero velocity.\n";
            return Eigen::Vector2d::Zero();
        }

        // 2) Compute dynamic window
        auto window = calcDynamicWindow(current_state, goal);
        std::cout << "[DEBUG] Dynamic Window - v: ["
                  << window.min_v << ", " << window.max_v 
                  << "], w: [" << window.min_w << ", "
                  << window.max_w << "]\n";

        // 3) Generate candidate trajectories
        auto trajectories = generateTrajectories(current_state, window);
        std::cout << "[DEBUG] Generated " << trajectories.size()
                  << " candidate trajectories.\n";

        // 4) Evaluate each trajectory and pick the best
        Trajectory best_traj;
        bool has_valid_traj = false;
        double min_cost = std::numeric_limits<double>::max();

        for (auto& traj : trajectories) {
            // Evaluate and weight cost
            auto cost = evaluateTrajectory(traj, goal, global_path);

            // // DEBUG PRINT: shows *weighted* cost
            // std::cout << "[DEBUG] Trajectory v=" << traj.velocity[0]
            //           << ", w=" << traj.velocity[1] << " => "
            //           << "Obstacle: " << cost.obstacle
            //           << ", Goal: "     << cost.goal
            //           << ", Speed: "    << cost.speed
            //           << ", Path: "     << cost.path
            //           << ", Total: "    << cost.total << "\n";

            // If it’s collision-free (obstacle < 1e6) and has lower total cost, choose it
            if (cost.obstacle < 1e6 && cost.total < min_cost) {
                min_cost = cost.total;
                best_traj = traj;
                has_valid_traj = true;
            }
        }

        // 5) If no valid trajectory found => do recovery
        if (!has_valid_traj) {
            std::cout << "[DEBUG] No valid trajectory found. Recovery mode.\n";
            return handleRecovery(current_state.pose, goal);
        }

        // 6) Visualize
        updateVisualizationData(best_traj, trajectories, vis_data);

        // 7) Output chosen velocity
        std::cout << "[DEBUG] Selected trajectory: v=" << best_traj.velocity[0]
                  << ", w=" << best_traj.velocity[1]
                  << " with cost=" << min_cost << "\n";

        return best_traj.velocity;
    }

private:
    // -------------------- Internal structs --------------------
    struct Trajectory {
        std::vector<Eigen::Vector3d> path;  // sequence of predicted poses
        Eigen::Vector2d velocity;           // (v, w)
    };

    struct Cost {
        double obstacle;  // raw obstacle cost
        double goal;      // raw goal cost
        double speed;     // raw speed cost
        double path;      // raw path cost
        double total;     // final, weighted total cost
    };

    struct Window {
        double min_v;
        double max_v;
        double min_w;
        double max_w;
    };

    // -------------------- Parameters --------------------
    double max_speed_;
    double min_speed_;
    double max_yawrate_;
    double max_accel_;
    double max_decel_;
    double max_dyawrate_;
    double speed_resolution_;
    double yawrate_resolution_;
    double dt_;
    double predict_time_;    // (If you want to unify with sim_steps_)
    int    sim_steps_;
    double goal_cost_gain_;
    double speed_cost_gain_;
    double obstacle_cost_gain_;
    double path_cost_gain_;
    double robot_radius_;
    double goal_tolerance_           = 0.1;
    double recovery_rotation_rate_   = 0.5;
    double stuck_timeout_            = 5.0;
    double stuck_speed_threshold_    = 0.05;

    std::shared_ptr<ObstacleChecker> obstacle_checker_;
    std::vector<Eigen::Vector2d> footprint_;
    rclcpp::Time last_valid_cmd_time_;
    Eigen::Vector2d last_cmd_vel_ = Eigen::Vector2d::Zero();

    // -------------------- Param loader --------------------
    void loadParams(const Params& params) {
        max_speed_          = params.getParam<double>("max_speed");
        min_speed_          = params.getParam<double>("min_speed");
        max_yawrate_        = params.getParam<double>("max_yawrate");
        max_accel_          = params.getParam<double>("max_accel");
        max_decel_          = params.getParam<double>("max_decel");
        max_dyawrate_       = params.getParam<double>("max_dyawrate");
        speed_resolution_   = params.getParam<double>("speed_resolution");
        yawrate_resolution_ = params.getParam<double>("yawrate_resolution");
        dt_                 = params.getParam<double>("dt");
        predict_time_       = params.getParam<double>("predict_time");
        sim_steps_          = params.getParam<int>("sim_steps");
        goal_cost_gain_     = params.getParam<double>("goal_cost_gain");
        speed_cost_gain_    = params.getParam<double>("speed_cost_gain");
        obstacle_cost_gain_ = params.getParam<double>("obstacle_cost_gain");
        path_cost_gain_     = params.getParam<double>("path_cost_gain");
        robot_radius_       = params.getParam<double>("robot_radius");
        
        // (Optional) unify sim_steps_ with predict_time_
        // sim_steps_ = static_cast<int>(std::round(predict_time_ / dt_));

        // Load footprint if available
        if (params.hasParam("footprint")) {
            std::vector<double> raw_footprint =
                params.getParam<std::vector<double>>("footprint");
            if (raw_footprint.size() % 2 != 0) {
                throw std::runtime_error(
                    "Footprint must have an even number of x,y pairs!");
            }
            footprint_.resize(raw_footprint.size() / 2);
            for (size_t i = 0; i < raw_footprint.size(); i += 2) {
                footprint_[i / 2] =
                    Eigen::Vector2d(raw_footprint[i], raw_footprint[i + 1]);
            }
        } else {
            // Default approximate circular footprint
            const int num_points = 20;
            footprint_.resize(num_points);
            for (int i = 0; i < num_points; ++i) {
                double angle = 2 * M_PI * i / num_points;
                footprint_[i] = Eigen::Vector2d(
                    robot_radius_ * std::cos(angle),
                    robot_radius_ * std::sin(angle));
            }
        }
    }

    // -------------------- computeVelocityCommand helpers --------------------
    Window calcDynamicWindow(const State& state,
                             const Eigen::Vector3d& current_goal) const {
        Window window;
        // Basic linear velocity limits
        window.min_v = std::max(state.velocity[0] - max_decel_ * dt_, min_speed_);
        window.max_v = std::min(state.velocity[0] + max_accel_ * dt_, max_speed_);

        // Basic angular velocity limits
        window.min_w = std::max(state.velocity[1] - max_dyawrate_ * dt_, -max_yawrate_);
        window.max_w = std::min(state.velocity[1] + max_dyawrate_ * dt_,  max_yawrate_);

        // Optional: add an angular “bias” to help turn toward goal quickly
        double yaw_to_goal = std::atan2(current_goal.y() - state.pose.y(),
                                        current_goal.x() - state.pose.x());
        double yaw_error = yaw_to_goal - state.pose[2];
        yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

        // const double ANGULAR_BIAS_GAIN = 2.0;
        // if (yaw_error > 0.5) {
        //     window.min_w = std::max(window.min_w, ANGULAR_BIAS_GAIN * yaw_error);
        // } else if (yaw_error < -0.5) {
        //     window.max_w = std::min(window.max_w, ANGULAR_BIAS_GAIN * yaw_error);
        // }
        return window;
    }

    std::vector<Trajectory> generateTrajectories(const State& state,
                                                 const Window& window) {
        std::vector<Trajectory> trajectories;

        // Number of samples to try
        const int velocity_samples = 20;
        const int yaw_samples      = 20;

        // Create evenly spaced velocities
        auto v_samples = linspace(window.min_v, window.max_v, velocity_samples);
        auto w_samples = linspace(window.min_w, window.max_w, yaw_samples);

        // For each (v, w) pair, simulate forward in time
        for (double v : v_samples) {
            for (double w : w_samples) {
                Trajectory traj;
                traj.velocity = Eigen::Vector2d(v, w);
                traj.path     = generateTrajectory(state.pose, v, w);
                trajectories.push_back(traj);
            }
        }
        return trajectories;
    }

    std::vector<double> linspace(double start, double end, int num) {
        // Return vector of linearly spaced samples from start to end (inclusive)
        std::vector<double> res;
        if (num <= 1) {
            res.push_back(start);
            return res;
        }
        double step = (end - start) / (num - 1);
        for (int i = 0; i < num; ++i) {
            res.push_back(start + i * step);
        }
        return res;
    }

    std::vector<Eigen::Vector3d> generateTrajectory(
        const Eigen::Vector3d& start,
        double v, double w) const
    {
        std::vector<Eigen::Vector3d> path;
        path.reserve(sim_steps_);

        Eigen::Vector3d state = start;
        for (int i = 0; i < sim_steps_; ++i) {
            // Simple differential drive update
            state[0] += v * std::cos(state[2]) * dt_;
            state[1] += v * std::sin(state[2]) * dt_;
            state[2] += w * dt_;
            path.push_back(state);
        }
        return path;
    }

    // -------------------- Cost evaluation (with weighting) --------------------
    Cost evaluateTrajectory(const Trajectory& traj,
                            const Eigen::Vector3d& goal,
                            const std::vector<Eigen::Vector3d>& global_path) 
    {
        Cost c;
        c.obstacle = calcObstacleCost(traj);
        c.goal     = calcGoalCost(traj, goal);
        c.speed    = calcSpeedCost(traj);
        c.path     = calcPathCost(traj, global_path);

        // Apply your parameter gains right away:
        c.total = obstacle_cost_gain_ * c.obstacle
                + goal_cost_gain_     * c.goal
                + speed_cost_gain_    * c.speed
                + path_cost_gain_     * c.path;

        return c;
    }

    double calcObstacleCost(const Trajectory& traj) {
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& pose : traj.path) {
            // Check footprint collision
            if (obstacle_checker_->checkFootprintCollision(
                    pose.head<2>(), pose[2], footprint_))
            {
                return 1e6;  // collision => huge cost
            }
            // Track the closest obstacle
            double dist = obstacle_checker_->distanceToNearestObstacle(pose.head<2>());
            if (dist < min_dist) {
                min_dist = dist;
            }
        }

        // If obstacle is very close, we penalize heavily
        // e.g. 1/distance
        if (min_dist < 1.0) {
            return 5.0 / (min_dist + 1e-3);  // stronger penalty if <1.0m
        }
        return 1.0 / (min_dist + 1e-3);
    }

    double calcGoalCost(const Trajectory& traj, const Eigen::Vector3d& goal) {
        // Distance to final pose
        Eigen::Vector2d final_pos = traj.path.back().head<2>();
        double dist_cost = (final_pos - goal.head<2>()).norm();

        // Orientation error at the end
        double final_yaw  = traj.path.back()[2];
        double yaw_to_goal = std::atan2(goal.y() - final_pos.y(),
                                        goal.x() - final_pos.x());
        double yaw_error = std::abs(
            std::atan2(std::sin(yaw_to_goal - final_yaw),
                       std::cos(yaw_to_goal - final_yaw))
        );

        // Weighted sum: distance + orientation
        return dist_cost + 2.0 * yaw_error;  // originally 2.0
    }

    double calcSpeedCost(const Trajectory& traj) {
        // Reward for high linear & high angular speeds
        double linear_reward  = traj.velocity[0];
        double angular_reward = std::abs(traj.velocity[1]);

        // Example cost formula: we want v ~ max_speed, w ~ max_yawrate
        // so cost = (max_speed - v) + (max_yawrate - w)*0.2
        double cost = (max_speed_ - linear_reward)
                    + (max_yawrate_ - angular_reward) * 0.2;
        return cost;
    }


    
    double calcPathCost(const Trajectory& traj,
                        const std::vector<Eigen::Vector3d>& global_path) 
    {
        if (global_path.empty()) {
            return 0.0;  // no path => no cost
        }
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& wp : global_path) {
            for (const auto& tp : traj.path) {
                double dist = (wp.head<2>() - tp.head<2>()).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                }
            }
        }
        // If the path is far away, cost is high
        return min_dist;
    }

    // -------------------- Recovery & goal checks --------------------
    bool checkReachedGoal(const Eigen::Vector3d& pose,
                          const Eigen::Vector3d& goal) const {
        return (pose.head<2>() - goal.head<2>()).norm() < goal_tolerance_;
    }

    Eigen::Vector2d handleRecovery(const Eigen::Vector3d& current_pose,
                                   const Eigen::Vector3d& goal)
    {
        double desired_yaw =
            std::atan2(goal.y() - current_pose.y(),
                       goal.x() - current_pose.x());
        double yaw_error = shortest_angular_distance(current_pose[2],
                                                     desired_yaw);

        // Rotate in place to face the goal
        Eigen::Vector2d cmd(0.0, 0.0);
        cmd[1] = std::clamp(yaw_error * recovery_rotation_rate_,
                            -max_yawrate_, max_yawrate_);
        return cmd;
    }

    // -------------------- Visualization utility --------------------
    void updateVisualizationData(const Trajectory& best_traj,
                                 const std::vector<Trajectory>& all_trajs,
                                 DWAVisualization& vis_data)
    {
        vis_data.best_trajectory = best_traj.path;
        vis_data.best_velocity   = best_traj.velocity;

        vis_data.candidate_trajectories.clear();
        for (const auto& t : all_trajs) {
            vis_data.candidate_trajectories.push_back(t.path);
        }
    }

    // -------------------- Misc helpers --------------------
    static double shortest_angular_distance(double from, double to) {
        double diff = to - from;
        return std::atan2(std::sin(diff), std::cos(diff));
    }

    static double normalize_angle(double angle) {
        angle = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0) {
            angle += 2.0 * M_PI;
        }
        return angle - M_PI;
    }
};
