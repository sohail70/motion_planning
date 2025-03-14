#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "motion_planning/utils/params.hpp"

struct RobotPose {
    Eigen::Vector2d position;
    Eigen::VectorXd orientation;
};

struct ControlOutput {
    double linear_x;
    double angular_z;
    Eigen::Vector2d lookahead_point;
};

class Controller {
public:
    Controller(const Params& params) {
        kp_ = params.getParam<double>("kp_angular");
        kd_ = params.getParam<double>("kd_angular");
        max_angular_speed_ = params.getParam<double>("max_angular_speed");
        lookahead_distance_ = params.getParam<double>("lookahead_distance");
        control_loop_dt_ = params.getParam<double>("control_loop_dt");
        cross_track_gain_ = params.getParam<double>("cross_track_gain");
        dt_ = std::chrono::duration<double>(control_loop_dt_).count();
        max_speed_ = 4.0; // Increased base speed
        smoothing_factor_ = 0.2;
    }

    void followPath(const std::vector<Eigen::VectorXd>& new_path) {
        if (!isSubpath(current_path_, new_path)) {
            // Completely new path - reset progress
            current_path_ = new_path;
            progress_index_ = 0;
        } else {
            // Continue existing path - maintain progress index
            current_path_ = new_path;
        }
    }

    int64_t getLoopDtMs() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(control_loop_dt_)).count();
    }

    ControlOutput controlLoop(const RobotPose& robot_pose) {
        ControlOutput output{0.0, 0.0, Eigen::Vector2d{0, 0}};
        if (current_path_.empty()) return output;

        // 1. Path Progression & Lookahead Calculation
        progress_index_ = findClosestSegment(robot_pose.position);
        Eigen::Vector2d lookahead = getLookaheadPoint(robot_pose.position);
        output.lookahead_point = lookahead;

        // Force progression when near the current target point
        if ((current_path_[progress_index_].head<2>() - robot_pose.position).norm() < 0.3) {
            progress_index_ = std::min(progress_index_ + 1, current_path_.size() - 1);
        }

        // 2. Compute Heading & Steering Error
        double yaw = getYaw(robot_pose.orientation);
        double target_angle = atan2(lookahead.y() - robot_pose.position.y(),
                                    lookahead.x() - robot_pose.position.x());
        double angle_error = angleDiff(target_angle, yaw);

        // 3. Compute Cross-Track Error
        Eigen::Vector2d path_tangent = (current_path_[progress_index_ + 1].head<2>() -
                                        current_path_[progress_index_].head<2>()).normalized();
        Eigen::Vector2d robot_to_path = current_path_[progress_index_].head<2>() - robot_pose.position;
        double cross_track_error = robot_to_path.dot(Eigen::Vector2d(-path_tangent.y(), path_tangent.x()));

        // 4. Compute Steering Command with Smoothness
        double desired_curvature = (2.0 * cross_track_error) / (lookahead_distance_ * lookahead_distance_);
        double angular = (kp_ * angle_error) + 
                         (cross_track_gain_ * desired_curvature) + 
                         (kd_ * (angle_error - prev_error_) / dt_);
        
        prev_error_ = angle_error;

        // Apply Low-Pass Filter to Reduce Steering Oscillations
        angular = smoothing_factor_ * angular + (1 - smoothing_factor_) * prev_angular_;
        prev_angular_ = angular;

        // 5. Adaptive Speed Control
        double curvature = calculateCurvature();
        double speed = calculateSpeed(angle_error, curvature);

        // Ensure minimum forward speed
        output.linear_x = std::max(speed, 0.5); // Minimum speed of 0.5 m/s
        output.angular_z = std::clamp(angular, -max_angular_speed_, max_angular_speed_);
        return output;
    }

private:
    std::vector<Eigen::VectorXd> current_path_;
    double kp_, kd_, max_speed_, dt_;
    double max_angular_ = 2.5;
    double prev_error_ = 0.0;
    double current_speed_ = 0.0;
    double prev_angular_ = 0.0;
    double prev_steering_angle_ = 0.0;
    double max_angular_speed_;
    double lookahead_distance_;
    double cross_track_gain_;
    double current_linear_velocity_ = 0.0;
    double integral_error_ = 0.0;
    double smoothing_factor_ = 0.2;
    size_t progress_index_;
    double control_loop_dt_;

    // Add these constants
    const double BASE_LOOKAHEAD = 1.2;  // Increased from 0.8
    const double SPEED_LOOKAHEAD_FACTOR = 0.6;  // Increased from 0.4
    const double CURVATURE_LOOKAHEAD = 2.5;  // Increased from 2.0
    const double CURVATURE_GAIN = 1.8;  // How much curvature affects speed

    bool isSubpath(const std::vector<Eigen::VectorXd>& old_path, 
                   const std::vector<Eigen::VectorXd>& new_path) {
        if (old_path.empty() || new_path.empty()) return false;
        
        // Check if the first point of the new path is near any point in the old path
        for (const auto& old_pt : old_path) {
            if ((old_pt.head<2>() - new_path[0].head<2>()).norm() < 0.2) {
                return true;
            }
        }
        return false;
    }

    size_t findClosestSegment(const Eigen::Vector2d& pos) {
        size_t closest = progress_index_;
        double min_dist = INFINITY;

        // Search forward with hysteresis to prevent oscillation
        size_t start = std::max(0, (int)progress_index_ - 2);
        size_t end = std::min(current_path_.size(), progress_index_ + 10);
        
        for (size_t i = start; i < end; ++i) {
            double d = (current_path_[i].head<2>() - pos).squaredNorm();
            if (d < min_dist) {
                min_dist = d;
                closest = i;
            }
        }
        return closest;
    }

    Eigen::Vector2d getLookaheadPoint(const Eigen::Vector2d& pos) {
        double dynamic_lookahead = BASE_LOOKAHEAD + (current_speed_ * SPEED_LOOKAHEAD_FACTOR);
        size_t start_index = progress_index_;
        double accumulated = 0.0;

        // Always look at least 3 points ahead
        size_t min_lookahead_index = std::min(progress_index_ + 3, current_path_.size() - 1);
        Eigen::Vector2d min_lookahead = current_path_[min_lookahead_index].head<2>();

        for (size_t i = start_index; i < current_path_.size() - 1; ++i) {
            Eigen::Vector2d seg_start = current_path_[i].head<2>();
            Eigen::Vector2d seg_end = current_path_[i + 1].head<2>();
            double seg_length = (seg_end - seg_start).norm();

            if (accumulated + seg_length >= dynamic_lookahead) {
                double t = (dynamic_lookahead - accumulated) / seg_length;
                Eigen::Vector2d point = seg_start + t * (seg_end - seg_start);
                // Return whichever is further ahead
                return (point - pos).norm() > (min_lookahead - pos).norm() ? point : min_lookahead;
            }
            accumulated += seg_length;
        }
        return current_path_.back().head<2>();
    }

    double calculateCurvature() {
        // Look further ahead for upcoming curves
        size_t lookahead_index = std::min(
            progress_index_ + static_cast<size_t>(CURVATURE_LOOKAHEAD / BASE_LOOKAHEAD),
            current_path_.size() - 1
        );
        
        if (lookahead_index <= progress_index_ + 2) return 0.0;
        
        Eigen::Vector2d p1 = current_path_[progress_index_].head<2>();
        Eigen::Vector2d p2 = current_path_[(progress_index_ + lookahead_index) / 2].head<2>();
        Eigen::Vector2d p3 = current_path_[lookahead_index].head<2>();
        
        // Calculate curvature using triangle area method
        double area = 0.5 * std::abs(
            (p2.x() - p1.x()) * (p3.y() - p1.y()) - 
            (p3.x() - p1.x()) * (p2.y() - p1.y())
        );
        
        double denominator = (p2 - p1).norm() * (p3 - p2).norm() * (p3 - p1).norm();
        return denominator > 1e-6 ? (4 * area) / denominator : 0.0;
    }

    double calculateSpeed(double angle_error, double curvature) {
        const double MAX_ANGLE = M_PI / 6;  // 30 degrees
        const double MIN_SPEED = 0.5;       // Minimum speed
        
        // Angle-based penalty (less aggressive)
        double angle_penalty = 1.0 - std::abs(angle_error) / MAX_ANGLE;
        angle_penalty = std::clamp(angle_penalty, 0.0, 1.0);
        
        // Curvature-based penalty (gentler)
        double curve_penalty = 1.0 - std::tanh(CURVATURE_GAIN * std::abs(curvature));
        
        // Combine penalties
        double speed_scale = std::min(angle_penalty, curve_penalty);
        double target_speed = max_speed_ * speed_scale;
        
        // Apply minimum speed
        target_speed = std::max(target_speed, MIN_SPEED);
        
        // Smooth acceleration/deceleration
        double accel = (target_speed > current_speed_) ? 1.0 : 4.0;
        current_speed_ += accel * (target_speed - current_speed_) * dt_;
        
        return std::clamp(current_speed_, MIN_SPEED, max_speed_);
    }

    static double angleDiff(double a, double b) {
        double diff = a - b;
        return std::atan2(sin(diff), cos(diff)); // Wraps to [-π, π]
    }

    static double getYaw(const Eigen::VectorXd& quat) {
        return std::atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
                      1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    }
};