// controller.hpp
#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <chrono>
#include "motion_planning/utils/params.hpp"

struct RobotPose {
    Eigen::Vector2d position;  // 2D position (x, y)
    Eigen::VectorXd orientation;  // Orientation as a quaternion (x, y, z, w)
};

struct ControlOutput {
    double linear_x;  // Linear velocity
    double angular_z; // Angular velocity
};

class Controller {
public:
    Controller(const Params& params) {
        kp_angular_ = params.getParam<double>("kp_angular");
        max_angular_speed_ = params.getParam<double>("max_angular_speed");
        lookahead_distance_ = params.getParam<double>("lookahead_distance");
        control_loop_dt_ = params.getParam<double>("control_loop_dt");
    }

    int64_t getLoopDtMs(){
        std::chrono::duration<double> duration_in_seconds(control_loop_dt_);
        auto duration_in_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration_in_seconds);
        return duration_in_milliseconds.count();
    }

    void followPath(const std::vector<Eigen::VectorXd>& path) {
        current_path_ = path;
        progress_index_ = 0;
    }

    ControlOutput controlLoop(const RobotPose& robot_pose) {
        ControlOutput output{0.0, 0.0};

        if (current_path_.empty() || current_path_.size() == 1) {
            return output; // Stop the robot
        }

        size_t closest_index = findClosestPoint(robot_pose.position);
        size_t lookahead_index = findLookaheadPoint(robot_pose.position, closest_index);
        Eigen::Vector2d lookahead_point = current_path_[lookahead_index].head<2>();

        double curvature = calculatePathCurvature(lookahead_index);
        double steering_angle = calculateSteeringAngle(robot_pose, lookahead_point);

        double max_speed = 4.0 * (1.0 - std::tanh(2.0 * curvature));
        double min_speed = 0.5;
        double steering_penalty = 1.0 - (std::abs(steering_angle) / (0.8 * M_PI));
        steering_penalty = std::clamp(steering_penalty, 0.2, 1.0);

        double target_velocity = std::clamp(max_speed * steering_penalty, min_speed, 4.0);
        double dt = control_loop_dt_;
        double accel = (target_velocity > current_linear_velocity_) ? 1.0 : 4.0;
        current_linear_velocity_ += accel * (target_velocity - current_linear_velocity_) * dt;

        double angular_boost = 1.0 + 2.0 * curvature;
        double angular_z = kp_angular_ * steering_angle * angular_boost;
        angular_z = std::clamp(angular_z, -max_angular_speed_, max_angular_speed_);

        output.linear_x = current_linear_velocity_;
        output.angular_z = angular_z;
        lookahead_distance_ = current_linear_velocity_ * 3.0;
        lookahead_distance_ = std::clamp(lookahead_distance_, 1.5, 5.0);

        return output;
    }

private:
    std::vector<Eigen::VectorXd> current_path_;
    double kp_angular_;
    double max_angular_speed_;
    double lookahead_distance_;
    double current_linear_velocity_ = 0.0;
    double prev_steering_angle_ = 0.0;
    double smoothing_factor_ = 0.2;
    size_t progress_index_;
    double control_loop_dt_; // Duration in seconds

    size_t findClosestPoint(const Eigen::Vector2d& robot_position) {
        size_t closest_index = 0;
        double min_distance = std::numeric_limits<double>::max();
        for (size_t i = 0; i < current_path_.size(); ++i) {
            double dx = current_path_[i][0] - robot_position[0];
            double dy = current_path_[i][1] - robot_position[1];
            double distance = std::sqrt(dx * dx + dy * dy);
            if (distance < min_distance) {
                min_distance = distance;
                closest_index = i;
            }
        }
        return closest_index;
    }

    size_t findLookaheadPoint(const Eigen::Vector2d& robot_position, size_t closest_index) {
        for (int i = static_cast<int>(closest_index); i >= 0; --i) {
            size_t idx = static_cast<size_t>(i);
            double dx = current_path_[idx][0] - robot_position[0];
            double dy = current_path_[idx][1] - robot_position[1];
            double distance = std::sqrt(dx * dx + dy * dy);
            if (distance >= lookahead_distance_) {
                return idx;
            }
        }
        return 0;
    }

    double calculatePathCurvature(size_t lookahead_index) {
        const int num_points = 3;
        if (lookahead_index < num_points) return 0.0;
        std::vector<Eigen::Vector2d> points;
        for (int i = 0; i < num_points; ++i) {
            points.push_back(current_path_[lookahead_index - i].head<2>());
        }
        double dx1 = points[1].x() - points[0].x();
        double dy1 = points[1].y() - points[0].y();
        double dx2 = points[2].x() - points[1].x();
        double dy2 = points[2].y() - points[1].y();
        double cross = dx1 * dy2 - dy1 * dx2;
        double norm1 = std::hypot(dx1, dy1);
        double norm2 = std::hypot(dx2, dy2);
        return std::abs(cross) / (norm1 * norm2 * norm2);
    }

    double calculateSteeringAngle(const RobotPose& robot_pose, const Eigen::Vector2d& lookahead_point) {
        double dx = lookahead_point[0] - robot_pose.position[0];
        double dy = lookahead_point[1] - robot_pose.position[1];
        double target_angle = std::atan2(dy, dx);
        tf2::Quaternion q(
            robot_pose.orientation[0],
            robot_pose.orientation[1],
            robot_pose.orientation[2],
            robot_pose.orientation[3]
        );
        double robot_yaw = tf2::getYaw(q);
        double angle_error = target_angle - robot_yaw;
        angle_error = std::fmod(angle_error + M_PI, 2 * M_PI) - M_PI;
        angle_error = std::remainder(angle_error, 2 * M_PI);
        if (std::abs(angle_error) == M_PI) {
            angle_error = -M_PI;
        }
        return angle_error;

        // Apply smoothing
        // double smoothed_angle = smoothing_factor * angle_error + (1 - smoothing_factor) * prev_steering_angle;
        // prev_steering_angle = smoothed_angle;
        // return smoothed_angle;


    }
};