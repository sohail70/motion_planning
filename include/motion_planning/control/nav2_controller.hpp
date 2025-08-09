// Copyright 2025 Soheil E.nia

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Dense>
#include <vector>
#include "motion_planning/utils/params.hpp"

class Nav2Controller : public rclcpp::Node {
public:
    using FollowPath = nav2_msgs::action::FollowPath;
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

    // Nav2Controller()
    //     : Node("nav2_controller") {
    //     // Initialize the FollowPath action client
    //     follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");
    // }

    Nav2Controller(const Params& params): Node("nav2_controller") {
        // Example: Load parameters for Nav2Controller
        follow_path_topic_ = params.getParam<std::string>("follow_path_topic");
        max_speed_ = params.getParam<double>("max_speed");
    }

    void sendPathToNav2(const std::vector<Eigen::VectorXd>& new_path) {
        static std::vector<Eigen::VectorXd> last_sent_path;

        // Check if the new path is significantly different
        if (!isPathSignificantlyDifferent(new_path, last_sent_path)) {
            return; // No need to update Nav2
        }

        // Cancel the current goal if it exists
        if (goal_handle_) {
            follow_path_client_->async_cancel_goal(goal_handle_);
            RCLCPP_INFO(get_logger(), "Canceling current Nav2 goal due to path changes");
        }

        // Send the new path
        auto path = createPathFromNodes(new_path);
        auto goal_msg = FollowPath::Goal();
        goal_msg.path = path;
        goal_msg.controller_id = "";  // Use default controller

        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](std::shared_ptr<GoalHandleFollowPath> goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(get_logger(), "Goal rejected");
                } else {
                    goal_handle_ = goal_handle;
                    RCLCPP_INFO(get_logger(), "Nav2 is following the updated path");
                }
            };

        send_goal_options.result_callback =
            [this](const GoalHandleFollowPath::WrappedResult& result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(get_logger(), "Path execution complete!");
                } else {
                    RCLCPP_WARN(get_logger(), "Path execution failed or was canceled");
                }
            };

        follow_path_client_->async_send_goal(goal_msg, send_goal_options);
        last_sent_path = new_path; // Update the last sent path
    }

private:
    rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
    std::shared_ptr<GoalHandleFollowPath> goal_handle_;
    std::string follow_path_topic_;
    double max_speed_;

    bool isPathSignificantlyDifferent(
        const std::vector<Eigen::VectorXd>& new_path,
        const std::vector<Eigen::VectorXd>& old_path,
        double distance_threshold = 0.5) {
        if (new_path.size() != old_path.size()) return true;

        for (size_t i = 0; i < new_path.size(); ++i) {
            double dx = new_path[i][0] - old_path[i][0];
            double dy = new_path[i][1] - old_path[i][1];
            if (std::sqrt(dx * dx + dy * dy) > distance_threshold) {
                return true;
            }
        }
        return false;
    }

    nav_msgs::msg::Path createPathFromNodes(const std::vector<Eigen::VectorXd>& nodes) {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = now();

        for (const auto& node : nodes) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = node[0];
            pose.pose.position.y = node[1];
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        return path;
    }
};