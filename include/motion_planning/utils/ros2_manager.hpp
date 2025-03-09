// ros2_manager.hpp
#pragma once

#include "motion_planning/utils/obstacle_checker.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/occupancygrid_obstacle_checker.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/utils.h>
#include "motion_planning/control/controller.hpp"
#include "motion_planning/control/nav2_controller.hpp"

class ROS2Manager : public rclcpp::Node {
public:
    ROS2Manager(
        std::shared_ptr<ObstacleChecker> obstacle_checker,
        std::shared_ptr<RVizVisualization> visualizer,
        std::shared_ptr<Controller> controller,
        std::shared_ptr<Nav2Controller> nav2_controller,
        const Params& params)
        : Node("ros2_manager"),
          obstacle_checker_(obstacle_checker),
          visualizer_(visualizer),
          controller_(controller),
          nav2_controller_(nav2_controller),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tf_listener_(*tf_buffer_) {

        // Load ROS2Manager-specific parameters
        follow_path_ = params.getParam<bool>("follow_path");
        std::string controller_type_ = params.getParam<std::string>("controller");
        if (controller_type_ == "pure_pursuit") {
            use_pure_pursuit_ = true;
        } else if (controller_type_ == "nav2") {
            use_nav_ = true;
        }

        // Initialize ROS2 components
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        if (std::dynamic_pointer_cast<OccupancyGridObstacleChecker>(obstacle_checker_)) {
            map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", 10, std::bind(&ROS2Manager::mapCallback, this, std::placeholders::_1));
        }

        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ROS2Manager::updateCallback, this));

        goal_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&ROS2Manager::goalPoseCallback, this, std::placeholders::_1));

        if (follow_path_) {
            if (use_nav_) {
                // Use Nav2Controller for path following
                control_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(50),
                    std::bind(&ROS2Manager::nav2ControlLoop, this));
            } else if (use_pure_pursuit_) {
                // Use pure pursuit controller
                control_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(controller_->getLoopDtMs()),
                    std::bind(&ROS2Manager::controlLoop, this));
            }
        }
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(goal_pose_mutex_);
        latest_goal_pose_ = *msg;
        goal_received_ = true;
        new_goal_received_ = true; // Signal that a new goal has been received
    }

    bool hasNewGoal() {
        std::lock_guard<std::mutex> lock(goal_pose_mutex_);
        return new_goal_received_;
    }

    Eigen::VectorXd getStartPosition() {
        std::lock_guard<std::mutex> lock(goal_pose_mutex_);
        new_goal_received_ = false; // Reset the flag after retrieving the goal
        Eigen::VectorXd start_position(2);
        start_position << latest_goal_pose_.pose.position.x, latest_goal_pose_.pose.position.y;
        return start_position;
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        dynamic_cast<OccupancyGridObstacleChecker*>(obstacle_checker_.get())->updateGrid(msg);
        updateRobotPose();
    }

    void updateCallback() {
        updateRobotPose();
    }

    void controlLoop() {
        RobotPose robot_pose = getRobotPose();
        ControlOutput output = controller_->controlLoop(robot_pose);

        // Publish cmd_vel
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = output.linear_x;
        twist_msg->angular.z = output.angular_z;
        cmd_vel_publisher_->publish(std::move(twist_msg));
    }

    void nav2ControlLoop() {
        if (current_path_.empty()) return;
        nav2_controller_->sendPathToNav2(current_path_);
    }

    void followPath(const std::vector<Eigen::VectorXd>& path) {
        current_path_ = path;
        if (use_nav_) {
            nav2_controller_->sendPathToNav2(path);
        } else if (use_pure_pursuit_) {
            controller_->followPath(path);
        }
    }

private:
    std::shared_ptr<ObstacleChecker> obstacle_checker_;
    std::shared_ptr<RVizVisualization> visualizer_;
    std::shared_ptr<Controller> controller_;
    std::shared_ptr<Nav2Controller> nav2_controller_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;
    std::mutex goal_pose_mutex_;
    std::condition_variable goal_cv_;
    geometry_msgs::msg::PoseStamped latest_goal_pose_;
    bool goal_received_ = false;
    bool follow_path_;
    bool use_nav_ = false;
    bool use_pure_pursuit_ = false;
    bool new_goal_received_ = false;
    std::vector<Eigen::VectorXd> current_path_;

    RobotPose getRobotPose() {
        RobotPose pose;

        // Get the robot's pose from Gazebo or TF
        if (auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker_)) {
            pose.position = gazebo_checker->getRobotPosition();
            pose.orientation = gazebo_checker->getRobotOrientation();
        } else {
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
                pose.position = Eigen::Vector2d(transform.transform.translation.x, transform.transform.translation.y);
                pose.orientation = Eigen::VectorXd(4);
                pose.orientation << transform.transform.rotation.x,
                                    transform.transform.rotation.y,
                                    transform.transform.rotation.z,
                                    transform.transform.rotation.w;
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
                pose.position = Eigen::Vector2d(0.0, 0.0);
                pose.orientation = Eigen::VectorXd::Zero(4);
            }
        }

        return pose;
    }

    void updateRobotPose() {
        double robot_x = 0.0;
        double robot_y = 0.0;

        if (auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker_)) {
            Eigen::Vector2d robot_pos = gazebo_checker->getRobotPosition();
            robot_x = robot_pos.x();
            robot_y = robot_pos.y();
        } else {
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
                robot_x = transform.transform.translation.x;
                robot_y = transform.transform.translation.y;
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
                return;
            }
        }

        visualizeObstacles(robot_x, robot_y);
    }

    void visualizeObstacles(double robot_x, double robot_y) {
        std::vector<Eigen::VectorXd> obstacles;
        std::vector<double> radii;
        if (auto grid_checker = std::dynamic_pointer_cast<OccupancyGridObstacleChecker>(obstacle_checker_)) {
            obstacles = grid_checker->getObstaclesInRange(robot_x, robot_y);
        } else if (auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker_)) {
            for (const auto& obstacle_pos : gazebo_checker->getObstaclePositions()) {
                Eigen::VectorXd vec(2);
                vec << obstacle_pos.position;
                obstacles.push_back(vec);
                radii.push_back(obstacle_pos.radius);
            }
            Eigen::Vector2d robot_pos = gazebo_checker->getRobotPosition();
            obstacles.push_back(robot_pos);
            radii.push_back(1.0);
        }
        visualizer_->visualizeCylinder(obstacles, radii, "map");
    }
};