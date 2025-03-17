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
#include "motion_planning/control/dwa_planner.hpp"

class ROS2Manager : public rclcpp::Node {
public:
    ROS2Manager(
        std::shared_ptr<ObstacleChecker> obstacle_checker,
        std::shared_ptr<RVizVisualization> visualizer,
        std::shared_ptr<Controller> controller,
        std::shared_ptr<Nav2Controller> nav2_controller,
        std::shared_ptr<DWAPlanner> dwa_planner,  // Add DWA planner
        const Params& params)
        :  Node("ros2_manager", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", params.getParam<bool>("use_sim_time"))})),
          obstacle_checker_(obstacle_checker),
          visualizer_(visualizer),
          controller_(controller),
          nav2_controller_(nav2_controller),
          dwa_planner_(dwa_planner),  // Initialize DWA planner
        //   tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
        //   tf_listener_(*tf_buffer_),
          current_goal_(Eigen::Vector3d::Zero()) {


        // Load ROS2Manager-specific parameters
        follow_path_ = params.getParam<bool>("follow_path");
        std::string controller_type_ = params.getParam<std::string>("controller");
        
        // Controller type handling
        if (controller_type_ == "pure_pursuit") {
            use_pure_pursuit_ = true;
        } else if (controller_type_ == "nav2") {
            use_nav_ = true;
        } else if (controller_type_ == "dwa") {
            use_dwa_ = true;
        }

        // Initialize ROS2 components
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        if (std::dynamic_pointer_cast<OccupancyGridObstacleChecker>(obstacle_checker_)) {
            map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", 10, std::bind(&ROS2Manager::mapCallback, this, std::placeholders::_1));
        }

        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&ROS2Manager::updateCallback, this));

        goal_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&ROS2Manager::goalPoseCallback, this, std::placeholders::_1));

        if (follow_path_) {
            if (use_nav_) {
                control_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(10),
                    std::bind(&ROS2Manager::nav2ControlLoop, this));
            } else {
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

 

    void nav2ControlLoop() {
        if (current_path_.empty()) return;
        nav2_controller_->sendPathToNav2(current_path_);
    }

    Eigen::Vector3d getLocalGoal(const std::vector<Eigen::Vector3d>& global_path,
                               const Eigen::Vector2d& robot_pos) {
        if(global_path.empty()) return Eigen::Vector3d::Zero();

        // Find nearest point using sliding window from last index
        size_t nearest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        const size_t search_window = std::min<size_t>(last_nearest_idx_+20, global_path.size());
        
        for(size_t i = last_nearest_idx_; i < search_window; ++i) {
            double dist = (global_path[i].head<2>() - robot_pos).norm();
            if(dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        last_nearest_idx_ = nearest_idx;

        // Find lookahead point
        double accumulated_dist = 0.0;
        for(size_t i = nearest_idx; i < global_path.size()-1; ++i) {
            accumulated_dist += (global_path[i+1].head<2>() - global_path[i].head<2>()).norm();
            if(accumulated_dist >= LOOKAHEAD_DISTANCE) {
                return global_path[i+1];
            }
        }
        
        return global_path.back();  // Fallback to final goal
    }

    void followPath(const std::vector<Eigen::VectorXd>& path) {
        current_path_ = path;

        if (use_nav_) {
            nav2_controller_->sendPathToNav2(path);
        } else if (use_pure_pursuit_) {
            controller_->followPath(path);
        } else if (use_dwa_){
            std::vector<Eigen::Vector3d> dwa_path;
            if(!path.empty()) {
                // Convert path with orientation vectors
                for(size_t i=0; i<path.size(); i++) {
                    double yaw = 0;
                    if(i < path.size()-1) {
                        Eigen::Vector2d dir = path[i+1].head<2>() - path[i].head<2>();
                        yaw = std::atan2(dir.y(), dir.x());
                    }
                    dwa_path.emplace_back(path[i][0], path[i][1], yaw);
                }

                // Update local goal
                RobotPose pose = getRobotPose();
                current_goal_ = getLocalGoal(dwa_path, pose.position);
            }

            dwa_global_path_ = dwa_path;
            visualizer_->visualizeNodes({Eigen::Vector2d(current_goal_.x(), current_goal_.y())}, 
                                    "map", "1.0 0 0"); // Red

            // Eigen::Vector2d robot_pos = getRobotPose().position;
            // std::cout << "Local Goal Debug:\n"
            //         << "  Robot Position: (" << robot_pos.x() << ", " << robot_pos.y() << ")\n"
            //         << "  Selected Goal: (" << current_goal_.x() << ", " << current_goal_.y() << ")\n"
            //         << "  Distance: " << (robot_pos - current_goal_.head<2>()).norm() << "\n";


        }



    }

private:
    std::shared_ptr<ObstacleChecker> obstacle_checker_;
    std::shared_ptr<RVizVisualization> visualizer_;
    std::shared_ptr<Controller> controller_;
    std::shared_ptr<Nav2Controller> nav2_controller_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    // tf2_ros::TransformListener tf_listener_;
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

    // Add to private members
    Eigen::Vector3d current_goal_;
    geometry_msgs::msg::Twist current_cmd_vel_;

    const double LOOKAHEAD_DISTANCE = 1.0;  // 1.2m lookahead
    size_t last_nearest_idx_ = 0;



    std::shared_ptr<DWAPlanner> dwa_planner_;
    std::vector<Eigen::Vector3d> dwa_global_path_;
    bool use_dwa_ = false;

    DWAPlanner::State getDWAState() {
        RobotPose pose = getRobotPose();
        return {
            {pose.position.x(), pose.position.y(), getYaw(pose.orientation)},
            {current_cmd_vel_.linear.x, current_cmd_vel_.angular.z}
        };
    }

void controlLoop() {
    RobotPose robot_pose = getRobotPose();
    ControlOutput output;

    // 1) If using DWA:
    if(use_dwa_) {
        DWAVisualization dwa_vis;

        // Optionally visualize the local goal
        // std::vector<Eigen::VectorXd> look_position_;
        // Eigen::VectorXd vec(2);
        // vec << current_goal_.x(), current_goal_.y();
        // look_position_.push_back(vec);
        // visualizer_->visualizeNodes(look_position_, "map","0.0,1.0,1.0");

        // 2) Compute new velocity from DWA
        auto cmd_vel = dwa_planner_->computeVelocityCommand(
            getDWAState(), 
            current_goal_,
            dwa_global_path_,
            dwa_vis
        );

        // Visualize DWA candidate trajectories
        visualizeDWA(dwa_vis);

        // 3) Update the *member* variable (NOT a local shadow variable)
        current_cmd_vel_.linear.x  = cmd_vel[0];
        current_cmd_vel_.angular.z = cmd_vel[1];
    }
    // 4) If using pure pursuit:
    else if(use_pure_pursuit_) {
        output = controller_->controlLoop(robot_pose);

        // Update the *member* variable
        current_cmd_vel_.linear.x  = output.linear_x;
        current_cmd_vel_.angular.z = output.angular_z;

        // // Optionally visualize the lookahead point
        // std::vector<Eigen::VectorXd> look_position_;
        // Eigen::VectorXd vec(2);
        // vec << output.lookahead_point.x(), output.lookahead_point.y();
        // look_position_.push_back(vec);
        // visualizer_->visualizeNodes(look_position_, "map", "0.0,1.0,1.0");
    }

    // 5) Publish the *member* cmd_vel_
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>(current_cmd_vel_);
    cmd_vel_publisher_->publish(std::move(twist_msg));
}

void visualizeDWA(const DWAVisualization& data) {
    // Visualize candidate trajectories as lines
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> trajectory_edges;
    for (const auto& traj : data.candidate_trajectories) {
        for (size_t i = 1; i < traj.size(); ++i) {
            Eigen::VectorXd start(2);
            start << traj[i-1][0], traj[i-1][1];
            Eigen::VectorXd end(2);
            end << traj[i][0], traj[i][1];
            trajectory_edges.emplace_back(start, end);
        }
    }
    visualizer_->visualizeEdges(trajectory_edges, "map", "0.5,0.5,0.5"); // Gray color

    // // Visualize best trajectory as a line
    // std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> best_traj_edges;
    // for (size_t i = 1; i < data.best_trajectory.size(); ++i) {
    //     Eigen::VectorXd start(2);
    //     start << data.best_trajectory[i-1][0], data.best_trajectory[i-1][1];
    //     Eigen::VectorXd end(2);
    //     end << data.best_trajectory[i][0], data.best_trajectory[i][1];
    //     best_traj_edges.emplace_back(start, end);
    // }
    // visualizer_->visualizeEdges(best_traj_edges, "map", "0.0,1.0,0.0"); // Green color

    // // Visualize footprints as points
    // std::vector<Eigen::VectorXd> footprint_points;
    // for (const auto& footprint : data.footprints) {
    //     for (const auto& p : footprint) {
    //         Eigen::VectorXd point(2);
    //         point << p[0], p[1];
    //         footprint_points.push_back(point);
    //     }
    // }
    // visualizer_->visualizeNodes(footprint_points, "map", "0.0,0.0,1.0"); // Blue color
}

// void visualizeDWA(const DWAVisualization& data) {
//     // Convert visualization to use existing RVizVisualization methods
//     std::vector<Eigen::VectorXd> best_path;
//     for(const auto& p : data.best_trajectory) {
//         Eigen::VectorXd point(3);
//         point << p[0], p[1], 0.0;
//         best_path.push_back(point);
//     }
//     visualizer_->visualizeNodes(best_path, "dwa_best", "0.0,1.0,0.0");
// }



    // Add convertToRosPath function
    nav_msgs::msg::Path convertToRosPath(const std::vector<Eigen::VectorXd>& path) {
        nav_msgs::msg::Path ros_path;
        ros_path.header.frame_id = "map";
        ros_path.header.stamp = now();
        
        for(const auto& wp : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = wp[0];
            pose.pose.position.y = wp[1];
            ros_path.poses.push_back(pose);
        }
        return ros_path;
    }



    RobotPose getRobotPose() {
        RobotPose pose;

        // Get the robot's pose from Gazebo or TF
        if (auto gazebo_checker = std::dynamic_pointer_cast<GazeboObstacleChecker>(obstacle_checker_)) {
            pose.position = gazebo_checker->getRobotPosition();
            pose.orientation = gazebo_checker->getRobotOrientation();
        } else {
            // geometry_msgs::msg::TransformStamped transform;
            // try {
            //     transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            //     pose.position = Eigen::Vector2d(transform.transform.translation.x, transform.transform.translation.y);
            //     pose.orientation = Eigen::VectorXd(4);
            //     pose.orientation << transform.transform.rotation.x,
            //                         transform.transform.rotation.y,
            //                         transform.transform.rotation.z,
            //                         transform.transform.rotation.w;
            // } catch (tf2::TransformException& ex) {
            //     RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            //     pose.position = Eigen::Vector2d(0.0, 0.0);
            //     pose.orientation = Eigen::VectorXd::Zero(4);
            // }
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
            // geometry_msgs::msg::TransformStamped transform;
            // try {
            //     transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            //     robot_x = transform.transform.translation.x;
            //     robot_y = transform.transform.translation.y;
            // } catch (tf2::TransformException& ex) {
            //     RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            //     return;
            // }
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


    // Add getYaw implementation
    double getYaw(const Eigen::VectorXd& quat) {
        tf2::Quaternion q(quat[0], quat[1], quat[2], quat[3]);
        return tf2::getYaw(q);
    }

};