// Copyright 2025



#include "motion_planning/utils/ros2_manager.hpp"
#include "motion_planning/utils/occupancygrid_obstacle_checker.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("visualizer");
    auto obstacle_checker = std::make_shared<OccupancyGridObstacleChecker>(5.0); // LIDAR range = 15m
    auto visualizer = std::make_shared<RVizVisualization>(node);

    auto ros2_manager = std::make_shared<ROS2Manager>(obstacle_checker, visualizer);

    rclcpp::spin(ros2_manager);
    rclcpp::shutdown();
    return 0;
}




// #include "motion_planning/utils/ros2_manager.hpp"
// #include "motion_planning/utils/occupancygrid_obstacle_checker.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<rclcpp::Node>("visualizer");
//     auto obstacle_checker = std::make_shared<OccupancyGridObstacleChecker>(5.0); // LIDAR range = 5m
//     auto visualizer = std::make_shared<RVizVisualization>(node);

//     auto ros2_manager = std::make_shared<ROS2Manager>(obstacle_checker, visualizer);

//     rclcpp::spin(ros2_manager);
//     rclcpp::shutdown();
//     return 0;
// }