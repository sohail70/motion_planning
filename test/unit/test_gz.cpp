#include "motion_planning/utils/ros2_manager.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/parse_sdf.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("visualizer");
    auto obstacle_radii = parseSdfForObstacleRadii("/home/sohail/gazeb/GAZEBO_MOV/my_world2.sdf");
    for (auto& el : obstacle_radii) {
        std::cout<<el.first <<"  " << el.second << "\n";
    }
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>("tugbot", obstacle_radii); // Robot model name and obstacle radius

    auto visualizer = std::make_shared<RVizVisualization>(node);

    auto ros2_manager = std::make_shared<ROS2Manager>(obstacle_checker, visualizer);

    rclcpp::spin(ros2_manager);
    rclcpp::shutdown();
    return 0;
}