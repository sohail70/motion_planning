// // fmtx_ros2_node.cpp
// #include "fmtx.hpp"
// #include "occupancy_grid_obstacle_checker.hpp"
// #include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <memory>

// class FMTXROS2Node : public rclcpp::Node {
// public:
//     FMTXROS2Node()
//         : Node("fmtx_ros2_node") {
//         // Subscribe to the occupancy grid
//         occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//             "/map", 10, std::bind(&FMTXROS2Node::occupancyGridCallback, this, std::placeholders::_1));

//         // Initialize FMTX planner
//         auto statespace = std::make_unique<EuclideanStateSpace>(2, 1000);
//         auto problem_def = std::make_unique<ProblemDefinition>(2);
//         problem_def->setStart(Eigen::Vector2d(0.0, 0.0));
//         problem_def->setGoal(Eigen::Vector2d(10.0, 10.0));

//         // Create FMTX with the occupancy grid obstacle checker
//         fmtx_ = std::make_unique<FMTX>(std::move(statespace), std::move(problem_def), nullptr);
//     }

// private:
//     void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
//         // Update the occupancy grid
//         auto obstacle_checker = std::make_shared<OccupancyGridObstacleChecker>(msg);
//         fmtx_->setObstacleChecker(obstacle_checker);

//         // Run the planner
//         fmtx_->plan();
//     }

//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
//     std::unique_ptr<FMTX> fmtx_;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<FMTXROS2Node>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
