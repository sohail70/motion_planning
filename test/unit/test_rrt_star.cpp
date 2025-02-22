// Copyright 2025 Soheil E.nia


#include "motion_planning/state_space/euclidean_statespace.hpp"
#include "motion_planning/planners/planner_factory.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/occupancygrid_obstacle_checker.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/utils/ros2_manager.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // Create ROS node
    auto node = std::make_shared<rclcpp::Node>("rrtx_visualizer");
    auto visualization = std::make_shared<RVizVisualization>(node);
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>("tugbot", 5.0); // Robot model name and obstacle radius
    auto ros2_manager = std::make_shared<ROS2Manager>(obstacle_checker, visualization);


    bool using_factory = true;
    int dim = 2;
    auto problem_def = std::make_unique<ProblemDefinition>(dim);
    problem_def->setStart(Eigen::VectorXd::Zero(dim));
    problem_def->setGoal(Eigen::VectorXd::Ones(dim)*50);
    problem_def->setBounds(-50, 50);

    PlannerParams params;
    params.setParam("num_of_samples", 5000);
    params.setParam("use_kdtree", true);
    params.setParam("kdtree_type", "NanoFlann");


    std::unique_ptr<StateSpace> statespace = std::make_unique<EuclideanStateSpace>(dim, 5000);

    std::unique_ptr<Planner> planner;

    if (using_factory)
        planner = PlannerFactory::getInstance().createPlanner(PlannerType::RRTX, std::move(statespace), std::move(problem_def) ,obstacle_checker);
    else
        planner = std::make_unique<RRTX>(std::move(statespace), std::move(problem_def) , obstacle_checker);
    planner->setup(std::move(params) , visualization);

    // Plan the static one!
    planner->plan();

    // while (true) {
    //     auto obstacles = obstacle_checker->getObstaclePositions();
    //     dynamic_cast<RRTX*>(planner.get())->updateObstacleSamples(obstacles);
    //     // planner->plan();
    //     dynamic_cast<RRTX*>(planner.get())->visualizeTree();
    //     rclcpp::spin_some(ros2_manager);
    // }


    rclcpp::shutdown();

}
