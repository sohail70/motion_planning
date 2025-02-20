// Copyright 2025 Soheil E.nia
/**
 * TODO: Observer design for map. when it updates we need to check obstacle in fmtx
 * TODO: Create solve() and setPlanner() in Problem Definition Class    
 * TODO: Put variables in fmtx in a struct
 * TODO: instead of vUnvitesd in handleadd obstacle for vopen u need to track the vopens! or elseif the vunivsted gets large you are gonna have problems
 * TODO: why the problem with visual happens? --> oh its the problem with handleremoveobstalce which shoudlnt attach on its own! i guess
 * TODO: Implement Sensor range for obstalce detection otherwise its too demanding to keep amending the tree!
 * TODO: Add the obstalce hash map to the plan algorithm!
 * TODO: trakc the vOpen nodes so that you wouldn't loop over vUnvisted in the handleAdd/Remove functions
 * TODO: the hash you created maybe its best to used ordered containers instead of unordered ones!
 * 
 * 
 * TODO: whats wrong with this philosophy? we add it to unvisted (prosmisin) but we don't change its status ! just to recheck if its worth it! --> in the end it might stay in the unvisted so we clear the unvisted! -->if it gets better then fine and if it doesnt then we keep the current one! because the cost must be better than the current one! --> with this we don't need to add the parent of the promising! 
 */
#include "motion_planning/state_space/euclidean_statespace.hpp"
#include "motion_planning/planners/planner_factory.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/occupancygrid_obstacle_checker.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/utils/ros2_manager.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // Create ROS node
    auto node = std::make_shared<rclcpp::Node>("fmtx_visualizer");
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
    params.setParam("num_of_samples", 1000);
    params.setParam("use_kdtree", true);
    params.setParam("kdtree_type", "NanoFlann");


    std::unique_ptr<StateSpace> statespace = std::make_unique<EuclideanStateSpace>(dim, 5000);

    std::unique_ptr<Planner> planner;

    if (using_factory)
        planner = PlannerFactory::getInstance().createPlanner(PlannerType::FMTX, std::move(statespace), std::move(problem_def) ,obstacle_checker);
    else
        planner = std::make_unique<FMTX>(std::move(statespace), std::move(problem_def) , obstacle_checker);
    planner->setup(std::move(params) , visualization);

    // Plan the static one!
    planner->plan();

    while (true) {
        auto obstacles = obstacle_checker->getObstaclePositions();
        dynamic_cast<FMTX*>(planner.get())->updateObstacleSamples(obstacles);
        // planner->plan();
        dynamic_cast<FMTX*>(planner.get())->visualizeTree();
        rclcpp::spin_some(ros2_manager);
    }


    // // Start the timer
    // auto start_time = std::chrono::high_resolution_clock::now();

    // // Run the loop for 20 seconds
    // while (std::chrono::duration_cast<std::chrono::seconds>(
    //         std::chrono::high_resolution_clock::now() - start_time).count() < 20) {
    //     // Your existing code
    //     auto obstacles = obstacle_checker->getObstaclePositions();
    //     dynamic_cast<FMTX*>(planner.get())->updateObstacleSamples(obstacles);
    //     planner->plan();
    //     dynamic_cast<FMTX*>(planner.get())->visualizeTree();
    //     rclcpp::spin_some(ros2_manager);
    // }


    rclcpp::shutdown();

}
