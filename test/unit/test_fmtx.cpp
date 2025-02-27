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
 * TODO: implement the max length to find the scaling factor! 
 * TODO: how to make the set difference faster in the first place? is it possible? 
 * TODO: whats wrong with this philosophy? we add it to unvisted (prosmising) but we don't change its status ! just to recheck if its worth it! --> in the end it might stay in the unvisted so we clear the unvisted! -->if it gets better then fine and if it doesnt then we keep the current one! because the cost must be better than the current one! --> with this we don't need to add the parent of the promising!  --> but i guess we can't clear the vUnvisted because of later early exit! we need to keep track of nodes that we haven't checked before? --> test this with python! --> maybe as before keep track of the promising in a different set also!
 *       but the above idea makes sense because when we add obstalce we need to make the children's cost to inf because now the newCost most certainly is more than their previous cost! but in remove obstalce the new cost is most certainly is less than their previous cost so we don't need to invalidate any thing and if they arelucky they'll be updated anyways
 *       about the keep tracking of the vUnvisted: tracking the addobstacle nodes is easy! --> they are already in the vunvisted for that pass (or you can loop thorugh them to see whihc node has cost of inf! which you don't have to since as i told you ahve them and can store them!) , but about the remove obstalce parts its hard since you find them as you go so nothing to store and you would hope if you need those parts of the map the while loop handles them! --> and also you end up clearing the vPromsing to not running the algoithm for no reason! --> also for this idea you can't v_unvisted_set.clear() because you need them --> or maybe you can create another set to store the not updated ones! --> but according to my experience don't create new variables because you'll end up doing redundant works in other places
 * TODO: to implement the above idea forget about using a robot. just change the goal place randomly on different nodes and see the tree! --> sometimes near the root to make the algorithm to store and sometimes on the end of the leaves to test the stored!
 * TODO: make all of this independent of ros2 (is visualization marker can be replaced with direct rviz api) 
 * 
 * 
 * 
 * 
 * WARN: some nodes might not get the chance to connect so they'll stay in vUnvisited (and they are not on sample on obstalces!) ---> the  reason they stay is because of object inflation you put not because of persistent vPromising
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

    bool use_robot = false; 
    bool using_factory = true;
    int dim = 2;
    auto problem_def = std::make_unique<ProblemDefinition>(dim);
    problem_def->setStart(Eigen::VectorXd::Zero(dim));
    problem_def->setGoal(Eigen::VectorXd::Ones(dim)*50);
    problem_def->setBounds(-50, 50);

    PlannerParams params;
    params.setParam("num_of_samples", 10000);
    params.setParam("use_kdtree", true);
    params.setParam("kdtree_type", "NanoFlann");


    std::unique_ptr<StateSpace> statespace = std::make_unique<EuclideanStateSpace>(dim, 5000);

    std::unique_ptr<Planner> planner;

    if (using_factory)
        planner = PlannerFactory::getInstance().createPlanner(PlannerType::FMTX, std::move(statespace), std::move(problem_def) ,obstacle_checker);
    else
        planner = std::make_unique<FMTX>(std::move(statespace), std::move(problem_def) , obstacle_checker);
    planner->setup(std::move(params) , visualization);

    // auto robot = obstacle_checker->getRobotPosition();
    // if (robot(0) != 0.0 && robot(1) != 0.0 && use_robot==true) // Else it will only use the setGoal to set the vbot
    //     dynamic_cast<FMTX*>(planner.get())->setRobotIndex(robot);
    // Plan the static one!
    planner->plan();

    while (true) {
        auto obstacles = obstacle_checker->getObstaclePositions();
        auto robot = obstacle_checker->getRobotPosition();
        if (robot(0) != 0.0 && robot(1) != 0.0 && use_robot==true) // Else it will only use the setGoal to set the vbot
            dynamic_cast<FMTX*>(planner.get())->setRobotIndex(robot);
        dynamic_cast<FMTX*>(planner.get())->updateObstacleSamples(obstacles);
        // planner->plan();

        dynamic_cast<FMTX*>(planner.get())->visualizePath(dynamic_cast<FMTX*>(planner.get())->getPathIndex());
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
