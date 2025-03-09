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
 * TODO: one thing about your implementation is that you can add new nodes but you just have to add it to kdtree and the neighbors need to be calculated in near on itw own. all in all you can add a feature to add a node to the scene if you want just and it doesnt have to be random and can be considered as users help in narrow corridor!
 *       But it seems easier to just connect the newly added node fast instead of heappush to vopen and leave it to plan function
 *       maybe we should add batch of nodes so that fmtx would be useful i don't know
 *       im merely thinking about the narrow pathway problem
 * TODO: You can add hold static obstalce in gazeboObstacleChecker for static envrionment with use_range and use_robot test!
 *       for dynamic obstalce we ignore their last know location 
 * 
 * WARN: some nodes might not get the chance to connect so they'll stay in vUnvisited (and they are not on sample on obstalces!) ---> the  reason they stay is because of object inflation you put not because of persistent vPromising
 * 
 * 
 * TODO: CAN I go back and forth between the main plan function and the for loop that pushes nodes into v open ! so that i go there when its necessary and the robot nodes not been reached and then provide more v open nodes for it!
 *       AT THE END OF THE WHILE LOOP PUT AN IF CONDITON TO CHECK IF VOPEN IS EMPTY BUT THE ROBOT NODE INDEX DOESNT HAVE A PARENT THEN YOU NEED MORE VOPEN FROM SOME OTHER SIDE!
 *       for the refill vopen function maybe use kd tree querying the robot_node_index with the radius of search (to be found!) and then loop thorugh that chunk and if any of those nodes are in v unvisted then add their neighbors to the vopen like the for loop in the update samples!  or maybe not use raidus but a batch size of 100 closest nodes!
 *       BUT IM NOT SURE IF IT WORKS. YOU HAVE TO COMPLETELY KNOW ABOUT THE REGION AND PU ALL THE NECESSARY NODES INTO VOPEN FOR THAT REGION SO THAT THE PLAN CONNECTION PROCEDURE DOES ITS JOB
 * 
 */

#include "motion_planning/state_space/euclidean_statespace.hpp"
#include "motion_planning/planners/planner_factory.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/occupancygrid_obstacle_checker.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/utils/ros2_manager.hpp"
#include "motion_planning/utils/parse_sdf.hpp"

#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <thread>
#include <atomic>

std::atomic<bool> running{true}; // Flag to control the infinite loop
pid_t child_pid = -1;

void runRosGzBridge() {
    // Launch parameter_bridge as a child process with suppressed logs
    child_pid = fork();
    if (child_pid == 0) {
        // Child process: Execute the ros_gz_bridge command with suppressed logs
        execlp("ros2", "ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", 
               "-p", "config_file:=/home/sohail/jazzy_ws/src/simple_robot/params/ros_gz_bridge.yaml",
               "--log-level", "error", // Suppress logs below "error" severity
               (char *)NULL);
        // If execlp fails, print an error and exit
        std::cerr << "Failed to launch parameter_bridge: " << strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
    } else if (child_pid < 0) {
        // Fork failed
        std::cerr << "Failed to fork process: " << strerror(errno) << std::endl;
        return;
    }

    // Parent process: Wait for the child process to finish
    int status;
    waitpid(child_pid, &status, 0);
    if (WIFEXITED(status)) {
        std::cout << "ros_gz_bridge exited with status: " << WEXITSTATUS(status) << std::endl;
    } else {
        std::cerr << "ros_gz_bridge terminated abnormally" << std::endl;
    }
}

void sigint_handler(int sig) {
    if (child_pid > 0) {
        // Terminate the child process
        kill(child_pid, SIGTERM);
        waitpid(child_pid, nullptr, 0);
        child_pid = -1;
    }
    running = false; // Stop the main loop
    rclcpp::shutdown();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Set up SIGINT handler
    struct sigaction sa;
    sa.sa_handler = sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGINT, &sa, nullptr) < 0) {
        std::cerr << "Failed to set signal handler: " << strerror(errno) << std::endl;
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    // Launch ros_gz_bridge in a separate thread
    std::thread ros_gz_bridge_thread(runRosGzBridge);
    ros_gz_bridge_thread.detach(); // Detach the thread to run independently


    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Create Params for Controller
    Params controller_params;
    controller_params.setParam("kp_angular", 1.0);
    controller_params.setParam("max_angular_speed", 1.5);
    controller_params.setParam("lookahead_distance", 1.5);
    controller_params.setParam("control_loop_dt", 0.05);

    // Create Params for Nav2Controller
    Params nav2_controller_params;
    nav2_controller_params.setParam("follow_path_topic", "/follow_path");
    nav2_controller_params.setParam("max_speed", 2.0);

    // Create Params for ROS2Manager
    Params ros2_manager_params;
    ros2_manager_params.setParam("follow_path", true);
    ros2_manager_params.setParam("controller", "pure_pursuit");

    Params gazebo_params;
    gazebo_params.setParam("robot_model_name", "tugbot");
    gazebo_params.setParam("default_robot_x", 48.0); // in case you want to test the planner without running gz sim
    gazebo_params.setParam("default_robot_y", 48.0);
    gazebo_params.setParam("world_name", "default");
    gazebo_params.setParam("use_range", false);
    gazebo_params.setParam("sensor_range", 20.0);
    gazebo_params.setParam("persistent_static_obstacles", true);

    Params planner_params;
    planner_params.setParam("num_of_samples", 5000);
    planner_params.setParam("use_kdtree", true); // for now the false is not impelmented! maybe i should make it default! can't think of a case of not using it but i just wanted to see the performance without it for low sample cases.
    planner_params.setParam("kdtree_type", "NanoFlann");
    planner_params.setParam("partial_update", false);
    planner_params.setParam("obs_cache", true);
    planner_params.setParam("partial_plot", false);
    planner_params.setParam("use_heuristic", false);


    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Create Controller and Nav2Controller objects
    auto controller = std::make_shared<Controller>(controller_params);
    auto nav2_controller = std::make_shared<Nav2Controller>(nav2_controller_params); // TODO: later i should omit this from ros2 manager by creating a highe level abstraction for controllers!

    /////////////////////////////////////////////////////////////////////////////////////////////////

    // Create ROS node
    auto node = std::make_shared<rclcpp::Node>("fmtx_visualizer");
    auto visualization = std::make_shared<RVizVisualization>(node);

    auto obstacle_radii = parseSdfForObstacleRadii("/home/sohail/gazeb/GAZEBO_MOV/my_world2_dr.sdf");
    for (auto& el : obstacle_radii) {
        std::cout << el.first << "  " << el.second << "\n";
    }
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(gazebo_params, obstacle_radii);
    auto ros2_manager = std::make_shared<ROS2Manager>(obstacle_checker, visualization, controller, nav2_controller, ros2_manager_params);





    bool use_rviz_goal = false;



    int dim = 2;
    Eigen::VectorXd start_position = Eigen::VectorXd::Zero(dim);
    // if (use_rviz_goal==true){
        // Start a spinner thread to process ROS2Manager's callbacks
        // std::thread ros2_spinner_thread([ros2_manager]() {
        //     rclcpp::spin(ros2_manager);
        // });
        // ros2_spinner_thread.detach(); // Detach to run independently
        start_position = ros2_manager->getStartPosition(); // I put a cv in here so i needed the above thread so it wouldn't stop the ros2 callbacks! --> also if use_rviz_goal==false no worries because the default value for this func is 0,0
    // }


    bool follow_path = true;
    auto problem_def = std::make_shared<ProblemDefinition>(dim);
    problem_def->setStart(start_position); //Root of the tree
    // problem_def->setStart(Eigen::VectorXd::Zero(dim));
    problem_def->setGoal(Eigen::VectorXd::Ones(dim) * 50); // where the robot starts!
    problem_def->setBounds(-50, 50);



    std::unique_ptr<StateSpace> statespace = std::make_unique<EuclideanStateSpace>(dim, 5000);
    std::unique_ptr<Planner> planner = PlannerFactory::getInstance().createPlanner(PlannerType::FMTX, std::move(statespace),problem_def, obstacle_checker);
    planner->setup(planner_params, visualization);

    // auto start = std::chrono::high_resolution_clock::now();
    // planner->plan();
    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // if (duration.count() > 0)
    //     std::cout << "Time taken for the Static env: " << duration.count() << " milliseconds\n";

    while (running && rclcpp::ok()) {
        if (ros2_manager->hasNewGoal()) {
            start_position = ros2_manager->getStartPosition();
            problem_def->setStart(start_position); //Root of the tree
            planner->setup(planner_params, visualization);
            // planner->plan();
        }



        auto obstacles = obstacle_checker->getObstaclePositions();
        auto robot = obstacle_checker->getRobotPosition();
        dynamic_cast<FMTX*>(planner.get())->setRobotIndex(robot);

        auto start = std::chrono::high_resolution_clock::now();
        dynamic_cast<FMTX*>(planner.get())->updateObstacleSamples(obstacles);
        planner->plan();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (duration.count() > 0)
            std::cout << "Time taken for the update : " << duration.count() << " milliseconds\n";


        std::vector<Eigen::VectorXd> shortest_path_;
        // shortest_path_ = dynamic_cast<FMTX*>(planner.get())->getPathPositions();
        shortest_path_ = dynamic_cast<FMTX*>(planner.get())->getSmoothedPathPositions(3,3);
        ros2_manager->followPath(shortest_path_);
  
        // dynamic_cast<FMTX*>(planner.get())->visualizePath(dynamic_cast<FMTX*>(planner.get())->getPathIndex());
        dynamic_cast<FMTX*>(planner.get())->visualizeSmoothedPath(shortest_path_);
        dynamic_cast<FMTX*>(planner.get())->visualizeTree();
        rclcpp::spin_some(ros2_manager);
    }

    // // Start the timer
    // auto start_time = std::chrono::high_resolution_clock::now();

    // // Run the loop for 20 seconds
    // while (std::chrono::duration_cast<std::chrono::seconds>(
    //         std::chrono::high_resolution_clock::now() - start_time).count() < 20) {
    //     auto obstacles = obstacle_checker->getObstaclePositions();
    //     auto robot = obstacle_checker->getRobotPosition();
    //     if (robot(0) != 0.0 && robot(1) != 0.0 && use_robot==true) // Else it will only use the setGoal to set the vbot
    //         dynamic_cast<FMTX*>(planner.get())->setRobotIndex(robot);
    //     dynamic_cast<FMTX*>(planner.get())->updateObstacleSamples(obstacles);

    //     // dynamic_cast<FMTX*>(planner.get())->visualizePath(dynamic_cast<FMTX*>(planner.get())->getPathIndex());
    //     // dynamic_cast<FMTX*>(planner.get())->visualizeTree();
    //     rclcpp::spin_some(ros2_manager);
    // }


    
    // Cleanup
    if (child_pid > 0) {
        kill(child_pid, SIGTERM);
        waitpid(child_pid, nullptr, 0);
    }
    rclcpp::shutdown();
    return 0;
}







