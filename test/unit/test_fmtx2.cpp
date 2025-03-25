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
 * 
 * OBSERVATION MATH EXTENSION: I think the ignore sample approach is good if you use inflation it pretty much covers that --> maybe a simple formula to find how much minimum inflation is needed to activate ignore sample approach (which should be also dependant on the number of samples)
 *  
 */

#include "motion_planning/state_space/euclidean_statespace.hpp"
#include "motion_planning/planners/planner_factory.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/utils/occupancygrid_obstacle_checker.hpp"
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"
#include "motion_planning/utils/ros2_manager.hpp"
#include "motion_planning/utils/parse_sdf.hpp"
#include <valgrind/callgrind.h>

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
    // // Launch parameter_bridge as a child process with suppressed logs
    // child_pid = fork();
    // if (child_pid == 0) {
    //     // Child process: Execute the ros_gz_bridge command with suppressed logs
    //     execlp("ros2", "ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", 
    //            "-p", "config_file:=/home/sohail/jazzy_ws/src/simple_robot/params/ros_gz_bridge.yaml",
    //            "--log-level", "error", // Suppress logs below "error" severity
    //            (char *)NULL);
    //     // If execlp fails, print an error and exit
    //     std::cerr << "Failed to launch parameter_bridge: " << strerror(errno) << std::endl;
    //     exit(EXIT_FAILURE);
    // } else if (child_pid < 0) {
    //     // Fork failed
    //     std::cerr << "Failed to fork process: " << strerror(errno) << std::endl;
    //     return;
    // }

    // // Parent process: Wait for the child process to finish
    // int status;
    // waitpid(child_pid, &status, 0);
    // if (WIFEXITED(status)) {
    //     std::cout << "ros_gz_bridge exited with status: " << WEXITSTATUS(status) << std::endl;
    // } else {
    //     std::cerr << "ros_gz_bridge terminated abnormally" << std::endl;
    // }
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
    // Create Params for Pure-Pursuit Controller
    Params controller_params;
    controller_params.setParam("kp_angular", 1.2);
    controller_params.setParam("ki_angular", 0.05);
    controller_params.setParam("kd_angular", 0.2);
    controller_params.setParam("cross_track_gain", 0.8);
    controller_params.setParam("max_angular_speed", 0.5);
    controller_params.setParam("lookahead_distance", 1.5);
    controller_params.setParam("control_loop_dt", 0.005);

    // Create Params for Nav2Controller
    Params nav2_controller_params;
    nav2_controller_params.setParam("follow_path_topic", "/follow_path");
    nav2_controller_params.setParam("max_speed", 2.0);


    Params DWA;
    // ========== Core motion limits ==========
    DWA.setParam("max_speed",         3.0);   // Robot can go up to 3 m/s
    DWA.setParam("min_speed",        -1.0);   // Allow reversing if needed
    DWA.setParam("max_yawrate",       0.8);   // Turn rate up to 1.5 rad/s
    DWA.setParam("max_accel",         3.0);   // Accelerate up to 2 m/s^2
    DWA.setParam("max_decel",         3.0);   // Decelerate up to 2 m/s^2
    DWA.setParam("max_dyawrate",      2.0);   // Angular acceleration limit
    DWA.setParam("robot_radius",      0.3);

    // ========== Sampling and horizon ==========
    DWA.setParam("dt",               0.1);    // Simulation time step in DWA
    DWA.setParam("predict_time",     5.0);    // 2s horizon for quick re-planning
    int sim_steps_ = (int)(DWA.getParam<double>("predict_time") / DWA.getParam<double>("dt"));
    // -> 2.0 / 0.1 = 20 steps
    DWA.setParam("sim_steps", sim_steps_);

    DWA.setParam("speed_resolution",  0.1);
    DWA.setParam("yawrate_resolution",0.1);

    // ========== Cost weights ==========
    DWA.setParam("obstacle_cost_gain",  5.0); // Higher => more aggressive obstacle avoidance
    DWA.setParam("speed_cost_gain",     0.3); // Medium => encourages higher speed, but not crazy
    DWA.setParam("goal_cost_gain",      3.0); // Balanced
    DWA.setParam("path_cost_gain",      0.3); // Enough to stay near path, but not too strict

    

    // Create Params for ROS2Manager
    Params ros2_manager_params;
    ros2_manager_params.setParam("use_sim_time", true);
    ros2_manager_params.setParam("follow_path", false);
    ros2_manager_params.setParam("controller", "pure_pursuit");
    // ros2_manager_params.setParam("controller", "dwa");

    Params gazebo_params;
    gazebo_params.setParam("robot_model_name", "tugbot");
    gazebo_params.setParam("default_robot_x", 48.0); // in case you want to test the planner without running gz sim
    gazebo_params.setParam("default_robot_y", 48.0);
    gazebo_params.setParam("world_name", "default");
    gazebo_params.setParam("use_range", false); // use_range and partial_update and use_heuristic are related! --> take care of this later!
    gazebo_params.setParam("sensor_range", 20.0);
    /*
        When you use ignore_sample == true i don't think would need a inflation specially in low sample case --> math can be proved i guess.
    */
    gazebo_params.setParam("inflation", 0.0); //1.5 meters --> this will be added to obstalce radius when obstalce checking --> minimum should be D-ball containing the robot
    gazebo_params.setParam("persistent_static_obstacles", true);

    Params planner_params;
    planner_params.setParam("num_of_samples", 10000);
    planner_params.setParam("use_kdtree", true); // for now the false is not impelmented! maybe i should make it default! can't think of a case of not using it but i just wanted to see the performance without it for low sample cases.
    planner_params.setParam("kdtree_type", "NanoFlann");
    planner_params.setParam("partial_update", true);
    planner_params.setParam("obs_cache", true);
    planner_params.setParam("partial_plot", false);
    planner_params.setParam("use_heuristic", false); // TODO: I need to verify if its legit workingor not.
    planner_params.setParam("ignore_sample", false); // false: no explicit obstalce check  -  true: explicit obstalce check in dynamic update --> when ignore_sample true the prune is not happening anymore so doesnt matter what you put there
    planner_params.setParam("prune", true); // prune == true means do an obstalce check in handlAdd/Remove and set the neighbor cost to inf and DO NOT  obstalce check in plan , prune==false means do not do an obstalce check in handleAdd/Remove and delay it in plan --> the delayed part makes it more expensive in case of high obstalce but in case of low obstalce its faster! (also for high number of samples the delayed part is slower)--> prune true overall is faster i guess
    /*
        IMPORTANT NOTE: prune vs plan? in prune we do obstacle check in local vicinity of obstalce and set cost to neighbor to inf in add obstalce and reset in remove obstalce
                        and since we invalidated the edges between those nodes on obstalce and their neighbor, we don't need to do an obstacle check in plan function 
                        but i wonder what if we do not do an obstacle check in add/remove obstalce and postpone the check to plan ? this is in line with fmt philosophy but
                        the thing is then we have to do lots of obstacle checks for all the orphaned edges! as opposed to do only local obstacle checks so the question boild down
                        to number of obstacle checks in local vicnity of the obstalce with ALL their neighbors and the number of delayed obstacle checks in plan function where only the query
                        current edge-> best_neighbor edge. in my tests the prune version where we do not delay checks works faster but maybe at high dimensional space its better to use the delayed version!
                        thats another topic of research i'll do later!

                        all in all prune is like doing the rrtx approach in setting the distance to inf based on explicit obstacle check between a node in obstalce and its neighbors
    */

    /////////////////////////////////////////////////////////////////////////////////////////////////

    // Create ROS node
    auto node = std::make_shared<rclcpp::Node>("fmtx_visualizer");
    auto visualization = std::make_shared<RVizVisualization>(node);

    auto obstacle_radii = parseSdfForObstacleRadii("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world.sdf");
    // auto obstacle_radii = parseSdfForObstacleRadii("/home/sohail/gazeb/GAZEBO_MOV/static_world2.sdf");
    // auto obstacle_radii = parseSdfForObstacleRadii("/home/sohail/gazeb/GAZEBO_MOV/static_removable_world.sdf");
    for (auto& el : obstacle_radii) {
        std::cout << el.first << "  " << el.second << "\n";
    }
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(gazebo_params, obstacle_radii);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Create Controller and Nav2Controller objects
    auto controller = std::make_shared<Controller>(controller_params);
    auto nav2_controller = std::make_shared<Nav2Controller>(nav2_controller_params); // TODO: later i should omit this from ros2 manager by creating a highe level abstraction for controllers!
    auto dwa_controller =  std::make_shared<DWAPlanner>(DWA,obstacle_checker);

    /////////////////////////////////////////////////////////////////////////////////////////////////

    auto ros2_manager = std::make_shared<ROS2Manager>(obstacle_checker, visualization, controller, nav2_controller,dwa_controller, ros2_manager_params);


    int dim = 2;
    Eigen::VectorXd start_position = ros2_manager->getStartPosition(); // I put a cv in here so i needed the above thread so it wouldn't stop the ros2 callbacks! --> also if use_rviz_goal==false no worries because the default value for this func is 0,0


    auto problem_def = std::make_shared<ProblemDefinition>(dim);
    problem_def->setStart(start_position); //Root of the tree
    // problem_def->setStart(Eigen::VectorXd::Zero(dim));
    problem_def->setGoal(Eigen::VectorXd::Ones(dim) * 50); // where the robot starts!
    problem_def->setBounds(-50, 50);



    std::unique_ptr<StateSpace> statespace = std::make_unique<EuclideanStateSpace>(dim, 30000);
    std::unique_ptr<Planner> planner = PlannerFactory::getInstance().createPlanner(PlannerType::FMTX, std::move(statespace),problem_def, obstacle_checker);
    planner->setup(planner_params, visualization);

    auto start = std::chrono::high_resolution_clock::now();
    planner->plan();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken for the update : " << duration.count() 
                << " milliseconds\n";


    //----------- Waiting for the Sim Clock to start ------------ //
    bool simulation_is_paused = true;
    auto node_clock = ros2_manager->get_clock();
    // We'll store the initial sim time
    rclcpp::Time last_time = node_clock->now();
    std::cout << "[DEBUG] Initially, last_time = " << last_time.seconds() 
            << " (sim seconds)\n";
    std::cout << "[INFO] Waiting for gz-sim to unpause...\n";

    while (rclcpp::ok() && simulation_is_paused)
    {
        // 1) Spin to process any incoming clock message
        rclcpp::spin_some(ros2_manager);

        // 2) Get current sim time
        rclcpp::Time current_time = node_clock->now();
        double dt = (current_time - last_time).seconds();

        // // 3) Print debug
        // std::cout << "[DEBUG] last_time=" << last_time.seconds() 
        //         << ", current_time=" << current_time.seconds() 
        //         << ", dt=" << dt << "\n";

        // 4) Check if it’s advanced
        if (current_time > last_time) {
            std::cout << "[DEBUG] => current_time is strictly greater than last_time, so sim is unpaused.\n";
            simulation_is_paused = false;
            std::cout << "[INFO] Simulation unpaused; starting to log data.\n";
        }
        else {
            // If we land here, sim time hasn't advanced since last check
            // std::cout << "[DEBUG] => Simulation still paused, waiting...\n";
            rclcpp::sleep_for(std::chrono::milliseconds(1));
        }

        last_time = current_time;
    }
    //----------------------------------------------------------- //




    // auto start = std::chrono::high_resolution_clock::now();
    // planner->plan();
    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // if (duration.count() > 0)
    //     std::cout << "Time taken for the Static env: " << duration.count() << " milliseconds\n";



    // rclcpp::Rate loop_rate(2); // 2 Hz (500ms per loop)
    rclcpp::Rate loop_rate(30); // 10 Hz (100ms per loop)

    // Suppose you have a boolean that decides if we want a 20s limit
    bool limited = true;  // or read from params, or pass as an argument

    // Capture the "start" time if we plan to limit the loop
    auto start_time = std::chrono::steady_clock::now();
    auto time_limit = std::chrono::seconds(20);

    std::vector<double> sim_durations;

    // The main loop
    while (running && rclcpp::ok()) {

        // 1) If we are limiting to 20s, check if we've exceeded that
        if (limited) {
            auto now = std::chrono::steady_clock::now();
            if (now - start_time > time_limit) {
                std::cout << "[INFO] 20 seconds have passed. Exiting loop.\n";
                break;  // exit the loop
            }
        }

        // 2) Planner logic ...
        if (ros2_manager->hasNewGoal()) {
            start_position = ros2_manager->getStartPosition(); 
            auto snapshot = obstacle_checker->getAtomicSnapshot();
            problem_def->setStart(start_position);
            problem_def->setGoal(snapshot.robot_position);
            planner->setup(planner_params, visualization);
            // planner->plan();   // if you wanted to do it right here
        }

        // auto obstacles = obstacle_checker->getObstaclePositions();
        // auto robot = obstacle_checker->getRobotPosition();

        auto snapshot = obstacle_checker->getAtomicSnapshot();
        auto& obstacles = snapshot.obstacles;
        auto& robot = snapshot.robot_position;
        
        // // Immediately after getting snapshot:
        // if (obstacles.empty()) {
        //     std::cout << "WARNING: Empty obstacles in snapshot!\n";
        //     continue; // Skip planning iteration
        // }
        
        // // Check robot position validity
        // if (robot.hasNaN() || robot.norm() > 1e6) { 
        //     std::cout << "Invalid robot position: " << robot.transpose() << "\n";
        //     continue;
        // }

        dynamic_cast<FMTX*>(planner.get())->setRobotIndex(robot);
        // auto t0 = std::chrono::high_resolution_clock::now();
        auto start = std::chrono::high_resolution_clock::now();
        dynamic_cast<FMTX*>(planner.get())->updateObstacleSamples(obstacles);
        planner->plan();
        auto end = std::chrono::high_resolution_clock::now();
        // auto t1 = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (duration.count() > 0) {
            std::cout << "Time taken for the update : " << duration.count() 
                    << " milliseconds\n";
        }
        sim_durations.push_back(duration.count());

// // Check if obstacles moved during planning
// auto post_snapshot = obstacle_checker->getAtomicSnapshot();
// if (post_snapshot.obstacles != snapshot.obstacles) {
//   std::cout << "WARNING: Obstacles changed during " 
//             << std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count()
//             << "ms planning!\n";
// }


        std::vector<Eigen::VectorXd> shortest_path_ = dynamic_cast<FMTX*>(planner.get())->getSmoothedPathPositions(5, 2);
        ros2_manager->followPath(shortest_path_);
    
//         std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Let visualization "catch up"



        dynamic_cast<FMTX*>(planner.get())->visualizeSmoothedPath(shortest_path_);
        dynamic_cast<FMTX*>(planner.get())->visualizeTree();
        rclcpp::spin_some(ros2_manager);
        loop_rate.sleep();
    }

    if (limited==true){
        // 1) Get the current local time
        std::time_t now = std::time(nullptr); 
        std::tm* local_tm = std::localtime(&now);

        // 2) Extract day, month, year, hour, minute, second
        int day    = local_tm->tm_mday;           // day of month [1-31]
        int month  = local_tm->tm_mon + 1;        // months since January [0-11]; add 1
        int year   = local_tm->tm_year + 1900;    // years since 1900
        int hour   = local_tm->tm_hour;           // hours since midnight [0-23]
        int minute = local_tm->tm_min;            // minutes after hour [0-59]
        int second = local_tm->tm_sec;            // seconds after minute [0-60]

        // 3) Build your file name, e.g. "sim_times_13_3_2025_14_58_12.csv"
        std::string filename = "sim_times_" +
            std::to_string(day)    + "_" +
            std::to_string(month)  + "_" +
            std::to_string(year)   + "_" +
            std::to_string(hour)   + "_" +
            std::to_string(minute) + "_" +
            std::to_string(second) + ".csv";

        std::cout << "Writing durations to: " << filename << std::endl;

        // 4) Write durations to that file
        std::ofstream out(filename);
        if (!out.is_open()) {
            std::cerr << "Error: failed to open " << filename << std::endl;
            return 1;
        }

        for (auto &d : sim_durations) {
            out << d << "\n";
        }
        out.close();

        std::cout << "Done writing CSV.\n";
    }








    ////////////////////////////////////////////////////////



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


  // Variables for frequency calculation
    // std::deque<std::chrono::milliseconds> loop_times; // Store last N loop durations
    // const size_t window_size = 10; // Number of iterations to average over
    // auto last_time = std::chrono::high_resolution_clock::now();

    // while (running && rclcpp::ok()) {
    //     auto loop_start = std::chrono::high_resolution_clock::now();

    //     if (ros2_manager->hasNewGoal()) {
    //         start_position = ros2_manager->getStartPosition(); // The goal you provided through rviz2
    //         problem_def->setStart(start_position); // Root of the tree
    //         problem_def->setGoal(obstacle_checker->getRobotPosition());
    //         planner->setup(planner_params, visualization);
    //         // planner->plan();
    //     }

    //     auto obstacles = obstacle_checker->getObstaclePositions();
    //     auto robot = obstacle_checker->getRobotPosition();
    //     dynamic_cast<FMTX*>(planner.get())->setRobotIndex(robot);

    //     auto start = std::chrono::high_resolution_clock::now();
    //     dynamic_cast<FMTX*>(planner.get())->updateObstacleSamples(obstacles);
    //     planner->plan();
    //     auto end = std::chrono::high_resolution_clock::now();
    //     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //     if (duration.count() > 0)
    //         std::cout << "Time taken for the update : " << duration.count() << " milliseconds\n";

    //     std::vector<Eigen::VectorXd> shortest_path_;
    //     shortest_path_ = dynamic_cast<FMTX*>(planner.get())->getSmoothedPathPositions(5, 2);
    //     ros2_manager->followPath(shortest_path_);

    //     dynamic_cast<FMTX*>(planner.get())->visualizeSmoothedPath(shortest_path_);
    //     dynamic_cast<FMTX*>(planner.get())->visualizeTree();
    //     rclcpp::spin_some(ros2_manager);

    //     // Calculate loop duration
    //     auto loop_end = std::chrono::high_resolution_clock::now();
    //     auto loop_duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
    //     loop_times.push_back(loop_duration);

    //     // Keep only the last N loop durations
    //     if (loop_times.size() > window_size) {
    //         loop_times.pop_front();
    //     }

    //     // Calculate average loop duration and frequency
    //     if (loop_times.size() == window_size) {
    //         double avg_duration_ms = 0.0;
    //         for (const auto& time : loop_times) {
    //             avg_duration_ms += time.count();
    //         }
    //         avg_duration_ms /= window_size;

    //         double frequency_hz = 1000.0 / avg_duration_ms; // Convert ms to Hz
    //         std::cout << "Average loop frequency: " << frequency_hz << " Hz\n";
    //     }

    //     last_time = loop_end;
    // }




    
    // Cleanup
    if (child_pid > 0) {
        kill(child_pid, SIGTERM);
        waitpid(child_pid, nullptr, 0);
    }
    rclcpp::shutdown();
    return 0;
}







