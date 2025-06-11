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
 * INSIGHT: What would negative edge weight in dijkstra mean here?  
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


#include <gz/transport/Node.hh>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/server_control.pb.h>  // Often contains Boolean definition
#include <gz/msgs/boolean.pb.h>  // For Boolean response

// 2. Corrected simulation control function
void resetAndPlaySimulation()
{
    // Create Gazebo transport node
    gz::transport::Node node;
    
    // Reset the world
    {
        gz::msgs::WorldControl reset_req;
        reset_req.mutable_reset()->set_all(true);
        
        // Boolean response type is now properly included
        gz::msgs::Boolean reset_res;
        bool result;
        unsigned int timeout = 3000; // ms
        
        bool executed = node.Request("/world/default/control", 
                                   reset_req,
                                   timeout,
                                   reset_res,
                                   result);
        
        if (!executed || !result || !reset_res.data()) {
            std::cerr << "Failed to reset world" << std::endl;
            return;
        }
        std::cout << "World reset successfully" << std::endl;
    }

    // Brief pause to ensure reset completes
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Play the simulation
    {
        gz::msgs::WorldControl play_req;
        play_req.set_pause(false);  // Unpause to play
        
        gz::msgs::Boolean play_res;
        bool result;
        
        bool executed = node.Request("/world/default/control",
                                   play_req,
                                   3000,
                                   play_res,
                                   result);
        
        if (!executed || !result || !play_res.data()) {
            std::cerr << "Failed to play simulation" << std::endl;
            return;
        }
        std::cout << "Simulation playing successfully" << std::endl;
    }
}








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
    // ─────────────────────────────────────────────────────────────────────────────
    // 1) Parse your flags
    int num_samples = 5000;
    double factor = 2.0;
    unsigned int seed = 42;
    int run_secs = 30;


    for(int i = 1; i < argc; ++i) {
        std::string s{argv[i]};
        if(s == "--samples" && i+1 < argc) {
        num_samples = std::stoi(argv[++i]);
        }
        else if(s == "--factor" && i+1 < argc) {
        factor = std::stod(argv[++i]);
        }
        else if(s == "--seed" && i+1 < argc) {
        seed = std::stoi(argv[++i]);
        }
        else if(s == "--duration" && i+1 < argc) {
        run_secs = std::stoi(argv[++i]);
        }
        else if(s == "--help") {
        std::cout << "Usage: " << argv[0]
                    << " [--samples N] [--factor F] [--seed S] [--duration T]\n";
        return 0;
        }
    }

    // 2) Seed RNG
    std::srand(seed);
    std::cout << "[INFO] seed=" << seed
                << ", samples=" << num_samples
                << ", factor=" << factor
                << ", duration=" << run_secs << "s\n";

    // ─────────────────────────────────────────────────────────────────────────────




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
    gazebo_params.setParam("inflation", 0.0); //2.0 meters --> this will be added to obstalce radius when obstalce checking --> minimum should be D-ball containing the robot
    gazebo_params.setParam("persistent_static_obstacles", true);

    Params planner_params;
    planner_params.setParam("num_of_samples", num_samples);
    planner_params.setParam("factor", factor);
    planner_params.setParam("use_kdtree", true); // for now the false is not impelmented! maybe i should make it default! can't think of a case of not using it but i just wanted to see the performance without it for low sample cases.
    planner_params.setParam("kdtree_type", "NanoFlann");
    planner_params.setParam("partial_update", false); //update till the robot's costToInit
    planner_params.setParam("static_obs_presence", false); // to not process static obstalces twice because obstacle checker keeps sending all the obstalces! i geuss the persisten_static_obstalces needs to be true always
    /*
        we can cache and its useful because near obstacle there comes a time that too many z indices came up with the same best neighbor node for specific x index
        and since there is an obs in between then we end up needing to re check the same obstacle check between nodes
        I reckon if i use blocked_neighbor variable then we won't need this but that blocked_neighbor variable introduces it own overhead that only worth to use
        if we are using fmta 

        another thing i notices is this doesnt help much with performance in my 2D case. but im gonna leave it in case i have time to test it in more dimensions
    
    */
    planner_params.setParam("obs_cache", false); // TODO: later i should implement it in the fmt node (edge info) it self but right now im using a hash map in fmtx it self which i dont think is efficient but at the same time since im in replanning mode then restarting the obs status of edges is also maybe a challenge but i think its doable
    planner_params.setParam("partial_plot", false);
    planner_params.setParam("use_heuristic", false); // TODO: I need to verify if its legit workingor not.
    planner_params.setParam("ignore_sample", false); // false: no explicit obstalce check  -  true: explicit obstalce check in dynamic update --> when ignore_sample true the prune is not happening anymore so doesnt matter what you put there
    /*
        right now ignore samples is being used with specific way of finding the samples and also the collision check also happens in fmt expand
        later maybe i could combine it with obstale aware distance and no collision checks and see what happens
    */
    planner_params.setParam("prune", false); // prune == true means do an obstalce check in handlAdd/Remove and set the neighbor cost to inf and DO NOT  obstalce check in plan , prune==false means do not do an obstalce check in handleAdd/Remove and delay it in plan --> the delayed part makes it more expensive in case of high obstalce but in case of low obstalce its faster! (also for high number of samples the delayed part is slower)--> prune true overall is faster i guess
    /*
        IMPORTANT NOTE: prune vs plan? in prune we do obstacle check in local vicinity of obstalce and set cost to neighbor to inf in add obstalce and reset in remove obstalce
                        and since we invalidated the edges between those nodes on obstalce and their neighbor, we don't need to do an obstacle check in plan function 
                        but i wonder what if we do not do an obstacle check in add/remove obstalce and postpone the check to plan ? this is in line with fmt philosophy but
                        the thing is then we have to do lots of obstacle checks for all the orphaned edges! as opposed to do only local obstacle checks so the question boild down
                        to number of obstacle checks in local vicnity of the obstalce with ALL their neighbors and the number of delayed obstacle checks in plan function where only the query
                        current edge-> best_neighbor edge. in my tests the prune version where we do not delay checks works faster but maybe at high dimensional space its better to use the delayed version!
                        thats another topic of research i'll do later!

                        all in all prune is like doing the rrtx approach in setting the distance to inf based on explicit obstacle check between a node in obstalce and its neighbors

        NEW UPDATE: in 30 obstalce case testing with the whole environment visible doing the fmt style was much better!--> maybe i should put two variants!!! --> Need to decide !
    */

    /////////////////////////////////////////////////////////////////////////////////////////////////

    // Create ROS node
    auto node = std::make_shared<rclcpp::Node>("fmtx_visualizer");
    auto visualization = std::make_shared<RVizVisualization>(node);

    auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/dynamic_world.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/static_world2.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/static_removable_world.sdf");
    // auto obstacle_info = parseSdfObstacles("/home/sohail/gazeb/GAZEBO_MOV/static_world_box.sdf");
    for (const auto& [name, info] : obstacle_info) {
        std::cout << name << ": " << info << "\n";
    }
    auto obstacle_checker = std::make_shared<GazeboObstacleChecker>(gazebo_params, obstacle_info);

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



    std::shared_ptr<StateSpace> statespace = std::make_shared<EuclideanStateSpace>(dim, 30000, seed);
    std::unique_ptr<Planner> planner = PlannerFactory::getInstance().createPlanner(PlannerType::FMTX, statespace,problem_def, obstacle_checker);
    planner->setup(planner_params, visualization);

    auto start = std::chrono::high_resolution_clock::now();
    planner->plan(); // Doing static plan first just to because we are trying to compare the performance of replanning with RRTx, otherwise it can be commented out
    dynamic_cast<FMTX*>(planner.get())->dumpTreeToCSV("tree_run_fmtx.csv");
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken for the update : " << duration.count() 
                << " milliseconds\n";


    // //----------- Waiting for the Sim Clock to start ------------ //
    // bool simulation_is_paused = true;
    // auto node_clock = ros2_manager->get_clock();
    // // We'll store the initial sim time
    // rclcpp::Time last_time = node_clock->now();
    // std::cout << "[DEBUG] Initially, last_time = " << last_time.seconds() 
    //         << " (sim seconds)\n";
    // std::cout << "[INFO] Waiting for gz-sim to unpause...\n";

    // while (rclcpp::ok() && simulation_is_paused)
    // {
    //     // 1) Spin to process any incoming clock message
    //     rclcpp::spin_some(ros2_manager);

    //     // 2) Get current sim time
    //     rclcpp::Time current_time = node_clock->now();
    //     double dt = (current_time - last_time).seconds();

    //     // // 3) Print debug
    //     // std::cout << "[DEBUG] last_time=" << last_time.seconds() 
    //     //         << ", current_time=" << current_time.seconds() 
    //     //         << ", dt=" << dt << "\n";

    //     // 4) Check if it’s advanced
    //     if (current_time > last_time) {
    //         std::cout << "[DEBUG] => current_time is strictly greater than last_time, so sim is unpaused.\n";
    //         simulation_is_paused = false;
    //         std::cout << "[INFO] Simulation unpaused; starting to log data.\n";
    //     }
    //     else {
    //         // If we land here, sim time hasn't advanced since last check
    //         // std::cout << "[DEBUG] => Simulation still paused, waiting...\n";
    //         rclcpp::sleep_for(std::chrono::milliseconds(1));
    //     }

    //     last_time = current_time;
    // }
    // //----------------------------------------------------------- //


    resetAndPlaySimulation();



    // rclcpp::Rate loop_rate(2); // 2 Hz (500ms per loop)
    rclcpp::Rate loop_rate(30); // 10 Hz (100ms per loop)
    auto global_start = std::chrono::steady_clock::now();

    // Suppose you have a boolean that decides if we want a 20s limit
    bool limited = true; 

    // Capture the "start" time if we plan to limit the loop
    auto start_time = std::chrono::steady_clock::now();
    auto time_limit = std::chrono::seconds(run_secs);

    std::vector<double> sim_durations;
    std::vector<std::tuple<double, double>> sim_duration_2;
    // The main loop
    while (running && rclcpp::ok()) {

        if (limited) {
            auto now = std::chrono::steady_clock::now();
            if (now - start_time > time_limit) {
                std::cout << "[INFO] time_limit seconds have passed. Exiting loop.\n";
                break;  // exit the loop
            }
        }

        if (ros2_manager->hasNewGoal()) {
            start_position = ros2_manager->getStartPosition(); 
            auto snapshot = obstacle_checker->getAtomicSnapshot();
            problem_def->setStart(start_position);
            problem_def->setGoal(snapshot.robot_position);
            planner->setup(planner_params, visualization);
            // planner->plan();   // if you wanted to do it right here
        }


        auto snapshot = obstacle_checker->getAtomicSnapshot();
        auto& obstacles = snapshot.obstacles;
        auto& robot = snapshot.robot_position;
        // std::cout<<"obstalce size: "<<obstacles.size()<<"\n";

        dynamic_cast<FMTX*>(planner.get())->setRobotIndex(robot);
        auto start = std::chrono::steady_clock::now();
        dynamic_cast<FMTX*>(planner.get())->updateObstacleSamples(obstacles);
        planner->plan();
        auto end = std::chrono::steady_clock::now();

        // Original metric (preserved)
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (duration.count() > 0) {
            std::cout << "time taken for the update : " << duration.count() 
                    << " milliseconds\n";
        }
        sim_durations.push_back(duration.count());

        // New metrics (elapsed_s, duration_ms)
        double elapsed_s = std::chrono::duration<double>(start - global_start).count();
        double duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
        sim_duration_2.emplace_back(elapsed_s, duration_ms);


        // std::vector<Eigen::VectorXd> shortest_path_ = dynamic_cast<FMTX*>(planner.get())->getSmoothedPathPositions(5, 2);
        // ros2_manager->followPath(shortest_path_);

        // dynamic_cast<FMTX*>(planner.get())->visualizeSmoothedPath(shortest_path_);
        // dynamic_cast<FMTX*>(planner.get())->visualizeHeapAndUnvisited();
        dynamic_cast<FMTX*>(planner.get())->visualizeTree();
        rclcpp::spin_some(ros2_manager);
        loop_rate.sleep();
    }


    const bool SAVE_TIMED_DATA = true; // Set to false to save raw durations

    int num_of_samples_ = planner_params.getParam<int>("num_of_samples");

    if (limited) {
        std::time_t now_time = std::time(nullptr); 
        std::tm* local_tm = std::localtime(&now_time);

        int day    = local_tm->tm_mday;
        int month  = local_tm->tm_mon + 1;
        int year   = local_tm->tm_year + 1900;
        int hour   = local_tm->tm_hour;
        int minute = local_tm->tm_min;
        int second = local_tm->tm_sec;

        // Create base filename with timestamp and samples
        std::string planner_type = "fmtx";
        std::string base_filename = "sim_" + planner_type + "_" +
            std::to_string(num_of_samples_) + "samples_" + 
            std::to_string(day) + "_" +
            std::to_string(month) + "_" +
            std::to_string(year) + "_" +
            std::to_string(hour) + "_" +
            std::to_string(minute) + "_" +
            std::to_string(second);

        if (SAVE_TIMED_DATA) {
            // Save timed data (elapsed_s, duration_ms)
            std::string filename = base_filename + "_timed.csv";
            std::cout << "Writing timed durations to: " << filename << std::endl;

            std::ofstream out(filename);
            if (!out.is_open()) {
                std::cerr << "Error: failed to open " << filename << std::endl;
                return 1;
            }

            out << "elapsed_s,duration_ms\n"; // CSV header
            for (const auto& [elapsed, duration] : sim_duration_2) {
                out << elapsed << "," << duration << "\n";
            }
            out.close();
        } else {
            // Save raw durations (legacy format)
            std::string filename = base_filename + "_raw.csv";
            std::cout << "Writing raw durations to: " << filename << std::endl;

            std::ofstream out(filename);
            if (!out.is_open()) {
                std::cerr << "Error: failed to open " << filename << std::endl;
                return 1;
            }

            for (const auto& d : sim_durations) {
                out << d << "\n";
            }
            out.close();
        }

        std::cout << "Done writing CSV.\n";
    }
    
    // Cleanup
    if (child_pid > 0) {
        kill(child_pid, SIGTERM);
        waitpid(child_pid, nullptr, 0);
    }
    rclcpp::shutdown();
    return 0;
}







