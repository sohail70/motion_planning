// #include <iostream>
// #include <memory>
// #include <vector>
// #include <Eigen/Dense>
// #include <cmath> // For M_PI
// #include "rclcpp/rclcpp.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"
// #include "motion_planning/state_space/dubins_time_statespace.hpp" // Your new header

// /**
//  * @brief This program demonstrates and tests the DubinsTimeStateSpace.
//  * * It performs the following steps:
//  * 1. Initializes a ROS2 node for visualization.
//  * 2. Creates an instance of the DubinsTimeStateSpace with specific vehicle parameters.
//  * 3. Defines a start and an end pose in 4D (x, y, theta, time).
//  * 4. Calls the `steer` function to compute the optimal kinodynamic trajectory.
//  * 5. Visualizes the resulting curved path in RViz as a series of short line segments.
//  * 6. Also visualizes a set of random samples to show how theta and time are distributed.
//  * * To Compile:
//  * You will need to link against rclcpp, visualization_msgs, and your other libraries.
//  * Example in CMakeLists.txt:
//  * * find_package(rclcpp REQUIRED)
//  * find_package(visualization_msgs REQUIRED)
//  * ...
//  * add_executable(test_dubin_time test_dubin_time.cpp ...)
//  * ament_target_dependencies(test_dubin_time rclcpp visualization_msgs ...)
//  * target_link_libraries(test_dubin_time ${your_motion_planning_library})
//  */

// // Helper function to print vectors for debugging
// static void printVec(const Eigen::VectorXd& v) {
//     std::cout << "[ ";
//     for (int i = 0; i < v.size(); ++i) {
//         std::cout << v[i] << (i + 1 < v.size() ? ", " : " ");
//     }
//     std::cout << "]";
// }


// int main(int argc, char** argv) {
//     // --- 1. Initialization ---
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<rclcpp::Node>("dubin_time_test_node");
//     auto visualizer = std::make_shared<RVizVisualization>(node);
    
//     std::cout << "--- DubinsTimeStateSpace Test ---\n";

//     // --- 2. Create the Dubins State Space ---
//     const double min_turning_radius = 5.0; // meters
//     const double min_velocity = 2.0;       // m/s
//     const double max_velocity = 10.0;      // m/s
//     std::shared_ptr<StateSpace> dubins_time_ss = std::make_shared<DubinsTimeStateSpace>(min_turning_radius, min_velocity, max_velocity);

//     // --- 3. Define Start and End States ---
//     Eigen::VectorXd start_state(4);
//     // Start at (0,0), facing East (0 rad), at time t=20s
//     start_state << 0, 0, 0.0, 20.0;

//     Eigen::VectorXd end_state(4);
//     // End at (15, 10), facing North (PI/2 rad), at time t=15s
//     // The planner plans backward, so the "to" state has an earlier time.
//     end_state << 15, 10, -M_PI/2 , 15.0;

//     std::cout << "Attempting to steer from: "; printVec(start_state); std::cout << "\n";
//     std::cout << "To: "; printVec(end_state); std::cout << "\n";

//     auto start1 = std::chrono::high_resolution_clock::now();
//     auto l2_norm = (end_state-start_state).norm();
//     std::cout<<l2_norm<<"\n";
//     auto end1 = std::chrono::high_resolution_clock::now();
//     auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(end1 - start1);
//     std::cout << "Time taken for the l2norm : " << duration1.count() 
//                 << " microseconds\n";
//     // --- 4. Call the steer function ---
//     auto start = std::chrono::high_resolution_clock::now();
//     Trajectory traj = dubins_time_ss->steer(start_state, end_state);
//     auto end = std::chrono::high_resolution_clock::now();
//     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
//     std::cout << "Time taken for the steer : " << duration.count() 
//                 << " microseconds\n";


//     // --- 5. Visualize the Trajectory ---
//     if (traj.is_valid) {
//         std::cout << "SUCCESS: Found a valid trajectory with cost (over 4D space): " << traj.cost << "\n";
        
//         // Convert the list of waypoints into a list of line segments for visualization
//         std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> path_segments;
//         if (traj.path_points.size() > 1) {
//             for (size_t i = 0; i < traj.path_points.size() - 1; ++i) {
//                 path_segments.emplace_back(traj.path_points[i], traj.path_points[i+1]);
//             }

//             std::cout << std::fixed << std::setprecision(6);
//             for(auto & seg : traj.path_points){
//                 std::cout<<seg.transpose()<<"\n";
//             }
            
//         }
        
//         std::cout << "Visualizing trajectory with " << path_segments.size() << " segments in RViz...\n";
//         // Use a loop to keep publishing the marker
//         rclcpp::WallRate loop_rate(1); // Publish once per second
//         while(rclcpp::ok()) {
//             visualizer->visualizeEdges(path_segments, "map", "0.0,1.0,0.0", "dubins_path"); // Green line
//             rclcpp::spin_some(node);
//             loop_rate.sleep();
//         }

//     } else {
//         std::cout << "FAILURE: Could not find a valid trajectory between the states.\n";
//         std::cout << "This could be due to velocity constraints (too fast or too slow required).\n";
//     }

//     rclcpp::shutdown();
//     return 0;
// }



// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// #include <iostream>
// #include <memory>
// #include <vector>
// #include <chrono>
// #include <Eigen/Dense>
// #include <cmath> // For M_PI
// #include "rclcpp/rclcpp.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"
// #include "motion_planning/state_space/dubins_time_statespace.hpp" // Your header

// /**
//  * @brief This program demonstrates and tests the DubinsTimeStateSpace by planning
//  * a path through a sequence of waypoints.
//  *
//  * It performs the following steps:
//  * 1. Initializes a ROS2 node for visualization.
//  * 2. Creates an instance of the DubinsTimeStateSpace with specific vehicle parameters.
//  * 3. Defines a sequence of 4D waypoints (x, y, theta, time).
//  * 4. Iteratively calls the `steer` function to compute the optimal kinodynamic 
//  * trajectory between each consecutive pair of waypoints.
//  * 5. Because the planner works backward in time, each `steer` call is from the
//  * chronologically later state to the earlier state (e.g., from B to A).
//  * 6. The path segments from all steer operations are combined into a single
//  * trajectory.
//  * 7. Visualizes the resulting multi-segment path in RViz.
//  */

// // Helper function to print vectors for debugging
// static void printVec(const Eigen::VectorXd& v) {
//     std::cout << "[ ";
//     for (int i = 0; i < v.size(); ++i) {
//         std::cout << v[i] << (i + 1 < v.size() ? ", " : " ");
//     }
//     std::cout << "]";
// }


// int main(int argc, char** argv) {
//     // --- 1. Initialization ---
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<rclcpp::Node>("dubin_time_test_node");
//     auto visualizer = std::make_shared<RVizVisualization>(node);
    
//     std::cout << "--- DubinsTimeStateSpace Multi-Point Test ---\n";

//     // --- 2. Create the Dubins State Space ---
//     const double min_turning_radius = 5.0; // meters
//     const double min_velocity = 2.0;       // m/s
//     const double max_velocity = 10.0;      // m/s
//     std::shared_ptr<StateSpace> dubins_time_ss = std::make_shared<DubinsTimeStateSpace>(min_turning_radius, min_velocity, max_velocity);

//     // --- 3. Define a sequence of waypoints (A, B, C, D, E) ---
//     // The time component must be strictly increasing for a valid forward path.
//     std::vector<Eigen::VectorXd> waypoints;
    
//     // A: Start at (0,0), facing East, at t=0
//     waypoints.push_back((Eigen::VectorXd(4) << 0.0, 0.0, 0.0, 0.0).finished());
//     // B: t=15s
//     waypoints.push_back((Eigen::VectorXd(4) << 30.0, 25.0, M_PI / 3.0, 15.0).finished());
//     // C: t=30s
//     waypoints.push_back((Eigen::VectorXd(4) << 45.0, -10.0, -M_PI / 2.0, 30.0).finished());
//     // D: t=45s
//     waypoints.push_back((Eigen::VectorXd(4) << 20.0, -35.0, M_PI, 45.0).finished());
//     // E: End at t=60s
//     waypoints.push_back((Eigen::VectorXd(4) << 0.0, -15.0, M_PI / 2.0, 60.0).finished());

//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> full_path_segments;
//     double total_cost = 0;
//     bool path_is_valid = true;

//     // --- 4. Call steer for each segment ---
//     auto planning_start_time = std::chrono::high_resolution_clock::now();

//     // Plan path segments in order: A->B, B->C, C->D, D->E
//     for (size_t i = 0; i < waypoints.size() - 1; ++i) {
//         const auto& segment_start_state = waypoints[i];
//         const auto& segment_end_state = waypoints[i+1];

//         std::cout << "\n--- Planning Segment " << i << " (" << (char)('A'+i+1) << " to " << (char)('A'+i) << ") ---\n";
//         std::cout << "From: "; printVec(segment_end_state); std::cout << "\n";
//         std::cout << "To:   "; printVec(segment_start_state); std::cout << "\n";

//         // The planner works backward in time, so the 'from' state for the steer
//         // function is the one with the later time (the end of our segment), and
//         // the 'to' state is the one with the earlier time (the start of our segment).
//         Trajectory traj = dubins_time_ss->steer(segment_end_state, segment_start_state);

//         if (traj.is_valid) {
//             std::cout << "SUCCESS: Found a valid trajectory for segment " << i << " with cost (Over 4D space): " << traj.cost << "\n";
//             total_cost += traj.cost;

//             // Add the generated path segments to our master list for visualization.
//             // The trajectory points are from the chronologically earlier state to the later one.
//             if (traj.path_points.size() > 1) {
//                 for (size_t j = 0; j < traj.path_points.size() - 1; ++j) {
//                     full_path_segments.emplace_back(traj.path_points[j], traj.path_points[j+1]);
//                 }
//                 std::cout << std::fixed << std::setprecision(6);
//                 for(auto & seg : traj.path_points){
//                     std::cout<<seg.transpose()<<"\n";
//                 }
//             }
//         } else {
//             std::cout << "FAILURE: Could not find a valid trajectory for segment " << i << ".\n";
//             path_is_valid = false;
//             break; // Stop if any segment fails
//         }
//     }
    


//     auto planning_end_time = std::chrono::high_resolution_clock::now();
//     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(planning_end_time - planning_start_time);
    
//     // --- 5. Visualize the final Trajectory ---
//     if (path_is_valid) {
//         std::cout << "\n----------------------------------------\n";
//         std::cout << "SUCCESS: Full path found!\n";
//         std::cout << "Total Planning Time: " << duration.count() << " ms\n";
//         std::cout << "Total Trajectory Time (Cost): " << total_cost << "s\n";
//         std::cout << "Visualizing trajectory with " << full_path_segments.size() << " segments in RViz...\n";
//         std::cout << "----------------------------------------\n";
        
//         // Use a loop to keep publishing the marker
//         rclcpp::WallRate loop_rate(1); // Publish once per second
//         while(rclcpp::ok()) {
//             // FIXED: Changed visualizeWaypoints to visualizeNodes and adapted arguments
//             visualizer->visualizeNodes(waypoints, "map", {1.0f, 0.0f, 0.0f}, "waypoints"); // Red points for waypoints
//             // FIXED: Removed the 5th argument (scale) to match the function definition
//             visualizer->visualizeEdges(full_path_segments, "map", "0.0,1.0,0.0", "dubins_path"); // Green line for path
//             rclcpp::spin_some(node);
//             loop_rate.sleep();
//         }

//     } else {
//         std::cout << "\n----------------------------------------\n";
//         std::cout << "FAILURE: Could not connect all waypoints.\n";
//         std::cout << "----------------------------------------\n";
//     }

//     rclcpp::shutdown();
//     return 0;
// }

/////////////////////////////////////////////////


#include <iostream>
#include <memory>
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/state_space/dubins_time_statespace.hpp"
#include "motion_planning/utils/ros2_manager_dubin.hpp"

// Helper function to print vectors for debugging
static void printVec(const Eigen::VectorXd& v) {
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "[ ";
    for (int i = 0; i < v.size(); ++i) {
        std::cout << std::setw(9) << v[i] << (i + 1 < v.size() ? ", " : " ");
    }
    std::cout << "]";
}


int main(int argc, char** argv) {
    // --- 1. Initialization ---
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<rclcpp::Node>("dubin_time_test_node");
    executor.add_node(node);
    auto visualizer = std::make_shared<RVizVisualization>(node);
     
    auto dubins_manager = std::make_shared<DubinsROS2Manager>(visualizer);
    executor.add_node(dubins_manager);

    std::cout << "--- DubinsTimeStateSpace Multi-Point Backward Simulation Test ---\n";

    // --- 2. Create the Dubins State Space ---
    const double min_turning_radius = 5.0;
    const double min_velocity = 2.0;
    const double max_velocity = 20.0;
    auto dubins_time_ss = std::make_shared<DubinsTimeStateSpace>(min_turning_radius, min_velocity, max_velocity);

    // --- 3. Define Waypoints (A, B, C, D, E) ---
    std::vector<Eigen::VectorXd> waypoints;
    waypoints.push_back((Eigen::VectorXd(4) << 0.0,    0.0,   0.0,           10.0).finished()); // A
    waypoints.push_back((Eigen::VectorXd(4) << 30.0,   25.0,  M_PI / 3.0,    25.0).finished()); // B
    waypoints.push_back((Eigen::VectorXd(4) << 45.0,  -10.0, -M_PI / 2.0,    40.0).finished()); // C
    waypoints.push_back((Eigen::VectorXd(4) << 20.0,  -35.0,  M_PI,          55.0).finished()); // D
    waypoints.push_back((Eigen::VectorXd(4) << 0.0,   -15.0,  M_PI / 2.0,    70.0).finished()); // E

    std::vector<Eigen::VectorXd> forward_path_for_viz;
    bool path_is_valid = true;

    // --- 4. Plan each segment backward and ANALYZE the output ---
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        const auto& segment_start_state = waypoints[i];   // e.g., A
        const auto& segment_end_state = waypoints[i+1]; // e.g., B
        
        std::cout << "\n--- Planning Segment " << i << " (from " << (char)('A'+i+1) << " to " << (char)('A'+i) << ") ---\n";
        
        // steer(from, to) plans from a state with higher time to one with lower time
        Trajectory traj = dubins_time_ss->steer(segment_end_state, segment_start_state);

        if (traj.is_valid) {
            std::cout << "SUCCESS: Found valid trajectory for segment " << i << ". Cost: " << traj.cost << "\n";
            
            /*********************************************************************************/
            /* --- DEBUG AND ANALYSIS BLOCK ---                                              */
            /*********************************************************************************/
            std::cout << "  --- Analyzing Raw Backward Trajectory (from steer) ---\n";
            if (traj.path_points.size() < 2) {
                std::cout << "  [WARNING] Trajectory has less than 2 points. Cannot analyze duration.\n";
            } else {
                bool segment_is_valid = true;
                for (size_t k = 0; k < traj.path_points.size() - 1; ++k) {
                    const auto& p1 = traj.path_points[k];
                    const auto& p2 = traj.path_points[k+1];
                    double time_dur = p1(3) - p2(3);

                    // Print every sub-segment for detailed analysis
                    std::cout << "    Sub-Seg " << k << ": ";
                    printVec(p1);
                    std::cout << " -> ";
                    printVec(p2);
                    std::cout << " | Time Duration: " << std::fixed << std::setprecision(6) << time_dur;

                    if (time_dur <= 1e-9) { // Check for non-positive duration
                        std::cout << " <-- [WARNING] NON-POSITIVE TIME DURATION!";
                        segment_is_valid = false;
                    }
                    std::cout << "\n";
                }
                if (!segment_is_valid) {
                     RCLCPP_ERROR(node->get_logger(), "CRITICAL BUG DETECTED in steer() output for segment %zu.", i);
                } else {
                     RCLCPP_INFO(node->get_logger(), "Steer output for segment %zu is temporally valid.", i);
                }
            }
            std::cout << "  -----------------------------------------------------\n";
            /*********************************************************************************/

            // For visualization, we need a forward path (A->B).
            std::reverse(traj.path_points.begin(), traj.path_points.end());
            forward_path_for_viz.insert(forward_path_for_viz.end(), traj.path_points.begin(), traj.path_points.end());
        
        } else {
            std::cout << "FAILURE: Could not find a valid trajectory for segment " << i << ".\n";
            path_is_valid = false;
            break;
        }
    }
     
    // --- 5. Visualize and Simulate ---
    if (path_is_valid) {
        std::cout << "\nSUCCESS: Full path found! Starting simulation...\n";
         
        // Visualize the static path from A to E
        std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> viz_segments;
        if(forward_path_for_viz.size() > 1) {
            for(size_t j=0; j < forward_path_for_viz.size() - 1; ++j) {
                viz_segments.emplace_back(forward_path_for_viz[j], forward_path_for_viz[j+1]);
            }
        }
        visualizer->visualizeNodes(waypoints, "map", {1.0f, 0.0f, 0.0f}, "waypoints");
        visualizer->visualizeEdges(viz_segments, "map", "0.0,1.0,0.0", "dubins_path");

        // Prepare the path for BACKWARD simulation (E -> A)
        std::vector<Eigen::VectorXd> backward_path_for_sim = forward_path_for_viz;
        std::reverse(backward_path_for_sim.begin(), backward_path_for_sim.end());

        dubins_manager->setPlannedDubinsPath(backward_path_for_sim);
        dubins_manager->setInitialState(waypoints.back()); // Start at E

        RCLCPP_INFO(node->get_logger(), "Spinning executor to run simulation. Press Ctrl-C to exit.");
        executor.spin();

    } else {
        std::cout << "\nFAILURE: Could not connect all waypoints.\n";
    }

    rclcpp::shutdown();
    return 0;
}