#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <cmath> // For M_PI
#include "rclcpp/rclcpp.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/state_space/dubins_time_statespace.hpp" // Your new header

/**
 * @brief This program demonstrates and tests the DubinsTimeStateSpace.
 * * It performs the following steps:
 * 1. Initializes a ROS2 node for visualization.
 * 2. Creates an instance of the DubinsTimeStateSpace with specific vehicle parameters.
 * 3. Defines a start and an end pose in 4D (x, y, theta, time).
 * 4. Calls the `steer` function to compute the optimal kinodynamic trajectory.
 * 5. Visualizes the resulting curved path in RViz as a series of short line segments.
 * 6. Also visualizes a set of random samples to show how theta and time are distributed.
 * * To Compile:
 * You will need to link against rclcpp, visualization_msgs, and your other libraries.
 * Example in CMakeLists.txt:
 * * find_package(rclcpp REQUIRED)
 * find_package(visualization_msgs REQUIRED)
 * ...
 * add_executable(test_dubin_time test_dubin_time.cpp ...)
 * ament_target_dependencies(test_dubin_time rclcpp visualization_msgs ...)
 * target_link_libraries(test_dubin_time ${your_motion_planning_library})
 */

// Helper function to print vectors for debugging
static void printVec(const Eigen::VectorXd& v) {
    std::cout << "[ ";
    for (int i = 0; i < v.size(); ++i) {
        std::cout << v[i] << (i + 1 < v.size() ? ", " : " ");
    }
    std::cout << "]";
}


int main(int argc, char** argv) {
    // --- 1. Initialization ---
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dubin_time_test_node");
    auto visualizer = std::make_shared<RVizVisualization>(node);
    
    std::cout << "--- DubinsTimeStateSpace Test ---\n";

    // --- 2. Create the Dubins State Space ---
    const double min_turning_radius = 5.0; // meters
    const double min_velocity = 2.0;       // m/s
    const double max_velocity = 10.0;      // m/s
    std::shared_ptr<StateSpace> dubins_time_ss = std::make_shared<DubinsTimeStateSpace>(min_turning_radius, min_velocity, max_velocity);

    // --- 3. Define Start and End States ---
    Eigen::VectorXd start_state(4);
    // Start at (0,0), facing East (0 rad), at time t=20s
    start_state << 0, 0, 0.0, 20.0;

    Eigen::VectorXd end_state(4);
    // End at (15, 10), facing North (PI/2 rad), at time t=15s
    // The planner plans backward, so the "to" state has an earlier time.
    end_state << 15, 10, -M_PI/4 , 15.0;

    std::cout << "Attempting to steer from: "; printVec(start_state); std::cout << "\n";
    std::cout << "To: "; printVec(end_state); std::cout << "\n";

    // --- 4. Call the steer function ---
    Trajectory traj = dubins_time_ss->steer(start_state, end_state);

    // --- 5. Visualize the Trajectory ---
    if (traj.is_valid) {
        std::cout << "SUCCESS: Found a valid trajectory with cost (time elapsed): " << traj.cost << "s\n";
        
        // Convert the list of waypoints into a list of line segments for visualization
        std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> path_segments;
        if (traj.path_points.size() > 1) {
            for (size_t i = 0; i < traj.path_points.size() - 1; ++i) {
                path_segments.emplace_back(traj.path_points[i], traj.path_points[i+1]);
            }
        }
        
        std::cout << "Visualizing trajectory with " << path_segments.size() << " segments in RViz...\n";
        // Use a loop to keep publishing the marker
        rclcpp::WallRate loop_rate(1); // Publish once per second
        while(rclcpp::ok()) {
            visualizer->visualizeEdges(path_segments, "map", "0.0,1.0,0.0", "dubins_path"); // Green line
            rclcpp::spin_some(node);
            loop_rate.sleep();
        }

    } else {
        std::cout << "FAILURE: Could not find a valid trajectory between the states.\n";
        std::cout << "This could be due to velocity constraints (too fast or too slow required).\n";
    }

    rclcpp::shutdown();
    return 0;
}
