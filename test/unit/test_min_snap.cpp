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
#include "motion_planning/state_space/min_snap_statespace.hpp"
#include "motion_planning/utils/ros2_manager_min_snap.hpp"

// Helper to print vectors nicely
static void printVec(const Eigen::VectorXd& v) {
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "[ ";
    for (int i = 0; i < v.size(); ++i) {
        std::cout << std::setw(9) << v[i] << (i + 1 < v.size() ? ", " : " ");
    }
    std::cout << "]";
}

int main(int argc, char** argv) {
    // --- Initialization ---
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<rclcpp::Node>("min_snap_test_node");
    auto visualizer = std::make_shared<RVizVisualization>(node);
    auto manager = std::make_shared<MinSnapROS2Manager>(visualizer);

    executor.add_node(node);
    executor.add_node(manager);
     
    std::cout << "--- MinSnapStateSpace Multi-Point Backward Planning (from Root) Test ---\n";

    // --- Create the Min-Snap State Space ---
    const double v_max = 5.0; // m/s
    const double a_max = 2.0;  // m/s^2
    const double seed = 42;
    const double w_vel = 0.0;
    const double w_acc = 0.0;
    const double w_snap = 1.0;
    auto min_snap_ss = std::make_shared<MinSnapStateSpace>(5, v_max, a_max, w_vel, w_acc, w_snap, seed);

    // --- Define 5D Waypoints (x, y, z, yaw, time) ---
    std::vector<Eigen::VectorXd> waypoints;
    waypoints.push_back((Eigen::VectorXd(5) <<  0.0,   0.0,  5.0,  0.0,         10.0).finished()); // A (Root)
    waypoints.push_back((Eigen::VectorXd(5) << 20.0,  15.0,  27.0,  M_PI,  30.0).finished()); // B
    waypoints.push_back((Eigen::VectorXd(5) << 40.0,   0.0,  4.0, -M_PI / 4.0,  50.0).finished()); // C
    waypoints.push_back((Eigen::VectorXd(5) << 20.0, -15.0,  7.0, -M_PI * 0.75, 70.0).finished()); // D (Goal)
    // waypoints.push_back((Eigen::VectorXd(5) <<  0.0,   0.0,  5.0,  M_PI,        90.0).finished()); // E (Goal) - REMOVED

    // --- Data Storage ---
    std::vector<Trajectory> all_traj_segments; // Segments stored in planning order (A->B, B->C, etc.)
    bool path_is_valid = true;

    // --- Correct Forward Propagation Planning ---
    Eigen::VectorXd v_initial = Eigen::VectorXd::Zero(4); // Start at rest at the root (A)
    Eigen::VectorXd a_initial = Eigen::VectorXd::Zero(4); // Start at rest at the root (A)

    // Loop FORWARD chronologically to correctly propagate derivatives from the root
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        const auto& parent_state = waypoints[i];   // The root-side of the segment (e.g., A)
        const auto& child_state = waypoints[i+1];  // The leaf-side of the segment (e.g., B)
        
        std::cout << "\n--- Planning Segment from " << (char)('A'+i) << " to " << (char)('A'+i+1) << " ---\n";
        
        Trajectory traj;
        bool is_last_segment = (i == waypoints.size() - 2);

        // NOTE: The steer functions always plan backward in time (from child to parent),
        // but they take "initial" conditions corresponding to the parent state.
        std::cout << "  > Providing Initial Vel (at " << (char)('A'+i) << "): "; printVec(v_initial); std::cout << "\n";
        std::cout << "  > Providing Initial Acc (at " << (char)('A'+i) << "): "; printVec(a_initial); std::cout << "\n";
        
        if (is_last_segment) {
            // For the final segment (now C->D), we must enforce that it ends at rest.
            std::cout << "  Final segment: Constraining end-point to be at rest." << std::endl;
            Eigen::VectorXd v_final_at_goal = Eigen::VectorXd::Zero(4);
            Eigen::VectorXd a_final_at_goal = Eigen::VectorXd::Zero(4);
            // steer_with_initial_and_final
            traj = min_snap_ss->steer( child_state, parent_state, v_final_at_goal, a_final_at_goal, v_initial, a_initial);
        } else {
            // For all other segments, we provide the known derivatives at the parent
            // and leave the child endpoint free to be optimized.
            // steer_with_initial
            traj = min_snap_ss->steer(child_state, parent_state, v_initial, a_initial);
        }

        if (traj.is_valid) {
            std::cout << "SUCCESS: Found valid trajectory for segment." << std::endl;
            std::cout << "  > Calculated Final Vel (at " << (char)('A'+i+1) << "):  "; printVec(traj.final_velocity); std::cout << "\n";
            std::cout << "  > Calculated Final Acc (at " << (char)('A'+i+1) << "):  "; printVec(traj.final_acceleration); std::cout << "\n";
            
            all_traj_segments.push_back(traj);

            // CRITICAL: Propagate the derivatives for the next planning step
            v_initial = traj.final_velocity;
            a_initial = traj.final_acceleration;
        
        } else {
            std::cout << "FAILURE: Could not find a valid trajectory for this segment.\n";
            path_is_valid = false;
            break;
        }
    }
     
    // --- Visualization and Simulation ---
    if (path_is_valid) {
        std::cout << "\nSUCCESS: Full path planned! Visualizing and starting simulation...\n";
         
        // 1. Assemble path for VISUALIZATION (A -> D)
        std::vector<Eigen::VectorXd> full_path_for_viz;
        for(const auto& seg : all_traj_segments) {
            if (full_path_for_viz.empty()) {
                full_path_for_viz.insert(full_path_for_viz.end(), seg.path_points.begin(), seg.path_points.end());
            } else {
                full_path_for_viz.insert(full_path_for_viz.end(), seg.path_points.begin() + 1, seg.path_points.end());
            }
        }
        
        // 2. Visualize the static path
        std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> viz_segments;
        if(full_path_for_viz.size() > 1) {
            for(size_t j=0; j < full_path_for_viz.size() - 1; ++j) {
                viz_segments.emplace_back(full_path_for_viz[j], full_path_for_viz[j+1]);
            }
        }
        visualizer->visualizeNodes(waypoints, "map", {1.0f, 0.0f, 0.0f}, "waypoints");
        visualizer->visualizeEdges(viz_segments, "map", "0.0,1.0,0.0", "min_snap_path");

        // 3. Set up the manager for SIMULATION (needs segments D -> A)
        std::reverse(all_traj_segments.begin(), all_traj_segments.end());
        manager->setInitialState(waypoints.back());
        manager->setPlannedTrajectory(all_traj_segments);

        // 4. Spin the executor to run everything
        RCLCPP_INFO(node->get_logger(), "Spinning to run simulation. The arrow should follow the path from D to A. Press Ctrl-C to exit.");
        executor.spin();

    } else {
        std::cout << "\nFAILURE: Could not connect all waypoints.\n";
    }

    rclcpp::shutdown();
    return 0;
}