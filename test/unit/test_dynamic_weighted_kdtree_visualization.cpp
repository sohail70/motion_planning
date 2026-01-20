// Copyright 2025 Soheil E.nia
#include <iostream>
#include <memory>
#include <vector>
#include <random>
#include <chrono>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include "motion_planning/utils/dynamic_weighted_nano_flann.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"

Eigen::VectorXd sample_random_point(std::mt19937& gen,
                                    const Eigen::VectorXd& lower_bounds,
                                    const Eigen::VectorXd& upper_bounds)
{
    Eigen::VectorXd point(4);
    for (int i = 0; i < 4; ++i) {
        std::uniform_real_distribution<> distrib(lower_bounds(i), upper_bounds(i));
        point(i) = distrib(gen);
    }
    return point;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto vis_node = std::make_shared<rclcpp::Node>("dynamic_nanoflann_visualizer",
        rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
    auto visualization = std::make_shared<RVizVisualization>(vis_node);
    RCLCPP_INFO(vis_node->get_logger(), "Visualization node created.");

    const int dimension = 4;
    const int num_sample_points = 5000;
    const double search_radius = 15.0;

    Eigen::VectorXd lower_bounds(dimension), upper_bounds(dimension);
    lower_bounds << -50.0, -50.0, -M_PI, 0.0;
    upper_bounds << 50.0, 50.0, M_PI, 40.0;

    Eigen::VectorXd weights(dimension);
    weights << 1.0, 1.0, 1.0, 1.0;

    std::vector<int> wrap_dims = {2};
    std::vector<double> wrap_periods = {2.0 * M_PI};

    // Use DynamicWeightedNanoFlann instead of WeightedNanoFlann
    auto kdtree = std::make_unique<DynamicWeightedNanoFlann>(dimension, weights, wrap_dims, wrap_periods);
    RCLCPP_INFO(vis_node->get_logger(), "DynamicWeightedNanoFlann KD-Tree initialized.");

    std::vector<Eigen::VectorXd> sampled_points;
    sampled_points.reserve(num_sample_points);
    
    std::mt19937 generator(42);
    RCLCPP_INFO(vis_node->get_logger(), "Generating and adding %d random samples incrementally...", num_sample_points);
    
    auto start_time = std::chrono::high_resolution_clock::now();

    // Dynamic Addition Test: Add points one by one to verify O(log N) behavior
    for (int i = 0; i < num_sample_points; ++i) {
        Eigen::VectorXd p = sample_random_point(generator, lower_bounds, upper_bounds);
        sampled_points.push_back(p);
        kdtree->addPoint(p); // Dynamic add
        
        // Optional: Print progress every 1000 points
        if ((i + 1) % 1000 == 0) {
            RCLCPP_INFO(vis_node->get_logger(), "Added %d points...", i + 1);
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    RCLCPP_INFO(vis_node->get_logger(), "Finished adding points. Time: %.4f seconds. Total points: %zu", elapsed.count(), kdtree->size());

    // NOTE: Do NOT call buildTree() for the dynamic version. It is already built.
    
    Eigen::VectorXd query_point(dimension);
    query_point << 5.0, 5.0, M_PI / 2.0, 20.0;

    RCLCPP_INFO(vis_node->get_logger(), "Performing radius search with radius: %.2f", search_radius);
    std::vector<size_t> near_indices = kdtree->radiusSearch(query_point, search_radius);
    RCLCPP_INFO(vis_node->get_logger(), "Found %zu neighbors.", near_indices.size());

    // Prepare visualization data
    std::vector<Eigen::VectorXd> all_points_vis;
    all_points_vis.reserve(sampled_points.size());
    for (const auto& p : sampled_points) {
        all_points_vis.push_back(p.head<2>());
    }

    std::vector<Eigen::VectorXd> near_points_vis;
    near_points_vis.reserve(near_indices.size());
    for (const auto& idx : near_indices) {
        near_points_vis.push_back(sampled_points[idx].head<2>());
    }

    std::vector<Eigen::VectorXd> query_point_vis;
    query_point_vis.push_back(query_point.head<2>());

    auto vis_timer = vis_node->create_wall_timer(
        std::chrono::seconds(1),
        [&]() {
            RCLCPP_INFO(vis_node->get_logger(), "Publishing visualization markers...");
            // Visualize the neighbors found in GREEN
            visualization->visualizeNodes(near_points_vis, "map", {0.0f, 1.0f, 0.0f}, "near_neighbors");
            // Visualize all sampled points in RED
            visualization->visualizeNodes(all_points_vis, "map", {1.0f, 0.0f, 0.0f}, "all_samples");
            // Visualize the query point in BLUE
            visualization->visualizeNodes(query_point_vis, "map", {0.0f, 0.0f, 1.0f}, "query_point");
            // Visualize the search radius as a cyan circle.
            visualization->visualizeCircle(query_point.head<2>(), search_radius, "map", {0.0f, 1.0f, 1.0f}, "search_radius_circle");
        });

    rclcpp::spin(vis_node);
    rclcpp::shutdown();
    return 0;
}


// // Copyright 2025 Soheil E.nia --> 2D case for simplicity that its working! above has 4D and hard to be convinced even though correct!
// #include <iostream>
// #include <memory>
// #include <vector>
// #include <random>
// #include <cmath>
// #include <Eigen/Dense>
// #include <rclcpp/rclcpp.hpp>
// #include "motion_planning/utils/dynamic_weighted_nano_flann.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"

// Eigen::VectorXd sample_random_point_2d(std::mt19937& gen, double min_val, double max_val)
// {
//     Eigen::VectorXd point(2);
//     std::uniform_real_distribution<> distrib(min_val, max_val);
//     point(0) = distrib(gen);
//     point(1) = distrib(gen);
//     return point;
// }

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto vis_node = std::make_shared<rclcpp::Node>("dynamic_nanoflann_simple_test",
//         rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
//     auto visualization = std::make_shared<RVizVisualization>(vis_node);
//     RCLCPP_INFO(vis_node->get_logger(), "Visualization node created.");

//     // --- CONFIGURATION ---
//     const int dimension = 2; // Only X and Y
//     const int num_sample_points = 100; // Small number for debugging
//     const double search_radius = 15.0; 
    
//     // Bounds for X and Y
//     const double min_bound = -20.0;
//     const double max_bound = 20.0;

//     // Weights (1.0 for both means standard Euclidean distance)
//     Eigen::VectorXd weights(dimension);
//     weights << 1.0, 1.0;

//     // No wrapping for this simple test
//     std::vector<int> wrap_dims = {}; 
//     std::vector<double> wrap_periods = {};

//     // --- INITIALIZATION ---
//     auto kdtree = std::make_unique<DynamicWeightedNanoFlann>(dimension, weights, wrap_dims, wrap_periods);
//     RCLCPP_INFO(vis_node->get_logger(), "DynamicWeightedNanoFlann initialized (2D, No Wrapping).");

//     std::vector<Eigen::VectorXd> sampled_points;
//     sampled_points.reserve(num_sample_points);
    
//     std::mt19937 generator(42); // Fixed seed for reproducibility

//     RCLCPP_INFO(vis_node->get_logger(), "Generating %d random points...", num_sample_points);
    
//     for (int i = 0; i < num_sample_points; ++i) {
//         Eigen::VectorXd p = sample_random_point_2d(generator, min_bound, max_bound);
//         sampled_points.push_back(p);
//         kdtree->addPoint(p);
//     }
//     RCLCPP_INFO(vis_node->get_logger(), "Points added to tree. Total size: %zu", kdtree->size());

//     // --- QUERY ---
//     // Fixed query point near the center
//     Eigen::VectorXd query_point(dimension);
//     query_point << 0.0, 0.0; 

//     RCLCPP_INFO(vis_node->get_logger(), "--------------------------------------------------");
//     RCLCPP_INFO(vis_node->get_logger(), "Query Point: [%.2f, %.2f]", query_point(0), query_point(1));
//     RCLCPP_INFO(vis_node->get_logger(), "Search Radius: %.2f", search_radius);
//     RCLCPP_INFO(vis_node->get_logger(), "--------------------------------------------------");

//     std::vector<size_t> near_indices = kdtree->radiusSearch(query_point, search_radius);
    
//     RCLCPP_INFO(vis_node->get_logger(), "Found %zu neighbors.", near_indices.size());
    
//     // --- DEBUG LOGS ---
//     // Print every found neighbor to verify distance manually
//     for (size_t idx : near_indices) {
//         Eigen::VectorXd p = sampled_points[idx];
//         double dist = (p - query_point).norm();
//         RCLCPP_INFO(vis_node->get_logger(), "Neighbor ID: %zu | Pos: [%.2f, %.2f] | Dist: %.4f", idx, p(0), p(1), dist);
//     }
//     RCLCPP_INFO(vis_node->get_logger(), "--------------------------------------------------");

//     // --- VISUALIZATION ---
//     std::vector<Eigen::VectorXd> all_points_vis;
//     for (const auto& p : sampled_points) {
//         all_points_vis.push_back(p); // Already 2D
//     }

//     std::vector<Eigen::VectorXd> near_points_vis;
//     for (const auto& idx : near_indices) {
//         near_points_vis.push_back(sampled_points[idx]);
//     }

//     std::vector<Eigen::VectorXd> query_point_vis;
//     query_point_vis.push_back(query_point);

//     auto vis_timer = vis_node->create_wall_timer(
//         std::chrono::milliseconds(500), // Update faster (0.5s)
//         [&]() {
//             // Visualize the neighbors found in GREEN
//             visualization->visualizeNodes(near_points_vis, "map", {0.0f, 1.0f, 0.0f}, "near_neighbors");
//             // Visualize all sampled points in RED
//             visualization->visualizeNodes(all_points_vis, "map", {1.0f, 0.0f, 0.0f}, "all_samples");
//             // Visualize the query point in BLUE
//             visualization->visualizeNodes(query_point_vis, "map", {0.0f, 0.0f, 1.0f}, "query_point");
//             // Visualize the search radius as a cyan circle.
//             visualization->visualizeCircle(query_point, search_radius, "map", {0.0f, 1.0f, 1.0f}, "search_radius_circle");
//         });

//     rclcpp::spin(vis_node);
//     rclcpp::shutdown();
//     return 0;
// }