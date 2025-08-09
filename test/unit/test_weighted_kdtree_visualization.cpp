// Copyright 2025 Soheil E.nia

#include <iostream>
#include <memory>
#include <vector>
#include <random>
#include <chrono>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include "motion_planning/utils/weighted_nano_flann.hpp"
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
    auto vis_node = std::make_shared<rclcpp::Node>("nanoflann_visualizer",
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
    weights << 1.0, 1.0, 5.0, 1.0;

    std::vector<int> wrap_dims = {2};
    std::vector<double> wrap_periods = {2.0 * M_PI};

    auto kdtree = std::make_unique<WeightedNanoFlann>(dimension, weights, wrap_dims, wrap_periods);
    RCLCPP_INFO(vis_node->get_logger(), "WeightedNanoFlann KD-Tree initialized.");

    std::vector<Eigen::VectorXd> sampled_points;
    sampled_points.reserve(num_sample_points);
    
    std::mt19937 generator(42);

    RCLCPP_INFO(vis_node->get_logger(), "Generating %d random samples with fixed seed...", num_sample_points);
    for (int i = 0; i < num_sample_points; ++i) {
        sampled_points.push_back(sample_random_point(generator, lower_bounds, upper_bounds));
    }
    kdtree->addPoints(sampled_points);
    kdtree->buildTree();
    RCLCPP_INFO(vis_node->get_logger(), "KD-Tree built with %zu points.", kdtree->size());

    Eigen::VectorXd query_point(dimension);
    query_point << 5.0, 5.0, M_PI / 2.0, 20.0;

    RCLCPP_INFO(vis_node->get_logger(), "Performing radius search with radius: %.2f", search_radius);
    std::vector<size_t> near_indices = kdtree->radiusSearch(query_point, search_radius);
    RCLCPP_INFO(vis_node->get_logger(), "Found %zu neighbors.", near_indices.size());

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