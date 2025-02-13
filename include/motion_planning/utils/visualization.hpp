// Copyright 2025 Soheil E.nia

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>
#include <vector>

class FMTXVisualizer {
public:
    FMTXVisualizer(rclcpp::Node::SharedPtr node, const std::string& marker_topic = "fmtx_markers")
        : node_(node) {
        // Create a publisher for the visualization markers
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10);
    }

    // Visualize nodes as points
    void visualizeNodes(const std::vector<Eigen::VectorXd>& nodes, const std::string& frame_id = "map") {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = node_->now();
        marker.ns = "fmtx_nodes";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1; // Point width
        marker.scale.y = 0.1; // Point height
        marker.color.r = 0.0; // Red
        marker.color.g = 1.0; // Green
        marker.color.b = 0.0; // Blue
        marker.color.a = 1.0; // Fully opaque

        // Add nodes to the marker
        for (const auto& node : nodes) {
            geometry_msgs::msg::Point point;
            point.x = node.x();
            point.y = node.y();
            point.z = 0.0; // Assuming 2D
            marker.points.push_back(point);
        }

        // Publish the marker
        marker_pub_->publish(marker);
    }

    // Visualize edges as lines
    void visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id = "map") {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = node_->now();
        marker.ns = "fmtx_edges";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05; // Line width
        marker.color.r = 1.0;  // Red
        marker.color.g = 0.0;  // Green
        marker.color.b = 0.0;  // Blue
        marker.color.a = 1.0;  // Fully opaque

        // Add edges to the marker
        for (const auto& edge : edges) {
            geometry_msgs::msg::Point start, end;
            start.x = edge.first.x();
            start.y = edge.first.y();
            start.z = 0.0; // Assuming 2D
            end.x = edge.second.x();
            end.y = edge.second.y();
            end.z = 0.0; // Assuming 2D
            marker.points.push_back(start);
            marker.points.push_back(end);
        }

        // Publish the marker
        marker_pub_->publish(marker);
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

#endif // FMTX_VISUALIZER_HPP
