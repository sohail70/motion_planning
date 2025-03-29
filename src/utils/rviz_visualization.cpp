// Copyright 2025 Soheil E.nia

#include "motion_planning/utils/rviz_visualization.hpp"


// std::string getRandomColor() {
//     // Seed the random number generator
//     std::srand(std::time(nullptr));

//     // Generate random values for RGB between 0.0 and 1.0
//     float r = static_cast<float>(std::rand()) / RAND_MAX;
//     float g = static_cast<float>(std::rand()) / RAND_MAX;
//     float b = static_cast<float>(std::rand()) / RAND_MAX;

//     // Convert to string
//     std::ostringstream ss;
//     ss << std::fixed << std::setprecision(2) << r << "," << g << "," << b;
//     return ss.str();
// }


RVizVisualization::RVizVisualization(rclcpp::Node::SharedPtr node, const std::string& marker_topic)
    : node_(node),marker_id_counter_(0)  {
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10);
    marker_pub_2_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("marker2", 10);

}

void RVizVisualization::visualizeNodes(const std::vector<Eigen::VectorXd>& nodes, const std::string& frame_id) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = node_->now();
        marker.ns = "default_nodes";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.2; // Point width
        marker.scale.y = 0.2; // Point height
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

void RVizVisualization::visualizeNodes(const std::vector<Eigen::VectorXd>& nodes, const std::string& frame_id, const std::vector<float>& color, const std::string& ns) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    // marker.ns = "colored_nodes";
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.3; // Point width
    marker.scale.y = 0.3; // Point height
    // marker.lifetime = rclcpp::Duration(1, 0);  // 1 second visibility
    // marker.lifetime = rclcpp::Duration::from_seconds(0.2);  // Keep for 0.2 seconds




    // Set the color components
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = 1.0;


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


void RVizVisualization::visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id) {
       visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = node_->now();
        marker.ns = "default_edge";
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
void RVizVisualization::visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id, const std::string& color_str) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "colored_edge";
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05; // Line width

    // Parse the color string
    std::stringstream ss(color_str);
    std::string token;
    std::vector<float> color_components;
    while (std::getline(ss, token, ',')) {
        color_components.push_back(std::stof(token));
    }

    // Set the color components
    if (color_components.size() == 3) {
        marker.color.r = color_components[0];  // Red
        marker.color.g = color_components[1];  // Green
        marker.color.b = color_components[2];  // Blue
        marker.color.a = 1.0;  // Fully opaque
    } else {
        // Default to red if the color string is invalid
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
    }

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

void RVizVisualization::visualizeCylinder(
    const std::vector<Eigen::VectorXd>& obstacles, 
    const std::vector<double>& radii, 
    const std::string& frame_id,
    const std::vector<float>& color ,
    const std::string& ns) 
{
    if (obstacles.size() != radii.size()) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "Mismatch between number of obstacles (%zu) and radii (%zu).", 
                     obstacles.size(), radii.size());
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    // Iterate through obstacles and radii simultaneously
    for (size_t i = 0; i < obstacles.size(); ++i) {
        const auto& obstacle = obstacles[i];
        double radius = radii[i];  // Get the radius for this obstacle

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = node_->now();
        marker.ns = ns;
        marker.id = id++;  // Unique ID per marker
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position of the marker
        marker.pose.position.x = obstacle.x();
        marker.pose.position.y = obstacle.y();
        marker.pose.position.z = 0.05;  // Slightly above the ground

        // Set the scale of the marker (diameter = 2 * radius)
        marker.scale.x = 2 * radius;  // Diameter in x
        marker.scale.y = 2 * radius;  // Diameter in y
        marker.scale.z = 0.1;         // Height in z

        // Set the color of the marker
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = 0.5; // semi-transparent

        // Add the marker to the array
        marker_array.markers.push_back(marker);
    }

    // Publish all markers at once
    marker_pub_2_->publish(marker_array);
}