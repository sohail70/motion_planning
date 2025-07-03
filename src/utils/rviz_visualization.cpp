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
    marker_pub_3_ = node_->create_publisher<visualization_msgs::msg::Marker>("marker3", 10);

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

    for (const auto& node : nodes) {
        geometry_msgs::msg::Point point;
        point.x = node.x();
        point.y = node.y();
        point.z = (node.size() > 2) ? node.z() : 0.0; // <--- MODIFIED: Use Z if available, else 0
        marker.points.push_back(point);
    }
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
        // point.z = 0.0; // Assuming 2D
        point.z = (node.size() > 2) ? node.z() : 0.0; // <--- MODIFIED: Use Z if available, else 0
        marker.points.push_back(point);
    }

    // Publish the marker
    marker_pub_->publish(marker);
}

void RVizVisualization::visualizeSingleEdge(const Eigen::VectorXd& start_point, const Eigen::VectorXd& end_point, int edge_id, const std::string& frame_id) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "tree_edges_incremental"; // A dedicated namespace
    marker.id = edge_id; // Use the provided ID, which will be the child node's index
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    geometry_msgs::msg::Point start, end;
    start.x = start_point.x();
    start.y = start_point.y();
    start.z = (start_point.size() > 2) ? start_point.z() : 0.0;

    end.x = end_point.x();
    end.y = end_point.y();
    end.z = (end_point.size() > 2) ? end_point.z() : 0.0;

    marker.points.push_back(start);
    marker.points.push_back(end);

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
            // start.z = 0.0; // Assuming 2D
            start.z = (edge.first.size() > 2) ? edge.first.z() : 0.0; // <--- MODIFIED: Use Z if available, else 0

            end.x = edge.second.x();
            end.y = edge.second.y();
            // end.z = 0.0; // Assuming 2D
            end.z = (edge.second.size() > 2) ? edge.second.z() : 0.0; // <--- MODIFIED: Use Z if available, else 0

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
    marker.scale.x = 0.2; // Line width

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
        // start.z = 0.0; // Assuming 2D
        start.z = (edge.first.size() > 2) ? edge.first.z() : 0.0; // <--- MODIFIED

        end.x = edge.second.x();
        end.y = edge.second.y();
        // end.z = 0.0; // Assuming 2D
        end.z = (edge.second.size() > 2) ? edge.second.z() : 0.0; // <--- MODIFIED

        marker.points.push_back(start);
        marker.points.push_back(end);
    }

    // Publish the marker
    marker_pub_->publish(marker);
}


void RVizVisualization::visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id, const std::string& color_str, const std::string& ns) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = ns;
    marker.id = 10;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.2; // Line width

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
        // start.z = 0.0; // Assuming 2D
        start.z = (edge.first.size() > 2) ? edge.first.z() : 0.0; // <--- MODIFIED

        end.x = edge.second.x();
        end.y = edge.second.y();
        // end.z = 0.0; // Assuming 2D
        end.z = (edge.second.size() > 2) ? edge.second.z() : 0.0; // <--- MODIFIED

        marker.points.push_back(start);
        marker.points.push_back(end);
    }

    // Publish the marker
    marker_pub_3_->publish(marker);
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

    // ---> FIX: ADD THIS BLOCK TO CLEAR OLD MARKERS <---
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = ns;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    // ---> END OF FIX <---


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



void RVizVisualization::visualizeRobotArrow(
    const Eigen::VectorXd& robot_position,  // Position of the robot
    const Eigen::VectorXd& robot_orientation,  // Quaternion orientation
    const std::string& frame_id,
    const std::vector<float>& color,  // RGB color for the arrow
    const std::string& ns)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = ns;
    marker.id = 0;  // Only one marker for the robot
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;


    // Set position for the arrow (robot position)
    marker.pose.position.x = robot_position[0];
    marker.pose.position.y = robot_position[1];
    marker.pose.position.z = 0.0;  // Keep it 2D for now (z = 0)

    // Set orientation from the quaternion (robot_orientation)
    marker.pose.orientation.x = robot_orientation[0];
    marker.pose.orientation.y = robot_orientation[1];
    marker.pose.orientation.z = robot_orientation[2];
    marker.pose.orientation.w = robot_orientation[3];

    // Set the scale of the arrow
    marker.scale.x = 3.0;  // Shaft width
    marker.scale.y = 1.0;  // Shaft width
    marker.scale.z = 0.5;  // Arrowhead size

    // Set the color of the arrow
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;  // Fully opaque

    // Publish the marker (robot as an arrow)
    marker_pub_->publish(marker);
}
void RVizVisualization::visualizeCube(
    const std::vector<std::tuple<Eigen::Vector2d, double, double, double>>& box_obstacles,
    const std::string& frame_id,
    const std::vector<float>& color,
    const std::string& ns) 
{
    visualization_msgs::msg::MarkerArray marker_array;

    // ---> FIX: ADD THIS BLOCK TO CLEAR OLD MARKERS <---
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = ns;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    // ---> END OF FIX <---


    int id = 0;

    for (const auto& box : box_obstacles) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = node_->now();
        marker.ns = ns;
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Extract box parameters from the tuple
        const Eigen::Vector2d& position = std::get<0>(box);
        double width = std::get<1>(box);
        double height = std::get<2>(box);
        double rotation = std::get<3>(box);

        // Set position (z slightly above ground)
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = 0.05;

        // Convert yaw rotation to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, rotation);  // Roll, Pitch, Yaw (radians)
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        // Set scale (width, height, small depth)
        marker.scale.x = width;
        marker.scale.y = height;
        marker.scale.z = 0.1;  // Thickness in Z-axis

        // Set color and transparency
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = 0.5;  // Semi-transparent

        marker_array.markers.push_back(marker);
    }

    // Publish all box markers
    marker_pub_2_->publish(marker_array);
}



// void RVizVisualization::visualizeTrajectories(const std::vector<std::vector<Eigen::Vector2d>>& trajectories, 
//                           const std::string& frame_id,
//                           const std::vector<float>& color,
//                           const std::string& ns) {
//     // visualization_msgs::msg::MarkerArray marker_array;
    
//     // // Clear previous markers
//     // visualization_msgs::msg::Marker clear_marker;
//     // clear_marker.header.frame_id = frame_id;
//     // clear_marker.header.stamp = node_->now();
//     // clear_marker.ns = ns;
//     // clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
//     // marker_array.markers.push_back(clear_marker);

//     // // Create new trajectory markers
//     // for (size_t i = 0; i < trajectories.size(); ++i) {
//     //     const auto& trajectory = trajectories[i];
//     //     if (trajectory.empty()) continue;

//     //     visualization_msgs::msg::Marker line_strip;
//     //     line_strip.header.frame_id = frame_id;
//     //     line_strip.header.stamp = node_->now();
//     //     line_strip.ns = ns;
//     //     line_strip.id = i;
//     //     line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
//     //     line_strip.action = visualization_msgs::msg::Marker::ADD;
//     //     line_strip.scale.x = 0.05; // Line width
//     //     line_strip.color.r = color[0];
//     //     line_strip.color.g = color[1];
//     //     line_strip.color.b = color[2];
//     //     line_strip.color.a = 1.0;

//     //     for (const auto& point : trajectory) {
//     //         geometry_msgs::msg::Point p;
//     //         p.x = point.x();
//     //         p.y = point.y();
//     //         p.z = 0.0;
//     //         line_strip.points.push_back(p);
//     //     }

//     //     marker_array.markers.push_back(line_strip);
//     // }

//     // marker_pub_2_->publish(marker_array);
//     /////////////////////////////////////
//     visualization_msgs::msg::MarkerArray marker_array;
    
//     // Clear previous markers
//     visualization_msgs::msg::Marker clear_marker;
//     clear_marker.header.frame_id = frame_id;
//     clear_marker.header.stamp = node_->now();
//     clear_marker.ns = ns;
//     clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
//     marker_array.markers.push_back(clear_marker);

//     // Create point markers for each trajectory
//     for (size_t i = 0; i < trajectories.size(); ++i) {
//         const auto& trajectory = trajectories[i];
//         if (trajectory.empty()) continue;

//         visualization_msgs::msg::Marker dots;
//         dots.header.frame_id = frame_id;
//         dots.header.stamp = node_->now();
//         dots.ns = ns;
//         dots.id = i;
//         dots.type = visualization_msgs::msg::Marker::SPHERE_LIST;
//         dots.action = visualization_msgs::msg::Marker::ADD;

//         // Dot size
//         dots.scale.x = 0.4;
//         dots.scale.y = 0.4;
//         dots.scale.z = 0.4;

//         // Dot color
//         dots.color.r = color[0];
//         dots.color.g = color[1];
//         dots.color.b = color[2];
//         dots.color.a = 1.0;

//         for (const auto& point : trajectory) {
//             geometry_msgs::msg::Point p;
//             p.x = point.x();
//             p.y = point.y();
//             p.z = 0.0;
//                         p.z = (point.size() > 2) ? point[2] : 0.0; // <--- MODIFIED (This one uses Vector2d as input, so point.size() will always be 2. It should be changed to VectorXd or remove the z access here)

//             // p.z = (point.size() > 2) ? point.z() : 0.0; // <--- MODIFIED
//             dots.points.push_back(p);
//         }

//         marker_array.markers.push_back(dots);
//     }

//     marker_pub_2_->publish(marker_array);

// }

void RVizVisualization::visualizeTrajectories(const std::vector<std::vector<Eigen::Vector2d>>& trajectories, 
                          const std::string& frame_id,
                          const std::vector<float>& color,
                          const std::string& ns) {
    // --- Clearing Logic ---
    visualization_msgs::msg::MarkerArray markers_to_delete;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = ns; // Use the provided namespace
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers_to_delete.markers.push_back(clear_marker);
    marker_pub_2_->publish(markers_to_delete);


    // --- Adding Logic ---
    visualization_msgs::msg::MarkerArray markers_to_add;
    for (size_t i = 0; i < trajectories.size(); ++i) {
        const auto& trajectory = trajectories[i];
        if (trajectory.empty()) continue;

        visualization_msgs::msg::Marker dots;
        dots.header.frame_id = frame_id;
        dots.header.stamp = node_->now();
        dots.ns = ns;
        dots.id = i; // ID is now unique for each trajectory
        dots.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        dots.action = visualization_msgs::msg::Marker::ADD;

        dots.scale.x = 0.4;
        dots.scale.y = 0.4;
        dots.scale.z = 0.4;

        dots.color.r = color[0];
        dots.color.g = color[1];
        dots.color.b = color[2];
        dots.color.a = 1.0;

        for (const auto& point : trajectory) {
            geometry_msgs::msg::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = 0.0;
            dots.points.push_back(p);
        }
        markers_to_add.markers.push_back(dots);
    }

    if (!markers_to_add.markers.empty()) {
        marker_pub_2_->publish(markers_to_add);
    }
}

void RVizVisualization::visualizeTrajectories(const std::vector<std::vector<Eigen::VectorXd>>& trajectories, 
                          const std::string& frame_id,
                          const std::vector<float>& color,
                          const std::string& ns) {
    // visualization_msgs::msg::MarkerArray marker_array;
    
    // // Clear previous markers
    // visualization_msgs::msg::Marker clear_marker;
    // clear_marker.header.frame_id = frame_id;
    // clear_marker.header.stamp = node_->now();
    // clear_marker.ns = ns;
    // clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    // marker_array.markers.push_back(clear_marker);

    // // Create new trajectory markers
    // for (size_t i = 0; i < trajectories.size(); ++i) {
    //     const auto& trajectory = trajectories[i];
    //     if (trajectory.empty()) continue;

    //     visualization_msgs::msg::Marker line_strip;
    //     line_strip.header.frame_id = frame_id;
    //     line_strip.header.stamp = node_->now();
    //     line_strip.ns = ns;
    //     line_strip.id = i;
    //     line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    //     line_strip.action = visualization_msgs::msg::Marker::ADD;
    //     line_strip.scale.x = 0.05; // Line width
    //     line_strip.color.r = color[0];
    //     line_strip.color.g = color[1];
    //     line_strip.color.b = color[2];
    //     line_strip.color.a = 1.0;

    //     for (const auto& point : trajectory) {
    //         geometry_msgs::msg::Point p;
    //         p.x = point.x();
    //         p.y = point.y();
    //         p.z = 0.0;
    //         line_strip.points.push_back(p);
    //     }

    //     marker_array.markers.push_back(line_strip);
    // }

    // marker_pub_2_->publish(marker_array);
    /////////////////////////////////////
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = ns;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    // Create point markers for each trajectory
    for (size_t i = 0; i < trajectories.size(); ++i) {
        const auto& trajectory = trajectories[i];
        if (trajectory.empty()) continue;

        visualization_msgs::msg::Marker dots;
        dots.header.frame_id = frame_id;
        dots.header.stamp = node_->now();
        dots.ns = ns;
        dots.id = i;
        dots.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        dots.action = visualization_msgs::msg::Marker::ADD;

        // Dot size
        dots.scale.x = 0.4;
        dots.scale.y = 0.4;
        dots.scale.z = 0.4;

        // Dot color
        dots.color.r = color[0];
        dots.color.g = color[1];
        dots.color.b = color[2];
        dots.color.a = 1.0;

        for (const auto& point : trajectory) {
            geometry_msgs::msg::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = 0.0;
            p.z = (point.size() > 2) ? point[2] : 0.0; // <--- MODIFIED (This one uses Vector2d as input, so point.size() will always be 2. It should be changed to VectorXd or remove the z access here)

            // p.z = (point.size() > 2) ? point.z() : 0.0; // <--- MODIFIED
            dots.points.push_back(p);
        }

        marker_array.markers.push_back(dots);
    }

    marker_pub_2_->publish(marker_array);

}


void RVizVisualization::visualizeFutureGhosts(
    const ObstacleVector& obstacles,
    double prediction_horizon,
    const std::string& frame_id)
{
    // ==========================================================
    // --- Create a single MarkerArray for all our actions ---
    // ==========================================================
    visualization_msgs::msg::MarkerArray all_markers;

    // // --- Add clearing markers FIRST ---
    // // This tells RViz to delete all markers in these specific namespaces
    // // before processing the 'ADD' markers in this same message.
    // visualization_msgs::msg::Marker clear_ghosts_marker;
    // clear_ghosts_marker.header.frame_id = frame_id;
    // clear_ghosts_marker.header.stamp = node_->now();
    // clear_ghosts_marker.ns = "ghost_obstacles";
    // clear_ghosts_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    // all_markers.markers.push_back(clear_ghosts_marker);

    // visualization_msgs::msg::Marker clear_vectors_marker;
    // clear_vectors_marker.header = clear_ghosts_marker.header; // Copy header
    // clear_vectors_marker.ns = "velocity_vectors";
    // clear_vectors_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    // all_markers.markers.push_back(clear_vectors_marker);


    // ==========================================================
    // --- Now, add the new markers to the SAME array ---
    // ==========================================================
    int id = 0;
    for (const auto& obstacle : obstacles) {
        if (!obstacle.is_dynamic || obstacle.velocity.norm() < 0.01) {
            continue;
        }

        Eigen::Vector2d future_position = obstacle.position + obstacle.velocity * prediction_horizon;

        // 1. Create the velocity vector (a line) marker
        visualization_msgs::msg::Marker vector_line;
        vector_line.header.frame_id = frame_id;
        vector_line.header.stamp = node_->now();
        vector_line.ns = "velocity_vectors";
        vector_line.id = id; // Unique ID for this marker
        vector_line.type = visualization_msgs::msg::Marker::ARROW; // ARROW is better for velocity
        vector_line.action = visualization_msgs::msg::Marker::ADD;
        
        // ARROW uses start and end points
        geometry_msgs::msg::Point start_p, end_p;
        start_p.x = obstacle.position.x();
        start_p.y = obstacle.position.y();
        start_p.z = 0.1; // Lift it slightly off the ground
        end_p.x = future_position.x();
        end_p.y = future_position.y();
        end_p.z = 0.1;
        vector_line.points.push_back(start_p);
        vector_line.points.push_back(end_p);

        vector_line.scale.x = 0.1;  // Shaft diameter
        vector_line.scale.y = 0.2;  // Head diameter
        vector_line.scale.z = 0.2;  // Head length
        vector_line.color.r = 1.0f; // Orange
        vector_line.color.g = 0.5f;
        vector_line.color.b = 0.0f;
        vector_line.color.a = 0.8f; 
        all_markers.markers.push_back(vector_line);
        
        // 2. Create the "Ghost" obstacle marker
        visualization_msgs::msg::Marker ghost_marker;
        ghost_marker.header = vector_line.header;
        ghost_marker.ns = "ghost_obstacles";
        ghost_marker.id = id; // Use the same ID for the corresponding ghost
        ghost_marker.action = visualization_msgs::msg::Marker::ADD;
        ghost_marker.pose.position.x = future_position.x();
        ghost_marker.pose.position.y = future_position.y();
        ghost_marker.pose.position.z = 0.05;
        ghost_marker.color.r = 0.0f;
        ghost_marker.color.g = 0.8f;
        ghost_marker.color.b = 0.8f;
        ghost_marker.color.a = 0.35f; // Very transparent

        if (obstacle.type == Obstacle::CIRCLE) {
            ghost_marker.type = visualization_msgs::msg::Marker::CYLINDER;
            ghost_marker.scale.x = 2 * (obstacle.dimensions.radius + obstacle.inflation);
            ghost_marker.scale.y = 2 * (obstacle.dimensions.radius + obstacle.inflation);
            ghost_marker.scale.z = 0.1;
        } else { // BOX
            ghost_marker.type = visualization_msgs::msg::Marker::CUBE;
            Eigen::Quaterniond q(Eigen::AngleAxisd(obstacle.dimensions.rotation, Eigen::Vector3d::UnitZ()));
            ghost_marker.pose.orientation.x = q.x();
            ghost_marker.pose.orientation.y = q.y();
            ghost_marker.pose.orientation.z = q.z();
            ghost_marker.pose.orientation.w = q.w();
            ghost_marker.scale.x = obstacle.dimensions.width + 2 * obstacle.inflation;
            ghost_marker.scale.y = obstacle.dimensions.height + 2 * obstacle.inflation;
            ghost_marker.scale.z = 0.1;
        }
        all_markers.markers.push_back(ghost_marker);

        id++; // IMPORTANT: Increment the ID for the next obstacle
    }

    // Publish the single array containing all actions, only if it has add markers
    if (all_markers.markers.size() > 2) { // (It will have 2 markers if only DELETEALL actions are present)
        marker_pub_2_->publish(all_markers);
    }
}




void RVizVisualization::visualizeVelocityVectors(
    const std::vector<Eigen::Vector2d>& positions,
    const std::vector<Eigen::Vector2d>& velocities,
    const std::string& frame_id,
    const std::vector<float>& color,
    const std::string& ns)
{
    if (positions.size() != velocities.size()) {
        RCLCPP_ERROR(node_->get_logger(), "Mismatch between positions and velocities for vector visualization.");
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;

    // First, add a marker to delete all previous markers in this namespace
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = ns;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    // Now, create an ARROW marker for each velocity vector
    int id = 0;
    for (size_t i = 0; i < positions.size(); ++i) {
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = frame_id;
        arrow_marker.header.stamp = node_->now();
        arrow_marker.ns = ns;
        arrow_marker.id = id++;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;

        // The arrow starts at the obstacle's position
        geometry_msgs::msg::Point start_point;
        start_point.x = positions[i].x();
        start_point.y = positions[i].y();
        start_point.z = 0.5; // Lift slightly above the ground

        // The arrow ends at the position predicted 1.0 second in the future
        geometry_msgs::msg::Point end_point;
        end_point.x = positions[i].x() + velocities[i].x() * 1.0; // P = P0 + V*t (t=1.0s)
        end_point.y = positions[i].y() + velocities[i].y() * 1.0;
        end_point.z = 0.5;

        arrow_marker.points.push_back(start_point);
        arrow_marker.points.push_back(end_point);

        // Define arrow dimensions
        arrow_marker.scale.x = 0.2;  // Shaft diameter
        arrow_marker.scale.y = 0.4;  // Head diameter
        arrow_marker.scale.z = 0.4;  // Head length

        // Set color
        arrow_marker.color.r = color[0];
        arrow_marker.color.g = color[1];
        arrow_marker.color.b = color[2];
        arrow_marker.color.a = 1.0;

        marker_array.markers.push_back(arrow_marker);
    }

    // Publish the array of arrows
    marker_pub_2_->publish(marker_array);
}