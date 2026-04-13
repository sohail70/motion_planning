// Copyright 2025 Soheil E.nia

#include "motion_planning/utils/rviz_visualization.hpp"

RVizVisualization::RVizVisualization(rclcpp::Node::SharedPtr node, const std::string& marker_topic)
    : node_(node),marker_id_counter_(0)  {
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10);
    marker_pub_2_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("marker2", 10);
    marker_pub_3_ = node_->create_publisher<visualization_msgs::msg::Marker>("marker3", 10);
    marker_array_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

}

void RVizVisualization::visualizeAxes(
    const Eigen::VectorXd& lower_bounds,
    const Eigen::VectorXd& upper_bounds,
    double step_size,
    const std::string& frame_id) 
{
    if (lower_bounds.size() < 2 || upper_bounds.size() < 2) return;

    // Use a MarkerArray to send everything in ONE message so nothing gets dropped
    visualization_msgs::msg::MarkerArray axis_array;

    double text_height = 2.0; 
    double offset = text_height * 1.2; 
    int marker_id = 10000;

    // --- 1. DRAW THE BOUNDING BOX (Complete Square) ---
    visualization_msgs::msg::Marker lines;
    lines.header.frame_id = frame_id;
    lines.header.stamp = rclcpp::Time();
    lines.ns = "axis_lines";
    lines.id = 0;
    lines.type = visualization_msgs::msg::Marker::LINE_STRIP; // STRIP draws a continuous perimeter
    lines.action = visualization_msgs::msg::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    lines.scale.x = 0.2; // Line thickness
    lines.color.r = 0.0; lines.color.g = 0.0; lines.color.b = 0.0; lines.color.a = 1.0;

    geometry_msgs::msg::Point p1, p2, p3, p4;
    
    // Bottom-Left
    p1.x = lower_bounds[0]; p1.y = lower_bounds[1]; p1.z = 0.1;
    // Bottom-Right
    p2.x = upper_bounds[0]; p2.y = lower_bounds[1]; p2.z = 0.1;
    // Top-Right
    p3.x = upper_bounds[0]; p3.y = upper_bounds[1]; p3.z = 0.1;
    // Top-Left
    p4.x = lower_bounds[0]; p4.y = upper_bounds[1]; p4.z = 0.1;

    // Connect the dots in a loop to close the square
    lines.points.push_back(p1); // Start bottom-left
    lines.points.push_back(p2); // Draw to bottom-right
    lines.points.push_back(p3); // Draw to top-right
    lines.points.push_back(p4); // Draw to top-left
    lines.points.push_back(p1); // Close the square back to bottom-left

    axis_array.markers.push_back(lines);

    // --- 2. DRAW X-AXIS LABELS (Bottom Edge) ---
    for (double x = lower_bounds[0]; x <= upper_bounds[0] + 1e-5; x += step_size) {
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = frame_id;
        text_marker.header.stamp = rclcpp::Time();
        text_marker.ns = "axis_labels_x";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.pose.position.x = x;
        text_marker.pose.position.y = lower_bounds[1] - offset; 
        text_marker.pose.position.z = 0.1;

        std::stringstream ss;
        ss << std::fixed << std::setprecision(0) << x;
        text_marker.text = ss.str();
        text_marker.scale.z = text_height;
        text_marker.color.r = 0.0; text_marker.color.g = 0.0; text_marker.color.b = 0.0; text_marker.color.a = 1.0;
        
        axis_array.markers.push_back(text_marker);
    }

    // --- 3. DRAW Y-AXIS LABELS (Left Edge) ---
    for (double y = lower_bounds[1]; y <= upper_bounds[1] + 1e-5; y += step_size) {
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = frame_id;
        text_marker.header.stamp = rclcpp::Time();
        text_marker.ns = "axis_labels_y";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.pose.position.x = lower_bounds[0] - offset; 
        text_marker.pose.position.y = y;
        text_marker.pose.position.z = 0.1;

        std::stringstream ss;
        ss << std::fixed << std::setprecision(0) << y;
        text_marker.text = ss.str();
        text_marker.scale.z = text_height;
        text_marker.color.r = 0.0; text_marker.color.g = 0.0; text_marker.color.b = 0.0; text_marker.color.a = 1.0;

        axis_array.markers.push_back(text_marker);
    }

    // Publish the entire array instantly
    marker_array_pub_->publish(axis_array);
}


void RVizVisualization::visualizeTimeToGoal(double t_robot, double x, double y) 
{
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map"; 
    text_marker.header.stamp = rclcpp::Time();
    text_marker.ns = "time_tracker";
    text_marker.id = 9999; 
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    
    text_marker.pose.orientation.w = 1.0;

    text_marker.pose.position.x = x;
    text_marker.pose.position.y = y;
    text_marker.pose.position.z = 1.0; 

    // Completely removed the long text, just showing the number and 's'
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << t_robot << " s";
    text_marker.text = ss.str();

    text_marker.scale.z = 4.0; 

    // Changed to Pure Blue
    text_marker.color.r = 0.0; 
    text_marker.color.g = 0.0; 
    text_marker.color.b = 1.0; 
    text_marker.color.a = 1.0;

    // marker_pub_->publish(text_marker);
    marker_buffer_.markers.push_back(text_marker);
}


void RVizVisualization::takeScreenshot(double t_robot, bool is_final) 
{
    std::string home_dir = std::getenv("HOME");
    std::string save_path = home_dir + "/Desktop/"; 

    std::stringstream ss;
    if (is_final) {
        ss << save_path << "rviz_sim_FINAL_arrive_at_" << std::fixed << std::setprecision(2) << t_robot << ".png";
    } else {
        ss << save_path << "rviz_sim_" << std::fixed << std::setprecision(0) << t_robot << ".png";
    }
    std::string filename = ss.str();

    // The size of the final square image in pixels. 
    // You can tweak this (e.g., 800, 1000, 1200) to perfectly fit your -55 to 55 bounds!
    int crop_size = 950; 

    std::stringstream cmd_ss;
    cmd_ss << "gnome-screenshot -w -B -f " << filename 
           // 1. Chop off the 35px top menu bar
           << " && mogrify -chop 0x35 " << filename 
           // 2. Go to the exact center and crop a perfect square, discarding the rest (+repage resets the canvas)
           << " && mogrify -gravity center -crop " << crop_size << "x" << crop_size << "+0+0 +repage " << filename;
    
    int result = system(cmd_ss.str().c_str());
    if (result == 0) {
        RCLCPP_INFO(node_->get_logger(), "Saved, Chopped, and Center-Cropped: %s", filename.c_str());
    }
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
    // marker_pub_->publish(marker);
    marker_buffer_.markers.push_back(marker);
}



// void RVizVisualization::visualizeNodes(const std::vector<Eigen::VectorXd>& nodes, const std::string& frame_id, const std::vector<float>& color, const std::string& ns) {
//     visualization_msgs::msg::Marker marker;
//     marker.header.frame_id = frame_id;
//     marker.header.stamp = node_->now();
//     // marker.ns = "colored_nodes";
//     marker.ns = ns;
//     marker.id = 0;
//     marker.type = visualization_msgs::msg::Marker::POINTS;
//     marker.action = visualization_msgs::msg::Marker::ADD;
//     marker.scale.x = 0.8; // Point width
//     marker.scale.y = 0.8; // Point height
//     // marker.lifetime = rclcpp::Duration(1, 0);  // 1 second visibility
//     // marker.lifetime = rclcpp::Duration::from_seconds(0.2);  // Keep for 0.2 seconds




//     // Set the color components
//         marker.color.r = color[0];
//         marker.color.g = color[1];
//         marker.color.b = color[2];
//         marker.color.a = 1.0;


//     // Add nodes to the marker
//     for (const auto& node : nodes) {
//         geometry_msgs::msg::Point point;
//         point.x = node.x();
//         point.y = node.y();
//         // point.z = 0.0; // Assuming 2D
//         point.z = (node.size() > 2) ? node.z() : 0.0; // <--- MODIFIED: Use Z if available, else 0
//         marker.points.push_back(point);
//     }

//     // Publish the marker
//     // marker_pub_->publish(marker);
//     marker_buffer_.markers.push_back(marker);
// }
void RVizVisualization::visualizeNodes(const std::vector<Eigen::VectorXd>& nodes, const std::string& frame_id, const std::vector<float>& color, const std::string& ns) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    // marker.ns = "colored_nodes";
    marker.ns = ns;
    marker.id = 0;
    
    // CHANGED: Use SPHERE_LIST instead of POINTS
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // CHANGED: SPHERE_LIST requires X, Y, and Z to be set for a perfectly round sphere.
    // If you don't set Z, the spheres will be completely flat/invisible!
    marker.scale.x = 0.5; // Sphere width
    marker.scale.y = 0.5; // Sphere length
    marker.scale.z = 0.5; // Sphere height 
    
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
        point.z = (node.size() > 2) ? node.z() : 0.05;
        marker.points.push_back(point);
    }

    // Publish the marker
    // marker_pub_->publish(marker);
    marker_buffer_.markers.push_back(marker);
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

    // marker_pub_->publish(marker);
    marker_buffer_.markers.push_back(marker);
}


void RVizVisualization::visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id) {
       visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = node_->now();
        marker.ns = "default_edge";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.15; // Line width
        marker.color.r = 0.5;  // Red
        marker.color.g = 0.5;  // Green
        marker.color.b = 0.5;  // Blue
        marker.color.a = 1.0;

        // Add edges to the marker
        for (const auto& edge : edges) {
            geometry_msgs::msg::Point start, end;
            start.x = edge.first.x();
            start.y = edge.first.y();
            start.z = -0.05; // Assuming 2D
            // start.z = (edge.first.size() > 2) ? edge.first.z() : 0.0;

            end.x = edge.second.x();
            end.y = edge.second.y();
            end.z = -0.05; // Assuming 2D
            // end.z = (edge.second.size() > 2) ? edge.second.z() : 0.0;

            marker.points.push_back(start);
            marker.points.push_back(end);
        }

        // Publish the marker
        // marker_pub_->publish(marker);
        marker_buffer_.markers.push_back(marker);
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
        start.z = 0.0; // Assuming 2D
        // start.z = (edge.first.size() > 2) ? edge.first.z() : 0.0;

        end.x = edge.second.x();
        end.y = edge.second.y();
        end.z = 0.0; // Assuming 2D
        // end.z = (edge.second.size() > 2) ? edge.second.z() : 0.0;

        marker.points.push_back(start);
        marker.points.push_back(end);
    }

    // Publish the marker
    // marker_pub_->publish(marker);
    marker_buffer_.markers.push_back(marker);
}


void RVizVisualization::visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id, const std::string& color_str, const std::string& ns) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = ns;
    marker.id = 10;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.45; // Line width

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
        // start.z = (edge.first.size() > 2) ? edge.first.z() : 0.0;

        end.x = edge.second.x();
        end.y = edge.second.y();
        end.z = 0.0; // Assuming 2D
        // end.z = (edge.second.size() > 2) ? edge.second.z() : 0.0;

        marker.points.push_back(start);
        marker.points.push_back(end);
    }

    // Publish the marker
    // marker_pub_3_->publish(marker);
    marker_buffer_.markers.push_back(marker);
}



void RVizVisualization::visualizeEdges(
    const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges,
    const std::string& frame_id,
    const std::array<float,3>& color,
    float alpha,
    float line_width,
    const std::string& ns,
    int marker_id,
    bool dashed,
    double dash_length)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = ns;
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = line_width;   // thickness
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = alpha;

    // Optionally create dashed lines by splitting each segment into short segments
    for (const auto& e : edges) {
        Eigen::Vector2d p0 = e.first.head<2>();
        Eigen::Vector2d p1 = e.second.head<2>();
        Eigen::Vector2d diff = p1 - p0;
        double len = diff.norm();

        if (!dashed || len <= 2.0 * dash_length) {
            geometry_msgs::msg::Point s, t;
            s.x = p0.x(); s.y = p0.y(); s.z = (e.first.size() > 2 ? e.first.z() : -0.05);
            t.x = p1.x(); t.y = p1.y(); t.z = (e.second.size() > 2 ? e.second.z() : -0.05);
            marker.points.push_back(s);
            marker.points.push_back(t);
            continue;
        }

        Eigen::Vector2d dir = diff / len;
        double cursor = 0.0;
        bool draw = true;
        while (cursor < len) {
            double seg_start = cursor;
            double seg_end = std::min(cursor + dash_length, len);
            if (draw) {
                Eigen::Vector2d a = p0 + dir * seg_start;
                Eigen::Vector2d b = p0 + dir * seg_end;
                geometry_msgs::msg::Point sa, sb;
                sa.x = a.x(); sa.y = a.y(); sa.z = (e.first.size() > 2 ? e.first.z() : 0.0);
                sb.x = b.x(); sb.y = b.y(); sb.z = (e.second.size() > 2 ? e.second.z() : 0.0);
                marker.points.push_back(sa);
                marker.points.push_back(sb);
            }
            // advance: gap after dash
            cursor += dash_length;
            cursor += dash_length; // gap length equal to dash_length (tweak if wanted)
            draw = true; // keep drawing each other dash (pattern already enforces gaps)
        }
    }

    // Optional: set small lifetime so markers refresh cleanly (0 => persist until overwritten)
    // marker.lifetime = rclcpp::Duration::from_seconds(0.0);

    // publish; use one publisher (or the one you already use)
    // marker_pub_->publish(marker);
    marker_buffer_.markers.push_back(marker);
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

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = ns;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);


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
    // marker.pose.position.z = 0.0;  // Keep it 2D for now (z = 0)
    marker.pose.position.z = robot_position[2];

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



void RVizVisualization::visualizeQuadcopter(
    const Eigen::Vector3d& position,
    const Eigen::VectorXd& orientation_quat,
    const std::string& frame_id,
    const std::vector<float>& color,
    const std::string& ns)
{
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Clear previous markers in this namespace
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = ns;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    // Robot parameters
    const double arm_length = 6.0;
    const double arm_width = 0.4;
    const double body_size = 2.4;

    // Convert Eigen Quaternion to geometry_msgs::Quaternion
    geometry_msgs::msg::Quaternion orientation_msg;
    orientation_msg.x = orientation_quat[0];
    orientation_msg.y = orientation_quat[1];
    orientation_msg.z = orientation_quat[2];
    orientation_msg.w = orientation_quat[3];

    // --- 1. Central Body ---
    visualization_msgs::msg::Marker body_marker;
    body_marker.header.frame_id = frame_id;
    body_marker.header.stamp = node_->now();
    body_marker.ns = ns;
    body_marker.id = 0;
    body_marker.type = visualization_msgs::msg::Marker::CUBE;
    body_marker.action = visualization_msgs::msg::Marker::ADD;
    body_marker.pose.position.x = position.x();
    body_marker.pose.position.y = position.y();
    body_marker.pose.position.z = position.z();
    body_marker.pose.orientation = orientation_msg;
    body_marker.scale.x = body_size;
    body_marker.scale.y = body_size;
    body_marker.scale.z = 0.08;
    body_marker.color.r = color[0];
    body_marker.color.g = color[1];
    body_marker.color.b = color[2];
    body_marker.color.a = 1.0;
    marker_array.markers.push_back(body_marker);

    // --- 2. Arms ---
    // Define arm positions relative to the body center
    std::vector<Eigen::Vector3d> arm_offsets = {
        {arm_length / 2.0, arm_length / 2.0, 0},
        {arm_length / 2.0, -arm_length / 2.0, 0},
        {-arm_length / 2.0, arm_length / 2.0, 0},
        {-arm_length / 2.0, -arm_length / 2.0, 0}
    };

    Eigen::Quaterniond q(orientation_quat[3], orientation_quat[0], orientation_quat[1], orientation_quat[2]);

    for (int i = 0; i < 4; ++i) {
        visualization_msgs::msg::Marker arm_marker;
        arm_marker.header = body_marker.header;
        arm_marker.ns = ns;
        arm_marker.id = i + 1;
        arm_marker.type = visualization_msgs::msg::Marker::CUBE;
        arm_marker.action = visualization_msgs::msg::Marker::ADD;

        // Rotate the offset by the quad's orientation and add to the main position
        Eigen::Vector3d rotated_offset = q * arm_offsets[i];
        arm_marker.pose.position.x = position.x() + rotated_offset.x();
        arm_marker.pose.position.y = position.y() + rotated_offset.y();
        arm_marker.pose.position.z = position.z() + rotated_offset.z();
        
        // The arms should also have the same orientation as the body
        arm_marker.pose.orientation = orientation_msg;

        arm_marker.scale.x = arm_width;
        arm_marker.scale.y = arm_width;
        arm_marker.scale.z = 0.02;
        
        // Make arms a slightly different color
        arm_marker.color.r = color[0] * 0.7f;
        arm_marker.color.g = color[1] * 0.7f;
        arm_marker.color.b = color[2] * 0.7f;
        arm_marker.color.a = 1.0;
        marker_array.markers.push_back(arm_marker);
    }

    marker_pub_2_->publish(marker_array);
}

void RVizVisualization::visualizeCube(
    const std::vector<std::tuple<Eigen::Vector2d, double, double, double>>& box_obstacles,
    const std::string& frame_id,
    const std::vector<float>& color,
    const std::string& ns) 
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = ns;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);


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
void RVizVisualization::visualizeCube(
    const std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, double>>& box_obstacles,
    const std::string& frame_id,
    const std::vector<float>& color,
    const std::string& ns) 
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = ns;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    int id = 0;

    for (const auto& box : box_obstacles) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = node_->now();
        marker.ns = ns;
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        const Eigen::Vector3d& position = std::get<0>(box);
        const Eigen::Vector3d& dimensions = std::get<1>(box);
        double rotation = std::get<2>(box);

        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();

        // =========================================================================================
        // CORRECTED: Manually convert yaw to quaternion, removing the tf2 dependency.
        // =========================================================================================
        double cy = cos(rotation * 0.5);
        double sy = sin(rotation * 0.5);
        // A pure yaw rotation is around the Z-axis, so X and Y components are 0.
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = sy;
        marker.pose.orientation.w = cy;
        // =========================================================================================

        marker.scale.x = dimensions.x();
        marker.scale.y = dimensions.y();
        marker.scale.z = dimensions.z();

        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = 0.5;

        marker_array.markers.push_back(marker);
    }
    
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
    // --- Create a single MarkerArray for all our actions ---
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


    // --- Now, add the new markers to the SAME array ---
    int id = 0;
    for (const auto& obstacle : obstacles) {
        if (!obstacle.is_dynamic || obstacle.velocity.norm() < 0.01) {
            continue;
        }

        Eigen::Vector2d future_position = obstacle.position + obstacle.velocity * prediction_horizon;

        // Create the velocity vector (a line) marker
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

void RVizVisualization::visualizeVelocityVectors(
    const std::vector<Eigen::Vector3d>& positions, 
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

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = ns;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    int id = 0;
    for (size_t i = 0; i < positions.size(); ++i) {
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = frame_id;
        arrow_marker.header.stamp = node_->now();
        arrow_marker.ns = ns;
        arrow_marker.id = id++;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start_point;
        start_point.x = positions[i].x();
        start_point.y = positions[i].y();
        start_point.z = positions[i].z(); 

        geometry_msgs::msg::Point end_point;
        end_point.x = positions[i].x() + velocities[i].x(); // Predict 1 second ahead
        end_point.y = positions[i].y() + velocities[i].y();
        end_point.z = positions[i].z(); 

        arrow_marker.points.push_back(start_point);
        arrow_marker.points.push_back(end_point);

        arrow_marker.scale.x = 0.2;
        arrow_marker.scale.y = 0.4;
        arrow_marker.scale.z = 0.4;

        arrow_marker.color.r = color[0];
        arrow_marker.color.g = color[1];
        arrow_marker.color.b = color[2];
        arrow_marker.color.a = 1.0;

        marker_array.markers.push_back(arrow_marker);
    }

    marker_pub_2_->publish(marker_array);
}


void RVizVisualization::visualizeCircle(
    const Eigen::Vector2d& center,
    double radius,
    const std::string& frame_id,
    const std::vector<float>& color,
    const std::string& ns)
{
    visualization_msgs::msg::MarkerArray marker_array;

    // A marker to clear any previous circles in this namespace
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = ns;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    // The marker for the new circle
    visualization_msgs::msg::Marker circle_marker;
    circle_marker.header.frame_id = frame_id;
    circle_marker.header.stamp = node_->now();
    circle_marker.ns = ns;
    circle_marker.id = 0; // Unique ID for this circle
    circle_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    circle_marker.action = visualization_msgs::msg::Marker::ADD;

    // Position the circle's center
    circle_marker.pose.position.x = center.x();
    circle_marker.pose.position.y = center.y();
    circle_marker.pose.position.z = -0.1; // Place it slightly below the points

    // The scale of a CYLINDER is its (diameter_x, diameter_y, height)
    circle_marker.scale.x = radius * 2.0;
    circle_marker.scale.y = radius * 2.0;
    circle_marker.scale.z = 0.05; // Make it very thin

    // Set the color and transparency
    circle_marker.color.r = color[0];
    circle_marker.color.g = color[1];
    circle_marker.color.b = color[2];
    circle_marker.color.a = 0.35; // Semi-transparent

    marker_array.markers.push_back(circle_marker);

    // The publisher for cylinders and cubes uses a MarkerArray
    marker_pub_2_->publish(marker_array);
}



void RVizVisualization::visualizeText(
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<std::string>& texts,
    const std::string& frame_id,
    const std::string& ns)
{
    if (points.size() != texts.size()) {
        RCLCPP_ERROR(node_->get_logger(), "Mismatch between points and texts for visualization.");
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;

    // Clear previous text markers in this namespace
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = ns;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    // Create a new marker for each text element
    for (size_t i = 0; i < points.size(); ++i) {
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = frame_id;
        text_marker.header.stamp = node_->now();
        text_marker.ns = ns;
        text_marker.id = i;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;

        // Position the text slightly above the node
        text_marker.pose.position.x = points[i].x();
        text_marker.pose.position.y = points[i].y();
        text_marker.pose.position.z = points[i].z() + 0.3; // Z-offset

        // Set the text content
        text_marker.text = texts[i];

        // Set the scale (height of the text)
        text_marker.scale.z = 0.3;

        // Set the color
        text_marker.color.r = 1.0f;
        text_marker.color.g = 1.0f;
        text_marker.color.b = 1.0f;
        text_marker.color.a = 1.0; // Opaque white

        marker_array.markers.push_back(text_marker);
    }

    marker_pub_2_->publish(marker_array);
}


void RVizVisualization::visualizeSpheres(
    const std::vector<Eigen::VectorXd>& obstacles_positions, 
    const std::vector<double>& radii, 
    const std::string& frame_id,
    const std::vector<float>& color,
    const std::string& ns) 
{
    if (obstacles_positions.size() != radii.size()) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "Mismatch between number of obstacles (%zu) and radii (%zu).", 
                     obstacles_positions.size(), radii.size());
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = ns;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    int id = 0;
    for (size_t i = 0; i < obstacles_positions.size(); ++i) {
        const auto& obstacle_pos = obstacles_positions[i];
        double radius = radii[i];

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = node_->now();
        marker.ns = ns;
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE; // Use SPHERE type
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = obstacle_pos.x();
        marker.pose.position.y = obstacle_pos.y();
        // Use the Z coordinate from the passed vector
        marker.pose.position.z = (obstacle_pos.size() > 2) ? obstacle_pos.z() : 0.0;

        // For a SPHERE, scale is the diameter in all dimensions
        marker.scale.x = 2 * radius;
        marker.scale.y = 2 * radius;
        marker.scale.z = 2 * radius;

        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = 0.6; // A bit more solid than before

        marker_array.markers.push_back(marker);
    }

    marker_pub_2_->publish(marker_array);
}

void RVizVisualization::visualizeLineToNearest(
    const Eigen::Vector3d& robot_pos,
    const Eigen::Vector3d& nearest_obs_pos,
    const std::string& frame_id,
    const std::vector<float>& color,
    const std::string& ns)
{
    visualization_msgs::msg::MarkerArray marker_array;

    // Clear previous line
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = ns;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    // Create the line marker
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = frame_id;
    line_marker.header.stamp = node_->now();
    line_marker.ns = ns;
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;

    // Line thickness
    line_marker.scale.x = 0.1;

    // Line color
    line_marker.color.r = color[0];
    line_marker.color.g = color[1];
    line_marker.color.b = color[2];
    line_marker.color.a = 0.8;

    // Define start and end points
    geometry_msgs::msg::Point start_p, end_p;
    start_p.x = robot_pos.x();
    start_p.y = robot_pos.y();
    start_p.z = robot_pos.z();
    end_p.x = nearest_obs_pos.x();
    end_p.y = nearest_obs_pos.y();
    end_p.z = nearest_obs_pos.z();

    line_marker.points.push_back(start_p);
    line_marker.points.push_back(end_p);
    
    marker_array.markers.push_back(line_marker);
    marker_pub_2_->publish(marker_array);
}

void RVizVisualization::visualizeDottedLineToNearest(
    const Eigen::Vector3d& robot_pos,
    const Eigen::Vector3d& nearest_obs_pos,
    int num_points, // How many dots to draw
    const std::string& frame_id,
    const std::vector<float>& color,
    const std::string& ns)
{
    visualization_msgs::msg::MarkerArray marker_array;

    // Clear previous line/dots
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = ns;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    // Create the points marker
    visualization_msgs::msg::Marker points_marker;
    points_marker.header.frame_id = frame_id;
    points_marker.header.stamp = node_->now();
    points_marker.ns = ns;
    points_marker.id = 0;
    // --- CHANGE: Use POINTS type ---
    points_marker.type = visualization_msgs::msg::Marker::POINTS;
    points_marker.action = visualization_msgs::msg::Marker::ADD;

    // Size of each dot
    points_marker.scale.x = 0.2;
    points_marker.scale.y = 0.2;

    // Color of the dots
    points_marker.color.r = color[0];
    points_marker.color.g = color[1];
    points_marker.color.b = color[2];
    points_marker.color.a = 1.0;

    // Calculate the points along the line
    Eigen::Vector3d direction_vec = nearest_obs_pos - robot_pos;
    for (int i = 0; i <= num_points; ++i) {
        // Interpolate to find the position of the next dot
        Eigen::Vector3d point_pos = robot_pos + direction_vec * (static_cast<double>(i) / num_points);
        
        geometry_msgs::msg::Point p;
        p.x = point_pos.x();
        p.y = point_pos.y();
        p.z = point_pos.z();
        points_marker.points.push_back(p);
    }
    
    marker_array.markers.push_back(points_marker);
    marker_pub_2_->publish(marker_array);
}

void RVizVisualization::clearMarkers(const std::string& ns) {
    visualization_msgs::msg::MarkerArray clear_array;
    visualization_msgs::msg::Marker marker;
    
    marker.header.frame_id = "map";
    marker.header.stamp = node_->now(); // Use node_->now() instead of Clock().now()
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    
    // if ns is empty, DELETEALL wipes everything. 
    // If ns is provided, it wipes just that namespace.
    if (!ns.empty()) {
        marker.ns = ns;
    }
    
    clear_array.markers.push_back(marker);
    
    // Publish to the ARRAY publisher!
    marker_array_pub_->publish(clear_array);
    
    // Wipe the internal buffer so old frame data doesn't come back to life
    marker_buffer_.markers.clear(); 
}


// void RVizVisualization::visualizePathGradient(
//     const std::vector<Eigen::VectorXd>& path_waypoints,
//     const std::vector<double>& waypoint_costs, // <--- TRUE COSTS
//     const std::string& frame_id,
//     double global_max_cost) 
// {
//     if (path_waypoints.empty() || path_waypoints.size() != waypoint_costs.size()) return;
//     if (global_max_cost < 1e-6) global_max_cost = 1.0; 
    
//     visualization_msgs::msg::Marker marker;
//     marker.header.frame_id = frame_id;
//     marker.header.stamp = node_->now();
//     marker.ns = "path_gradient";
//     marker.id = 0;
//     marker.type = visualization_msgs::msg::Marker::LINE_STRIP; 
//     marker.action = visualization_msgs::msg::Marker::ADD;
//     marker.pose.orientation.w = 1.0;
//     marker.scale.x = 1.5; 
    
//     for (size_t i = 0; i < path_waypoints.size(); ++i) {
//         geometry_msgs::msg::Point pt;
//         pt.x = path_waypoints[i].x();
//         pt.y = path_waypoints[i].y();
//         pt.z = -0.02; 
//         marker.points.push_back(pt);
        
//         std_msgs::msg::ColorRGBA color;
//         color.a = 1.0; 
        
//         // NO EUCLIDEAN NORM! Use the exact topological cost.
//         double ratio = 1.0 - std::clamp(waypoint_costs[i] / global_max_cost, 0.0, 1.0); 
        
//         // Corrected Flawless HSV Gradient (Red -> Yellow -> Green -> Cyan -> Blue)
//         if (ratio < 0.25) { 
//             color.r = 1.0; 
//             color.g = ratio * 4.0; 
//             color.b = 0.0; 
//         } else if (ratio < 0.5) { 
//             color.r = 1.0 - (ratio - 0.25) * 4.0; 
//             color.g = 1.0; 
//             color.b = 0.0; 
//         } else if (ratio < 0.75) { 
//             color.r = 0.0; 
//             color.g = 1.0; 
//             color.b = (ratio - 0.5) * 4.0; 
//         } else { 
//             color.r = 0.0; 
//             color.g = 1.0 - (ratio - 0.75) * 4.0; 
//             color.b = 1.0; 
//         }
        
//         marker.colors.push_back(color);
//     }
    
//     // marker_pub_->publish(marker);
//     marker_buffer_.markers.push_back(marker);
// }

void RVizVisualization::visualizePathGradient(
    const std::vector<Eigen::VectorXd>& path_waypoints,
    const std::vector<double>& waypoint_costs, 
    const std::string& frame_id,
    double global_max_cost) 
{
    if (path_waypoints.empty() || path_waypoints.size() != waypoint_costs.size()) return;
    if (global_max_cost < 1e-6) global_max_cost = 1.0; 
    
    // Helper to get gradient colors
    auto getColor = [&](double cost) -> std_msgs::msg::ColorRGBA {
        std_msgs::msg::ColorRGBA color;
        color.a = 1.0; 
        double ratio = 1.0 - std::clamp(cost / global_max_cost, 0.0, 1.0); 
        if (ratio < 0.25) { color.r = 1.0; color.g = ratio * 4.0; color.b = 0.0; } 
        else if (ratio < 0.5) { color.r = 1.0 - (ratio - 0.25) * 4.0; color.g = 1.0; color.b = 0.0; } 
        else if (ratio < 0.75) { color.r = 0.0; color.g = 1.0; color.b = (ratio - 0.5) * 4.0; } 
        else { color.r = 0.0; color.g = 1.0 - (ratio - 0.75) * 4.0; color.b = 1.0; }
        return color;
    };

    // If Waypoint Size == 3, it's R2T (X, Y, T).
    // R2T has sharp, discontinuous corners that mathematically break LINE_STRIP.
    if (path_waypoints[0].size() == 3) {
        
        visualization_msgs::msg::Marker lines_marker;
        lines_marker.header.frame_id = frame_id;
        lines_marker.header.stamp = node_->now();
        lines_marker.ns = "path_gradient_lines";
        lines_marker.id = 0;
        lines_marker.type = visualization_msgs::msg::Marker::LINE_LIST; // Prevents twisting
        lines_marker.action = visualization_msgs::msg::Marker::ADD;
        lines_marker.pose.orientation.w = 1.0;
        lines_marker.scale.x = 1.5; 

        // R2T lines
        for (size_t i = 0; i < path_waypoints.size() - 1; ++i) {
            geometry_msgs::msg::Point pA, pB;
            pA.x = path_waypoints[i].x();   pA.y = path_waypoints[i].y();   pA.z = -0.02;
            pB.x = path_waypoints[i+1].x(); pB.y = path_waypoints[i+1].y(); pB.z = -0.02;
            
            // Skip zero-length R2T duplicates
            if (std::hypot(pB.x - pA.x, pB.y - pA.y) < 1e-3) continue;

            lines_marker.points.push_back(pA);
            lines_marker.points.push_back(pB);
            lines_marker.colors.push_back(getColor(waypoint_costs[i]));
            lines_marker.colors.push_back(getColor(waypoint_costs[i+1]));
        }
        
        marker_buffer_.markers.push_back(lines_marker);
    } 
    // Dubins (4D) or Thruster (5D) have smooth C1-continuous corners.
    // LINE_STRIP works flawlessly for them!
    else {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = node_->now();
        marker.ns = "path_gradient";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP; 
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.5; 
        
        for (size_t i = 0; i < path_waypoints.size(); ++i) {
            geometry_msgs::msg::Point pt;
            pt.x = path_waypoints[i].x();
            pt.y = path_waypoints[i].y();
            pt.z = -0.02; 
            marker.points.push_back(pt);
            marker.colors.push_back(getColor(waypoint_costs[i]));
        }
        marker_buffer_.markers.push_back(marker);
    }
}
void RVizVisualization::visualizeTreeGradient(
    const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges,
    const std::vector<double>& costs,
    const std::string& frame_id) 
{
    if (edges.empty() || edges.size() != costs.size()) return;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "tree_gradient";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05; // Thin lines for the tree so it doesn't look cluttered

    for (size_t i = 0; i < edges.size(); ++i) {
        geometry_msgs::msg::Point start, end;
        start.x = edges[i].first.x();  start.y = edges[i].first.y();  start.z = -0.1; 
        end.x = edges[i].second.x();   end.y = edges[i].second.y();   end.z = -0.1; 
        
        marker.points.push_back(start);
        marker.points.push_back(end);

        // Generate color based on the cost (Blue = Low Cost, Red = High Cost)
        std_msgs::msg::ColorRGBA color;
        color.a = 1.0; // Slightly transparent looks beautiful for dense trees
        double ratio = std::clamp(costs[i], 0.0, 1.0); 
        
        // Simple Blue to Red Gradient
        color.r = ratio;
        color.g = 0.0;
        color.b = 1.0 - ratio;

        // Line lists require one color per point (start and end of the segment)
        marker.colors.push_back(color);
        marker.colors.push_back(color);
    }
    
    marker_pub_->publish(marker);
}

void RVizVisualization::visualizeContinuousMesh(
    const std::vector<Eigen::VectorXd>& points,
    const std::vector<double>& costs,
    double global_max_cost,
    double neighborhood_radius,
    const Eigen::VectorXd& lower_bounds,
    const Eigen::VectorXd& upper_bounds,
    const std::string& frame_id) 
{
    if (points.empty()) return;
    if (global_max_cost < 1e-6) global_max_cost = 1.0;

    double res = 2.0; // Fast, coarse grid
    int nx = std::max(1, (int)std::ceil((upper_bounds[0] - lower_bounds[0]) / res)) + 1;
    int ny = std::max(1, (int)std::ceil((upper_bounds[1] - lower_bounds[1]) / res)) + 1;
    int grid_size = nx * ny;

    int min_gx = nx, max_gx = 0, min_gy = ny, max_gy = 0;
    
    std::vector<double> cell_min_cost(grid_size, std::numeric_limits<double>::infinity());
    std::vector<bool> cell_has_node(grid_size, false);

    for (size_t i = 0; i < points.size(); ++i) {
        int cx = std::round((points[i].x() - lower_bounds[0]) / res);
        int cy = std::round((points[i].y() - lower_bounds[1]) / res);
        if (cx >= 0 && cx < nx && cy >= 0 && cy < ny) {
            int idx = cx + cy * nx;
            
            if (!cell_has_node[idx] || costs[i] < cell_min_cost[idx]) {
                cell_min_cost[idx] = costs[i];
                cell_has_node[idx] = true;
            }
            
            if (cx < min_gx) min_gx = cx;
            if (cx > max_gx) max_gx = cx;
            if (cy < min_gy) min_gy = cy;
            if (cy > max_gy) max_gy = cy;
        }
    }

    // Because res is so large (2.5), we MUST enforce a minimum brush radius 
    // so the corners of the massive grid spaces can connect and blend colors!
    double effective_radius = std::max(neighborhood_radius, res * 1.5); 
    int brush_radius = std::max(1, (int)std::ceil(effective_radius / res)); 
    int brush_size = 2 * brush_radius + 1;
    std::vector<double> brush(brush_size * brush_size, 0.0);
    
    for (int dx = -brush_radius; dx <= brush_radius; ++dx) {
        for (int dy = -brush_radius; dy <= brush_radius; ++dy) {
            double dist = std::sqrt(dx*dx + dy*dy) * res;
            if (dist <= effective_radius) {
                double weight = std::max(0.0, 1.0 - (dist / effective_radius));
                brush[(dx + brush_radius) + (dy + brush_radius) * brush_size] = weight * weight;
            }
        }
    }

    std::vector<double> weight_grid(grid_size, 0.0);
    std::vector<double> cost_grid(grid_size, 0.0);

    for (int cx = min_gx; cx <= max_gx; ++cx) {
        for (int cy = min_gy; cy <= max_gy; ++cy) {
            int src_idx = cx + cy * nx;
            if (cell_has_node[src_idx]) {
                
                double best_cost = cell_min_cost[src_idx];
                
                for (int dx = -brush_radius; dx <= brush_radius; ++dx) {
                    for (int dy = -brush_radius; dy <= brush_radius; ++dy) {
                        int gx = cx + dx;
                        int gy = cy + dy;
                        if (gx >= 0 && gx < nx && gy >= 0 && gy < ny) {
                            double w = brush[(dx + brush_radius) + (dy + brush_radius) * brush_size];
                            if (w > 0.0) {
                                int tgt_idx = gx + gy * nx;
                                weight_grid[tgt_idx] += w;
                                cost_grid[tgt_idx] += w * best_cost;
                            }
                        }
                    }
                }
            }
        }
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "continuous_background";
    marker.id = 0; 
    
    // BACK TO TRIANGLES: This tells the GPU to blend the colors smoothly!
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST; 
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0; 
    // Triangles require scale to be exactly 1.0
    marker.scale.x = 1.0; 
    marker.scale.y = 1.0; 
    marker.scale.z = 1.0; 

    auto getColor = [&](double true_cost) {
        std_msgs::msg::ColorRGBA c;
        c.a = 0.5; // Fully opaque
        double ratio = 1.0 - std::clamp(true_cost / global_max_cost, 0.0, 1.0);

        if (ratio < 0.25) { 
            c.r = 1.0; c.g = ratio * 4.0; c.b = 0.0; 
        } else if (ratio < 0.5) { 
            c.r = 1.0 - (ratio - 0.25) * 4.0; c.g = 1.0; c.b = 0.0; 
        } else if (ratio < 0.75) { 
            c.r = 0.0; c.g = 1.0; c.b = (ratio - 0.5) * 4.0; 
        } else { 
            c.r = 0.0; c.g = 1.0 - (ratio - 0.75) * 4.0; c.b = 1.0; 
        }
        return c;
    };

    int start_x = std::max(0, min_gx - brush_radius);
    int end_x = std::min(nx - 2, max_gx + brush_radius); // -2 ensures bounds safety
    int start_y = std::max(0, min_gy - brush_radius);
    int end_y = std::min(ny - 2, max_gy + brush_radius);

    for (int x = start_x; x <= end_x; x++) {
        for (int y = start_y; y <= end_y; y++) {
            
            // Get the 4 corners of the current 2.5m grid space
            int i00 = x + y * nx;
            int i10 = (x + 1) + y * nx;
            int i01 = x + (y + 1) * nx;
            int i11 = (x + 1) + (y + 1) * nx;

            double w00 = weight_grid[i00], w10 = weight_grid[i10];
            double w01 = weight_grid[i01], w11 = weight_grid[i11];

            // If all 4 corners have valid data, draw the seamlessly blended square (2 triangles)
            if (w00 > 1e-6 && w10 > 1e-6 && w01 > 1e-6 && w11 > 1e-6) {
                double c00 = cost_grid[i00] / w00;
                double c10 = cost_grid[i10] / w10;
                double c01 = cost_grid[i01] / w01;
                double c11 = cost_grid[i11] / w11;

                geometry_msgs::msg::Point p00, p10, p01, p11;
                p00.x = lower_bounds[0] + x * res;       p00.y = lower_bounds[1] + y * res;       p00.z = -0.1;
                p10.x = lower_bounds[0] + (x + 1) * res; p10.y = lower_bounds[1] + y * res;       p10.z = -0.1;
                p01.x = lower_bounds[0] + x * res;       p01.y = lower_bounds[1] + (y + 1) * res; p01.z = -0.1;
                p11.x = lower_bounds[0] + (x + 1) * res; p11.y = lower_bounds[1] + (y + 1) * res; p11.z = -0.1;

                std_msgs::msg::ColorRGBA col00 = getColor(c00);
                std_msgs::msg::ColorRGBA col10 = getColor(c10);
                std_msgs::msg::ColorRGBA col01 = getColor(c01);
                std_msgs::msg::ColorRGBA col11 = getColor(c11);

                // Triangle 1
                marker.points.push_back(p00); marker.colors.push_back(col00);
                marker.points.push_back(p10); marker.colors.push_back(col10);
                marker.points.push_back(p01); marker.colors.push_back(col01);

                // Triangle 2
                marker.points.push_back(p10); marker.colors.push_back(col10);
                marker.points.push_back(p11); marker.colors.push_back(col11);
                marker.points.push_back(p01); marker.colors.push_back(col01);
            }
        }
    }
    
    marker_pub_->publish(marker);
}


void RVizVisualization::publishObstacleFrame(
    const std::vector<Eigen::VectorXd>& safe_cyls, const std::vector<double>& safe_radii,
    const std::vector<Eigen::VectorXd>& threat_cyls, const std::vector<double>& threat_radii,
    const std::vector<std::tuple<Eigen::Vector2d, double, double, double>>& safe_boxes,
    const std::vector<std::tuple<Eigen::Vector2d, double, double, double>>& threat_boxes,
    const std::vector<Eigen::Vector2d>& safe_vel_pos, const std::vector<Eigen::Vector2d>& safe_vel_val,
    const std::vector<Eigen::Vector2d>& threat_vel_pos, const std::vector<Eigen::Vector2d>& threat_vel_val,
    
    // --- NEW PARAMETERS FOR ROBOT ---
    const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& robot_trace_edges,
    const Eigen::Vector3d& robot_pos,
    const Eigen::VectorXd& robot_orientation, // Quaternion
    const std::vector<float>& robot_color,
    double robot_inflation,
    
    const std::string& frame_id)
{
    visualization_msgs::msg::MarkerArray frame_array;
    auto now_stamp = node_->now();

    std::vector<std::string> namespaces = {
        "obs_cyl_safe", "obs_cyl_threat", 
        "obs_box_safe", "obs_box_threat", 
        "obs_vel_safe", "obs_vel_threat",
        "robot_trace", "simulated_robot", "simulated_robot_inflation"
    };

    for (const auto& ns : namespaces) {
        visualization_msgs::msg::Marker clear_m;
        clear_m.header.frame_id = frame_id;
        clear_m.header.stamp = now_stamp;
        clear_m.ns = ns;
        clear_m.action = visualization_msgs::msg::Marker::DELETEALL;
        frame_array.markers.push_back(clear_m);
    }

    auto add_cylinders = [&](const std::vector<Eigen::VectorXd>& pos, const std::vector<double>& rad, 
                             const std::vector<float>& col, const std::string& ns) {
        for (size_t i = 0; i < pos.size(); ++i) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id; m.header.stamp = now_stamp; m.ns = ns; m.id = i;
            m.type = visualization_msgs::msg::Marker::CYLINDER; m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = pos[i].x(); m.pose.position.y = pos[i].y(); m.pose.position.z = 0.05;
            m.scale.x = 2 * rad[i]; m.scale.y = 2 * rad[i]; m.scale.z = 0.1;
            m.color.r = col[0]; m.color.g = col[1]; m.color.b = col[2]; m.color.a = col[3];
            frame_array.markers.push_back(m);
        }
    };

    auto add_boxes = [&](const std::vector<std::tuple<Eigen::Vector2d, double, double, double>>& boxes, 
                         const std::vector<float>& col, const std::string& ns) {
        for (size_t i = 0; i < boxes.size(); ++i) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id; m.header.stamp = now_stamp; m.ns = ns; m.id = i;
            m.type = visualization_msgs::msg::Marker::CUBE; m.action = visualization_msgs::msg::Marker::ADD;
            
            const auto& b = boxes[i];
            m.pose.position.x = std::get<0>(b).x(); m.pose.position.y = std::get<0>(b).y(); m.pose.position.z = 0.05;
            
            double rot = std::get<3>(b);
            m.pose.orientation.z = sin(rot * 0.5); m.pose.orientation.w = cos(rot * 0.5);
            
            m.scale.x = std::get<1>(b); m.scale.y = std::get<2>(b); m.scale.z = 0.1;
            m.color.r = col[0]; m.color.g = col[1]; m.color.b = col[2]; m.color.a = col[3];
            frame_array.markers.push_back(m);
        }
    };

    auto add_arrows = [&](const std::vector<Eigen::Vector2d>& pos, const std::vector<Eigen::Vector2d>& vel, 
                          const std::vector<float>& col, const std::string& ns) {
        for (size_t i = 0; i < pos.size(); ++i) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id; m.header.stamp = now_stamp; m.ns = ns; m.id = i;
            m.type = visualization_msgs::msg::Marker::ARROW; m.action = visualization_msgs::msg::Marker::ADD;
            
            geometry_msgs::msg::Point start, end;
            start.x = pos[i].x(); start.y = pos[i].y(); start.z = 0.5;
            end.x = pos[i].x() + vel[i].x(); end.y = pos[i].y() + vel[i].y(); end.z = 0.5;
            m.points.push_back(start); m.points.push_back(end);
            
            m.scale.x = 0.4; m.scale.y = 1.2; m.scale.z = 1.2;
            m.color.r = col[0]; m.color.g = col[1]; m.color.b = col[2]; m.color.a = 1.0;
            frame_array.markers.push_back(m);
        }
    };

    add_cylinders(safe_cyls, safe_radii, {0.89f, 0.26f, 0.2f, 1.0f}, "obs_cyl_safe");
    add_cylinders(threat_cyls, threat_radii, {1.0f, 0.0f, 0.0f, 0.8f}, "obs_cyl_threat");

    add_boxes(safe_boxes, {0.89f, 0.26f, 0.2f, 1.0f}, "obs_box_safe");
    add_boxes(threat_boxes, {1.0f, 0.0f, 0.0f, 0.8f}, "obs_box_threat");

    add_arrows(safe_vel_pos, safe_vel_val, {0.65f, 0.15f, 0.1f}, "obs_vel_safe"); 
    add_arrows(threat_vel_pos, threat_vel_val, {1.0f, 0.0f, 0.0f}, "obs_vel_threat"); 

    // 4. BUILD ROBOT TRACE
    if (!robot_trace_edges.empty()) {
        visualization_msgs::msg::Marker trace_m;
        trace_m.header.frame_id = frame_id;
        trace_m.header.stamp = now_stamp;
        trace_m.ns = "robot_trace";
        trace_m.id = 0;
        trace_m.type = visualization_msgs::msg::Marker::LINE_LIST;
        trace_m.action = visualization_msgs::msg::Marker::ADD;
        trace_m.scale.x = 0.45; 
        trace_m.color.r = 0.95f; trace_m.color.g = 0.6f; trace_m.color.b = 0.0f; trace_m.color.a = 1.0f;

        for (const auto& edge : robot_trace_edges) {
            geometry_msgs::msg::Point p1, p2;
            p1.x = edge.first.x(); p1.y = edge.first.y(); p1.z = 0.0;
            p2.x = edge.second.x(); p2.y = edge.second.y(); p2.z = 0.0;
            trace_m.points.push_back(p1);
            trace_m.points.push_back(p2);
        }
        frame_array.markers.push_back(trace_m);
    }

    
    // Robot Arrow
    {
        visualization_msgs::msg::Marker arrow_m;
        arrow_m.header.frame_id = frame_id;
        arrow_m.header.stamp = now_stamp;
        arrow_m.ns = "simulated_robot";
        arrow_m.id = 0;
        arrow_m.type = visualization_msgs::msg::Marker::ARROW;
        arrow_m.action = visualization_msgs::msg::Marker::ADD;

        arrow_m.pose.position.x = robot_pos.x();
        arrow_m.pose.position.y = robot_pos.y();
        arrow_m.pose.position.z = robot_pos.z();

        arrow_m.pose.orientation.x = robot_orientation[0];
        arrow_m.pose.orientation.y = robot_orientation[1];
        arrow_m.pose.orientation.z = robot_orientation[2];
        arrow_m.pose.orientation.w = robot_orientation[3];

        arrow_m.scale.x = 1.5;  // Length
        arrow_m.scale.y = 0.5;  // Width
        arrow_m.scale.z = 0.2;  // Head

        arrow_m.color.r = 0;//robot_color[0];
        arrow_m.color.g = 0;//robot_color[1];
        arrow_m.color.b = 0.6;
        arrow_m.color.a = 1.0;

        frame_array.markers.push_back(arrow_m);
    }

    // Robot Inflation Cylinder
    if (robot_inflation > 0.0) {
        visualization_msgs::msg::Marker cyl_m;
        cyl_m.header.frame_id = frame_id;
        cyl_m.header.stamp = now_stamp;
        cyl_m.ns = "simulated_robot_inflation";
        cyl_m.id = 0;
        cyl_m.type = visualization_msgs::msg::Marker::CYLINDER;
        cyl_m.action = visualization_msgs::msg::Marker::ADD;

        cyl_m.pose.position.x = robot_pos.x();
        cyl_m.pose.position.y = robot_pos.y();
        cyl_m.pose.position.z = robot_pos.z(); // Or 0.0 for ground
        cyl_m.pose.orientation.w = 1.0;

        cyl_m.scale.x = 2.0 * robot_inflation;
        cyl_m.scale.y = 2.0 * robot_inflation;
        cyl_m.scale.z = 0.05;

        cyl_m.color.r = robot_color[0];
        cyl_m.color.g = robot_color[1];
        cyl_m.color.b = robot_color[2];
        cyl_m.color.a = 1.0; // Transparent

        frame_array.markers.push_back(cyl_m);
    }

    // PUBLISH EVERYTHING AT ONCE
    marker_pub_2_->publish(frame_array);
}

void RVizVisualization::triggerPublish() {
    if (!marker_buffer_.markers.empty()) {
        marker_array_pub_->publish(marker_buffer_);
        marker_buffer_.markers.clear(); // Empty the buffer for the next control loop
    }
}