// Copyright 2025 Soheil E.nia

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker_array.hpp"

#include <Eigen/Dense>
#include <vector>
#include "motion_planning/utils/visualization.hpp"

class RVizVisualization : public Visualization {
public:
    RVizVisualization(rclcpp::Node::SharedPtr node, const std::string& marker_topic = "fmtx_markers");

    void visualizeNodes(const std::vector<Eigen::VectorXd>& nodes, const std::string& frame_id = "map") override;
    
    void visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id = "map") override;

    void visualizeCylinder(const std::vector<Eigen::VectorXd>& obstacles, double radius, const std::string& frame_id);


private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_2_;

};