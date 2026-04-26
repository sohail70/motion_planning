// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/pch.hpp"
// Abstract base class for visualization
class Visualization {
public:
    virtual ~Visualization() = default;

    // Visualize nodes as points
    virtual void visualizeNodes(const std::vector<Eigen::VectorXd>& nodes, const std::string& frame_id = "map") = 0;
    virtual void visualizeNodes(const std::vector<Eigen::VectorXd>& nodes, const std::string& frame_id, const std::vector<float>& color, const std::string& ns) = 0;

    // Visualize edges as lines
    virtual void visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id = "map") = 0;
    virtual void visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id, const std::string& color_str) = 0;
    virtual void visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id, const std::string& color_str,const std::string& ns) = 0;
    virtual void visualizeEdges( const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id, const std::array<float,3>& color, float alpha, float line_width, const std::string& ns, int marker_id, bool dashed = false, double dash_length = 0.5) = 0;
    virtual void visualizeSingleEdge(const Eigen::VectorXd& start_point, const Eigen::VectorXd& end_point, int edge_id, const std::string& frame_id) = 0 ;
    virtual void visualizeText( const std::vector<Eigen::Vector3d>& points, const std::vector<std::string>& texts, const std::string& frame_id, const std::string& ns) = 0 ;
    virtual void visualizePathGradient( const std::vector<Eigen::VectorXd>& path_waypoints, const std::vector<double>& waypoint_costs, const std::string& frame_id, double global_max_cost) = 0;
    virtual void visualizeContinuousMesh( const std::vector<Eigen::VectorXd>& points, const std::vector<double>& costs, double global_max_cost, double neighborhood_radius, const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds, const std::string& frame_id = "map") = 0;
    virtual void visualizeTreeGradient( const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::vector<double>& costs, const std::string& frame_id) = 0;
    virtual void triggerPublish() = 0;
};