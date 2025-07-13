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
    
    virtual void visualizeSingleEdge(const Eigen::VectorXd& start_point, const Eigen::VectorXd& end_point, int edge_id, const std::string& frame_id) = 0 ;
    virtual void visualizeText( const std::vector<Eigen::Vector3d>& points, const std::vector<std::string>& texts, const std::string& frame_id, const std::string& ns) = 0 ;
};