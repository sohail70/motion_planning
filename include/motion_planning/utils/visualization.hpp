// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/pch.hpp"

// Abstract base class for visualization
class Visualization {
public:
    virtual ~Visualization() = default;

    // Visualize nodes as points
    virtual void visualizeNodes(const std::vector<Eigen::VectorXd>& nodes, const std::string& frame_id = "map") = 0;
    virtual void visualizeNodes(const std::vector<Eigen::VectorXd>& nodes, const std::string& frame_id, const std::string& color_str) = 0;

    // Visualize edges as lines
    virtual void visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id = "map") = 0;
};