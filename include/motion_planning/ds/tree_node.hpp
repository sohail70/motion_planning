// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/ds/node.hpp"

class TreeNode : public Node {
 public:
    explicit TreeNode(std::unique_ptr<State> state, std::shared_ptr<Node> parent = nullptr);
    Eigen::VectorXd getStateVlaue() const override;
    double getCost() const override;
    std::shared_ptr<Node> getParent() const;
    void setParent(std::shared_ptr<Node> parent);

 private:
    std::unique_ptr<State> state_;
    std::shared_ptr<Node> parent_;
};
