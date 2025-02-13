// Copyright 2025 Soheil E.nia

#include "motion_planning/ds/tree_node.hpp"

TreeNode::TreeNode(std::unique_ptr<State> state, std::shared_ptr<Node> parent): state_(std::move(state)) {
    cost_to_root_ = std::numeric_limits<double>::infinity();
    parent_index_ = -1;
}

Eigen::VectorXd TreeNode::getStateVlaue() const {
    return state_->getValue();
}

void TreeNode::setCost(double cost) {
    cost_to_root_ = cost;
}

double TreeNode::getCost() const {
    return cost_to_root_;
}

void TreeNode::setParentIndex(int index) {
    parent_index_ = index;
}

int TreeNode::getParentIndex() const {
    return parent_index_;
}

void TreeNode::setChildrenIndex(int index) {
    children_indices_.push_back(index);
}

std::vector<int> TreeNode::getChildrenIndices() const {
    return children_indices_;
}