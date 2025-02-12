// Copyright 2025 Soheil E.nia

#include "motion_planning/ds/tree_node.hpp"

TreeNode::TreeNode(std::unique_ptr<State> state, std::shared_ptr<Node> parent): state_(std::move(state)) {

}

Eigen::VectorXd TreeNode::getStateVlaue() const {
    return state_->getValue();
}

double TreeNode::getCost() const {

}

std::shared_ptr<Node> TreeNode::getParent() const {

}

void TreeNode::setParent(std::shared_ptr<Node> parent) {

}