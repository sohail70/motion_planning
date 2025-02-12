// Copyright 2025 Soheil E.nia

#include "motion_planning/ds/tree_node.hpp"

TreeNode::TreeNode(std::unique_ptr<State> state, std::shared_ptr<Node> parent) {

}

const State& TreeNode::getState() const {

}

double TreeNode::getCost() const {

}

std::shared_ptr<Node> TreeNode::getParent() const {

}

void TreeNode::setParent(std::shared_ptr<Node> parent) {

}