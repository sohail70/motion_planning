// Copyright 2025 Soheil E.nia

#include"motion_planning/ds/fmt_node.hpp"
#include "motion_planning/ds/fmt_node.hpp"

FMTNode::FMTNode(std::shared_ptr<State> state, int index)
    : state_(state),
      index_(index),
      cost_(INFINITY),
      time_to_goal_(INFINITY),
      heuristic_(0.0),
      in_queue_(false),
      heap_index_(-1),
      in_unvisited_(false),
      parent_(nullptr) {

        // For min-snap
        const int num_axes = 4; // Or get from StateSpace
        final_velocity_ = Eigen::VectorXd::Zero(num_axes);
        final_acceleration_ = Eigen::VectorXd::Zero(num_axes);
      }

const Eigen::VectorXd& FMTNode::getStateValue() const { 
    return state_->getValue(); 
}

// double FMTNode::getCost() const noexcept { 
//     return cost_; 
// }

void FMTNode::setCost(double cost) noexcept { 
    cost_ = cost; 
}

FMTNode::NeighborMap& FMTNode::neighbors() noexcept { 
    return neighbors_; 
}

const FMTNode::NeighborMap& FMTNode::neighbors() const noexcept { 
    return neighbors_; 
}

void FMTNode::setParent(FMTNode* parent, double edge_cost) {
    // Early exit if parent is the same
    if (parent == parent_) { 
        edge_cost_ = edge_cost; // Update cost even if parent is same
        return;
    }
    // If parent has changed remove this node from its old parent's children list
    if(parent_ && parent_ != parent) {
        auto& childs = parent_->children_;
        childs.erase(std::remove(childs.begin(), childs.end(), this), childs.end());
    }
    
    parent_ = parent;
    edge_cost_ = edge_cost;
    
    // Add this node to the new parent's children list
    if(parent_ ){ //&& !hasChild(this, parent_->children_)) {
        parent_->children_.push_back(this);
    }
}


void FMTNode::setParent(FMTNode* parent, const Trajectory& trajectory_to_parent) {
    // Early exit if parent is the same
    if (parent == parent_) { 
        return;
    }
    // If parent has changed remove this node from its old parent's children list
    if(parent_ && parent_ != parent) {
        auto& childs = parent_->children_;
        childs.erase(std::remove(childs.begin(), childs.end(), this), childs.end());
    }
    
    parent_ = parent;
    
    // Add this node to the new parent's children list
    if(parent_ ){ //&& !hasChild(this, parent_->children_)) {
        parent_->children_.push_back(this);
        parent_trajectory_ = trajectory_to_parent;

    }
}



const Trajectory& FMTNode::getParentTrajectory() const {
    return parent_trajectory_;
}

void FMTNode::disconnectFromGraph() {
    // Break parent link
    if (parent_ != nullptr) {
        auto& siblings = parent_->children_;
        siblings.erase(std::remove(siblings.begin(), siblings.end(), this), siblings.end());
        parent_ = nullptr;
    }
    
    // Break child links
    for (FMTNode* child : children_) {
        if (child && child->parent_ == this) {
            child->parent_ = nullptr;
        }
    }
    children_.clear();
}

void FMTNode::sanityCheck() const {
    if (in_unvisited_ && in_queue_) {
        std::cerr << "Warning: Node " << index_ 
                  << " has both in_unvisited_ and in_queue_ set to true!" << std::endl;
    }
}

FMTNode* FMTNode::getParent() const noexcept { 
    return parent_; 
}

const std::vector<FMTNode*>& FMTNode::getChildren() const noexcept { 
    return children_; 
}

std::vector<FMTNode*>& FMTNode::getChildrenMutable() noexcept { 
    return children_; 
}

void FMTNode::setIndex(int index) noexcept { 
    index_ = index; 
}

int FMTNode::getIndex() const noexcept { 
    return index_; 
}


double FMTNode::getHeuristic() const { return heuristic_; }
void FMTNode::cacheHeuristic(double h) { 
    heuristic_ = h;
    heuristic_cached_ = true;
}
bool FMTNode::isHeuristicCached() const { return heuristic_cached_; }


double FMTNode::getTimeToGoal() const noexcept { return time_to_goal_; }
void FMTNode::setTimeToGoal(double time) noexcept { time_to_goal_ = time; }

void FMTNode::setFinalDerivatives(const Eigen::VectorXd& vel, const Eigen::VectorXd& accel) {
    final_velocity_ = vel;
    final_acceleration_ = accel;
}
const Eigen::VectorXd& FMTNode::getFinalVelocity() const {
    return final_velocity_;
}
const Eigen::VectorXd& FMTNode::getFinalAcceleration() const {
    return final_acceleration_;
}