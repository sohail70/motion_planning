#include "motion_planning/ds/bit_node.hpp"

BITNode::BITNode(std::shared_ptr<State> state, int index)
    : state_(std::move(state)),
      index_(index),
      cost_(INFINITY),
      heuristic_(0.0),
      in_queue_(false),
      in_samples_(false),
      unexpand_(false),
      samples_index_(-1),
      is_pruned_(false),
      heap_index_(-1) {}

const Eigen::VectorXd& BITNode::getStateValue() const { 
    return state_->getValue(); 
}

double BITNode::getCost() const noexcept { 
    return cost_; 
}

void BITNode::setCost(double cost) noexcept { 
    cost_ = cost; 
}

void BITNode::setParent(std::shared_ptr<BITNode> parent, double edge_cost) {
    auto current_parent = parent_.lock();
    if (current_parent && current_parent == parent) {
        edge_cost_ = edge_cost;
        return;
    }

    if (current_parent) {
        auto& childs = current_parent->children_;
        childs.erase(std::remove(childs.begin(), childs.end(), shared_from_this()), childs.end());
    }
    
    parent_ = parent;
    edge_cost_ = edge_cost;
    
    if (parent) {
        parent->children_.push_back(shared_from_this());
    }
}

void BITNode::disconnectFromGraph() {
    if (auto parent = parent_.lock()) {
        auto& siblings = parent->children_;
        siblings.erase(std::remove(siblings.begin(), siblings.end(), shared_from_this()), siblings.end());
        parent_.reset();
    }
    
    // Fix: Remove .lock() since children_ stores shared_ptr directly
    for (auto& child : children_) {
        child->parent_.reset(); // Access directly via shared_ptr
    }
    children_.clear();
}

std::shared_ptr<BITNode> BITNode::getParent() const noexcept { 
    return parent_.lock(); 
}

const std::vector<std::shared_ptr<BITNode>>& BITNode::getChildren() const noexcept { 
    return children_; 
}

std::vector<std::shared_ptr<BITNode>>& BITNode::getChildrenMutable() noexcept { 
    return children_; 
}

void BITNode::setIndex(int index) noexcept { 
    index_ = index; 
}

int BITNode::getIndex() const noexcept { 
    return index_; 
}


double BITNode::getHeuristic() const { return heuristic_; }
void BITNode::cacheHeuristic(double h) { 
    heuristic_ = h;
    heuristic_cached_ = true;
}
bool BITNode::isHeuristicCached() const { return heuristic_cached_; }



void BITNode::updateCostAndPropagate() {
    if (auto parent = parent_.lock()) {
        // New cost is parent's cost + edge cost to parent
        double newCost = parent->getCost() + edge_cost_;
        if (newCost < cost_) {
            cost_ = newCost;
            // Recursively update children
            for (auto& child : children_) {
                child->updateCostAndPropagate();
            }
        }
    }
}