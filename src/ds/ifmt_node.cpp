#include "motion_planning/ds/ifmt_node.hpp"

IFMTNode::IFMTNode(std::shared_ptr<State> state, int index)
    : state_(std::move(state)),
      index_(index),
      cost_(INFINITY),
      heuristic_(0.0),
      in_queue_(false),
      heap_index_(-1),
      in_unvisited_(false),
      parent_(std::weak_ptr<IFMTNode>()),
      edge_cost_(0.0) {}

const Eigen::VectorXd& IFMTNode::getStateValue() const { 
    return state_->getValue(); 
}

double IFMTNode::getCost() const noexcept { 
    return cost_; 
}

void IFMTNode::setCost(double cost) noexcept { 
    cost_ = cost; 
}

IFMTNode::NeighborMap& IFMTNode::neighbors() noexcept { 
    return neighbors_; 
}

const IFMTNode::NeighborMap& IFMTNode::neighbors() const noexcept { 
    return neighbors_; 
}

void IFMTNode::setParent(std::shared_ptr<IFMTNode> parent, double edge_cost) {
    auto current_parent = parent_.lock();
    if (current_parent == parent) { 
        edge_cost_ = edge_cost;
        return;
    }

    if (current_parent) {
        auto& children = current_parent->children_;
        children.erase(std::remove_if(children.begin(), children.end(),
            [this](const std::weak_ptr<IFMTNode>& child) {
                auto child_ptr = child.lock();
                return child_ptr && child_ptr->getIndex() == this->getIndex();
            }), children.end());
    }
    
    parent_ = parent;
    edge_cost_ = edge_cost;
    
    if (parent) {
        parent->children_.emplace_back(weak_from_this());
    }
}

void IFMTNode::disconnectFromGraph() {
    auto parent = parent_.lock();
    if (parent) {
        auto& siblings = parent->children_;
        siblings.erase(std::remove_if(siblings.begin(), siblings.end(),
            [this](const std::weak_ptr<IFMTNode>& child) {
                auto child_ptr = child.lock();
                return child_ptr && child_ptr->getIndex() == this->getIndex();
            }), siblings.end());
        parent_.reset();
    }

    for (auto& child_weak : children_) {
        if (auto child = child_weak.lock()) {
            if (child->parent_.lock().get() == this) {
                child->parent_.reset();
            }
        }
    }
    children_.clear();
}

std::shared_ptr<IFMTNode> IFMTNode::getParent() const noexcept { 
    return parent_.lock(); 
}

const std::vector<std::weak_ptr<IFMTNode>>& IFMTNode::getChildren() const noexcept { 
    return children_; 
}

std::vector<std::weak_ptr<IFMTNode>>& IFMTNode::getChildrenMutable() noexcept { 
    return children_; 
}

void IFMTNode::setIndex(int index) noexcept { 
    index_ = index; 
}

int IFMTNode::getIndex() const noexcept { 
    return index_; 
}


double IFMTNode::getHeuristic() const { return heuristic_; }
void IFMTNode::cacheHeuristic(double h) { 
    heuristic_ = h;
    heuristic_cached_ = true;
}
bool IFMTNode::isHeuristicCached() const { return heuristic_cached_; }