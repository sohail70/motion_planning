#include "motion_planning/ds/ifmt_node.hpp"

IFMTNode::IFMTNode(std::shared_ptr<State> state, int index)
    : state_(state),
      index_(index),
      cost_(INFINITY),
      heuristic_(0.0),
      in_queue_(false),
      in_samples_(false),
      is_new_(false),
      unexpand_(false),
      is_connected_(false),
      samples_index_(-1),
      heap_index_(-1),
      in_unvisited_(false),
      parent_(std::weak_ptr<IFMTNode>()),
      edge_cost_(0.0) {

        state_value_ = state->getValue();
      }

const Eigen::VectorXd& IFMTNode::getStateValue() const {
    // if (state_.expired()) { // Check if expired first
    //     throw std::runtime_error("State expired before locking");
    // }
    // auto state = state_.lock();
    // if (!state || state.use_count() == 0) {
    //     throw std::runtime_error("Invalid state access");
    // }
    // return state->getValue();
    return state_value_;
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

    this->samples_index_ = -1;
    this->in_samples_ = false;
    this->unexpand_ = false;
    this->in_queue_ = false;
    this->cost_ = INFINITY;


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

double IFMTNode::getGHat() const { return g_hat_; }
void IFMTNode::cacheGHat(double g_hat) { 
    g_hat_ = g_hat;
    g_hat_cached_ = true;
}


bool IFMTNode::isHeuristicCached() const { return heuristic_cached_; }



void IFMTNode::updateCostAndPropagate() {
    if (auto parent = parent_.lock()) {
        // New cost is parent's cost + edge cost to parent
        double newCost = parent->getCost() + edge_cost_;
        if (newCost < cost_) {
            cost_ = newCost;
            // Recursively update children
            for (auto& child_weak : children_) {
                if (auto child = child_weak.lock()) {
                    child->updateCostAndPropagate();
                }
            }
        }
    }
}