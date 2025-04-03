#include "motion_planning/ds/rrtx_node.hpp"

RRTxNode::RRTxNode(std::unique_ptr<State> state, int index)
    : state_(std::move(state)),
      parent_(nullptr),
      index_(index),
      in_queue_(false),
      heap_index_(-1),
      lmc_(INFINITY),
      cost_(INFINITY) {}

const Eigen::VectorXd& RRTxNode::getStateValue() const {
    return state_->getValue();
}

void RRTxNode::setCost(double cost) noexcept {
    cost_ = cost;
}

double RRTxNode::getCost() const noexcept {
    return cost_;
}

void RRTxNode::setLMC(double lmc) noexcept {
    lmc_ = lmc;
}

double RRTxNode::getLMC() const noexcept {
    return lmc_;
}

// boost::container::flat_map<RRTxNode*, EdgeInfo>& RRTxNode::incomingEdges() noexcept {
//     return incoming_edges_;
// }

// boost::container::flat_map<RRTxNode*, EdgeInfo>& RRTxNode::outgoingEdges() noexcept {
//     return outgoing_edges_;
// }


RRTxNode::EdgeMap& RRTxNode::incomingEdges() noexcept {
    return incoming_edges_;
}

const RRTxNode::EdgeMap& RRTxNode::incomingEdges() const noexcept {
    return incoming_edges_;
}

RRTxNode::EdgeMap& RRTxNode::outgoingEdges() noexcept {
    return outgoing_edges_;
}

const RRTxNode::EdgeMap& RRTxNode::outgoingEdges() const noexcept {
    return outgoing_edges_;
}


void RRTxNode::addNeighbor(RRTxNode* neighbor, bool is_outgoing_initial, bool is_incoming_initial, double dist) {
    outgoing_edges_[neighbor] = {dist, dist, is_outgoing_initial};
    neighbor->incoming_edges_[this] = {dist, dist, is_incoming_initial};
}

void RRTxNode::setParent(RRTxNode* parent, double edge_dist) {
    // ??? This was necessary for fmtx but i don't think its necessary for rrtx 
    if (parent == parent_) { 
        return;
    }
    if (parent_) {
        auto& succ = parent_->children_;
        succ.erase(std::remove(succ.begin(), succ.end(), this), succ.end());
    }
    
    parent_ = parent;
    if (parent ){ //&& !hasChild(parent->children_, this)) { // Check added
        parent->children_.push_back(this);
    }
}

void RRTxNode::disconnectFromGraph() {
    // 1. Clear parent relationship
    if (parent_ != nullptr) {
        auto& parent_children = parent_->children_;
        parent_children.erase(
            std::remove(parent_children.begin(), parent_children.end(), this),
            parent_children.end()
        );
        parent_ = nullptr;
    }

    // 2. Clear successor relationships
    for (RRTxNode* child : children_) {
        if (child && child->parent_ == this) {
            child->parent_ = nullptr;
        }
    }
    children_.clear();

    // 3. Clear edge relationships (both directions)
    for (auto& [neighbor, _] : outgoing_edges_) {
        if (neighbor) {
            neighbor->incoming_edges_.erase(this);
        }
    }

    for (auto& [neighbor, _] : incoming_edges_) {
        if (neighbor) {
            neighbor->outgoing_edges_.erase(this);
        }
    }

    incoming_edges_.clear();
    outgoing_edges_.clear();
}

const std::vector<RRTxNode*>& RRTxNode::getChildren() const noexcept {
    return children_;
}

void RRTxNode::setIndex(int index) noexcept {
    index_ = index;
}

int RRTxNode::getIndex() const noexcept {
    return index_;
}

RRTxNode* RRTxNode::getParent() const {
    return parent_;
}