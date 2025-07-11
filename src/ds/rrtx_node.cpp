#include "motion_planning/ds/rrtx_node.hpp"

RRTxNode::RRTxNode(std::shared_ptr<State> state, int index)
    : state_(state),
      parent_(nullptr),
      index_(index),
      in_queue_(false),
      heap_index_(-1),
      lmc_(INFINITY),
      cost_(INFINITY),
      time_to_goal_(INFINITY){}

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


// This function adds a single directed edge from 'this' object node to the 'target_node'.
void RRTxNode::addNeighbor(RRTxNode* target_node, bool is_outgoing_initial, bool is_incoming_initial, const Trajectory& trajectory) {

    // Create the EdgeInfo for the OUTGOING edge (this -> target_node)
    EdgeInfo outgoing_edge_info;
    outgoing_edge_info.distance = trajectory.cost; // Sorry for the bad name of distance! it shouldve been cost! or another better approach would be to add original_cost to the trajectory struct and handle this there!
    outgoing_edge_info.distance_original = trajectory.cost;
    outgoing_edge_info.is_initial = is_outgoing_initial; // Use the first boolean
    outgoing_edge_info.cached_trajectory = trajectory;
    outgoing_edge_info.is_trajectory_computed = true;

    // Add the edge to the current node's outgoing list.
    this->outgoing_edges_[target_node] = outgoing_edge_info;

    // Create the EdgeInfo for the corresponding INCOMING edge on the target.
    //    We can copy the data since the trajectory and cost are the same for this one-way link.
    EdgeInfo incoming_edge_info = outgoing_edge_info;
    // The 'is_initial' flag for the incoming edge is based on the target node's perspective.
    // In RRTx, when a new node 'v' connects to an old node 'u', the edge
    // v->u is initial for 'v', but the edge u<-v is a "running" or temporary edge for 'u'.
    incoming_edge_info.is_initial = is_incoming_initial;

    // 5. Add this edge to the target node's incoming list.
    target_node->incoming_edges_[this] = incoming_edge_info;
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


void RRTxNode::setParent(RRTxNode* parent, const Trajectory& trajectory_to_parent) {
    if (parent_ != parent) {
        // Your existing logic to remove from old parent's children list
        if (parent_) {
            auto& succ = parent_->children_;
            succ.erase(std::remove(succ.begin(), succ.end(), this), succ.end());
        }
        
        parent_ = parent;

        if (parent) {
            parent->children_.push_back(this);
        }
    }
    
    // Store the trajectory that leads to the new parent
    if (parent) {
        parent_trajectory_ = trajectory_to_parent;
    }
}
const Trajectory& RRTxNode::getParentTrajectory() const {
    return parent_trajectory_;
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

double RRTxNode::getTimeToGoal() const noexcept { return time_to_goal_; }
void RRTxNode::setTimeToGoal(double time) noexcept { time_to_goal_ = time; }