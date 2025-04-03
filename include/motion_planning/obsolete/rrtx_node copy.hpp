#pragma once

#include "motion_planning/state_space/state.hpp"
#include <vector>
#include <unordered_map>
#include <memory>
// #include "motion_planning/ds/node.hpp"
#include "boost/container/flat_map.hpp"
#include "motion_planning/ds/edge_info.hpp"


class RRTxNode  {
public:
    explicit RRTxNode(std::unique_ptr<State> state,int index = -1)
        : state_(std::move(state)),
          parent_(nullptr),
          index_(index),
          in_queue_(false),
          heap_index_(-1),
          lmc_(INFINITY),
          cost_(INFINITY) {}

    const Eigen::VectorXd& getStateValue() const { return state_->getValue(); }
    void setCost(double cost) noexcept { cost_ = cost; }
    double getCost() const noexcept { return cost_; }
    void setLMC(double lmc) noexcept { lmc_ = lmc; }
    double getLMC() const noexcept { return lmc_; }

    /*
        Incoming are "-" neighbor and outgoing are "+" neighbors
        well i thought sometime to separate all 4 types but i guess its no need. 
        The only place we have burden i guess(but negligable) is the cullNeighbor that we have to find specific
        node to cull! its negliglbe because number of neighbors are not alot and cull it selfs culls so that we wouldn't have so many neighbor
    */
    auto& incomingEdges() noexcept { return incoming_edges_; }
    auto& outgoingEdges() noexcept { return outgoing_edges_; }
    
    /*
        a->b    : mean b is outgoing neighbor of a, and a is incoming neighbor of b
    */
    void addNeighbor(RRTxNode* neighbor, bool is_outgoing_initial, bool is_incoming_initial, double dist) {
        outgoing_edges_[neighbor] = {dist,dist ,is_outgoing_initial};
        neighbor->incoming_edges_[this] = {dist,dist ,is_incoming_initial};
    }


    void setParent(RRTxNode* parent, double edge_dist) {
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


    void disconnectFromGraph() {
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


    const std::vector<RRTxNode*>& getChildren() const noexcept { return children_; }
    // std::vector<RRTxNode*>& successorsMutable() noexcept { return children_; }

    void setIndex(int index) noexcept { index_ = index; }
    int getIndex() const noexcept { return index_; }
    RRTxNode* getParent() const { return parent_; }


    bool in_queue_;
    size_t heap_index_;  // Tracks position in the priority queue

    
private:

    std::unique_ptr<State> state_;
    
    // std::unordered_map<RRTxNode*, EdgeInfo> incoming_edges_;
    // std::unordered_map<RRTxNode*, EdgeInfo> outgoing_edges_;
    boost::container::flat_map<RRTxNode*, EdgeInfo> incoming_edges_;
    boost::container::flat_map<RRTxNode*, EdgeInfo> outgoing_edges_;
    RRTxNode* parent_ = nullptr;
    double lmc_;
    double cost_;
    int index_;

    std::vector<RRTxNode*> children_; // Successor nodes
    double parent_edge_dist_;

    // // Add this helper function
    // static bool hasChild(const std::vector<RRTxNode*>& successors, RRTxNode* child) {
    //     return std::find(successors.begin(), successors.end(), child) != successors.end();
    // }

};

