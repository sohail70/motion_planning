#pragma once

#include "motion_planning/state_space/state.hpp"
#include <vector>
#include <unordered_map>
#include <memory>
#include "motion_planning/ds/node.hpp"
#include "boost/container/flat_map.hpp"

struct EdgeInfo {
    double distance;
    bool is_initial;  // True = persistent (N0), False = temporary (Nr)
};


class RRTxNode : public Node {
public:
    explicit RRTxNode(std::unique_ptr<State> state,int index = -1)
        : state_(std::move(state)),
          parent_(nullptr),
          index_(index),
          in_queue_(false),
          lmc_(INFINITY),
          cost_(INFINITY) {}

    // Core functionality
    Eigen::VectorXd getStateVlaue() const override { return state_->getValue(); }
    void setCost(double cost) noexcept override { cost_ = cost; }
    double getCost() const noexcept override { return cost_; }
    void setLMC(double lmc) noexcept override { lmc_ = lmc; }
    double getLMC() const noexcept override { return lmc_; }

    // Unified neighbor access
    auto& incomingEdges() noexcept { return incoming_edges_; }
    auto& outgoingEdges() noexcept { return outgoing_edges_; }
    
    // Modified addNeighbor with directional edge types
    void addNeighbor(RRTxNode* neighbor, bool is_outgoing_initial, bool is_incoming_initial, double dist) {
        outgoing_edges_[neighbor] = {dist, is_outgoing_initial};
        neighbor->incoming_edges_[this] = {dist, is_incoming_initial};
    }

    // void removeNeighbor(RRTxNode* neighbor) {
    //     outgoing_edges_.erase(neighbor);
    //     neighbor->incoming_edges_.erase(this);
    // }

    // Fixed setParent implementation
    void setParent(RRTxNode* parent, double edge_dist) {
        if (parent_) {
            auto& succ = parent_->successors_;
            succ.erase(std::remove(succ.begin(), succ.end(), this), succ.end());
            // parent_->outgoingEdges().erase(this);
            // incomingEdges().erase(parent_);
        }
        
        parent_ = parent;
        if (parent) {
            parent->successors_.push_back(this);
            // Add persistent bidirectional parent-child edges
            // parent->outgoingEdges()[this] = {edge_dist, true};
            // incomingEdges()[parent] = {edge_dist, true};
        }
    }

    // Successor access
    const std::vector<RRTxNode*>& successors() const noexcept { return successors_; }

    std::vector<RRTxNode*>& successorsMutable() noexcept { return successors_; }
    ///////////////////////////////////
        // Add missing implementations
    void setParentIndex(int index) override { /* Implement or mark final */ }
    int getParentIndex() const override { return -1; /* Implement properly */ }
    void setChildrenIndex(int index) override { /* Implement */ }
    std::vector<int>& getChildrenIndices() override { 
        static std::vector<int> dummy; return dummy; // Proper implementation needed
    }
    void setIndex(int index) noexcept { index_ = index; }
    int getIndex() const noexcept { return index_; }
    // Add missing parent accessor
    RRTxNode* getParent() const { return parent_; }
private:

    std::unique_ptr<State> state_;
    
    // std::unordered_map<RRTxNode*, EdgeInfo> incoming_edges_;
    // std::unordered_map<RRTxNode*, EdgeInfo> outgoing_edges_;
    boost::container::flat_map<RRTxNode*, EdgeInfo> incoming_edges_;
    boost::container::flat_map<RRTxNode*, EdgeInfo> outgoing_edges_;
    RRTxNode* parent_ = nullptr;
    bool in_queue_;
    double lmc_;
    double cost_;
    int index_;



    std::vector<RRTxNode*> successors_; // Successor nodes
    double parent_edge_dist_;

};

