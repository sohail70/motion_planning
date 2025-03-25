#pragma once

#include "motion_planning/state_space/state.hpp"
#include <vector>
#include <unordered_map>
#include <memory>
#include "motion_planning/ds/node.hpp"
#include "boost/container/flat_map.hpp"

struct EdgeInfo {
    double distance;
    double distance_original; // I use it in removeObstalce When i want to reset the distance
    bool is_initial;  // True = persistent (N0), False = temporary (Nr)
};


class RRTxNode : public Node {
public:
    explicit RRTxNode(std::unique_ptr<State> state,int index = -1)
        : state_(std::move(state)),
            // state_value_(static_cast<EuclideanState*>(state_.get())->getValue()), // Cache the value
          parent_(nullptr),
          index_(index),
          in_queue_(false),
          lmc_(INFINITY),
          cost_(INFINITY) {}

    // Core functionality
    const Eigen::VectorXd& getStateVlaue() const override { return state_->getValue(); }
    // const Eigen::VectorXd& getStateVlaue() const override { return state_value_;}
    void setCost(double cost) noexcept override { cost_ = cost; }
    double getCost() const noexcept override { return cost_; }
    void setLMC(double lmc) noexcept override { lmc_ = lmc; }
    double getLMC() const noexcept override { return lmc_; }

    // Unified neighbor access
    auto& incomingEdges() noexcept { return incoming_edges_; }
    auto& outgoingEdges() noexcept { return outgoing_edges_; }
    
    // Modified addNeighbor with directional edge types
    void addNeighbor(RRTxNode* neighbor, bool is_outgoing_initial, bool is_incoming_initial, double dist) {
        outgoing_edges_[neighbor] = {dist,dist ,is_outgoing_initial};
        neighbor->incoming_edges_[this] = {dist,dist ,is_incoming_initial};
    }

    // void removeNeighbor(RRTxNode* neighbor) {
    //     outgoing_edges_.erase(neighbor);
    //     neighbor->incoming_edges_.erase(this);
    // }

    // Fixed setParent implementation
    void setParent(RRTxNode* parent, double edge_dist) {
        // ??? This was necessary for fmtx but i don't think its necessary for rrtx 
        if (parent == parent_) { 
            return;
        }
        if (parent_) {
            auto& succ = parent_->successors_;
            succ.erase(std::remove(succ.begin(), succ.end(), this), succ.end());

        }
        
        parent_ = parent;
        if (parent ){ //&& !hasChild(parent->successors_, this)) { // Check added
            parent->successors_.push_back(this);
        }
    }


    void disconnectFromGraph() {
        // 1. Clear parent relationship
        if (parent_ != nullptr) {
            auto& parent_successors = parent_->successors_;
            parent_successors.erase(
                std::remove(parent_successors.begin(), parent_successors.end(), this),
                parent_successors.end()
            );
            parent_ = nullptr;
        }

        // 2. Clear successor relationships
        for (RRTxNode* successor : successors_) {
            if (successor && successor->parent_ == this) {
                successor->parent_ = nullptr;
            }
        }
        successors_.clear();

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

    // Eigen::VectorXd state_value_; // Cached state value

    std::vector<RRTxNode*> successors_; // Successor nodes
    double parent_edge_dist_;

    // Add this helper function
    static bool hasChild(const std::vector<RRTxNode*>& successors, RRTxNode* child) {
        return std::find(successors.begin(), successors.end(), child) != successors.end();
    }

};

