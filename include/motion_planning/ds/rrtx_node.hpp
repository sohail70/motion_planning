#pragma once

#include "motion_planning/state_space/state.hpp"
#include <vector>
#include <unordered_map>
#include <memory>
#include "motion_planning/ds/node.hpp"
#include "boost/container/flat_map.hpp"


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

    // Neighbor accessors
    // Return non-const references for initial neighbors
    auto& initialIn() noexcept { return N0_in_; }
    auto& initialOut() noexcept { return N0_out_; }
        // Add non-const accessors
    auto& currentIn() { return Nr_in_; }
    auto& currentOut() { return Nr_out_; }
    std::vector<RRTxNode*>& successorsMutable() { return successors_; }


    /*
        If node v has node u as an outgoing neighbor (N0_out or Nr_out), then u must have v as an incoming neighbor (N0_in or Nr_in).
    */
    void addOriginalNeighbor(RRTxNode* neighbor, double dist) {
        // For the new node: add the existing node as an original neighbor
        N0_out_[neighbor] = dist;          // New → Existing (original outgoing)
        neighbor->Nr_in_[this] = dist;     // Existing ← New (running incoming)
    }

    void addRunningNeighbor(RRTxNode* neighbor, double dist) {
        // For the existing node: add the new node as a running neighbor
        Nr_out_[neighbor] = dist;          // Existing → New (running outgoing)
        neighbor->N0_in_[this] = dist;     // New ← Existing (original incoming)
    }


    void removeCurrentOut(RRTxNode* neighbor) {
        auto it = Nr_out_.find(neighbor);
        if (it != Nr_out_.end()) {
            neighbor->Nr_in_.erase(this);
            Nr_out_.erase(it);
        }
    }

    // Parent management
    // void setParent(RRTxNode* parent, double edge_dist) noexcept {
    //     parent_ = parent;
    //     parent_edge_dist_ = edge_dist;
    //     if (parent) parent->successors_.push_back(this);
    // }

    void setParent(RRTxNode* parent, double edge_dist) noexcept {
        // Add reference counting or use weak_ptr
        if(parent_) {
            // Remove from old parent's successors
            auto& succ = parent_->successors_;
            succ.erase(std::remove(succ.begin(), succ.end(), this), succ.end());
        }
        
        parent_ = parent;
        parent_edge_dist_ = edge_dist;
        
        if (parent) {
            parent->successors_.push_back(this);
        }
    }


    // Successor access
    const std::vector<RRTxNode*>& successors() const noexcept { return successors_; }


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
    
    // // Neighbor storage: [node pointer -> distance]
    // std::unordered_map<RRTxNode*, double> N0_in_;   // Initial incoming
    // std::unordered_map<RRTxNode*, double> N0_out_;  // Initial outgoing
    
    // // Current neighbors with reverse reference
    // std::unordered_map<RRTxNode*, double> Nr_out_;  // Current outgoing
    // std::unordered_map<RRTxNode*, double> Nr_in_;  // Current incoming

    boost::container::flat_map<RRTxNode*, double> N0_in_;  
    boost::container::flat_map<RRTxNode*, double> N0_out_;
    boost::container::flat_map<RRTxNode*, double> Nr_out_;
    boost::container::flat_map<RRTxNode*, double> Nr_in_;

    // // Unified neighbor storage with edge type tagging
    // std::unordered_map<RRTxNode*, EdgeInfo> incoming_edges_;  // N⁻ = N0⁻ ∪ Nr⁻
    // std::unordered_map<RRTxNode*, EdgeInfo> outgoing_edges_;   // N⁺ = N0⁺ ∪ Nr⁺

    // struct EdgeInfo {
    //     double distance;
    //     bool is_initial;  // True = persistent (N0), False = temporary (Nr)
    // };



    std::vector<RRTxNode*> successors_; // Successor nodes
    RRTxNode* parent_;
    double parent_edge_dist_;
    bool in_queue_;
    double lmc_;
    double cost_;
    int index_; 
};


// class RRTxEdge {
//  public:
//     RRTxEdge(RRTxNode* start, RRTxNode* end, double dist)
//         : start_(start), end_(end), distance_(dist) {}

//     double getDistance() const { return distance_; }
//     RRTxNode* getStart() const { return start_; }
//     RRTxNode* getEnd() const { return end_; }

//  private:
//     RRTxNode* start_;
//     RRTxNode* end_;
//     double distance_;
// };