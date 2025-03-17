// fmtx_node.hpp
#pragma once

#include "motion_planning/state_space/state.hpp"
#include "motion_planning/ds/node.hpp"
#include <unordered_map>
#include <boost/container/flat_map.hpp>
#include <vector>

class FMTXNode : public Node {
public:
    using NeighborMap = boost::container::flat_map<FMTXNode*, double>;
    // using NeighborMap = std::unordered_map<FMTXNode*, double>;
    
    explicit FMTXNode(std::unique_ptr<State> state, int index = -1)
        : state_(std::move(state)),
          index_(index),
          cost_(INFINITY),
          in_queue_(false),
          in_unvisited_(false),
          parent_(nullptr) {}

    // Core interface
    const Eigen::VectorXd& getStateVlaue() const override { return state_->getValue(); }
    double getCost() const noexcept override { return cost_; }
    void setCost(double cost) noexcept override { cost_ = cost; }



    // Direct neighbor access
    NeighborMap& neighbors() noexcept { return neighbors_; }
    const NeighborMap& neighbors() const noexcept { return neighbors_; }


    // Parent-child management
    void setParent(FMTXNode* parent, double edge_cost) {
        // sometime the parent is the same but its cost has changed so we are here redundantly and if we do not early exist we have to use the hasChild below!!
        if (parent == parent_) { 
            edge_cost_ = edge_cost; // Update cost even if parent is same
            return;
        }
        if(parent_ && parent_ != parent) {
            // Remove from old parent's children
            auto& childs = parent_->children_;
            childs.erase(std::remove(childs.begin(), childs.end(), this), childs.end());
        }
        
        parent_ = parent;
        edge_cost_ = edge_cost;
        
        if(parent_ ){ //&& !hasChild(this, parent_->children_)) {
            parent_->children_.push_back(this);
        }
    }

    void sanityCheck() const {
        if (in_unvisited_ && in_queue_) {
            std::cerr << "Warning: Node " << index_ 
                      << " has both in_unvisited_ and in_queue_ set to true!" << std::endl;
        }
    }
// void setParent(FMTXNode* parent, double edge_cost) {
//     // Debug: Track calls
//     std::cout << "FMTXNode " << index_ 
//               << " setting parent to " << (parent ? std::to_string(parent->index_) : "nullptr") 
//               << std::endl;

//     if (parent_ && parent_ != parent) {
//         // Remove from old parent's children
//         auto& childs = parent_->children_;
//         childs.erase(std::remove(childs.begin(), childs.end(), this), childs.end());
//         std::cout << "  Removed from old parent " << parent_->index_ << "'s children." << std::endl;
//     }
    
//     parent_ = parent;
//     edge_cost_ = edge_cost;
    
//     if (parent_) {
//         // Check for duplicate before adding
//         auto& children = parent_->children_;
//         if (std::find(children.begin(), children.end(), this) == children.end()) {
//             children.push_back(this);
//             std::cout << "  Added to parent " << parent_->index_ << "'s children." << std::endl;
//         } else {
//             std::cout << "  Already in parent " << parent_->index_ << "'s children. Skipping." << std::endl;
//         }
//     }
// }


    FMTXNode* getParent() const noexcept { return parent_; }
    const std::vector<FMTXNode*>& getChildren() const noexcept { return children_; }
    std::vector<FMTXNode*>& getChildrenMutable() noexcept { return children_; }
    
    // Index management
    void setIndex(int index) noexcept { index_ = index; }
    int getIndex() const noexcept  { return index_; }

    // // Optimized memory layout
    // struct StateComparator {
    //     bool operator()(const FMTXNode* a, const FMTXNode* b) const {
    //         return a->state_->getValue().norm() < b->state_->getValue().norm();
    //     }
    // };



    // Add these implementations
    void setParentIndex(int index) override { parent_index_ = index; }
    int getParentIndex() const override { return parent_index_; }
    void setChildrenIndex(int index) override { children_indices_.push_back(index); }
    std::vector<int>& getChildrenIndices() override { return children_indices_; }
   // Add these implementations
   void setLMC(double lmc) override { /* FMTX doesn't use LMC */ }
   double getLMC() const override { return getCost(); }  // Map LMC to cost if needed

    bool in_queue_;
    bool in_unvisited_;
    double edge_cost_;
    std::unordered_set<int> blocked_best_neighbors;
private:
    std::unique_ptr<State> state_;
    NeighborMap neighbors_;         // neighbor node -> edge cost
    std::vector<FMTXNode*> children_; // direct children
    FMTXNode* parent_;              // direct parent
    double cost_;
    int index_;
    bool on_obstacle; // not using this now! maybelater instead of samples_in_obstalce!
    //////
        // Add these members
    int parent_index_ = -1;
    std::vector<int> children_indices_;

    static bool hasChild(FMTXNode* node, const std::vector<FMTXNode*>& children) {
        return std::find(children.begin(), children.end(), node) != children.end();
    }
};