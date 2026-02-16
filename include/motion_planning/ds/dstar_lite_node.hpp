// motion_planning/ds/dstar_lite_node.hpp
#pragma once
#include "motion_planning/state_space/state.hpp"
#include "motion_planning/pch.hpp"
#include "motion_planning/ds/edge_info.hpp"
#include <Eigen/Dense>
#include <limits>

class DStarLiteNode {
public:
    using NeighborMap = boost::container::flat_map<DStarLiteNode*, EdgeInfo>;

    explicit DStarLiteNode(std::shared_ptr<State> state, int index = -1)
        : state_(state), index_(index),
        g(std::numeric_limits<double>::infinity()),
        rhs(std::numeric_limits<double>::infinity()),
        priority_key_(std::numeric_limits<double>::infinity()),
        heap_index_(-1), in_queue_(false),
        neighbors_cached_(false),
        best_parent_(nullptr),
        time_to_goal_(INFINITY),   // initialize time
        k1(std::numeric_limits<double>::infinity()),
        k2(std::numeric_limits<double>::infinity())
    {}

    // Accessors
    const Eigen::VectorXd& getStateValue() const { return state_->getValue(); }
    int getIndex() const noexcept { return index_; }
    
    // --- Additions for Path Extraction ---
    // Helper to get parent (matches FMT* API)
    DStarLiteNode* getParent() const { return best_parent_; }

    // Helper to get the trajectory to the parent (matches FMT* API)
    const Trajectory& getParentTrajectory() const {
        return best_parent_trajectory_;
    }

    void setBestParent(DStarLiteNode* p, const Trajectory& traj) {
        // if (best_parent_ == p) return;
        
        // // Remove from old parent's children list
        // if (best_parent_) {
        //     auto& siblings = best_parent_->children_;
        //     siblings.erase(std::remove(siblings.begin(), siblings.end(), this), siblings.end());
        // }
        
        best_parent_ = p;
        best_parent_trajectory_ = traj;
        
        // // Add to new parent's children list
        // if (best_parent_) {
        //     best_parent_->children_.push_back(this);
        // }
    }



    double getTimeToGoal() const noexcept { return time_to_goal_; }
    void setTimeToGoal(double time) noexcept { time_to_goal_ = time; }
    // D* Lite Data
    double g;
    double rhs;
    double priority_key_; 
    
    // Graph Connectivity
    NeighborMap forward_neighbors_;
    NeighborMap backward_neighbors_;
    bool neighbors_cached_; 
    
    // Priority Queue Helpers
    size_t heap_index_;
    bool in_queue_;

    double time_to_goal_;

    double k1;
    double k2;

    // Members for caching the optimal parent (updated in updateVertex)
    DStarLiteNode* best_parent_; 
    Trajectory best_parent_trajectory_;
    // std::vector<DStarLiteNode*> children_;

private:
    std::shared_ptr<State> state_;
    int index_;
};