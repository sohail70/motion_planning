// Copyright Soheil E.nia 2025
#pragma once

#include "motion_planning/state_space/state.hpp"
#include "motion_planning/pch.hpp"
#include "motion_planning/ds/edge_info.hpp"
#include <memory>

class BITNode : public std::enable_shared_from_this<BITNode> {
public:
    enum NodeStatus { UNPROCESSED, IN_QUEUE, CONNECTED };
    NodeStatus status = UNPROCESSED;
    
    explicit BITNode(std::shared_ptr<State> state, int index = -1);

    const Eigen::VectorXd& getStateValue() const;
    double getCost() const noexcept;
    void setCost(double cost) noexcept;

    void setParent(std::shared_ptr<BITNode> parent, double edge_cost);
    void disconnectFromGraph();
    void sanityCheck() const;

    std::shared_ptr<BITNode> getParent() const noexcept;
    const std::vector<std::shared_ptr<BITNode>>& getChildren() const noexcept;
    std::vector<std::shared_ptr<BITNode>>& getChildrenMutable() noexcept;
    
    void setIndex(int index) noexcept;
    int getIndex() const noexcept;

    double getHeuristic() const;
    void cacheHeuristic(double h);
    bool isHeuristicCached() const;

    double getGHat() const;
    void cacheGHat(double g_hat);
    void updateCostAndPropagate();

    bool in_queue_;
    bool in_samples_;
    bool is_new_;
    bool unexpand_;
    int samples_index_;

    bool is_pruned_;
    size_t heap_index_;  // Tracks position in the priority queue
    double edge_cost_;
    std::unordered_set<int> blocked_best_neighbors;

private:
    std::shared_ptr<State> state_;
    std::vector<std::shared_ptr<BITNode>> children_;
    std::weak_ptr<BITNode> parent_;  // Use weak_ptr to break circular references
    double cost_;
    int index_;
    bool on_obstacle;
    double heuristic_;
    bool heuristic_cached_ = false;
    bool g_hat_cached_ = false;
    double g_hat_;
};

struct EdgeCandidate {
    double estimated_cost;
    std::shared_ptr<BITNode> from;
    std::shared_ptr<BITNode> to;
    
    EdgeCandidate(double cost, std::shared_ptr<BITNode> f, std::shared_ptr<BITNode> t) 
        : estimated_cost(cost), from(std::move(f)), to(std::move(t)) {}
    
    bool operator>(const EdgeCandidate& other) const {
        return estimated_cost > other.estimated_cost;
    }
};

/*
📦 std::priority_queue logic with Compare:
std::priority_queue<T, Container, Compare>
This treats Compare(a, b) as:
"Should a come after b in the heap?"
Or put differently:
"a is less prioritized than b if Compare(a, b) returns true"
This is inverted logic, and it’s why it gets confusing.
*/