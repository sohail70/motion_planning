#pragma once

#include "motion_planning/state_space/state.hpp"
#include "motion_planning/pch.hpp"
#include "motion_planning/ds/edge_info.hpp"
#include <memory>





class IFMTNode : public std::enable_shared_from_this<IFMTNode> {
public:
    using NeighborMap = boost::container::flat_map<std::shared_ptr<IFMTNode>, EdgeInfo>;
    
    explicit IFMTNode(std::shared_ptr<State> state, int index = -1);
    
    const Eigen::VectorXd& getStateValue() const;
    double getCost() const noexcept;
    void setCost(double cost) noexcept;

    NeighborMap& neighbors() noexcept;
    const NeighborMap& neighbors() const noexcept;

    void setParent(std::shared_ptr<IFMTNode> parent, double edge_cost);
    void disconnectFromGraph();
    void sanityCheck() const;

    std::shared_ptr<IFMTNode> getParent() const noexcept;
    const std::vector<std::weak_ptr<IFMTNode>>& getChildren() const noexcept;
    std::vector<std::weak_ptr<IFMTNode>>& getChildrenMutable() noexcept;

    void setIndex(int index) noexcept;
    int getIndex() const noexcept;

    double getHeuristic() const;
    void cacheHeuristic(double h);
    bool isHeuristicCached() const;

    double getGHat() const;
    void cacheGHat(double g_hat);

    void updateCostAndPropagate();

    size_t getUniqueId() const { 
        return reinterpret_cast<size_t>(this); 
    }

    bool in_queue_;
    size_t heap_index_;
    bool in_unvisited_;
    bool in_samples_;
    bool is_new_;
    bool unexpand_;
    bool is_connected_;
    int samples_index_;
    double edge_cost_;
    std::unordered_set<std::shared_ptr<IFMTNode>> blocked_best_neighbors;


private:
    // std::shared_ptr<State> state_;
    std::weak_ptr<State> state_;
    Eigen::VectorXd state_value_;
    NeighborMap neighbors_;
    double cost_;
    int index_;
    std::weak_ptr<IFMTNode> parent_;
    std::vector<std::weak_ptr<IFMTNode>> children_;
    double heuristic_;
    bool heuristic_cached_ = false;
    bool g_hat_cached_ = false;
    double g_hat_;
};
struct IEdgeCandidate {
    double estimated_cost;
    std::shared_ptr<IFMTNode> from;
    std::shared_ptr<IFMTNode> to;
    double distance;
    
    IEdgeCandidate(double cost, std::shared_ptr<IFMTNode> f, std::shared_ptr<IFMTNode> t, double d) 
        : estimated_cost(cost), from(std::move(f)), to(std::move(t)), distance(d) {}
    
    bool operator>(const IEdgeCandidate& other) const {
        return estimated_cost > other.estimated_cost;
    }
};

namespace std {
    template <>
    struct hash<IFMTNode> {
        size_t operator()(const IFMTNode& node) const noexcept {
            return std::hash<size_t>{}(node.getUniqueId());
        }
    };
}