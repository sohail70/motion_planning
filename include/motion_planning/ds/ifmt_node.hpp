// Copyright 2025 Soheil E.nia
#pragma once

#include "motion_planning/state_space/state.hpp"
#include "motion_planning/pch.hpp"
#include "motion_planning/ds/edge_info.hpp"
#include <memory>

class IFMTNode : public std::enable_shared_from_this<IFMTNode> {
public:
    using NeighborMap = boost::container::flat_map<std::shared_ptr<IFMTNode>, EdgeInfo>;
    struct NeighborInfo {
        std::shared_ptr<IFMTNode> node;
        EdgeInfo edge_;
        // double distance;
        // double distance_original;
        
        // For sorting comparison --> sorting only based on c+h and not involving g because for the duration of a batch, g could change but c+h is the same so we sort ony once per batch and then maybe update the fmin (g+c+h) because g has changed
        bool operator<(const NeighborInfo& other) const {
            return (edge_.distance + node->getHeuristic()) < 
                   (other.edge_.distance + other.node->getHeuristic());
        }
    };
    
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

    //----------------
    std::vector<NeighborInfo> neighbors_sorted_;
    size_t next_neighbor_idx_ = 0;
    double current_min_f_ = INFINITY;

    // // Implementation of key neighbor methods
    // inline void addNeighbor(std::shared_ptr<IFMTNode> node, double distance) {
    //     neighbors_sorted_.push_back({
    //         node, 
    //         distance, 
    //         distance,  // distance_original
    //     });
    //     // Mark for re-sort on next access
    //     next_neighbor_idx_ = 0;
    //     current_min_f_ = INFINITY;
    // }

    inline void addNeighbor(std::shared_ptr<IFMTNode> node, double distance) {
        // Prevent duplicates
        auto it = std::find_if(neighbors_sorted_.begin(), neighbors_sorted_.end(),
            [&node](const NeighborInfo& ni) { return ni.node == node; });
        
        if(it == neighbors_sorted_.end()) {
            neighbors_sorted_.push_back({node, distance, distance});
            // Insertion sort to maintain order (better for incremental additions)
            for(auto rit = neighbors_sorted_.rbegin(); rit != neighbors_sorted_.rend()-1; ++rit) {
                if(*rit < *(rit+1)) break;
                std::iter_swap(rit, rit+1);
            }
            updateCurrentMinF();
        }
    }



    inline void removeNeighbor(std::shared_ptr<IFMTNode> node) {
        neighbors_sorted_.erase(
            std::remove_if(neighbors_sorted_.begin(), neighbors_sorted_.end(),
                [&node](const NeighborInfo& info) {
                    return info.node == node;
                }),
            neighbors_sorted_.end());
        updateCurrentMinF();
    }

    inline void sortNeighbors() {
        std::sort(neighbors_sorted_.begin(), neighbors_sorted_.end());
        next_neighbor_idx_ = 0;
        updateCurrentMinF();
    }

    inline void updateCurrentMinF() {
        if (next_neighbor_idx_ < neighbors_sorted_.size()) {
            const auto& next = neighbors_sorted_[next_neighbor_idx_];
            current_min_f_ = cost_ + next.edge_.distance + next.node->getHeuristic();
        } else {
            if(index_ != 0)
                current_min_f_ = INFINITY;
        }
    }

    inline void advanceNeighbor() {
        if (next_neighbor_idx_ < neighbors_sorted_.size()) {
            ++next_neighbor_idx_;
            updateCurrentMinF();
        }
    }



    //---------------



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