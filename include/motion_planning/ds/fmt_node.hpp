// Copyright Soheil E.nia 2025
#pragma once

#include "motion_planning/state_space/state.hpp"
#include "motion_planning/pch.hpp"
#include "motion_planning/ds/edge_info.hpp"

class FMTNode {
public:
    /*
        Why flat_map? it worked better than std::unordered_map --> you can also uncomment the unordered_map to test.
        Reasons: 1) Lots of looping over neighbors in FMTX
                 2) The number of neighbors are limited because we pre-sample in FMTx so cost of insertion which is log(n) in flat map due to sorting is negligble
    */
    using NeighborMap = boost::container::flat_map<FMTNode*, EdgeInfo>;
    // using NeighborMap = std::unordered_map<FMTNode*, EdgeInfo>;
    
    explicit FMTNode(std::shared_ptr<State> state, int index = -1);
    
    const Eigen::VectorXd& getStateValue() const;
    // double getCost() const noexcept;
    inline double getCost() const { 
        return cost_; 
    }
    void setCost(double cost) noexcept;

    NeighborMap& neighbors() noexcept;
    const NeighborMap& neighbors() const noexcept;

    /*
        we set parent and manage children at the same time 
        with the added condition in FMTx, i.e.,  " ... || x->getCost() > (z->getCost() + cost_to_neighbor.distance" which is basically enables rewiring,
        one node may have rewired into the same parent! this seems weird but the cost of that parent is infact reduced and thats why you reached to last phase of
        the fmtx to setParent function, so we are here in setParent redundantly and if we do not early exit we have to use the hasChild below or else we
        are gonna end up using "parent_children_.push_back(this)" alot!
    */
    void setParent(FMTNode* parent, double edge_cost);
    
    void setParent(FMTNode* parent, const Trajectory& trajectory_to_parent);

    // I used raw pointers for speed and I use this in clear function inside setup() to be sure!
    void disconnectFromGraph();

    /*
        Well my rule is that nodes shouldn't be at both in_unvisited and in_queue and we should decide before going into plan() function --> but how about your added condition 
        " ... || x->getCost() > (z->getCost() + cost_to_neighbor.distance" isn't this somehow behind the scene not checking if a node is in vopen and just decides to treat it as v unvisited? Yes, but!

        every unvisted then becomes v open but because v univsted does have inf cost its gonna cause problem if its on vopen because best_neighbor is gonna get selected out of vopens who has the best cost 
        and imagine at some point we only have inf costs in out vopens!
        at the same time vopen pops from lowest cost so if its downstream it shoulnt be a problem
        even though i explicitly don't have nodes 

        I don't use the following function anywhere now but you can put it in plan() function to see whats what
    */
    void sanityCheck() const;

    FMTNode* getParent() const noexcept;
    const std::vector<FMTNode*>& getChildren() const noexcept;
    std::vector<FMTNode*>& getChildrenMutable() noexcept;
    
    void setIndex(int index) noexcept;
    int getIndex() const noexcept;

    double getHeuristic() const;
    void cacheHeuristic(double h);
    bool isHeuristicCached() const;


    double getTimeToGoal() const noexcept;
    void setTimeToGoal(double time) noexcept;



    // NEW: Distinct accessors for forward and backward neighbors
    NeighborMap& forwardNeighbors() noexcept { return forward_neighbors_; }
    const NeighborMap& forwardNeighbors() const noexcept { return forward_neighbors_; }
    
    NeighborMap& backwardNeighbors() noexcept { return backward_neighbors_; }
    const NeighborMap& backwardNeighbors() const noexcept { return backward_neighbors_; }

    bool isNeighborsCached() const { return neighbors_cached_; }

    const Trajectory& getParentTrajectory() const;



    bool in_queue_;
    size_t heap_index_;  // Tracks position in the priority queue

    bool in_unvisited_;
    double edge_cost_;
    /*
        blocked_best_neighbor: one thing about FMT is it exhausts the z nodes as it goes and in some bad cases near obstacles 
        we keep getting the same best_neighbor out of the best_neighbor procedure and that x node's edge to that best_neighbor
        is in obstacle and there is nothing we can do about it. the algorithm goes forward to pop another z node from heap and we keep having
        the same problem for that x node and the best neighbor node that we calculate
        this is not a problem for dijktra like fast marching tree even though it causes sub-optimal solutions in low sample case and it is mentioned in 
        fmt paper also but if you want to use A-star like heuristic its gonna create a problem because z node pop procedure in dijkstra-like is breadth-first
        and in the end that blocked_best_neighbor node will end up poped from the vopen heap but in case of A-star like which is depth first it will not get poped easily
        and it will exhaust alot of z nodes (i mean heap nodes - expansion nodes if you will) so we need to exclude those from the best_neighbor candidate list!
        so we fill blocked_best_neighbors in the else part of the cost update in the main plan function
    */
    std::unordered_set<int> blocked_best_neighbors;
    std::vector<FMTNode*> children_;
    FMTNode* parent_;
    bool neighbors_cached_ = false;
    int bad_count = 0;
private:
    std::shared_ptr<State> state_;
    NeighborMap neighbors_;

    // REPLACED the single 'neighbors_' map
    NeighborMap forward_neighbors_;  // Nodes reachable FROM this node
    NeighborMap backward_neighbors_; // Nodes that can reach this node

    Trajectory parent_trajectory_; 


    double cost_; // This is optimization cost
    double time_to_goal_; // This is the pure accumulated time
    int index_;
    bool on_obstacle; // not using this now! maybe later instead of samples_in_obstalce!


    double heuristic_;
    bool heuristic_cached_ = false;
    // static bool hasChild(FMTNode* node, const std::vector<FMTNode*>& children) {
    //     return std::find(children.begin(), children.end(), node) != children.end();
    // }



};