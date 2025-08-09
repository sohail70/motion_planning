// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/state_space/state.hpp"
#include "motion_planning/ds/edge_info.hpp"
#include "motion_planning/pch.hpp"

class RRTxNode {
public:
    using EdgeMap = boost::container::flat_map<RRTxNode*, EdgeInfo>;

    explicit RRTxNode(std::shared_ptr<State> state, int index = -1);
    
    const Eigen::VectorXd& getStateValue() const;
    void setCost(double cost) noexcept;
    double getCost() const noexcept;
    void setLMC(double lmc) noexcept;
    double getLMC() const noexcept;

    /*
        Incoming are "-" neighbor and outgoing are "+" neighbors
        well i thought sometime to separate all 4 types but i guess its no need. 
        The only place we have burden i guess(but negligable) is the cullNeighbor that we have to find specific
        node to cull! its negliglbe because number of neighbors are not alot and cull it selfs culls so that we wouldn't have so many neighbor

        example:
            When extend is called for the new node 65, it finds node 59 in its neighborhood. 
            It creates two directed edges to form a bidirectional link:
            Edge 65 -> 59: This is an edge from a new node to an old node.
            is_outgoing_initial (for node 65) is set to true.
            is_incoming_initial (for node 59) is set to false.
            Edge 59 -> 65: This is an edge from an old node to a new node.
            is_outgoing_initial (for node 59) is set to false.
            is_incoming_initial (for node 65) is set to true.
    */
    // auto& incomingEdges() noexcept;
    // auto& outgoingEdges() noexcept;
    EdgeMap& incomingEdges() noexcept;
    const EdgeMap& incomingEdges() const noexcept;
    EdgeMap& outgoingEdges() noexcept;
    const EdgeMap& outgoingEdges() const noexcept;    


    /*
        a->b    : mean b is outgoing neighbor of a, and a is incoming neighbor of b
    */
    void addNeighbor(RRTxNode* neighbor, bool is_outgoing_initial, bool is_incoming_initial, double dist);

    // overloaded addNeighbor function to add a single, one-way edge with its trajectory --> For kinodynamic case where we provide trajectory
    void addNeighbor(RRTxNode* target_node, bool is_outgoing_initial, bool is_incoming_initial ,const Trajectory& trajectory);



    void setParent(RRTxNode* parent, double edge_dist);
    void setParent(RRTxNode* parent, const Trajectory& trajectory_to_parent);

    void disconnectFromGraph();

    const std::vector<RRTxNode*>& getChildren() const noexcept;
    // std::vector<RRTxNode*>& successorsMutable() noexcept { return children_; }

    void setIndex(int index) noexcept;
    int getIndex() const noexcept;
    RRTxNode* getParent() const;
    double getTimeToGoal() const noexcept;
    void setTimeToGoal(double time) noexcept;
    const Trajectory& getParentTrajectory() const;

    bool in_queue_;
    size_t heap_index_;  // Tracks position in the priority queue
    int bad_count = 0;


private:
    std::shared_ptr<State> state_;
    
    // std::unordered_map<RRTxNode*, EdgeInfo> incoming_edges_;
    // std::unordered_map<RRTxNode*, EdgeInfo> outgoing_edges_;
    // boost::container::flat_map<RRTxNode*, EdgeInfo> incoming_edges_;
    // boost::container::flat_map<RRTxNode*, EdgeInfo> outgoing_edges_;
    EdgeMap incoming_edges_;
    EdgeMap outgoing_edges_;
    RRTxNode* parent_ = nullptr;
    Trajectory parent_trajectory_; 
    double lmc_;
    double cost_;
    int index_;
    double time_to_goal_; // This is the pure accumulated time
    std::vector<RRTxNode*> children_; // Successor nodes
    double parent_edge_dist_;

    // // Add this helper function
    // static bool hasChild(const std::vector<RRTxNode*>& successors, RRTxNode* child) {
    //     return std::find(successors.begin(), successors.end(), child) != successors.end();
    // }
};