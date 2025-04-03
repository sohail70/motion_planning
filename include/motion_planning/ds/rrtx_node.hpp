#pragma once

#include "motion_planning/state_space/state.hpp"
#include "motion_planning/ds/edge_info.hpp"
#include "motion_planning/pch.hpp"


class RRTxNode {
public:
    using EdgeMap = boost::container::flat_map<RRTxNode*, EdgeInfo>;

    explicit RRTxNode(std::unique_ptr<State> state, int index = -1);
    
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

    void setParent(RRTxNode* parent, double edge_dist);

    void disconnectFromGraph();

    const std::vector<RRTxNode*>& getChildren() const noexcept;
    // std::vector<RRTxNode*>& successorsMutable() noexcept { return children_; }

    void setIndex(int index) noexcept;
    int getIndex() const noexcept;
    RRTxNode* getParent() const;

    bool in_queue_;
    size_t heap_index_;  // Tracks position in the priority queue

private:
    std::unique_ptr<State> state_;
    
    // std::unordered_map<RRTxNode*, EdgeInfo> incoming_edges_;
    // std::unordered_map<RRTxNode*, EdgeInfo> outgoing_edges_;
    // boost::container::flat_map<RRTxNode*, EdgeInfo> incoming_edges_;
    // boost::container::flat_map<RRTxNode*, EdgeInfo> outgoing_edges_;
    EdgeMap incoming_edges_;
    EdgeMap outgoing_edges_;
    RRTxNode* parent_ = nullptr;
    double lmc_;
    double cost_;
    int index_;

    std::vector<RRTxNode*> children_; // Successor nodes
    double parent_edge_dist_;

    // // Add this helper function
    // static bool hasChild(const std::vector<RRTxNode*>& successors, RRTxNode* child) {
    //     return std::find(successors.begin(), successors.end(), child) != successors.end();
    // }
};