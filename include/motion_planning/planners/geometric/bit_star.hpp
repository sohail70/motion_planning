// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/bit_node.hpp"
#include "motion_planning/utils/visualization.hpp"
#include "boost/container/flat_map.hpp"
#include "motion_planning/ds/priority_queue.hpp"

class BITStar : public Planner {
public:
    BITStar(std::shared_ptr<StateSpace> statespace, 
                   std::shared_ptr<ProblemDefinition> problem_def,
                   std::shared_ptr<ObstacleChecker> obs_checker);
    
    void setup(const Params& params, std::shared_ptr<Visualization> visualization) override;
    void plan() override;
    std::vector<std::shared_ptr<BITNode>> getPathNodes() const;

    std::vector<Eigen::VectorXd> getPathPositions() const;
    void setStart(const Eigen::VectorXd& start) override;
    void setGoal(const Eigen::VectorXd& goal) override;
    void setRobotIndex(const Eigen::VectorXd& robot_position);

    void near(int node_index);
    void near2sample( const std::shared_ptr<BITNode>& node, std::vector<std::shared_ptr<BITNode>>& near_nodes);
    void near2tree( const std::shared_ptr<BITNode>& node, std::vector<std::shared_ptr<BITNode>>& near_nodes);

    void visualizeTree();
    void visualizeHeapAndUnvisited();
    void visualizePath(const std::vector<std::shared_ptr<BITNode>>& path_nodes);

    void visualizeSmoothedPath(const std::vector<Eigen::VectorXd>& shortest_path_);
    std::vector<Eigen::VectorXd> getSmoothedPathPositions(int num_intermediates, int smoothing_window) const;
    std::vector<Eigen::VectorXd> smoothPath(const std::vector<Eigen::VectorXd>& path, int window_size) const;
    std::vector<Eigen::VectorXd> interpolatePath(const std::vector<Eigen::VectorXd>& path, int num_intermediates) const;

    double heuristic(int current_index);
    void clearPlannerState();
    
    void addBatchOfSamples(int num_samples);
    void addBatchOfSamplesUninformed(int num_samples);

    Eigen::VectorXd sampleInEllipsoid(const Eigen::VectorXd& center, const Eigen::MatrixXd& R, double a, double b);
    Eigen::MatrixXd computeRotationMatrix(const Eigen::VectorXd& dir);
    Eigen::VectorXd sampleUnitBall(int d);
    void prune();
    void pruneSamples();
    void updateChildrenCosts(std::shared_ptr<BITNode> node);

private:


    // std::vector<Eigen::VectorXd> nodes;

    std::shared_ptr<State> start_;
    std::shared_ptr<State> goal_;
    std::vector<std::shared_ptr<State>> path_;
    std::vector<std::shared_ptr<BITNode>> tree_;
    std::shared_ptr<KDTree> kdtree_samples_;
    std::shared_ptr<KDTree> kdtree_tree_;

    std::shared_ptr<StateSpace> statespace_;
    std::shared_ptr<ProblemDefinition> problem_;
    std::shared_ptr<Visualization> visualization_;
    std::shared_ptr<ObstacleChecker> obs_checker_;

//     // PriorityQueue<std::shared_ptr<BITNode>, FMTBITComparator> vertex_queue_;
//     std::priority_queue<
//     std::pair<double, std::shared_ptr<BITNode>>,
//     std::vector<std::pair<double, std::shared_ptr<BITNode>>>,
//     FMTBITComparator
// > vertex_queue_;
    

    PriorityQueue2<BITNode, FMTBITComparator> vertex_queue_;



    std::vector<std::shared_ptr<BITNode>> samples_;
    std::vector<std::shared_ptr<BITNode>> vsol_;
    std::vector<std::shared_ptr<BITNode>> unconnected_;
    std::vector<std::shared_ptr<BITNode>> unexpanded_;
    
    std::shared_ptr<BITNode> robot_node_;
    Eigen::VectorXd robot_position_;
    std::unordered_map<std::pair<int, int>, bool, pair_hash> obstacle_check_cache;
    std::priority_queue<EdgeCandidate, std::vector<EdgeCandidate>, 
                       std::greater<EdgeCandidate>> edge_queue_;
    
    std::unordered_set<std::shared_ptr<BITNode>> unprocessed_nodes_;
    std::vector<std::shared_ptr<BITNode>> unconnected_nodes_container_;

    int collision_check_ = 0;

    double ci_ = std::numeric_limits<double>::infinity();
    double current_best_cost_ = INFINITY;
    double lower_bound_;
    double upper_bound_;
    double neighborhood_radius_;
    double max_edge_length_;
    int num_of_samples_;
    int num_batch_;
    int root_state_index_;
    int robot_state_index_;
    int d;
    double mu;
    double zetaD;
    double gamma;
    double factor;
    bool use_kdtree;
    bool obs_cache = false;
    bool partial_plot = false;
};