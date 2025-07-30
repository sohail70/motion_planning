// Copyright 2025 Soheil E.nia
#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/rrtx_node.hpp"
#include "motion_planning/utils/visualization.hpp"
#include "motion_planning/ds/priority_queue.hpp"

class KinodynamicRRTX : public Planner {
 public:
    KinodynamicRRTX(std::shared_ptr<StateSpace> statespace, 
        std::shared_ptr<ProblemDefinition> problem_def,
        std::shared_ptr<ObstacleChecker> obs_checker);
    
    void setup(const Params& params, std::shared_ptr<Visualization> visualization) override;
    void plan() override;
    std::vector<int> getPathIndex() const;
    std::vector<Eigen::VectorXd> getPathPositions() const;

    void setStart(const Eigen::VectorXd& start) override;
    void setGoal(const Eigen::VectorXd& goal) override;
    void clearPlannerState() ;

    // Obstacle management
    void updateObstacleSamples(const ObstacleVector& obstacles);
    void updateRobotPosition(const Eigen::VectorXd& new_position);

    // Visualization
    void visualizeTree();

    void visualizePath(const std::vector<Eigen::VectorXd>& path_waypoints);

    void visualizeSmoothedPath(const std::vector<Eigen::VectorXd>& shortest_path_);

    // Path smoothing
    std::vector<Eigen::VectorXd> getSmoothedPathPositions(int num_intermediates, 
                                                        int smoothing_window) const;

std::unordered_set<int> findSamplesNearObstacles(const ObstacleVector& obstacles, double max_length);

    bool isValidEdge(RRTxNode* from, RRTxNode* to, const EdgeInfo& edge) const;

    void setClock(rclcpp::Clock::SharedPtr clock);

    void setRobotState(const Eigen::VectorXd& robot_state);

    bool isPathStillValid(const std::vector<Eigen::VectorXd>& path, const Eigen::VectorXd& current_robot_state) const;

    bool arePathsSimilar(const std::vector<Eigen::VectorXd>& path_a, const std::vector<Eigen::VectorXd>& path_b, double tolerance) const;

    void dumpTreeToCSV(const std::string& filename) const;


    struct ReplanMetrics {
        long long rewire_neighbor_searches = 0;
        int obstacle_checks = 0;
        int orphaned_nodes = 0;
        double path_cost = 0.0;
    };

    const ReplanMetrics& getLastReplanMetrics() const { return last_replan_metrics_; }
    double getRobotTimeToGo() const { return robot_current_time_to_goal_; }

 private:
    // Core data structures
    std::vector<std::shared_ptr<RRTxNode>> tree_;
    std::shared_ptr<KDTree> kdtree_;
    PriorityQueue<RRTxNode, RRTxComparator> inconsistency_queue_;
    
    // State management
    std::shared_ptr<StateSpace> statespace_;
    std::shared_ptr<ProblemDefinition> problem_;
    std::shared_ptr<ObstacleChecker> obs_checker_;
    std::shared_ptr<Visualization> visualization_;

    // Node tracking
    // RRTxNode* vbot_node_ = nullptr;  // Added missing declaration
    RRTxNode*  vbot_node_;
    std::unordered_set<int> Vc_T_; // Store indices instead of pointers

    std::unordered_set<int> samples_in_obstacles_;


    // Algorithm parameters
    double neighborhood_radius_;
    double epsilon_ = 1e-6;
    double gamma_;
    double delta = 20.0; 
    double factor;
    int num_of_samples_;
    int dimension_;
    int root_state_index_ = -1;
    int robot_state_index_ = -1;
    size_t sample_counter = 0;
    bool cap_samples_ = true;
    bool update_obstacle = false;
    bool partial_update;
    bool ignore_sample;

    Eigen::VectorXd robot_position_;
    std::unordered_map<int, double> edge_length_;
    int max_length_edge_ind;
    double max_length;
    int vbot_index_;
    int vgoal_index_;
    std::unordered_set<int> v_indices_;
    Eigen::VectorXd lower_bounds_;
    Eigen::VectorXd upper_bounds_;
    bool use_kdtree;


    int kd_dim ; 


    bool static_obs_presence;
    ObstacleVector seen_statics_;


    rclcpp::Clock::SharedPtr clock_;
    Eigen::VectorXd robot_continuous_state_;
    double robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();


    FMTNode* robot_anchor_node_ = nullptr;
    Trajectory robot_bridge_trajectory_;



    int findNodeIndex(RRTxNode* node) const;

    // Helper methods
    bool extend(Eigen::VectorXd v);
    std::unordered_map<RRTxNode*, Trajectory> findParent(std::shared_ptr<RRTxNode> v, const std::vector<size_t>& candidate_indices);

    void rewireNeighbors(RRTxNode* v);
    void reduceInconsistency();
    void cullNeighbors(RRTxNode* v);
    void makeParentOf(RRTxNode* child, RRTxNode* new_parent, double edge_dist);
    void updateLMC(RRTxNode* v);
    void verifyQueue(RRTxNode* node);  // Fixed signature
    void propagateDescendants();
    void verifyOrphan(RRTxNode* node);
    double shrinkingBallRadius() const;
    void addNewObstacle(const std::vector<int>& added_indices);
    void removeObstacle(const std::vector<int>& removed_indices);
    Eigen::VectorXd saturate(const Eigen::VectorXd& newPoint, const Eigen::VectorXd& closestPoint, double delta);

    // Path smoothing implementations
    std::vector<Eigen::VectorXd> smoothPath(const std::vector<Eigen::VectorXd>& path, 
                                          int window_size) const;
    std::vector<Eigen::VectorXd> interpolatePath(const std::vector<Eigen::VectorXd>& path,
                                               int num_intermediates) const;

    
    int obs_check = 0;
    int mode;

    ReplanMetrics last_replan_metrics_; 


    std::unordered_map<std::string, Obstacle> previous_obstacles_;


    mutable std::unordered_map<int, std::vector<Obstacle>> node_to_threats_map_;


};