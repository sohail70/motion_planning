// Copyright 2025 Soheil E.nia
#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/tree_node.hpp"
#include "motion_planning/utils/visualization.hpp"



class RRTX : public Planner {
 public:
    RRTX(std::unique_ptr<StateSpace> statespace, 
        std::unique_ptr<ProblemDefinition> problem_def,
        std::shared_ptr<ObstacleChecker> obs_checker);
    
    void setup(const PlannerParams& params, std::shared_ptr<Visualization> visualization) override;
    void plan() override;
    std::vector<int> getPathIndex() const override;
    void setStart(const Eigen::VectorXd& start) override;
    void setGoal(const Eigen::VectorXd& goal) override;

    void updateRobotPosition(const Eigen::VectorXd& new_position);
    void updateObstacleSamples(const std::vector<Eigen::Vector2d>& obstacles);

    void visualizeTree();
    void visualizePath(std::vector<int> path_indices);
    void setRobotIndex(const Eigen::VectorXd& robot_position);

 private:
    std::vector<std::shared_ptr<TreeNode>> tree_;
    std::shared_ptr<KDTree> kdtree_;
    std::shared_ptr<Visualization> visualization_;
    std::shared_ptr<ObstacleChecker> obs_checker_;
    std::unique_ptr<StateSpace> statespace_;
    std::unique_ptr<ProblemDefinition> problem_;
    std::unordered_set<int> v_indices_;

    std::unordered_set<int> samples_in_obstacles_; 

    std::unordered_map<int, QueueElement> handle_map_; 

    int vbot_index_;
    int vgoal_index_;
    double neighborhood_radius_;
    double epsilon_ = 1e-6; 

    int num_of_samples_;
    double lower_bound_;
    double upper_bound_;
    int root_state_index_;
    int robot_state_index_;
    bool use_kdtree;
    int dimension_;
    double gamma_;
    size_t sample_counter;
    bool cap_samples_ = true;
    double delta = 10.0; // Step size limit


    UpdatablePriorityQueue inconsistency_queue_;







    // Neighbor management
    std::unordered_map<int, std::unordered_set<int>> N0_in_;   // Original incoming neighbors
    std::unordered_map<int, std::unordered_set<int>> N0_out_;  // Original outgoing neighbors
    std::unordered_map<int, std::unordered_set<int>> Nr_in_;   // Running incoming neighbors
    std::unordered_map<int, std::unordered_set<int>> Nr_out_;  // Running outgoing neighbors

    // Distance dictionary
    std::unordered_map<int, std::unordered_map<int, double>> distance_;  // Key1: node index, Key2: neighbor index, Value: distance

    std::unordered_set<int> Vc_T_;




    // RRTX functions
    double shrinkingBallRadius() const;
    void extend(Eigen::VectorXd v);
    void rewireNeighbors(int v_index);
    void reduceInconsistency();
    void findParent(Eigen::VectorXd v, const std::vector<size_t>& candidates);
    void cullNeighbors(int v_index);
    void propagateDescendants();
    void verifyQueue(int v_index);
    void updateLMC(int v_index);
    void handleObstacleChanges();
    void makeParentOf(int child_index, int parent_index);
    void addNewObstacle(const std::vector<int>& added_samples);
    void removeObstacle(const std::vector<int>& removed_samples);
    void verifyOrphan(int v_index);


    std::unordered_set<int> findSamplesNearObstacles(
        const std::vector<Eigen::Vector2d>& obstacles, 
        double obstacle_radius
    );
    // Obstacle management
    std::unordered_set<int> obstacle_samples_;
    std::vector<Eigen::Vector2d> current_obstacles_;
};
