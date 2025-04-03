// Modified RRTX header (rrtx.hpp)
#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/tree_node.hpp"
#include "motion_planning/utils/visualization.hpp"

// struct NeighborInfo {
//     int index;
//     double distance;
// };

class RRTX : public Planner {
 public:
    RRTX(std::unique_ptr<StateSpace> statespace, 
        std::unique_ptr<ProblemDefinition> problem_def,
        std::shared_ptr<ObstacleChecker> obs_checker);
    
    void setup(const Params& params, std::shared_ptr<Visualization> visualization) override;
    void plan() override;
    std::vector<int> getPathIndex() const override;
    void setStart(const Eigen::VectorXd& start) override;
    void setGoal(const Eigen::VectorXd& goal) override;

    // RRTX-specific interface
    void updateRobotPosition(const Eigen::VectorXd& new_position);
    void updateObstacles(const std::vector<Eigen::Vector2d>& new_obstacles);
    void visualizeTree();
    void visualizePath(std::vector<int> path_indices);



 private:
    // Core data structures
    std::vector<std::shared_ptr<TreeNode>> tree_;
    std::shared_ptr<KDTree> kdtree_;
    std::shared_ptr<Visualization> visualization_;
    std::shared_ptr<ObstacleChecker> obs_checker_;
    std::unique_ptr<StateSpace> statespace_;
    std::unique_ptr<ProblemDefinition> problem_;
    std::unordered_set<int> v_indices_;
            

    // Algorithm state
    int vbot_index_;          // Robot's current node
    int vgoal_index_;         // Goal node index
    double neighborhood_radius_;
    double epsilon_ = 1e-6;   // Consistency threshold

    int num_of_samples_;
    double lower_bound_;
    double upper_bound_;
    int root_state_index_;
    int robot_state_index_;
    bool use_kdtree;
    int dimension_;
    double gamma_;
    // Priority queue for inconsistency propagation
    using QueueElement = std::pair<double, int>;
    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<>> inconsistency_queue_;


    // Neighbor management
    std::unordered_map<int, std::unordered_set<int>> N0_in_;   // Original incoming neighbors
    std::unordered_map<int, std::unordered_set<int>> N0_out_;  // Original outgoing neighbors
    std::unordered_map<int, std::unordered_set<int>> Nr_in_;   // Running incoming neighbors
    std::unordered_map<int, std::unordered_set<int>> Nr_out_;  // Running outgoing neighbors

    // Distance dictionary
    std::unordered_map<int, std::unordered_map<int, double>> distance_;  // Key1: node index, Key2: neighbor index, Value: distance





    // Helper functions
    double shrinkingBallRadius() const;
    void extend(Eigen::VectorXd v);
    void rewireNeighbors(int v_index);
    void reduceInconsistency();
    void findParent(Eigen::VectorXd v, const std::vector<size_t>& candidates);
    void cullNeighbors(int v_index);
    void propagateDescendants(int v_index);
    void verifyQueue(int v_index);
    void updateLMC(int v_index);
    void handleObstacleChanges();
    void makeParentOf(int child_index, int parent_index);

    
    // Obstacle management
    std::unordered_set<int> obstacle_samples_;
    std::vector<Eigen::Vector2d> current_obstacles_;
};
