// Copyright 2025 Soheil E.Nia
#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/tree_node.hpp"
#include "motion_planning/utils/visualization.hpp"

struct NeighborInfo {
    int index;
    double distance;
};


class FMTX : public Planner {
 public:
            FMTX(std::unique_ptr<StateSpace> statespace , std::unique_ptr<ProblemDefinition> problem_def , std::shared_ptr<ObstacleChecker> obs_checker);
            void setup(const PlannerParams& params, std::shared_ptr<Visualization> visualization) override;
            void plan() override;
            std::vector<int> getPathIndex() const override;
            void setStart(const Eigen::VectorXd& start) override;
            void setGoal(const Eigen::VectorXd& goal) override;

            // int getGoalIndex() {return robot_state_index_;}
            // int getStarIndex() {return root_state_index_;}

            std::vector<NeighborInfo> near(int node_index);
            void visualizeTree();
            std::unordered_set<int> findSamplesNearObstacles(const std::vector<Eigen::Vector2d>& obstacles, double obstacle_radius);
            void updateObstacleSamples(const std::vector<Eigen::Vector2d>& obstacles);
            std::unordered_set<int> getDescendants(int node_index);
            void handleAddedObstacleSamples(const std::vector<int>& added);
            void handleRemovedObstacleSamples(const std::vector<int>& removed);

 private:
            std::shared_ptr<State> start_;
            std::shared_ptr<State> goal_;
            std::vector<std::shared_ptr<State>> path_;
            std::vector<std::shared_ptr<TreeNode>> tree_;
            std::shared_ptr<KDTree> kdtree_;

            std::unique_ptr<StateSpace> statespace_;
            std::unique_ptr<ProblemDefinition> problem_;
            
            std::shared_ptr<Visualization> visualization_;
            std::shared_ptr<ObstacleChecker> obs_checker_;


            std::unordered_set<int> v_open_set_;
            std::unordered_set<int> v_unvisited_set_;
            std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, std::greater<>> v_open_heap_;

            std::unordered_map<int, std::vector<NeighborInfo>> neighbors_dict_;
            std::unordered_set<int> samples_in_obstacles_; // Current samples in obstacles

            // struct MaxEdge {
            //     double length = 0.0;    // Length of the largest edge
            //     int from = -1;          // Parent node index
            //     int to = -1;            // Child node index
            // } max_edge_;                // Instance of the struct

            // MaxEdge getLargestEdge() const { return max_edge_; }

            int num_of_samples_;
            double lower_bound_;
            double upper_bound_;
            int root_state_index_;
            int robot_state_index_;
            bool use_kdtree;
            double neighborhood_radius_;

};

