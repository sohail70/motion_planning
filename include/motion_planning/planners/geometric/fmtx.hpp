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
          FMTX(std::unique_ptr<StateSpace> statespace , std::unique_ptr<ProblemDefinition> problem_def);
          void setup(const PlannerParams& params, std::shared_ptr<Visualization> visualization) override;
          void plan() override;
          std::vector<int> getPathIndex() const override;
          void setStart(const Eigen::VectorXd& start) override;
          void setGoal(const Eigen::VectorXd& goal) override;

          // int getGoalIndex() {return robot_state_index_;}
          // int getStarIndex() {return root_state_index_;}

          std::vector<NeighborInfo> near(int node_index);
          void visualizeTree();


 private:
          std::shared_ptr<State> start_;
          std::shared_ptr<State> goal_;
          std::vector<std::shared_ptr<State>> path_;
          std::vector<std::shared_ptr<TreeNode>> tree_;
          std::shared_ptr<KDTree> kdtree_;

          std::unique_ptr<StateSpace> statespace_;
          std::unique_ptr<ProblemDefinition> problem_;
          
          std::shared_ptr<Visualization> visualization_;


          std::unordered_set<int> v_open_set_;
          std::unordered_set<int> v_unvisited_set_;
          std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, std::greater<>> v_open_heap_;

          std::unordered_map<int, std::vector<NeighborInfo>> neighbors_dict_;



          int num_of_samples_;
          double lower_bound_;
          double upper_bound_;
          int root_state_index_;
          int robot_state_index_;
          bool use_kdtree;
          double neighborhood_radius_;

};

