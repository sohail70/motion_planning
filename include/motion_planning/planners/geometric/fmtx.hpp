// Copyright 2025 Soheil E.Nia
#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/tree_node.hpp"


class FMTX : public Planner {
 public:
      FMTX(std::unique_ptr<StateSpace> statespace , std::unique_ptr<ProblemDefinition> problem_def);
      void setup(const PlannerParams& params) override;
      void plan() override;
      std::vector<std::shared_ptr<State>> getPath() const override;
      void setStart(const Eigen::VectorXd& start) override;
      void setGoal(const Eigen::VectorXd& goal) override;

      int getGoalIndex() {return robot_state_index_;}
      int getStarIndex() {return root_state_index_;}


 private:
      std::shared_ptr<State> start_;
      std::shared_ptr<State> goal_;
      std::vector<std::shared_ptr<State>> path_;
      std::vector<std::shared_ptr<TreeNode>> tree_;
      std::shared_ptr<KDTree> kdtree_;

      std::unique_ptr<StateSpace> statespace_;
      std::unique_ptr<ProblemDefinition> problem_;

      int num_of_samples_;
      int lower_bound_;
      int upper_bound_;

      int root_state_index_;
      int robot_state_index_;

};

