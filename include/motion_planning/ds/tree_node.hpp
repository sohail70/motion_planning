// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/ds/node.hpp"

class TreeNode : public Node {
 public:
      explicit TreeNode(std::unique_ptr<State> state, std::shared_ptr<Node> parent = nullptr);
      Eigen::VectorXd getStateVlaue() const override;
      
      void setCost(double cost) override;
      double getCost() const override;
      
      void setLMC(double lmc) override;
      double getLMC() const override;

      void setParentIndex(int index);
      int getParentIndex() const override;
      
      void setChildrenIndex(int index) override;
      std::vector<int>& getChildrenIndices() override; // Its not const and it return a reference so that i can change it in the planner! whats a better a way to handle the change here?



 private:
      std::unique_ptr<State> state_;
      int parent_index_;
      std::vector<int> children_indices_;
      double cost_to_root_ = std::numeric_limits<double>::infinity(); // TODO: i hope it doesnt mess up fmtx!
      double look_ahead_ = std::numeric_limits<double>::infinity(); //TODO: should i ? in the find parent in rrtx it doesnt go to the if condtion if the lmc is initialized to zero!



};
