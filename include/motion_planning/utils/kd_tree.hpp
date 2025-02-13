// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/state_space/state.hpp"
#include "motion_planning/state_space/euclidean_state.hpp"
#include "motion_planning/pch.hpp"
class KDTree {
 public:
      virtual ~KDTree() = default;
      virtual void addPoint(const Eigen::VectorXd& stateValue) = 0;
      virtual void addPoints(const std::vector<Eigen::VectorXd>& statesValues) = 0;
      virtual void addPoints(const Eigen::MatrixXd& states) = 0;

      virtual void buildTree() = 0;
      virtual std::vector<size_t> knnSearch(const Eigen::VectorXd& query , int k) const = 0;
      virtual std::vector<size_t> radiusSearch(const Eigen::VectorXd& query , double radius) const = 0;
 private:

};
