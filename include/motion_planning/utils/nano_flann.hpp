// Copyright 2025 Soheil E.nia

#pragma once

#include <Eigen/Dense>

#include "motion_planning/utils/kd_tree.hpp"
#include <nanoflann.hpp>


class NanoFlann : public KDTree {
 public:
          using NFKDTree = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd>;
          explicit NanoFlann(int dimension);
          void addPoint(const Eigen::VectorXd& stateValue) override;
          void addPoints(const std::vector<Eigen::VectorXd>& statesValues) override;
          void addPoints(const Eigen::MatrixXd& states) override;

          void buildTree() override; 
          std::vector<size_t> knnSearch(const Eigen::VectorXd& query , int k) const override;
          std::vector<size_t> radiusSearch(const Eigen::VectorXd& query , double radius) const override;
          std::pair<std::vector<size_t>, std::vector<size_t>> radiusSearchDual(const Eigen::VectorXd& query, double radius1, double radius2) const override;


 private:
          int dimension_;
          Eigen::MatrixXd data_;
          std::unique_ptr<NFKDTree> kdtree_;
}; 
