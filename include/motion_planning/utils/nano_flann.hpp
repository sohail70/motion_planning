// Copyright 2025 Soheil E.nia

#pragma once

#include <Eigen/Dense>

#include "motion_planning/utils/kd_tree.hpp"
#include <nanoflann.hpp>


class NanoFlann : public KDTree {
 public:
    using NFKDTree = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd>;
    explicit NanoFlann(int dimension);
    void addPoint(const std::unique_ptr<State>& state) override;
    void addPoints(const std::vector<std::unique_ptr<State>>& states) override;
    void addPoints(const Eigen::MatrixXd& states) override;

    std::vector<std::shared_ptr<State>> knnSearch(const std::unique_ptr<State>& query , int k) const override;
    std::vector<std::shared_ptr<State>> radiusSearch(const std::unique_ptr<State>& query , double radius) const override;

 private:
    int dimension_;
    Eigen::MatrixXd data_;
    std::unique_ptr<NFKDTree> kdtree_;
}; 
