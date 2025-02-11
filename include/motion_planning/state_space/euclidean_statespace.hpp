// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/state_space/state.hpp"
#include "motion_planning/state_space/statespace.hpp"
#include "motion_planning/state_space/euclidean_state.hpp"
#include <Eigen/Dense>


class EuclideanStateSpace : public StateSpace {
 public:
    explicit EuclideanStateSpace(int dimension,int capacity);

    std::unique_ptr<State> allocState(const Eigen::VectorXd& value) const override;
    void sampleUniform(double min, double max);
    void sampleUniform(double min, double max, int k);
    

    double distance(const std::unique_ptr<State>& state1, const std::unique_ptr<State>& state2) const override;
    std::unique_ptr<State> interpolate(const std::unique_ptr<State>& state1, const std::unique_ptr<State>& state2, double t) const;
    bool isValid(const std::unique_ptr<State>& state) const override;

 private:
};