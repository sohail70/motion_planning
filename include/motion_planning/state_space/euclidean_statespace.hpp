// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/state_space/state.hpp"
#include "motion_planning/state_space/statespace.hpp"
#include "motion_planning/state_space/euclidean_state.hpp"
#include <Eigen/Dense>


class EuclideanStateSpace : public StateSpace {
 public:
    explicit EuclideanStateSpace(int dimension,int capacity, unsigned int seed = 42);

    std::shared_ptr<State> addState(const Eigen::VectorXd& value);
    std::shared_ptr<State> sampleUniform(double min, double max);
    void sampleUniform(double min, double max, int k);
    

    double distance(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2) const override;
    std::shared_ptr<State> interpolate(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2, double t) const;
    bool isValid(const std::shared_ptr<State>& state) const override;


    Trajectory steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const override;

 private:
};