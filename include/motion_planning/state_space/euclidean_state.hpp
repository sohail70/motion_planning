// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/state_space/state.hpp"


class EuclideanState : public State {
 public:
    explicit EuclideanState(int dimension);
    EuclideanState(const Eigen::VectorXd& values);
    std::unique_ptr<State> clone() const override;
    bool equals(const State& other) const override;
    std::string toString() const override;
    Eigen::VectorXd getValue() const override;
 private:
    Eigen::VectorXd value_;
};
