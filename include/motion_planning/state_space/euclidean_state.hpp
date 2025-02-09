// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/state_space/state.hpp"

class EuclideanState: public State {
 public:
    explicit EuclideanState(int dimension_);
    EuclideanState(const std::vector<double>& values);
    std::unique_ptr<State> clone() const override;
    bool equals(const State& other) const override;
    std::string toString() const override;

    std::vector<double> value_;
};


