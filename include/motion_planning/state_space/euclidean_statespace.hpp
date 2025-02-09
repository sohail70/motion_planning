// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/state_space/state.hpp"
#include "motion_planning/state_space/statespace.hpp"
#include "motion_planning/state_space/euclidean_state.hpp"


class EuclideanStateSpace : public StateSpace {
 public:
    explicit EuclideanStateSpace(int dimension);
    std::unique_ptr<State> allocState() const override;
    std::unique_ptr<State> sampleUniform() const;
    double distance(const State& state1, const State& state2) const override;
    std::unique_ptr<State> interpolate(const State& state1,
                                       const State& state2, double t) const;  // t in [0, 1]

 private:
    int dimension_;
};
