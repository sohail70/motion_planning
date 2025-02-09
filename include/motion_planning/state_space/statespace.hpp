// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/state_space/euclidean_state.hpp"


class StateSpace {
 public:
    virtual ~StateSpace() = default;
    virtual std::unique_ptr<State> allocState() const = 0;
    virtual std::unique_ptr<State> sampleUniform() const = 0;
    virtual double distance(const State& state1, const State& state2) const = 0;
    virtual std::unique_ptr<State> interpolate(const State& state1, const State& state2, double t) const = 0;
    // virtual bool isValid(const State& state) const = 0; 
};







