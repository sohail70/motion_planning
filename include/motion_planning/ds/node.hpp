// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/state_space/state.hpp"

class Node {
 public:
    virtual ~Node() = default;
    virtual Eigen::VectorXd getStateVlaue() const = 0;
    virtual double getCost() const = 0;
};





