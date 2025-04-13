// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/pch.hpp"

class State {
 public:
    virtual std::shared_ptr<State> clone() const = 0; // Clone the state
    virtual bool equals(const State& other) const = 0; // Compare states
    virtual std::string toString() const = 0; // Serialize to string
    virtual const Eigen::VectorXd& getValue() const = 0; // Ensure all states provide vector data

    virtual ~State() = default;
 private:
};
