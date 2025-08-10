// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/state_space/statespace.hpp"
#include "motion_planning/state_space/euclidean_state.hpp"
#include <Eigen/Dense>

/**
 * Represents a state space that is a product of a Euclidean space (R^D) and a time dimension (T).
 * This class is suitable for planning problems where the state is defined by (x, y, ..., time).
 * It assumes motion is a straight line in the Euclidean dimensions at a constant velocity.
 */
class RDTStateSpace : public StateSpace {
public:
    RDTStateSpace(int euclidean_dimension, double min_velocity, double max_velocity, double robot_velocity, int initial_capacity = 1000, unsigned int seed = 42);

    std::shared_ptr<State> addState(const Eigen::VectorXd& value) override;
    std::shared_ptr<State> sampleUniform(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds) override;
    double distance(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2) const override;
    bool isValid(const std::shared_ptr<State>& state) const override;

    Trajectory steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const override;

    std::shared_ptr<State> sampleUniform(double min, double max) override;
    void sampleUniform(double min, double max, int k) override;
    std::shared_ptr<State> interpolate(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2, double t) const override;


private:
    int euclidean_dim_;
    double min_velocity_;
    double max_velocity_;
    double robot_velocity_;
    Eigen::VectorXd distance_weights_;
};