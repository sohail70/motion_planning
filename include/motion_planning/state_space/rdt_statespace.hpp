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
    RDTStateSpace(int euclidean_dimension, double min_velocity, double max_velocity, unsigned int seed = 42, bool is_geometric_mode = false);

    std::shared_ptr<State> addState(const Eigen::VectorXd& value) override;
    std::shared_ptr<State> sampleUniform(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds) override;
    double distance(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2) const override;
    bool isValid(const std::shared_ptr<State>& state) const override;

    Trajectory steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const override;

    std::shared_ptr<State> sampleUniform(double min, double max) override;
    Eigen::VectorXd sampleUniformUnregistered(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds) override;
    void sampleUniform(double min, double max, int k) override;
    std::shared_ptr<State> interpolate(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2, double t) const override;

    double getMaxVelocity() const override { return max_velocity_; }
    double getMaxAcceleration() const override { return 0.0; } // R2T is 1st-order (constant velocity)

    Trajectory generateEmergencyManeuver(const Eigen::VectorXd& state, double dt) const;
    std::vector<Trajectory> getEscapePrimitives(const Eigen::VectorXd& state, double dt) const;

private:
    std::mt19937 rng_;
    int euclidean_dim_;
    double min_velocity_;
    double max_velocity_;
    Eigen::VectorXd distance_weights_;
    bool is_geometric_mode_;
};