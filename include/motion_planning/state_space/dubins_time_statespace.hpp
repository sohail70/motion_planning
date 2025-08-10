// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/state_space/dubins_statespace.hpp" // Inherit from the 3D Dubins class
/**
 * TODO: Fix the hover 
 */

enum class HoverDirection {
    RIGHT,
    LEFT
};

/**
 * DubinsTimeStateSpace
 * Extends DubinsStateSpace to include a time dimension and velocity constraints.
 * This class models a Dubins vehicle in a 4D state space (x, y, theta, time).
 * It reuses the geometric path calculations from the base DubinsStateSpace and adds
 * time-based validation, such as enforcing minimum and maximum vehicle speeds.
 * The cost of a trajectory in this space is defined as the time elapsed.
 */
class DubinsTimeStateSpace : public DubinsStateSpace {
public:

    DubinsTimeStateSpace(double min_turning_radius, double min_velocity, double max_velocity, unsigned int seed = 42);

    ~DubinsTimeStateSpace() override = default;


    /**
     * Calculates the shortest path between two 4D states, respecting Dubins
     * kinematics and velocity constraints.
     * from The 4D starting state (x, y, theta, t).
     * to The 4D ending state (x, y, theta, t).
     * A Trajectory object containing the path, cost (time elapsed), and validity.
     */
    Trajectory steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const override;

    /**
     * Calculates the weighted distance between two 4D states for the KD-Tree.
     */
    double distance(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2) const override;
    
    /**
     * Generates a random 4D sample within the specified bounds.
     */
    std::shared_ptr<State> sampleUniform(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds);

    /**
     * Creates a circular hover path at a given state.
     * hover_state The 4D state (x, y, theta, time) where the hover should begin.
     * duration The desired duration of the hover maneuver.
     * direction The turning direction for the hover circle (RIGHT or LEFT).
     * A valid, executable Trajectory for the hover path.
     */
    Trajectory createHoverPath(const Eigen::VectorXd& hover_state, double duration, HoverDirection direction) const;


private:
    double min_velocity_;
    double max_velocity_;
};
