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
    std::shared_ptr<State> sampleUniform(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds) override;

    Eigen::VectorXd sampleUniformUnregistered(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds) override;
    /**
     * Creates a circular hover path at a given state.
     * hover_state The 4D state (x, y, theta, time) where the hover should begin.
     * duration The desired duration of the hover maneuver.
     * direction The turning direction for the hover circle (RIGHT or LEFT).
     * A valid, executable Trajectory for the hover path.
     */
    Trajectory createHoverPath(const Eigen::VectorXd& hover_state, double duration, HoverDirection direction) const;



    double getMaxVelocity() const override { return max_velocity_; }
    double getMaxAcceleration() const override { return 0.0; } // Dubins is typically 1st-order velocity

    // Heading bias. State = (x, y, theta, t_to_goal). In a goal-rooted tree the robot travels
    // goal-ward, so bias the drawn heading toward the goal direction (with a wide spread, so
    // angular support is kept). Nearby samples then share compatible headings -> short Dubins
    // links (<= r_n). Tree-independent -> AO-safe. (Experimental heuristic; tune 'spread'.)
    void shapeKinodynamicSample(Eigen::VectorXd& sample, const Eigen::Vector2d& root_xy) const override {
        if (sample.size() < 4) return;
        const Eigen::Vector2d d = root_xy - sample.head(2);
        if (d.norm() < 1e-9) return;
        const double theta_goal = std::atan2(d.y(), d.x());
        const double spread = 0.6;                 // <1 compresses toward goal heading (~+-108 deg)
        double theta = theta_goal + spread * sample[2];   // sample[2] (theta) drawn uniform in [-pi,pi]
        while (theta >  M_PI) theta -= 2.0 * M_PI;
        while (theta < -M_PI) theta += 2.0 * M_PI;
        sample[2] = theta;
    }

    Trajectory generateEmergencyManeuver(const Eigen::VectorXd& state, double dt) const;
    std::vector<Trajectory> getEscapePrimitives(const Eigen::VectorXd& state, double dt) const;


private:
    std::mt19937 rng_;
    double min_velocity_;
    double max_velocity_;
};
