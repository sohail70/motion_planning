#pragma once

#include "motion_planning/state_space/dubins_statespace.hpp" // Inherit from the 3D Dubins class

enum class HoverDirection {
    RIGHT,
    LEFT
};

/**
 * @class DubinsTimeStateSpace
 * @brief Extends DubinsStateSpace to include a time dimension and velocity constraints.
 *
 * This class models a Dubins vehicle in a 4D state space (x, y, theta, time).
 * It reuses the geometric path calculations from the base DubinsStateSpace and adds
 * time-based validation, such as enforcing minimum and maximum vehicle speeds.
 * The cost of a trajectory in this space is defined as the time elapsed.
 */
class DubinsTimeStateSpace : public DubinsStateSpace {
public:
    /**
     * @brief Construct a new DubinsTimeStateSpace object.
     * @param min_turning_radius The vehicle's minimum turning radius.
     * @param min_velocity The minimum forward velocity of the vehicle.
     * @param max_velocity The maximum forward velocity of the vehicle.
     */
    DubinsTimeStateSpace(double min_turning_radius, double min_velocity, double max_velocity, unsigned int seed = 42);

    ~DubinsTimeStateSpace() override = default;

    // --- Override virtual functions to handle the 4th dimension (time) ---

    /**
     * @brief Calculates the shortest path between two 4D states, respecting Dubins
     * kinematics and velocity constraints.
     * @param from The 4D starting state (x, y, theta, t).
     * @param to The 4D ending state (x, y, theta, t).
     * @return A Trajectory object containing the path, cost (time elapsed), and validity.
     */
    Trajectory steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const override;

    /**
     * @brief Calculates the weighted distance between two 4D states for the KD-Tree.
     */
    double distance(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2) const override;
    
    /**
     * @brief Generates a random 4D sample within the specified bounds.
     */
    std::shared_ptr<State> sampleUniform(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds);

    /**
     * @brief Creates a circular hover path at a given state.
     * @param hover_state The 4D state (x, y, theta, time) where the hover should begin.
     * @param duration The desired duration of the hover maneuver.
     * @param direction The turning direction for the hover circle (RIGHT or LEFT).
     * @return A valid, executable Trajectory for the hover path.
     */
    Trajectory createHoverPath(const Eigen::VectorXd& hover_state, double duration, HoverDirection direction) const;






private:
    double min_velocity_;
    double max_velocity_;
};
