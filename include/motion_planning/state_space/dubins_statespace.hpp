// Create a new file: motion_planning/state_space/dubins_statespace.hpp
#pragma once

#include "motion_planning/state_space/statespace.hpp"
#include "motion_planning/ds/edge_info.hpp"
#include <vector>



class DubinsStateSpace : public StateSpace {
public:
    // Constructor takes the vehicle's turning radius, a key parameter.
    explicit DubinsStateSpace(double min_turning_radius);
    
    // The core steering function that calculates the Dubins path between two states.
    virtual Trajectory steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const override;

    // --- Overriding virtual functions from StateSpace ---

    // This provides a fast, approximate distance for the KD-Tree.
    double distance(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2) const override;
    
    // You will need to implement these based on your EuclideanStateSpace.
    std::shared_ptr<State> addState(const Eigen::VectorXd& value) override;
    std::shared_ptr<State> sampleUniform(double min, double max) override;
    void sampleUniform(double min, double max, int k) override;
    std::shared_ptr<State> interpolate(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2, double t) const override;
    bool isValid(const std::shared_ptr<State>& state) const override;

protected:
    double min_turning_radius_;
    Eigen::VectorXd weights_; // To balance x, y vs. theta in the distance metric.
};