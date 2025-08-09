// Copyright 2025 Soheil E.nia
#pragma once

#include "motion_planning/state_space/statespace.hpp"
#include "motion_planning/ds/edge_info.hpp"
#include <vector>



class DubinsStateSpace : public StateSpace {
public:
    // Constructor takes the vehicle's turning radius, a key parameter.
    explicit DubinsStateSpace(double min_turning_radius, int dimension = 3, unsigned int seed = 42);
    
    // The core steering function that calculates the Dubins path between two states.
    virtual Trajectory steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const override;


    // This provides a fast, approximate distance for the KD-Tree.
    double distance(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2) const override;
    
    // You will need to implement these based on your EuclideanStateSpace.
    std::shared_ptr<State> addState(const Eigen::VectorXd& value) override;
    std::shared_ptr<State> sampleUniform(double min, double max) override;
    void sampleUniform(double min, double max, int k) override;
    std::shared_ptr<State> sampleUniform(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds) override;
    std::shared_ptr<State> interpolate(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2, double t) const override;
    bool isValid(const std::shared_ptr<State>& state) const override;

    // static inline void sample_arc_optimized( std::vector<Eigen::VectorXd>& path_points,
    //     const Eigen::VectorXd& from_state,
    //     const Eigen::Vector2d& start_point_2d,
    //     const Eigen::Vector2d& end_point_2d,
    //     const Eigen::Vector2d& center_2d,
    //     double radius,
    //     bool is_clockwise,
    //     double angular_resolution_rad);

    // static inline double arc_len_optimized( const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C, double r, bool clockwise);

protected:
    double min_turning_radius_;
    Eigen::VectorXd weights_; // To balance x, y vs. theta in the distance metric --> But i do not use this because i implemented this in kd tree
};