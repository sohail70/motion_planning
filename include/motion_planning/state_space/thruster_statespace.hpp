// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/state_space/statespace.hpp"
#include "motion_planning/state_space/euclidean_state.hpp" // Re-use EuclideanState for state values
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream> // For debug output
#include "motion_planning/ds/edge_info.hpp"
#include <omp.h> 

class ThrusterSteerStateSpace : public StateSpace {
public:
    // State is [x, y, z, vx, vy, vz, t] for 3D position and velocity
    // Dimension is typically 7 (3 pos + 3 vel + 1 time)
    // The control input 'u' is acceleration in this model (bang-bang control)
    ThrusterSteerStateSpace(int dimension, double max_acceleration, unsigned int seed = 42);

    // Override StateSpace methods
    std::shared_ptr<State> addState(const Eigen::VectorXd& value) override;
    std::shared_ptr<State> sampleUniform(double min = 0.0, double max = 1.0) override;
    void sampleUniform(double min = 0.0, double max = 1.0, int k = 1) override;
    std::shared_ptr<State> sampleUniform(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds) override;
    double distance(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2) const override;
    std::shared_ptr<State> interpolate(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2, double t) const override;
    bool isValid(const std::shared_ptr<State>& state) const override;


    // The main steering function for the 3rd-order thruster
    // from: [x, y, z, vx, vy, vz, t_start]
    // to:   [x, y, z, vx, vy, vz, t_end]
    Trajectory steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const override;

    // Helper for 1D steering 
    // Returns [t1, t2, a1, a2, v_coast] or NaNs if no solution
    Eigen::VectorXd steering1D(double x_start, double x_end, double v_start, double v_end, double t_start, double t_end, double a_max) const;

    // Helper for ND steering 
    // Returns tuple: {success, Time, X, V, A}
    struct NDSteeringResult {
        bool success;
        Eigen::VectorXd Time; // (num_interleaved_points x 1)
        Eigen::MatrixXd X;    // (num_interleaved_points x dim_spatial_vel) position
        Eigen::MatrixXd V;    // (num_interleaved_points x dim_spatial_vel) velocity
        Eigen::MatrixXd A;    // (num_interleaved_points-1 x dim_spatial_vel) acceleration between time points
    };
    NDSteeringResult steeringND(const Eigen::VectorXd& x_start, const Eigen::VectorXd& x_end,
                                const Eigen::VectorXd& v_start, const Eigen::VectorXd& v_end,
                                double t_start, double t_end, const Eigen::VectorXd& a_max_vec) const;

    // Helper for fine-graining the trajectory
    std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
    fineGrain(const Eigen::VectorXd& Time_raw, const Eigen::MatrixXd& A_raw,
              const Eigen::MatrixXd& V_raw, const Eigen::MatrixXd& X_raw, double dt_res) const;
    
    double getGeometricDistance(const NDSteeringResult& result) const;
private:
    double max_acceleration_; // Maximum absolute acceleration per dimension
    Eigen::VectorXd weights_; // To balance






    // Helper to extract spatial position and velocity from a state vector that includes time
    Eigen::VectorXd getSpatialPosition(const Eigen::VectorXd& state) const;
    Eigen::VectorXd getSpatialVelocity(const Eigen::VectorXd& state) const;


};
