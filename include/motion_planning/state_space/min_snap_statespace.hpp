#pragma once

#include "motion_planning/state_space/statespace.hpp"
#include <qpOASES.hpp>

class MinSnapStateSpace : public StateSpace {
public:
    MinSnapStateSpace( int dimension, double v_max, double a_max, double w_vel = 0.0, double w_accel = 0.0, double w_snap = 1.0, unsigned int seed = 42);


    std::shared_ptr<State> addState(const Eigen::VectorXd& value) override;
    std::shared_ptr<State> sampleUniform(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds) override;
    double distance(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2) const override;
    bool isValid(const std::shared_ptr<State>& state) const override;

    // // Generates a trajectory from a start point (to) to an end point (from), assuming start is at rest.
    // // The final velocity/acceleration are unconstrained but respect dynamic limits.
    // Trajectory steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const override;
    // Generates a trajectory with a specified initial velocity and acceleration.
    // The final velocity/acceleration are unconstrained but respect dynamic limits.
    Trajectory steer_with_initial(const Eigen::VectorXd& from, const Eigen::VectorXd& to,
                                  const Eigen::VectorXd& v0, const Eigen::VectorXd& a0) const;

    // Generates a trajectory with specified initial AND final derivatives (e.g., to come to a stop).
    Trajectory steer_with_initial_and_final(const Eigen::VectorXd& from, const Eigen::VectorXd& to,
                                            const Eigen::VectorXd& v0, const Eigen::VectorXd& a0,
                                            const Eigen::VectorXd& v1, const Eigen::VectorXd& a1) const;

    // This was original steer with free endpoints
    Trajectory steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const override;

    // This was 'steer_with_initial'
    Trajectory steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to,
                             const Eigen::VectorXd& to_vel, const Eigen::VectorXd& to_accel) const override;

    // This was 'steer_with_initial_and_final'
    Trajectory steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to,
                             const Eigen::VectorXd& from_vel, const Eigen::VectorXd& from_accel,
                             const Eigen::VectorXd& to_vel, const Eigen::VectorXd& to_accel) const override;



    Eigen::MatrixXd calculateCombinedHessian( double T, double w_vel, double w_accel, double w_snap) const;

    std::shared_ptr<State> sampleUniform(double min, double max) override;
    void sampleUniform(double min, double max, int k) override;
    std::shared_ptr<State> interpolate(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2, double t) const override;

    bool prefersLazyNear() const override { return true; }


private:
    double v_max_;
    double a_max_;
    int num_coeffs_ = 8; // For a 7th-order polynomial
    int num_axes_ = 4;   // x, y, z, yaw

    double w_vel_;
    double w_accel_;
    double w_snap_;
    unsigned int seed_;

};