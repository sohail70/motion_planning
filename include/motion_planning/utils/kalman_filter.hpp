#pragma once

#include <Eigen/Dense>

// class KalmanFilter {
// public:
//     // Constructor initializes the filter matrices.
//     // n: number of state variables (6 for [px, py, vx, vy, ax, ay])
//     // m: number of measurement variables (2 for [px, py])
//     KalmanFilter(int n = 6, int m = 2);

//     // Initializes the state vector and covariance
//     void init(const Eigen::VectorXd& x0);

//     // Predicts the next state based on the model
//     void predict(double dt);

//     // Updates the state estimate with a new measurement
//     void update(const Eigen::VectorXd& z);

//     // Get the current estimated state
//     Eigen::VectorXd getState() const { return x_hat_; }

// private:
//     // State-space matrices
//     Eigen::MatrixXd F_; // State transition model
//     Eigen::MatrixXd H_; // Measurement model
//     Eigen::MatrixXd Q_; // Process noise covariance
//     Eigen::MatrixXd R_; // Measurement noise covariance
//     Eigen::MatrixXd P_; // Estimate error covariance
//     Eigen::MatrixXd I_; // Identity matrix

//     // State vector
//     Eigen::VectorXd x_hat_; // Estimated state
// };
// ///////////////////////////////////////////////////////////////////////





class KalmanFilter {
public:
    // MODIFIED: Constructor now takes Singer model parameters.
    // The state dimension 'n' and measurement dimension 'm' are fixed to 6 and 2.
    KalmanFilter(double alpha = 0.1, double sigma_a = 1.0);

    // No changes to other public methods
    void init(const Eigen::VectorXd& x0);
    void predict(double dt);
    void update(const Eigen::VectorXd& z);
    Eigen::VectorXd getState() const { return x_hat_; }

private:
    Eigen::VectorXd x_hat_; // State estimate [px, py, vx, vy, ax, ay]
    Eigen::MatrixXd P_;     // State covariance
    Eigen::MatrixXd F_;     // State transition matrix
    Eigen::MatrixXd Q_;     // Process noise covariance
    Eigen::MatrixXd R_;     // Measurement noise covariance
    Eigen::MatrixXd H_;     // Measurement matrix
    Eigen::MatrixXd I_;     // Identity matrix

    // NEW: Member variables for Singer model parameters
    double alpha_;          // The maneuver frequency (1 / maneuver time constant)
    double sigma_a_sq_;     // The variance of the maximum expected acceleration
};