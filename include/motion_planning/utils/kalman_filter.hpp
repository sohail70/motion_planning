// Copyright 2025 Soheil E.nia

#pragma once

#include <Eigen/Dense>

// Defines the available Kalman Filter models.
enum class KalmanModelType {
    CONSTANT_VELOCITY,
    CONSTANT_ACCELERATION,
    SINGER
};

class KalmanFilter {
public:
    // Constructor that accepts the model type and optional parameters for the Singer model.
    KalmanFilter(KalmanModelType model_type, double singer_alpha = 0.1, double singer_sigma_a = 5.0);

    // Public interface for the filter.
    void init(const Eigen::VectorXd& x0);
    void predict(double dt);
    void update(const Eigen::VectorXd& z);
    Eigen::VectorXd getState() const { return x_hat_; }

private:
    // Member variables to store the filter's state and model type.
    KalmanModelType model_type_;
    Eigen::VectorXd x_hat_; 
    Eigen::MatrixXd F_, H_, P_, Q_, R_, I_;
    
    // Parameters specific to the Singer model.
    double alpha_;
    double sigma_a_sq_;

    // Private helper functions to initialize matrices for each model.
    void init_cv();
    void init_ca();
    void init_singer(double alpha, double sigma_a);
};