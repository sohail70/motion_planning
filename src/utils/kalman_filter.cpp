// Copyright 2025 Soheil E.nia

#include "motion_planning/utils/kalman_filter.hpp"
#include <stdexcept>

// The constructor acts as a switch to call the correct initialization function.
KalmanFilter::KalmanFilter(KalmanModelType model_type, double singer_alpha, double singer_sigma_a)
    : model_type_(model_type) {
    switch (model_type_) {
        case KalmanModelType::CONSTANT_VELOCITY:
            init_cv();
            break;
        case KalmanModelType::CONSTANT_ACCELERATION:
            init_ca();
            break;
        case KalmanModelType::SINGER:
            init_singer(singer_alpha, singer_sigma_a);
            break;
        default:
            throw std::runtime_error("Invalid KalmanFilter model type specified.");
    }
}

// The predict method uses a switch to apply the correct motion model logic.
void KalmanFilter::predict(double dt) {
    switch (model_type_) {
        case KalmanModelType::CONSTANT_VELOCITY:
            F_(0, 2) = dt;
            F_(1, 3) = dt;
            break;

        case KalmanModelType::CONSTANT_ACCELERATION:
            F_(0, 2) = dt;    
            F_(1, 3) = dt;    
            F_(0, 4) = 0.5 * dt * dt;
            F_(1, 5) = 0.5 * dt * dt;
            F_(2, 4) = dt;    
            F_(3, 5) = dt;    
            break;

        case KalmanModelType::SINGER:
            { 
                double e_alpha_dt = std::exp(-alpha_ * dt);
                F_.setIdentity(); // Reset F for Singer model
                F_(0, 2) = dt;
                F_(1, 3) = dt;
                F_(0, 4) = (alpha_ * dt - 1.0 + e_alpha_dt) / (alpha_ * alpha_);
                F_(1, 5) = (alpha_ * dt - 1.0 + e_alpha_dt) / (alpha_ * alpha_);
                F_(2, 4) = (1.0 - e_alpha_dt) / alpha_;
                F_(3, 5) = (1.0 - e_alpha_dt) / alpha_;
                F_(4, 4) = e_alpha_dt;
                F_(5, 5) = e_alpha_dt;
            }
            break;
    }
    
    x_hat_ = F_ * x_hat_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

// The init and update methods are model-agnostic.
void KalmanFilter::init(const Eigen::VectorXd& x0) {
    if (x0.size() != x_hat_.size()) {
        throw std::runtime_error("Initial state vector has wrong dimension for the selected KF model.");
    }
    x_hat_ = x0;
}

void KalmanFilter::update(const Eigen::VectorXd& z) {
    Eigen::VectorXd y = z - H_ * x_hat_;
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    x_hat_ = x_hat_ + K * y;
    P_ = (I_ - K * H_) * P_;
}

// --- Private Initialization Helpers ---

void KalmanFilter::init_cv() {
    int n = 4; // State: [px, py, vx, vy]
    int m = 2; // Measurement: [px, py]

    x_hat_.resize(n);
    F_ = Eigen::MatrixXd::Identity(n, n);
    H_ = Eigen::MatrixXd::Zero(m, n);
    P_ = Eigen::MatrixXd::Identity(n, n);
    Q_ = Eigen::MatrixXd::Identity(n, n);
    R_ = Eigen::MatrixXd::Identity(m, m);
    I_ = Eigen::MatrixXd::Identity(n, n);

    H_(0, 0) = 1.0; H_(1, 1) = 1.0;
    R_ *= 1e-5;
    Q_ *= 0.01;
    Q_(2, 2) = 0.5; Q_(3, 3) = 0.5;
}

void KalmanFilter::init_ca() {
    int n = 6; // State: [px, py, vx, vy, ax, ay]
    int m = 2;

    x_hat_.resize(n);
    F_ = Eigen::MatrixXd::Identity(n, n);
    H_ = Eigen::MatrixXd::Zero(m, n);
    P_ = Eigen::MatrixXd::Identity(n, n);
    Q_ = Eigen::MatrixXd::Identity(n, n);
    R_ = Eigen::MatrixXd::Identity(m, m);
    I_ = Eigen::MatrixXd::Identity(n, n);
    
    H_(0, 0) = 1.0; H_(1, 1) = 1.0;
    R_ *= 1e-5;
    Q_ *= 0.01;
    Q_(4, 4) = 5.0; Q_(5, 5) = 5.0;
}

void KalmanFilter::init_singer(double alpha, double sigma_a) {
    int n = 6;
    int m = 2;

    alpha_ = alpha;
    sigma_a_sq_ = sigma_a * sigma_a;

    x_hat_.resize(n);
    F_ = Eigen::MatrixXd::Identity(n, n);
    H_ = Eigen::MatrixXd::Zero(m, n);
    P_ = Eigen::MatrixXd::Identity(n, n);
    Q_ = Eigen::MatrixXd::Identity(n, n);
    R_ = Eigen::MatrixXd::Identity(m, m);
    I_ = Eigen::MatrixXd::Identity(n, n);

    H_(0, 0) = 1.0; H_(1, 1) = 1.0;
    R_ *= 0.5;
    Q_ *= 0.01;
    Q_(4, 4) = sigma_a_sq_; Q_(5, 5) = sigma_a_sq_;
}