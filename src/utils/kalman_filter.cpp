#include "motion_planning/utils/kalman_filter.hpp"


// KalmanFilter::KalmanFilter(int n, int m) {
//     x_hat_ = Eigen::VectorXd::Zero(n);
    
//     F_ = Eigen::MatrixXd::Identity(n, n);
//     H_ = Eigen::MatrixXd::Zero(m, n);
//     Q_ = Eigen::MatrixXd::Identity(n, n) * 0.1; // Tune this
//     R_ = Eigen::MatrixXd::Identity(m, m) * 0.5; // Tune this
//     P_ = Eigen::MatrixXd::Identity(n, n);
//     I_ = Eigen::MatrixXd::Identity(n, n);

//     // We only measure position
//     H_(0, 0) = 1.0; // px
//     H_(1, 1) = 1.0; // py
// }

// void KalmanFilter::init(const Eigen::VectorXd& x0) {
//     x_hat_ = x0;
// }

// void KalmanFilter::predict(double dt) {
//     // Update state transition matrix F for the given time step dt
//     // Based on constant acceleration model:
//     // p_new = p_old + v*dt + 0.5*a*dt^2
//     // v_new = v_old + a*dt
//     // a_new = a_old
//     F_(0, 2) = dt;    // px contribution from vx
//     F_(1, 3) = dt;    // py contribution from vy
//     F_(0, 4) = 0.5 * dt * dt; // px contribution from ax
//     F_(1, 5) = 0.5 * dt * dt; // py contribution from ay
//     F_(2, 4) = dt;    // vx contribution from ax
//     F_(3, 5) = dt;    // vy contribution from ay
    
//     x_hat_ = F_ * x_hat_;
//     P_ = F_ * P_ * F_.transpose() + Q_;
// }

// void KalmanFilter::update(const Eigen::VectorXd& z) {
//     Eigen::VectorXd y = z - H_ * x_hat_; // Measurement residual
//     Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
//     Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse(); // Kalman gain

//     x_hat_ = x_hat_ + K * y;
//     P_ = (I_ - K * H_) * P_;
// }
// /////////////////////////////////





// MODIFIED: New constructor implementation
KalmanFilter::KalmanFilter(double alpha, double sigma_a) {
    // Store Singer model parameters
    alpha_ = alpha;
    sigma_a_sq_ = sigma_a * sigma_a;

    // Initialize matrices with correct dimensions
    x_hat_ = Eigen::VectorXd::Zero(6);
    P_ = Eigen::MatrixXd::Identity(6, 6);
    F_ = Eigen::MatrixXd::Identity(6, 6);
    Q_ = Eigen::MatrixXd::Identity(6, 6);
    H_ = Eigen::MatrixXd::Zero(2, 6);
    I_ = Eigen::MatrixXd::Identity(6, 6);

    // Measurement noise covariance (tune this based on sensor accuracy)
    R_ = Eigen::MatrixXd::Identity(2, 2) * 0.5;

    // Measurement matrix H maps the 6D state to a 2D measurement (px, py)
    H_(0, 0) = 1.0;
    H_(1, 1) = 1.0;
}

void KalmanFilter::init(const Eigen::VectorXd& x0) {
    x_hat_ = x0;
}

// REPLACED: The entire predict function is new
void KalmanFilter::predict(double dt) {
    // --- 1. Calculate the State Transition Matrix F for the Singer Model ---
    double e_alpha_dt = std::exp(-alpha_ * dt);

    F_.setIdentity();
    // Position update from velocity and acceleration
    F_(0, 2) = dt;
    F_(1, 3) = dt;
    F_(0, 4) = (alpha_ * dt - 1.0 + e_alpha_dt) / (alpha_ * alpha_);
    F_(1, 5) = (alpha_ * dt - 1.0 + e_alpha_dt) / (alpha_ * alpha_);

    // Velocity update from acceleration
    F_(2, 4) = (1.0 - e_alpha_dt) / alpha_;
    F_(3, 5) = (1.0 - e_alpha_dt) / alpha_;

    // Acceleration update (decays over time)
    F_(4, 4) = e_alpha_dt;
    F_(5, 5) = e_alpha_dt;

    // --- 2. Calculate the Process Noise Covariance Matrix Q ---
    Q_.setZero();
    double t1 = 1.0 / (2.0 * alpha_ * alpha_ * alpha_ * alpha_ * alpha_) * (1.0 - e_alpha_dt * e_alpha_dt - 4.0 * alpha_ * dt * e_alpha_dt + 2.0 * alpha_ * dt + (2.0/3.0) * alpha_ * alpha_ * alpha_ * dt*dt*dt);
    double t2 = 1.0 / (2.0 * alpha_ * alpha_ * alpha_ * alpha_) * (e_alpha_dt * e_alpha_dt + 1 - 2*e_alpha_dt + 2*alpha_*dt*e_alpha_dt - 2*alpha_*dt + alpha_*alpha_*dt*dt);
    double t3 = 1.0 / (2.0 * alpha_ * alpha_ * alpha_) * (1.0 - e_alpha_dt*e_alpha_dt - 2*alpha_*dt*e_alpha_dt);
    double t4 = 1.0 / (2.0 * alpha_ * alpha_) * (e_alpha_dt*e_alpha_dt - 1 + 2*alpha_*dt);
    double t5 = 1.0 / (2.0 * alpha_) * (1 - e_alpha_dt*e_alpha_dt);

    // Populate the upper triangle of the symmetric Q matrix for one dimension
    Q_(0, 0) = t1; Q_(0, 2) = t2; Q_(0, 4) = t3;
                   Q_(2, 2) = t4; Q_(2, 4) = t5;
                                  Q_(4, 4) = t5; // Note: Q(4,4) uses t5, not t4
    
    // Fill in the other dimension
    Q_(1, 1) = Q_(0, 0); Q_(1, 3) = Q_(0, 2); Q_(1, 5) = Q_(0, 4);
                       Q_(3, 3) = Q_(2, 2); Q_(3, 5) = Q_(2, 4);
                                          Q_(5, 5) = Q_(4, 4);
    
    // Reflect the upper triangle to the lower triangle and scale by acceleration variance
    Q_ = Q_ + Q_.transpose().eval();
    Q_ *= sigma_a_sq_;


    // --- 3. Perform the state and covariance prediction ---
    x_hat_ = F_ * x_hat_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd& z) {
    Eigen::VectorXd y = z - H_ * x_hat_; // Measurement residual
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse(); // Kalman gain

    x_hat_ = x_hat_ + K * y;
    P_ = (I_ - K * H_) * P_;
}