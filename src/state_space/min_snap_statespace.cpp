#include "motion_planning/state_space/min_snap_statespace.hpp"

// Constructor
MinSnapStateSpace::MinSnapStateSpace(int dimension, double v_max, double a_max, double w_vel, double w_accel, double w_snap, unsigned int seed)
    : StateSpace(dimension),
      v_max_(v_max),
      a_max_(a_max),
      w_vel_(w_vel),
      w_accel_(w_accel),
      w_snap_(w_snap),
      seed_(seed) {
    std::srand(seed_);
}


double normalizeAngle(double angle);
// Steer function with free endpoints
Trajectory MinSnapStateSpace::steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
    const double T = from.tail<1>()[0] - to.tail<1>()[0];

    if (T <= 1e-6) return Trajectory{false};
    double spatial_dist = (from.head<3>() - to.head<3>()).norm();
    if (spatial_dist / T > v_max_) return Trajectory{false};

    auto basis = [&](int deriv, double tau) {
        Eigen::RowVectorXd r = Eigen::RowVectorXd::Zero(num_coeffs_);
        for (int i = deriv; i < num_coeffs_; ++i) {
            double coeff = 1.0;
            for (int k = 0; k < deriv; ++k) coeff *= (i - k);
            r(i) = coeff * std::pow(tau, i - deriv);
        }
        return r;
    };

    std::vector<Eigen::VectorXd> optimal_coeffs;
    Eigen::VectorXd final_velocities(num_axes_);
    Eigen::VectorXd final_accelerations(num_axes_);

    for (int axis = 0; axis < num_axes_; ++axis) {
        Eigen::MatrixXd H = calculateCombinedHessian(T, w_vel_, w_accel_, w_snap_);
        Eigen::VectorXd g = Eigen::VectorXd::Zero(num_coeffs_);

        const int neq = 2;
        Eigen::MatrixXd Aeq(neq, num_coeffs_);
        Aeq.row(0) = basis(0, 0.0); // from
        Aeq.row(1) = basis(0, 1.0); // to
        Eigen::VectorXd beq(neq);
        double start_pos = from(axis);
        double end_pos = to(axis);
        if (axis == 3) { end_pos = start_pos + normalizeAngle(end_pos - start_pos); }
        beq << start_pos, end_pos;

        const int K = 10;
        const int n_ineq_points = K + 1; // K intermediate points + 1 endpoint
        Eigen::MatrixXd Aineq(4 * n_ineq_points, num_coeffs_);
        Eigen::VectorXd ub_ineq(4 * n_ineq_points);
        
        int current_row = 0;
        for (int k = 0; k < n_ineq_points; ++k) {
            double tau = (k < K) ? (double(k + 1) / double(K + 1)) : 1.0;
            Eigen::RowVectorXd vrow = basis(1, tau) / T;
            Eigen::RowVectorXd arow = basis(2, tau) / (T * T);
            Aineq.row(current_row) = vrow;  ub_ineq(current_row++) = v_max_;
            Aineq.row(current_row) = -vrow; ub_ineq(current_row++) = v_max_;
            Aineq.row(current_row) = arow;  ub_ineq(current_row++) = a_max_;
            Aineq.row(current_row) = -arow; ub_ineq(current_row++) = a_max_;
        }
        Eigen::VectorXd lb_ineq = Eigen::VectorXd::Constant(Aineq.rows(), -1e20);

        Eigen::MatrixXd A(Aeq.rows() + Aineq.rows(), num_coeffs_);
        A << Aeq, Aineq;
        Eigen::VectorXd lbA(A.rows()), ubA(A.rows());
        lbA.head(Aeq.rows()) = beq; ubA.head(Aeq.rows()) = beq;
        lbA.tail(Aineq.rows()) = lb_ineq; ubA.tail(Aineq.rows()) = ub_ineq;

        qpOASES::SQProblem qp(num_coeffs_, A.rows());
        qpOASES::Options opts; opts.printLevel = qpOASES::PL_NONE;
        qp.setOptions(opts);
        int nWSR = 100;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_r = H;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_r = A;
        if (qp.init(H_r.data(), g.data(), A_r.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWSR) != qpOASES::SUCCESSFUL_RETURN) return Trajectory{false};
        
        Eigen::VectorXd p_optimal(num_coeffs_);
        if (qp.getPrimalSolution(p_optimal.data()) != qpOASES::SUCCESSFUL_RETURN) return Trajectory{false};
        
        final_velocities(axis) = (basis(1, 0.0) * p_optimal)(0) / T;      // Velocity at 'from' (child)
        final_accelerations(axis) = (basis(2, 0.0) * p_optimal)(0) / (T*T); // Acceleration at 'from' (child)
        optimal_coeffs.push_back(p_optimal);
    }
    
    Trajectory traj;
    traj.is_valid = true;
    traj.time_duration = T;
    traj.final_velocity = final_velocities;
    traj.final_acceleration = final_accelerations;
    
    const int num_path_points = 50;
    traj.path_points.reserve(num_path_points + 1);
    for(int i = 0; i <= num_path_points; ++i) {
        double tau = static_cast<double>(i) / num_path_points;
        Eigen::VectorXd point(from.size());
        for (int axis = 0; axis < num_axes_; ++axis) {
            point(axis) = (optimal_coeffs[axis].transpose() * basis(0, tau).transpose())(0);
            if (axis == 3) point(axis) = normalizeAngle(point(axis));
        }
        point(num_axes_) = from(num_axes_) - (T * tau);
        traj.path_points.push_back(point);
    }
    
    // // Reverse path points to be intuitive (parent -> child)
    // std::reverse(traj.path_points.begin(), traj.path_points.end());

    traj.geometric_distance = (from.head<3>() - to.head<3>()).norm();
    traj.cost = std::sqrt(T*T + traj.geometric_distance*traj.geometric_distance);
    traj.coeffs_per_axis = optimal_coeffs;

    return traj;
}

// Steer function with known derivatives at the PARENT ('to') node
// Trajectory MinSnapStateSpace::steer_with_initial(const Eigen::VectorXd& from, const Eigen::VectorXd& to, const Eigen::VectorXd& v0, const Eigen::VectorXd& a0) const {
Trajectory MinSnapStateSpace::steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to, const Eigen::VectorXd& v0, const Eigen::VectorXd& a0) const {
    const double T = from.tail<1>()[0] - to.tail<1>()[0];

    if (T <= 1e-6) return Trajectory{false};
    double spatial_dist = (from.head<3>() - to.head<3>()).norm();
    if (spatial_dist / T > v_max_) return Trajectory{false};

    auto basis = [&](int deriv, double tau) -> Eigen::RowVectorXd {
        Eigen::RowVectorXd r = Eigen::RowVectorXd::Zero(num_coeffs_);
        for (int i = deriv; i < num_coeffs_; ++i) {
            double c = 1.0; for (int k = 0; k < deriv; ++k) c *= double(i - k);
            r(i) = c * std::pow(tau, i - deriv);
        }
        return r;
    };

    Eigen::MatrixXd H_base = calculateCombinedHessian(T, w_vel_, w_accel_, w_snap_);
    
    std::vector<Eigen::VectorXd> coeffs_per_axis;
    coeffs_per_axis.reserve(num_axes_);
    
    const int K = 10;
    const int n_ineq_points = K; // Only check intermediate points
    Eigen::MatrixXd Aineq(4 * n_ineq_points, num_coeffs_);
    Eigen::VectorXd ub_ineq(4 * n_ineq_points);

    int current_row = 0;
    for (int k = 0; k < K; ++k) {
        double tau = double(k + 1) / double(K + 1);
        Eigen::RowVectorXd vrow = basis(1, tau) / T;
        Eigen::RowVectorXd arow = basis(2, tau) / (T * T);
        Aineq.row(current_row) = vrow;  ub_ineq(current_row++) = v_max_;
        Aineq.row(current_row) = -vrow; ub_ineq(current_row++) = v_max_;
        Aineq.row(current_row) = arow;  ub_ineq(current_row++) = a_max_;
        Aineq.row(current_row) = -arow; ub_ineq(current_row++) = a_max_;
    }
    Eigen::VectorXd lb_ineq = Eigen::VectorXd::Constant(Aineq.rows(), -1e20);

    for (int axis = 0; axis < num_axes_; ++axis) {
        bool solved = false;
        Eigen::VectorXd best_solution;
        double start_pos = from(axis);
        double end_pos = to(axis);
        if (axis == 3) { end_pos = start_pos + normalizeAngle(end_pos - start_pos); }
        Eigen::VectorXd g = Eigen::VectorXd::Zero(num_coeffs_);

        for (int attempt = 0; attempt < 3; ++attempt) {
            Eigen::MatrixXd Aeq; Eigen::VectorXd beq;
            if (attempt == 0) { // C^2
                Aeq.resize(4, num_coeffs_); beq.resize(4);
                Aeq.row(0)=basis(0,0.0); Aeq.row(1)=basis(0,1.0); Aeq.row(2)=basis(1,1.0)/T; Aeq.row(3)=basis(2,1.0)/(T*T);
                beq << start_pos, end_pos, v0(axis), a0(axis);
            } else if (attempt == 1) { // C^1
                Aeq.resize(3, num_coeffs_); beq.resize(3);
                Aeq.row(0)=basis(0,0.0); Aeq.row(1)=basis(0,1.0); Aeq.row(2)=basis(1,1.0)/T;
                beq << start_pos, end_pos, v0(axis);
            } else { // C^0
                Aeq.resize(2, num_coeffs_); beq.resize(2);
                Aeq.row(0)=basis(0,0.0); Aeq.row(1)=basis(0,1.0);
                beq << start_pos, end_pos;
            }

            Eigen::MatrixXd A(Aeq.rows() + Aineq.rows(), num_coeffs_);
            Eigen::VectorXd lbA(A.rows()), ubA(A.rows());
            A.topRows(Aeq.rows()) = Aeq; A.bottomRows(Aineq.rows()) = Aineq;
            lbA.head(Aeq.rows()) = beq; ubA.head(Aeq.rows()) = beq;
            lbA.tail(Aineq.rows()) = lb_ineq; ubA.tail(Aineq.rows()) = ub_ineq;

            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_r = H_base;
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_r = A;
            qpOASES::SQProblem qp(num_coeffs_, A.rows());
            qpOASES::Options opts; opts.printLevel = qpOASES::PL_NONE; qp.setOptions(opts);
            int nWSR = 300;
            if (qp.init(H_r.data(), g.data(), A_r.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWSR) == qpOASES::SUCCESSFUL_RETURN) {
                Eigen::VectorXd sol(num_coeffs_);
                if (qp.getPrimalSolution(sol.data()) == qpOASES::SUCCESSFUL_RETURN) {
                    best_solution = sol; solved = true; break;
                }
            }
        }
        if (!solved) return Trajectory{false};
        coeffs_per_axis.push_back(best_solution);
    }

    Trajectory result;
    result.is_valid = true;
    result.time_duration = T;

    Eigen::VectorXd final_vel(num_axes_), final_acc(num_axes_);
    for(int axis=0; axis<num_axes_; ++axis){
        final_vel(axis) = (basis(1, 0.0) * coeffs_per_axis[axis])(0) / T;      // Velocity at 'from'
        final_acc(axis) = (basis(2, 0.0) * coeffs_per_axis[axis])(0) / (T*T); // Acceleration at 'from'
    }
    result.final_velocity = final_vel;
    result.final_acceleration = final_acc;

    const int num_samples = 50;
    for (int i = 0; i <= num_samples; ++i) {
        double tau = double(i) / double(num_samples);
        Eigen::VectorXd pt(from.size());
        for (int axis = 0; axis < num_axes_; ++axis) {
            pt(axis) = (coeffs_per_axis[axis].transpose() * basis(0, tau).transpose())(0);
            if (axis == 3) pt(axis) = normalizeAngle(pt(axis));
        }
        pt(num_axes_) = from(num_axes_) - tau * T;
        result.path_points.push_back(pt);
    }
    
    // // Reverse path points to be intuitive (parent -> child) ---
    // std::reverse(result.path_points.begin(), result.path_points.end());
    
    result.geometric_distance = (from.head<3>() - to.head<3>()).norm();
    result.cost = std::sqrt(T*T + result.geometric_distance*result.geometric_distance);
    result.coeffs_per_axis = coeffs_per_axis;

    return result;
}

// Steer function with known derivatives at BOTH ends
// Trajectory MinSnapStateSpace::steer_with_initial_and_final(const Eigen::VectorXd& from, const Eigen::VectorXd& to, const Eigen::VectorXd& v0, const Eigen::VectorXd& a0, const Eigen::VectorXd& v1, const Eigen::VectorXd& a1) const {
Trajectory MinSnapStateSpace::steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to, const Eigen::VectorXd& v0, const Eigen::VectorXd& a0, const Eigen::VectorXd& v1, const Eigen::VectorXd& a1) const {
    const double T = from.tail<1>()[0] - to.tail<1>()[0];

    if (T <= 1e-6) return Trajectory{false};
    double spatial_dist = (from.head<3>() - to.head<3>()).norm();
    if (spatial_dist / T > v_max_) return Trajectory{false};

    auto basis = [&](int deriv, double tau) -> Eigen::RowVectorXd {
        Eigen::RowVectorXd r = Eigen::RowVectorXd::Zero(num_coeffs_);
        for (int i = deriv; i < num_coeffs_; ++i) {
            double c = 1.0; for (int k = 0; k < deriv; ++k) c *= (i - k);
            r(i) = c * std::pow(tau, i - deriv);
        }
        return r;
    };

    const int K = 10;
    Eigen::MatrixXd Aineq(4 * K, num_coeffs_);
    Eigen::VectorXd ub_ineq(4 * K);
    int current_row = 0;
    for (int k = 0; k < K; ++k) {
        double tau = static_cast<double>(k + 1) / (K + 1);
        Eigen::RowVectorXd vrow = basis(1, tau) / T;
        Eigen::RowVectorXd arow = basis(2, tau) / (T * T);
        Aineq.row(current_row) = vrow;  ub_ineq(current_row++) = v_max_;
        Aineq.row(current_row) = -vrow; ub_ineq(current_row++) = v_max_;
        Aineq.row(current_row) = arow;  ub_ineq(current_row++) = a_max_;
        Aineq.row(current_row) = -arow; ub_ineq(current_row++) = a_max_;
    }
    Eigen::VectorXd lb_ineq = Eigen::VectorXd::Constant(Aineq.rows(), -1e20);

    std::vector<Eigen::VectorXd> optimal_coeffs;
    optimal_coeffs.reserve(num_axes_);
    bool overall_success = true;

    for (int axis = 0; axis < num_axes_; ++axis) {
        Eigen::MatrixXd H = calculateCombinedHessian(T, w_vel_, w_accel_, w_snap_);
        Eigen::VectorXd g = Eigen::VectorXd::Zero(num_coeffs_);
        double start_pos = from(axis);
        double end_pos = to(axis);
        if (axis == 3) { end_pos = start_pos + normalizeAngle(end_pos - start_pos); }

        bool solved_axis = false;
        Eigen::VectorXd p_optimal_axis(num_coeffs_);
        
        for (int attempt = 0; attempt < 3; ++attempt) {
            Eigen::MatrixXd Aeq; Eigen::VectorXd beq;
            // Note: Relaxation is on the 'from' (child) state as per FMTX convention
            if (attempt == 0) { // C^2
                Aeq.resize(6, num_coeffs_); beq.resize(6);
                Aeq.row(0)=basis(0,0.0); Aeq.row(1)=basis(1,0.0)/T;     Aeq.row(2)=basis(2,0.0)/(T*T);
                Aeq.row(3)=basis(0,1.0); Aeq.row(4)=basis(1,1.0)/T;     Aeq.row(5)=basis(2,1.0)/(T*T);
                beq << start_pos, v0(axis), a0(axis), end_pos, v1(axis), a1(axis);
            } else if (attempt == 1) { // Relax a0
                Aeq.resize(5, num_coeffs_); beq.resize(5);
                Aeq.row(0)=basis(0,0.0); Aeq.row(1)=basis(1,0.0)/T;
                Aeq.row(2)=basis(0,1.0); Aeq.row(3)=basis(1,1.0)/T; Aeq.row(4)=basis(2,1.0)/(T*T);
                beq << start_pos, v0(axis), end_pos, v1(axis), a1(axis);
            } else { // Relax v0 and a0
                Aeq.resize(4, num_coeffs_); beq.resize(4);
                Aeq.row(0)=basis(0,0.0);
                Aeq.row(1)=basis(0,1.0); Aeq.row(2)=basis(1,1.0)/T; Aeq.row(3)=basis(2,1.0)/(T*T);
                beq << start_pos, end_pos, v1(axis), a1(axis);
            }
            
            Eigen::MatrixXd A(Aeq.rows() + Aineq.rows(), num_coeffs_);
            Eigen::VectorXd lbA(A.rows()), ubA(A.rows());
            A.topRows(Aeq.rows()) = Aeq; A.bottomRows(Aineq.rows()) = Aineq;
            lbA.head(Aeq.rows()) = beq; ubA.head(Aeq.rows()) = beq;
            lbA.tail(Aineq.rows()) = lb_ineq; ubA.tail(Aineq.rows()) = ub_ineq;

            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_r = H;
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_r = A;
            qpOASES::SQProblem qp(num_coeffs_, A.rows());
            qpOASES::Options opts; opts.printLevel = qpOASES::PL_NONE; qp.setOptions(opts);
            int nWSR = 150;
            if (qp.init(H_r.data(), g.data(), A_r.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWSR) == qpOASES::SUCCESSFUL_RETURN) {
                if (qp.getPrimalSolution(p_optimal_axis.data()) == qpOASES::SUCCESSFUL_RETURN) {
                    solved_axis = true; break;
                }
            }
        }
        if (!solved_axis) { overall_success = false; break; }
        optimal_coeffs.push_back(p_optimal_axis);
    }
    if (!overall_success) return Trajectory{false};

    Trajectory result_traj;
    result_traj.is_valid = true;
    result_traj.time_duration = T;
    
    Eigen::VectorXd final_v(num_axes_), final_a(num_axes_);
    for(int axis=0; axis<num_axes_; ++axis){
        final_v(axis) = (basis(1, 0.0) * optimal_coeffs[axis])(0) / T;
        final_a(axis) = (basis(2, 0.0) * optimal_coeffs[axis])(0) / (T*T);
    }
    result_traj.final_velocity = final_v;
    result_traj.final_acceleration = final_a;

    const int num_path_points = 50;
    result_traj.path_points.reserve(num_path_points + 1);
    for (int i = 0; i <= num_path_points; ++i) {
        double tau = static_cast<double>(i) / num_path_points;
        Eigen::VectorXd point(from.size());
        for (int axis = 0; axis < num_axes_; ++axis) {
            point(axis) = (optimal_coeffs[axis].transpose() * basis(0, tau).transpose())(0);
            if (axis == 3) point(axis) = normalizeAngle(point(axis));
        }
        point(num_axes_) = from(num_axes_) - (T * tau);
        result_traj.path_points.push_back(point);
    }
    
    // std::reverse(result_traj.path_points.begin(), result_traj.path_points.end());

    result_traj.geometric_distance = (from.head<3>() - to.head<3>()).norm();
    result_traj.cost = std::sqrt(T*T + result_traj.geometric_distance*result_traj.geometric_distance);
    result_traj.coeffs_per_axis = optimal_coeffs;
    
    return result_traj;
}


Eigen::MatrixXd MinSnapStateSpace::calculateCombinedHessian(double T, double w_vel, double w_accel, double w_snap) const {
    Eigen::MatrixXd H_total = Eigen::MatrixXd::Zero(num_coeffs_, num_coeffs_);

    if (w_vel > 1e-6) {
        Eigen::MatrixXd H_vel = Eigen::MatrixXd::Zero(num_coeffs_, num_coeffs_);
        for (int i = 1; i < num_coeffs_; ++i) {
            for (int j = 1; j < num_coeffs_; ++j) {
                H_vel(i, j) = (double(i * j) / (i + j - 1.0));
            }
        }
        H_total += w_vel * H_vel / T;
    }
    if (w_accel > 1e-6) {
        Eigen::MatrixXd H_accel = Eigen::MatrixXd::Zero(num_coeffs_, num_coeffs_);
        for (int i = 2; i < num_coeffs_; ++i) {
            for (int j = 2; j < num_coeffs_; ++j) {
                H_accel(i, j) = (i * (i - 1.) * j * (j - 1.)) / (i + j - 3.0);
            }
        }
        H_total += w_accel * H_accel / std::pow(T, 3);
    }
    if (w_snap > 1e-6) {
        Eigen::MatrixXd H_snap = Eigen::MatrixXd::Zero(num_coeffs_, num_coeffs_);
        for (int i = 4; i < num_coeffs_; ++i) {
            for (int j = 4; j < num_coeffs_; ++j) {
                double ci = i * (i - 1.) * (i - 2.) * (i - 3.);
                double cj = j * (j - 1.) * (j - 2.) * (j - 3.);
                H_snap(i, j) = (ci * cj) / (i + j - 7.0);
            }
        }
        H_total += w_snap * H_snap / std::pow(T, 7);
    }
    H_total += 1e-9 * Eigen::MatrixXd::Identity(num_coeffs_, num_coeffs_);
    return H_total;
}
//////////////////////////////////

// Steer function with known derivatives at the PARENT ('to') node
Trajectory MinSnapStateSpace::steer_with_initial(const Eigen::VectorXd& from, const Eigen::VectorXd& to, const Eigen::VectorXd& v0, const Eigen::VectorXd& a0) const {
    const double T = from.tail<1>()[0] - to.tail<1>()[0];

    if (T <= 1e-6) return Trajectory{false};
    double spatial_dist = (from.head<3>() - to.head<3>()).norm();
    if (spatial_dist / T > v_max_) return Trajectory{false};

    auto basis = [&](int deriv, double tau) -> Eigen::RowVectorXd {
        Eigen::RowVectorXd r = Eigen::RowVectorXd::Zero(num_coeffs_);
        for (int i = deriv; i < num_coeffs_; ++i) {
            double c = 1.0; for (int k = 0; k < deriv; ++k) c *= double(i - k);
            r(i) = c * std::pow(tau, i - deriv);
        }
        return r;
    };

    Eigen::MatrixXd H_base = calculateCombinedHessian(T, w_vel_, w_accel_, w_snap_);
    
    std::vector<Eigen::VectorXd> coeffs_per_axis;
    coeffs_per_axis.reserve(num_axes_);
    
    const int K = 10;
    const int n_ineq_points = K; // Only check intermediate points
    Eigen::MatrixXd Aineq(4 * n_ineq_points, num_coeffs_);
    Eigen::VectorXd ub_ineq(4 * n_ineq_points);

    int current_row = 0;
    for (int k = 0; k < K; ++k) {
        double tau = double(k + 1) / double(K + 1);
        Eigen::RowVectorXd vrow = basis(1, tau) / T;
        Eigen::RowVectorXd arow = basis(2, tau) / (T * T);
        Aineq.row(current_row) = vrow;  ub_ineq(current_row++) = v_max_;
        Aineq.row(current_row) = -vrow; ub_ineq(current_row++) = v_max_;
        Aineq.row(current_row) = arow;  ub_ineq(current_row++) = a_max_;
        Aineq.row(current_row) = -arow; ub_ineq(current_row++) = a_max_;
    }
    Eigen::VectorXd lb_ineq = Eigen::VectorXd::Constant(Aineq.rows(), -1e20);

    for (int axis = 0; axis < num_axes_; ++axis) {
        bool solved = false;
        Eigen::VectorXd best_solution;
        double start_pos = from(axis);
        double end_pos = to(axis);
        if (axis == 3) { end_pos = start_pos + normalizeAngle(end_pos - start_pos); }
        Eigen::VectorXd g = Eigen::VectorXd::Zero(num_coeffs_);

        for (int attempt = 0; attempt < 3; ++attempt) {
            Eigen::MatrixXd Aeq; Eigen::VectorXd beq;
            if (attempt == 0) { // C^2
                Aeq.resize(4, num_coeffs_); beq.resize(4);
                Aeq.row(0)=basis(0,0.0); Aeq.row(1)=basis(0,1.0); Aeq.row(2)=basis(1,1.0)/T; Aeq.row(3)=basis(2,1.0)/(T*T);
                beq << start_pos, end_pos, v0(axis), a0(axis);
            } else if (attempt == 1) { // C^1
                Aeq.resize(3, num_coeffs_); beq.resize(3);
                Aeq.row(0)=basis(0,0.0); Aeq.row(1)=basis(0,1.0); Aeq.row(2)=basis(1,1.0)/T;
                beq << start_pos, end_pos, v0(axis);
            } else { // C^0
                Aeq.resize(2, num_coeffs_); beq.resize(2);
                Aeq.row(0)=basis(0,0.0); Aeq.row(1)=basis(0,1.0);
                beq << start_pos, end_pos;
            }

            Eigen::MatrixXd A(Aeq.rows() + Aineq.rows(), num_coeffs_);
            Eigen::VectorXd lbA(A.rows()), ubA(A.rows());
            A.topRows(Aeq.rows()) = Aeq; A.bottomRows(Aineq.rows()) = Aineq;
            lbA.head(Aeq.rows()) = beq; ubA.head(Aeq.rows()) = beq;
            lbA.tail(Aineq.rows()) = lb_ineq; ubA.tail(Aineq.rows()) = ub_ineq;

            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_r = H_base;
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_r = A;
            qpOASES::SQProblem qp(num_coeffs_, A.rows());
            qpOASES::Options opts; opts.printLevel = qpOASES::PL_NONE; qp.setOptions(opts);
            int nWSR = 300;
            if (qp.init(H_r.data(), g.data(), A_r.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWSR) == qpOASES::SUCCESSFUL_RETURN) {
                Eigen::VectorXd sol(num_coeffs_);
                if (qp.getPrimalSolution(sol.data()) == qpOASES::SUCCESSFUL_RETURN) {
                    best_solution = sol; solved = true; break;
                }
            }
        }
        if (!solved) return Trajectory{false};
        coeffs_per_axis.push_back(best_solution);
    }

    Trajectory result;
    result.is_valid = true;
    result.time_duration = T;

    Eigen::VectorXd final_vel(num_axes_), final_acc(num_axes_);
    for(int axis=0; axis<num_axes_; ++axis){
        final_vel(axis) = (basis(1, 0.0) * coeffs_per_axis[axis])(0) / T;      // Velocity at 'from'
        final_acc(axis) = (basis(2, 0.0) * coeffs_per_axis[axis])(0) / (T*T); // Acceleration at 'from'
    }
    result.final_velocity = final_vel;
    result.final_acceleration = final_acc;

    const int num_samples = 50;
    for (int i = 0; i <= num_samples; ++i) {
        double tau = double(i) / double(num_samples);
        Eigen::VectorXd pt(from.size());
        for (int axis = 0; axis < num_axes_; ++axis) {
            pt(axis) = (coeffs_per_axis[axis].transpose() * basis(0, tau).transpose())(0);
            if (axis == 3) pt(axis) = normalizeAngle(pt(axis));
        }
        pt(num_axes_) = from(num_axes_) - tau * T;
        result.path_points.push_back(pt);
    }
    
    // // Reverse path points to be intuitive (parent -> child) ---
    // std::reverse(result.path_points.begin(), result.path_points.end());
    
    result.geometric_distance = (from.head<3>() - to.head<3>()).norm();
    result.cost = std::sqrt(T*T + result.geometric_distance*result.geometric_distance);
    result.coeffs_per_axis = coeffs_per_axis;

    return result;
}

// Steer function with known derivatives at BOTH ends
Trajectory MinSnapStateSpace::steer_with_initial_and_final(const Eigen::VectorXd& from, const Eigen::VectorXd& to, const Eigen::VectorXd& v0, const Eigen::VectorXd& a0, const Eigen::VectorXd& v1, const Eigen::VectorXd& a1) const {
    const double T = from.tail<1>()[0] - to.tail<1>()[0];

    if (T <= 1e-6) return Trajectory{false};
    double spatial_dist = (from.head<3>() - to.head<3>()).norm();
    if (spatial_dist / T > v_max_) return Trajectory{false};

    auto basis = [&](int deriv, double tau) -> Eigen::RowVectorXd {
        Eigen::RowVectorXd r = Eigen::RowVectorXd::Zero(num_coeffs_);
        for (int i = deriv; i < num_coeffs_; ++i) {
            double c = 1.0; for (int k = 0; k < deriv; ++k) c *= (i - k);
            r(i) = c * std::pow(tau, i - deriv);
        }
        return r;
    };

    const int K = 10;
    Eigen::MatrixXd Aineq(4 * K, num_coeffs_);
    Eigen::VectorXd ub_ineq(4 * K);
    int current_row = 0;
    for (int k = 0; k < K; ++k) {
        double tau = static_cast<double>(k + 1) / (K + 1);
        Eigen::RowVectorXd vrow = basis(1, tau) / T;
        Eigen::RowVectorXd arow = basis(2, tau) / (T * T);
        Aineq.row(current_row) = vrow;  ub_ineq(current_row++) = v_max_;
        Aineq.row(current_row) = -vrow; ub_ineq(current_row++) = v_max_;
        Aineq.row(current_row) = arow;  ub_ineq(current_row++) = a_max_;
        Aineq.row(current_row) = -arow; ub_ineq(current_row++) = a_max_;
    }
    Eigen::VectorXd lb_ineq = Eigen::VectorXd::Constant(Aineq.rows(), -1e20);

    std::vector<Eigen::VectorXd> optimal_coeffs;
    optimal_coeffs.reserve(num_axes_);
    bool overall_success = true;

    for (int axis = 0; axis < num_axes_; ++axis) {
        Eigen::MatrixXd H = calculateCombinedHessian(T, w_vel_, w_accel_, w_snap_);
        Eigen::VectorXd g = Eigen::VectorXd::Zero(num_coeffs_);
        double start_pos = from(axis);
        double end_pos = to(axis);
        if (axis == 3) { end_pos = start_pos + normalizeAngle(end_pos - start_pos); }

        bool solved_axis = false;
        Eigen::VectorXd p_optimal_axis(num_coeffs_);
        
        for (int attempt = 0; attempt < 3; ++attempt) {
            Eigen::MatrixXd Aeq; Eigen::VectorXd beq;
            // Note: Relaxation is on the 'from' (child) state as per FMTX convention
            if (attempt == 0) { // C^2
                Aeq.resize(6, num_coeffs_); beq.resize(6);
                Aeq.row(0)=basis(0,0.0); Aeq.row(1)=basis(1,0.0)/T;     Aeq.row(2)=basis(2,0.0)/(T*T);
                Aeq.row(3)=basis(0,1.0); Aeq.row(4)=basis(1,1.0)/T;     Aeq.row(5)=basis(2,1.0)/(T*T);
                beq << start_pos, v0(axis), a0(axis), end_pos, v1(axis), a1(axis);
            } else if (attempt == 1) { // Relax a0
                Aeq.resize(5, num_coeffs_); beq.resize(5);
                Aeq.row(0)=basis(0,0.0); Aeq.row(1)=basis(1,0.0)/T;
                Aeq.row(2)=basis(0,1.0); Aeq.row(3)=basis(1,1.0)/T; Aeq.row(4)=basis(2,1.0)/(T*T);
                beq << start_pos, v0(axis), end_pos, v1(axis), a1(axis);
            } else { // Relax v0 and a0
                Aeq.resize(4, num_coeffs_); beq.resize(4);
                Aeq.row(0)=basis(0,0.0);
                Aeq.row(1)=basis(0,1.0); Aeq.row(2)=basis(1,1.0)/T; Aeq.row(3)=basis(2,1.0)/(T*T);
                beq << start_pos, end_pos, v1(axis), a1(axis);
            }
            
            Eigen::MatrixXd A(Aeq.rows() + Aineq.rows(), num_coeffs_);
            Eigen::VectorXd lbA(A.rows()), ubA(A.rows());
            A.topRows(Aeq.rows()) = Aeq; A.bottomRows(Aineq.rows()) = Aineq;
            lbA.head(Aeq.rows()) = beq; ubA.head(Aeq.rows()) = beq;
            lbA.tail(Aineq.rows()) = lb_ineq; ubA.tail(Aineq.rows()) = ub_ineq;

            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_r = H;
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_r = A;
            qpOASES::SQProblem qp(num_coeffs_, A.rows());
            qpOASES::Options opts; opts.printLevel = qpOASES::PL_NONE; qp.setOptions(opts);
            int nWSR = 150;
            if (qp.init(H_r.data(), g.data(), A_r.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWSR) == qpOASES::SUCCESSFUL_RETURN) {
                if (qp.getPrimalSolution(p_optimal_axis.data()) == qpOASES::SUCCESSFUL_RETURN) {
                    solved_axis = true; break;
                }
            }
        }
        if (!solved_axis) { overall_success = false; break; }
        optimal_coeffs.push_back(p_optimal_axis);
    }
    if (!overall_success) return Trajectory{false};

    Trajectory result_traj;
    result_traj.is_valid = true;
    result_traj.time_duration = T;
    
    Eigen::VectorXd final_v(num_axes_), final_a(num_axes_);
    for(int axis=0; axis<num_axes_; ++axis){
        final_v(axis) = (basis(1, 0.0) * optimal_coeffs[axis])(0) / T;
        final_a(axis) = (basis(2, 0.0) * optimal_coeffs[axis])(0) / (T*T);
    }
    result_traj.final_velocity = final_v;
    result_traj.final_acceleration = final_a;

    const int num_path_points = 50;
    result_traj.path_points.reserve(num_path_points + 1);
    for (int i = 0; i <= num_path_points; ++i) {
        double tau = static_cast<double>(i) / num_path_points;
        Eigen::VectorXd point(from.size());
        for (int axis = 0; axis < num_axes_; ++axis) {
            point(axis) = (optimal_coeffs[axis].transpose() * basis(0, tau).transpose())(0);
            if (axis == 3) point(axis) = normalizeAngle(point(axis));
        }
        point(num_axes_) = from(num_axes_) - (T * tau);
        result_traj.path_points.push_back(point);
    }
    
    // std::reverse(result_traj.path_points.begin(), result_traj.path_points.end());

    result_traj.geometric_distance = (from.head<3>() - to.head<3>()).norm();
    result_traj.cost = std::sqrt(T*T + result_traj.geometric_distance*result_traj.geometric_distance);
    result_traj.coeffs_per_axis = optimal_coeffs;
    
    return result_traj;
}






/////////////////////////////////
// For state types, we can just reuse EuclideanState.
std::shared_ptr<State> MinSnapStateSpace::addState(const Eigen::VectorXd& value) {
    return StateSpace::addState(std::make_shared<EuclideanState>(value));
}

// The distance function is used by the KD-tree to find neighbors.
double MinSnapStateSpace::distance(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2) const {
    Eigen::ArrayXd diff = state1->getValue().array() - state2->getValue().array();
    return (diff).matrix().norm();
}

// Uniform sampling works the same way as in the Euclidean space.
std::shared_ptr<State> MinSnapStateSpace::sampleUniform(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds) {
    if (min_bounds.size() != dimension_ || max_bounds.size() != dimension_) {
        throw std::invalid_argument("Bounds vectors must match the state space dimension.");
    }
    Eigen::VectorXd values(dimension_);
    for (int i = 0; i < dimension_; ++i) {
        double random_coeff = static_cast<double>(rand()) / RAND_MAX;
        values[i] = min_bounds[i] + (max_bounds[i] - min_bounds[i]) * random_coeff;
    }
    return StateSpace::addState(std::make_shared<EuclideanState>(values));
}

bool MinSnapStateSpace::isValid(const std::shared_ptr<State>& state) const {
    return true;
}

std::shared_ptr<State> MinSnapStateSpace::sampleUniform(double min, double max) {
    throw std::runtime_error("This sampleUniform overload is not supported in MinSnapStateSpace. Use bounds vector.");
}

void MinSnapStateSpace::sampleUniform(double min, double max, int k) {
    throw std::runtime_error("This sampleUniform overload is not supported in MinSnapStateSpace. Use bounds vector.");
}

std::shared_ptr<State> MinSnapStateSpace::interpolate(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2, double t) const {
     Eigen::VectorXd interpolated_val = state1->getValue() + t * (state2->getValue() - state1->getValue());
     return std::make_shared<EuclideanState>(interpolated_val);
}