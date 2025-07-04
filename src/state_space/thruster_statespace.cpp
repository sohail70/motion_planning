#include "motion_planning/state_space/thruster_statespace.hpp"
#include <stdexcept>
#include <iomanip> // For std::fixed, std::setprecision

ThrusterSteerStateSpace::ThrusterSteerStateSpace(int dimension, double max_acceleration)
    : StateSpace(dimension), max_acceleration_(max_acceleration) {
    // For 3D position and 3D velocity, plus time, dimension is 7.
    // Ensure dimension is 2*D_spatial + 1.
    if (dimension <= 1 || (dimension - 1) % 2 != 0) {
        throw std::invalid_argument("ThrusterStateSpace dimension must be 2*D_spatial + 1 (e.g., 7 for 3D pos/vel/time).");
    }
    // Set up weights for KD-Tree distance, if desired. Otherwise, unit weights.
    // These weights would be for: [x,y,z,vx,vy,vz,t]
    weights_.resize(dimension_);
    weights_.setOnes(); // Default to unit weights
    // Example: Emphasize position, deemphasize velocity and time
    // if (dimension_ == 7) {
    //     weights_ << 1.0, 1.0, 1.0,  0.5, 0.5, 0.5,  0.1; 
    // }
}

std::shared_ptr<State> ThrusterSteerStateSpace::addState(const Eigen::VectorXd& value) {
    return StateSpace::addState(std::make_shared<EuclideanState>(value));
}

std::shared_ptr<State> ThrusterSteerStateSpace::sampleUniform(double min, double max) {
    Eigen::VectorXd values = Eigen::VectorXd::Random(dimension_);
    values = min + (max - min) * (values.array() + 1) / 2;
    return StateSpace::addState(std::make_shared<EuclideanState>(values));
}

void ThrusterSteerStateSpace::sampleUniform(double min, double max, int k) {
    // Not directly used by FMTX. Implement if needed for other planners.
    // For now, this samples `k` states and adds them.
    for (int i = 0; i < k; ++i) {
        // sampleUniform(min, max);
    }
}

std::shared_ptr<State> ThrusterSteerStateSpace::sampleUniform(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds) {
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

double ThrusterSteerStateSpace::distance(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2) const {
    // This is the distance for the KD-Tree. For Thruster systems,
    // a simple Euclidean distance in the full state space is common, possibly weighted.
    Eigen::VectorXd diff = state1->getValue() - state2->getValue();
    return diff.cwiseProduct(weights_).norm();
}

std::shared_ptr<State> ThrusterSteerStateSpace::interpolate(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2, double t) const {
    // For visualization or debugging, a linear interpolation in the state space
    // can be used, but the true path is given by the steer function.
    Eigen::VectorXd interpolated = state1->getValue() + t * (state2->getValue() - state1->getValue());
    return std::make_shared<EuclideanState>(interpolated);
}

bool ThrusterSteerStateSpace::isValid(const std::shared_ptr<State>& state) const {
    // Add validation logic if states can be invalid (e.g., outside bounds, in obstacles).
    return true; // Assume always valid if no specific constraints are given
}

// Helper to extract spatial position and velocity from a state vector that includes time
Eigen::VectorXd ThrusterSteerStateSpace::getSpatialPosition(const Eigen::VectorXd& state) const {
    return state.head((dimension_ - 1) / 2); // First D_spatial dimensions
}

Eigen::VectorXd ThrusterSteerStateSpace::getSpatialVelocity(const Eigen::VectorXd& state) const {
    return state.segment((dimension_ - 1) / 2, (dimension_ - 1) / 2); // Next D_spatial dimensions
}

Eigen::VectorXd ThrusterSteerStateSpace::steering1D(double x_start, double x_end, double v_start, double v_end, double t_start, double t_end, double a_max) const {
    
    // --- Initial Setup and Pre-computation ---
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n--- 1D Steer: x0=" << x_start << ", x1=" << x_end << ", v0=" << v_start << ", v1=" << v_end << ", t0=" << t_start << ", t1=" << t_end << ", a_max=" << a_max << std::endl;

    double orig_x_start = x_start, orig_x_end = x_end, orig_v_start = v_start, orig_v_end = v_end;

    double delta_t = t_end - t_start;
    const double EPS = 1e-9;

    if (delta_t < EPS) { 
        std::cout << "  Delta_t non-positive." << std::endl;
        return Eigen::VectorXd::Constant(5, std::numeric_limits<double>::quiet_NaN()); 
    }

    // we assume that the starting velocity is positive, if not then just need to
    // flip signs on all x and v and then solve, and then flip signs back
    bool flip_flag = false;
    if (v_start < 0) { 
        std::cout << "  DEBUG: Flipping problem (v_start negative)." << std::endl;
        flip_flag = true;
        x_start = -x_start;
        x_end = -x_end;
        v_start = -v_start;
        v_end = -v_end;
    }
    double delta_x = x_end - x_start;
    // solve everything for between 0 and delta_t, then add in

    double t_hat = std::abs(v_end - v_start) / a_max;
    // first make sure that we have enough time to do the required velocity
    if (t_hat > delta_t + EPS) {
        std::cout << "  Not enough time for velocity change." << std::endl;
        return Eigen::VectorXd::Constant(5, std::numeric_limits<double>::quiet_NaN()); 
    }

    double v_max_val = (delta_t * a_max + v_start + v_end) / 2.0; 
    double v_min_val = (v_start + v_end - delta_t * a_max) / 2.0; 

    double t1 = NAN, t2 = NAN, a1 = NAN, a2 = NAN, v_coast = NAN;

    // --- Case A: v_end >= 0 (after potential flip) both start and end velocity is positive ---
    if (v_end >= -EPS) { 
        std::cout << "  DEBUG: Entering Overall Case A (v_end >= 0).\n";
        double tau_1 = (v_start * v_start) / (2.0 * a_max); // distance to v = 0 from start
        double t_a = v_start / a_max;  // time to zero v from start
        double tau_2 = (v_end * v_end) / (2.0 * a_max);  // magnitude of distance from v = 0 to goal
        double t_b = delta_t - v_end / a_max; // -time to zero v from goal

        // === START OF CORRECTED STRUCTURE ===
        // first few are for non-overlapping tau triangles and second few are for overlapping tau triangles
        if (t_a <= t_b) {
            std::cout << "  DEBUG: Case A, Scenario 1 (t_a <= t_b).\n";
            // max and min positins that can be reached at t_end
            double delta_x_max = (delta_t + t_a + delta_t - t_b) * v_max_val / 2.0 - tau_1 - tau_2;
            double delta_x_min = (delta_t - (delta_t-t_b) - t_a) * v_min_val / 2.0 + tau_1 + tau_2;

            if (x_start + delta_x_max + EPS < x_end || x_start + delta_x_min - EPS > x_end) {
                std::cout << "  DEBUG: Unreachable in this scenario. End is too high or end is too low to get to in time \n";
                return Eigen::VectorXd::Constant(5, std::numeric_limits<double>::quiet_NaN());
            }

            if (delta_x < tau_1 + tau_2) {
                // sub-case 1, we need negative velocity at some point
                double z = std::sqrt((delta_x - delta_x_min) * a_max);
                double t_delta_min_v = (t_a + t_b) / 2.0;
                v_coast = v_min_val + z; // velocity to coast at
                t1 = t_delta_min_v - z / a_max; // time at start of coast
                t2 = t_delta_min_v + z / a_max; // time at end of coast
            } else if (delta_x < delta_t * std::min(v_start, v_end) + std::pow(v_start - v_end, 2) / (2.0 * a_max)) {
                // sub-case 2, reduce velocitym wait, increase velocity
                double z = std::sqrt((delta_x - delta_x_min) * a_max);
                double t_delta_min_v = (t_a + t_b) / 2.0;
                v_coast = v_min_val + z; // velocity to coast at
                t1 = t_delta_min_v - z / a_max; // time at start of coast
                t2 = t_delta_min_v + z / a_max; // time at end of coast
            } else if (delta_x < delta_t * std::max(v_start, v_end) - std::pow(v_start - v_end, 2) / (2.0 * a_max)) {
                // sub-case 3, reduce velocity, wait, reduce velocity OR
                //             increase velocity, wait, increase velocity
                if (tau_1 < tau_2) {
                    v_coast = (delta_x + tau_1 - tau_2) / (t_b + t_a);
                    t1 = (v_coast - v_start) / a_max;
                    t2 = t_b + v_coast / a_max;
                } else {
                    v_coast = (delta_x - tau_1 + tau_2) / (2 * delta_t - t_a - t_b);
                    t1 = (v_start - v_coast) / a_max;
                    t2 = delta_t - (v_coast - v_end) / a_max;
                }
            } else {
                // subcase 4 , increase velocity, wait, decrease velocity
                double z = std::sqrt((delta_x_max - delta_x) * a_max);
                double t_delta_max_v = (v_max_val - v_start) / a_max;
                v_coast = v_max_val - z; // velocity to coast at
                t1 = t_delta_max_v - z / a_max; // time at start of coast
                t2 = t_delta_max_v + z / a_max; // time at end of coast
            }
        } else { // Case A, Scenario 2 (t_b < t_a)
            std::cout << "  DEBUG: Case A, Scenario 2 (t_b < t_a).\n";
            double t_v_max = (v_max_val - v_start) / a_max;
            double delta_x_max = v_max_val * delta_t - std::pow(v_max_val - v_start, 2) / (2*a_max) - std::pow(v_max_val - v_end, 2) / (2*a_max);

            if (delta_x > delta_x_max + EPS) {
                std::cout << "  DEBUG: Unreachable (end is too high to get to in time).\n";
                return Eigen::VectorXd::Constant(5, std::numeric_limits<double>::quiet_NaN());
            }

            double delta_x_min = -(delta_t - (delta_t-t_b) - t_a) * v_min_val / 2.0 + tau_1 + tau_2;
            if(delta_x < delta_x_min - EPS) {
                std::cout << "  DEBUG: Unreachable (end is too low to get to in time).\n";
                return Eigen::VectorXd::Constant(5, std::numeric_limits<double>::quiet_NaN());
            }
            
            double tau_3 = v_min_val * v_min_val / a_max;
            if (delta_x < tau_1 + tau_2 - tau_3) {
                // sub-case 1, we need negative velocity at some point
                std::cout<< " DEBUG: Need negative velocity and cannot attain it. \n" ;
                return Eigen::VectorXd::Constant(5, std::numeric_limits<double>::quiet_NaN());
            } else if ((tau_1 < tau_2 && delta_x <= tau_1 + tau_2 + t_b * v_start) || (tau_2 <= tau_1 && delta_x <= tau_1 + tau_2 + (delta_t - t_a) * v_end)) {
                // sub-case 2 reduce velocity, wait, increase velocity
                // but final velocity is more
                double z = std::sqrt((delta_x - tau_1 - tau_2 + tau_3) * a_max);
                v_coast = (t_a - t_b) / 2.0 * a_max + z;
                t1 = (t_a + t_b) / 2.0 - z / a_max;
                t2 = (t_a + t_b) / 2.0 + z / a_max;
            } else if (delta_x < delta_t * std::max(v_end, v_start) - std::pow(v_end-v_start, 2)/(2*a_max)) {
                // sub-case 3, reduce velocity, wait, reduce velocity OR
                //             increase velocity, wait, increase velocity
                if(tau_1 < tau_2) {
                    double x_i = t_b + v_start/a_max;
                    double z = (delta_x - tau_1 - tau_2 - v_start*t_b) / x_i;
                    v_coast = v_start + z;
                    t1 = z/a_max;
                    t2 = z/a_max + x_i;
                } else {
                    double x_i = delta_t - (t_a - v_end/a_max);
                    double z = (delta_x - tau_1 - tau_2 - (delta_t-t_a)*v_end) / x_i;
                    v_coast = v_end + z;
                    t1 = delta_t - z/a_max - x_i;
                    t2 = delta_t - z/a_max;
                }
            } else {
                // sub-case 4, increase velocity, wait, decrease velocity
                double z = std::sqrt((delta_x_max - delta_x) * a_max);
                v_coast = v_max_val - z;
                t1 = t_v_max - z/a_max;
                t2 = t_v_max + z/a_max;
            }
        }
        // === END OF CORRECTED STRUCTURE ===

    } 
    // --- Case B: v_start > 0 and  v_end < 0 (after potential flip) ---
    else { 
        std::cout << "  DEBUG: Entering Overall Case B (v_start > 0 and v_end < 0).\n";
        double tau_1 = (v_start * v_start) / (2.0 * a_max); 
        double t_a = v_start / a_max; 
        double tau_2_abs_v_end = (v_end * v_end) / (2.0 * a_max); 
        double t_b = delta_t + v_end / a_max; // -time to zero v from goal 
            
        // first calculte the max and min positions that can be reached at t_end
        // to make sure that a solution exists.
        
        double delta_x_max = (t_a + t_b) * v_max_val / 2.0 - tau_1 - tau_2_abs_v_end; 
        double delta_x_min = (2.0 * delta_t - t_b - t_a) * v_min_val / 2.0 + tau_1 + tau_2_abs_v_end;

        if (x_start + delta_x_max + EPS < x_end || x_start + delta_x_min - EPS > x_end) {
            std::cout << "  DEBUG: Unreachable in this scenario. End is too high or end is to low to get to in time. \n";
            return Eigen::VectorXd::Constant(5, std::numeric_limits<double>::quiet_NaN());
        }
        // The problem can be solved, so figure out which one of the 4 basic sub-case we need to solve
         
        if (delta_x < tau_1 - tau_2_abs_v_end + v_end * (t_b - t_a)) { 
            // sub-case 1: decelerate to negative velocity, wait, then accelerate
            // (to still negative velocity)
            double z = std::sqrt((delta_x - delta_x_min) * a_max);
            double t_delta_min_v = (v_start - v_min_val) / a_max;
            v_coast = v_min_val + z; // velocity to coast at
            t1 = t_delta_min_v - z / a_max; // time at start of coast
            t2 = t_delta_min_v + z / a_max; // time at end of coast
        } else if (delta_x < tau_1 - tau_2_abs_v_end + v_start * (delta_t + (v_end - v_start) / a_max)) {
            // sub-case 2 and 3: decelerate to negative/positive velocity, wait, then
            // decelerate more
            v_coast = (delta_x - tau_1 + tau_2_abs_v_end) / (t_b - t_a); //velocity to coast at
            t1 = t_a - v_coast / a_max; // time at start of coast
            t2 = t_b - v_coast / a_max; // time at end of coast
        } else { 
            // sub-case 4: accelerate, wait, decelerate
            double z = std::sqrt((delta_x_max - delta_x) * a_max);
            double t_delta_max_v = (v_max_val - v_start) / a_max;
            v_coast = v_max_val - z; // velocity to coast at
            t1 = t_delta_max_v - z / a_max; // time at start of coast
            t2 = t_delta_max_v + z / a_max; // time at end of coast
        }
    }

    // --- Final assignment of accelerations and return value ---
    // a_1 is acceleration from t_start to t_1
    // a_2 is acceleration from t_2 to t_end
    // (acceleration from t_1 to t_2 is 0)
    // v_coast is the velocity we coast at from t_1 to t_2
    double dt1 = t1;
    double dt2 = delta_t - t2;
    if (std::abs(dt1) < EPS) dt1 = EPS; // Avoid division by zero
    if (std::abs(dt2) < EPS) dt2 = EPS;

    a1 = (v_coast - v_start) / dt1;
    a2 = (v_end - v_coast) / dt2;
    
    // Clamp accelerations to their max value
    a1 = std::max(-a_max, std::min(a_max, a1));
    a2 = std::max(-a_max, std::min(a_max, a2));

    // Final checks before assigning result
    if (t1 < -EPS || t1 > delta_t + EPS || t2 < -EPS || t2 > delta_t + EPS || t1 > t2 + EPS) {
        std::cout << "  DEBUG: Final time check failed *before* adding t_start: t1=" << t1 << ", t2=" << t2 << ", dt=" << delta_t << std::endl;
        return Eigen::VectorXd::Constant(5, std::numeric_limits<double>::quiet_NaN()); 
    }
    
    Eigen::VectorXd result = Eigen::VectorXd::Constant(5, std::numeric_limits<double>::quiet_NaN());
    result << t1 + t_start, t2 + t_start, (flip_flag ? -a1 : a1), (flip_flag ? -a2 : a2), (flip_flag ? -v_coast : v_coast);
    std::cout << "  DEBUG: Final returned result (global times): t1=" << result[0] << ", t2=" << result[1] << ", a1=" << result[2] << ", a2=" << result[3] << ", vc=" << result[4] << std::endl;

    return result;
}

// Implements Julia's steering_ND
/*
    solves the ND steering function for the two point bouandary value problem
    all inputs are vectors of length D. a contains max acceleration magnitudes per
    dimension. x is position, v is velocity. This returns a vector of times and 
    three matricies, one each for position, velocity, and acceleration at those times
    maticies are (t X D). Note that acceleration(i,:) gives the acceleration from time
    i to time i+1, while velocity and position give the exact velocities and positions
    a each time. This also returns a flag indicating (true) if the solve is successful
*/
ThrusterSteerStateSpace::NDSteeringResult ThrusterSteerStateSpace::steeringND(
    const Eigen::VectorXd& x_start, const Eigen::VectorXd& x_end,
    const Eigen::VectorXd& v_start, const Eigen::VectorXd& v_end,
    double t_start, double t_end, const Eigen::VectorXd& a_max_vec) const {

    // find dimensionality of space
    int D_spatial = x_start.size();
    NDSteeringResult result;
    result.success = false;
    const double EPS = 1e-9;

    // for raw_t note that first and last ime are the same for all D and so left out of here
    Eigen::MatrixXd raw_t(2, D_spatial); // t1, t2 for each dimension (relative to segment start, not global time)
    Eigen::MatrixXd raw_a_vals(3, D_spatial); // a1, v_coast, a2 (Julia's rets1D contains a1, a2, v_coast)

    // first solve the 1D problem in each dimension (a is between t, while v, x are at t), each D gets 4 times including start/goal that things change

    for (int d = 0; d < D_spatial; ++d) {
        Eigen::VectorXd rets1D = steering1D(x_start[d], x_end[d], v_start[d], v_end[d], t_start, t_end, a_max_vec[d]);
        // Note: rets1D = [t_1 t_2 a_1 a_2 v_coast]
        if (rets1D.hasNaN()) {
            std::cout << "1D steering failed for dimension " << d << std::endl;
            return result; // Cannot solve if any 1D fails
        }
        raw_t.col(d) = rets1D.head<2>(); // t1, t2 from steering1D
        raw_a_vals.col(d) << rets1D[2], rets1D[4], rets1D[3]; // a1, v_coast, a2 from steering1D
    }

    // Interleave different times to create a combined trajectory
    // This creates event points when any dimension's acceleration changes.
    std::set<double> unique_times_set;
    unique_times_set.insert(t_start);
    unique_times_set.insert(t_end);
    for (int d = 0; d < D_spatial; ++d) {
        // Only insert if within the overall segment time.
        if (raw_t(0, d) > t_start + EPS && raw_t(0, d) < t_end - EPS) {
            unique_times_set.insert(raw_t(0, d)); // t1
        }
        if (raw_t(1, d) > t_start + EPS && raw_t(1, d) < t_end - EPS) {
            unique_times_set.insert(raw_t(1, d)); // t2
        }
    }

    // Convert set to sorted vector
    std::vector<double> unique_times_vec(unique_times_set.begin(), unique_times_set.end());
    
    // Time points for the interleaved trajectory
    Eigen::VectorXd Time_interleaved = Eigen::VectorXd::Map(unique_times_vec.data(), unique_times_vec.size());
    
    // Matrices to store state and acceleration at interleaved time points
    Eigen::MatrixXd X_interleaved(Time_interleaved.size(), D_spatial);
    Eigen::MatrixXd V_interleaved(Time_interleaved.size(), D_spatial);
    Eigen::MatrixXd A_interleaved(Time_interleaved.size() - 1, D_spatial); // A applies *between* times

    // Calculate X, V, A at interleaved time points for each dimension
    for (int d = 0; d < D_spatial; ++d) {
        double current_x = x_start[d];
        double current_v = v_start[d];
        double current_t_segment_start = t_start;

        double t1_dim = raw_t(0, d); // Global t1 for this dimension
        double t2_dim = raw_t(1, d); // Global t2 for this dimension
        
        double a1_dim = raw_a_vals(0, d);
        double v_coast_dim = raw_a_vals(1, d);
        double a2_dim = raw_a_vals(2, d);

        for (int i = 0; i < Time_interleaved.size(); ++i) {
            double next_t_interleaved = Time_interleaved[i];
            double dt_from_prev_interleaved = next_t_interleaved - current_t_segment_start;

            // Determine current acceleration for this dimension's segment
            double a_current_segment;
            if (current_t_segment_start >= t2_dim - EPS) { // After t2_dim, use a2
                a_current_segment = a2_dim;
            } else if (current_t_segment_start >= t1_dim - EPS) { // After t1_dim, before t2_dim, use coast (0.0 accel)
                a_current_segment = 0.0;
            } else { // Before t1_dim, use a1
                a_current_segment = a1_dim;
            }
            
            // Integrate state (x, v) for this dt_from_prev_interleaved interval
            current_x = current_x + current_v * dt_from_prev_interleaved + 0.5 * a_current_segment * dt_from_prev_interleaved * dt_from_prev_interleaved;
            current_v = current_v + a_current_segment * dt_from_prev_interleaved;
            
            X_interleaved.row(i).col(d) << current_x;
            V_interleaved.row(i).col(d) << current_v;
            
            // Store acceleration for the *interval ending at* Time_interleaved[i] (or starting at previous Time_interleaved[i-1])
            if (i > 0) {
                A_interleaved.row(i-1).col(d) << a_current_segment;
            }
            current_t_segment_start = next_t_interleaved;
        }
    }
    
    result.success = true;
    result.Time = Time_interleaved;
    result.X = X_interleaved;
    result.V = V_interleaved;
    result.A = A_interleaved; 
    return result;
}

// Implements Julia's fineGrain
std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
ThrusterSteerStateSpace::fineGrain(const Eigen::VectorXd& Time_raw, const Eigen::MatrixXd& A_raw,
                                   const Eigen::MatrixXd& V_raw, const Eigen::MatrixXd& X_raw, double dt_res) const {
    
    int D_spatial = X_raw.cols();
    int num_raw_points = Time_raw.size();
    const double EPS = 1e-9;

    // Determine total number of points required after fine-graining
    int total_fine_points = 0;
    for (int i = 0; i < num_raw_points - 1; ++i) {
        double segment_duration = Time_raw[i+1] - Time_raw[i];
        total_fine_points += static_cast<int>(std::ceil(segment_duration / dt_res));
    }
    total_fine_points += 1; // Add for the very last point

    // Use a vector to collect points, then convert to Eigen::VectorXd/MatrixXd
    // This avoids excessive resizing of Eigen matrices
    std::vector<double> fine_Time_vec;
    std::vector<Eigen::VectorXd> fine_X_vec;
    std::vector<Eigen::VectorXd> fine_V_vec;
    std::vector<Eigen::VectorXd> fine_A_vec;

    fine_Time_vec.reserve(total_fine_points);
    fine_X_vec.reserve(total_fine_points);
    fine_V_vec.reserve(total_fine_points);
    fine_A_vec.reserve(total_fine_points);

    for (int i = 0; i < num_raw_points - 1; ++i) {
        double t_segment_start = Time_raw[i];
        double t_segment_end = Time_raw[i+1];
        Eigen::VectorXd x_segment_start = X_raw.row(i).transpose();
        Eigen::VectorXd v_segment_start = V_raw.row(i).transpose();
        Eigen::VectorXd a_segment = A_raw.row(i).transpose(); // Accel applies over this interval

        fine_Time_vec.push_back(t_segment_start);
        fine_X_vec.push_back(x_segment_start);
        fine_V_vec.push_back(v_segment_start);
        fine_A_vec.push_back(a_segment);

        for (double t_interp = t_segment_start + dt_res; t_interp < t_segment_end - EPS; t_interp += dt_res) {
            double dt = t_interp - t_segment_start;
            
            fine_Time_vec.push_back(t_interp);
            // fine_X_vec.push_back(x_segment_start + v_segment_start * dt + 0.5 * a_segment.array() * dt * dt);
            fine_X_vec.push_back(x_segment_start + v_segment_start * dt + (0.5 * a_segment.array() * dt * dt).matrix());

            // fine_V_vec.push_back(v_segment_start + a_segment * dt);
            fine_V_vec.push_back(v_segment_start + (a_segment.array() * dt).matrix()); // Changed a_segment * dt to (a_segment.array() * dt).matrix()

            fine_A_vec.push_back(a_segment);
        }
    }

    // Add the very last raw point
    fine_Time_vec.push_back(Time_raw[num_raw_points - 1]);
    fine_X_vec.push_back(X_raw.row(num_raw_points - 1).transpose());
    fine_V_vec.push_back(V_raw.row(num_raw_points - 1).transpose());
    fine_A_vec.push_back(Eigen::VectorXd::Zero(D_spatial)); // Last accel is not defined by segment, set to zero or appropriate.


    // Convert std::vector to Eigen::VectorXd/MatrixXd
    Eigen::VectorXd Time_fine_out = Eigen::VectorXd::Map(fine_Time_vec.data(), fine_Time_vec.size());
    Eigen::MatrixXd X_fine_out(fine_X_vec.size(), D_spatial);
    Eigen::MatrixXd V_fine_out(fine_V_vec.size(), D_spatial);
    Eigen::MatrixXd A_fine_out(fine_A_vec.size(), D_spatial);

    for (size_t i = 0; i < fine_X_vec.size(); ++i) {
        X_fine_out.row(i) = fine_X_vec[i].transpose();
        V_fine_out.row(i) = fine_V_vec[i].transpose();
        A_fine_out.row(i) = fine_A_vec[i].transpose();
    }

    return std::make_tuple(Time_fine_out, A_fine_out, V_fine_out, X_fine_out);
}

// Main steer function to find optimal trajectory between two full states
// Trajectory ThrusterSteerStateSpace::steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
//     Trajectory traj_out;
//     traj_out.is_valid = false;
//     traj_out.cost = std::numeric_limits<double>::infinity();
//     const double EPS = 1e-9;

//     // from: [x, y, z, vx, vy, vz, t_start]
//     // to:   [x, y, z, vx, vy, vz, t_end]
    
//     // Ensure states have correct dimension
//     if (from.size() != dimension_ || to.size() != dimension_) {
//         std::cerr << "Error: 'from' or 'to' state has incorrect dimension." << std::endl;
//         return traj_out;
//     }

//     int D_spatial = (dimension_ - 1) / 2; // e.g., 3 for 3D position/velocity

//     // Extract spatial positions and velocities
//     Eigen::VectorXd from_pos = from.head(D_spatial);
//     Eigen::VectorXd from_vel = from.segment(D_spatial, D_spatial);
//     double from_time = from[dimension_ - 1];

//     Eigen::VectorXd to_pos = to.head(D_spatial);
//     Eigen::VectorXd to_vel = to.segment(D_spatial, D_spatial);
//     double to_time = to[dimension_ - 1];

//     // IGNORE THE COMMENT BELOW FOR NOW!
//     // Planning happens in "reverse time" for FMTX/RRTX.
//     // The 'from' state has a "later" time (closer to robot) than 'to' (closer to goal).
//     // So, target_delta_t = from_time - to_time.
//     // Julia code for DFMT uses t_start for the earlier time, t_end for later.
//     // Here, to use steeringND, we need t_start < t_end.
//     // So, we effectively "reverse" the problem for steeringND, then reverse solution.
//     double actual_duration = to_time - from_time; // This is the total time cost if path is valid

//     // if (actual_duration < EPS) { // Duration must be positive
//     //     std::cout << "Error: Actual duration is non-positive: " << actual_duration << std::endl;
//     //     return traj_out;
//     // }

//     // Call N-D steering
//     // Max acceleration vector (assuming same for all spatial dimensions)
//     Eigen::VectorXd a_max_vec = Eigen::VectorXd::Constant(D_spatial, max_acceleration_);

//     // The Julia steering_ND takes (x_start, x_end, v_start, v_end, t_start, t_end, a).
//     // It assumes t_start is the earlier time, t_end is the later time.
//     // So for our problem (from_time (later) to to_time (earlier)):
//     // - steeringND_t_start = to_time
//     // - steeringND_t_end = from_time
//     // - The X and V are also reversed for steeringND input.

//     // NDSteeringResult nd_result = steeringND(to_pos, from_pos, to_vel, from_vel, to_time, from_time, a_max_vec);
//     NDSteeringResult nd_result = steeringND(from_pos, to_pos, from_vel, to_vel, from_time, to_time, a_max_vec);
    
//     if (!nd_result.success) {
//         std::cout << "ND steering failed." << std::endl;
//         return traj_out;
//     }

//     // Fine-grain the trajectory for output
//     double discretization_step = 0.5; // meters (or unit of position distance for spatial dimensions)
//                                      // This could be made dynamic or a parameter.

//     // Calculate fine-grained trajectory
//     auto [fine_Time, fine_A, fine_V, fine_X] = 
//         fineGrain(nd_result.Time, nd_result.A, nd_result.V, nd_result.X, discretization_step);

//     // Populate Trajectory object
//     traj_out.cost = actual_duration; // Total time is the cost for time-optimal thruster
//     traj_out.is_valid = true;

//     // --- MODIFIED: Populate execution_data with the forward-time trajectory (from 'to' to 'from') ---
//     traj_out.execution_data.is_valid = true;
//     traj_out.execution_data.total_cost = actual_duration;
//     traj_out.execution_data.Time = fine_Time;
//     traj_out.execution_data.X = fine_X;
//     traj_out.execution_data.V = fine_V;
//     traj_out.execution_data.A = fine_A;



//     traj_out.path_points.reserve(fine_Time.size());
//     for (int i = 0; i < fine_Time.size(); ++i) {
//         Eigen::VectorXd state_at_t(dimension_);
//         state_at_t.head(D_spatial) = fine_X.row(i).transpose();
//         state_at_t.segment(D_spatial, D_spatial) = fine_V.row(i).transpose();
//         state_at_t[dimension_ - 1] = fine_Time[i];
//         traj_out.path_points.push_back(state_at_t);
//     }
    
//     // Reverse the path_points since steer is from `from` to `to`, but solution was computed `to` to `from`.
//     std::reverse(traj_out.path_points.begin(), traj_out.path_points.end());

//     return traj_out;
// }

Trajectory ThrusterSteerStateSpace::steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
    Trajectory traj_out;
    traj_out.is_valid = false;
    traj_out.cost = std::numeric_limits<double>::infinity();
    const double EPS = 1e-9;

    if (from.size() != dimension_ || to.size() != dimension_) {
        std::cerr << "Error: 'from' or 'to' state has incorrect dimension." << std::endl;
        return traj_out;
    }

    int D_spatial = (dimension_ - 1) / 2;

    Eigen::VectorXd from_pos = getSpatialPosition(from);
    Eigen::VectorXd from_vel = getSpatialVelocity(from);
    double from_time = from[dimension_ - 1];

    Eigen::VectorXd to_pos = getSpatialPosition(to);
    Eigen::VectorXd to_vel = getSpatialVelocity(to);
    double to_time = to[dimension_ - 1];

    double duration = from_time - to_time;

    if (duration < EPS) {
        return traj_out;
    }

    // Plan in forward time (from earlier 'to' state to later 'from' state)
    Eigen::VectorXd a_max_vec = Eigen::VectorXd::Constant(D_spatial, max_acceleration_);
    NDSteeringResult nd_result = steeringND(to_pos, from_pos, to_vel, from_vel, to_time, from_time, a_max_vec);
    
    if (!nd_result.success) {
        return traj_out;
    }

    // Discretize the trajectory
    double discretization_step = 0.5; // You can adjust this resolution
    auto [fine_Time, fine_A, fine_V, fine_X] = 
        fineGrain(nd_result.Time, nd_result.A, nd_result.V, nd_result.X, discretization_step);

    // ✅ ROBUST MANUAL REVERSAL
    // Manually reverse the trajectory to create the correct countdown path.
    long num_points = fine_Time.size();
    if (num_points == 0) {
        return traj_out; // Should not happen if fineGrain is successful
    }

    traj_out.is_valid = true;
    traj_out.cost = duration;
    traj_out.execution_data.is_valid = true;
    traj_out.execution_data.total_cost = duration;

    // Resize the execution data matrices
    traj_out.execution_data.Time.resize(num_points);
    traj_out.execution_data.X.resize(num_points, D_spatial);
    traj_out.execution_data.V.resize(num_points, D_spatial);
    // Note: The fine-grained acceleration matrix has one less row than the number of points
    traj_out.execution_data.A.resize(num_points > 1 ? num_points - 1 : 0, D_spatial);

    // Populate the matrices by looping through the fine-grained data in reverse
    for (long i = 0; i < num_points; ++i) {
        long reverse_i = num_points - 1 - i;
        traj_out.execution_data.Time(i) = fine_Time(reverse_i);
        traj_out.execution_data.X.row(i) = fine_X.row(reverse_i);
        traj_out.execution_data.V.row(i) = fine_V.row(reverse_i);
    }
    
    // Reverse the acceleration data, which applies to intervals
    for (long i = 0; i < traj_out.execution_data.A.rows(); ++i) {
        // A[i] corresponds to the interval from T[i] to T[i+1]
        // In reverse, this is the interval from fine_Time[N-1-i] to fine_Time[N-2-i]
        // The acceleration for that interval is fine_A[N-2-i]
        long reverse_a_idx = fine_A.rows() - 1 - i;
        traj_out.execution_data.A.row(i) = fine_A.row(reverse_a_idx);
    }

    // Populate the coarse path_points vector for other uses
    traj_out.path_points.reserve(num_points);
    for (long i = 0; i < num_points; ++i) {
        Eigen::VectorXd state_at_t(dimension_);
        state_at_t.head(D_spatial) = traj_out.execution_data.X.row(i).transpose();
        state_at_t.segment(D_spatial, D_spatial) = traj_out.execution_data.V.row(i).transpose();
        state_at_t[dimension_ - 1] = traj_out.execution_data.Time(i);
        traj_out.path_points.push_back(state_at_t);
    }
    
    return traj_out;
}