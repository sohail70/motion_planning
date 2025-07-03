#include "motion_planning/state_space/rdt_statespace.hpp"
#include <stdexcept>
#include <random>

RDTStateSpace::RDTStateSpace(int euclidean_dimension, double min_velocity, double max_velocity, double robot_velocity, int initial_capacity)
    // The total dimension is the number of spatial dimensions plus one for time.
    : StateSpace(euclidean_dimension + 1, initial_capacity),
      euclidean_dim_(euclidean_dimension),
      min_velocity_(min_velocity),
      max_velocity_(max_velocity),
      robot_velocity_(robot_velocity)
{
    
    std::srand(42); // TODO: For sampling the same batch every time just for debug and test. --> remove it later.

    // These weights are used in the distance() function for the KD-tree.
    // This helps balance the contribution of spatial distance vs. time difference.
    // You can tune the time_weight (last element). A smaller value makes time less
    // important when finding "near" neighbors.
    distance_weights_.resize(dimension_);
    distance_weights_.head(euclidean_dim_).setOnes(); // Weight of 1.0 for all spatial dimensions
    distance_weights_(dimension_ - 1) = 1.0;          // Weight for the time dimension
}

// // The 'steer' function is the most important part. It defines the valid connections.
// // Override the steer function to add time and velocity checks
// Trajectory RDTStateSpace::steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
//     Trajectory traj;
//     traj.is_valid = false;

//     // Extract spatial and time components from the state vectors
//     const Eigen::VectorXd& from_spatial = from.head(euclidean_dim_);
//     const Eigen::VectorXd& to_spatial = to.head(euclidean_dim_);
//     double from_time = from(dimension_ - 1);
//     double to_time = to(dimension_ - 1);

//     // *** THE CORRECTION IS HERE ***
//     // In a backward search, the new node 'from' must have a later time than the existing node 'to'.
//     double time_duration = from_time - to_time;

//     // If time flows backward or stands still, the trajectory is invalid.
//     if (time_duration <= 1e-6) { // Use a small epsilon for floating-point safety
//         return traj;
//     }

//     double spatial_distance = (to_spatial - from_spatial).norm();

//     // Check if the required speed to cover the spatial distance in the given time
//     // is within the robot's physical limits.
//     double required_speed = spatial_distance / time_duration;
//     if (required_speed > max_velocity_ || required_speed < min_velocity_) {
//         return traj;
//     }

//     // --- If valid, populate the trajectory ---
//     traj.is_valid = true;
//     traj.time_duration = time_duration;
//     // For a time-optimal planner, the cost IS the time elapsed.
//     // traj.cost = time_duration;
//     traj.cost = std::sqrt(std::pow(time_duration,2) + std::pow (spatial_distance,2));

//     // --- CRITICAL ADDITION ---
//     // You must add at least the start and end points for the collision checker to use.
//     traj.path_points.push_back(from);
//     traj.path_points.push_back(to);

//     // // Generate intermediate points for collision checking. --> NO NEED, I ASSUME ONE SEGMENT FOR A LINE AND IN OBSTALCE CHECKER I OBSTALCE CHECK AN EDGE!
//     // int num_steps = 10;
//     // traj.path_points.reserve(num_steps + 1);
//     // for (int i = 0; i <= num_steps; ++i) {
//     //     double t_local = static_cast<double>(i) / num_steps;
//     //     Eigen::VectorXd point(dimension_);
//     //     // Interpolate from 'from' (t_local=0) to 'to' (t_local=1)
//     //     point.head(euclidean_dim_) = from_spatial + t_local * (to_spatial - from_spatial);
//     //     point(dimension_ - 1) = from_time + t_local * (to_time - from_time);
//     //     traj.path_points.push_back(point);
//     // }

//     return traj;
// }


// // Lie --> wiht different speed --> messes up time --> but i think the important thing is to have const vel in each seg right?
// /*
//     Maximum Flexibility: This is the "hit the gas" model. It uses the full dynamic capabilities of the robot. 
//     If a path requires a short burst of speed at max_velocity_, this function can create that path.
// */
// // Trajectory RDTStateSpace::steer(const Eigen::VectorXd& from,
// //                                 const Eigen::VectorXd& to) const
// // {
// //     Trajectory traj;
// //     traj.is_valid = false;

// //     // Extract spatial and time components
// //     const Eigen::VectorXd& from_spatial = from.head(euclidean_dim_);
// //     const Eigen::VectorXd& to_spatial   = to.head(euclidean_dim_);
// //     double from_time = from(dimension_ - 1);
// //     double to_time   = to(dimension_ - 1);

// //     // In backward search: 'from' must lie in time AFTER 'to'
// //     double time_duration = from_time - to_time;
// //     if (time_duration <= 1e-6) {  // non-positive duration is invalid
// //         return traj;
// //     }

// //     // Compute straight-line distance
// //     double spatial_distance = (to_spatial - from_spatial).norm();

// //     // Minimum speed needed to cover the distance in the sampled time
// //     double req_speed = spatial_distance / time_duration;

// //     // Choose an actual speed:
// //     //  • Prefer robot_velocity_ if it meets or exceeds req_speed
// //     //  • Otherwise fall back to req_speed
// //     double actual_speed = std::max(req_speed, robot_velocity_);

// //     // Clamp within the robot's hard limits
// //     actual_speed = std::min(actual_speed, max_velocity_);
// //     actual_speed = std::max(actual_speed, min_velocity_);

// //     // If after clamping we still can't meet the required speed, this connection fails
// //     if (actual_speed < req_speed - 1e-6) {
// //         return traj;
// //     }

// //     // Recompute the time-of-flight using the chosen speed
// //     time_duration = spatial_distance / actual_speed;

// //     // --- Build a valid trajectory ---
// //     traj.is_valid      = true;
// //     traj.time_duration = time_duration;
// //     // Cost can be time-optimal or a combination of time+distance
// //     traj.cost = std::sqrt(time_duration * time_duration +
// //                           spatial_distance * spatial_distance);

// //     // Provide the start and end states for collision checking
// //     traj.path_points.push_back(from);
// //     traj.path_points.push_back(to);

// //     return traj;
// // }



// // constant speed assumption (TRUTH) --> speed is constant on each segment! 
// // This is the correct steer function for rdt_statespace.cpp
// // It enforces a constant velocity, which is required for the rest of
// // the system's time calculations to be accurate.

// Trajectory RDTStateSpace::steer(const Eigen::VectorXd& from,
//                                 const Eigen::VectorXd& to) const
// {
//     Trajectory traj;
//     traj.is_valid = false;

//     // Extract spatial and time components
//     const Eigen::VectorXd& from_spatial = from.head(euclidean_dim_);
//     const Eigen::VectorXd& to_spatial   = to.head(euclidean_dim_);
//     double from_time = from(dimension_ - 1); // time-to-go for the new node
//     double to_time   = to(dimension_ - 1);   // time-to-go for the existing node

//     // This is the time difference between the two pre-sampled states.
//     double time_diff_samples = from_time - to_time;

//     // A connection is only valid if the new node has a greater time-to-go.
//     if (time_diff_samples <= 1e-6) {
//         return traj;
//     }

//     // Calculate the straight-line distance in space.
//     double spatial_distance = (to_spatial - from_spatial).norm();

//     // ====================================================================================
//     // --- The Constant Velocity Model Logic ---
//     // ====================================================================================

//     // 1. Calculate the time it would *actually* take our robot to travel this distance
//     //    at its defined, constant velocity.
//     double time_duration_at_const_vel = (robot_velocity_ > 1e-6)
//                                       ? (spatial_distance / robot_velocity_)
//                                       : std::numeric_limits<double>::infinity();

//     // 2. The connection is kinodynamically valid ONLY IF the robot can make it in time.
//     //    The time required at its constant speed must be less than or equal to the
//     //    time allotted by the difference between the sampled nodes. This allows the robot
//     //    to "wait" at a node if it arrives early, but it cannot go faster than its defined speed.
//     if (time_duration_at_const_vel > time_diff_samples + 1e-6) { // Add tolerance
//         // The robot is not fast enough to connect these two samples in time.
//         // This edge is invalid.
//         return traj;
//     }

//     // --- If we reach here, the connection is valid. ---
//     // Build the trajectory using the time based on our constant velocity model.
//     traj.is_valid      = true;
//     traj.time_duration = time_duration_at_const_vel;

//     // The cost should reflect this model. Here we use a combination of the
//     // true time duration and the spatial distance.
//     traj.cost = std::sqrt(std::pow(traj.time_duration, 2) +
//                           std::pow(spatial_distance, 2));

//     // Provide the start and end states for the collision checker.
//     traj.path_points.push_back(from);
//     traj.path_points.push_back(to);

//     return traj;
// }


// MODEL 2: VARIABLE VELOCITY STEER FUNCTION
// This steer function allows the robot to use any speed within its physical limits.
// This is more flexible and corresponds to your suggestion.

Trajectory RDTStateSpace::steer(const Eigen::VectorXd& from,
                                const Eigen::VectorXd& to) const
{
    Trajectory traj;
    traj.is_valid = false;

    // Extract spatial and time components
    const Eigen::VectorXd& from_spatial = from.head(euclidean_dim_);
    const Eigen::VectorXd& to_spatial   = to.head(euclidean_dim_);
    double from_time = from(dimension_ - 1); // time-to-go for the new node
    double to_time   = to(dimension_ - 1);   // time-to-go for the existing node

    // The time allotted by the difference between the sampled nodes' time-to-go.
    double time_duration = from_time - to_time;

    // A connection is only valid if time flows forward (time-to-go decreases).
    if (time_duration <= 1e-6) {
        return traj;
    }

    // Calculate the straight-line distance in space.
    double spatial_distance = (to_spatial - from_spatial).norm();

    // =======================================================================
    // --- The Variable Velocity Model Logic ---
    // =======================================================================
    // 1. Calculate the speed *required* to connect the two states in the allotted time.
    double required_speed = spatial_distance / time_duration;

    // 2. The connection is valid ONLY IF this required speed is within the robot's
    //    physical capabilities.
    if (required_speed > max_velocity_ + 1e-6 || required_speed < min_velocity_ - 1e-6) {
        // The robot is either too slow or would have to move too fast.
        return traj;
    }

    // --- If we reach here, the connection is valid. ---
    traj.is_valid      = true;
    traj.time_duration = time_duration; // The time taken IS the allotted time.

    // CRITICAL: The cost function must be chosen carefully.
    // Using just `time_duration` as the cost would make the planner prefer
    // slow-moving paths. A combined cost is better.
    traj.cost = std::sqrt(std::pow(traj.time_duration, 2) +
                          std::pow(spatial_distance, 2));

    // Provide the start and end states for the collision checker.
    traj.path_points.push_back(from);
    traj.path_points.push_back(to);

    // --- NEW: Add the single analytical segment for the collision checker ---
    traj.analytical_segments.clear();
    traj.analytical_segments.push_back({
        .type         = SegmentType::LINE,
        .duration     = time_duration,
        .start_point  = from_spatial,
        .end_point    = to_spatial
    });
    
    return traj;
}


// For state types, we can just reuse EuclideanState.
std::shared_ptr<State> RDTStateSpace::addState(const Eigen::VectorXd& value) {
    return StateSpace::addState(std::make_shared<EuclideanState>(value));
}

// The distance function is used by the KD-tree to find neighbors.
// We use a weighted Euclidean distance to balance space and time.
double RDTStateSpace::distance(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2) const {
    Eigen::ArrayXd diff = state1->getValue().array() - state2->getValue().array();
    return (diff * distance_weights_.array()).matrix().norm();
}

// Uniform sampling works the same way as in the Euclidean space.
std::shared_ptr<State> RDTStateSpace::sampleUniform(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds) {
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

bool RDTStateSpace::isValid(const std::shared_ptr<State>& state) const {
    return true; // Can add bounds checking here if needed
}

// --- Provide simple implementations for unused pure virtual methods ---
std::shared_ptr<State> RDTStateSpace::sampleUniform(double min, double max) {
    throw std::runtime_error("This sampleUniform overload is not supported in RDTStateSpace. Use bounds vector.");
}

void RDTStateSpace::sampleUniform(double min, double max, int k) {
    throw std::runtime_error("This sampleUniform overload is not supported in RDTStateSpace. Use bounds vector.");
}

std::shared_ptr<State> RDTStateSpace::interpolate(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2, double t) const {
     Eigen::VectorXd interpolated_val = state1->getValue() + t * (state2->getValue() - state1->getValue());
     return std::make_shared<EuclideanState>(interpolated_val);
}