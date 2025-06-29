#include "motion_planning/state_space/dubins_time_statespace.hpp"
#include <stdexcept>

// Helper function from the base class file
double normalizeAngle(double angle);

DubinsTimeStateSpace::DubinsTimeStateSpace(double min_turning_radius, double min_velocity, double max_velocity)
    : DubinsStateSpace(min_turning_radius, 4), // Call the base class constructor
      min_velocity_(min_velocity),
      max_velocity_(max_velocity) {
    
    std::srand(42); // TODO: For sampling the same batch every time just for debug and test. --> remove it later.

    // Set the dimension to 4 for this state space
    dimension_ = 4;
    
    // Resize weights for 4D: x, y, theta, time
    weights_.resize(4);
    // These weights are for the KD-Tree's approximate distance function.
    // Time is given a moderate weight to influence neighbor selection.
    weights_ << 1.0, 1.0, 0.4, 0.8; 
}

// I ALREADY DID THE SCALING IN KD TREE!!!
// Override the distance function for a 4D state
double DubinsTimeStateSpace::distance(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2) const {
    Eigen::VectorXd diff = state1->getValue() - state2->getValue();
    if (diff.size() != 4) return std::numeric_limits<double>::infinity();
    
    diff(2) = normalizeAngle(diff(2)); // Normalize theta
    return diff.cwiseProduct(weights_).norm();
}

// Override the steer function to add time and velocity checks
Trajectory DubinsTimeStateSpace::steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
    // static int count = 0 ;
    // count++;
    // auto start = std::chrono::steady_clock::now();
    // --- Step 1: Get the geometric path from the base class ---
    // This reuses the complex Dubins calculation for the (x, y, theta) subspace.
    Trajectory traj_geom = DubinsStateSpace::steer(from, to);

    if (!traj_geom.is_valid) {
        return traj_geom; // If no geometric path exists, return immediately.
    }

    // --- Step 2: Add Time-Based Checks ---
    // This logic is derived from the Julia code's `validMove` and time parameterization.
    
    // Planners like RRTX/FMTX often plan backward from the goal, so 'from' has a later time than 'to'.
    double time_elapsed = from[3] - to[3];

    // Check 1: Time must flow in the correct direction.
    if (time_elapsed <= 1e-6) { // Use a small epsilon for floating point safety
        traj_geom.is_valid = false;
        return traj_geom;
    }

    // Check 2: The required velocity must be within the vehicle's physical limits.
    double workspace_distance = traj_geom.cost; // The geometric path length
    traj_geom.geometric_distance = workspace_distance;
    double required_velocity = workspace_distance / time_elapsed;
    // std::cout<<"Req Velocity: "<<required_velocity<<"\n";
    if (required_velocity < min_velocity_ || required_velocity > max_velocity_) {
        traj_geom.is_valid = false;
        return traj_geom;
    }

    // --- Step 3: Parameterize the Path with Time ---
    // The waypoints from the base class are 3D. We upgrade them to 4D.
    std::vector<Eigen::VectorXd> path_points_4d;
    path_points_4d.push_back(from); // The first point is the full 'from' state

    double cumulative_dist = 0.0;
    for (size_t i = 1; i < traj_geom.path_points.size(); ++i) {
        // Calculate spatial distance between waypoints
        cumulative_dist += (traj_geom.path_points[i].head<2>() - traj_geom.path_points[i-1].head<2>()).norm();
        
        // Calculate the interpolated time at this waypoint
        double t = from[3] - (cumulative_dist / required_velocity);
        
        Eigen::VectorXd new_point(4);
        new_point << traj_geom.path_points[i].head<3>(), t; // (x,y,theta) from geom, plus new time
        path_points_4d.push_back(new_point);
    }
    
    // Replace the 3D path with the new 4D path
    traj_geom.path_points = path_points_4d;

    // The true cost of this trajectory is the time it takes to execute --> NO I THINK THE SPATIAL DATA NEEDS TO BE INCLUDED ALSO!
    traj_geom.cost = std::sqrt(std::pow(time_elapsed,2) + std::pow (workspace_distance,2));
    traj_geom.time_duration = time_elapsed;
    // auto end = std::chrono::steady_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // if (duration.count() > 0) 
    //     std::cout << "time taken for the steer : " << duration.count() << " microseconds\n";
    // std::cout<<count<<"\n";

    return traj_geom;
}

// // Override sampling for 4D
// std::shared_ptr<State> DubinsTimeStateSpace::sampleUniform(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds) {
//     if (min_bounds.size() != 4 || max_bounds.size() != 4) {
//         throw std::invalid_argument("Bounds must be 4-dimensional for DubinsTimeStateSpace.");
//     }

//     Eigen::VectorXd values(4);

//     // Sample x and y
//     values[0] = min_bounds[0] + (max_bounds[0] - min_bounds[0]) * (static_cast<double>(rand()) / RAND_MAX);
//     values[1] = min_bounds[1] + (max_bounds[1] - min_bounds[1]) * (static_cast<double>(rand()) / RAND_MAX);
    
//     // Sample theta
//     values[2] = min_bounds[2] + (max_bounds[2] - min_bounds[2]) * (static_cast<double>(rand()) / RAND_MAX);
    
//     // Sample time
//     values[3] = min_bounds[3] + (max_bounds[3] - min_bounds[3]) * (static_cast<double>(rand()) / RAND_MAX);

//     return this->addState(values);
// }


std::shared_ptr<State> DubinsTimeStateSpace::sampleUniform(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds)  {
    if (min_bounds.size() != 4 || max_bounds.size() != 4) {
        throw std::invalid_argument("Bounds must be 4-dimensional for DubinsTimeStateSpace.");
    }
    Eigen::VectorXd values(4);
    for (int i = 0; i < 4; ++i) {
        double random_coeff = static_cast<double>(rand()) / RAND_MAX;
        values[i] = min_bounds[i] + (max_bounds[i] - min_bounds[i]) * random_coeff;
    }
    return this->addState(values);
}