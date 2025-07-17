// Copyright Soheil E.nia 2025

#pragma once

// NEW: Struct to hold the full, fine-grained trajectory for execution
// This will contain time, position, velocity, and acceleration profiles.
struct ExecutionTrajectory {
    bool is_valid = false;
    double total_cost = 0.0; // Total duration of the trajectory
    Eigen::VectorXd Time;    // Nx1 vector of time points
    Eigen::MatrixXd X;       // NxD_spatial matrix of positions (x,y,z)
    Eigen::MatrixXd V;       // NxD_spatial matrix of velocities (vx,vy,vz)
    Eigen::MatrixXd A;       // (N-1)xD_spatial matrix of accelerations for intervals
};

enum class SegmentType { ARC, LINE };

// Holds the complete analytical definition of a single path segment
struct AnalyticalSegment {
    SegmentType type;
    double duration = 0.0; // Initially zero, calculated in DubinsTimeStateSpace

    // Geometric properties
    Eigen::Vector2d start_point;
    Eigen::Vector2d end_point;

    // Arc-specific data
    Eigen::Vector2d center = {0, 0};
    double radius = 0.0;
    bool is_clockwise = false;
};


// This struct holds the result of the steering function.
// It's the primary way the StateSpace communicates path details to the Planner.
struct Trajectory {
    bool is_valid = false;
    double cost = std::numeric_limits<double>::infinity(); // Considering time (ALSO! on top of geometric distance) if Time is also in the state space!
    double geometric_distance = std::numeric_limits<double>::infinity(); // Geometric distance is the pure path length, crucial for the Ball-Box theorem.
    double time_duration = std::numeric_limits<double>::infinity(); // For getCollidingObstalces function's time input
    std::vector<Eigen::VectorXd> path_points;
    ExecutionTrajectory execution_data; // Detailed profile for simulation and execution

    // --- Data for Analytical Collision Checking ---
    // std::vector<AnalyticalSegment> analytical_segments;
    
    // Helper fields to pass info from base to derived steer function
    std::string maneuver_type;
    std::vector<Eigen::Vector2d> maneuver_pts;
    std::vector<Eigen::Vector2d> maneuver_centers;

};


struct EdgeInfo {
    double distance;
    double distance_original; // I use it in removeObstalce When i want to reset the distance


    /*
        This is specifically for rrtx
        Initial = original (N0)   --> at the moment of adding that sample which nodes are around that sample? these are original neighbor
        Temporaty = Running  (Nr) --> after some time the neighbors to an "old" sample grows but those are temporary neighbors that will be culled in cullNeighbor
    */
    bool is_initial;  // True = persistent (N0), False = temporary (Nr)  --> not used in Fmtx


    ///////////////////////////////////
    // The simple Euclidean distance from near() can still be useful for heuristics
    // double approximate_distance; 

    // --- Caching for Kinodynamics ---
    Trajectory cached_trajectory;
    bool       is_trajectory_computed = false;



    /**
     * @brief [THE CRITICAL CHANGE] A set that stores the unique names or IDs of
     * every obstacle currently predicted to collide with this edge's trajectory.
     *
     * - If this set is EMPTY, the edge is considered VALID.
     * - If this set is NOT EMPTY, the edge is INVALID and must be ignored by the planner.
     */
    std::unordered_set<std::string> invalidating_obstacles;


};
