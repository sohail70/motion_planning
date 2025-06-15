// Copyright Soheil E.nia 2025

#pragma once
// This struct holds the result of the steering function.
// It's the primary way the StateSpace communicates path details to the Planner.
struct Trajectory {
    bool is_valid = false;
    double cost = std::numeric_limits<double>::infinity();
    std::vector<Eigen::VectorXd> path_points;
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
};
