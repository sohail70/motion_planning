// Copyright 2025 SOheil E.nia

#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/state_space/state.hpp"
#include "motion_planning/state_space/statespace.hpp"
#include "motion_planning/utils/problem_definition.hpp"
#include "motion_planning/utils/params.hpp"

#include "motion_planning/utils/nano_flann.hpp"
#include "motion_planning/utils/weighted_nano_flann.hpp"
#include "motion_planning/utils/lie_kd_tree.hpp"
#include "motion_planning/utils/visualization.hpp"

#include "motion_planning/utils/obstacle_checker.hpp"

enum class PlannerType{
    FMT,
    ANYFMT,
    InformedANYFMT,
    InformedANYFMTA,
    FMTA,
    FMTX,
    RRTX,
    RRTStar,
    RRT,
    BITStar,
    KinodynamicFMTX,
    KinodynamicRRTX,
};







// Define the hash function for std::pair<int, int>
struct pair_hash {
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2>& p) const {
        auto hash1 = std::hash<T1>{}(p.first);
        auto hash2 = std::hash<T2>{}(p.second);
        return hash1 ^ (hash2 << 1); // Combine the two hash values
    }
};

struct TupleHasher {
    size_t operator()(const std::tuple<int, int, int>& k) const {
        // A simple way to combine hashes
        return std::hash<int>()(std::get<0>(k)) ^
               std::hash<int>()(std::get<1>(k)) ^
               std::hash<int>()(std::get<2>(k));
    }
};

struct TupleComparator {
    bool operator()(const std::tuple<int, int, int>& a, const std::tuple<int, int, int>& b) const {
        return a < b;  // relies on std::tuple's built-in lexicographic comparison
    }
};



class Planner {
 public:
    Planner() = default;
    virtual ~Planner() = default;

    virtual void setStart(const Eigen::VectorXd& start) = 0;
    virtual void setGoal(const Eigen::VectorXd& goal) = 0;
    
    virtual void setup(const Params& params , std::shared_ptr<Visualization> visualization) = 0;
    virtual void plan() = 0;
    // virtual std::vector<int> getPathIndex() const = 0;

 protected:


};


