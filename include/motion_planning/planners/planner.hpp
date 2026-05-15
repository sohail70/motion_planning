// Copyright 2025 SOheil E.nia

#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/state_space/state.hpp"
#include "motion_planning/state_space/statespace.hpp"
#include "motion_planning/utils/problem_definition.hpp"
#include "motion_planning/utils/params.hpp"

#include "motion_planning/utils/nano_flann.hpp"
#include "motion_planning/utils/weighted_nano_flann.hpp"
#include "motion_planning/utils/dynamic_weighted_nano_flann.hpp"
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
    KinodynamicANYFMTX,
    KinodynamicANYRRTX,
    KinodynamicPRMStarDStarLite,
    
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


struct ReplanMetrics {
    int       obstacle_checks     = 0;   // collision queries — dominant cost
    double    path_cost           = 0.0; // final solution quality after repair
    int revisits = 0; // Specific to FMTX
};

class Planner {
    public:
        Planner() = default;
        virtual ~Planner() = default;
        virtual void setStart(const Eigen::VectorXd& start) = 0;
        virtual void setGoal(const Eigen::VectorXd& goal) = 0;
        virtual void setup(const Params& params , std::shared_ptr<Visualization> visualization) = 0;
        virtual void plan() = 0;
        virtual void setClock(std::shared_ptr<rclcpp::Clock> clock) { }
        virtual void setRobotState(const Eigen::VectorXd& state) { }
        virtual std::vector<Eigen::VectorXd> getPathPositions() const { return {}; } 
        virtual void updateObstacles(const ObstacleVector& obstacles) { }
        virtual void visualizeTree() {};
        virtual void visualizeTreeGradient() {};
        virtual void visualizeTreeReal() {};
        virtual void visualizePath(const std::vector<Eigen::VectorXd>& path) {}
        virtual void visualizePathGradient(const std::vector<Eigen::VectorXd>& path) {}
        virtual void visualizeSearchArea() {};
        virtual const ReplanMetrics& getLastReplanMetrics() const { 
            static ReplanMetrics empty_metrics; 
            return empty_metrics; 
        }
        virtual double getNeighborhoodRadius() {return 0;};
        virtual int getTreeSize() = 0;
        virtual void logGraphState(std::ofstream& out_file, int cycle_number) const {};
        virtual double getAvgOutDegree() const { return 0.0; }
        virtual double getAvgInDegree() const { return 0.0; }
        virtual int getIsolatedNodeCount() const { return 0; }
        virtual void resetMetrics() {}

    protected:


};
