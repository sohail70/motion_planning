// Copyright 2025 Soheil E.nia
#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/rrtx_node.hpp"
#include "motion_planning/utils/visualization.hpp"
#include "motion_planning/ds/priority_queue.hpp"

#define RRTX_INFO(msg)  std::cout << "\033[0;32m[INFO] " << msg << "\033[0m\n"
#define RRTX_WARN(msg)  std::cout << "\033[1;33m[WARN] " << msg << "\033[0m\n"
#define RRTX_ERROR(msg) std::cerr << "\033[1;31m[ERROR] " << msg << "\033[0m\n"
#define RRTX_FATAL(msg) std::cerr << "\033[1;37;41m[FATAL] " << msg << "\033[0m\n"

class KinodynamicRRTX : public Planner {
 public:
    KinodynamicRRTX(std::shared_ptr<StateSpace> statespace, 
        std::shared_ptr<ProblemDefinition> problem_def,
        std::shared_ptr<ObstacleChecker> obs_checker);
    
    void setup(const Params& params, std::shared_ptr<Visualization> visualization) override;
    void plan() override;
    std::vector<Eigen::VectorXd> getPathPositions() const;

    void setStart(const Eigen::VectorXd& start) override;
    void setGoal(const Eigen::VectorXd& goal) override;
    void clearPlannerState() ;

    void updateObstacleSamples(const ObstacleVector& obstacles);

    void visualizeTree();

    void visualizePath(const std::vector<Eigen::VectorXd>& path_waypoints);

    void setRobotState(const Eigen::VectorXd& robot_state);

    void dumpTreeToCSV(const std::string& filename) const;

    void visualizeTreeReal();


    bool hasCycleInParentGraph(std::string& report);
    double getAvgOutDegree() const {
        if (tree_.empty()) return 0.0;
        long long total_out = 0;
        for (const auto& node_ptr : tree_) {
            total_out += node_ptr->outgoingEdges().size(); 
        }
        return static_cast<double>(total_out) / tree_.size();
    }

    double getAvgInDegree() const {
        if (tree_.empty()) return 0.0;
        long long total_in = 0;
        for (const auto& node_ptr : tree_) {
            total_in += node_ptr->incomingEdges().size(); 
        }
        return static_cast<double>(total_in) / tree_.size();
    }



    double getNeighborhoodRadius(){return neighborhood_radius_;}

    const ReplanMetrics& getLastReplanMetrics() const { return last_replan_metrics_; }
    void resetMetrics() { last_replan_metrics_ = ReplanMetrics(); }
    double getRobotTimeToGo() const { return robot_current_time_to_goal_; }

    bool isRobotSafe();
    int getTreeSize() { return tree_.size();}
    bool runForensics();

    // struct EdgeEval {
    //     RRTxNode* neighbor;
    //     bool fwd_exists = false; Trajectory fwd_traj; bool fwd_safe = false; 
    //     std::vector<const Obstacle*> fwd_blockers; 
    //     bool rev_exists = false; Trajectory rev_traj; bool rev_safe = false; 
    //     std::vector<const Obstacle*> rev_blockers; 
    // };

    struct EdgeEval {
        RRTxNode* neighbor;
        bool fwd_exists = false; 
        std::shared_ptr<Trajectory> fwd_traj; // Changed to shared_ptr
        bool fwd_safe = false; 
        std::vector<const Obstacle*> fwd_blockers; 
        
        bool rev_exists = false; 
        std::shared_ptr<Trajectory> rev_traj; // Changed to shared_ptr
        bool rev_safe = false; 
        std::vector<const Obstacle*> rev_blockers; 
    };

    std::vector<EdgeEval> evaluated_edges;
 private:
    std::vector<std::shared_ptr<RRTxNode>> tree_;
    std::shared_ptr<KDTree> kdtree_;
    PriorityQueue<RRTxNode, RRTxComparator> inconsistency_queue_;
    std::shared_ptr<StateSpace> statespace_;
    std::shared_ptr<ProblemDefinition> problem_;
    std::shared_ptr<ObstacleChecker> obs_checker_;
    std::shared_ptr<Visualization> visualization_;
    RRTxNode*  vbot_node_;
    std::unordered_set<int> Vc_T_;
    double neighborhood_radius_;
    double epsilon_ = 1e-6;
    double gamma_;
    double delta = 20.0; 
    double factor;
    int num_of_samples_;
    int dimension_;
    size_t sample_counter = 0;
    bool partial_update;
    Eigen::VectorXd lower_bounds_;
    Eigen::VectorXd upper_bounds_;
    bool use_kdtree;
    int kd_dim ; 
    Eigen::VectorXd robot_continuous_state_;
    double robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
    bool extend(Eigen::VectorXd v);
    void rewireNeighbors(RRTxNode* v);
    void reduceInconsistency();
    void cullNeighbors(RRTxNode* v);
    void updateLMC(RRTxNode* v);
    void verifyQueue(RRTxNode* node);  // Fixed signature
    void propagateDescendants();
    void verifyOrphan(RRTxNode* node);
    double shrinkingBallRadius() const;
    void addNewObstacle(const Obstacle& ob);
    void removeObstacle(const Obstacle& ob);
    Eigen::VectorXd saturate(const Eigen::VectorXd& newPoint, const Eigen::VectorXd& closestPoint, double delta);
    ReplanMetrics last_replan_metrics_; 
    std::unordered_map<std::string, Obstacle> previous_obstacles_;
    double bridge_cost_;
    bool is_geometric_mode_;
};