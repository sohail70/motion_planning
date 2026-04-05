// Copyright 2025 Soheil E.Nia
#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/fmt_node.hpp"
#include "motion_planning/utils/visualization.hpp"
#include "motion_planning/ds/priority_queue.hpp"
#include "motion_planning/state_space/min_snap_statespace.hpp" // TODO: put the steer with initial in state space virtual method!


// --- Custom Colored Logging Macros ---
#define FMTX_INFO(msg)  std::cout << "\033[0;32m[INFO] " << msg << "\033[0m\n"       // Green
#define FMTX_WARN(msg)  std::cout << "\033[1;33m[WARN] " << msg << "\033[0m\n"       // Yellow
#define FMTX_ERROR(msg) std::cerr << "\033[1;31m[ERROR] " << msg << "\033[0m\n"      // Bold Red
#define FMTX_FATAL(msg) std::cerr << "\033[1;37;41m[FATAL] " << msg << "\033[0m\n"   // White text on Red background


class KinodynamicANYFMTX : public Planner {
    public:
        KinodynamicANYFMTX(std::shared_ptr<StateSpace> statespace , std::shared_ptr<ProblemDefinition> problem_def , std::shared_ptr<ObstacleChecker> obs_checker);
        void setup(const Params& params, std::shared_ptr<Visualization> visualization) override;
        void plan() override;
        std::vector<Eigen::VectorXd> getPathPositions() const;
        void setStart(const Eigen::VectorXd& start) override;
        void setGoal(const Eigen::VectorXd& goal) override;

        FMTNode* getRobotNode() const {
            return robot_node_;
        }
        void setRobotState(const Eigen::VectorXd& robot_state);
        void visualizeTree();
        void visualizeTreeReal();
        void visualizePath(const std::vector<Eigen::VectorXd>& path_waypoints);
        void updateObstacleSamples(const ObstacleVector& obstacles);
        void addNewObstacle(const Obstacle& ob);
        void removeObstacle(const Obstacle& ob);
        void clearPlannerState();
        void dumpTreeToCSV(const std::string& filename) const;
        const ReplanMetrics& getLastReplanMetrics() const { return last_replan_metrics_; }
        void resetMetrics() { last_replan_metrics_ = ReplanMetrics(); }
        double getRobotTimeToGo() const { return robot_current_time_to_goal_; }
        bool isRobotSafe();
        int getTreeSize() { return tree_.size();}

        // Adds a batch of samples to the existing tree and updates structures
        void addBatchOfSamplesEager(int num_samples); // O(n log(n))
        double getAvgOutDegree() const {
            if (tree_.empty()) return 0.0;
            long long total_out = 0;
            for (const auto& node_ptr : tree_) {
                total_out += node_ptr->forwardNeighbors().size(); 
            }
            return static_cast<double>(total_out) / tree_.size();
        }

        double getAvgInDegree() const {
            if (tree_.empty()) return 0.0;
            long long total_in = 0;
            for (const auto& node_ptr : tree_) {
                total_in += node_ptr->backwardNeighbors().size(); 
            }
            return static_cast<double>(total_in) / tree_.size();
        }
        double getNeighborhoodRadius(){return neighborhood_radius_;}

        bool runCollisionForensics();
        bool runCostForensics();
        bool runGlobalCostForensics();
        bool runTreePropagationForensics();

        struct SuboptimalityMetrics {
            int total_nodes_updated = 0;
            int nodes_with_missed_better = 0;
            int total_missed_better_parents = 0;
            double total_cost_gap = 0.0;
            int nodes_with_cost_gap = 0;
            double max_cost_gap = 0.0;
            int revisits = 0;
            std::unordered_map<FMTNode*, bool> costUpdated; 
        };
        void analyzeSuboptimality(FMTNode* x, FMTNode* best_parent_for_x, FMTNode* z, SuboptimalityMetrics& metrics);
        void printDebugSummary(const SuboptimalityMetrics& metrics);

    private:
        bool updateNeighbors(const Eigen::VectorXd& sample_val, FMTNode* new_node);
        void cullNeighbors(FMTNode* v);
        void shrinkingBallRadius();



        std::vector<std::shared_ptr<FMTNode>> tree_;
        std::shared_ptr<KDTree> kdtree_;
        std::shared_ptr<StateSpace> statespace_;
        std::shared_ptr<ProblemDefinition> problem_;
        std::shared_ptr<Visualization> visualization_;
        std::shared_ptr<ObstacleChecker> obs_checker_;
        PriorityQueue<FMTNode, FMTComparator> v_open_heap_;
        FMTNode* robot_node_ = nullptr;
        int num_of_samples_;
        Eigen::VectorXd lower_bounds_;
        Eigen::VectorXd upper_bounds_;
        int root_state_index_;
        int robot_state_index_;
        double neighborhood_radius_;
        int k_neighbors_;
        bool obs_cache = false;
        bool partial_update = false;
        bool use_knn = false;
        double factor;
        
        int kd_dim ; 
        int dimension_;
        Eigen::VectorXd robot_continuous_state_;
        double robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();


        

        ReplanMetrics last_replan_metrics_; 
        std::unordered_map<std::string, Obstacle> previous_obstacles_;


        double delta;
        double bridge_cost_;
        bool is_geometric_mode_;


        double epsilon;
        Trajectory current_bridge_trajectory_;

};

