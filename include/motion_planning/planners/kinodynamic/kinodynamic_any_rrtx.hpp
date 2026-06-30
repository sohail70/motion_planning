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


class KinodynamicANYRRTX : public Planner {
    public:
        KinodynamicANYRRTX(std::shared_ptr<StateSpace> statespace, 
            std::shared_ptr<ProblemDefinition> problem_def,
            std::shared_ptr<ObstacleChecker> obs_checker);
        
        void setup(const Params& params, std::shared_ptr<Visualization> visualization) override;
        void plan() override;
        std::vector<Eigen::VectorXd> getPathPositions() const;

        void setStart(const Eigen::VectorXd& start) override;
        void setGoal(const Eigen::VectorXd& goal) override;
        void clearPlannerState() ;

        void updateObstacles(const ObstacleVector& obstacles);
        void visualizeTree();
        void visualizeTreeGradient();
        void visualizeTreeReal();
        void visualizePath(const std::vector<Eigen::VectorXd>& path_waypoints);
        void visualizePathGradient(const std::vector<Eigen::VectorXd>& path_waypoints);

        void setRobotState(const Eigen::VectorXd& robot_state);

        void dumpTreeToCSV(const std::string& filename) const;


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
        int getTreeSize() { return tree_.size()-num_pillar_nodes_;}

        bool runCollisionForensics();
        bool runCostForensics();

        struct ScalingMetrics {
            long long total_samples = 0;
            long long total_extend_edges = 0;  // Edges checked during extend()
            long long total_rewire_edges = 0;  // Edges checked during rewireNeighbors()
            long long total_lmc_edges = 0;     // Edges checked during updateLMC()
            long long total_reduce_iterations = 0; // Total pops from inconsistency queue
        };
        ScalingMetrics metrics_;



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

        void logGraphState(std::ofstream& out_file, int cycle_number) const override {
            const int robot_anchor_id =
                vbot_node_ ? vbot_node_->getIndex() : -1;

            const double robot_g =
                vbot_node_ ? vbot_node_->getG() : std::numeric_limits<double>::infinity();

            const double robot_lmc =
                vbot_node_ ? vbot_node_->getLMC() : std::numeric_limits<double>::infinity();

            const double bridge_cost = bridge_cost_;

            const double robot_total_cost =
                (vbot_node_ && std::isfinite(robot_g) && std::isfinite(bridge_cost))
                    ? (robot_g + bridge_cost)
                    : std::numeric_limits<double>::infinity();

            const double robot_time_to_goal = robot_current_time_to_goal_;

            for (const auto& node : tree_) {
                if (!node) continue;

                auto state = node->getStateValue();
                const bool is_robot_anchor = (vbot_node_ && node.get() == vbot_node_);

                out_file << cycle_number << ","
                        << node->getIndex() << ","
                        << state[0] << "," << state[1] << ","
                        << node->getG() << ","
                        << node->getLMC() << ","
                        << (node->getParent() ? node->getParent()->getIndex() : -1) << ","
                        << (is_robot_anchor ? 1 : 0) << ","
                        << robot_anchor_id << ","
                        << robot_g << ","
                        << robot_lmc << ","
                        << bridge_cost << ","
                        << robot_total_cost << ","
                        << robot_time_to_goal
                        << "\n";
            }
        }


        bool isCurrentBridgeSafe(const ObstacleVector& obstacles) const override;
        bool hasReachedAnchor(const Eigen::VectorXd& current_sim_state) const override;
        std::vector<Eigen::VectorXd> getLivePathPositions(const Eigen::VectorXd& current_state) const;
        bool hasShortcut(const Eigen::VectorXd& robot_state, double threshold);
        void setCurrentRobotTime(double T_robot);

    private:
        std::vector<std::unique_ptr<RRTxNode>> tree_;
        std::shared_ptr<KDTree> kdtree_;
        PriorityQueue<RRTxNode, RRTxComparator> inconsistency_queue_;
        
        std::shared_ptr<StateSpace> statespace_;
        std::shared_ptr<ProblemDefinition> problem_;
        std::shared_ptr<ObstacleChecker> obs_checker_;
        std::shared_ptr<Visualization> visualization_;

        RRTxNode*  vbot_node_ = nullptr;  // robot anchor; nullptr until setGoal/setRobotState (read by the time-cone anchor guard)
        std::unordered_set<int> Vc_T_;
        double neighborhood_radius_;
        double epsilon_;
        double gamma_;
        double delta = 20.0; 
        double factor;
        double T_robot = std::numeric_limits<double>::infinity();  // +inf => time-cone prune is a no-op until setCurrentRobotTime()
        int root_state_index_;
        int num_of_samples_;
        int dimension_;
        size_t sample_counter = 0;
        bool cap_samples_ = true;
        bool partial_update;
        Eigen::VectorXd lower_bounds_;
        Eigen::VectorXd upper_bounds_;
        bool use_kdtree;
        int kd_dim ; 
        bool static_obs_presence;
        Eigen::VectorXd robot_continuous_state_;
        double robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
        Trajectory robot_bridge_trajectory_;
        bool extend(Eigen::VectorXd v);
        void rewireNeighbors(RRTxNode* v);
        void reduceInconsistency();
        void cullNeighbors(RRTxNode* v);
        void updateLMC(RRTxNode* v);
        void verifyQueue(RRTxNode* node);
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
        Trajectory current_bridge_trajectory_;
        double global_max_cost_ = -1;
        void injectTimePillarNodes(const Eigen::VectorXd& goal_state_val, int num_pillar_nodes);
        int num_pillar_nodes_;
        std::unordered_set<int> time_pillar_indices_;
};