#pragma once

#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/dstar_lite_node.hpp"
#include "motion_planning/utils/visualization.hpp"
#include "motion_planning/ds/priority_queue.hpp" 
#include "motion_planning/pch.hpp"

#define LLPT_INFO(msg)  std::cout << "\033[0;36m[INFO] [LLPT] " << msg << "\033[0m\n"
#define LLPT_WARN(msg)  std::cout << "\033[1;33m[WARN] [LLPT] " << msg << "\033[0m\n"
#define LLPT_ERROR(msg) std::cerr << "\033[1;31m[ERROR] [LLPT] " << msg << "\033[0m\n"



class KinodynamicLLPTStar : public Planner {
    public:
        KinodynamicLLPTStar(std::shared_ptr<StateSpace> statespace, 
                        std::shared_ptr<ProblemDefinition> pdef,
                        std::shared_ptr<ObstacleChecker> obs_checker);

        virtual void setup(const Params& params, std::shared_ptr<Visualization> visualization) override;
        virtual void plan() override;
        
        virtual void setStart(const Eigen::VectorXd& start) override;
        virtual void setGoal(const Eigen::VectorXd& goal) override;

        void updateObstacles(const ObstacleVector& turned_obstacles);
        std::vector<Eigen::VectorXd> getPathPositions() const;

        void setRobotState(const Eigen::VectorXd& robot_state, bool anchor_was_reached);
        
        // --- Visualization & Metrics ---
        void visualizeTree();
        void visualizeTreeGradient();
        void visualizePath(const std::vector<Eigen::VectorXd>& path_waypoints);
        void visualizePathGradient(const std::vector<Eigen::VectorXd>& path_waypoints);

        const ReplanMetrics& getLastReplanMetrics() const { return last_replan_metrics_; }
        void resetMetrics() { last_replan_metrics_ = ReplanMetrics(); }
        
        int getTreeSize() { return nodes_.size()-num_pillar_nodes_; }
        double getAvgOutDegree() const {
            if (nodes_.empty()) return 0.0;
            long long total_out = 0;
            for (const auto& node_ptr : nodes_) {
                total_out += node_ptr->forward_neighbors_.size(); 
            }
            return static_cast<double>(total_out) / nodes_.size();
        }

        double getAvgInDegree() const {
            if (nodes_.empty()) return 0.0;
            long long total_in = 0;
            for (const auto& node_ptr : nodes_) {
                total_in += node_ptr->backward_neighbors_.size(); 
            }
            return static_cast<double>(total_in) / nodes_.size();
        }

        double getNeighborhoodRadius(){return connection_radius_;}
        double getLastAnchorRepairMs() const { return last_anchor_repair_ms_; }
        void resetLastAnchorRepairMs() { last_anchor_repair_ms_ = 0.0; }


        bool isCurrentBridgeSafe(const ObstacleVector& obstacles) const;
        bool hasReachedAnchor(const Eigen::VectorXd& current_sim_state) const;
        std::vector<Eigen::VectorXd> getLivePathPositions(const Eigen::VectorXd& current_state) const;
        bool hasShortcut(const Eigen::VectorXd& robot_state, double threshold) override;
        void setCurrentRobotTime(double T_robot);


    private:
        int collision_checked_ = 0;
        void near(int node_index); // NEW: Neighbor caching function
        // --- LLPT Core Procedures (from paper Algorithms 1-6) ---
        
        // Calculates priority keys [k1, k2] matching LLPT logic
        DStarLiteKey calculateKey(DStarLiteNode* u);
        
        // Equivalent to heuristic lower bound of edge weights in LLPT
        double heuristic(DStarLiteNode* a, DStarLiteNode* b);
        
        void initialize(DStarLiteNode* start, DStarLiteNode* goal);

        // Algo 5: UpdateNode(v, u) - Evaluates inconsistency and pushes to queue
        void updateNode(DStarLiteNode* v, DStarLiteNode* u);
        void updateVertex(DStarLiteNode* u); // Helper for single node queue update
        
        // Algo 4: ComputeShortestPath() - Repairs the spanning tree
        void computeShortestPath();
        void resolvePathLazy();

        // Algo 2: EvaluateEdge(N) - Lazily checks collisions on the current shortest path
        // Returns the list of nodes that became invalidated (collided)
        std::vector<DStarLiteNode*> evaluateEdge(int max_evaluations);

        // Algo 3: PropagateCostToLeave(v) - Cascades infinity cost to descendants when an edge breaks
        void propagateCostToLeave(DStarLiteNode* v);
        void removeFromTree(DStarLiteNode* v);

        // Algo 6: ExtendSearchGraph() - Graph densification during leftover planning time
        bool extendSearchGraph();
        Eigen::VectorXd saturate(const Eigen::VectorXd& newPoint, const Eigen::VectorXd& closestPoint, double delta);

        // Returns true if lmc (rhs) or best parent changed
        bool recomputeLMC(DStarLiteNode* s); 

        // --- Dynamic Obstacle Handling ---
        void addNewObstacle(const Obstacle& ob);
        void removeObstacle(const Obstacle& ob);

        // --- Geometric / Kinodynamic extensions ---
        void near(DStarLiteNode* new_node); // Neighbors for newly sampled nodes
        std::vector<DStarLiteNode*> getNeighbors(DStarLiteNode* u);

        // --- Members ---
        std::shared_ptr<StateSpace> statespace_;
        std::shared_ptr<ProblemDefinition> problem_def_;
        std::shared_ptr<ObstacleChecker> obs_checker_;
        std::shared_ptr<Visualization> visualization_;
        
        std::vector<std::unique_ptr<DStarLiteNode>> nodes_;
        DStarLitePriorityQueue open_queue_;
        
        DStarLiteNode* start_node_=nullptr;
        DStarLiteNode* goal_node_=nullptr;
        
        double km_; // Key modifier for moving start node
        bool partial_update_ = false;

        // LLPT Specific Parameters
        int num_samples_;
        int eval_batch_size_ = 100; // N parameter from LLPT paper (edges evaluated per inner loop)
        
        // PRM/Graph parameters
        double connection_radius_;
        int kd_dim_;
        std::shared_ptr<KDTree> kdtree_;
        bool use_knn_;
        int k_neighbors_;
        bool is_geometric_mode_;
        
        ReplanMetrics last_replan_metrics_;
        std::unordered_map<std::string, Obstacle> previous_obstacles_;

        // --- Caching & Helpers ---
        std::unordered_set<int> time_pillar_indices_;
        int num_pillar_nodes_;
        void injectTimePillarNodes(const Eigen::VectorXd& goal_state_val, int num_pillar_nodes);


        // --- Members ---
        
        Eigen::VectorXd robot_continuous_state_; 
        Eigen::VectorXd lower_bounds_;
        Eigen::VectorXd upper_bounds_;


        bool use_kdtree_;
        bool use_heuristic;
        double factor_;
        double delta; 
        double gamma_;
        double mu_;
        double zetaD_;
        double T_robot = std::numeric_limits<double>::infinity();  // +inf => time-cone prune is a no-op until setCurrentRobotTime()
        
        double bridge_cost_;

        bool neighbor_precache_; // Parameter from config




        DStarLiteNode* last_start_node = nullptr; 
        std::unordered_map<DStarLiteNode*, DStarLiteNode*> dijkstra_tree_parents_;





bool isPathFullyEvaluated() const {
    DStarLiteNode* cur = start_node_;
    while (cur && cur != goal_node_) {
        DStarLiteNode* parent = cur->getParent();
        if (!parent) return false;
        if (cur->forward_neighbors_.at(parent).last_eval_epoch != global_eval_epoch_)
            return false;
        cur = parent;
    }
    return true;
}

        void debugCompareDijkstraVsDStarLite();
        
        std::unordered_set<DStarLiteNode*> orphans_;
        Trajectory current_bridge_trajectory_;
        double global_max_cost_ = -1;


        double last_anchor_repair_ms_ = 0.0; // computeshortstpath duration in the setRobotState in case heuristic is on!

        
        // NEW
        uint64_t global_eval_epoch_ = 1; // Start at one because extend function adds new edges with zero epoch so this ensured collision checking happens right after new samples are added and made the shortest path shorter!!
        std::vector<std::pair<DStarLiteNode*, DStarLiteNode*>> collided_edges_;

        // // Add this helper function:
        // double getLazyWeight(const EdgeInfo& edge) const {
        //     if (edge.last_eval_epoch != global_eval_epoch_)
        //         return edge.distance_original;
        //     return edge.distance;
        // }

// double getLazyWeight(const EdgeInfo& edge) const {
//     if (edge.last_eval_epoch != 0)   // has been evaluated at least once
//         return edge.distance;        // could be finite or inf
//     return edge.distance_original;   // never evaluated – optimistic
// }


double getLazyWeight(const EdgeInfo& edge) const {
    if (edge.last_eval_epoch == global_eval_epoch_)
        return edge.distance;
    return edge.distance_original;
}






};