#pragma once

#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/dstar_lite_node.hpp"
#include "motion_planning/utils/visualization.hpp"
#include "motion_planning/ds/priority_queue.hpp" 
#include "motion_planning/pch.hpp"

#define LLPT_INFO(msg)  std::cout << "\033[0;36m[INFO] [LLPT] " << msg << "\033[0m\n"
#define LLPT_WARN(msg)  std::cout << "\033[1;33m[WARN] [LLPT] " << msg << "\033[0m\n"
#define LLPT_ERROR(msg) std::cerr << "\033[1;31m[ERROR] [LLPT] " << msg << "\033[0m\n"

// Reusing your custom priority queue logic
using LLPT_PQ = PriorityQueue<DStarLiteNode, DStarLiteComparator>;

class KinodynamicLLPT : public Planner {
    public:
        KinodynamicLLPT(std::shared_ptr<StateSpace> statespace, 
                        std::shared_ptr<ProblemDefinition> pdef,
                        std::shared_ptr<ObstacleChecker> obs_checker);

        virtual void setup(const Params& params, std::shared_ptr<Visualization> visualization) override;
        virtual void plan() override;
        
        virtual void setStart(const Eigen::VectorXd& start) override;
        virtual void setGoal(const Eigen::VectorXd& goal) override;

        void updateObstacles(const ObstacleVector& turned_obstacles);
        std::vector<Eigen::VectorXd> getPathPositions() const;

        void setRobotState(const Eigen::VectorXd& robot_state);
        
        // --- Visualization & Metrics ---
        void visualizeTree();
        void visualizePath(const std::vector<Eigen::VectorXd>& path_waypoints);
        
        const ReplanMetrics& getLastReplanMetrics() const { return last_replan_metrics_; }
        void resetMetrics() { last_replan_metrics_ = ReplanMetrics(); }
        
        int getTreeSize() { return nodes_.size(); }
        void logGraphState(std::ofstream& out_file, int cycle_number) const override;

    private:
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

        // Algo 2: EvaluateEdge(N) - Lazily checks collisions on the current shortest path
        // Returns the list of nodes that became invalidated (collided)
        std::vector<DStarLiteNode*> evaluateEdge(int max_evaluations);

        // Algo 3: PropagateCostToLeave(v) - Cascades infinity cost to descendants when an edge breaks
        void propagateCostToLeave(DStarLiteNode* v);
        void removeFromTree(DStarLiteNode* v);

        // Algo 6: ExtendSearchGraph() - Graph densification during leftover planning time
        void extendSearchGraph();

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
        LLPT_PQ open_queue_;
        
        DStarLiteNode* start_node_;
        DStarLiteNode* goal_node_;
        
        double km_; // Key modifier for moving start node
        bool partial_update_ = false;

        // LLPT Specific Parameters
        int num_initial_samples_;
        int eval_batch_size_; // N parameter from LLPT paper (edges evaluated per inner loop)
        double max_planning_time_ms_; // Bounds the ExtendSearchGraph time limits
        
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
};