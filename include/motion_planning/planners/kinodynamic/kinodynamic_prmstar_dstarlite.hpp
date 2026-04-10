#pragma once

#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/dstar_lite_node.hpp"
#include "motion_planning/utils/visualization.hpp"
#include "motion_planning/ds/priority_queue.hpp" // Your custom PQ
#include "motion_planning/pch.hpp"


#define DSTARLITE_INFO(msg)  std::cout << "\033[0;32m[INFO] " << msg << "\033[0m\n"
#define DSTARLITE_WARN(msg)  std::cout << "\033[1;33m[WARN] " << msg << "\033[0m\n"
#define DSTARLITE_ERROR(msg) std::cerr << "\033[1;31m[ERROR] " << msg << "\033[0m\n"
#define DSTARLITE_FATAL(msg) std::cerr << "\033[1;37;41m[FATAL] " << msg << "\033[0m\n"

// Use the standard DStarLiteComparator which expects std::pair<double, Node*>
// This matches your existing priority_queue.hpp definition
using DStarLitePQ = PriorityQueue<DStarLiteNode, DStarLiteComparator>;

class KinodynamicPRMStarDStarLite : public Planner {
    public:
        KinodynamicPRMStarDStarLite(std::shared_ptr<StateSpace> statespace, 
                                    std::shared_ptr<ProblemDefinition> pdef,
                                    std::shared_ptr<ObstacleChecker> obs_checker);

        virtual void setup(const Params& params, std::shared_ptr<Visualization> visualization) override;
        virtual void plan() override;
        
        virtual void setStart(const Eigen::VectorXd& start) override;
        virtual void setGoal(const Eigen::VectorXd& goal) override;

        void updateObstacles(const ObstacleVector& turned_obstacles);
        std::vector<Eigen::VectorXd> getPathPositions() const;

        void setRobotState(const Eigen::VectorXd& robot_state);
        void visualizeTree();
        void visualizeTreeGradient();
        void visualizePath(const std::vector<Eigen::VectorXd>& path_waypoints);
        void visualizePathGradient(const std::vector<Eigen::VectorXd>& path_waypoints);

        bool recomputeRHS(DStarLiteNode* s);
        const ReplanMetrics& getLastReplanMetrics() const { return last_replan_metrics_; }
        void resetMetrics() { last_replan_metrics_ = ReplanMetrics(); }


        int getTreeSize() { return nodes_.size();}

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
        void checkIsolatedNodes() const;

        void logGraphState(std::ofstream& out_file, int cycle_number) const override {
            // یک تابع لامبدا برای تبدیل NeighborMap به رشته متنی (فرمت: id:cost|id:cost)
            auto serializeNeighbors = [](const DStarLiteNode::NeighborMap& nmap) {
                std::string s = "";
                for (const auto& pair : nmap) {
                    s += std::to_string(pair.first->getIndex()) + ":" + std::to_string(pair.second.distance) + "|";
                }
                if (!s.empty()) {
                    s.pop_back(); // حذف آخرین کاراکتر '|'
                }
                return s;
            };

            for (const auto& node : nodes_) {
                if (!node) continue;
                
                auto state = node->getStateValue();
                
                out_file << cycle_number << ","
                        << node->getIndex() << ","
                        << state[0] << "," << state[1] << ","
                        << node->rhs << ","
                        << (node->best_parent_ ? node->best_parent_->getIndex() : -1) << ","
                        << serializeNeighbors(node->forward_neighbors_) << ","
                        << serializeNeighbors(node->backward_neighbors_) 
                        << "\n";
            }
        }




    private:
        // --- D* Lite Core ---
        double heuristic(DStarLiteNode* a, DStarLiteNode* b);
        
        // Updates the node's internal priority_key_ based on g, rhs, and heuristic
        void updateNodePriority(DStarLiteNode* u);

        void initialize(DStarLiteNode* start, DStarLiteNode* goal);
        void updateVertex(DStarLiteNode* u);
        void computeShortestPath();
        double computeShortestPathDijkstraMode();

        // --- PRM* Construction ---
        void near(int node_index); // NEW: Neighbor caching function
        void connectNeighbors(DStarLiteNode* u); // This might become redundant if near does it all
        std::vector<DStarLiteNode*> getNeighbors(DStarLiteNode* u);
        DStarLiteNode* findNearestNode(const Eigen::VectorXd& state);

        // --- Dynamic Obstacle Handling ---
        void addNewObstacle(const Obstacle& ob);
        void removeObstacle(const Obstacle& ob);

        // --- Members ---
        std::shared_ptr<StateSpace> statespace_;
        std::shared_ptr<ProblemDefinition> problem_def_;
        std::shared_ptr<ObstacleChecker> obs_checker_;
        std::shared_ptr<Visualization> visualization_;
        
        Eigen::VectorXd robot_continuous_state_; 
        Eigen::VectorXd lower_bounds_;
        Eigen::VectorXd upper_bounds_;

        std::vector<std::unique_ptr<DStarLiteNode>> nodes_;
        
        // DStarLitePQ open_queue_; 
        DStarLitePriorityQueue open_queue_;
        
        DStarLiteNode* start_node_;
        DStarLiteNode* goal_node_;
        double km_;
        bool partial_update = false;

        int num_samples_;
        double connection_radius_;
        bool use_kdtree_;
        int kd_dim_;
        std::shared_ptr<KDTree> kdtree_;
        bool use_knn_;
        int k_neighbors_;
        double factor_;
        double gamma_;
        double mu_;
        double zetaD_;
        
        double bridge_cost_;

        ReplanMetrics last_replan_metrics_; 
        bool neighbor_precache_; // Parameter from config

        // --- Helper to cache neighbors for a specific node ---
        void cacheNeighbors(DStarLiteNode* u);

        std::unordered_map<std::string, Obstacle> previous_obstacles_;

        bool is_geometric_mode_;

        DStarLiteNode* last_start_node = nullptr; 
        std::unordered_map<DStarLiteNode*, DStarLiteNode*> dijkstra_tree_parents_;


        int grid_dim_per_side_; // Stores the N for an NxN grid
        bool use_grid_sampling_;


        void debugCompareDijkstraVsDStarLite();
        
        DStarLiteKey calculateKey(DStarLiteNode* u);
        std::unordered_set<DStarLiteNode*> orphans_;
        Trajectory current_bridge_trajectory_;
        double global_max_cost_ = -1;

        void injectTimePillarNodes(const Eigen::VectorXd& goal_state_val, int num_pillar_nodes);

        int num_pillar_nodes_;
        std::unordered_set<int> time_pillar_indices_;
        
};