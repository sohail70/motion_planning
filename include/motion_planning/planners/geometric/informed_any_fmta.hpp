// Copyright 2025 Soheil E.Nia
#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/ifmt_node.hpp"
#include "motion_planning/utils/visualization.hpp"
#include "boost/container/flat_map.hpp"
#include "motion_planning/ds/priority_queue.hpp"
struct PairHash2 {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1);
    }
};

class InformedANYFMTA : public Planner {
 public:
            InformedANYFMTA(std::shared_ptr<StateSpace> statespace , std::shared_ptr<ProblemDefinition> problem_def , std::shared_ptr<ObstacleChecker> obs_checker);
            void setup(const Params& params, std::shared_ptr<Visualization> visualization) override;
            void plan() override;
            void plan2();
            // std::vector<size_t> getPathIndex() const;
            std::vector<Eigen::VectorXd> getPathPositions() const;
            void setStart(const Eigen::VectorXd& start) override;
            void setGoal(const Eigen::VectorXd& goal) override;
            void setRobotIndex(const Eigen::VectorXd& robot_position);


            void near(int node_index);
            std::vector<std::pair<std::shared_ptr<IFMTNode>,EdgeInfo>> near2(const std::vector<std::shared_ptr<IFMTNode>>& search_set,std::shared_ptr<IFMTNode> node, bool sort);
            void near2sample( const std::shared_ptr<IFMTNode>& node, std::vector<std::pair<std::shared_ptr<IFMTNode>,EdgeInfo >>& near_nodes);
            void near2tree( const std::shared_ptr<IFMTNode>& node, std::vector<std::pair<std::shared_ptr<IFMTNode>,EdgeInfo >>& near_nodes);

            void visualizeHeapAndUnvisited();
            // void visualizePath(std::vector<size_t> path_indices);
            void visualizeSmoothedPath(const std::vector<Eigen::VectorXd>& shortest_path_);
            std::vector<Eigen::VectorXd> getSmoothedPathPositions(int num_intermediates, int smoothing_window ) const;
            std::vector<Eigen::VectorXd> smoothPath(const std::vector<Eigen::VectorXd>& path, int window_size) const;
            std::vector<Eigen::VectorXd> interpolatePath(const std::vector<Eigen::VectorXd>& path, int num_intermediates) const;



            double heuristic(int current_index);
            void clearPlannerState();
            
            void addBatchOfSamples(int num_samples);
            void addBatchOfSamplesUninformed(int num_samples);

            void updateNeighbors(int node_index);
            Eigen::VectorXd sampleInEllipsoid(const Eigen::VectorXd& center, const Eigen::MatrixXd& R, double a, double b);
            Eigen::MatrixXd computeRotationMatrix(const Eigen::VectorXd& dir);
            Eigen::VectorXd sampleUnitBall(int d);


            std::vector<std::shared_ptr<IFMTNode>> getPathNodes() const;
            void visualizePath(const std::vector<std::shared_ptr<IFMTNode>>& path_nodes);
            void visualizeTree();

            void prune();
            void pruneSamples();

            void processNode(
                std::shared_ptr<IFMTNode> x,
                std::shared_ptr<IFMTNode> z,
                const EdgeInfo& cost_to_neighbor,
                bool is_sample_neighbor,
                std::vector<std::shared_ptr<IFMTNode>>& new_open_,
                bool& connected
            );


            
 private:
            std::shared_ptr<State> start_;
            std::shared_ptr<State> goal_;
            std::vector<std::shared_ptr<State>> path_;

            std::shared_ptr<KDTree> kdtree_samples_;
            std::shared_ptr<KDTree> kdtree_tree_;


            std::vector<std::shared_ptr<IFMTNode>> tree_;
            std::vector<std::shared_ptr<IFMTNode>> samples_;
            std::shared_ptr<IFMTNode> robot_node_;

//     PriorityQueue<EdgeCandidate,std::greater<EdgeCandidate> > edge_queue_;
    
            std::priority_queue<IEdgeCandidate, std::vector<IEdgeCandidate>, 
                        std::greater<IEdgeCandidate>> edge_queue_;
    


            std::shared_ptr<StateSpace> statespace_;
            std::shared_ptr<ProblemDefinition> problem_;
            
            std::shared_ptr<Visualization> visualization_;
            std::shared_ptr<ObstacleChecker> obs_checker_;


        //     PriorityQueue<IFMTNode, IFMTComparator> v_open_heap_;
            PriorityQueue2<IFMTNode, IFMTComparator> v_open_heap_;
            std::vector<std::shared_ptr<IFMTNode>> new_opens_;

            std::vector<std::shared_ptr<IFMTNode>> open_nodes;

            std::unordered_map<std::pair<int, int>, bool, pair_hash> obstacle_check_cache;

            Eigen::VectorXd robot_position_;


            int num_of_samples_;
            int num_batch_;
            double lower_bound_;
            double upper_bound_;
            double max_edge_length_;
            int root_state_index_;
            int robot_state_index_;
            bool use_kdtree;
            double neighborhood_radius_;
            bool obs_cache = false;
            bool partial_plot = false;

            double ci_ = std::numeric_limits<double>::infinity();

            int collision_check_ = 0;

            // Debug tracking members
            // std::unordered_set<std::pair<size_t, size_t>> processed_edges_;
            std::unordered_set<std::pair<size_t, size_t>, PairHash2> processed_edges_;

            size_t duplicate_checks_ = 0;
            size_t total_checks_ = 0;

            int d;
            double mu;
            double zetaD;
            double gamma;
            double factor;

};

