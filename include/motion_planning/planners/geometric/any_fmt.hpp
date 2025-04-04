// Copyright 2025 Soheil E.Nia
#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/fmt_node.hpp"
#include "motion_planning/utils/visualization.hpp"
#include "boost/container/flat_map.hpp"
#include "motion_planning/ds/priority_queue.hpp"


class ANYFMT : public Planner {
 public:
            ANYFMT(std::unique_ptr<StateSpace> statespace , std::shared_ptr<ProblemDefinition> problem_def , std::shared_ptr<ObstacleChecker> obs_checker);
            void setup(const Params& params, std::shared_ptr<Visualization> visualization) override;
            void plan() override;
            std::vector<size_t> getPathIndex() const;
            std::vector<Eigen::VectorXd> getPathPositions() const;
            void setStart(const Eigen::VectorXd& start) override;
            void setGoal(const Eigen::VectorXd& goal) override;
            void setRobotIndex(const Eigen::VectorXd& robot_position);


            void near(int node_index);

            void visualizeTree();
            void visualizeHeapAndUnvisited();
            void visualizePath(std::vector<size_t> path_indices);
            void visualizeSmoothedPath(const std::vector<Eigen::VectorXd>& shortest_path_);
            std::vector<Eigen::VectorXd> getSmoothedPathPositions(int num_intermediates, int smoothing_window ) const;
            std::vector<Eigen::VectorXd> smoothPath(const std::vector<Eigen::VectorXd>& path, int window_size) const;
            std::vector<Eigen::VectorXd> interpolatePath(const std::vector<Eigen::VectorXd>& path, int num_intermediates) const;



            double heuristic(int current_index);
            void clearPlannerState();
            
            void addBatchOfSamples(int num_samples);
            void updateNeighbors(int node_index);
 private:
            std::shared_ptr<State> start_;
            std::shared_ptr<State> goal_;
            std::vector<std::shared_ptr<State>> path_;
            std::vector<std::shared_ptr<FMTNode>> tree_;
            std::shared_ptr<KDTree> kdtree_;

            std::unique_ptr<StateSpace> statespace_;
            std::shared_ptr<ProblemDefinition> problem_;
            
            std::shared_ptr<Visualization> visualization_;
            std::shared_ptr<ObstacleChecker> obs_checker_;


            PriorityQueue<FMTNode, FMTComparator> v_open_heap_;

            std::unordered_map<std::pair<int, int>, bool, pair_hash> obstacle_check_cache;

            Eigen::VectorXd robot_position_;
            FMTNode* robot_node_;


            int num_of_samples_;
            int num_batch_;
            double lower_bound_;
            double upper_bound_;
            int root_state_index_;
            int robot_state_index_;
            bool use_kdtree;
            double neighborhood_radius_;
            bool obs_cache = false;
            bool partial_plot = false;

            int d;
            double mu;
            double zetaD;
            double gamma;
            double factor;

};

