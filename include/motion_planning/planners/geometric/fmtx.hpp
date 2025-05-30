// Copyright 2025 Soheil E.Nia
#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/fmt_node.hpp"
#include "motion_planning/utils/visualization.hpp"
#include "motion_planning/ds/priority_queue.hpp"

class FMTX : public Planner {
 public:
            FMTX(std::shared_ptr<StateSpace> statespace , std::shared_ptr<ProblemDefinition> problem_def , std::shared_ptr<ObstacleChecker> obs_checker);
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



            std::unordered_set<int> findSamplesNearObstacles(const std::vector<Obstacle>& obstacles, double scale_factor);
            std::pair<std::unordered_set<int>,std::unordered_set<int>> findSamplesNearObstaclesDual(const std::vector<Obstacle>& obstacles, double scale_factor);

            void updateObstacleSamples(const std::vector<Obstacle>& obstacles);
            std::unordered_set<int> getDescendants(int node_index);



            void handleAddedObstacleSamples(const std::vector<int>& added);
            void handleRemovedObstacleSamples(const std::vector<int>& removed);



            double heuristic(int current_index);



            void clearPlannerState();

 private:
            std::shared_ptr<State> start_;
            std::shared_ptr<State> goal_;
            std::vector<std::shared_ptr<State>> path_;
            std::vector<std::shared_ptr<FMTNode>> tree_;
            std::shared_ptr<KDTree> kdtree_;

            std::shared_ptr<StateSpace> statespace_;
            std::shared_ptr<ProblemDefinition> problem_;
            
            std::shared_ptr<Visualization> visualization_;
            std::shared_ptr<ObstacleChecker> obs_checker_;


            PriorityQueue<FMTNode, FMTComparator> v_open_heap_;
            Eigen::VectorXd robot_position_;
            FMTNode* robot_node_;

            std::unordered_set<int> samples_in_obstacles_; // Current samples in obstacles
            std::unordered_set<int> current_; // Current samples in obstacles

            std::unordered_map<int , double> edge_length_;
            int max_length_edge_ind = -1;
            double max_length = -std::numeric_limits<double>::infinity();

            int checks = 0;

            int num_of_samples_;
            double lower_bound_;
            double upper_bound_;
            int root_state_index_;
            int robot_state_index_;
            bool use_kdtree;
            double neighborhood_radius_;
            bool obs_cache = false;
            bool partial_plot = false;
            bool use_heuristic = false;
            bool partial_update = false;
            bool ignore_sample;
            bool prune;

            bool in_dynamic = false;

            double mu;
            double zetaD;
            double gamma;
            double factor;

            std::unordered_set<int> v_open_set_;

            std::unordered_set<int> dir;
            


            bool static_obs_presence;
            std::vector<Obstacle> seen_statics_;

            

};

