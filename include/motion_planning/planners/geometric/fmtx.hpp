// Copyright 2025 Soheil E.Nia
#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/tree_node.hpp"
#include "motion_planning/utils/visualization.hpp"

struct NeighborInfo {
    int index;
    double distance;
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




class FMTX : public Planner {
 public:
            FMTX(std::unique_ptr<StateSpace> statespace , std::unique_ptr<ProblemDefinition> problem_def , std::shared_ptr<ObstacleChecker> obs_checker);
            void setup(const PlannerParams& params, std::shared_ptr<Visualization> visualization) override;
            void plan() override;
            std::vector<int> getPathIndex() const override;
            void setStart(const Eigen::VectorXd& start) override;
            void setGoal(const Eigen::VectorXd& goal) override;
            void setRobotIndex(const Eigen::VectorXd& robot_position);

            // int getGoalIndex() {return robot_state_index_;}
            // int getStarIndex() {return root_state_index_;}

            std::vector<NeighborInfo> near(int node_index);
            void visualizeTree();
            void visualizePath(std::vector<int> path_indices);

            std::unordered_set<int> findSamplesNearObstacles(const std::vector<Obstacle>& obstacles, double scale_factor);
            std::pair<std::unordered_set<int>,std::unordered_set<int>> findSamplesNearObstaclesDual(const std::vector<Obstacle>& obstacles, double scale_factor);

            void updateObstacleSamples(const std::vector<Obstacle>& obstacles);
            std::unordered_set<int> getDescendants(int node_index);
            // std::unordered_set<int> getDescendants(const std::vector<int>& node_index);



            void handleAddedObstacleSamples(const std::vector<int>& added);
            void handleRemovedObstacleSamples(const std::vector<int>& removed);

            void handleInflatedZoneAdditions(const std::vector<int>& added_inflated);


            double heuristic(int current_index);

            bool isValidYnear(int index, 
                                    const std::unordered_set<int>& v_open_set, 
                                    const std::vector<std::unordered_set<int>>& invalid_connections, 
                                    int xIndex, 
                                    bool use_heuristic);

 private:
            std::shared_ptr<State> start_;
            std::shared_ptr<State> goal_;
            std::vector<std::shared_ptr<State>> path_;
            std::vector<std::shared_ptr<TreeNode>> tree_;
            std::shared_ptr<KDTree> kdtree_;

            std::unique_ptr<StateSpace> statespace_;
            std::unique_ptr<ProblemDefinition> problem_;
            
            std::shared_ptr<Visualization> visualization_;
            std::shared_ptr<ObstacleChecker> obs_checker_;


            std::unordered_set<int> v_open_set_;
            std::unordered_set<int> v_unvisited_set_;
            // std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, std::greater<>> v_open_heap_;

            PriorityQueue v_open_heap_;

            std::unordered_set<int> boundary_;


            std::unordered_map<int, std::vector<NeighborInfo>> neighbors_dict_;
            std::unordered_set<int> samples_in_obstacles_; // Current samples in obstacles
            std::unordered_set<int> samples_in_obstacles_2_; // Current samples in obstacles
            std::unordered_set<int> inflated_samples_;


            std::unordered_map<int , double> edge_length_;
            int max_length_edge_ind = -1;
            double max_length = -std::numeric_limits<double>::infinity();

            int num_of_samples_;
            double lower_bound_;
            double upper_bound_;
            int root_state_index_;
            int robot_state_index_;
            bool use_kdtree;
            double neighborhood_radius_;
            bool obs_cache = true;
            // bool use_range = false; // THIS SHOULD BE USED IN THE OBSTALCE CHECKER LEVEL NOT IN THE PLANNER LEVEL! --> LATER REMOVE THIS
            bool partial_plot = true;
            // bool inflation = false;
            bool use_heuristic = false;


            // std::vector<std::vector<bool>> invalid_best_neighbors;
            // std::vector<std::vector<bool>> invalid_best_neighbors;
            std::vector<std::unordered_set<int>> invalid_best_neighbors; // Sparse storage of invalid neighbors



            // int current_timestamp = 0; 
            

};

