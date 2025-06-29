// Copyright 2025 Soheil E.Nia
#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/fmt_node.hpp"
#include "motion_planning/utils/visualization.hpp"
#include "motion_planning/ds/priority_queue.hpp"

class KinodynamicFMTX : public Planner {
 public:
            KinodynamicFMTX(std::shared_ptr<StateSpace> statespace , std::shared_ptr<ProblemDefinition> problem_def , std::shared_ptr<ObstacleChecker> obs_checker);
            void setup(const Params& params, std::shared_ptr<Visualization> visualization) override;
            void plan() override;
            std::vector<size_t> getPathIndex() const;
            std::vector<Eigen::VectorXd> getPathPositions() const;
            void setStart(const Eigen::VectorXd& start) override;
            void setGoal(const Eigen::VectorXd& goal) override;
            void setRobotIndex(const Eigen::VectorXd& robot_position);

            FMTNode* getRobotNode() const {
                return robot_node_;
            }

            bool isPathStillValid(const std::vector<Eigen::VectorXd>& path, const Eigen::VectorXd& current_robot_state) const {
                if (path.size() < 2) {
                    return true; // An empty or single-point path is considered valid.
                }

                // Get the global time context from the robot's current state.
                const double t_now = clock_->now().seconds();

                // The robot's total time-to-go is the 3rd element (index 2) of its state vector.
                // This path comes from getPathPositions(), which correctly starts with the robot's continuous state.
                const double robot_total_time_to_go = path.front()(2);

                // Calculate the single, predicted global time of arrival at the goal for this entire path.
                const double t_arrival = t_now + robot_total_time_to_go;

                // Check every segment of the path using the predictive, time-aware checker.
                for (size_t i = 0; i < path.size() - 1; ++i) {
                    const Eigen::VectorXd& segment_start_state = path[i];
                    const Eigen::VectorXd& segment_end_state = path[i+1];

                    // The obstacle checker expects a Trajectory object. We build one for the segment.
                    Trajectory segment_traj;
                    segment_traj.path_points.push_back(segment_start_state); //
                    segment_traj.path_points.push_back(segment_end_state); //
                    segment_traj.time_duration = segment_start_state(2) - segment_end_state(2); // Time-to-go must decrease.
                    
                    // If duration is non-positive, the path's time ordering is broken.
                    if (segment_traj.time_duration <= 1e-6) {
                        RCLCPP_WARN(rclcpp::get_logger("fmtx_validator"), "Path invalidated by non-positive time duration on segment %zu.", i);
                        return false;
                    }

                    // Calculate the global time at which the robot is scheduled to START this specific segment.
                    const double segment_global_start_time = t_arrival - segment_start_state(2);

                    // Perform the full predictive check, identical to the one inside the planner.
                    if (!obs_checker_->isTrajectorySafe(segment_traj, segment_global_start_time)) { //
                        // If any segment is predicted to be in collision, the entire path is invalid.
                        RCLCPP_WARN(rclcpp::get_logger("fmtx_validator"), "Path invalidated by predictive check on segment %zu.", i);
                        return false;
                    }
                }

                // If all segments are predictively safe, the path is still valid.
                return true;
            }
    
            bool arePathsSimilar(const std::vector<Eigen::VectorXd>& path_a, const std::vector<Eigen::VectorXd>& path_b, double tolerance) const {
                // If paths have different numbers of waypoints, they are not similar.
                if (path_a.size() != path_b.size()) {
                    return false;
                }

                // Check each waypoint pair for proximity.
                for (size_t i = 0; i < path_a.size(); ++i) {
                    // If the distance between corresponding points is greater than the tolerance,
                    // the paths are different.
                    if ((path_a[i] - path_b[i]).norm() > tolerance) {
                        return false;
                    }
                }

                // If all waypoints are within the tolerance, the paths are considered similar.
                return true;
            }



            /**
             * @brief Re-anchors the planner's search to the robot's current continuous state.
             * Finds the best node in the tree to serve as the new starting point (leaf node)
             * for the backward search, considering both proximity and existing cost. This is
             * the crucial link between the continuous simulation and the discrete graph.
             * @param robot_state The current 3D state (x,y,Time) of the robot.
             */
void setRobotState(const Eigen::VectorXd& robot_state) {

    robot_continuous_state_ = robot_state;


    // if (tree_.empty() || robot_state.size() != 3) {
    //     robot_node_ = nullptr;
    //     return;
    // }

    // // 1. Store the continuous state internally for getExecutablePath to use.
    // robot_continuous_state_ = robot_state;

    // // 2. Find the best discrete node in the tree to anchor the search.
    // const double search_radius = 10.0;
    // auto nearby_indices = kdtree_->radiusSearch(robot_continuous_state_.head<2>(), search_radius);

    // FMTNode* best_node_to_connect = nullptr;
    // double min_cost_via_node = std::numeric_limits<double>::infinity();

    // for (const auto& index : nearby_indices) {
    //     FMTNode* candidate_node = tree_[index].get();
    //     if (candidate_node->getCost() == INFINITY) continue;

    //     Trajectory traj = statespace_->steer(robot_continuous_state_, candidate_node->getStateValue());
        
    //     if (traj.is_valid && obs_checker_->isTrajectorySafe(traj, clock_->now().seconds())) {
    //         double cost_via_candidate = traj.cost + candidate_node->getCost();
    //         if (cost_via_candidate < min_cost_via_node) {
    //             min_cost_via_node = cost_via_candidate;
    //             best_node_to_connect = candidate_node;
    //         }
    //     }
    // }
    
    // // Set the robot's anchor node to the best one found.
    // if (best_node_to_connect) {
    //     robot_node_ = best_node_to_connect;
    // } else {
    //     // Fallback if no valid connection is found
    //     auto nearest_idx_vec = kdtree_->knnSearch(robot_continuous_state_.head<2>(), 1);
    //     if (!nearest_idx_vec.empty()) {
    //         robot_node_ = tree_[nearest_idx_vec[0]].get();
    //     } else {
    //         robot_node_ = nullptr;
    //     }
    // }
}


            void near(int node_index);

            void visualizeTree();
            void visualizeHeapAndUnvisited();
            void visualizePath(const std::vector<Eigen::VectorXd>& path_waypoints);
            void visualizeSmoothedPath(const std::vector<Eigen::VectorXd>& shortest_path_);
            std::vector<Eigen::VectorXd> getSmoothedPathPositions(int num_intermediates, int smoothing_window ) const;
            std::vector<Eigen::VectorXd> smoothPath(const std::vector<Eigen::VectorXd>& path, int window_size) const;
            std::vector<Eigen::VectorXd> interpolatePath(const std::vector<Eigen::VectorXd>& path, int num_intermediates) const;



            std::unordered_set<int> findSamplesNearObstacles(const std::vector<Obstacle>& obstacles, double scale_factor);
            std::pair<std::unordered_set<int>,std::unordered_set<int>> findSamplesNearObstaclesDual(const std::vector<Obstacle>& obstacles, double scale_factor);

            bool updateObstacleSamples(const std::vector<Obstacle>& obstacles);
            std::unordered_set<int> getDescendants(int node_index);



            void handleAddedObstacleSamples(const std::vector<int>& added);
            void handleRemovedObstacleSamples(const std::vector<int>& removed);



            double heuristic(int current_index);



            void clearPlannerState();
            
            void dumpTreeToCSV(const std::string& filename) const;

            void setClock(rclcpp::Clock::SharedPtr clock);


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
            FMTNode* robot_node_ = nullptr;

            std::unordered_set<int> samples_in_obstacles_; // Current samples in obstacles
            std::unordered_set<int> current_; // Current samples in obstacles

            std::unordered_map<int , double> edge_length_;
            int max_length_edge_ind = -1;
            double max_length = -std::numeric_limits<double>::infinity();

            int checks = 0;

            int num_of_samples_;
            Eigen::VectorXd lower_bounds_;
            Eigen::VectorXd upper_bounds_;
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

            rclcpp::Clock::SharedPtr clock_; // Store the clock

            // NEW: Member variable to store the robot's continuous state
            Eigen::VectorXd robot_continuous_state_;

            

};

