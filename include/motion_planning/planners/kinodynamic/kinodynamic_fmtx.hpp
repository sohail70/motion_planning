// Copyright 2025 Soheil E.Nia
#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/fmt_node.hpp"
#include "motion_planning/utils/visualization.hpp"
#include "motion_planning/ds/priority_queue.hpp"
#include <omp.h> 

class KinodynamicFMTX : public Planner {
 public:
            KinodynamicFMTX(std::shared_ptr<StateSpace> statespace , std::shared_ptr<ProblemDefinition> problem_def , std::shared_ptr<ObstacleChecker> obs_checker);
            void setup(const Params& params, std::shared_ptr<Visualization> visualization) override;
            void plan() override;
            std::vector<size_t> getPathIndex() const;
            std::vector<Eigen::VectorXd> getPathPositions() const;
            void setStart(const Eigen::VectorXd& start) override;
            void setGoal(const Eigen::VectorXd& goal) override;

            FMTNode* getRobotNode() const {
                return robot_node_;
            }

            // bool isPathStillValid(const std::vector<Eigen::VectorXd>& path, const Eigen::VectorXd& current_robot_state) const {
            //     if (path.size() < 2) {
            //         return true; // An empty or single-point path is considered valid.
            //     }

            //     const double t_now = clock_->now().seconds();

            //     // Get the robot's total time-to-go from the last element of its state vector.
            //     // FIX: Use .size() - 1 to access the last element for compatibility with older Eigen versions.
            //     const double robot_total_time_to_go = path.front()(path.front().size() - 1);

            //     // Calculate the single, predicted global time of arrival for this entire path.
            //     const double t_arrival = t_now + robot_total_time_to_go;

            //     // Check every segment of the path.
            //     for (size_t i = 0; i < path.size() - 1; ++i) {
            //         const Eigen::VectorXd& segment_start_state = path[i];
            //         const Eigen::VectorXd& segment_end_state = path[i+1];

            //         Trajectory segment_traj;
            //         segment_traj.path_points.push_back(segment_start_state);
            //         segment_traj.path_points.push_back(segment_end_state);
                    
            //         // FIX: Calculate time duration using the last element of each state.
            //         segment_traj.time_duration = segment_start_state(segment_start_state.size() - 1) - segment_end_state(segment_end_state.size() - 1);
                    
            //         if (segment_traj.time_duration <= 1e-6) {
            //             RCLCPP_WARN(rclcpp::get_logger("fmtx_validator"), "Path invalidated by non-positive time duration on segment %zu.", i);
            //             return false;
            //         }

            //         // FIX: Calculate the segment's global start time using the last element.
            //         const double segment_global_start_time = t_arrival - segment_start_state(segment_start_state.size() - 1);

            //         if (!obs_checker_->isTrajectorySafe(segment_traj, segment_global_start_time)) {
            //             RCLCPP_WARN(rclcpp::get_logger("fmtx_validator"), "Path invalidated by predictive check on segment %zu.", i);
            //             return false;
            //         }
            //     }

            //     return true;
            // }
           /*********************************************************************************/
            /* --- FINAL CORRECTED isPathStillValid ---                                      */
            /*********************************************************************************/

            
            bool isPathStillValid(const std::vector<Eigen::VectorXd>& path, const Eigen::VectorXd& current_robot_state) const;


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


            void printCacheStatus() const;



            
            /**
             * @brief Reconstructs the full, fine-grained execution trajectory from the robot to the goal.
             * @return A complete ExecutionTrajectory object containing Time, X, V, and A matrices.
             */
            ExecutionTrajectory getFinalExecutionTrajectory() const;


            /**
             * @brief Re-anchors the planner's search to the robot's current continuous state.
             * Finds the best node in the tree to serve as the new starting point (leaf node)
             * for the backward search, considering both proximity and existing cost. This is
             * the crucial link between the continuous simulation and the discrete graph.
             * @param robot_state The current 3D state (x,y,Time) of the robot.
             */
            void setRobotState(const Eigen::VectorXd& robot_state);



            void near(int node_index);

            void visualizeTree();
            void visualizeHeapAndUnvisited();
            void visualizePath(const std::vector<Eigen::VectorXd>& path_waypoints);
            void visualizeSmoothedPath(const std::vector<Eigen::VectorXd>& shortest_path_);
            std::vector<Eigen::VectorXd> getSmoothedPathPositions(int num_intermediates, int smoothing_window ) const;
            std::vector<Eigen::VectorXd> smoothPath(const std::vector<Eigen::VectorXd>& path, int window_size) const;
            std::vector<Eigen::VectorXd> interpolatePath(const std::vector<Eigen::VectorXd>& path, int num_intermediates) const;



            std::unordered_set<int> findSamplesNearObstacles(const ObstacleVector& obstacles, double scale_factor);
            std::pair<std::unordered_set<int>,std::unordered_set<int>> findSamplesNearObstaclesDual(const ObstacleVector& obstacles, double scale_factor);

            bool updateObstacleSamples(const ObstacleVector& obstacles);
            std::unordered_set<int> getDescendants(int node_index);



            void handleAddedObstacleSamples(const std::vector<int>& added);
            void handleRemovedObstacleSamples(const std::vector<int>& removed);



            double heuristic(int current_index);



            void clearPlannerState();
            
            void dumpTreeToCSV(const std::string& filename) const;

            void setClock(rclcpp::Clock::SharedPtr clock);


            struct ReplanMetrics {
                long long rewire_neighbor_searches = 0;
                int obstacle_checks = 0;
                int orphaned_nodes = 0;
                double path_cost = 0.0;
            };

            const ReplanMetrics& getLastReplanMetrics() const { return last_replan_metrics_; }
            double getRobotTimeToGo() const { return robot_current_time_to_goal_; }


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
            bool neighbor_precache = false;

            bool in_dynamic = false;

            double mu;
            double zetaD;
            double gamma;
            double factor;

            std::unordered_set<int> v_open_set_;

            std::unordered_set<int> dir;
            
            int kd_dim ; 

            bool static_obs_presence;
            ObstacleVector seen_statics_;

            rclcpp::Clock::SharedPtr clock_; // Store the clock

            // NEW: Member variable to store the robot's continuous state
            Eigen::VectorXd robot_continuous_state_;
            double robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();


            FMTNode* robot_anchor_node_ = nullptr;          // ✅ ADD: Stores the best node to connect to.
            Trajectory robot_bridge_trajectory_;    // ✅ ADD: Stores the path from the robot to the anchor.
            

            ReplanMetrics last_replan_metrics_; 

};

