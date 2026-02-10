#include "motion_planning/planners/kinodynamic/kinodynamic_prmstar_dstarlite.hpp"

#define DEBUG_WITH_DIJKSTRA_ 1
#define USE_INVALIDATING_SET_STRATEGY 1
#define USE_GRID_SAMPLING 1

// Constructor
KinodynamicPRMStarDStarLite::KinodynamicPRMStarDStarLite(
    std::shared_ptr<StateSpace> statespace, 
    std::shared_ptr<ProblemDefinition> pdef,
    std::shared_ptr<ObstacleChecker> obs_checker)
    : statespace_(statespace), problem_def_(pdef), obs_checker_(obs_checker), km_(0.0) {
    std::cout << "KinodynamicPRMStarDStarLite Constructor \n";
}

void KinodynamicPRMStarDStarLite::setStart(const Eigen::VectorXd& start) {
    problem_def_->setStart(start);
}

void KinodynamicPRMStarDStarLite::setGoal(const Eigen::VectorXd& goal) {
    problem_def_->setGoal(goal);
}
void KinodynamicPRMStarDStarLite::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    auto start_time = std::chrono::high_resolution_clock::now();
    visualization_ = visualization;
    
    // 1. Load Parameters
    num_samples_ = params.getParam<int>("num_of_samples");
    use_kdtree_ = params.getParam<bool>("use_kdtree");
    kd_dim_ = params.getParam<int>("kd_dim", 2);
    std::string kdtree_type = params.getParam<std::string>("kdtree_type");
    use_knn_ = params.getParam<bool>("use_knn", false);
    factor_ = params.getParam<double>("factor", 1.0);
    neighbor_precache_ = params.getParam<bool>("precache_neighbors", false);
    is_geometric_mode_ = params.getParam<bool>("is_geometric_mode", false);
    bool use_grid_sampling;
#if USE_GRID_SAMPLING
    use_grid_sampling = true;
#endif
    lower_bounds_ = problem_def_->getLowerBound();
    upper_bounds_ = problem_def_->getUpperBound();

    // 2. Initialize KD-Tree
    if (use_kdtree_ && kdtree_type == "NanoFlann") {
        Eigen::VectorXd weights(kd_dim_);
        switch (kd_dim_) {
            case 2: weights << 1.0, 1.0; break;
            case 3: weights << 1.0, 1.0, 1.0; break;
            case 4: weights << 1.0, 1.0, 1.0, 1.0; break;
            case 5: weights << 1.0, 1.0, 1.0, 1.0, 1.0; break;
            default: throw std::runtime_error("Unsupported k-d tree dimension");
        }
        kdtree_ = std::make_shared<WeightedNanoFlann>(kd_dim_, weights);
    } else if (use_kdtree_ && kdtree_type == "LieKDTree") {
        kdtree_ = std::make_unique<LieSplittingKDTree>(statespace_->getDimension(), statespace_);
    } else {
        throw std::runtime_error("PRM* D* Lite requires a KD-Tree.");
    }

    // 3. Build Graph (Sample + Identify Start/Goal)
    // We do this BEFORE building the KD-Tree so the KD-Tree includes Start/Goal.
    const Eigen::VectorXd& start_state_val = problem_def_->getGoal(); // I accidentally used the getGoal and setGoal in reverse in my main loop so dont get confused this is correct!
    const Eigen::VectorXd& goal_state_val = problem_def_->getStart();


    // // Add Goal Node --> root
    // auto goal_state_ptr = statespace_->addState(goal_state_val);
    // auto goal_node = std::make_unique<DStarLiteNode>(goal_state_ptr, 0);
    // goal_node_ = goal_node.get(); // Set pointer immediately
    // goal_node->setTimeToGoal(0);
    // nodes_.push_back(std::move(goal_node));

    // // Add Start Node --> robot 
    // auto start_state_ptr = statespace_->addState(start_state_val);
    // auto start_node = std::make_unique<DStarLiteNode>(start_state_ptr, 1);
    // start_node_ = start_node.get(); // Set pointer immediately
    // start_node->setTimeToGoal(0);
    // nodes_.push_back(std::move(start_node));


    // 3. Build Graph (Sample + Identify Start/Goal)
    // NOTE: We do NOT manually add Start/Goal here anymore.
    // We will generate the grid first, then pick corners.

    // --- GRID SAMPLING LOGIC ---
    if (use_grid_sampling) {
        std::cout << "Using GRID sampling strategy." << std::endl;
        
        // 1. Calculate grid steps
        int samples_per_dim = static_cast<int>(std::pow(num_samples_, 1.0 / kd_dim_));
        if (samples_per_dim < 2) samples_per_dim = 2;

        // --- SAVE THIS FOR LATER USE IN near() ---
        grid_dim_per_side_ = samples_per_dim;
        use_grid_sampling_ = true;
        // -----------------------------------------

        std::cout << "Grid dimension: " << samples_per_dim << "x" << samples_per_dim << std::endl;

        // Calculate step size for each dimension
        Eigen::VectorXd step_size = (upper_bounds_ - lower_bounds_) / (samples_per_dim - 1);

        std::vector<int> indices(kd_dim_, 0);
        int count = 0;
        
        // Total points in grid = samples_per_dim^kd_dim_
        int total_grid_points = static_cast<int>(std::pow(samples_per_dim, kd_dim_));
        
        for (int i = 0; i < total_grid_points; ++i) {
            Eigen::VectorXd state_val(kd_dim_);
            
            // Construct state based on current indices
            for (int d = 0; d < kd_dim_; ++d) {
                state_val(d) = lower_bounds_(d) + indices[d] * step_size(d);
            }

            // Add state to statespace
            auto state_ptr = statespace_->addState(state_val);
            auto node = std::make_unique<DStarLiteNode>(state_ptr, nodes_.size());
            
            if (!is_geometric_mode_) {
                double absolute_t = node->getStateValue().tail<1>()[0];
                node->setTimeToGoal(absolute_t);
            } else {
                node->setTimeToGoal(0.0);
            }
            nodes_.push_back(std::move(node));

            // Increment indices (like a digital clock)
            for (int d = 0; d < kd_dim_; ++d) {
                indices[d]++;
                if (indices[d] < samples_per_dim) {
                    break; // Carry over successful
                } else {
                    indices[d] = 0; // Reset and carry to next dimension
                }
            }
        }

        // --- ASSIGN START AND GOAL FROM GRID CORNERS ---
        // Assuming 2D grid for simplicity of corner logic:
        // Bottom-Left (Min X, Min Y) -> Goal (Index 0)
        // Top-Right (Max X, Max Y) -> Start (Last Index)
        
        // Goal is at (0,0) -> Index 0
        goal_node_ = nodes_[0].get();
        goal_node_->setTimeToGoal(0);
        std::cout << "Grid Goal Node ID: " << goal_node_->getIndex() 
                  << " at " << goal_node_->getStateValue().transpose() << std::endl;

        // Start is at (N-1, N-1) -> Index (Total - 1)
        start_node_ = nodes_[nodes_.size() - 1].get();
        start_node_->setTimeToGoal(0);
        std::cout << "Grid Start Node ID: " << start_node_->getIndex() 
                  << " at " << start_node_->getStateValue().transpose() << std::endl;

    } else {
        // --- ORIGINAL UNIFORM SAMPLING ---
        std::cout << "Using UNIFORM random sampling strategy." << std::endl;
        use_grid_sampling_ = false;
        
        // In uniform mode, we still need to add Start/Goal manually as before
        const Eigen::VectorXd& start_state_val = problem_def_->getGoal(); 
        const Eigen::VectorXd& goal_state_val = problem_def_->getStart();

        auto goal_state_ptr = statespace_->addState(goal_state_val);
        auto goal_node = std::make_unique<DStarLiteNode>(goal_state_ptr, 0);
        goal_node_ = goal_node.get(); 
        goal_node->setTimeToGoal(0);
        nodes_.push_back(std::move(goal_node));

        auto start_state_ptr = statespace_->addState(start_state_val);
        auto start_node = std::make_unique<DStarLiteNode>(start_state_ptr, 1);
        start_node_ = start_node.get(); 
        start_node->setTimeToGoal(0);
        nodes_.push_back(std::move(start_node));

        for (int i = 0; i < num_samples_; ++i) {
            auto state_ptr = statespace_->sampleUniform(lower_bounds_, upper_bounds_);
            auto node = std::make_unique<DStarLiteNode>(state_ptr, nodes_.size());
            if (!is_geometric_mode_) {
                double absolute_t = node->getStateValue().tail<1>()[0];
                node->setTimeToGoal(absolute_t);
            } else {
                node->setTimeToGoal(0.0);
            }
            nodes_.push_back(std::move(node));
        }
    }


    
    // // Add Random Samples
    // for (int i = 0; i < num_samples_; ++i) {
    //     auto state_ptr = statespace_->sampleUniform(lower_bounds_, upper_bounds_);
    //     auto node = std::make_unique<DStarLiteNode>(state_ptr, nodes_.size());
    //     // --- SET TIME TO GOAL HERE (Initialization) ---
    //     if (!is_geometric_mode_) {
    //         // In kinodynamic mode, the time to goal is the absolute time 
    //         // stored in the last dimension of the state vector.
    //         double absolute_t = node->getStateValue().tail<1>()[0];
    //         node->setTimeToGoal(absolute_t);
    //     } else {
    //         // In geometric mode, time is irrelevant.
    //         node->setTimeToGoal(0.0);
    //     }
    //     nodes_.push_back(std::move(node));
    // }

    std::cout << "PRM Built with " << nodes_.size() << " nodes (including Start/Goal).\n";

    // 4. Build KD-Tree
    if (use_kdtree_) {
        Eigen::MatrixXd all_samples = statespace_->getSamplesCopy();
        Eigen::MatrixXd spatial_samples_only = all_samples.leftCols(kd_dim_).eval();
        kdtree_->addPoints(spatial_samples_only);
        kdtree_->buildTree();
    }

    // 5. Calculate Radius
    int d = statespace_->getDimension();
    if (use_knn_) {
        double k0_fmt_star_practical = std::pow(2.0, d) * (M_E / d);
        k_neighbors_ = static_cast<int>(std::ceil(factor_ * k0_fmt_star_practical * std::log(statespace_->getNumStates())));
    } else {
        Eigen::VectorXd range = upper_bounds_ - lower_bounds_;
        mu_ = range.prod();
        zetaD_ = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
        gamma_ = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu_ / zetaD_, 1.0 / d);
        connection_radius_ = factor_ * gamma_ * std::pow(std::log(statespace_->getNumStates()) / statespace_->getNumStates(), 1.0 / d);
    }

    // 6. Neighbor Pre-caching
    if (neighbor_precache_) {
        std::cout << "Forcing neighbor caching for all " << nodes_.size() << " nodes..." << std::endl;
        auto cache_start = std::chrono::high_resolution_clock::now();
        for (size_t i = 0; i < nodes_.size(); ++i) {
            near(i);
        }
        auto cache_end = std::chrono::high_resolution_clock::now();
        auto cache_duration = std::chrono::duration_cast<std::chrono::milliseconds>(cache_end - cache_start);
        std::cout << "Neighbor caching complete. Time taken: " << cache_duration.count() << " ms." << std::endl;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";


    std::cout << "\n=== D* Lite Connectivity Check ===" << std::endl;
    
    // Check Start Node
    std::cout << "Start Node ID: " << start_node_->getIndex() << std::endl;
    std::cout << "Start Backward Neighbors (Incoming): " << start_node_->backward_neighbors_.size() << std::endl;
    std::cout << "Start Forward Neighbors (Outgoing): " << start_node_->forward_neighbors_.size() << std::endl;
    
    // Check Goal Node
    std::cout << "Goal Node ID: " << goal_node_->getIndex() << std::endl;
    std::cout << "Goal Backward Neighbors (Incoming): " << goal_node_->backward_neighbors_.size() << std::endl;
    std::cout << "Goal Forward Neighbors (Outgoing): " << goal_node_->forward_neighbors_.size() << std::endl;
    
    std::cout << "====================================\n" << std::endl;


}

// // --- NEW: Neighbor Caching Logic ---
// void KinodynamicPRMStarDStarLite::near(int node_index) {
//     auto node = nodes_[node_index].get();
    
//     if (node->neighbors_cached_) return;
//     // 1. Get candidate neighbors
//     std::vector<size_t> candidate_indices;
//     if (use_knn_) {
//         if (k_neighbors_ > 0) {
//             candidate_indices = kdtree_->knnSearch(node->getStateValue().head(kd_dim_), k_neighbors_);
//         }
//     } else {
//         if (connection_radius_ > 0) {
//             candidate_indices = kdtree_->radiusSearch(node->getStateValue().head(kd_dim_), connection_radius_);
//         }
//     }
//     // 2. Populate Neighbors (Proactive Strategy)
//     // We assume statespace_->prefersLazyNear() is FALSE for your setup, 
//     // meaning we compute trajectories now.
    
//     for (int idx : candidate_indices) {
//         if (idx == node_index) continue;
//         DStarLiteNode* neighbor = nodes_[idx].get();
        
//         // --- Test FORWARD connection (Node -> Neighbor) ---
//         Trajectory traj_forward = statespace_->steer(node->getStateValue(), neighbor->getStateValue());
        
//         // We only care if the trajectory is valid (steering constraints).
//         // We do NOT check obstacle collisions here.
//         if (traj_forward.is_valid) {
//             EdgeInfo info_forward;
//             info_forward.distance = traj_forward.cost; 
//             info_forward.distance_original = info_forward.distance;
//             info_forward.cached_trajectory = traj_forward;
//             info_forward.is_trajectory_computed = true;
//             // No invalidating_obstacles insertion here!
//             // Store: Node -> Neighbor
//             node->forward_neighbors_[neighbor] = info_forward;
//             // Store Reverse: Neighbor -> Node (This is the backward edge for Neighbor)
//             neighbor->backward_neighbors_[node] = info_forward;
//         }
        
//         // --- Test BACKWARD connection (Neighbor -> Node) ---
//         Trajectory traj_backward;
        
//         // OPTIMIZATION: If geometric mode is on, steer(A,B) == steer(B,A).
//         // We can reuse the forward trajectory to save computation time.
//         if (is_geometric_mode_) {
//             traj_backward = traj_forward;
//         } else {
//             // CRITICAL: In kinodynamic mode, this is a different trajectory!
//             traj_backward = statespace_->steer(neighbor->getStateValue(), node->getStateValue());
//         }
        
//         if (traj_backward.is_valid) {
//             EdgeInfo info_backward;
//             info_backward.distance = traj_backward.cost;
//             info_backward.distance_original = info_backward.distance;
//             info_backward.cached_trajectory = traj_backward;
//             info_backward.is_trajectory_computed = true;
//             // Store: Neighbor -> Node
//             neighbor->forward_neighbors_[node] = info_backward;
//             // Store Reverse: Node -> Neighbor (This is the backward edge for Node)
//             node->backward_neighbors_[neighbor] = info_backward;
//         }
//     }
//     node->neighbors_cached_ = true;
// }

void KinodynamicPRMStarDStarLite::near(int node_index) {
    auto node = nodes_[node_index].get();
    if (node->neighbors_cached_) return;

    // --- GRID NEIGHBOR LOGIC ---
    if (use_grid_sampling_) {
        // 1. Convert linear index to 2D grid coordinates (row, col)
        // Assuming row-major order: index = row * width + col
        int row = node_index / grid_dim_per_side_;
        int col = node_index % grid_dim_per_side_;

        // 2. Define the 8 directions (dx, dy)
        // (Up, Down, Left, Right, and 4 Diagonals)
        int directions[8][2] = {
            {-1, 0}, {1, 0}, {0, -1}, {0, 1},  // Cardinals
            {-1, -1}, {-1, 1}, {1, -1}, {1, 1} // Diagonals
        };

        // 3. Iterate through directions
        for (int i = 0; i < 8; ++i) {
            int n_row = row + directions[i][0];
            int n_col = col + directions[i][1];

            // 4. Check boundaries (don't connect to nodes outside the grid)
            if (n_row >= 0 && n_row < grid_dim_per_side_ &&
                n_col >= 0 && n_col < grid_dim_per_side_) {
                
                // 5. Convert back to linear index
                int neighbor_idx = n_row * grid_dim_per_side_ + n_col;

                // Safety check
                if (neighbor_idx >= 0 && neighbor_idx < (int)nodes_.size()) {
                    DStarLiteNode* neighbor = nodes_[neighbor_idx].get();
                    
                    // --- CONNECT NODES (Same logic as before) ---
                    
                    // Test FORWARD connection (Node -> Neighbor)
                    Trajectory traj_forward = statespace_->steer(node->getStateValue(), neighbor->getStateValue());
                    if (traj_forward.is_valid) {
                        EdgeInfo info_forward;
                        info_forward.distance = traj_forward.cost;
                        info_forward.distance_original = info_forward.distance;
                        info_forward.cached_trajectory = traj_forward;
                        info_forward.is_trajectory_computed = true;
                        
                        node->forward_neighbors_[neighbor] = info_forward;
                        neighbor->backward_neighbors_[node] = info_forward;
                    }

                    // Test BACKWARD connection (Neighbor -> Node)
                    Trajectory traj_backward;
                    if (is_geometric_mode_) {
                        traj_backward = traj_forward;
                    } else {
                        traj_backward = statespace_->steer(neighbor->getStateValue(), node->getStateValue());
                    }
                    
                    if (traj_backward.is_valid) {
                        EdgeInfo info_backward;
                        info_backward.distance = traj_backward.cost;
                        info_backward.distance_original = info_backward.distance;
                        info_backward.cached_trajectory = traj_backward;
                        info_backward.is_trajectory_computed = true;
                        
                        neighbor->forward_neighbors_[node] = info_backward;
                        node->backward_neighbors_[neighbor] = info_backward;
                    }
                }
            }
        }
    } 
    // --- ORIGINAL KD-TREE LOGIC ---
    else {
        // 1. Get candidate neighbors
        std::vector<size_t> candidate_indices;
        if (use_knn_) {
            if (k_neighbors_ > 0) {
                candidate_indices = kdtree_->knnSearch(node->getStateValue().head(kd_dim_), k_neighbors_);
            }
        } else {
            if (connection_radius_ > 0) {
                candidate_indices = kdtree_->radiusSearch(node->getStateValue().head(kd_dim_), connection_radius_);
            }
        }

        // 2. Populate Neighbors
        for (int idx : candidate_indices) {
            if (idx == node_index) continue;
            DStarLiteNode* neighbor = nodes_[idx].get();

            // Test FORWARD connection
            Trajectory traj_forward = statespace_->steer(node->getStateValue(), neighbor->getStateValue());
            if (traj_forward.is_valid) {
                EdgeInfo info_forward;
                info_forward.distance = traj_forward.cost;
                info_forward.distance_original = info_forward.distance;
                info_forward.cached_trajectory = traj_forward;
                info_forward.is_trajectory_computed = true;
                
                node->forward_neighbors_[neighbor] = info_forward;
                neighbor->backward_neighbors_[node] = info_forward;
            }

            // Test BACKWARD connection
            Trajectory traj_backward;
            if (is_geometric_mode_) {
                traj_backward = traj_forward;
            } else {
                traj_backward = statespace_->steer(neighbor->getStateValue(), node->getStateValue());
            }
            if (traj_backward.is_valid) {
                EdgeInfo info_backward;
                info_backward.distance = traj_backward.cost;
                info_backward.distance_original = info_backward.distance;
                info_backward.cached_trajectory = traj_backward;
                info_backward.is_trajectory_computed = true;
                
                neighbor->forward_neighbors_[node] = info_backward;
                node->backward_neighbors_[neighbor] = info_backward;
            }
        }
    }

    node->neighbors_cached_ = true;
}



void KinodynamicPRMStarDStarLite::plan() {
    if (!start_node_ || !goal_node_) return;
    initialize(start_node_, goal_node_);
    computeShortestPath();
    #if DEBUG_WITH_DIJKSTRA_
        computeShortestPathDijkstraMode(); 
        debugCompareDijkstraVsDStarLite();
    #endif
    
}

double KinodynamicPRMStarDStarLite::heuristic(DStarLiteNode* a, DStarLiteNode* b) {
    // return (a->getStateValue() - b->getStateValue()).norm();
    return 0; // FOR DEBUG WITH DIJKSTRA MODE!
}

DStarLiteKey KinodynamicPRMStarDStarLite::calculateKey(DStarLiteNode* u) {
    double min_val = std::min(u->g, u->rhs);
    double h_val = (start_node_) ? heuristic(u, start_node_) : 0.0;
    
    return { min_val + h_val + km_, min_val };
}


void KinodynamicPRMStarDStarLite::initialize(DStarLiteNode* start, DStarLiteNode* goal) {
    km_ = 0.0;
    
    // No need to loop through nodes to set g/rhs to infinity because 
    // the DStarLiteNode constructor does that, and we assume the graph 
    // hasn't changed its structure (nodes added/removed) since setup.
    // We only need to ensure the queue is clean.
    
    open_queue_.clear();
    
    // However, if you are re-planning from scratch (e.g. after a reset), 
    // you MUST reset g/rhs. If this is called only once at startup, 
    // the constructor handles it. 
    // If you call this to reset the search, uncomment the loop:
    /*
    for (auto& node : nodes_) {
        node->g = std::numeric_limits<double>::infinity();
        node->rhs = std::numeric_limits<double>::infinity();
        node->heap_index_ = -1;
        node->in_queue_ = false;
    }
    */

    start_node_ = start;
    goal_node_ = goal;
    
    goal->rhs = 0.0;
    
    open_queue_.add(goal, calculateKey(goal));
}

// void KinodynamicPRMStarDStarLite::updateVertex(DStarLiteNode* u) {
//     // Case 1: Goal Node (The Root)
//     if (u == goal_node_) {
//         u->rhs = 0.0;
//         u->best_parent_ = nullptr;
//         u->best_parent_trajectory_ = Trajectory();
//     } 
//     // Case 2: Ordinary Node
//     else {
//         double min_rhs = std::numeric_limits<double>::infinity();
        
//         // --- PASS 1: Find the absolute minimum RHS value ---
//         for (auto& [succ, edge_info] : u->forward_neighbors_) {
//             if (!edge_info.invalidating_obstacles.empty()) continue;
//             if (succ->g == std::numeric_limits<double>::infinity()) continue;
            
//             double cost = edge_info.distance + succ->g;
//             if (cost < min_rhs) {
//                 min_rhs = cost;
//             }
//         }
        
//         // --- PASS 2: Find the Best Parent (With Deterministic Tie-Breaking) ---
//         DStarLiteNode* best_parent = nullptr;
//         Trajectory best_traj;

//         if (std::isfinite(min_rhs)) {
//             std::vector<DStarLiteNode*> candidates;
//             candidates.reserve(4); // Optimization: avoid allocations for small numbers
            
//             for (auto& [succ, edge_info] : u->forward_neighbors_) {
//                 if (!edge_info.invalidating_obstacles.empty()) continue;
//                 if (succ->g == std::numeric_limits<double>::infinity()) continue;

//                 double cost = edge_info.distance + succ->g;
                
//                 // Collect ANY parent that gives us the optimal cost (within epsilon)
//                 if (std::abs(cost - min_rhs) < 1e-9) {
//                     candidates.push_back(succ);
//                 }
//             }

//             // CRITICAL SORT: Ensure we always pick the parent with the lowest Index
//             // This matches the "Strict" Dijkstra logic.
//             if (!candidates.empty()) {
//                 std::sort(candidates.begin(), candidates.end(), 
//                     [](DStarLiteNode* a, DStarLiteNode* b) {
//                         return a->getIndex() < b->getIndex(); 
//                     });

//                 // The winner is the first one after sorting
//                 best_parent = candidates[0];
//                 best_traj = u->forward_neighbors_[best_parent].cached_trajectory;
//             }
//         }

//         // --- UPDATE STATE ---
//         u->rhs = min_rhs;
//         u->best_parent_ = best_parent;
//         u->best_parent_trajectory_ = best_traj;
//     }

//     // --- UPDATE PRIORITY QUEUE ---
//     if (u->in_queue_) open_queue_.remove(u);
    
//     if (u->g != u->rhs) {
//         open_queue_.add(u, calculateKey(u));
//     }
// }

void KinodynamicPRMStarDStarLite::updateVertex(DStarLiteNode* u) {
    if (u == goal_node_) {
        u->rhs = 0.0;
        u->best_parent_ = nullptr;
        u->best_parent_trajectory_ = Trajectory();
    } 
    else {
        double min_rhs = std::numeric_limits<double>::infinity();
        DStarLiteNode* best_parent = nullptr;
        Trajectory best_traj;

#if DEBUG_WITH_DIJKSTRA_
        // Debug Mode: Sorting for Determinism
        for (auto& [succ, edge_info] : u->forward_neighbors_) {
            if (edge_info.distance == std::numeric_limits<double>::infinity()) continue;
            if (succ->g == std::numeric_limits<double>::infinity()) continue;
            double cost = edge_info.distance + succ->g;
            if (cost < min_rhs) min_rhs = cost;
        }
        if (std::isfinite(min_rhs)) {
            std::vector<DStarLiteNode*> candidates;
            candidates.reserve(4);
            for (auto& [succ, edge_info] : u->forward_neighbors_) {
                if (edge_info.distance == std::numeric_limits<double>::infinity()) continue;
                if (succ->g == std::numeric_limits<double>::infinity()) continue;
                double cost = edge_info.distance + succ->g;
                if (std::abs(cost - min_rhs) < 1e-9) candidates.push_back(succ);
            }
            if (!candidates.empty()) {
                std::sort(candidates.begin(), candidates.end(), [](DStarLiteNode* a, DStarLiteNode* b) { return a->getIndex() < b->getIndex(); });
                best_parent = candidates[0];
                best_traj = u->forward_neighbors_[best_parent].cached_trajectory;
            }
        }
#else
        // Production Mode: Fast One-Pass
        for (auto& [succ, edge_info] : u->forward_neighbors_) {
            // BOTH STRATEGIES: Trust the distance value
            if (edge_info.distance == std::numeric_limits<double>::infinity()) continue;
            if (succ->g == std::numeric_limits<double>::infinity()) continue;

            double cost = edge_info.distance + succ->g;
            if (cost < min_rhs) {
                min_rhs = cost;
                best_parent = succ;
                best_traj = edge_info.cached_trajectory;
            }
        }
#endif

        u->rhs = min_rhs;
        u->best_parent_ = best_parent;
        u->best_parent_trajectory_ = best_traj;
    }

    if (u->in_queue_) open_queue_.remove(u);
    if (u->g != u->rhs) open_queue_.add(u, calculateKey(u));
}


void KinodynamicPRMStarDStarLite::printNodeDetails(const std::string& prefix, DStarLiteNode* node) {
    if (!node) {
        std::cout << prefix << " (NULL)" << std::endl;
        return;
    }
    Eigen::VectorXd val = node->getStateValue();
    // Assuming 2D or 3D, printing first two components for visual verification
    std::cout << prefix 
              << " [ID: " << node->getIndex() 
              << "] Pos: (" << val(0) << ", " << val(1) << ")" 
              << " g: " << (node->g == std::numeric_limits<double>::infinity() ? -1.0 : node->g)
              << " rhs: " << (node->rhs == std::numeric_limits<double>::infinity() ? -1.0 : node->rhs)
              << std::endl;
}

void KinodynamicPRMStarDStarLite::computeShortestPath() {
    if (!start_node_ || !goal_node_) return;

    // Paper condition: while (U.TopKey() < CalculateKey(s_start) OR rhs(s_start) != g(s_start))
    // while (open_queue_.topKey() < calculateKey(start_node_) || start_node_->rhs != start_node_->g) {
    while (!open_queue_.empty()) { //FOR DEBUG
        
        DStarLiteKey k_old = open_queue_.topKey();
        DStarLiteNode* u = open_queue_.pop();
        if (!u) break;

        DStarLiteKey k_new = calculateKey(u);

        if (k_old < k_new) {
            // Lazy update: the key in the heap was stale, re-insert with fresh k_m
            open_queue_.add(u, k_new);
        } 
        else if (u->g > u->rhs) {
            // Overconsistent: cost decreased
            u->g = u->rhs;
            // Propagate to predecessors (backward neighbors)
            for (auto& [pred, edge_info] : u->backward_neighbors_) {
                updateVertex(pred);
            }
        } 
        else {
            // Underconsistent: cost increased
            u->g = std::numeric_limits<double>::infinity();
            // Update u itself and all predecessors
            updateVertex(u);
            for (auto& [pred, edge_info] : u->backward_neighbors_) {
                updateVertex(pred);
            }
        }
    }
}
// double KinodynamicPRMStarDStarLite::computeShortestPathDijkstraMode() {
//     if (!start_node_ || !goal_node_) return -1.0;

//     using NodePair = std::pair<double, DStarLiteNode*>;
//     auto cmp = [](const NodePair& a, const NodePair& b){ return a.first > b.first; };
//     std::priority_queue<NodePair, std::vector<NodePair>, decltype(cmp)> pq(cmp);

//     std::unordered_map<DStarLiteNode*, double> local_g_map;
//     std::unordered_map<DStarLiteNode*, DStarLiteNode*> local_parent_map;

//     const double EPS = 1e-9;

//     // init from goal (cost-to-go of goal == 0)
//     pq.push({0.0, goal_node_});
//     local_g_map[goal_node_] = 0.0;
//     local_parent_map[goal_node_] = nullptr;

//     while (!pq.empty()) {
//         auto [cur_cost, u] = pq.top(); pq.pop();

//         // stale entry check
//         auto it = local_g_map.find(u);
//         if (it != local_g_map.end() && it->second + EPS < cur_cost) {
//             continue;
//         }

//         // For correctness on directed graphs we must iterate predecessors:

//         // For correctness on directed graphs we must iterate predecessors:
//         for (auto& [pred, edge_info] : u->backward_neighbors_) {
//             if (!edge_info.is_trajectory_computed) continue;
//             if (edge_info.distance == std::numeric_limits<double>::infinity()) continue;

//             double edge_start_time = pred->getTimeToGoal();
//             if (!obs_checker_->isTrajectorySafe(edge_info.cached_trajectory, edge_start_time)) continue;

//             double new_cost = cur_cost + edge_info.distance;
            
//             auto itpred = local_g_map.find(pred);
//             bool should_update = false;

//             if (itpred == local_g_map.end()) {
//                 should_update = true; // First time seeing node
//             } 
//             else if (new_cost < itpred->second - 1e-9) {
//                 should_update = true; // Strictly cheaper
//             }
//             else if (std::abs(new_cost - itpred->second) < 1e-9) {
//                 // TIE-BREAKER: Prefer Lower Index (Matches your D* Lite sort)
//                 if (local_parent_map.find(pred) != local_parent_map.end()) {
//                     DStarLiteNode* current_parent = local_parent_map[pred];
//                     if (current_parent && u->getIndex() < current_parent->getIndex()) {
//                         should_update = true;
//                     }
//                 }
//             }

//             if (should_update) {
//                 local_g_map[pred] = new_cost;
//                 local_parent_map[pred] = u; 
//                 pq.push({new_cost, pred});
//             }
//         }
//     }

//     double dijkstra_cost = -1.0;
//     auto it_start = local_g_map.find(start_node_);
//     if (it_start != local_g_map.end()) {
//         dijkstra_cost = it_start->second;
//         std::cout << "[Dijkstra Mode] Exhaustive search complete. Start cost: " << dijkstra_cost << std::endl;

//         double main_planner_cost = start_node_->g;
//         if (std::isfinite(main_planner_cost) && std::abs(dijkstra_cost - main_planner_cost) < 1e-6) {
//             std::cout << "\033[1;32m[SUCCESS] Dijkstra Cost matches D* Lite Cost!\033[0m" << std::endl;
//         } else {
//             std::cout << "\033[1;31m[FAILURE] Cost Mismatch! Dijkstra: " << dijkstra_cost 
//                       << " vs D* Lite: " << main_planner_cost << "\033[0m" << std::endl;
//         }
//     } else {
//         std::cout << "[Dijkstra Mode] Start node is unreachable in current world." << std::endl;
//     }

//     dijkstra_tree_parents_ = local_parent_map;
//     return dijkstra_cost;
// }


double KinodynamicPRMStarDStarLite::computeShortestPathDijkstraMode() {
    if (!start_node_ || !goal_node_) return -1.0;

    using NodePair = std::pair<double, DStarLiteNode*>;
    auto cmp = [](const NodePair& a, const NodePair& b){ return a.first > b.first; };
    std::priority_queue<NodePair, std::vector<NodePair>, decltype(cmp)> pq(cmp);

    std::unordered_map<DStarLiteNode*, double> local_g_map;
    std::unordered_map<DStarLiteNode*, DStarLiteNode*> local_parent_map;

    const double EPS = 1e-9;

    pq.push({0.0, goal_node_});
    local_g_map[goal_node_] = 0.0;
    local_parent_map[goal_node_] = nullptr;

    while (!pq.empty()) {
        auto [cur_cost, u] = pq.top(); pq.pop();

        auto it = local_g_map.find(u);
        if (it != local_g_map.end() && it->second + EPS < cur_cost) continue;

        // Iterate Predecessors (Backward Neighbors)
        for (auto& [pred, edge_info] : u->backward_neighbors_) {
            
            // 1. PURE COST CHECK: If it's blocked, it's INF. Dijkstra skips it.
            if (edge_info.distance == std::numeric_limits<double>::infinity()) continue;

            // Note: We REMOVED the 'obs_checker_->isTrajectorySafe' check here.
            // Why? Because we want to verify what the graph *thinks* is true.
            // If the graph thinks an unsafe edge is valid, we WANT Dijkstra to find it 
            // so we see the mismatch error (proving addNewObstacle failed).

            double new_cost = cur_cost + edge_info.distance;
            
            auto itpred = local_g_map.find(pred);
            bool should_update = false;

            if (itpred == local_g_map.end()) {
                should_update = true;
            } 
            else if (new_cost < itpred->second - EPS) {
                should_update = true;
            }
            else if (std::abs(new_cost - itpred->second) < EPS) {
                // Tie-breaker matching D* Lite
                if (local_parent_map.find(pred) != local_parent_map.end()) {
                    DStarLiteNode* current_parent = local_parent_map[pred];
                    if (current_parent && u->getIndex() < current_parent->getIndex()) {
                        should_update = true;
                    }
                }
            }

            if (should_update) {
                local_g_map[pred] = new_cost;
                local_parent_map[pred] = u;
                pq.push({new_cost, pred});
            }
        }
    }

    double dijkstra_cost = -1.0;
    auto it_start = local_g_map.find(start_node_);
    if (it_start != local_g_map.end()) {
        dijkstra_cost = it_start->second;
        std::cout << "[Dijkstra Mode] Exhaustive search complete. Start cost: " << dijkstra_cost << std::endl;

        double main_planner_cost = start_node_->g;
        // Use a looser tolerance for floats
        if (std::isfinite(main_planner_cost) && std::abs(dijkstra_cost - main_planner_cost) < 1e-4) {
            std::cout << "\033[1;32m[SUCCESS] Dijkstra Cost matches D* Lite Cost!\033[0m" << std::endl;
        } else {
            std::cout << "\033[1;31m[FAILURE] Cost Mismatch! Dijkstra: " << dijkstra_cost 
                      << " vs D* Lite: " << main_planner_cost << "\033[0m" << std::endl;
        }
    } else {
        std::cout << "[Dijkstra Mode] Start node is unreachable in current world." << std::endl;
    }

    dijkstra_tree_parents_ = local_parent_map;
    return dijkstra_cost;
}

void KinodynamicPRMStarDStarLite::debugCompareDijkstraVsDStarLite() {
    std::cout << "\n========== DEBUG: COST COMPARISON ==========" << std::endl;
    std::cout << std::left << std::setw(10) << "NodeID" 
              << std::setw(15) << "Dijkstra g" 
              << std::setw(15) << "D* Lite g" 
              << std::setw(15) << "Diff" 
              << std::setw(10) << "Status" 
              << std::setw(20) << "Dijkstra Parent" 
              << std::setw(20) << "D* Lite Parent" << std::endl;
    std::cout << "--------------------------------------------------------------------------------" << std::endl;

    int mismatch_count = 0;
    double max_diff = 0.0;

    // 1. Run Dijkstra to get ground truth
    std::unordered_map<DStarLiteNode*, double> dijkstra_costs;
    std::unordered_map<DStarLiteNode*, DStarLiteNode*> dijkstra_parents;
    
    using NodePair = std::pair<double, DStarLiteNode*>;
    auto cmp = [](const NodePair& a, const NodePair& b){ return a.first > b.first; };
    std::priority_queue<NodePair, std::vector<NodePair>, decltype(cmp)> pq(cmp);
    
    pq.push({0.0, goal_node_});
    dijkstra_costs[goal_node_] = 0.0;
    dijkstra_parents[goal_node_] = nullptr;

    while (!pq.empty()) {
        auto [cur_cost, u] = pq.top(); pq.pop();
        if (dijkstra_costs[u] + 1e-9 < cur_cost) continue;
        
        for (auto& [pred, edge_info] : u->backward_neighbors_) {
            if (!edge_info.is_trajectory_computed) continue;
            if (edge_info.distance == std::numeric_limits<double>::infinity()) continue;
            
            double new_cost = cur_cost + edge_info.distance;
            
            // Standard Dijkstra update
            auto it = dijkstra_costs.find(pred);
            bool update = false;
            if (it == dijkstra_costs.end()) update = true;
            else if (new_cost < it->second - 1e-9) update = true;
            else if (std::abs(new_cost - it->second) < 1e-9) {
                // Tie-break by Index to match D* Lite
                if (dijkstra_parents[pred] && u->getIndex() < dijkstra_parents[pred]->getIndex()) update = true;
            }

            if (update) {
                dijkstra_costs[pred] = new_cost;
                dijkstra_parents[pred] = u;
                pq.push({new_cost, pred});
            }
        }
    }

    // 2. Compare and Log Detailed Errors
    for (const auto& node_ptr : nodes_) {
        DStarLiteNode* n = node_ptr.get();
        double d_g = (dijkstra_costs.find(n) != dijkstra_costs.end()) ? dijkstra_costs[n] : std::numeric_limits<double>::infinity();
        double ds_g = n->g;
        
        double diff = std::abs(d_g - ds_g);
        if (diff > max_diff) max_diff = diff;

        std::string status = "OK";
        bool is_mismatch = false;

        if (std::isfinite(d_g) && std::isfinite(ds_g)) {
            if (diff > 1e-5) { status = "MISMATCH"; is_mismatch = true; }
        } else if (std::isfinite(d_g) != std::isfinite(ds_g)) {
            status = "ONE_INF"; is_mismatch = true;
        }

        if (is_mismatch) {
            mismatch_count++;
            
            auto get_parent_str = [](DStarLiteNode* p) { return p ? std::to_string(p->getIndex()) : "None"; };

            std::cout << std::left << std::setw(10) << n->getIndex() 
                      << std::setw(15) << (std::isfinite(d_g) ? std::to_string(d_g).substr(0, 6) : "INF")
                      << std::setw(15) << (std::isfinite(ds_g) ? std::to_string(ds_g).substr(0, 6) : "INF")
                      << std::setw(15) << diff
                      << std::setw(10) << status 
                      << std::setw(20) << get_parent_str(dijkstra_parents[n])
                      << std::setw(20) << get_parent_str(n->best_parent_) << std::endl;

            // --- DEEP INSPECTION LOG ---
            DStarLiteNode* p_dijkstra = dijkstra_parents[n];
            if (p_dijkstra) {
                std::cout << "   >>> INSPECTING EDGE: Node " << n->getIndex() << " -> Parent " << p_dijkstra->getIndex() << "\n";
                
                // Check FORWARD (D* Lite uses this)
                double fwd_dist = -1.0;
                bool fwd_exists = false;
                std::string fwd_blockers = "None";
                
                auto it_fwd = n->forward_neighbors_.find(p_dijkstra);
                if (it_fwd != n->forward_neighbors_.end()) {
                    fwd_exists = true;
                    fwd_dist = it_fwd->second.distance;
                    if (!it_fwd->second.invalidating_obstacles.empty()) {
                        fwd_blockers = "";
                        for(auto& s : it_fwd->second.invalidating_obstacles) fwd_blockers += s + " ";
                    }
                }

                // Check BACKWARD (Dijkstra uses this)
                double bwd_dist = -1.0;
                bool bwd_exists = false;
                std::string bwd_blockers = "None";

                auto it_bwd = p_dijkstra->backward_neighbors_.find(n);
                if (it_bwd != p_dijkstra->backward_neighbors_.end()) {
                    bwd_exists = true;
                    bwd_dist = it_bwd->second.distance;
                    if (!it_bwd->second.invalidating_obstacles.empty()) {
                        bwd_blockers = "";
                        for(auto& s : it_bwd->second.invalidating_obstacles) bwd_blockers += s + " ";
                    }
                }

                // Print Comparison
                std::cout << "       [D* Lite Sees] Forward Edge (" << n->getIndex() << "->" << p_dijkstra->getIndex() << "): "
                          << (fwd_exists ? (std::isinf(fwd_dist) ? "INF" : std::to_string(fwd_dist)) : "MISSING")
                          << " | Blockers: " << fwd_blockers << "\n";
                
                std::cout << "       [Dijkstra Sees] Backward Edge (" << p_dijkstra->getIndex() << "<-" << n->getIndex() << "): "
                          << (bwd_exists ? (std::isinf(bwd_dist) ? "INF" : std::to_string(bwd_dist)) : "MISSING")
                          << " | Blockers: " << bwd_blockers << "\n";

                if (fwd_exists && bwd_exists && std::abs(fwd_dist - bwd_dist) > 1e-9) {
                    std::cout << "       !!! SYNC ERROR: Forward and Backward edges do not match! !!!\n";
                }
                std::cout << "\n";
            }
        }
    }
    
    std::cout << "--------------------------------------------------------------------------------" << std::endl;
    std::cout << "Total Mismatches: " << mismatch_count << " / " << nodes_.size() << std::endl;
    std::cout << "Max Difference: " << max_diff << std::endl;
    std::cout << "============================================\n" << std::endl;
}




void KinodynamicPRMStarDStarLite::updateObstacleSamples(const ObstacleVector& turned_obstacles) {
    bool graph_changed = false; // Flag to track updates
    if (turned_obstacles.empty()) return;

    // 1. Manage km update (Standard D* Lite logic)
    // We need the OLD start position before we might have updated it elsewhere
    // Assuming 'start_node_' is the CURRENT robot position
    last_start_node = start_node_; 

    // --- ROBOT TIME HANDLING ---
    double T_robot = 0.0;
    if (!is_geometric_mode_) {
        // Only extract T_robot if we are in kinodynamic mode
        if (robot_continuous_state_.size() > 0) {
            T_robot = robot_continuous_state_(robot_continuous_state_.size() - 1);
        }
    }

    for (const auto& incoming_ob : turned_obstacles) {
        Obstacle& stored_ob = previous_obstacles_[incoming_ob.name];
        
        // Remove OLD Tube
        if (!stored_ob.predicted_path.empty()) {
            // Pass the flag by reference to know if costs actually changed
            removeObstacle(stored_ob); 
        }

        // Update Logic
        stored_ob = incoming_ob; 
        stored_ob.predicted_path = obs_checker_->generatePrediction(stored_ob, T_robot);

        // Add NEW Tube
        addNewObstacle(stored_ob);
    }

    // 2. Trigger D* Lite Repair
    // if (graph_changed) {
        // Update km before replanning
        // km += h(last_start, current_start)
        if (last_start_node) {
            km_ += heuristic(last_start_node, start_node_);
            // km_ += 0;
        }
        last_start_node = start_node_;

        computeShortestPath();

        // --- VERIFICATION RUN (Uncomment to test) ---
        // This will run Dijkstra and print if the costs match.
        // WARNING: This is slow! Only enable for debugging.
        #if DEBUG_WITH_DIJKSTRA_
            computeShortestPathDijkstraMode(); 
            debugCompareDijkstraVsDStarLite();
        #endif

    // }
}



// // USING THE INVALIDATING SET AS THE PRIMARY OBSTACLE CHECK CACHING!
// void KinodynamicPRMStarDStarLite::addNewObstacle(const Obstacle& ob) {
//     if (ob.predicted_path.empty()) return;

//     // 1. Calculate Radius
//     double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
//                    std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
//     double search_radius;
//     if (is_geometric_mode_) {
//         search_radius = obs_r + ob.inflation + connection_radius_;
//     } else {
//         double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
//         search_radius = obs_r + ob.inflation + connection_radius_ + gap_coverage_inflation;
//     }

//     // 2. Find Candidates
//     std::unordered_set<int> unique_node_indices;
//     for (const auto& point_3d : ob.predicted_path) {
//         Eigen::VectorXd query(kd_dim_);
//         if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
//         else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
        
//         std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
//         for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
//     }

//     // 3. Process Edges
//     for (int idx : unique_node_indices) {
//         DStarLiteNode* u = nodes_[idx].get();
//         bool u_needs_update = false;

//         for (auto& [v, edge_info] : u->forward_neighbors_) {
//             // Check collision
//             double edge_start_time = u->getTimeToGoal();
//             if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(edge_info.cached_trajectory, edge_start_time, ob)) {
                
//                 // --- A. Block Forward Edge (u -> v) ---
//                 // Always add the blocker name, even if already INF
//                 if (edge_info.distance != std::numeric_limits<double>::infinity()) {
//                     edge_info.distance = std::numeric_limits<double>::infinity();
//                     u_needs_update = true;
//                 }
//                 edge_info.invalidating_obstacles.insert(ob.name);

//                 // --- B. Block Backward Edge (v <- u) [CRITICAL SYNC] ---
//                 // Dijkstra uses this. It MUST be updated.
//                 auto it_back = v->backward_neighbors_.find(u);
//                 if (it_back != v->backward_neighbors_.end()) {
//                     it_back->second.distance = std::numeric_limits<double>::infinity();
//                     it_back->second.invalidating_obstacles.insert(ob.name);
//                 }

//                 // --- C. Geometric Symmetry (Block v -> u) ---
//                 if (is_geometric_mode_) {
//                     auto it_rev = v->forward_neighbors_.find(u);
//                     if (it_rev != v->forward_neighbors_.end()) {
//                         it_rev->second.distance = std::numeric_limits<double>::infinity();
//                         it_rev->second.invalidating_obstacles.insert(ob.name);
                        
//                         // Sync the backward edge for that reverse connection (u <- v)
//                         auto it_rev_back = u->backward_neighbors_.find(v);
//                         if (it_rev_back != u->backward_neighbors_.end()) {
//                             it_rev_back->second.distance = std::numeric_limits<double>::infinity();
//                             it_rev_back->second.invalidating_obstacles.insert(ob.name);
//                         }
//                         updateVertex(v); // v changed
//                     }
//                 }
//             }
//         }
//         if (u_needs_update) updateVertex(u);
//     }
// }
// void KinodynamicPRMStarDStarLite::removeObstacle(const Obstacle& ob_to_remove) {
//     if (ob_to_remove.predicted_path.empty()) return;

//     // 1. Setup Radius (Same as above)
//     double obs_r = (ob_to_remove.type == Obstacle::CIRCLE) ? ob_to_remove.dimensions.radius : 
//                    std::hypot(ob_to_remove.dimensions.width/2.0, ob_to_remove.dimensions.height/2.0);
//     double search_radius;
//     if (is_geometric_mode_) {
//         search_radius = obs_r + ob_to_remove.inflation + connection_radius_;
//     } else {
//         double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
//         search_radius = obs_r + ob_to_remove.inflation + connection_radius_ + gap_coverage_inflation;
//     }

//     // 2. Find Candidates
//     std::unordered_set<int> unique_node_indices;
//     for (const auto& point_3d : ob_to_remove.predicted_path) {
//         Eigen::VectorXd query(kd_dim_);
//         if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
//         else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
        
//         std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
//         for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
//     }

//     ObstacleVector all_obstacles = obs_checker_->getObstacles();

//     // 3. Restore Logic
//     for (int idx : unique_node_indices) {
//         DStarLiteNode* u = nodes_[idx].get();
//         bool u_needs_update = false;

//         for (auto& [v, edge_info] : u->forward_neighbors_) {
            
//             // Step 1: Was it blocked by THIS obstacle? (Use Set)
//             if (edge_info.invalidating_obstacles.erase(ob_to_remove.name) > 0) {
                
//                 // Step 2: Is it blocked by ANY OTHER obstacle? (Use Physics)
//                 bool conflicts_with_other = false;
//                 double edge_start_time = u->getTimeToGoal();

//                 // Only check physics if the set isn't empty, OR as a safety double-check
//                 // Generally if the set is not empty, we know it's blocked.
//                 if (edge_info.invalidating_obstacles.empty()) {
//                     for (const auto& other_ob : all_obstacles) {
//                         if (other_ob.name == ob_to_remove.name) continue; 

//                         if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(edge_info.cached_trajectory, edge_start_time, other_ob)) {
//                             conflicts_with_other = true;
//                             edge_info.invalidating_obstacles.insert(other_ob.name);
//                             break; 
//                         }
//                     }
//                 } else {
//                     conflicts_with_other = true; // Set is not empty -> still blocked
//                 }

//                 // Step 3: Restore if truly free
//                 if (!conflicts_with_other) {
//                     // A. Restore Forward
//                     edge_info.distance = edge_info.distance_original;
//                     edge_info.invalidating_obstacles.clear();
//                     u_needs_update = true;

//                     // B. Restore Backward (Sync Fix)
//                     auto it_back = v->backward_neighbors_.find(u);
//                     if (it_back != v->backward_neighbors_.end()) {
//                         it_back->second.distance = it_back->second.distance_original;
//                         it_back->second.invalidating_obstacles.clear();
//                     }

//                     // C. Restore Symmetric (Geometric Mode)
//                     if (is_geometric_mode_) {
//                         auto it_rev = v->forward_neighbors_.find(u);
//                         if (it_rev != v->forward_neighbors_.end()) {
//                             it_rev->second.distance = it_rev->second.distance_original;
//                             it_rev->second.invalidating_obstacles.clear();
                            
//                             auto it_rev_back = u->backward_neighbors_.find(v);
//                             if (it_rev_back != u->backward_neighbors_.end()) {
//                                 it_rev_back->second.distance = it_rev_back->second.distance_original;
//                                 it_rev_back->second.invalidating_obstacles.clear();
//                             }
//                             updateVertex(v);
//                         }
//                     }
//                 }
//             }
//         }
//         if (u_needs_update) updateVertex(u);
//     }
// }


void KinodynamicPRMStarDStarLite::addNewObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    // 1. Calculate Radius
    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    double search_radius;
    if (is_geometric_mode_) {
        search_radius = obs_r + ob.inflation + connection_radius_;
    } else {
        double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
        search_radius = obs_r + ob.inflation + connection_radius_ + gap_coverage_inflation;
    }

    // 2. Gather Unique Nodes
    std::unordered_set<int> unique_node_indices;
    for (const auto& point_3d : ob.predicted_path) {
        Eigen::VectorXd query(kd_dim_);
        if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
        else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
        
        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
    }

    // 3. Process Edges
    for (int idx : unique_node_indices) {
        DStarLiteNode* u = nodes_[idx].get();
        bool u_needs_update = false;

        for (auto& [v, edge_info] : u->forward_neighbors_) {
            
#if !USE_INVALIDATING_SET_STRATEGY
            // STRATEGY B (Physics Only): 
            // Optimization: If edge is already broken, we don't care who broke it.
            // We will check all obstacles on removal anyway.
            if (edge_info.distance == std::numeric_limits<double>::infinity()) continue;
#endif

            // STRATEGY A (Set):
            // We MUST check collision even if edge is INF, to populate the set correctly.
            // (Code falls through to collision check)

            double edge_start_time = u->getTimeToGoal();

            if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(edge_info.cached_trajectory, edge_start_time, ob)) {
                
                // 1. Block Forward
                if (edge_info.distance != std::numeric_limits<double>::infinity()) {
                    edge_info.distance = std::numeric_limits<double>::infinity();
                    u_needs_update = true;
                }

#if USE_INVALIDATING_SET_STRATEGY
                // Set Logic: Record the specific blocker
                edge_info.invalidating_obstacles.insert(ob.name);
#endif

                // 2. Block Backward [SYNC]
                auto it_back = v->backward_neighbors_.find(u);
                if (it_back != v->backward_neighbors_.end()) {
                    it_back->second.distance = std::numeric_limits<double>::infinity();
#if USE_INVALIDATING_SET_STRATEGY
                    it_back->second.invalidating_obstacles.insert(ob.name);
#endif
                }

                // 3. Geometric Symmetry
                if (is_geometric_mode_) {
                    auto it_rev = v->forward_neighbors_.find(u);
                    if (it_rev != v->forward_neighbors_.end()) {
                        bool rev_changed = false;
                        if (it_rev->second.distance != std::numeric_limits<double>::infinity()) {
                            it_rev->second.distance = std::numeric_limits<double>::infinity();
                            rev_changed = true;
                        }
#if USE_INVALIDATING_SET_STRATEGY
                        it_rev->second.invalidating_obstacles.insert(ob.name);
#endif
                        
                        auto it_rev_back = u->backward_neighbors_.find(v);
                        if (it_rev_back != u->backward_neighbors_.end()) {
                            it_rev_back->second.distance = std::numeric_limits<double>::infinity();
#if USE_INVALIDATING_SET_STRATEGY
                            it_rev_back->second.invalidating_obstacles.insert(ob.name);
#endif
                        }
                        if (rev_changed) updateVertex(v);
                    }
                }
            }
        }
        if (u_needs_update) updateVertex(u);
    }
}
void KinodynamicPRMStarDStarLite::removeObstacle(const Obstacle& ob_to_remove) {
    if (ob_to_remove.predicted_path.empty()) return;

    // 1. Setup Radius
    double obs_r = (ob_to_remove.type == Obstacle::CIRCLE) ? ob_to_remove.dimensions.radius : 
                   std::hypot(ob_to_remove.dimensions.width/2.0, ob_to_remove.dimensions.height/2.0);
    double search_radius;
    if (is_geometric_mode_) {
        search_radius = obs_r + ob_to_remove.inflation + connection_radius_;
    } else {
        double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
        search_radius = obs_r + ob_to_remove.inflation + connection_radius_ + gap_coverage_inflation;
    }

    // 2. Gather Candidates
    std::unordered_set<int> unique_node_indices;
    for (const auto& point_3d : ob_to_remove.predicted_path) {
        Eigen::VectorXd query(kd_dim_);
        if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
        else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
        
        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
    }

#if !USE_INVALIDATING_SET_STRATEGY
    // STRATEGY B: We need the full world state to check collisions
    ObstacleVector all_obstacles = obs_checker_->getObstacles();
#endif

    // 3. Restore Logic
    for (int idx : unique_node_indices) {
        DStarLiteNode* u = nodes_[idx].get();
        bool u_needs_update = false;

        for (auto& [v, edge_info] : u->forward_neighbors_) {
            
            // Common: Only check blocked edges
            if (edge_info.distance == std::numeric_limits<double>::infinity()) {
                
                bool should_restore = false;

// -----------------------------------------------------------
// STRATEGY A: INVALIDATING SET (O(1))
// -----------------------------------------------------------
#if USE_INVALIDATING_SET_STRATEGY
                
                // 1. Was blocked by this?
                if (edge_info.invalidating_obstacles.erase(ob_to_remove.name) > 0) {
                    // 2. Is it empty now?
                    if (edge_info.invalidating_obstacles.empty()) {
                        should_restore = true;
                    }
                }

// -----------------------------------------------------------
// STRATEGY B: PHYSICS CHECK ALL (O(N))
// -----------------------------------------------------------
#else
                double edge_start_time = u->getTimeToGoal();

                // 1. Was blocked by THIS (Old)?
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(edge_info.cached_trajectory, edge_start_time, ob_to_remove)) {
                    
                    // 2. Is blocked by ANY OTHER (Current)?
                    bool conflicts_with_other = false;
                    for (const auto& other_ob : all_obstacles) {
                        if (other_ob.name == ob_to_remove.name) continue; 

                        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(edge_info.cached_trajectory, edge_start_time, other_ob)) {
                            conflicts_with_other = true;
                            break; 
                        }
                    }
                    if (!conflicts_with_other) {
                        should_restore = true;
                    }
                }
#endif
// -----------------------------------------------------------

                if (should_restore) {
                    // A. Restore Forward
                    edge_info.distance = edge_info.distance_original;
                    u_needs_update = true;

                    // B. Restore Backward
                    auto it_back = v->backward_neighbors_.find(u);
                    if (it_back != v->backward_neighbors_.end()) {
                        it_back->second.distance = it_back->second.distance_original;
#if USE_INVALIDATING_SET_STRATEGY
                        it_back->second.invalidating_obstacles.erase(ob_to_remove.name);
#endif
                    }

                    // C. Restore Symmetric
                    if (is_geometric_mode_) {
                        auto it_rev = v->forward_neighbors_.find(u);
                        if (it_rev != v->forward_neighbors_.end()) {
                            it_rev->second.distance = it_rev->second.distance_original;
#if USE_INVALIDATING_SET_STRATEGY
                            it_rev->second.invalidating_obstacles.erase(ob_to_remove.name);
#endif
                            
                            auto it_rev_back = u->backward_neighbors_.find(v);
                            if (it_rev_back != u->backward_neighbors_.end()) {
                                it_rev_back->second.distance = it_rev_back->second.distance_original;
#if USE_INVALIDATING_SET_STRATEGY
                                it_rev_back->second.invalidating_obstacles.erase(ob_to_remove.name);
#endif
                            }
                            updateVertex(v);
                        }
                    }
                }
            }
        }
        if (u_needs_update) updateVertex(u);
    }
}

void KinodynamicPRMStarDStarLite::setRobotState(const Eigen::VectorXd& robot_state) {
    robot_continuous_state_ = robot_state;

    double robot_time;
    // Extract time for collision checking
    if(is_geometric_mode_)
        robot_time = 0;
    else
        robot_time = robot_continuous_state_(robot_continuous_state_.size() - 1);
    // --- 1. HYSTERESIS LOGIC ---
    const double hysteresis_factor = 0.98;
    double cost_of_current_anchor = std::numeric_limits<double>::infinity();
    
    if (start_node_ && start_node_->g != std::numeric_limits<double>::infinity()) {
        Trajectory bridge = statespace_->steer(robot_continuous_state_, start_node_->getStateValue());
        if (bridge.is_valid && obs_checker_->isTrajectorySafe(bridge, robot_time)) {
            cost_of_current_anchor = bridge.cost + start_node_->g;
        }
    }

    DStarLiteNode* best_candidate_node = nullptr;
    double best_candidate_cost = std::numeric_limits<double>::infinity();
    
    // --- 2. RADIUS EXPANSION LOGIC ---
    double current_search_radius = connection_radius_;
    const int max_attempts = 5;
    const double radius_multiplier = 2.0;
    
    // std::cout << "\n========== [setRobotState] ==========" << std::endl;
    // std::cout << "Robot Pos: (" << robot_state(0) << ", " << robot_state(1) << ")" << std::endl;
    // std::cout << "Initial Radius: " << current_search_radius << std::endl;

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        std::vector<size_t> candidate_indices;
        
        // Search within current radius
        if (use_kdtree_) {
            candidate_indices = kdtree_->radiusSearch(robot_state.head(kd_dim_), current_search_radius);
        }

        // std::cout << "Attempt " << attempt << " (Radius: " << current_search_radius 
        //           << "): Found " << candidate_indices.size() << " candidates." << std::endl;

        // Evaluate Candidates
        for (size_t idx : candidate_indices) {
            DStarLiteNode* candidate = nodes_[idx].get();

            // 1. CRITICAL: Candidate must be reachable from Goal (Finite G)
            if (candidate->g == std::numeric_limits<double>::infinity()) {
                continue;
            }

            // 2. Check Steering & Collision
            Trajectory bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());
            
            if (!bridge.is_valid) continue;
            if (!obs_checker_->isTrajectorySafe(bridge, robot_time)) continue;

            // 3. Calculate Total Cost: Bridge Cost + Node Cost
            double total_cost = bridge.cost + candidate->g;

            if (total_cost < best_candidate_cost) {
                best_candidate_cost = total_cost;
                best_candidate_node = candidate;
                bridge_cost_ = bridge.cost;
            }
        }

        // If we found a valid candidate, stop expanding
        if (best_candidate_node) {
            // std::cout << "  -> Found valid candidate in this radius." << std::endl;
            break;
        }

        // Expand radius for next attempt
        current_search_radius *= radius_multiplier;
    }

    // --- 3. ASSIGNMENT ---
    if (best_candidate_node && best_candidate_cost < cost_of_current_anchor * hysteresis_factor) {
        std::cout << ">>> SWITCHING Anchor to Node " << best_candidate_node->getIndex() 
                  << " (Cost: " << best_candidate_cost << ") <<<" << std::endl;
        
        // Update km (Key Modifier) for D* Lite if the start node changed
        if (start_node_ && start_node_ != best_candidate_node) {
            km_ += heuristic(start_node_, best_candidate_node);
            // km_ += 0;
        }
        
        start_node_ = best_candidate_node;
    } else if (start_node_ && cost_of_current_anchor != std::numeric_limits<double>::infinity()) {
        // std::cout << ">>> KEEPING Current Anchor Node " << start_node_->getIndex() 
        //           << " (Cost: " << cost_of_current_anchor << ") <<<" << std::endl;
    } else {
        std::cout << ">>> FAILURE: No valid anchor found even after expanding radius! <<<" << std::endl;
        start_node_ = nullptr;
        bridge_cost_= std::numeric_limits<double>::infinity();
    }
    
    // std::cout << "======================================\n" << std::endl;

    // // 4. Trigger Repair if necessary
    // if (open_queue_.empty() && start_node_ && start_node_->g != start_node_->rhs) {
    //      std::cout << "Triggering Repair..." << std::endl;
    //      computeShortestPath();
    // }
}

// void KinodynamicPRMStarDStarLite::visualizeGraph() {
//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> one_way_edges;

//     edges.reserve(nodes_.size() * (use_knn_ ? k_neighbors_ : 20));
//     one_way_edges.reserve(nodes_.size() * 2); // Reserve some space for one-way edges

//     for (const auto& node_ptr : nodes_) {
//         DStarLiteNode* u = node_ptr.get();

//         for (const auto& [neighbor, edge_info] : u->forward_neighbors_) {
//             if (edge_info.is_trajectory_computed) {
//                 const Eigen::Vector2d p1 = u->getStateValue().head<2>();
//                 const Eigen::Vector2d p2 = neighbor->getStateValue().head<2>();

//                 // Check if the reverse connection exists
//                 bool has_reverse = false;
//                 auto it = neighbor->forward_neighbors_.find(u);
//                 if (it != neighbor->forward_neighbors_.end()) {
//                     has_reverse = it->second.is_trajectory_computed;
//                 }

//                 if (has_reverse) {
//                     // Bidirectional connection -> Draw Gray
//                     edges.emplace_back(p1, p2);
//                 } else {
//                     // One-way connection -> Draw Red (Warning)
//                     one_way_edges.emplace_back(p1, p2);
//                 }
//             }
//         }
//     }

//     // 1. Visualize Normal Edges (Gray)
//     visualization_->visualizeEdges(edges, "map", "0.7,0.7,0.7", "prm_graph");

//     // 2. Visualize One-Way Edges (Red) - Optional, helps debugging
//     if (!one_way_edges.empty()) {
//         std::cout << "[Visualize] Found " << one_way_edges.size() << " one-way edges." << std::endl;
//         visualization_->visualizeEdges(one_way_edges, "map", "1.0,0.0,0.0", "prm_one_way");
//     }
// }


// void KinodynamicPRMStarDStarLite::visualizeTree() {
//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> all_valid_edges;
//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> spanning_tree_edges;
    
    
//     std::vector<Eigen::VectorXd> tree_nodes;

//     for (const auto& node_ptr : nodes_) {
//         DStarLiteNode* u = node_ptr.get();
//         tree_nodes.push_back(node_ptr->getStateValue());
        
//         // Skip nodes that haven't been discovered by D* Lite yet
//         if (u->rhs == std::numeric_limits<double>::infinity() || u->g == std::numeric_limits<double>::infinity()) continue;

//         for (const auto& [neighbor, edge_info] : u->forward_neighbors_) {
//             if (edge_info.distance == std::numeric_limits<double>::infinity()) continue;
            
//             const Eigen::Vector2d p1 = u->getStateValue().head<2>();
//             const Eigen::Vector2d p2 = neighbor->getStateValue().head<2>();

//             // 1. D* Lite Spanning Tree (Green)
//             if (u->getParent() == neighbor) {
//                 spanning_tree_edges.emplace_back(p1, p2);
//             } else {
//                 all_valid_edges.emplace_back(p1, p2);
//             }
//         }
        
//         // 2. Dijkstra Tree Visualization (Blue)
//         // Check if this node was visited by Dijkstra
//         if (dijkstra_tree_parents_.find(u) != dijkstra_tree_parents_.end()) {
//             DStarLiteNode* parent = dijkstra_tree_parents_[u];
//             if (parent) {
//                 const Eigen::Vector2d p1 = u->getStateValue().head<2>();
//                 const Eigen::Vector2d p2 = parent->getStateValue().head<2>();
//             }
//         }
//     }

//     // Draw Nodes
//     visualization_->visualizeNodes(tree_nodes, "map", 
//                             std::vector<float>{0.0f, 1.0f, 0.0f},  // Green nodes
//                             "tree_nodes");

//     // Draw D* Lite Tree (Green)
//     if (!spanning_tree_edges.empty()) {
//         visualization_->visualizeEdges(spanning_tree_edges, "map", "0.0,1.0,0.0", "dslite_tree");
//     }

// }


// void KinodynamicPRMStarDStarLite::visualizeTree() {
//     if (!goal_node_) return;

//     // 1) Find nodes that can currently reach the goal via finite forward edges.
//     std::unordered_set<DStarLiteNode*> reachable;
//     std::queue<DStarLiteNode*> q;
//     reachable.insert(goal_node_);
//     q.push(goal_node_);

//     while (!q.empty()) {
//         DStarLiteNode* cur = q.front(); q.pop();
//         // Walk predecessors: nodes that have forward edge pred -> cur
//         for (auto& [pred, edge_info] : cur->backward_neighbors_) {
//             // edge_info here corresponds to pred->cur if you maintain symmetrical insertion
//             if (edge_info.distance == std::numeric_limits<double>::infinity()) continue;
//             if (reachable.insert(pred).second) {
//                 q.push(pred);
//             }
//         }
//     }

//     // 2) Build edge lists but only include edges where both endpoints are in 'reachable'.
//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> dslite_tree_edges;
//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> other_valid_edges;
//     std::vector<Eigen::VectorXd> reachable_nodes_pos;

//     for (const auto& node_ptr : nodes_) {
//         DStarLiteNode* u = node_ptr.get();
//         if (reachable.find(u) == reachable.end()) continue; // skip nodes that cannot reach goal

//         // add node position for drawing
//         reachable_nodes_pos.push_back(u->getStateValue());

//         // collect forward edges u->v only when v is also reachable and edge finite
//         for (const auto& [v, edge_info] : u->forward_neighbors_) {
//             if (edge_info.distance == std::numeric_limits<double>::infinity()) continue;
//             if (reachable.find(v) == reachable.end()) continue; // only draw edges fully inside component
//             const Eigen::Vector2d p1 = u->getStateValue().head<2>();
//             const Eigen::Vector2d p2 = v->getStateValue().head<2>();

//             // If this forward neighbor is the chosen successor (best_parent_), draw as tree edge
//             if (u->best_parent_ == v) {
//                 dslite_tree_edges.emplace_back(p1, p2);
//             } else {
//                 other_valid_edges.emplace_back(p1, p2);
//             }
//         }
//     }

//     // Debug summary
//     std::cout << "[Viz] reachable_nodes=" << reachable_nodes_pos.size()
//               << " dslite_tree_edges=" << dslite_tree_edges.size()
//               << " other_valid_edges=" << other_valid_edges.size() << std::endl;

//     // 3) Visualize
//     // Draw reachable nodes (green)
//     if (!reachable_nodes_pos.empty()) {
//         visualization_->visualizeNodes(reachable_nodes_pos, "map",
//                                       std::vector<float>{0.0f, 1.0f, 0.0f}, "dslite_reachable_nodes");
//     }

//     // // Draw non-tree valid edges faint/gray
//     // if (!other_valid_edges.empty()) {
//     //     // If your Visualization has the rich overload:
//     //     visualization_->visualizeEdges(other_valid_edges, "map",
//     //                                    std::array<float,3>{0.7f, 0.7f, 0.7f}, 0.35f, 0.02f,
//     //                                    "valid_edges", 3, false, 0.2);
//     // } else {
//     //     // fallback if no rich overload: convert to old string-based call
//     //     // visualization_->visualizeEdges(other_valid_edges, "map", "0.7,0.7,0.7", "valid_edges");
//     // }

//     // Draw D* Lite spanning tree edges thin & bright on top
//     if (!dslite_tree_edges.empty()) {
//         visualization_->visualizeEdges(dslite_tree_edges, "map",
//                                        std::array<float,3>{0.0f, 1.0f, 0.0f}, 0.95f, 0.02f,
//                                        "dslite_tree", 2, false, 0.2);
//     }
// }




void KinodynamicPRMStarDStarLite::visualizeTree() {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> dslite_tree_edges;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> other_valid_edges;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> dijkstra_tree_edges;
    std::vector<Eigen::VectorXd> all_node_positions;
    std::vector<Eigen::VectorXd> dslite_nodes_pos;
    std::vector<Eigen::VectorXd> dijkstra_nodes_pos;

    size_t count_dslite_nodes = 0;

    // Build maps for quick lookup of Dijkstra visited nodes
    std::unordered_set<DStarLiteNode*> dijkstra_visited;
    for (const auto &kv : dijkstra_tree_parents_) dijkstra_visited.insert(kv.first);

    for (const auto& node_ptr : nodes_) {
        DStarLiteNode* u = node_ptr.get();
        Eigen::VectorXd pos = node_ptr->getStateValue();
        all_node_positions.push_back(pos);

        if (u->g != std::numeric_limits<double>::infinity()) {
            ++count_dslite_nodes;
            dslite_nodes_pos.push_back(pos);
        }

        // Draw forward edges for D* Lite tree (only edges with finite cost)
        for (const auto& [neighbor, edge_info] : u->forward_neighbors_) {
            if (edge_info.distance == std::numeric_limits<double>::infinity()) continue;
            Eigen::Vector2d p1 = u->getStateValue().head<2>();
            Eigen::Vector2d p2 = neighbor->getStateValue().head<2>();

            // If neighbor is the chosen parent (best_parent_) -> part of D* Lite tree
            if (u->best_parent_ == neighbor) {
                dslite_tree_edges.emplace_back(p1, p2);
            } else {
                other_valid_edges.emplace_back(p1, p2);
            }
        }

        // Dijkstra: if visited, draw node and an edge to its stored parent/successor
        auto it = dijkstra_tree_parents_.find(u);
        if (it != dijkstra_tree_parents_.end()) {
            dijkstra_nodes_pos.push_back(pos);
            DStarLiteNode* parent = it->second; // your computeShortestPathDijkstraMode stored successor here
            if (parent) {
                Eigen::Vector2d p1 = u->getStateValue().head<2>();
                Eigen::Vector2d p2 = parent->getStateValue().head<2>();
                dijkstra_tree_edges.emplace_back(p1, p2);
            }
        }
    }

    std::cout << "[Viz] total_nodes=" << nodes_.size()
              << " dslite_nodes=" << count_dslite_nodes
              << " dijkstra_nodes=" << dijkstra_nodes_pos.size()
              << " dslite_tree_edges=" << dslite_tree_edges.size()
              << " dijkstra_tree_edges=" << dijkstra_tree_edges.size() << std::endl;

    // // Draw all nodes (light gray)
    // visualization_->visualizeNodes(all_node_positions, "map",
    //                                std::vector<float>{0.6f, 0.6f, 0.6f}, "all_nodes");

    // // Draw nodes visited by D* Lite (green)
    // if (!dslite_nodes_pos.empty())
    //     visualization_->visualizeNodes(dslite_nodes_pos, "map",
    //                                    std::vector<float>{0.0f, 1.0f, 0.0f}, "dslite_nodes");

    // // Draw nodes visited by Dijkstra (blue)
    // if (!dijkstra_nodes_pos.empty())
    //     visualization_->visualizeNodes(dijkstra_nodes_pos, "map",
    //                                    std::vector<float>{0.0f, 0.0f, 1.0f}, "dijkstra_nodes");

    // draw Dijkstra base layer (thick, opaque)
    visualization_->visualizeEdges(dijkstra_tree_edges, "map",
        std::array<float,3>{0.0f, 0.0f, 1.0f},   // blue
        1.0f,                                    // alpha (opaque)
        0.12f,                                   // thick line
        "dijkstra_tree",                         // namespace
        1,                                       // marker id
        false,                                   // dashed
        0.5);

    // draw D* Lite overlay (thin, slightly transparent)
    visualization_->visualizeEdges(dslite_tree_edges, "map",
        std::array<float,3>{0.0f, 1.0f, 0.0f},   // green
        0.85f,                                   // alpha (slightly transparent)
        0.02f,                                   // thin line
        "dslite_tree",                           // namespace
        2,                                       // marker id
        false,                                   // dashed
        0.2);

    // // Optionally draw other valid edges lightly
    // if (!other_valid_edges.empty()) {
    //     visualization_->visualizeEdges(other_valid_edges, "map", "0.7,0.7,0.7", "valid_edges");
    // }
}


std::vector<Eigen::VectorXd> KinodynamicPRMStarDStarLite::getPathPositions() const{
    std::vector<Eigen::VectorXd> path;
    if (!start_node_ || !goal_node_) return path;

    // std::cout << "\n--- [Extract Path] ---" << std::endl;
    // std::cout << "Anchor Node: " << start_node_->getIndex() << std::endl;

    // 1. Add Bridge (Robot -> Anchor)
    Trajectory bridge = statespace_->steer(robot_continuous_state_, start_node_->getStateValue());
    if (bridge.is_valid && !bridge.path_points.empty()) {
        path.insert(path.end(), bridge.path_points.begin(), bridge.path_points.end());
        // std::cout << "Added bridge to anchor." << std::endl;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("DStarLite"), "Cannot steer from robot to anchor node.");
        return {};
    }

    // 2. Traverse Graph (Anchor -> Goal)
    DStarLiteNode* current_node = start_node_;
    int steps = 0;
    const int max_steps = nodes_.size();

    while (current_node != goal_node_) {
        if (steps++ > max_steps) {
            // RCLCPP_ERROR(rclcpp::get_logger("DStarLite"), "Path extraction exceeded max steps.");
            break;
        }
        
        DStarLiteNode* next_node = current_node->best_parent_;
        if (!next_node) {
            // RCLCPP_ERROR(rclcpp::get_logger("DStarLite"),
            //              "Path broken at Node %d. Next parent is NULL.",
            //              current_node->getIndex());
            break;
        }

        const Trajectory& traj = current_node->best_parent_trajectory_;
        if (traj.is_valid && traj.path_points.size() > 1) {
            path.insert(path.end(), traj.path_points.begin() + 1, traj.path_points.end());
        } else {
            path.push_back(next_node->getStateValue());
        }
        current_node = next_node;
    }
    
    // std::cout << "Path extracted with " << path.size() << " points." << std::endl;
    // std::cout << "----------------------\n" << std::endl;

    return path;
}


void KinodynamicPRMStarDStarLite::visualizePath(const std::vector<Eigen::VectorXd>& path_waypoints) {
    // A path needs at least two points to have an edge.
    if (path_waypoints.size() < 2) {
        return;
    }

    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    // Iterate through the waypoints to create line segments.
    // The loop goes to size() - 1 to prevent going out of bounds.
    for (size_t i = 0; i < path_waypoints.size() - 1; ++i) {
        // Create an edge from the current point to the next point.
        const Eigen::VectorXd& start_point = path_waypoints[i];
        const Eigen::VectorXd& end_point = path_waypoints[i+1];
        edges.emplace_back(start_point.head(2), end_point.head(2));
    }

    // Use your existing visualization class to draw the edges.
    // We'll use a distinct namespace and color (e.g., green and thick) to see it clearly.
    if (visualization_) {
        // The last argument is a namespace to keep it separate from the main tree visualization.
        visualization_->visualizeEdges(edges, "map", "0.0,1.0,0.0", "executable_path");
    }
}
