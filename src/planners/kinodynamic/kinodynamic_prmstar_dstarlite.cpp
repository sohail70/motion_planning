#include "motion_planning/planners/kinodynamic/kinodynamic_prmstar_dstarlite.hpp"

#define DEBUG_WITH_DIJKSTRA_ 0
#define USE_INVALIDATING_SET_STRATEGY 0
#define USE_THREAT_SET_STRATEGY 0
#define USE_GRID_SAMPLING 0
#define USE_PROPAGATE_DESCENDANTS 0  // This is not standard D* Lite but i put it for test purposes
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
    bool use_grid_sampling = false;
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


    bool use_rrtx_saved_samples_ = false;
    // 3. Build Graph (Sample + Identify Start/Goal)
    if (use_rrtx_saved_samples_) {
        std::cout << "Using RRTX saved samples." << std::endl;
        std::string filepath = "/home/sohail/motion_planning/build/rrtx_tree_nodes.csv";
        std::cout << "Loading nodes from file: " << filepath << "\n";
        std::ifstream fin(filepath);
        if (!fin.is_open()) {
            throw std::runtime_error("Failed to open node file: " + filepath);
        }

        std::string line;
        std::getline(fin, line); // Skip header

        std::string cell;
        while (std::getline(fin, line)) {
            std::stringstream lineStream(line);
            std::vector<double> state_values;
            
            std::getline(lineStream, cell, ','); // Skip node_id
            for(int i = 0; i < statespace_->getDimension(); ++i) {
                std::getline(lineStream, cell, ',');
                state_values.push_back(std::stod(cell));
            }

            Eigen::Map<Eigen::VectorXd> state_vec(state_values.data(), state_values.size());
            auto state_ptr = statespace_->addState(state_vec);
            auto node = std::make_unique<DStarLiteNode>(state_ptr, nodes_.size());
            
            // Set time to goal based on the state vector
            if (!is_geometric_mode_) {
                double absolute_t = node->getStateValue().tail<1>()[0];
                node->setTimeToGoal(absolute_t);
            } else {
                node->setTimeToGoal(0.0);
            }
            
            nodes_.push_back(std::move(node));
        }
        fin.close();
        std::cout << "Loaded " << nodes_.size() << " nodes from file.\n";

        // Find Start and Goal nodes
        // Remember D* Lite semantics: getGoal() = Search Root, getStart() = Robot Position
        const Eigen::VectorXd& mission_goal = problem_def_->getGoal();   
        const Eigen::VectorXd& mission_start = problem_def_->getStart(); 

        DStarLiteNode* best_search_root = nullptr;
        DStarLiteNode* best_robot_node = nullptr;
        double min_dist_to_root = std::numeric_limits<double>::infinity();
        double min_dist_to_robot = std::numeric_limits<double>::infinity();

        for (const auto& node_ptr : nodes_) {
            double dist_to_root = (node_ptr->getStateValue() - mission_goal).norm();
            if (dist_to_root < min_dist_to_root) {
                min_dist_to_root = dist_to_root;
                best_search_root = node_ptr.get();
            }

            double dist_to_robot = (node_ptr->getStateValue() - mission_start).norm();
            if (dist_to_robot < min_dist_to_robot) {
                min_dist_to_robot = dist_to_robot;
                best_robot_node = node_ptr.get();
            }
        }

        if (!best_search_root) throw std::runtime_error("Could not find root node.");
        if (!best_robot_node) throw std::runtime_error("Could not find robot node.");

        // D* Lite reverse semantics assignment
        start_node_ = best_search_root;
        start_node_->setTimeToGoal(0.0); 

        goal_node_ = best_robot_node; // TTG is already parsed from the CSV correctly

        std::cout << "Successfully identified start and goal nodes." << "\n";

    } // --- GRID SAMPLING LOGIC ---
    else if (use_grid_sampling) {
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

        for (int i = 0; i < num_samples_ - 2; ++i) {
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

// // Feb17 --> uses stack variable trajectory in edgeinfo
// void KinodynamicPRMStarDStarLite::near(int node_index) {
//     auto node = nodes_[node_index].get();
//     if (node->neighbors_cached_) return;

//     // --- GRID NEIGHBOR LOGIC ---
//     if (use_grid_sampling_) {
//         // 1. Convert linear index to 2D grid coordinates (row, col)
//         // Assuming row-major order: index = row * width + col
//         int row = node_index / grid_dim_per_side_;
//         int col = node_index % grid_dim_per_side_;

//         // 2. Define the 8 directions (dx, dy)
//         // (Up, Down, Left, Right, and 4 Diagonals)
//         int directions[8][2] = {
//             {-1, 0}, {1, 0}, {0, -1}, {0, 1},  // Cardinals
//             {-1, -1}, {-1, 1}, {1, -1}, {1, 1} // Diagonals
//         };

//         // 3. Iterate through directions
//         for (int i = 0; i < 8; ++i) {
//             int n_row = row + directions[i][0];
//             int n_col = col + directions[i][1];

//             // 4. Check boundaries (don't connect to nodes outside the grid)
//             if (n_row >= 0 && n_row < grid_dim_per_side_ &&
//                 n_col >= 0 && n_col < grid_dim_per_side_) {
                
//                 // 5. Convert back to linear index
//                 int neighbor_idx = n_row * grid_dim_per_side_ + n_col;

//                 // Safety check
//                 if (neighbor_idx >= 0 && neighbor_idx < (int)nodes_.size()) {
//                     DStarLiteNode* neighbor = nodes_[neighbor_idx].get();
                    
//                     // --- CONNECT NODES (Same logic as before) ---
                    
//                     // Test FORWARD connection (Node -> Neighbor)
//                     Trajectory traj_forward = statespace_->steer(node->getStateValue(), neighbor->getStateValue());
//                     if (traj_forward.is_valid && traj_forward.cost <= connection_radius_ + 0.01) {
//                         EdgeInfo info_forward;
//                         info_forward.distance = traj_forward.cost;
//                         info_forward.distance_original = info_forward.distance;
//                         info_forward.cached_trajectory = traj_forward;
//                         info_forward.is_trajectory_computed = true;
                        
//                         node->forward_neighbors_[neighbor] = info_forward;
//                         neighbor->backward_neighbors_[node] = info_forward;
//                     }

//                     // Test BACKWARD connection (Neighbor -> Node)
//                     Trajectory traj_backward;
//                     if (is_geometric_mode_) {
//                         traj_backward = traj_forward;
//                     } else {
//                         traj_backward = statespace_->steer(neighbor->getStateValue(), node->getStateValue());
//                     }
                    
//                     if (traj_backward.is_valid && traj_backward.cost <= connection_radius_ + 0.01) {
//                         EdgeInfo info_backward;
//                         info_backward.distance = traj_backward.cost;
//                         info_backward.distance_original = info_backward.distance;
//                         info_backward.cached_trajectory = traj_backward;
//                         info_backward.is_trajectory_computed = true;
                        
//                         neighbor->forward_neighbors_[node] = info_backward;
//                         node->backward_neighbors_[neighbor] = info_backward;
//                     }
//                 }
//             }
//         }
//     } 
//     // --- ORIGINAL KD-TREE LOGIC ---
//     else {
//         // 1. Get candidate neighbors
//         std::vector<size_t> candidate_indices;
//         if (use_knn_) {
//             if (k_neighbors_ > 0) {
//                 candidate_indices = kdtree_->knnSearch(node->getStateValue().head(kd_dim_), k_neighbors_);
//             }
//         } else {
//             if (connection_radius_ > 0) {
//                 candidate_indices = kdtree_->radiusSearch(node->getStateValue().head(kd_dim_), connection_radius_);
//             }
//         }

//         // 2. Populate Neighbors
//         for (int idx : candidate_indices) {
//             if (idx == node_index) continue;
//             DStarLiteNode* neighbor = nodes_[idx].get();

//             // Test FORWARD connection
//             Trajectory traj_forward = statespace_->steer(node->getStateValue(), neighbor->getStateValue());
//             if (traj_forward.is_valid) {
//                 EdgeInfo info_forward;
//                 info_forward.distance = traj_forward.cost;
//                 info_forward.distance_original = info_forward.distance;
//                 info_forward.cached_trajectory = traj_forward;
//                 info_forward.is_trajectory_computed = true;
                
//                 node->forward_neighbors_[neighbor] = info_forward;
//                 neighbor->backward_neighbors_[node] = info_forward;
//             }

//             // Test BACKWARD connection
//             Trajectory traj_backward;
//             if (is_geometric_mode_) {
//                 traj_backward = traj_forward;
//             } else {
//                 traj_backward = statespace_->steer(neighbor->getStateValue(), node->getStateValue());
//             }
//             if (traj_backward.is_valid) {
//                 EdgeInfo info_backward;
//                 info_backward.distance = traj_backward.cost;
//                 info_backward.distance_original = info_backward.distance;
//                 info_backward.cached_trajectory = traj_backward;
//                 info_backward.is_trajectory_computed = true;
                
//                 neighbor->forward_neighbors_[node] = info_backward;
//                 node->backward_neighbors_[neighbor] = info_backward;
//             }
//         }
//     }

//     node->neighbors_cached_ = true;
// }

void KinodynamicPRMStarDStarLite::near(int node_index) {
    auto node = nodes_[node_index].get();
    if (node->neighbors_cached_) return;

    // --- GRID NEIGHBOR LOGIC ---
    if (use_grid_sampling_) {
        int row = node_index / grid_dim_per_side_;
        int col = node_index % grid_dim_per_side_;

        int directions[8][2] = {
            {-1, 0}, {1, 0}, {0, -1}, {0, 1},  // Cardinals
            {-1, -1}, {-1, 1}, {1, -1}, {1, 1} // Diagonals
        };

        for (int i = 0; i < 8; ++i) {
            int n_row = row + directions[i][0];
            int n_col = col + directions[i][1];

            if (n_row >= 0 && n_row < grid_dim_per_side_ &&
                n_col >= 0 && n_col < grid_dim_per_side_) {
                
                int neighbor_idx = n_row * grid_dim_per_side_ + n_col;

                if (neighbor_idx >= 0 && neighbor_idx < (int)nodes_.size()) {
                    DStarLiteNode* neighbor = nodes_[neighbor_idx].get();
                    
                    // Test FORWARD connection
                    Trajectory traj_forward = statespace_->steer(node->getStateValue(), neighbor->getStateValue());
                    std::shared_ptr<Trajectory> shared_traj_forward; // Declare outside for reuse
                    
                    if (traj_forward.is_valid && traj_forward.cost <= connection_radius_ + 0.01) {
                        shared_traj_forward = std::make_shared<Trajectory>(std::move(traj_forward));
                        
                        EdgeInfo info_forward;
                        info_forward.distance = shared_traj_forward->cost;
                        info_forward.distance_original = shared_traj_forward->cost;
                        info_forward.cached_trajectory = shared_traj_forward;
                        info_forward.is_trajectory_computed = true;
                        
                        node->forward_neighbors_[neighbor] = info_forward;
                        neighbor->backward_neighbors_[node] = info_forward;
                    }

                    // Test BACKWARD connection
                    if (is_geometric_mode_) {
                        if (shared_traj_forward) { // Only assign if forward was valid
                            EdgeInfo info_backward;
                            info_backward.distance = shared_traj_forward->cost;
                            info_backward.distance_original = shared_traj_forward->cost;
                            info_backward.cached_trajectory = shared_traj_forward; // Reuse exact pointer!
                            info_backward.is_trajectory_computed = true;
                            
                            neighbor->forward_neighbors_[node] = info_backward;
                            node->backward_neighbors_[neighbor] = info_backward;
                        }
                    } else {
                        Trajectory traj_backward = statespace_->steer(neighbor->getStateValue(), node->getStateValue());
                        if (traj_backward.is_valid && traj_backward.cost <= connection_radius_ + 0.01) {
                            auto shared_traj_backward = std::make_shared<Trajectory>(std::move(traj_backward));
                            
                            EdgeInfo info_backward;
                            info_backward.distance = shared_traj_backward->cost;
                            info_backward.distance_original = shared_traj_backward->cost;
                            info_backward.cached_trajectory = shared_traj_backward;
                            info_backward.is_trajectory_computed = true;
                            
                            neighbor->forward_neighbors_[node] = info_backward;
                            node->backward_neighbors_[neighbor] = info_backward;
                        }
                    }
                }
            }
        }
    } 
    // --- ORIGINAL KD-TREE LOGIC ---
    else {
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

        for (int idx : candidate_indices) {
            if (idx == node_index) continue;
            DStarLiteNode* neighbor = nodes_[idx].get();

            // Test FORWARD connection
            Trajectory traj_forward = statespace_->steer(node->getStateValue(), neighbor->getStateValue());
            std::shared_ptr<Trajectory> shared_traj_forward;
            
            if (traj_forward.is_valid) {
                shared_traj_forward = std::make_shared<Trajectory>(std::move(traj_forward));
                
                EdgeInfo info_forward;
                info_forward.distance = shared_traj_forward->cost;
                info_forward.distance_original = shared_traj_forward->cost;
                info_forward.cached_trajectory = shared_traj_forward;
                info_forward.is_trajectory_computed = true;
                
                node->forward_neighbors_[neighbor] = info_forward;
                neighbor->backward_neighbors_[node] = info_forward;
            }

            // Test BACKWARD connection
            if (is_geometric_mode_) {
                if (shared_traj_forward) { // Only assign if forward was valid
                    EdgeInfo info_backward;
                    info_backward.distance = shared_traj_forward->cost;
                    info_backward.distance_original = shared_traj_forward->cost;
                    info_backward.cached_trajectory = shared_traj_forward; // Reuse pointer
                    info_backward.is_trajectory_computed = true;
                    
                    neighbor->forward_neighbors_[node] = info_backward;
                    node->backward_neighbors_[neighbor] = info_backward;
                }
            } else {
                Trajectory traj_backward = statespace_->steer(neighbor->getStateValue(), node->getStateValue());
                if (traj_backward.is_valid) {
                    auto shared_traj_backward = std::make_shared<Trajectory>(std::move(traj_backward));
                    
                    EdgeInfo info_backward;
                    info_backward.distance = shared_traj_backward->cost;
                    info_backward.distance_original = shared_traj_backward->cost;
                    info_backward.cached_trajectory = shared_traj_backward;
                    info_backward.is_trajectory_computed = true;
                    
                    neighbor->forward_neighbors_[node] = info_backward;
                    node->backward_neighbors_[neighbor] = info_backward;
                }
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



// void KinodynamicPRMStarDStarLite::updateVertex(DStarLiteNode* u) {
//     if (u == goal_node_) {
//         u->rhs = 0.0;
//         u->best_parent_ = nullptr;
//         u->best_parent_trajectory_ = Trajectory();
//     } 
//     else {
//         double min_rhs = std::numeric_limits<double>::infinity();
//         DStarLiteNode* best_parent = nullptr;
//         Trajectory best_traj;

// #if DEBUG_WITH_DIJKSTRA_
//         // Debug Mode: Sorting for Determinism
//         for (auto& [succ, edge_info] : u->forward_neighbors_) {
//             if (edge_info.distance == std::numeric_limits<double>::infinity()) continue;
//             if (succ->g == std::numeric_limits<double>::infinity()) continue;
//             double cost = edge_info.distance + succ->g;
//             if (cost < min_rhs) min_rhs = cost;
//         }
//         if (std::isfinite(min_rhs)) {
//             std::vector<DStarLiteNode*> candidates;
//             candidates.reserve(4);
//             for (auto& [succ, edge_info] : u->forward_neighbors_) {
//                 if (edge_info.distance == std::numeric_limits<double>::infinity()) continue;
//                 if (succ->g == std::numeric_limits<double>::infinity()) continue;
//                 double cost = edge_info.distance + succ->g;
//                 if (std::abs(cost - min_rhs) < 1e-9) candidates.push_back(succ);
//             }
//             if (!candidates.empty()) {
//                 std::sort(candidates.begin(), candidates.end(), [](DStarLiteNode* a, DStarLiteNode* b) { return a->getIndex() < b->getIndex(); });
//                 best_parent = candidates[0];
//                 best_traj = u->forward_neighbors_[best_parent].cached_trajectory;
//             }
//         }
// #else
//         // Production Mode: Fast One-Pass
//         for (auto& [succ, edge_info] : u->forward_neighbors_) {
//             // BOTH STRATEGIES: Trust the distance value
//             if (edge_info.distance == std::numeric_limits<double>::infinity()) continue;
//             if (succ->g == std::numeric_limits<double>::infinity()) continue;

//             double cost = edge_info.distance + succ->g;
//             if (cost < min_rhs) {
//                 min_rhs = cost;
//                 best_parent = succ;
//                 best_traj = edge_info.cached_trajectory;
//             }
//         }
// #endif

//         u->rhs = min_rhs;
//         // u->best_parent_ = best_parent;
//         // u->best_parent_trajectory_ = best_traj;
//         u->setBestParent(best_parent, best_traj);
//     }

//     if (u->in_queue_) open_queue_.remove(u);
//     if (u->g != u->rhs) open_queue_.add(u, calculateKey(u));
// }

void KinodynamicPRMStarDStarLite::updateVertex(DStarLiteNode* u) {
    bool is_consistent = (u->g == u->rhs);
    
    if (!is_consistent && u->in_queue_) {
        // Line 07": Update priority
        open_queue_.update(u, calculateKey(u));
    } 
    else if (!is_consistent && !u->in_queue_) {
        // Line 08": Insert into queue
        open_queue_.add(u, calculateKey(u));
    } 
    else if (is_consistent && u->in_queue_) {
        // Line 09": Remove from queue
        open_queue_.remove(u);
    }
}

// Equivalent to Line 27": rhs(s) = min_{s' in Succ(s)} (c(s, s') + g(s'))
void KinodynamicPRMStarDStarLite::recomputeRHS(DStarLiteNode* s) {
    if (s == goal_node_) return; // Goal is always 0
    
    double min_rhs = std::numeric_limits<double>::infinity(); // There is no orphan handling like RRTx here so we start with INF and recompute from scratch
    DStarLiteNode* best_parent = nullptr;
    // Trajectory best_traj;
    std::shared_ptr<Trajectory> best_traj = nullptr;

    for (auto& [succ, edge_info] : s->forward_neighbors_) {
        // Trust the edge distance (handled by obstacle invalidation logic)
        if (edge_info.distance == std::numeric_limits<double>::infinity()) continue;
        if (succ->g == std::numeric_limits<double>::infinity()) continue;

        double cost = edge_info.distance + succ->g;
        if (cost < min_rhs - 1e-9) {
            min_rhs = cost;
            best_parent = succ;
            best_traj = edge_info.cached_trajectory;
        }
#if DEBUG_WITH_DIJKSTRA_
        // 2. TIE-BREAKER: Equal cost, but lower Node Index
        else if (std::abs(cost - min_rhs) <= 1e-9) {
            if (best_parent && succ->getIndex() < best_parent->getIndex()) {
                // Keep min_rhs the same, but switch to the preferred parent
                best_parent = succ;
                best_traj = edge_info.cached_trajectory;
            }
        }
#endif
    }

    s->rhs = min_rhs;
    s->setBestParent(best_parent, best_traj);
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

// void KinodynamicPRMStarDStarLite::computeShortestPath() {
//     if (!start_node_ || !goal_node_) return;

//     // Paper condition: while (U.TopKey() < CalculateKey(s_start) OR rhs(s_start) != g(s_start))
//     while (open_queue_.topKey() < calculateKey(start_node_) || start_node_->rhs != start_node_->g) {
//     // while (!open_queue_.empty()) { //FOR DEBUG
        
//         DStarLiteKey k_old = open_queue_.topKey();
//         DStarLiteNode* u = open_queue_.pop();
//         if (!u) break;

//         DStarLiteKey k_new = calculateKey(u);

//         if (k_old < k_new) {
//             // Lazy update: the key in the heap was stale, re-insert with fresh k_m
//             open_queue_.add(u, k_new);
//         } 
//         else if (u->g > u->rhs) {
//             // Overconsistent: cost decreased
//             u->g = u->rhs;
//             // Propagate to predecessors (backward neighbors)
//             for (auto& [pred, edge_info] : u->backward_neighbors_) {
//                 updateVertex(pred);
//             }
//         } 
//         else {
//             // Underconsistent: cost increased
//             u->g = std::numeric_limits<double>::infinity();
//             // Update u itself and all predecessors
//             updateVertex(u);
//             for (auto& [pred, edge_info] : u->backward_neighbors_) {
//                 updateVertex(pred);
//             }
//         }
//     }
// }


void KinodynamicPRMStarDStarLite::computeShortestPath() {
    if (!start_node_ || !goal_node_) return;

    // // Line 10": while U.TopKey() < CalculateKey(s_start) OR rhs(s_start) > g(s_start)
    // // Note: D* Lite optimized version strictly uses > here, not !=
    // while (!open_queue_.empty() && 
    //        (open_queue_.topKey() < calculateKey(start_node_) || start_node_->rhs > start_node_->g)) {

    // 1. Get the standard discrete key for the anchor node
    DStarLiteKey target_key = calculateKey(start_node_);
    
    // 2. INFLATE the key by the continuous bridge cost. 
    // This perfectly mirrors your RRTx logic: min_key > vbot_node_->getCost() + bridge_cost_
    // It forces the queue to resolve the local continuous neighborhood around the robot.
    if (std::isfinite(bridge_cost_)) {
        target_key.k1 += bridge_cost_;
        target_key.k2 += bridge_cost_;
    }

#if DEBUG_WITH_DIJKSTRA_
    while (!open_queue_.empty()){
#endif

#if !DEBUG_WITH_DIJKSTRA_
        // 3. Modified Termination Condition
    while (!open_queue_.empty() && 
            (open_queue_.topKey() < target_key || start_node_->rhs > start_node_->g)) {
#endif      
    
        // Lines 11"-13"
        DStarLiteNode* u = open_queue_.top();
        DStarLiteKey k_old = open_queue_.topKey();
        if (!u) break;

        DStarLiteKey k_new = calculateKey(u);

        // Line 14"
        if (k_old < k_new) {
            // update
            open_queue_.update(u, k_new);
        } 
        // Line 16": Overconsistent (Found a shortcut!)
        else if (u->g > u->rhs) {
            u->g = u->rhs;
            open_queue_.remove(u);
            // PUSH LOGIC (Lines 19"-21"): O(1) per predecessor!
            for (auto& [pred, edge_info] : u->backward_neighbors_) {
                if (pred != goal_node_) {
                    double new_cost = edge_info.distance + u->g;
                    // If this new path through u is better, PUSH the update
                    if (pred->rhs > new_cost + 1e-9) {
                        pred->rhs = new_cost;
                        pred->setBestParent(u, edge_info.cached_trajectory);
                    }
#if DEBUG_WITH_DIJKSTRA_
                    // 2. TIE-BREAKER: Equal cost, but lower Node Index
                    else if (std::abs(pred->rhs - new_cost) <= 1e-9) {
                        if (pred->getParent() && u->getIndex() < pred->getParent()->getIndex()) {
                            pred->rhs = new_cost; // strictly speaking, cost is the same
                            pred->setBestParent(u, edge_info.cached_trajectory);
                        }
                    }
#endif

                }
                updateVertex(pred);
            }
        } 
        // Line 22": Underconsistent (Path was blocked!)
        else {
            double g_old = u->g;
            u->g = std::numeric_limits<double>::infinity();
            
            // Lines 25"-28": Process Predecessors
            for (auto& [pred, edge_info] : u->backward_neighbors_) {
                // Was u the reason rhs(s) had its value? If yes, then rhs(s) must be recomputed
               if (pred->getParent() == u) { // parent(s) = u  EQUIVALENT TO  rhs(s) = c(s,u) + g(u)
                    recomputeRHS(pred); // I check the S not equal s_goal inside this functions
                } 
                updateVertex(pred);
            }
            
            // Process u itself (part of Pred(u) U {u}) no need for the rhs condtion here because Did u depend on itself for its rhs? that can no happen!
            if (u != goal_node_) {
                recomputeRHS(u);
            }
            updateVertex(u);
        }
    }
}



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
                        // for(auto& s : it_fwd->second.invalidating_obstacles) fwd_blockers += s + " ";
                        // CORRECT: Access the name property through the pointer
                        for(const Obstacle* ob_ptr : it_fwd->second.invalidating_obstacles) {
                            fwd_blockers += ob_ptr->name + " ";
                        }
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
                        // for(auto& s : it_bwd->second.invalidating_obstacles) bwd_blockers += s + " ";

                        // CORRECT: Access the name property through the pointer
                        for(const Obstacle* ob_ptr : it_bwd->second.invalidating_obstacles) {
                            bwd_blockers += ob_ptr->name + " ";
                        }
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
#if USE_PROPAGATE_DESCENDANTS
    propagateDescendants();
#endif

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





// void KinodynamicPRMStarDStarLite::propagateDescendants() {
//     if (orphans_.empty()) return;

//     std::queue<DStarLiteNode*> q;
//     std::unordered_set<DStarLiteNode*> processed_orphans;

//     // 1. Initialize Queue
//     for (auto u : orphans_) {
//         q.push(u);
//     }

//     // 2. BFS to find all descendants
//     while (!q.empty()) {
//         DStarLiteNode* curr = q.front(); 
//         q.pop();
        
//         processed_orphans.insert(curr);

//         for (DStarLiteNode* child : curr->children_) {
//             if (processed_orphans.find(child) == processed_orphans.end()) {
//                 q.push(child);
//             }
//         }
//     }

//     // 3. The "Kill" Phase (Bypass the Priority Queue)
//     for (DStarLiteNode* u : processed_orphans) {
//         u->g = std::numeric_limits<double>::infinity();
//         u->rhs = std::numeric_limits<double>::infinity();
//         u->setBestParent(nullptr, Trajectory());
        
//         if (u->in_queue_) {
//             open_queue_.remove(u);
//         }
//     }

//     // 4. The "Re-Integrate" Phase
//     // Force them to look at their forward neighbors. If a neighbor survived,
//     // this orphan will get a finite RHS and jump BACK into the queue as overconsistent!
//     for (DStarLiteNode* u : processed_orphans) {
//         recomputeRHS(u); // <-- ADDED THIS: Node needs to see if any neighbors survived
//         updateVertex(u);
//     }

//     orphans_.clear();
// }

// // INVALIDATING SET STRATEGY!!
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

//     // 2. Gather Unique Nodes
//     std::unordered_set<int> unique_node_indices;
//     for (const auto& point_3d : ob.predicted_path) {
//         Eigen::VectorXd query(kd_dim_);
//         if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
//         else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
//         else if (kd_dim_ == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
//         else if (kd_dim_ == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
//         std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
//         for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
//     }

//     // --- HELPER LAMBDA: Clean Encapsulation ---
//     auto checkAndBlockEdge = [&](DStarLiteNode* node, DStarLiteNode* neighbor, EdgeInfo& edge, bool& u_needs_update) {
// #if !USE_INVALIDATING_SET_STRATEGY
//         if (edge.distance == std::numeric_limits<double>::infinity()) return; 
// #endif
//         const double edge_start_ttg = node->getTimeToGoal();
//         last_replan_metrics_.obstacle_checks++;

//         // Physics Check
//         if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), edge_start_ttg, ob)) {
            
//             // 1. Mark as Blocked (Forward Edge)
//             if (edge.distance != std::numeric_limits<double>::infinity()) {
//                 edge.distance = std::numeric_limits<double>::infinity();
//                 u_needs_update = true;
//             }
            
// #if USE_INVALIDATING_SET_STRATEGY
//             edge.invalidating_obstacles.insert(ob.name);
// #endif

//             // 2. Symmetric Update (Backward Neighbor perspective)
//             if (neighbor->backward_neighbors_.count(node)) {
//                 auto& inc_edge = neighbor->backward_neighbors_.at(node);
//                 inc_edge.distance = std::numeric_limits<double>::infinity();
// #if USE_INVALIDATING_SET_STRATEGY
//                 inc_edge.invalidating_obstacles.insert(ob.name);
// #endif
//             }

//             // 3. Geometric Mode Optimization (Break reverse path immediately)
//             if (is_geometric_mode_) {
//                 if (neighbor->forward_neighbors_.count(node)) {
//                     auto& rev_edge = neighbor->forward_neighbors_.at(node);
//                     bool rev_changed = false;
//                     if (rev_edge.distance != std::numeric_limits<double>::infinity()) {
//                         rev_edge.distance = std::numeric_limits<double>::infinity();
//                         rev_changed = true;
//                     }
// #if USE_INVALIDATING_SET_STRATEGY
//                     rev_edge.invalidating_obstacles.insert(ob.name);
// #endif
//                     if (node->backward_neighbors_.count(neighbor)) {
//                         auto& rev_inc_edge = node->backward_neighbors_.at(neighbor);
//                         rev_inc_edge.distance = std::numeric_limits<double>::infinity();
// #if USE_INVALIDATING_SET_STRATEGY
//                         rev_inc_edge.invalidating_obstacles.insert(ob.name);
// #endif
//                     }
//                     if (rev_changed) {
//                         recomputeRHS(neighbor); 
//                         updateVertex(neighbor);
//                     }
//                 }
//             }
//         }
//     };
//     // ------------------------------------------------

//     // 3. Process Nodes
//     for (int idx : unique_node_indices) {
//         DStarLiteNode* u = nodes_[idx].get();
//         bool u_needs_update = false;
        
//         for (auto& [neighbor, edge] : u->forward_neighbors_) {
//             checkAndBlockEdge(u, neighbor, edge, u_needs_update);
//         }
        
//         if (u_needs_update) {
//             recomputeRHS(u); 
//             updateVertex(u);
//         }
//     }
// }

// void KinodynamicPRMStarDStarLite::removeObstacle(const Obstacle& ob) {
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

//     // 2. Gather Unique Nodes
//     std::unordered_set<int> unique_node_indices;
//     for (const auto& point_3d : ob.predicted_path) {
//         Eigen::VectorXd query(kd_dim_);
//         if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
//         else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
//         else if (kd_dim_ == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
//         else if (kd_dim_ == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
//         std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
//         for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
//     }

// #if !USE_INVALIDATING_SET_STRATEGY
//     ObstacleVector all_obstacles = obs_checker_->getObstacles();
// #endif

//     // --- HELPER LAMBDA: Clean Encapsulation ---
//     auto checkAndRestoreEdge = [&](DStarLiteNode* node, DStarLiteNode* neighbor, EdgeInfo& edge, bool& u_needs_update) {
//         if (edge.distance == std::numeric_limits<double>::infinity()) {
//             bool should_restore = false;

// #if USE_INVALIDATING_SET_STRATEGY
//             if (edge.invalidating_obstacles.erase(ob.name) > 0) {
//                 if (edge.invalidating_obstacles.empty()) {
//                     should_restore = true;
//                 }
//             }
// #else
//             const double ttg = node->getTimeToGoal();
//             if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ttg, ob)) {
//                 bool conflicts_with_other = false;
//                 for (const auto& other_ob : all_obstacles) {
//                     if (other_ob.name == ob.name) continue; 
//                     last_replan_metrics_.obstacle_checks++;
//                     if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ttg, other_ob)) {
//                         conflicts_with_other = true;
//                         break; 
//                     }
//                 }
//                 if (!conflicts_with_other) {
//                     should_restore = true;
//                 }
//             }
// #endif

//             if (should_restore) {
//                 // 1. Restore Forward
//                 edge.distance = edge.distance_original;
//                 u_needs_update = true;
                
//                 // 2. Restore Backward (Symmetry)
//                 if (neighbor->backward_neighbors_.count(node)) {
//                     auto& inc_edge = neighbor->backward_neighbors_.at(node);
//                     inc_edge.distance = edge.distance_original;
// #if USE_INVALIDATING_SET_STRATEGY
//                     inc_edge.invalidating_obstacles.erase(ob.name);
// #endif
//                 }

//                 // 3. Geometric Mode Optimization (Restore reverse path immediately)
//                 if (is_geometric_mode_) {
//                     if (neighbor->forward_neighbors_.count(node)) {
//                         auto& rev_edge = neighbor->forward_neighbors_.at(node);
//                         rev_edge.distance = rev_edge.distance_original;
// #if USE_INVALIDATING_SET_STRATEGY
//                         rev_edge.invalidating_obstacles.erase(ob.name);
// #endif
//                         if (node->backward_neighbors_.count(neighbor)) {
//                             auto& rev_inc_edge = node->backward_neighbors_.at(neighbor);
//                             rev_inc_edge.distance = rev_inc_edge.distance_original;
// #if USE_INVALIDATING_SET_STRATEGY
//                             rev_inc_edge.invalidating_obstacles.erase(ob.name);
// #endif
//                         }
//                         recomputeRHS(neighbor);
//                         updateVertex(neighbor);
//                     }
//                 }
//             }
//         }
//     };
//     // ------------------------------------------------

//     // 3. Process Nodes
//     for (int idx : unique_node_indices) {
//         DStarLiteNode* u = nodes_[idx].get();
//         bool u_needs_update = false;
        
//         for (auto& [neighbor, edge] : u->forward_neighbors_) {
//             checkAndRestoreEdge(u, neighbor, edge, u_needs_update);
//         }
        
//         if (u_needs_update) {
//             recomputeRHS(u);
//             updateVertex(u);
//         }
//     }
// }

// // USING THREAT SET STRATEGY!
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

//     // 2. Gather Unique Nodes
//     std::unordered_set<int> unique_node_indices;
//     for (const auto& point_3d : ob.predicted_path) {
//         Eigen::VectorXd query(kd_dim_);
//         if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
//         else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
//         else if (kd_dim_ == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
//         else if (kd_dim_ == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
//         std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
//         for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
//     }

//     // --- HELPER LAMBDA ---
//     auto checkAndBlockEdge = [&](DStarLiteNode* node, DStarLiteNode* neighbor, EdgeInfo& edge, bool& u_needs_update) {
//         // THE FILTER: Skip if already blocked
//         if (edge.distance == std::numeric_limits<double>::infinity()) return; 

//         const double edge_start_ttg = node->getTimeToGoal();
//         last_replan_metrics_.obstacle_checks++;

//         // Physics Check ONLY against the new obstacle
//         if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), edge_start_ttg, ob)) {
            
//             // 1. Mark as Blocked (Forward Edge)
//             edge.distance = std::numeric_limits<double>::infinity();
//             u_needs_update = true;

//             // 2. Symmetric Update (Backward Neighbor perspective)
//             if (neighbor->backward_neighbors_.count(node)) {
//                 neighbor->backward_neighbors_.at(node).distance = std::numeric_limits<double>::infinity();
//             }

//             // 3. Geometric Mode Optimization (Break reverse path immediately)
//             if (is_geometric_mode_) {
//                 if (neighbor->forward_neighbors_.count(node)) {
//                     auto& rev_edge = neighbor->forward_neighbors_.at(node);
//                     bool rev_changed = false;
                    
//                     if (rev_edge.distance != std::numeric_limits<double>::infinity()) {
//                         rev_edge.distance = std::numeric_limits<double>::infinity();
//                         rev_changed = true;
//                     }
//                     if (node->backward_neighbors_.count(neighbor)) {
//                         node->backward_neighbors_.at(neighbor).distance = std::numeric_limits<double>::infinity();
//                     }
//                     if (rev_changed) {
//                         recomputeRHS(neighbor); 
//                         updateVertex(neighbor);
//                     }
//                 }
//             }
//         }
//     };
//     // ------------------------------------------------

//     // 3. Process Nodes
//     for (int idx : unique_node_indices) {
//         DStarLiteNode* u = nodes_[idx].get();
//         bool u_needs_update = false;
        
//         // --- 1. ADD THREAT TO THE NODE ---
//         u->threats_.insert(ob.name);
        
//         // --- 2. CHECK EDGES ---
//         for (auto& [neighbor, edge] : u->forward_neighbors_) {
//             checkAndBlockEdge(u, neighbor, edge, u_needs_update);
//         }
        
//         // --- 3. TRIGGER D* LITE UPDATE ---
//         if (u_needs_update) {
//             recomputeRHS(u); 
//             updateVertex(u);
//         }
//     }
// }
// void KinodynamicPRMStarDStarLite::removeObstacle(const Obstacle& ob) {
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

//     // 2. Gather Unique Nodes
//     std::unordered_set<int> unique_node_indices;
//     for (const auto& point_3d : ob.predicted_path) {
//         Eigen::VectorXd query(kd_dim_);
//         if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
//         else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
//         else if (kd_dim_ == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
//         else if (kd_dim_ == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
//         std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
//         for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
//     }

//     // --- HELPER LAMBDA ---
//     auto checkAndRestoreEdge = [&](DStarLiteNode* node, DStarLiteNode* neighbor, EdgeInfo& edge, bool& u_needs_update) {
//         if (edge.distance == std::numeric_limits<double>::infinity()) {
//             const double ttg = node->getTimeToGoal();
            
//             // THE SPEEDUP: Check against the node's current threat list
//             bool is_safe = true;
//             for (const std::string& threat_name : node->threats_) {
//                 last_replan_metrics_.obstacle_checks++;
                
//                 // Fetch the fully updated obstacle directly from the tracking map
//                 const Obstacle& threat_ob = previous_obstacles_.at(threat_name); 
                
//                 if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ttg, threat_ob)) {
//                     is_safe = false;
//                     break; // Edge is still physically blocked by another threat
//                 }
//             }

//             if (is_safe) {
//                 // 1. Restore Forward
//                 edge.distance = edge.distance_original;
//                 u_needs_update = true;
                
//                 // 2. Restore Backward (Symmetry)
//                 if (neighbor->backward_neighbors_.count(node)) {
//                     neighbor->backward_neighbors_.at(node).distance = edge.distance_original;
//                 }

//                 // 3. Geometric Mode Optimization (Restore reverse path immediately)
//                 if (is_geometric_mode_) {
//                     if (neighbor->forward_neighbors_.count(node)) {
//                         auto& rev_edge = neighbor->forward_neighbors_.at(node);
//                         rev_edge.distance = rev_edge.distance_original;
                        
//                         if (node->backward_neighbors_.count(neighbor)) {
//                             node->backward_neighbors_.at(neighbor).distance = rev_edge.distance_original;
//                         }
                        
//                         recomputeRHS(neighbor);
//                         updateVertex(neighbor);
//                     }
//                 }
//             }
//         }
//     };
//     // ------------------------------------------------

//     // 3. Process Nodes
//     for (int idx : unique_node_indices) {
//         DStarLiteNode* u = nodes_[idx].get();
//         bool u_needs_update = false;
        
//         // --- 1. REMOVE THREAT FROM THE NODE ---
//         u->threats_.erase(ob.name);
        
//         // --- 2. RESTORE EDGES ---
//         for (auto& [neighbor, edge] : u->forward_neighbors_) {
//             checkAndRestoreEdge(u, neighbor, edge, u_needs_update);
//         }
        
//         // --- 3. TRIGGER D* LITE UPDATE ---
//         if (u_needs_update) {
//             recomputeRHS(u);
//             updateVertex(u);
//         }
//     }
// }


void KinodynamicPRMStarDStarLite::setRobotState(const Eigen::VectorXd& robot_state) {
    robot_continuous_state_ = robot_state;

    // Extract actual planner-time (Time-to-Go) from the state (last element)
    double robot_time_to_go = 0.0;
    if (!is_geometric_mode_ && robot_continuous_state_.size() > 0) {
        robot_time_to_go = robot_continuous_state_(robot_continuous_state_.size() - 1);
    }

    // --- QUERY POINT CONSTRUCTION ---
    Eigen::VectorXd query_point = Eigen::VectorXd::Zero(kd_dim_);
    if (robot_continuous_state_.size() >= 2) {
        query_point(0) = robot_continuous_state_(0);
        query_point(1) = robot_continuous_state_(1);
    }
    if (kd_dim_ == 3) {
        query_point(2) = robot_time_to_go;
    } else if (kd_dim_ == 4) {
        query_point(2) = robot_continuous_state_(2); 
        query_point(3) = robot_time_to_go;
    } else if (kd_dim_ == 5) {
        query_point = robot_continuous_state_; 
    }

    // --- HYSTERESIS LOGIC ---
    const double hysteresis_factor = 0.98;
    double cost_of_current_anchor = std::numeric_limits<double>::infinity();
    
    if (start_node_ && start_node_->g != std::numeric_limits<double>::infinity()) {
        Trajectory bridge = statespace_->steer(robot_continuous_state_, start_node_->getStateValue());
        if (bridge.is_valid && obs_checker_->isTrajectorySafe(bridge, robot_time_to_go)) {
            last_replan_metrics_.obstacle_checks += obs_checker_->getObstaclesSize();
            cost_of_current_anchor = bridge.cost + start_node_->g;
        }
    }

    DStarLiteNode* best_candidate_node = nullptr;
    Trajectory best_candidate_bridge;
    double best_candidate_cost = std::numeric_limits<double>::infinity();
    
    // --- RADIUS EXPANSION LOGIC ---
    double current_search_radius = connection_radius_;
    const int max_attempts = 5;
    const double radius_multiplier = 2.0;

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        std::vector<size_t> candidate_indices;
        
        if (use_kdtree_) {
            candidate_indices = kdtree_->radiusSearch(query_point, current_search_radius);
        }

        for (size_t idx : candidate_indices) {
            DStarLiteNode* candidate = nodes_[idx].get();

            // Candidate must be reachable from Goal (Finite G)
            if (candidate->g == std::numeric_limits<double>::infinity()) {
                continue;
            }

            // Check Steering & Collision
            Trajectory bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());
            if (!bridge.is_valid) continue;

            last_replan_metrics_.obstacle_checks += obs_checker_->getObstaclesSize();
            if (!obs_checker_->isTrajectorySafe(bridge, robot_time_to_go)) continue;

            double total_cost = bridge.cost + candidate->g;

            if (total_cost < best_candidate_cost) {
                best_candidate_cost = total_cost;
                best_candidate_node = candidate;
                best_candidate_bridge = bridge;
                bridge_cost_ = bridge.cost;
            }
        }

        if (best_candidate_node) break;
        current_search_radius *= radius_multiplier;
    }

    // --- ASSIGNMENT ---
    if (best_candidate_node && best_candidate_cost < cost_of_current_anchor * hysteresis_factor) {
        // Update km (Key Modifier) for D* Lite if the start node changed
        if (start_node_ && start_node_ != best_candidate_node) {
            km_ += heuristic(start_node_, best_candidate_node);
        }
        
        start_node_ = best_candidate_node;
        last_replan_metrics_.path_cost = best_candidate_cost;
    } else if (start_node_ && cost_of_current_anchor != std::numeric_limits<double>::infinity()) {
        last_replan_metrics_.path_cost = cost_of_current_anchor;
    } else {
        start_node_ = nullptr;
        bridge_cost_ = std::numeric_limits<double>::infinity();
        last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
    }

    // INTERNAL DEBUG VISUALIZATION
    if (visualization_) {
        std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> debug_edges;
        if (start_node_) {
            Trajectory viz_bridge = statespace_->steer(robot_continuous_state_, start_node_->getStateValue());
            
            if (viz_bridge.path_points.size() >= 2) {
                for (size_t i = 0; i < viz_bridge.path_points.size() - 1; ++i) {
                    debug_edges.emplace_back(viz_bridge.path_points[i], viz_bridge.path_points[i+1]);
                }
            } else {
                debug_edges.emplace_back(robot_continuous_state_, start_node_->getStateValue());
            }
            visualization_->visualizeEdges(debug_edges, "map", "0.0,1.0,1.0", "debug_anchor_trajectory");
            
            std::vector<Eigen::VectorXd> anchor_pt = { start_node_->getStateValue().head<2>() };
            visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");
        } else {
            // CLEAR THE VISUALIZATION
            visualization_->visualizeEdges({}, "map", "0.0,0.0,0.0", "debug_anchor_trajectory");
            visualization_->visualizeNodes({}, "map", {0.0f, 0.0f, 0.0f}, "debug_anchor_point");
        }
    }

    // ANCHOR LOGGING (Manual Throttle)
    static auto last_log_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() > 1000) {
        if (start_node_) {
            // Hardcoding 35.0 based on your Thruster config file 'time_budget'
            double initial_budget = 35.0; 
            double forward_sim_time = initial_budget - robot_time_to_go;

            RCLCPP_WARN(
                rclcpp::get_logger("DStarLite_Anchor"), 
                "Anchor Connected: Node [%d] | Node Cost: %.2f | Total Path Cost: %.2f | T_Goal: %.2f | Sim_Time: %.2f", 
                start_node_->getIndex(), 
                start_node_->g, 
                last_replan_metrics_.path_cost,
                robot_time_to_go,
                forward_sim_time
            );
        } else {
            RCLCPP_WARN(
                rclcpp::get_logger("DStarLite_Anchor"), 
                "Anchor Status: NULL (Robot is lost or searching...)"
            );
        }
        last_log_time = now;
    }
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

    // =====================================================================
    // 1. FAST BFS: Find the mathematically valid, connected tree
    // =====================================================================
    std::unordered_set<DStarLiteNode*> connected_to_goal;
    if (goal_node_) {
        std::queue<DStarLiteNode*> q;
        connected_to_goal.insert(goal_node_);
        q.push(goal_node_);

        while (!q.empty()) {
            DStarLiteNode* cur = q.front();
            q.pop();

            // Look at all predecessors (nodes that point TO 'cur')
            for (const auto& [pred, edge_info] : cur->backward_neighbors_) {
                // If 'pred' claims 'cur' as its parent AND the edge is physically safe
                if (pred->best_parent_ == cur && edge_info.distance != std::numeric_limits<double>::infinity()) {
                    if (connected_to_goal.insert(pred).second) {
                        q.push(pred);
                    }
                }
            }
        }
    }
    // =====================================================================

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

        // Draw forward edges for D* Lite tree
        for (const auto& [neighbor, edge_info] : u->forward_neighbors_) {
            if (edge_info.distance == std::numeric_limits<double>::infinity()) continue;
            Eigen::Vector2d p1 = u->getStateValue().head<2>();
            Eigen::Vector2d p2 = neighbor->getStateValue().head<2>();

            // ONLY draw the D* Lite edge if the node is actually connected to the goal
            if (u->best_parent_ == neighbor && connected_to_goal.find(u) != connected_to_goal.end()) {
                dslite_tree_edges.emplace_back(p1, p2);
            } else {
                other_valid_edges.emplace_back(p1, p2);
            }
        }

        // Dijkstra: if visited, draw node and an edge to its stored parent/successor
        auto it = dijkstra_tree_parents_.find(u);
        if (it != dijkstra_tree_parents_.end()) {
            dijkstra_nodes_pos.push_back(pos);
            DStarLiteNode* parent = it->second; 
            if (parent) {
                Eigen::Vector2d p1 = u->getStateValue().head<2>();
                Eigen::Vector2d p2 = parent->getStateValue().head<2>();
                dijkstra_tree_edges.emplace_back(p1, p2);
            }
        }
    }
    // std::cout << "[Viz] total_nodes=" << nodes_.size()
    //           << " dslite_nodes=" << count_dslite_nodes
    //           << " dijkstra_nodes=" << dijkstra_nodes_pos.size()
    //           << " dslite_tree_edges=" << dslite_tree_edges.size()
    //           << " dijkstra_tree_edges=" << dijkstra_tree_edges.size() << std::endl;

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
        std::array<float,3>{1.0f, 1.0f, 1.0f},   // gray
        0.5f,                                   // alpha (slightly transparent)
        0.05f,                                   // thin line
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
    // if (!start_node_ || !goal_node_) return path;

    if (!start_node_ || start_node_->g == INFINITY) {
        DSTARLITE_ERROR("[DSTARLITE_Path_Assembly] Robot has no valid anchor node in the tree. Cannot build path.");
        return {}; // Return empty path
    }
    // else {
    //     DSTARLITE_INFO("[DSTARLITE_Path_Assembly] Robot has found a valid anchor node in the tree.");
    // }
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

        auto traj = current_node->best_parent_trajectory_;
        if (traj->is_valid && traj->path_points.size() > 1) {
            path.insert(path.end(), traj->path_points.begin() + 1, traj->path_points.end());
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



// ==============================================================================================
// STRATEGY 1: INVALIDATING SET (Edge-Level Pointer Caching)
// ==============================================================================================
#if USE_INVALIDATING_SET_STRATEGY

void KinodynamicPRMStarDStarLite::addNewObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    double search_radius = is_geometric_mode_ ? (obs_r + ob.inflation + connection_radius_) : (obs_r + ob.inflation + connection_radius_ + obs_r * (std::sqrt(2.0) - 1.0));

    std::unordered_set<int> unique_node_indices;
    for (const auto& point_3d : ob.predicted_path) {
        Eigen::VectorXd query(kd_dim_);
        if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
        else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
        else if (kd_dim_ == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
        else if (kd_dim_ == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
    }

    auto checkAndBlockEdge = [&](DStarLiteNode* node, DStarLiteNode* neighbor, EdgeInfo& edge, bool& u_needs_update) {
        const double edge_start_ttg = node->getTimeToGoal();
        last_replan_metrics_.obstacle_checks++;

        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), edge_start_ttg, ob)) {
            
            if (edge.distance != std::numeric_limits<double>::infinity()) {
                edge.distance = std::numeric_limits<double>::infinity();
                u_needs_update = true;
            }
            
            // Pointer insertion without duplicates (Forward Edge)
            if (std::find(edge.invalidating_obstacles.begin(), edge.invalidating_obstacles.end(), &ob) == edge.invalidating_obstacles.end()) {
                edge.invalidating_obstacles.push_back(&ob);
            }

            // // Symmetric Update (Backward Edge)
            // if (neighbor->backward_neighbors_.count(node)) {
            //     auto& inc_edge = neighbor->backward_neighbors_.at(node);
            //     inc_edge.distance = std::numeric_limits<double>::infinity();
            //     if (std::find(inc_edge.invalidating_obstacles.begin(), inc_edge.invalidating_obstacles.end(), &ob) == inc_edge.invalidating_obstacles.end()) {
            //         inc_edge.invalidating_obstacles.push_back(&ob);
            //     }
            // }

        if (neighbor->backward_neighbors_.count(node)) {
            auto& inc_edge = neighbor->backward_neighbors_.at(node);
            inc_edge.distance = std::numeric_limits<double>::infinity();
            // Correct Pointer Insertion
            if (std::find(inc_edge.invalidating_obstacles.begin(), inc_edge.invalidating_obstacles.end(), &ob) == inc_edge.invalidating_obstacles.end()) {
                inc_edge.invalidating_obstacles.push_back(&ob);
            }
        }

            // Geometric Mode Optimization (Break reverse path immediately)
            if (is_geometric_mode_ && neighbor->forward_neighbors_.count(node)) {
                auto& rev_edge = neighbor->forward_neighbors_.at(node);
                bool rev_changed = (rev_edge.distance != std::numeric_limits<double>::infinity());
                rev_edge.distance = std::numeric_limits<double>::infinity();
                
                if (std::find(rev_edge.invalidating_obstacles.begin(), rev_edge.invalidating_obstacles.end(), &ob) == rev_edge.invalidating_obstacles.end()) {
                    rev_edge.invalidating_obstacles.push_back(&ob);
                }

                if (node->backward_neighbors_.count(neighbor)) {
                    auto& rev_inc_edge = node->backward_neighbors_.at(neighbor);
                    rev_inc_edge.distance = std::numeric_limits<double>::infinity();
                    if (std::find(rev_inc_edge.invalidating_obstacles.begin(), rev_inc_edge.invalidating_obstacles.end(), &ob) == rev_inc_edge.invalidating_obstacles.end()) {
                        rev_inc_edge.invalidating_obstacles.push_back(&ob);
                    }
                }
                if (rev_changed) {
                    recomputeRHS(neighbor); 
                    updateVertex(neighbor);
                }
            }
        }
    };

    for (int idx : unique_node_indices) {
        DStarLiteNode* u = nodes_[idx].get();
        bool u_needs_update = false;
        
        for (auto& [neighbor, edge] : u->forward_neighbors_) {
            checkAndBlockEdge(u, neighbor, edge, u_needs_update);
        }
        
        if (u_needs_update) {
            recomputeRHS(u); 
            updateVertex(u);
        }
    }
}

void KinodynamicPRMStarDStarLite::removeObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    double search_radius = is_geometric_mode_ ? (obs_r + ob.inflation + connection_radius_) : (obs_r + ob.inflation + connection_radius_ + obs_r * (std::sqrt(2.0) - 1.0));

    std::unordered_set<int> unique_node_indices;
    for (const auto& point_3d : ob.predicted_path) {
        Eigen::VectorXd query(kd_dim_);
        if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
        else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
        else if (kd_dim_ == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
        else if (kd_dim_ == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
    }

    auto checkAndRestoreEdge = [&](DStarLiteNode* node, DStarLiteNode* neighbor, EdgeInfo& edge, bool& u_needs_update) {
        if (edge.distance == std::numeric_limits<double>::infinity()) {
            
            // O(1) Swap-and-Pop from edge invalidation list
            auto it = std::find(edge.invalidating_obstacles.begin(), edge.invalidating_obstacles.end(), &ob);
            if (it != edge.invalidating_obstacles.end()) {
                *it = edge.invalidating_obstacles.back();
                edge.invalidating_obstacles.pop_back();

                if (edge.invalidating_obstacles.empty()) {
                    edge.distance = edge.distance_original;
                    u_needs_update = true;

                    if (neighbor->backward_neighbors_.count(node)) {
                        auto& inc_edge = neighbor->backward_neighbors_.at(node);
                        inc_edge.distance = edge.distance_original;
                        auto inc_it = std::find(inc_edge.invalidating_obstacles.begin(), inc_edge.invalidating_obstacles.end(), &ob);
                        if (inc_it != inc_edge.invalidating_obstacles.end()) {
                            *inc_it = inc_edge.invalidating_obstacles.back();
                            inc_edge.invalidating_obstacles.pop_back();
                        }
                    }

                    if (is_geometric_mode_ && neighbor->forward_neighbors_.count(node)) {
                        auto& rev_edge = neighbor->forward_neighbors_.at(node);
                        auto rev_it = std::find(rev_edge.invalidating_obstacles.begin(), rev_edge.invalidating_obstacles.end(), &ob);
                        if (rev_it != rev_edge.invalidating_obstacles.end()) {
                            *rev_it = rev_edge.invalidating_obstacles.back();
                            rev_edge.invalidating_obstacles.pop_back();

                            if (rev_edge.invalidating_obstacles.empty()) {
                                rev_edge.distance = rev_edge.distance_original;
                                if (node->backward_neighbors_.count(neighbor)) {
                                    auto& rev_inc_edge = node->backward_neighbors_.at(neighbor);
                                    rev_inc_edge.distance = rev_edge.distance_original;
                                    auto rev_inc_it = std::find(rev_inc_edge.invalidating_obstacles.begin(), rev_inc_edge.invalidating_obstacles.end(), &ob);
                                    if (rev_inc_it != rev_inc_edge.invalidating_obstacles.end()) {
                                        *rev_inc_it = rev_inc_edge.invalidating_obstacles.back();
                                        rev_inc_edge.invalidating_obstacles.pop_back();
                                    }
                                }
                                recomputeRHS(neighbor);
                                updateVertex(neighbor);
                            }
                        }
                    }
                }
            }
        }
    };

    for (int idx : unique_node_indices) {
        DStarLiteNode* u = nodes_[idx].get();
        bool u_needs_update = false;
        
        for (auto& [neighbor, edge] : u->forward_neighbors_) {
            checkAndRestoreEdge(u, neighbor, edge, u_needs_update);
        }
        
        if (u_needs_update) {
            recomputeRHS(u);
            updateVertex(u);
        }
    }
}

// ==============================================================================================
// STRATEGY 2: THREAT SET (Node-Level Pointer Filtering)
// ==============================================================================================
#elif USE_THREAT_SET_STRATEGY

void KinodynamicPRMStarDStarLite::addNewObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    double search_radius = is_geometric_mode_ ? (obs_r + ob.inflation + connection_radius_) : (obs_r + ob.inflation + connection_radius_ + obs_r * (std::sqrt(2.0) - 1.0));

    std::unordered_set<int> unique_node_indices;
    for (const auto& point_3d : ob.predicted_path) {
        Eigen::VectorXd query(kd_dim_);
        if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
        else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
        else if (kd_dim_ == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
        else if (kd_dim_ == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
    }

    auto checkAndBlockEdge = [&](DStarLiteNode* node, DStarLiteNode* neighbor, EdgeInfo& edge, bool& u_needs_update) {
        if (edge.distance == std::numeric_limits<double>::infinity()) return; 

        const double edge_start_ttg = node->getTimeToGoal();
        last_replan_metrics_.obstacle_checks++;

        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), edge_start_ttg, ob)) {
            
            edge.distance = std::numeric_limits<double>::infinity();
            u_needs_update = true;

            if (neighbor->backward_neighbors_.count(node)) {
                neighbor->backward_neighbors_.at(node).distance = std::numeric_limits<double>::infinity();
            }

            if (is_geometric_mode_ && neighbor->forward_neighbors_.count(node)) {
                auto& rev_edge = neighbor->forward_neighbors_.at(node);
                bool rev_changed = false;
                if (rev_edge.distance != std::numeric_limits<double>::infinity()) {
                    rev_edge.distance = std::numeric_limits<double>::infinity();
                    rev_changed = true;
                }
                if (node->backward_neighbors_.count(neighbor)) {
                    node->backward_neighbors_.at(neighbor).distance = std::numeric_limits<double>::infinity();
                }
                if (rev_changed) {
                    recomputeRHS(neighbor); 
                    updateVertex(neighbor);
                }
            }
        }
    };

    for (int idx : unique_node_indices) {
        DStarLiteNode* u = nodes_[idx].get();
        bool u_needs_update = false;
        
        // 1. ADD POINTER THREAT TO NODE
        if (std::find(u->threats_.begin(), u->threats_.end(), &ob) == u->threats_.end()) {
            u->threats_.push_back(&ob);
        }
        
        // 2. CHECK EDGES
        for (auto& [neighbor, edge] : u->forward_neighbors_) {
            checkAndBlockEdge(u, neighbor, edge, u_needs_update);
        }
        
        if (u_needs_update) {
            recomputeRHS(u); 
            updateVertex(u);
        }
    }
}

void KinodynamicPRMStarDStarLite::removeObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    double search_radius = is_geometric_mode_ ? (obs_r + ob.inflation + connection_radius_) : (obs_r + ob.inflation + connection_radius_ + obs_r * (std::sqrt(2.0) - 1.0));

    std::unordered_set<int> unique_node_indices;
    for (const auto& point_3d : ob.predicted_path) {
        Eigen::VectorXd query(kd_dim_);
        if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
        else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
        else if (kd_dim_ == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
        else if (kd_dim_ == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
    }

    auto checkAndRestoreEdge = [&](DStarLiteNode* node, DStarLiteNode* neighbor, EdgeInfo& edge, bool& u_needs_update) {
        if (edge.distance == std::numeric_limits<double>::infinity()) {
            const double ttg = node->getTimeToGoal();
            bool is_safe = true;
            
            // Loop natively through pointers
            for (const Obstacle* threat_ptr : node->threats_) {
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ttg, *threat_ptr)) {
                    is_safe = false;
                    break;
                }
            }

            if (is_safe) {
                edge.distance = edge.distance_original;
                u_needs_update = true;
                
                if (neighbor->backward_neighbors_.count(node)) {
                    neighbor->backward_neighbors_.at(node).distance = edge.distance_original;
                }

                if (is_geometric_mode_ && neighbor->forward_neighbors_.count(node)) {
                    auto& rev_edge = neighbor->forward_neighbors_.at(node);
                    rev_edge.distance = rev_edge.distance_original;
                    if (node->backward_neighbors_.count(neighbor)) {
                        node->backward_neighbors_.at(neighbor).distance = rev_edge.distance_original;
                    }
                    recomputeRHS(neighbor);
                    updateVertex(neighbor);
                }
            }
        }
    };

    for (int idx : unique_node_indices) {
        DStarLiteNode* u = nodes_[idx].get();
        bool u_needs_update = false;
        
        // 1. O(1) SWAP-AND-POP THREAT REMOVAL
        auto it = std::find(u->threats_.begin(), u->threats_.end(), &ob);
        if (it != u->threats_.end()) {
            *it = u->threats_.back();
            u->threats_.pop_back();
        }
        
        // 2. RESTORE EDGES
        for (auto& [neighbor, edge] : u->forward_neighbors_) {
            checkAndRestoreEdge(u, neighbor, edge, u_needs_update);
        }
        
        if (u_needs_update) {
            recomputeRHS(u);
            updateVertex(u);
        }
    }
}

// ==============================================================================================
// STRATEGY 3: DEFAULT (Brute-Force Fallback)
// ==============================================================================================
// #else

// void KinodynamicPRMStarDStarLite::addNewObstacle(const Obstacle& ob) {
//     if (ob.predicted_path.empty()) return;

//     double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
//     double search_radius = is_geometric_mode_ ? (obs_r + ob.inflation + connection_radius_) : (obs_r + ob.inflation + connection_radius_ + obs_r * (std::sqrt(2.0) - 1.0));

//     std::unordered_set<int> unique_node_indices;
//     for (const auto& point_3d : ob.predicted_path) {
//         Eigen::VectorXd query(kd_dim_);
//         if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
//         else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
//         else if (kd_dim_ == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
//         else if (kd_dim_ == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
//         std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
//         for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
//     }

//     auto checkAndBlockEdge = [&](DStarLiteNode* node, DStarLiteNode* neighbor, EdgeInfo& edge, bool& u_needs_update) {
//         if (edge.distance == std::numeric_limits<double>::infinity()) return; 

//         const double edge_start_ttg = node->getTimeToGoal();
//         last_replan_metrics_.obstacle_checks++;

//         if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), edge_start_ttg, ob)) {
//             edge.distance = std::numeric_limits<double>::infinity();
//             u_needs_update = true;

//             if (neighbor->backward_neighbors_.count(node)) {
//                 neighbor->backward_neighbors_.at(node).distance = std::numeric_limits<double>::infinity();
//             }

//             if (is_geometric_mode_ && neighbor->forward_neighbors_.count(node)) {
//                 auto& rev_edge = neighbor->forward_neighbors_.at(node);
//                 bool rev_changed = false;
//                 if (rev_edge.distance != std::numeric_limits<double>::infinity()) {
//                     rev_edge.distance = std::numeric_limits<double>::infinity();
//                     rev_changed = true;
//                 }
//                 if (node->backward_neighbors_.count(neighbor)) {
//                     node->backward_neighbors_.at(neighbor).distance = std::numeric_limits<double>::infinity();
//                 }
//                 if (rev_changed) {
//                     recomputeRHS(neighbor); 
//                     updateVertex(neighbor);
//                 }
//             }
//         }
//     };

//     for (int idx : unique_node_indices) {
//         DStarLiteNode* u = nodes_[idx].get();
//         bool u_needs_update = false;
        
//         for (auto& [neighbor, edge] : u->forward_neighbors_) {
//             checkAndBlockEdge(u, neighbor, edge, u_needs_update);
//         }
        
//         if (u_needs_update) {
//             recomputeRHS(u); 
//             updateVertex(u);
//         }
//     }
// }

// void KinodynamicPRMStarDStarLite::removeObstacle(const Obstacle& ob) {
//     if (ob.predicted_path.empty()) return;

//     double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
//     double search_radius = is_geometric_mode_ ? (obs_r + ob.inflation + connection_radius_) : (obs_r + ob.inflation + connection_radius_ + obs_r * (std::sqrt(2.0) - 1.0));

//     std::unordered_set<int> unique_node_indices;
//     for (const auto& point_3d : ob.predicted_path) {
//         Eigen::VectorXd query(kd_dim_);
//         if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
//         else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
//         else if (kd_dim_ == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
//         else if (kd_dim_ == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
//         std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
//         for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
//     }

//     const ObstacleVector& all_obstacles = obs_checker_->getObstacles();

//     auto checkAndRestoreEdge = [&](DStarLiteNode* node, DStarLiteNode* neighbor, EdgeInfo& edge, bool& u_needs_update) {
//         if (edge.distance == std::numeric_limits<double>::infinity()) {
//             const double ttg = node->getTimeToGoal();
//             bool should_restore = false;
            
//             if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ttg, ob)) {
//                 bool conflicts_with_other = false;
//                 for (const auto& other_ob : all_obstacles) {
//                     if (other_ob.name == ob.name) continue; 
//                     last_replan_metrics_.obstacle_checks++;
//                     if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ttg, other_ob)) {
//                         conflicts_with_other = true;
//                         break; 
//                     }
//                 }
//                 if (!conflicts_with_other) {
//                     should_restore = true;
//                 }
//             }

//             if (should_restore) {
//                 edge.distance = edge.distance_original;
//                 u_needs_update = true;
                
//                 if (neighbor->backward_neighbors_.count(node)) {
//                     neighbor->backward_neighbors_.at(node).distance = edge.distance_original;
//                 }

//                 if (is_geometric_mode_ && neighbor->forward_neighbors_.count(node)) {
//                     auto& rev_edge = neighbor->forward_neighbors_.at(node);
//                     rev_edge.distance = rev_edge.distance_original;
//                     if (node->backward_neighbors_.count(neighbor)) {
//                         node->backward_neighbors_.at(neighbor).distance = rev_edge.distance_original;
//                     }
//                     recomputeRHS(neighbor);
//                     updateVertex(neighbor);
//                 }
//             }
//         }
//     };

//     for (int idx : unique_node_indices) {
//         DStarLiteNode* u = nodes_[idx].get();
//         bool u_needs_update = false;
        
//         for (auto& [neighbor, edge] : u->forward_neighbors_) {
//             checkAndRestoreEdge(u, neighbor, edge, u_needs_update);
//         }
        
//         if (u_needs_update) {
//             recomputeRHS(u);
//             updateVertex(u);
//         }
//     }
// }

// #endif



#else

void KinodynamicPRMStarDStarLite::addNewObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    double search_radius = is_geometric_mode_ ? (obs_r + ob.inflation + connection_radius_) : (obs_r + ob.inflation + connection_radius_ + obs_r * (std::sqrt(2.0) - 1.0));

    std::unordered_set<int> unique_node_indices;
    for (const auto& point_3d : ob.predicted_path) {
        Eigen::VectorXd query(kd_dim_);
        if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
        else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
        else if (kd_dim_ == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
        else if (kd_dim_ == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
    }

    auto checkAndBlockEdge = [&](DStarLiteNode* u, DStarLiteNode* v, EdgeInfo& edge, bool& u_needs_update) {
        if (edge.distance == INFINITY) return; // already blocked

        const double edge_start_ttg = u->getTimeToGoal();
        last_replan_metrics_.obstacle_checks++;

        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), edge_start_ttg, ob)) {
            // OLD and NEW
            double c_old = edge.distance;
            double c_new = INFINITY;

            // set forward cost to INF
            edge.distance = c_new;
            u_needs_update = true;

            // mirror to backward if stored
            if (v->backward_neighbors_.count(u)) {
                v->backward_neighbors_.at(u).distance = c_new;
            }

            // ---- optimized D* Lite logic for increased cost ----
            // If u was using this edge as its best (rhs contribution), recompute from scratch.
            if (u->getParent() == v) {
                if (u != goal_node_) {
                    recomputeRHS(u);
                }
            }
            // ----------------------------------------------------

            // handle geometric reverse edge if present (mirror)
            if (is_geometric_mode_ && v->forward_neighbors_.count(u)) {
                auto& rev_edge = v->forward_neighbors_.at(u);
                double rev_old = rev_edge.distance;
                if (rev_old != INFINITY) {
                    rev_edge.distance = INFINITY;
                    if (u->backward_neighbors_.count(v)) {
                        u->backward_neighbors_.at(v).distance = INFINITY;
                    }
                    // // If reverse edge existed and was the best for v, we must recompute v too
                    // if (v != goal_node_ && std::isfinite(rev_old) && u->g != INFINITY) {
                    //     double rev_old_contrib = rev_old + u->g;
                    //     if (std::abs(v->rhs - rev_old_contrib) <= 1e-9) {
                    //         recomputeRHS(v);
                    //     }
                    // }
                    if (v->getParent() == u && v != goal_node_) {
                        recomputeRHS(v);
                    }

                    
                    // push neighbor v into queue if needed
                    updateVertex(v);
                }
            }
        }
    };

    for (int idx : unique_node_indices) {
        DStarLiteNode* u = nodes_[idx].get();
        bool u_needs_update = false;
        
        for (auto& [neighbor, edge] : u->forward_neighbors_) {
            checkAndBlockEdge(u, neighbor, edge, u_needs_update);
        }
        
        if (u_needs_update) {
            // recomputeRHS(u); 
            updateVertex(u);
        }
    }
}

void KinodynamicPRMStarDStarLite::removeObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    double search_radius = is_geometric_mode_ ? (obs_r + ob.inflation + connection_radius_) : (obs_r + ob.inflation + connection_radius_ + obs_r * (std::sqrt(2.0) - 1.0));

    std::unordered_set<int> unique_node_indices;
    for (const auto& point_3d : ob.predicted_path) {
        Eigen::VectorXd query(kd_dim_);
        if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
        else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
        else if (kd_dim_ == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
        else if (kd_dim_ == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
    }

    const ObstacleVector& all_obstacles = obs_checker_->getObstacles();

    auto checkAndRestoreEdge = [&](DStarLiteNode* u, DStarLiteNode* v, EdgeInfo& edge, bool& u_needs_update) {
        if (edge.distance != INFINITY) return; // already available

        const double ttg = u->getTimeToGoal();
        bool should_restore = false;

        // Check if obstacle blocked it and whether other obstacles still block it
        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ttg, ob)) {
            bool conflicts_with_other = false;
            for (const auto& other_ob : all_obstacles) {
                if (other_ob.name == ob.name) continue;
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ttg, other_ob)) {
                    conflicts_with_other = true;
                    break;
                }
            }
            if (!conflicts_with_other) should_restore = true;
        }

        if (should_restore) {
            double c_old = edge.distance;                 // INF
            double c_new = edge.distance_original;        // restored finite cost

            // restore forward edge
            edge.distance = c_new;
            u_needs_update = true;

            // restore backward mirror if present
            if (v->backward_neighbors_.count(u)) {
                v->backward_neighbors_.at(u).distance = c_new;
            }

            // ---- optimized D* Lite logic for decreased cost ----
            // Only tighten rhs(u) if this gives a smaller rhs (and successor has finite g)
            if (v->g != INFINITY && c_new != INFINITY) {
                double candidate = c_new + v->g;
                if (candidate + 1e-9 < u->rhs) {
                    u->rhs = candidate;
                    u->setBestParent(v, edge.cached_trajectory);
                }
            }
            // ----------------------------------------------------

            // handle geometric reverse edge restoration
            if (is_geometric_mode_ && v->forward_neighbors_.count(u)) {
                auto& rev_edge = v->forward_neighbors_.at(u);
                double rev_old = rev_edge.distance;
                double rev_new = rev_edge.distance_original;
                rev_edge.distance = rev_new;
                if (u->backward_neighbors_.count(v)) {
                    u->backward_neighbors_.at(v).distance = rev_new;
                }

                // tighten v's rhs if applicable
                if (u->g != INFINITY && rev_new != INFINITY) {
                    double candidate_rev = rev_new + u->g;
                    if (candidate_rev + 1e-9 < v->rhs) {
                        v->rhs = candidate_rev;
                        v->setBestParent(u, rev_edge.cached_trajectory);
                    }
                }
                updateVertex(v);
            }
        }
    };

    for (int idx : unique_node_indices) {
        DStarLiteNode* u = nodes_[idx].get();
        bool u_needs_update = false;
        
        for (auto& [neighbor, edge] : u->forward_neighbors_) {
            checkAndRestoreEdge(u, neighbor, edge, u_needs_update);
        }
        
        if (u_needs_update) {
            // recomputeRHS(u);
            updateVertex(u);
        }
    }
}

#endif