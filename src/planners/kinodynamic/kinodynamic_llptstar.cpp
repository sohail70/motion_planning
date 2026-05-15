#include "motion_planning/planners/kinodynamic/kinodynamic_prmstar_dstarlite.hpp"

#define DEBUG_WITH_DIJKSTRA_ 0
#define USE_INVALIDATING_SET_STRATEGY 0
#define USE_THREAT_SET_STRATEGY 0
#define USE_GRID_SAMPLING 0
#define USE_RECOVERY 0 // Emergency Fallback
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

void KinodynamicPRMStarDStarLite::injectTimePillarNodes(const Eigen::VectorXd& goal_state_val, int num_pillar_nodes) {
    double max_time = upper_bounds_(statespace_->getDimension() - 1); 

    for (int i = 1; i <= num_pillar_nodes; ++i) {
        Eigen::VectorXd pillar_state = goal_state_val;

        // Set velocities to ZERO so the robot can safely stop at the goal.
        // Safety check to ensure we only apply this to states with velocity dimensions
        if (pillar_state.size() >= 4) {
            pillar_state(2) = 0.0;
            pillar_state(3) = 0.0;
        }

        // Distribute evenly across time
        double t_val = (max_time / num_pillar_nodes) * i; 
        pillar_state(statespace_->getDimension() - 1) = t_val;

        auto state_ptr = statespace_->addState(pillar_state);
        auto node = std::make_unique<DStarLiteNode>(state_ptr, nodes_.size());
        node->setTimeToGoal(t_val);
        node->rhs = 0;
        time_pillar_indices_.insert(node->getIndex());
        nodes_.push_back(std::move(node));
    }
}


void KinodynamicPRMStarDStarLite::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    std::cout << "------------------------------------------------------------\n";
    auto start_time = std::chrono::high_resolution_clock::now();
    visualization_ = visualization;
    
    num_samples_ = params.getParam<int>("num_of_samples");
    kd_dim_ = params.getParam<int>("kd_dim", 2);
    partial_update = params.getParam<bool>("partial_update");
    std::string kdtree_type = params.getParam<std::string>("kdtree_type");
    use_knn_ = params.getParam<bool>("use_knn", false);
    factor_ = params.getParam<double>("factor", 1.0);
    is_geometric_mode_ = params.getParam<bool>("is_geometric_mode", false);
    bool use_grid_sampling = false;
    num_pillar_nodes_ = params.getParam<int>("num_pillar_nodes", 50);
    if (is_geometric_mode_) num_pillar_nodes_ = 0;
    use_heuristic = params.getParam<bool>("heuristic", true);
#if USE_GRID_SAMPLING
    use_grid_sampling = true;
#endif
    lower_bounds_ = problem_def_->getLowerBound();
    upper_bounds_ = problem_def_->getUpperBound();

    if (kdtree_type == "NanoFlann") {
        Eigen::VectorXd weights(kd_dim_);
        switch (kd_dim_) {
            case 2: weights << 1.0, 1.0; break;
            case 3: weights << 1.0, 1.0, 1.0; break;
            case 4: weights << 1.0, 1.0, 1.0, 1.0; break;
            case 5: weights << 1.0, 1.0, 1.0, 1.0, 1.0; break;
            default: throw std::runtime_error("Unsupported k-d tree dimension");
        }
        kdtree_ = std::make_shared<WeightedNanoFlann>(kd_dim_, weights);
    } else if (kdtree_type == "LieKDTree") {
        kdtree_ = std::make_unique<LieSplittingKDTree>(statespace_->getDimension(), statespace_);
    } else {
        throw std::runtime_error("PRM* D* Lite requires a KD-Tree.");
    }

    const Eigen::VectorXd& start_state_val = problem_def_->getGoal();
    const Eigen::VectorXd& goal_state_val = problem_def_->getStart();



    bool use_rrtx_saved_samples_ = false;
    if (use_rrtx_saved_samples_) {
        std::cout << "Using RRTX saved samples." << std::endl;
        std::string filepath = "/home/sohail/motion_planning/build/rrtx_tree_nodes.csv";
        std::cout << "Loading nodes from file: " << filepath << "\n";
        std::ifstream fin(filepath);
        if (!fin.is_open()) {
            throw std::runtime_error("Failed to open node file: " + filepath);
        }

        std::string line;
        std::getline(fin, line);

        std::string cell;
        while (std::getline(fin, line)) {
            std::stringstream lineStream(line);
            std::vector<double> state_values;
            
            std::getline(lineStream, cell, ',');
            for(int i = 0; i < statespace_->getDimension(); ++i) {
                std::getline(lineStream, cell, ',');
                state_values.push_back(std::stod(cell));
            }

            Eigen::Map<Eigen::VectorXd> state_vec(state_values.data(), state_values.size());
            auto state_ptr = statespace_->addState(state_vec);
            auto node = std::make_unique<DStarLiteNode>(state_ptr, nodes_.size());
            
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

        start_node_ = best_search_root;
        start_node_->setTimeToGoal(0.0); 

        goal_node_ = best_robot_node; // TTG is already parsed from the CSV correctly

        std::cout << "Successfully identified start and goal nodes." << "\n";

    }
    else if (use_grid_sampling) {
        std::cout << "Using GRID sampling strategy." << std::endl;
        
        int samples_per_dim = static_cast<int>(std::pow(num_samples_, 1.0 / kd_dim_));
        if (samples_per_dim < 2) samples_per_dim = 2;

        grid_dim_per_side_ = samples_per_dim;
        use_grid_sampling_ = true;
        // -----------------------------------------

        std::cout << "Grid dimension: " << samples_per_dim << "x" << samples_per_dim << std::endl;

        Eigen::VectorXd step_size = (upper_bounds_ - lower_bounds_) / (samples_per_dim - 1);

        std::vector<int> indices(kd_dim_, 0);
        int count = 0;
        
        int total_grid_points = static_cast<int>(std::pow(samples_per_dim, kd_dim_));
        
        for (int i = 0; i < total_grid_points; ++i) {
            Eigen::VectorXd state_val(kd_dim_);
            
            for (int d = 0; d < kd_dim_; ++d) {
                state_val(d) = lower_bounds_(d) + indices[d] * step_size(d);
            }

            auto state_ptr = statespace_->addState(state_val);
            auto node = std::make_unique<DStarLiteNode>(state_ptr, nodes_.size());
            
            if (!is_geometric_mode_) {
                double absolute_t = node->getStateValue().tail<1>()[0];
                node->setTimeToGoal(absolute_t);
            } else {
                node->setTimeToGoal(0.0);
            }
            nodes_.push_back(std::move(node));

            for (int d = 0; d < kd_dim_; ++d) {
                indices[d]++;
                if (indices[d] < samples_per_dim) {
                    break;
                } else {
                    indices[d] = 0;
                }
            }
        }
        
        goal_node_ = nodes_[0].get();
        goal_node_->setTimeToGoal(0);
        std::cout << "Grid Goal Node ID: " << goal_node_->getIndex() 
                  << " at " << goal_node_->getStateValue().transpose() << std::endl;

        start_node_ = nodes_[nodes_.size() - 1].get();
        start_node_->setTimeToGoal(0);
        std::cout << "Grid Start Node ID: " << start_node_->getIndex() 
                  << " at " << start_node_->getStateValue().transpose() << std::endl;

    } else {
        std::cout << "Using UNIFORM random sampling strategy with Goal Time Pillar." << std::endl;
        use_grid_sampling_ = false;
        
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
        start_node->setTimeToGoal(std::numeric_limits<double>::infinity());
        nodes_.push_back(std::move(start_node));

        time_pillar_indices_.insert(goal_node_->getIndex());
        if (!is_geometric_mode_) injectTimePillarNodes(goal_state_val, num_pillar_nodes_);

        // Generate the remaining uniform samples normally
        int remaining_samples = num_samples_ - 2 - num_pillar_nodes_;
        for (int i = 0; i < remaining_samples; ++i) {
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



    Eigen::MatrixXd all_samples = statespace_->getSamplesCopy();
    Eigen::MatrixXd spatial_samples_only = all_samples.leftCols(kd_dim_).eval();
    // kdtree_->addPoints(spatial_samples_only);
    kdtree_->addPoints(all_samples);
    kdtree_->buildTree();


    int d = statespace_->getDimension();
    if (use_knn_) {
        double k0_fmt_star_practical = std::pow(2.0, d) * (M_E / d);
        k_neighbors_ = static_cast<int>(std::ceil(factor_ * k0_fmt_star_practical * std::log(statespace_->getNumStates())));
    } else {
        Eigen::VectorXd range = upper_bounds_ - lower_bounds_;
        mu_ = range.prod();
        zetaD_ = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
        /*
            Mind that PRM* doenst have the 1/d exponent (Karaman paper) for the "2" 
            But in FMT* paper they mention the prm* rn can be lower (Remark 4.5). 
            So I use RRT*'s rn which is in between FMT* and PRM*. On top of this we use 
            a factor anyway so no worries!
        */
        gamma_ = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu_ / zetaD_, 1.0 / d); 
        connection_radius_ = factor_ * gamma_ * std::pow(std::log(statespace_->getNumStates()) / statespace_->getNumStates(), 1.0 / d);
        std::cout << "Computed value of rn: " << connection_radius_ << std::endl;
        std::cout<<"factor: "<<factor_<<"\n";
    }
    std::cout << "Forcing neighbor caching for all " << nodes_.size() << " nodes..." << std::endl;
    for (size_t i = 0; i < nodes_.size(); ++i) {
        near(i);
    }

    checkIsolatedNodes(); // Since we presample, some nodes might not get to have any neighbors in non geometric tests. This is just a report

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "------------------------------------------------------------\n";
}

void KinodynamicPRMStarDStarLite::near(int node_index) {
    auto node = nodes_[node_index].get();
    if (node->neighbors_cached_) return;

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
                    std::shared_ptr<Trajectory> shared_traj_forward;
                    
                    if (traj_forward.is_valid && traj_forward.cost <= connection_radius_ + std::numeric_limits<double>::epsilon()) {
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
                            info_backward.cached_trajectory = shared_traj_forward;
                            info_backward.is_trajectory_computed = true;
                            
                            neighbor->forward_neighbors_[node] = info_backward;
                            node->backward_neighbors_[neighbor] = info_backward;
                        }
                    } else {
                        Trajectory traj_backward = statespace_->steer(neighbor->getStateValue(), node->getStateValue());
                        if (traj_backward.is_valid && traj_backward.cost <= connection_radius_ + std::numeric_limits<double>::epsilon()) {
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
    else {
        std::vector<size_t> candidate_indices;
        if (use_knn_) {
            if (k_neighbors_ > 0) {
                candidate_indices = kdtree_->knnSearch(node->getStateValue().head(kd_dim_), k_neighbors_);
            }
        } else {
            if (connection_radius_ > 0) {
                candidate_indices = kdtree_->radiusSearch(node->getStateValue().head(kd_dim_), connection_radius_ + std::numeric_limits<double>::epsilon());
            }
        }

        for (int idx : candidate_indices) {
            if (idx == node_index) continue;
            DStarLiteNode* neighbor = nodes_[idx].get();

            // Test FORWARD connection
            Trajectory traj_forward = statespace_->steer(node->getStateValue(), neighbor->getStateValue());
            std::shared_ptr<Trajectory> shared_traj_forward;
            
            if (traj_forward.is_valid && traj_forward.cost <= connection_radius_ + std::numeric_limits<double>::epsilon()) {
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
                    info_backward.cached_trajectory = shared_traj_forward;
                    info_backward.is_trajectory_computed = true;
                    
                    neighbor->forward_neighbors_[node] = info_backward;
                    node->backward_neighbors_[neighbor] = info_backward;
                }
            } else {
                Trajectory traj_backward = statespace_->steer(neighbor->getStateValue(), node->getStateValue());
                if (traj_backward.is_valid && traj_backward.cost <= connection_radius_ + std::numeric_limits<double>::epsilon()) {
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


void KinodynamicPRMStarDStarLite::checkIsolatedNodes() {
    // std::vector<int> isolated_indices;

    std::cout << "Checking " << nodes_.size() << " nodes for isolation (zero neighbors)..." << std::endl;

    for (size_t i = 0; i < nodes_.size(); ++i) {
        const auto* node = nodes_[i].get();
        
        if (node->forward_neighbors_.empty() && node->backward_neighbors_.empty()) {
            isolated_nodes_count_++;
            // isolated_indices.push_back(i);
        }
    }

    if (isolated_nodes_count_ > 0) {
        std::cout << "Found " << isolated_nodes_count_ 
                  << " isolated nodes (nodes with 0 neighbors) out of " 
                  << nodes_.size() << " total nodes." << std::endl;
        
        /*
        std::cout << "Isolated node indices: ";
        for (int idx : isolated_indices) {
            std::cout << idx << " ";
        }
        std::cout << std::endl;
        */
    } else {
        std::cout << "All nodes are connected to at least one neighbor!" << std::endl;
    }
}

void KinodynamicPRMStarDStarLite::extendSearchGraph() {
    // Line 1: vrand <- RandomSample()
    Eigen::VectorXd sample = Eigen::VectorXd::Random(kd_dim_);
    sample = (lower_bounds_.head(kd_dim_).array() + 
             (upper_bounds_.head(kd_dim_) - lower_bounds_.head(kd_dim_)).array() * 
             ((sample.array() + 1.0) / 2.0)).matrix();

    // Line 2: vnearest <- Nearest(vrand)
    std::vector<size_t> nearest_indices = kdtree_->knnSearch(sample, 1);
    if (nearest_indices.empty()) return;
    DStarLiteNode* v_nearest = nodes_[nearest_indices[0]].get();

    // Line 3: vnew <- Steer(vnearest, vrand, delta)
    double delta = 1.0; // Saturate step size
    Eigen::VectorXd new_state_val = saturate(sample, v_nearest->getStateValue(), delta);

    // Line 4: if vnew is in the free configuration space then
    // Eager check for the node, lazy check for the edges
    if (!obs_checker_->isObstacleFree(new_state_val)) {
        return; 
    }

    // Line 5: V <- V U {vnew}
    auto state_ptr = statespace_->addState(new_state_val);
    auto new_node_ptr = std::make_unique<DStarLiteNode>(state_ptr, nodes_.size());
    
    // FIX: Push safely BEFORE extracting the raw pointer to use downstream!
    nodes_.push_back(std::move(new_node_ptr));
    DStarLiteNode* v_new = nodes_.back().get();

    // Setup node costs
    v_new->g = std::numeric_limits<double>::infinity();
    v_new->rhs = std::numeric_limits<double>::infinity();

    if (!is_geometric_mode_) {
        double absolute_t = v_new->getStateValue().tail<1>()[0];
        v_new->setTimeToGoal(absolute_t);
    } else {
        v_new->setTimeToGoal(0.0);
    }

    // Add to KD-Tree
    Eigen::MatrixXd new_point(1, kd_dim_);
    new_point.row(0) = v_new->getStateValue().head(kd_dim_);
    kdtree_->addPoints(new_point); 

    // Calculate dynamic connection radius r (LLPT Eq. 4)
    double current_r = connection_radius_ * std::pow(std::log(nodes_.size()) / nodes_.size(), 1.0 / kd_dim_);
    std::vector<size_t> neighbor_indices = kdtree_->radiusSearch(v_new->getStateValue().head(kd_dim_), current_r);

    // Line 6: for u in N(vnew) do
    for (size_t idx : neighbor_indices) {
        DStarLiteNode* u = nodes_[idx].get();
        if (u == v_new) continue;

        // Line 7: if free-space local trajectory exists
        // LLPT Lazy logic: Kinematic Steer must succeed, but collision is NOT checked
        
        // --- FORWARD EDGE (v_new -> u) ---
        Trajectory traj_fwd = statespace_->steer(v_new->getStateValue(), u->getStateValue());
        if (traj_fwd.is_valid && traj_fwd.cost <= current_r + 1e-9) {
            
            // Line 8: E <- E U { e(vnew, u) }
            auto shared_fwd = std::make_shared<Trajectory>(std::move(traj_fwd));
            EdgeInfo info_fwd;
            info_fwd.distance = shared_fwd->cost;
            info_fwd.distance_original = shared_fwd->cost;
            info_fwd.cached_trajectory = shared_fwd;
            info_fwd.is_trajectory_computed = true;
            info_fwd.is_evaluated = false; // LLPT Lazy Flag

            v_new->forward_neighbors_[u] = info_fwd;
            u->backward_neighbors_[v_new] = info_fwd;

            // Line 9: UpdateNode(vnew, u)
            updateNode(v_new, u);
        }

        // --- BACKWARD EDGE (u -> v_new) ---
        Trajectory traj_bwd;
        if (is_geometric_mode_ && v_new->forward_neighbors_.count(u)) {
            traj_bwd = *(v_new->forward_neighbors_.at(u).cached_trajectory); // reverse mirror
        } else {
            traj_bwd = statespace_->steer(u->getStateValue(), v_new->getStateValue());
        }

        if (traj_bwd.is_valid && traj_bwd.cost <= current_r + 1e-9) {
            // Line 8: E <- E U { e(u, vnew) }
            auto shared_bwd = std::make_shared<Trajectory>(std::move(traj_bwd));
            EdgeInfo info_bwd;
            info_bwd.distance = shared_bwd->cost;
            info_bwd.distance_original = shared_bwd->cost;
            info_bwd.cached_trajectory = shared_bwd;
            info_bwd.is_trajectory_computed = true;
            info_bwd.is_evaluated = false; // LLPT Lazy Flag

            u->forward_neighbors_[v_new] = info_bwd;
            v_new->backward_neighbors_[u] = info_bwd;

            // Line 9: UpdateNode(u, vnew)
            updateNode(u, v_new);
        }
    }
}
void KinodynamicPRMStarDStarLite::updateNode(DStarLiteNode* v, DStarLiteNode* u) {
    // LLPT Algorithm 5: UpdateNode(v, u)
    if (!v->forward_neighbors_.count(u)) return;
    
    double edge_cost = v->forward_neighbors_.at(u).distance;
    if (edge_cost == std::numeric_limits<double>::infinity() || u->rhs == std::numeric_limits<double>::infinity()) {
        return;
    }

    // Line 1: if lmc(v) > w_bar(v,u) + lmc(u)
    double candidate_lmc = edge_cost + u->rhs;
    if (v->rhs > candidate_lmc + 1e-9) {
        // Line 2: MakeParentOf(v, u)
        v->setBestParent(u, v->forward_neighbors_.at(u).cached_trajectory);
        // Line 3: lmc(v) = w_bar(v,u) + lmc(u)
        v->rhs = candidate_lmc;
        // Line 4: Q.update(v)
        updateVertex(v); 
    }
}

void KinodynamicPRMStarDStarLite::plan() {
    if (!start_node_ || !goal_node_) return;
    
    // 1. Ensure the path is currently safe (in case updateObstacles wasn't called)
    resolvePathLazy();
    
    // 2. LLPT Graph Densification (Algorithm 6)
    // Add a batch of new samples to improve the optimal path
    int samples_to_add = 50; // Tune this to fit within your slice_time
    for (int i = 0; i < samples_to_add; ++i) {
        extendSearchGraph(); 
    }
    
    // 3. If extendSearchGraph added new shortcuts to the queue, resolve them!
    resolvePathLazy();
}
// double KinodynamicPRMStarDStarLite::heuristic(DStarLiteNode* a, DStarLiteNode* b) {
// #if USE_HEURISTIC
//     const Eigen::VectorXd& val_a = a->getStateValue();
//     const Eigen::VectorXd& val_b = b->getStateValue();
    
//     // Spatial distance (Common to all state spaces)
//     // All spaces have x at index 0 and y at index 1.
//     double dx = val_a(0) - val_b(0);
//     double dy = val_a(1) - val_b(1);
//     double spatial_dist_sq = dx * dx + dy * dy;

//     // State Space Switching based on KD-Tree Dimension
//     if (kd_dim_ == 2) {
//         // R2 Space: [x, y]
//         // Pure geometric distance.
//         return std::sqrt(spatial_dist_sq);
//     } 
//     else if (kd_dim_ == 3) {
//         // R2T Space: [x, y, time]
//         double dt = val_a(2) - val_b(2);
//         double c_t = 1.0; 
        
//         return std::sqrt(spatial_dist_sq + std::pow(c_t * dt, 2));
//         // return std::sqrt(spatial_dist_sq);
//     } 
//     else if (kd_dim_ == 4) {
//         // Dubins Time Space: [x, y, theta, time]
//         // Theta is at index 2, Time is at index 3.
//         // We intentionally IGNORE theta (index 2) to prevent overestimating!
//         double dt = val_a(3) - val_b(3);
//         double c_t = 1.0; 
        
//         return std::sqrt(spatial_dist_sq + std::pow(c_t * dt, 2));
//     } 
//     else if (kd_dim_ == 5) {
//         // Thruster Space: [x, y, vx, vy, time]
//         // Velocities are at indices 2 and 3, Time is at index 4.
//         double dvx = val_a(2) - val_b(2);
//         double dvy = val_a(3) - val_b(3);
//         double dv_dist_sq = dvx * dvx + dvy * dvy;
//         double dt = val_a(4) - val_b(4);
//         // Weights MUST match the ones in your ThrusterSteerStateSpace::steer function
//         double c_x = 1.0;
//         double c_v = 1.0;
//         double c_t = 1.0;
//         return std::sqrt((c_x * c_x * spatial_dist_sq) + 
//                          (c_v * c_v * dv_dist_sq) + 
//                          (c_t * c_t * dt * dt));
//     }
    
//     // Fallback
//     return 0.0;
// #else
//     return 0.0;
// #endif
// }

double KinodynamicPRMStarDStarLite::heuristic(DStarLiteNode* a, DStarLiteNode* b) {
    if (use_heuristic){
        if (!a || !b) return 0.0;

        const Eigen::VectorXd& val_a = a->getStateValue();
        const Eigen::VectorXd& val_b = b->getStateValue();
        
        // Spatial distance (Common to all state spaces)
        double dx = val_a(0) - val_b(0);
        double dy = val_a(1) - val_b(1);
        double spatial_dist_sq = dx * dx + dy * dy;

        double h_val = 0.0;

        // State Space Switching based on KD-Tree Dimension
        if (kd_dim_ == 2) {
            h_val = std::sqrt(spatial_dist_sq);
        } 
        else if (kd_dim_ == 3 || kd_dim_ == 4) {
            // R2T and Dubins: dt is DROPPED to preserve Triangle Inequality 
            // with the zero-cost Time Pillar edges!
            h_val = std::sqrt(spatial_dist_sq);
        } 
        else if (kd_dim_ == 5) {
            // Thruster Space: dt is DROPPED!
            double dvx = val_a(2) - val_b(2);
            double dvy = val_a(3) - val_b(3);
            double dv_dist_sq = dvx * dvx + dvy * dvy;
            
            double c_x = 1.0;
            double c_v = 1.0;
            h_val = std::sqrt((c_x * c_x * spatial_dist_sq) + (c_v * c_v * dv_dist_sq));
        }
        
        // // =========================================================
        // // THE KINODYNAMIC TIE-BREAKER (EARLY ARRIVAL)
        // // Because wait edges cost 0.0, all Time Pillar nodes are equal.
        // // We subtract a microscopic amount based on the node's T value 
        // // to mathematically force D* Lite to pick the highest T (earliest arrival)!
        // // =========================================================
        // if (kd_dim_ > 2) {
        //     double time_a = val_a(kd_dim_ - 1);
        //     h_val -= (time_a * 1e-5); 
        // }

        // Ensure the heuristic never drops below 0
        return std::max(0.0, h_val);
    }
    else
        return 0.0;
}

DStarLiteKey KinodynamicPRMStarDStarLite::calculateKey(DStarLiteNode* u) {
    double min_val = std::min(u->g, u->rhs);
    double h_val = (start_node_) ? heuristic(u, start_node_) : 0.0;
    return { min_val + h_val + km_, min_val };
}


void KinodynamicPRMStarDStarLite::initialize(DStarLiteNode* start, DStarLiteNode* goal) {
    km_ = 0.0;
    open_queue_.clear(); // It removes the pillars so dont use it!
    start_node_ = start;
    goal_node_ = goal;
    goal->rhs = 0.0;

    open_queue_.add(goal, calculateKey(goal));
    for (int idx : time_pillar_indices_) {
        DStarLiteNode* node = nodes_[idx].get();
        node->rhs = 0.0;
        open_queue_.add(node, calculateKey(node));
    }

}



void KinodynamicPRMStarDStarLite::updateVertex(DStarLiteNode* u) {
    bool is_consistent = (u->g == u->rhs);


    if (!is_consistent && u->in_queue_) {
        // Update priority
        open_queue_.update(u, calculateKey(u));
    } 
    else if (!is_consistent && !u->in_queue_) {
        // Insert into queue
        open_queue_.add(u, calculateKey(u));
    } 
    else if (is_consistent && u->in_queue_) {
        // Remove from queue
        open_queue_.remove(u);
    }
}

// returns true if rhs (or best parent) actually changed
bool KinodynamicPRMStarDStarLite::recomputeRHS(DStarLiteNode* s) {
    if (s == goal_node_) return false;
    // SACRED ROOT PROTECTION: 
    // The main goal and all Time Pillars are mathematical sinks.
    // They must NEVER recompute their RHS. Their cost is eternally 0.0.
    if (time_pillar_indices_.find(s->getIndex()) != time_pillar_indices_.end()) {
        return false; 
    }


    double min_rhs = std::numeric_limits<double>::infinity();
    DStarLiteNode* best_parent = nullptr;
    std::shared_ptr<Trajectory> best_traj = nullptr;

    for (auto& [succ, edge_info] : s->forward_neighbors_) {
        if (edge_info.distance == std::numeric_limits<double>::infinity()) continue;
        if (succ->g == std::numeric_limits<double>::infinity()) continue;
        double cost = edge_info.distance + succ->g;
        if (cost < min_rhs - 1e-9) {
            min_rhs = cost;
            best_parent = succ;
            best_traj = edge_info.cached_trajectory;
        }
#if DEBUG_WITH_DIJKSTRA_
        else if (std::abs(cost - min_rhs) <= 1e-9) {
            if (best_parent && succ->getIndex() < best_parent->getIndex()) {
                best_parent = succ;
                best_traj = edge_info.cached_trajectory;
            }
        }
#endif
    }

    // double old_rhs = s->rhs;
    // // Only change rhs and parent if it actually changed
    // if (std::isfinite(old_rhs) != std::isfinite(min_rhs) || std::abs(old_rhs - min_rhs) > 1e-9) {
    //     s->rhs = min_rhs;
    //     s->setBestParent(best_parent, best_traj);

    //     return true;
    // }
    // // nothing changed
    // return false;


    // ALWAYS update the parent and rhs, regardless of cost!
    double old_rhs = s->rhs;
    s->rhs = min_rhs;
    s->setBestParent(best_parent, best_traj);

    // Only return true (triggering a queue update) if the mathematical cost changed
    if (std::isfinite(old_rhs) != std::isfinite(min_rhs) || std::abs(old_rhs - min_rhs) > 1e-9) {
        return true;
    }
    
    return false;

}

void KinodynamicPRMStarDStarLite::computeShortestPath() {
    if (!start_node_ || !goal_node_) return;

    
    while (!open_queue_.empty()) {
        
        // Recalculate target_key dynamically EVERY iteration
        DStarLiteKey target_key = calculateKey(start_node_);

        // Strict D* Lite Termination Condition
        // Check if start_node is fully consistent
        bool start_is_consistent = false;
        if (std::isinf(start_node_->g) && std::isinf(start_node_->rhs)) {
            start_is_consistent = true;
        } else {
            start_is_consistent = (std::abs(start_node_->g - start_node_->rhs) <= 1e-9);
        }


        // // Ensure the entire path from anchor to goal is consistent
        // bool path_to_goal_consistent = false;
        // if (start_is_consistent) {
        //     DStarLiteNode* node = start_node_;
        //     path_to_goal_consistent = true;
        //     while (node != goal_node_ &&
        //         time_pillar_indices_.find(node->getIndex()) == time_pillar_indices_.end()) {
        //         DStarLiteNode* parent = node->getParent();
        //         if (!parent || std::isinf(parent->rhs) || parent->g != parent->rhs) {
        //             path_to_goal_consistent = false;
        //             break;
        //         }
        //         node = parent;
        //     }
        // }

        // if (partial_update) {
        //     if (!(open_queue_.topKey() < target_key || !start_is_consistent || !path_to_goal_consistent))
        //         break;
        // }


        
        if (partial_update) {
            if (!(open_queue_.topKey() < target_key || !start_is_consistent ))
                break;
        }

        DStarLiteNode* u = open_queue_.top();
        DStarLiteKey k_old = open_queue_.topKey();
        if (!u) break;

        DStarLiteKey k_new = calculateKey(u);

        if (k_old < k_new) {
            // update
            open_queue_.update(u, k_new);
        } 
        // Overconsistent (Found a shortcut!)
        else if (u->g > u->rhs) {
            u->g = u->rhs;
            open_queue_.remove(u);
            
            for (auto& [pred, edge_info] : u->backward_neighbors_) {
                if (pred != goal_node_) {
                    double new_cost = edge_info.distance + u->g;
                    // If this new path through u is better, PUSH the update
                    if (pred->rhs > new_cost + 1e-9) {
                        pred->rhs = new_cost;
                        pred->setBestParent(u, edge_info.cached_trajectory);
                        updateVertex(pred); // only call when rhs changed
                    }
#if DEBUG_WITH_DIJKSTRA_
                    // TIE-BREAKER: Equal cost, but lower Node Index
                    else if (std::abs(pred->rhs - new_cost) <= 1e-9) {
                        if (pred->getParent() && u->getIndex() < pred->getParent()->getIndex()) {
                            pred->rhs = new_cost; // strictly speaking, cost is the same
                            pred->setBestParent(u, edge_info.cached_trajectory);
                            updateVertex(pred); // parent changed => update
                        }
                    }
#endif
                }
            }
        } 
        // Underconsistent (Path was blocked!)
        else {
            u->g = std::numeric_limits<double>::infinity();
            
            // Process Predecessors (s in pred(u))
            for (auto& [pred, edge_info] : u->backward_neighbors_) {
                // if(rhs(s) == c(s,u) + g_old)
                if (pred->getParent() == u) {
                    
                    // if(s != s_goal) rhs(s) = min(c(s,s') + g(s'))
                    bool changed = recomputeRHS(pred);  
                    
                    if (changed) {
                        updateVertex(pred);
                    }
                }
            }
            // Process u itself (the union {u} part)
            recomputeRHS(u);
            updateVertex(u);
        }
    }
}

void KinodynamicPRMStarDStarLite::resolvePathLazy() {
    bool path_is_collision_free = false;
    int max_inner_iterations = 100; // Safety breakout for real-time systems
    int iterations = 0;

    while (!path_is_collision_free && iterations++ < max_inner_iterations) {
        
        // 1. Process queue to find the optimistic shortest path
        computeShortestPath(); 

        if (start_node_->rhs == std::numeric_limits<double>::infinity()) {
            DSTARLITE_WARN("[LLPT] No solution exists in the current graph!");
            break; 
        }

        // 2. Lazily evaluate the first N edges on this path (Algorithm 2)
        // Set eval_batch_size_ to something like 5 or 10 based on the paper
        std::vector<DStarLiteNode*> collided_nodes = evaluateEdge(eval_batch_size_);

        if (collided_nodes.empty()) {
            // The path is verified safe!
            path_is_collision_free = true; 
        } else {
            // 3. Collision found! Instantly tear down the broken subtree (Algorithm 3)
            for (auto* v : collided_nodes) {
                propagateCostToLeave(v); 
            }
        }
    }
}

void KinodynamicPRMStarDStarLite::updateObstacles(const ObstacleVector& turned_obstacles) {
    if (turned_obstacles.empty()) return;

    double T_robot = 0.0;
    if (!is_geometric_mode_ && robot_continuous_state_.size() > 0) {
        T_robot = robot_continuous_state_(robot_continuous_state_.size() - 1);
    }

    for (const auto& incoming_ob : turned_obstacles) {
        Obstacle& stored_ob = previous_obstacles_[incoming_ob.name];
        
        // Remove OLD Tube
        if (!stored_ob.predicted_path.empty()) {
            removeObstacle(stored_ob); // NOW ONLY MARKS DIRTY & RESTORES OPTIMISTIC COST
        }

        // Update Logic
        stored_ob = incoming_ob; 
        stored_ob.predicted_path = obs_checker_->generatePrediction(stored_ob, T_robot);

        // Add NEW Tube
        addNewObstacle(stored_ob); // NOW ONLY MARKS EDGES is_evaluated = false
    }

    // Replace computeShortestPath() with the lazy loop
    resolvePathLazy();

    // VERIFICATION RUN
    #if DEBUG_WITH_DIJKSTRA_
        computeShortestPathDijkstraMode(); 
        debugCompareDijkstraVsDStarLite();
    #endif
}

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

    for (int idx : unique_node_indices) {
        DStarLiteNode* u = nodes_[idx].get();
        
        for (auto& [neighbor, edge] : u->forward_neighbors_) {
            // LLPT LAZY LOGIC: We do NOT evaluate the collision here!
            // We just mark the edge as "dirty" because the environment changed.
            // If the edge was previously collision-free, it stays collision-free for now.
            edge.is_evaluated = false;
            
            if (is_geometric_mode_ && neighbor->forward_neighbors_.count(u)) {
                neighbor->forward_neighbors_.at(u).is_evaluated = false;
            }
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

    // LLPT LAZY LOGIC: We DO NOT fetch all_obstacles and we DO NOT check collisions.
    for (int idx : unique_node_indices) {
        DStarLiteNode* u = nodes_[idx].get();
        bool u_needs_update = false;
        
        for (auto& [v, edge] : u->forward_neighbors_) {
            // We only care if the edge is currently BLOCKED
            if (edge.distance == std::numeric_limits<double>::infinity()) {
                
                double c_new = edge.distance_original;
                
                // Optimistically restore the edge
                edge.distance = c_new;
                edge.is_evaluated = false; // It must be rigorously checked if selected
                u_needs_update = true;

                if (v->backward_neighbors_.count(u)) {
                    v->backward_neighbors_.at(u).distance = c_new;
                }

                // ---- standard overconsistent update ----
                if (v->g != std::numeric_limits<double>::infinity() && c_new != std::numeric_limits<double>::infinity()) {
                    double candidate = c_new + v->g;
                    if (candidate + 1e-9 < u->rhs) {
                        u->rhs = candidate;
                        u->setBestParent(v, edge.cached_trajectory);
                    }
                }

                // handle geometric reverse edge restoration
                if (is_geometric_mode_ && v->forward_neighbors_.count(u)) {
                    auto& rev_edge = v->forward_neighbors_.at(u);
                    double rev_new = rev_edge.distance_original;
                    
                    rev_edge.distance = rev_new;
                    rev_edge.is_evaluated = false;
                    
                    if (u->backward_neighbors_.count(v)) {
                        u->backward_neighbors_.at(v).distance = rev_new;
                    }

                    if (u->g != std::numeric_limits<double>::infinity()) {
                        double candidate_rev = rev_new + u->g;
                        if (candidate_rev + 1e-9 < v->rhs) {
                            v->rhs = candidate_rev;
                            v->setBestParent(u, rev_edge.cached_trajectory);
                        }
                    }
                    updateVertex(v);
                }
            }
        }
        
        if (u_needs_update) {
            updateVertex(u);
        }
    }
}

void KinodynamicPRMStarDStarLite::propagateCostToLeave(DStarLiteNode* v) {
    // 1. Disconnect from broken parent
    v->setBestParent(nullptr, nullptr); 
    
    // 2. Set LMC (rhs) to infinity and push to queue so it finds a new path later
    v->rhs = std::numeric_limits<double>::infinity();
    updateVertex(v);
    
    // 3. Instantly cascade to all descendants that rely on 'v'
    for (auto& [pred, edge_info] : v->backward_neighbors_) {
        if (pred->getParent() == v) {
            propagateCostToLeave(pred);
        }
    }
}

std::vector<DStarLiteNode*> KinodynamicPRMStarDStarLite::evaluateEdge(int max_evals) {
    std::vector<DStarLiteNode*> collided_nodes;
    DStarLiteNode* current = start_node_;
    int evals = 0;
    
    while (current != goal_node_ && current != nullptr && evals < max_evals) {
        DStarLiteNode* parent = current->getParent(); // Next step towards goal
        if (!parent) break;
        
        auto& edge = current->forward_neighbors_[parent];
        
        if (!edge.is_evaluated) {
            last_replan_metrics_.obstacle_checks++;
            
            // The ACTUAL collision check happens here, just-in-time
            bool safe = true;
            for (const auto& ob : obs_checker_->getObstacles()) {
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*edge.cached_trajectory, ob)) {
                    safe = false;
                    break;
                }
            }
            
            edge.is_evaluated = true;
            
            if (!safe) {
                // Edge is blocked. Set cost to infinity.
                edge.distance = std::numeric_limits<double>::infinity();
                if (parent->backward_neighbors_.count(current)) {
                    parent->backward_neighbors_[current].distance = std::numeric_limits<double>::infinity();
                }
                collided_nodes.push_back(current);
            }
            evals++;
        }
        current = parent;
    }
    return collided_nodes;
}




void KinodynamicPRMStarDStarLite::setRobotState(const Eigen::VectorXd& robot_state) {
    robot_continuous_state_ = robot_state;

    double robot_time_to_go = 0.0;
    if (!is_geometric_mode_ && robot_continuous_state_.size() > 0) {
        robot_time_to_go = robot_continuous_state_(robot_continuous_state_.size() - 1);
    }

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

    const double hysteresis_factor = 0.98;
    double cost_of_current_anchor = std::numeric_limits<double>::infinity();
    
    Trajectory bridge;
    bool safe = true;
    if (start_node_ && start_node_->rhs != std::numeric_limits<double>::infinity()) {
        bridge = statespace_->steer(robot_continuous_state_, start_node_->getStateValue());
        if (bridge.is_valid) {
            const auto& obstacles = obs_checker_->getObstacles();
            for (const auto& ob : obstacles) {
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(bridge, ob)) {
                    safe = false;
                    break;
                }
            }

            if (safe) {
                cost_of_current_anchor = bridge.cost + start_node_->rhs;
                // return;
            }
        }
    }

    // Tracks the best node that ALREADY has a valid path to the goal
    DStarLiteNode* best_connected_node = nullptr;
    Trajectory best_connected_bridge;
    double best_connected_cost = std::numeric_limits<double>::infinity();
    
    // Tracks the safest physical node to reach, even if it is currently unexplored (g = infinity)
    DStarLiteNode* best_fallback_node = nullptr;
    Trajectory best_fallback_bridge;
    double best_fallback_cost = std::numeric_limits<double>::infinity();
    
    double current_search_radius = connection_radius_;
    const int max_attempts = 5;
    const double radius_multiplier = 2.0;
    std::unordered_set<size_t> tested_indices;
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        std::vector<size_t> candidate_indices = kdtree_->radiusSearch(query_point, current_search_radius);

        for (size_t idx : candidate_indices) {
            if (!tested_indices.insert(idx).second) {
                continue;
            }
            DStarLiteNode* candidate = nodes_[idx].get();

            // Check Steering & Collision FIRST (Do not skip nodes just because g == inf!)
            Trajectory temp_bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());
            if (!temp_bridge.is_valid) continue;

            bool safe = true;
            const auto& obstacles = obs_checker_->getObstacles();
            for (const auto& ob : obstacles) {
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(temp_bridge, ob)) {
                    safe = false;
                    break; // early exit on collision
                }
            }

            if (!safe) continue;


#if USE_RECOVERY
            // Fallback Tracking: Track the easiest physical node to reach
            if (temp_bridge.cost < best_fallback_cost) {
                best_fallback_cost = temp_bridge.cost;
                best_fallback_node = candidate;
                best_fallback_bridge = temp_bridge;
            }
#endif



            // Only attach to nodes that are fully consistent and NOT in the queue!
            if (candidate->rhs != std::numeric_limits<double>::infinity() && 
                candidate->g == candidate->rhs && 
                !candidate->in_queue_) {
                
                double total_cost = temp_bridge.cost + candidate->rhs;
                if (total_cost < best_connected_cost) {
                    best_connected_cost = total_cost;
                    best_connected_node = candidate;
                    best_connected_bridge = temp_bridge;
                }
            }



        }

        // Only break early if we found a CONNECTED node. 
        // If we only found a fallback, keep expanding radius to see if a connected one is slightly further away!
        if (best_connected_node) break;
        current_search_radius *= radius_multiplier;
    }


    // DECISION LOGIC
    // 1) Better connected anchor
    if (best_connected_node &&
        best_connected_cost < cost_of_current_anchor * hysteresis_factor) {
        if (start_node_ && start_node_ != best_connected_node) {
            km_ += heuristic(start_node_, best_connected_node);
        }
            
        start_node_ = best_connected_node;
        bridge_cost_ = best_connected_bridge.cost;
        last_replan_metrics_.path_cost = best_connected_cost;
        current_bridge_trajectory_ = best_connected_bridge;

        // THE FOLLOWING IS CRITICAL WHEN HEURISTIC IS ON! OTHERWISE IF THE updates
        // go uniform then the parent of a node surely gets updated sooner than the node
        // it self but with heuristic on and chaning the anchor node, hence the subtree 
        // constantly that new subtree might not be fully updated because in D star lite 
        // its not like RRTX so we do not immeidately inform the children because there 
        // is no orphaning procedure (propagate descendants)
        // CRITICAL: After moving the start to a new anchor, we must recompute
        // the shortest path. The D* Lite priority queue uses keys that depend
        // on the heuristic h(start, node) and the key modifier km. When the
        // start jumps arbitrarily, the heuristic values of the nodes already in
        // the queue become relative to the old start. Inconsistent ancestors
        // (e.g. a parent whose edge was blocked by an earlier obstacle)
        // might now have a key larger than CalcKey(start), causing the next
        // computeShortestPath() to terminate early and leave the anchor's path
        // broken.  Calling computeShortestPath() immediately recalculates all
        // keys with the correct start and ensures the entire chain up to the
        // goal is fully repaired.
        if(use_heuristic){
            auto t1 = std::chrono::steady_clock::now();
            resolvePathLazy();
            auto t2 = std::chrono::steady_clock::now();
            last_anchor_repair_ms_ = std::chrono::duration<double, std::milli>(t2 - t1).count();
            // double ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
            // RCLCPP_INFO(rclcpp::get_logger("DStarLite"), "Anchor repair search took: %.6f ms", ms);
        }
    } 
    // 2) Keep current anchor with fresh bridge
    else if (safe && start_node_ &&
             cost_of_current_anchor != std::numeric_limits<double>::infinity() &&
             bridge.is_valid) {
        bridge_cost_ = bridge.cost;
        last_replan_metrics_.path_cost = cost_of_current_anchor;
        current_bridge_trajectory_ = bridge;
    } 
    // 3) Recovery: go to nearest safe tree node (HIGHEST PRIORITY)
#if USE_RECOVERY
    else if (best_fallback_node) {
        // start_node_ = best_fallback_node;
        bridge_cost_ = best_fallback_cost;
        last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
        current_bridge_trajectory_ = best_fallback_bridge;
        DSTARLITE_WARN("[Set Robot State] USE_RECOVERY: Falling back to nearest safe tree node.");
    }
#endif
    // 4) Blind cached reuse (NO obstacle check - survival mode)
    else if (start_node_ &&
             current_bridge_trajectory_.is_valid &&
             !current_bridge_trajectory_.path_points.empty()) {
        bridge_cost_ = current_bridge_trajectory_.cost;
        cost_of_current_anchor = current_bridge_trajectory_.cost + start_node_->rhs;
        last_replan_metrics_.path_cost = cost_of_current_anchor;
        DSTARLITE_WARN("[Set Robot State] Fresh steer failed near anchor/root. Reusing cached bridge.");
    } 
    // 5) Truly trapped
    else {
        start_node_ = nullptr;
        bridge_cost_ = std::numeric_limits<double>::infinity();
        current_bridge_trajectory_ = Trajectory();
        last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
        DSTARLITE_WARN("[Set Robot State] LOST SAFE ANCHOR. TRULY TRAPPED.");
    }




// // --- DIAGNOSTIC: check anchor's ancestor consistency ---
// if (start_node_ && start_node_ != goal_node_ &&
//     time_pillar_indices_.find(start_node_->getIndex()) == time_pillar_indices_.end()) {
//     DStarLiteNode* check = start_node_->getParent();
//     int depth = 0;
//     while (check && check != goal_node_ && depth++ < 10) {
//         if (time_pillar_indices_.find(check->getIndex()) != time_pillar_indices_.end()) break;
//         if (std::isinf(check->rhs) || check->g != check->rhs) {
//             DSTARLITE_WARN("[AnchorCheck] Anchor " << start_node_->getIndex()
//                            << " (g=" << start_node_->g << ", rhs=" << start_node_->rhs
//                            << ") has inconsistent ancestor "
//                            << check->getIndex() << " (g=" << check->g
//                            << ", rhs=" << check->rhs << ")");
//             break;
//         }
//         check = check->getParent();
//     }
// }

}

void KinodynamicPRMStarDStarLite::visualizeTree() {
std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> dslite_tree_edges;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> other_valid_edges;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> dijkstra_tree_edges;
    std::vector<Eigen::VectorXd> all_node_positions;
    std::vector<Eigen::VectorXd> dslite_nodes_pos;
    std::vector<Eigen::VectorXd> dijkstra_nodes_pos;

    std::unordered_set<DStarLiteNode*> connected_to_goal;
    std::queue<DStarLiteNode*> q;

    Eigen::VectorXd goal_spatial = problem_def_->getStart().head(2); // The physical mission goal

    // 1. Seed the BFS with ALL Time Pillars + the Main Goal
    for (auto& node_ptr : nodes_) {
        DStarLiteNode* node = node_ptr.get();
        
        // Check if the node is physically located at the goal
        if ((node->getStateValue().head(2) - goal_spatial).norm() < 1e-3) {
            if (node->rhs == 0.0) { // Safety check: is it actually a root?
                connected_to_goal.insert(node);
                q.push(node);
            }
        }
    }

    // 2. Run the BFS to find all connected branches across all roots
    while (!q.empty()) {
        DStarLiteNode* cur = q.front();
        q.pop();

        for (const auto& [pred, edge_info] : cur->backward_neighbors_) {
            if (pred->best_parent_ == cur && edge_info.distance != std::numeric_limits<double>::infinity()) {
                if (connected_to_goal.insert(pred).second) {
                    q.push(pred);
                }
            }
        }
    }

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


    if (start_node_) {
        std::vector<Eigen::VectorXd> anchor_pt = { start_node_->getStateValue().head<2>() };
        visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");
    }
    
    // draw D* Lite overlay (dark gray)
    visualization_->visualizeEdges(dslite_tree_edges, "map",
        std::array<float,3>{0.5f, 0.5f, 0.5f},   // dark gray
        1.0f,                                    // slightly higher alpha so it's visible
        0.15f,                                   // thin line
        "dslite_tree",                           // namespace
        2,                                       // marker id
        false,                                   // dashed
        0.2);

    // draw D* Lite overlay (deep red)
    visualization_->visualizeEdges(dijkstra_tree_edges, "map",
        std::array<float,3>{1.0f, 0.0f, 0.0f},   
        1.0f,                                    // alpha
        0.05f,                                   // thin line
        "dijkstra_tree",                           // namespace
        3,                                       // marker id
        false,                                   // dashed
        0.2);

    // // Optionally draw other valid edges lightly
    // if (!other_valid_edges.empty()) {
    //     visualization_->visualizeEdges(other_valid_edges, "map", "0.7,0.7,0.7", "valid_edges");
    // }
}
std::vector<Eigen::VectorXd> KinodynamicPRMStarDStarLite::getPathPositions() const{
    // FIXED: Only check for null pointer, NOT rhs == infinity!
    // Recovery nodes have rhs == infinity but are still valid anchors!
    if (!start_node_ || start_node_->rhs == std::numeric_limits<double>::infinity()) {
        DSTARLITE_ERROR("[DSTARLITE_Path_Assembly] Robot has no valid anchor node. Cannot build path");
        return {};
    }

    // Safety check on the cached bridge computed in setRobotState
    if (!current_bridge_trajectory_.is_valid || current_bridge_trajectory_.path_points.empty()) {
        DSTARLITE_ERROR("DSTARLITE_Path_Assembly: Cached bridge trajectory is invalid. Cannot build path");
        return {};
    }



    // Start the final path with the CACHED bridge trajectory! (Zero computation time)
    std::vector<Eigen::VectorXd> path = current_bridge_trajectory_.path_points;

    // Traverse Graph (Anchor -> Goal)
    DStarLiteNode* current_node = start_node_;
    int steps = 0;
    const int max_steps = nodes_.size();

    while (current_node != goal_node_) {
        if (steps++ > max_steps) {
            DSTARLITE_WARN("[DSTARLITE_Path_Assembly] Cycle detected. Aborting.");
            break;
        }
        
        DStarLiteNode* next_node = current_node->best_parent_;

        if (!next_node) {
            break; // Reached goal or dead end (expected for recovery nodes)
        }

        auto traj = current_node->best_parent_trajectory_;
        if (traj && traj->is_valid && traj->path_points.size() > 1) {
            // Skip the first point to avoid duplicate overlapping points at the nodes
            path.insert(path.end(), traj->path_points.begin() + 1, traj->path_points.end());
        } else {
            path.push_back(next_node->getStateValue());
        }
        current_node = next_node;
    }

    return path;
}



void KinodynamicPRMStarDStarLite::visualizeTreeGradient() {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    std::vector<double> edge_costs;

    if (!nodes_.empty()) {
        edges.reserve(nodes_.size());
        edge_costs.reserve(nodes_.size());
    }
    
    // Find max cost in the D* Lite tree to normalize the gradient
    double max_tree_cost = 0.001; 
    for (const auto& node_ptr : nodes_) {
        DStarLiteNode* u = node_ptr.get();
        // DStarLiteNode directly accesses the 'g' value
        if (u->g > max_tree_cost && !std::isinf(u->g)) {
            max_tree_cost = u->g;
        }
    }

    for (const auto& node_ptr : nodes_) {
        DStarLiteNode* u = node_ptr.get();
        DStarLiteNode* parent_node = u->best_parent_; // The successor node pointing towards the goal

        if (parent_node && !std::isinf(u->g)) {
            // Draw edge from current node to its best parent
            edges.emplace_back(u->getStateValue(), parent_node->getStateValue());
            
            // Normalize the cost between 0.0 and 1.0
            edge_costs.push_back(u->g / max_tree_cost); 
        }
    }
    
    if (start_node_) {
        std::vector<Eigen::VectorXd> anchor_pt = { start_node_->getStateValue().head<2>() };
        visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");
    }
    visualization_->visualizeTreeGradient(edges, edge_costs, "map");
}

void KinodynamicPRMStarDStarLite::visualizePathGradient(const std::vector<Eigen::VectorXd>& path_waypoints) {
    // D* Lite uses start_node_ to track the robot's physical location
    if (path_waypoints.size() < 2 || !visualization_ || start_node_ == nullptr) {
        return;
    }
    
    double robot_g = start_node_->g;
    if (std::isinf(robot_g)) return;

    double current_robot_cost = robot_g + bridge_cost_;
    if (global_max_cost_ < current_robot_cost) {
        global_max_cost_ = current_robot_cost;
    }
    double max_c = (global_max_cost_ > 0.0) ? global_max_cost_ : 1.0;
    std::vector<double> path_costs(path_waypoints.size(), 0.0);
    size_t N = path_waypoints.size();
    
    for (size_t i = 0; i < N; ++i) {
        double fraction = (double)(N - 1 - i) / (N - 1);
        path_costs[i] = fraction * current_robot_cost;
    }

    visualization_->visualizePathGradient(path_waypoints, path_costs, "map", max_c);
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

    if (visualization_) {
        visualization_->visualizeEdges(edges, "map", "0.0,1.0,0.0", "executable_path");
    }
}


