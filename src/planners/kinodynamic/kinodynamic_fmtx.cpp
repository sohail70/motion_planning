// Copyright 2025 Soheil E.nia
// TODO : fix the KNN usage because with knn there is not neighborhood radisu constraints (check near function)

#define DEBUG 0
#define VIS 0
// The Threat Set is the bridge that allows a lazy algorithm to behave with the same spatial intelligence as an eager one
#define USE_THREAT_SET_STRATEGY 0
#define STATIC 0
#define USE_RECOVERY 0 // Emergency Fallback
#include "motion_planning/planners/kinodynamic/kinodynamic_fmtx.hpp"

KinodynamicFMTX::KinodynamicFMTX(std::shared_ptr<StateSpace> statespace ,std::shared_ptr<ProblemDefinition> problem_def, std::shared_ptr<ObstacleChecker> obs_checker) :  statespace_(statespace), problem_(problem_def), obs_checker_(obs_checker) {
    std::cout<< "KinodynamicFMTX Constructor \n";
}


void KinodynamicFMTX::clearPlannerState() {
    v_open_heap_.clear();
    for (auto& node : tree_) {
        node->disconnectFromGraph();
        node.reset();  
    }
    tree_.clear();
    statespace_->reset();
    kdtree_.reset();
    root_state_index_ = -1;
    robot_state_index_ = -1;
}


// void KinodynamicFMTX::injectTimePillarNodes(const Eigen::VectorXd& goal_state_val, int num_pillar_nodes) {
//     double max_time = upper_bounds_(statespace_->getDimension() - 1); 


//     for (int i = 1; i <= num_pillar_nodes; ++i) {
//         Eigen::VectorXd pillar_state = goal_state_val;

//         // FIX 1: Safely zero velocities ONLY for 5D (x, y, vx, vy, t)
//         if (statespace_->getDimension() == 5) {
//             pillar_state(2) = 0.0; // vx
//             pillar_state(3) = 0.0; // vy
//         }

//         // Distribute evenly across time
//         double t_val = (max_time / num_pillar_nodes) * i; 
//         pillar_state(statespace_->getDimension() - 1) = t_val;

//         auto state_ptr = statespace_->addState(pillar_state);
//         auto node = std::make_unique<FMTNode>(state_ptr, tree_.size());
//         node->setTimeToGoal(t_val);
        
//         // Set them as Zero-Cost Roots so the backward search can start from them!
//         node->setLMC(0.0);
//         node->setG(0.0);
//         v_open_heap_.add(node.get(), 0.0);

//         time_pillar_indices_.insert(node->getIndex());
//         tree_.push_back(std::move(node));
//     }

// }


void KinodynamicFMTX::injectTimePillarNodes(const Eigen::VectorXd& goal_state_val, int num_pillar_nodes) {
    if (num_pillar_nodes <= 0) return;

    auto start_state_val_ = problem_->getGoal();
    // 1. Calculate minimum required time (using only X and Y coordinates)
    // We assume the first two dimensions of all state spaces are X and Y.
    double dist = (goal_state_val.head(2) - start_state_val_.head(2)).norm();
    double min_arrival_time = dist / statespace_->getMaxVelocity();

    // 2. Get the maximum time budget from the problem bounds
    int time_dim_index = statespace_->getDimension() - 1;
    double max_time = upper_bounds_(time_dim_index); 

    // If max_time is less than min_arrival_time, the goal is physically unreachable.
    if (max_time <= min_arrival_time) {
        FMTX_ERROR("Time budget is too strict. Goal is physically unreachable at max velocity.");
        return; 
    }

    // 3. Define the reachable time window
    double time_window = max_time - min_arrival_time;

    // 4. Inject the Time Pillars
    for (int i = 1; i <= num_pillar_nodes; ++i) {
        // Copy the goal state. Because goal_state_val already has vx=0, vy=0 in the Thruster 
        // configuration, this naturally copies those zeros without needing hardcoded 5D checks!
        Eigen::VectorXd pillar_state = goal_state_val;

        // Distribute time evenly across the REACHABLE window
        // double t_val = min_arrival_time + (time_window / num_pillar_nodes) * i; 

        // 1. Calculate how long the journey takes (distributed between min_arrival and max_time)
        double travel_time = min_arrival_time + (time_window / num_pillar_nodes) * i; 
        
        // 2. Subtract travel_time from max_time because time decreases from the robot to the goal
        double t_val = max_time - travel_time; 

        pillar_state(time_dim_index) = t_val;

        auto state_ptr = statespace_->addState(pillar_state);
        auto node = std::make_unique<FMTNode>(state_ptr, tree_.size());
        
        // This time is inherently valid and reachable
        node->setTimeToGoal(t_val); 
        
        node->setLMC(0.0);
        node->setG(0.0);
        v_open_heap_.add(node.get(), 0.0);

        time_pillar_indices_.insert(node->getIndex()); 

        tree_.push_back(std::move(node));
    }
}


void KinodynamicFMTX::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    std::cout << "------------------------------------------------------------\n";
    auto start = std::chrono::high_resolution_clock::now();
    // clearPlannerState();
    visualization_ = visualization;
    num_of_samples_ = params.getParam<int>("num_of_samples");
    partial_update = params.getParam<bool>("partial_update");
#if DEBUG
    partial_update = false; // Since the runCostForensic and runGlobalCostForensic is not queue-aware. we use this to make sure the propagation is solid in the debug mode
#endif
    factor_ = params.getParam<double>("factor", 1.0);
    is_geometric_mode_ = params.getParam<bool>("is_geometric_mode", false);
    lower_bounds_ = problem_->getLowerBound();
    upper_bounds_ = problem_->getUpperBound();
    kd_dim = params.getParam<int>("kd_dim", 2);
    std::string kdtree_type = params.getParam<std::string>("kdtree_type");
    use_knn = params.getParam<bool>("use_knn", false);
    num_pillar_nodes_ = params.getParam<int>("num_pillar_nodes", 50);
    if (is_geometric_mode_) num_pillar_nodes_ = 0;

#if DEBUG
    num_pillar_nodes_ = 0;
#endif

    if (kdtree_type == "NanoFlann"){
        Eigen::VectorXd weights(kd_dim);
        switch (kd_dim) {
            case 2: // (x, y)
                weights << 1.0, 1.0; // Weights for x, y,
                break;
            case 3: // (x, y, time)
                weights << 1.0, 1.0, 1.0; // Weights for x, y, time
                break;
            case 4: // (x, y, theta, time)
                weights << 1.0, 1.0, 1.0, 1.0; // Weights for x, y, theta, time
                break;
            case 5:
                weights << 1.0, 1.0, 1.0, 1.0, 1.0; // Weights for x, y, vx, vy, time
                break;
            default: 
                FMTX_ERROR("Unsupported k-d tree dimension: " << kd_dim);
        }
        kdtree_ = std::make_shared<WeightedNanoFlann>(kd_dim, weights);
    } else if (kdtree_type == "LieKDTree"){
        kdtree_ = std::make_unique<LieSplittingKDTree>(statespace_->getDimension(), statespace_);
    } else {
        throw std::runtime_error("FMTX requires a KD-Tree.");
    }
    std::cout << "num_of_samples=" << num_of_samples_
                << ", bounds=[" << lower_bounds_ << ", " << upper_bounds_ << "]\n";


    bool use_rrtx_saved_samples_ = false;
    if (use_rrtx_saved_samples_) {
        std::string filepath = "./rrtx_tree_nodes.csv";
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

            // Read the state vector (x0, x1, x2...)
            for(int i = 0; i < statespace_->getDimension(); ++i) {
                std::getline(lineStream, cell, ',');
                state_values.push_back(std::stod(cell));
            }

            // Create an Eigen vector and then the FMTNode
            Eigen::Map<Eigen::VectorXd> state_vec(state_values.data(), state_values.size());
            auto node = std::make_unique<FMTNode>(statespace_->addState(state_vec), tree_.size());


            if (!is_geometric_mode_) {
                if (state_vec.size() > 0) {
                    double absolute_t = state_vec(state_vec.size() - 1);  // last component = time
                    node->setTimeToGoal(absolute_t);
                }
            } else {
                node->setTimeToGoal(0.0);
            }


            tree_.push_back(std::move(node));
        }
        fin.close();
        std::cout << "Loaded " << tree_.size() << " nodes from file.\n";

        const Eigen::VectorXd& start_state_val = problem_->getStart();
        const Eigen::VectorXd& goal_state_val = problem_->getGoal();

        FMTNode* root_node_ptr = nullptr;
        FMTNode* robot_node_ptr = nullptr;
        double min_dist_to_start = std::numeric_limits<double>::infinity();
        double min_dist_to_goal = std::numeric_limits<double>::infinity();

        for (const auto& node_ptr : tree_) {
            // Find the node closest to the tree root (the destination)
            double dist_to_start = (node_ptr->getStateValue() - start_state_val).norm();
            if (dist_to_start < min_dist_to_start) {
                min_dist_to_start = dist_to_start;
                root_node_ptr = node_ptr.get();
            }

            // Find the node closest to the robot's initial state
            double dist_to_goal = (node_ptr->getStateValue() - goal_state_val).norm();
            if (dist_to_goal < min_dist_to_goal) {
                min_dist_to_goal = dist_to_goal;
                robot_node_ptr = node_ptr.get();
            }
        }

        if (!root_node_ptr) throw std::runtime_error("Could not find any node near the start state.");
        if (!robot_node_ptr) throw std::runtime_error("Could not find any node near the goal state.");
        
        robot_node_ = robot_node_ptr;
        root_state_index_ = root_node_ptr->getIndex();
        robot_state_index_ = robot_node_ptr->getIndex();

        // Configure the root node (the destination of the backward search)
        root_node_ptr->setLMC(0);
        root_node_ptr->setTimeToGoal(0);
        v_open_heap_.add(root_node_ptr, 0.0);

        // Configure the goal node (the robot's starting position)
        robot_node_ptr->setTimeToGoal(goal_state_val(goal_state_val.size() - 1));

        std::cout<<"Successfully identified start  and goal  nodes."<<"\n";
    }
    else{
        // setStart(problem_->getStart());
        // setGoal(problem_->getGoal());
        // // Protect the original main root (T=0)
        // time_pillar_indices_.insert(root_state_index_); 
        // if(!is_geometric_mode_) injectTimePillarNodes(problem_->getStart(), num_pillar_nodes_);
        // for (int i = 0 ; i < num_of_samples_ - 2 - num_pillar_nodes_; i++) {
        //     auto node = std::make_unique<FMTNode>(statespace_->sampleUniform(lower_bounds_ , upper_bounds_),tree_.size());
        //     if (!is_geometric_mode_) {
        //         // Kinodynamic: Set time from state vector
        //         double absolute_t = node->getStateValue().tail<1>()[0];
        //         node->setTimeToGoal(absolute_t);
        //     } else {
        //         // Geometric: Time is irrelevant, set to 0
        //         node->setTimeToGoal(0.0);
        //     }
        //     tree_.push_back(std::move(node));
        // }
        ///////////////////////////
        setStart(problem_->getStart());
        setGoal(problem_->getGoal());
        // Protect the original main root (T=0)
        time_pillar_indices_.insert(root_state_index_);
        if (!is_geometric_mode_) injectTimePillarNodes(problem_->getStart(), num_pillar_nodes_);

        // Forward-cone constants. T_horizon = time upper bound, since the robot
        // is static during presampling.
        const int    time_idx  = statespace_->getDimension() - 1;
        const double t_lo      = lower_bounds_(time_idx);
        const double t_hi      = upper_bounds_(time_idx);
        const double T_horizon = t_hi;

        // Pillars are NOT counted against the sample budget: generate the full
        // num_of_samples_ space-filling samples (minus the 2 endpoints).
        for (int i = 0; i < num_of_samples_ - 2; i++) {
            if (is_geometric_mode_) {
                auto node = std::make_unique<FMTNode>(
                    statespace_->addState(statespace_->sampleUnregistered(lower_bounds_, upper_bounds_)), tree_.size());
                node->setTimeToGoal(0.0);
                tree_.push_back(std::move(node));
                continue;
            }

            // Sample unregistered so we can re-project the time component into
            // the feasible forward-cone before committing it to the statespace.
            Eigen::VectorXd sample_val =
                statespace_->sampleUnregistered(lower_bounds_, upper_bounds_);

            // One-sided goal-reachability cone (shared StateSpace helper; budget = T_horizon).
            const bool cone_ok = statespace_->remapTimeToGoalCone(
                sample_val, Eigen::Vector2d(problem_->getStart().head(2)), T_horizon, t_lo, t_hi);
            // Invariant: with diag_xy / v_max <= t_hi the cone is always non-empty.
            assert(cone_ok && "time budget too small for workspace diagonal");
            (void)cone_ok;
            const double t_new = sample_val(time_idx);

            auto node = std::make_unique<FMTNode>(statespace_->addState(sample_val), tree_.size());
            node->setTimeToGoal(t_new);
            tree_.push_back(std::move(node));
        }


    }


    Eigen::MatrixXd all_samples = statespace_->getSamplesCopy();
    Eigen::MatrixXd spatial_samples_only = all_samples.leftCols(kd_dim).eval();
    // kdtree_->addPoints(spatial_samples_only);
    kdtree_->addPoints(all_samples);
    kdtree_->buildTree();

#if STATIC
    // SPATIAL KD-tree (x,y only)
    spatial_kdtree_ = std::make_shared<DynamicWeightedNanoFlann>(2, Eigen::Vector2d(1.0,1.0));
    Eigen::MatrixXd spatial_samples = all_samples.leftCols(2);  // Just (x,y)
    spatial_kdtree_->addPoints(spatial_samples);
    spatial_kdtree_->buildTree();
#endif



    /////////////////// Neighborhood Radius ////////////////////////////////
    // Genuine space-filling sample count (exclude co-located root + pillars).
    int N = static_cast<int>(statespace_->getNumStates())- num_pillar_nodes_;
    if (N < 2) N = 2;

    if (use_knn) {
        int d = statespace_->getDimension();
        // Practical k-NN parameter from the FMT* paper's experiments 
        double k0_fmt_star_practical = std::pow(2.0, d) * (M_E / d);
        k_neighbors_ = static_cast<int>(std::ceil(factor_ * k0_fmt_star_practical * std::log(N)));
        // // // Standard k-NN parameter for RRT*
        // double k0_rrt_star = M_E * (1.0 + 1.0 / d);
        // k_neighbors_ = static_cast<int>(std::ceil(factor * k0_rrt_star * std::log(statespace_->getNumStates())));
        std::cout << "k-NN formula. k = " << k_neighbors_ << "\n";
    } else {
        int d = statespace_->getDimension();
        Eigen::VectorXd range = upper_bounds_ - lower_bounds_;
        double mu = range.prod(); // .prod() computes the product of all coefficients
        std::cout<<"mu "<<mu<<"\n";
        double zetaD = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
        // double gamma = 2 * std::pow(1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); //Real FMT star gamma which is smaller than rrt star which makes the neighborhood size less than rrt star
        double gamma = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d);
        neighborhood_radius_ = factor_ * gamma * std::pow(std::log(N) / N, 1.0 / d);
        // neighborhood_radius_ = factor * gamma * std::pow(std::log(num_of_samples_) / num_of_samples_, 1.0 / d);
        std::cout<<"factor: "<<factor_<<"\n";
        std::cout << "Computed value of rn: " << neighborhood_radius_ << std::endl;
    }


    // In complex state spaces with complex steer function its better to cache before leaving the robot in the wild!
    std::cout << "Forcing neighbor caching for all " << tree_.size() << " nodes..." << std::endl;

    for (size_t i = 0; i < tree_.size(); ++i) {
        near(i);
    }

    checkIsolatedNodes(); // Since we presample, some nodes might not get to have any neighbors in non geometric tests. This is just a report

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "------------------------------------------------------------\n";
}

void KinodynamicFMTX::checkIsolatedNodes() {
    // std::vector<int> isolated_indices;

    std::cout << "Checking " << tree_.size() << " nodes for isolation (zero neighbors)..." << std::endl;

    for (size_t i = 0; i < tree_.size(); ++i) {
        const auto* node = tree_[i].get();
        
        if (node->forwardNeighbors().empty() && node->backwardNeighbors().empty()) {
            isolated_nodes_count_++;
            // isolated_indices.push_back(i);
        }
    }

    if (isolated_nodes_count_ > 0) {
        std::cout << "Found " << isolated_nodes_count_ 
                  << " isolated nodes (nodes with 0 neighbors) out of " 
                  << tree_.size() << " total nodes." << std::endl;
        
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



bool KinodynamicFMTX::runGlobalCostForensics() {
    int nodes_checked = 0;
    int drift_violations = 0;
    double max_drift = 0.0;
    FMTNode* worst_node = nullptr;

    // Use -2.0 as uninitialized, -1.0 as "currently evaluating" (to catch cycles)
    std::vector<double> true_costs(tree_.size(), -2.0);
    
    std::function<double(FMTNode*)> computeTrueCost = [&](FMTNode* node) -> double {
        int idx = node->getIndex();
        
        if (idx == root_state_index_) return 0.0;
        
        if (node->getParent() == nullptr || node->getLMC() == std::numeric_limits<double>::infinity()) {
            return std::numeric_limits<double>::infinity();
        }
        
        // Cycle detection
        if (true_costs[idx] == -1.0) {
            return std::numeric_limits<double>::infinity();
        }
        
        // Memoized result
        if (true_costs[idx] >= 0.0) {
            return true_costs[idx];
        }

        true_costs[idx] = -1.0;

        double parent_true_cost = computeTrueCost(node->getParent());
        
        if (parent_true_cost == std::numeric_limits<double>::infinity()) {
            true_costs[idx] = std::numeric_limits<double>::infinity();
            return std::numeric_limits<double>::infinity();
        }

        double edge_cost = 0.0;
        if (node->getParentTrajectory() != nullptr) {
            edge_cost = node->getParentTrajectory()->cost;
        } else {
            true_costs[idx] = std::numeric_limits<double>::infinity();
            return std::numeric_limits<double>::infinity();
        }

        true_costs[idx] = parent_true_cost + edge_cost;
        return true_costs[idx];
    };

    // Evaluate all nodes
    for (size_t i = 0; i < tree_.size(); ++i) {
        FMTNode* node = tree_[i].get();
        double stored_cost = node->getLMC(); 

        if (stored_cost != std::numeric_limits<double>::infinity() && node->getIndex() != root_state_index_) {
            nodes_checked++;
            double actual_true_cost = computeTrueCost(node);

            if (actual_true_cost != std::numeric_limits<double>::infinity()) {
                // Absolute difference (catches both pessimistic and optimistic drifts)
                double drift = std::abs(stored_cost - actual_true_cost);
                
                if (drift > max_drift) {
                    max_drift = drift;
                    worst_node = node;
                }

                if (drift > 1e-5) { 
                    drift_violations++;
                }
            }
        }
    }
    
    if (worst_node != nullptr && max_drift > 1e-5) {
        std::cout << "-> Maximum cost drift observed: " << max_drift 
                  << " (at Node " << worst_node->getIndex() << ")\n";
                  
        std::cout << "\n\033[1;31m[X-RAY TRACE OF WORST VIOLATOR: NODE " << worst_node->getIndex() << "]\033[0m\n";
        std::cout << "------------------------------------------------------\n";
        FMTNode* curr = worst_node;
        while (curr != nullptr && curr->getIndex() != root_state_index_) {
            FMTNode* parent = curr->getParent();
            double edge_cost = (curr->getParentTrajectory() != nullptr) ? curr->getParentTrajectory()->cost : 0.0;
            
            std::cout << "Node " << curr->getIndex() 
                      << " | Stored g(v): " << curr->getLMC() << "\n";
            std::cout << "   -> Parent " << parent->getIndex() 
                      << " | Stored g(v): " << parent->getLMC() << "\n";
            std::cout << "   -> Trajectory Edge Cost: " << edge_cost << "\n";
            
            // Check the exact Bellman math for this single step
            double expected_lmc = parent->getLMC() + edge_cost;
            std::cout << "   => Expected lmc(v): " << expected_lmc 
                      << " | Local Math Error: " << (curr->getLMC() - expected_lmc) << "\n";
            std::cout << "------------------------------------------------------\n";
            
            curr = parent;
        }
        std::cout << "Root Node " << root_state_index_ << " reached.\n\n";
    }
    
    if (drift_violations == 0) {
        std::cout << "[FMTx FORENSICS] --- \033[1;32mPASSED\033[0m: All nodes possess exact true cost! ---\n\n";
        return true;
    } else {
        std::cout << "[FMTx FORENSICS] --- \033[1;31mFAILED\033[0m: Found " << drift_violations << " nodes with broken optimal substructure. Check trace above. ---\n\n";
        return false;
    }
}

bool KinodynamicFMTX::runCostForensics() {
    std::cout << "\n[FMTx FORENSICS] --- STARTING STRICT LOCAL CONSISTENCY VERIFICATION ---" << std::endl;
    int nodes_checked = 0;
    int violations = 0;
    int queue_waiting = 0;
    double max_local_inconsistency = 0.0;
    int max_node = -1;

    for (size_t i = 0; i < tree_.size(); ++i) {
        FMTNode* node = tree_[i].get();
        double g_node = node->getLMC();

        // Skip root or unconnected nodes
        if (node->getIndex() == root_state_index_ || g_node == std::numeric_limits<double>::infinity()) {
            continue;
        }

        FMTNode* parent = node->getParent();
        if (parent == nullptr) continue; // Safety check

        double g_parent = parent->getLMC();
        double edge_cost = (node->getParentTrajectory() != nullptr) ? node->getParentTrajectory()->cost : 0.0;

        // In exact FMTX, getLMC() acts as both g and lmc. They must match.
        double lmc = g_parent + edge_cost;
        
        // Calculate absolute inconsistency
        double inconsistency = std::abs(g_node - lmc);

        if (inconsistency > 1e-5) {
            if (node->in_queue_) {
                // Node's parent cost changed, making this node locally inconsistent, 
                // but it's correctly residing in the queue waiting for expansion.
                queue_waiting++;
                continue; 
            } else {
                // REAL BUG: Graph consistency is broken and node is stranded out of queue!
                violations++;
                std::cout << "\033[1;31m[REAL VIOLATION]\033[0m Node " << node->getIndex() 
                          << " | Stored g(v): " << g_node 
                          << " | True LMC: " << lmc 
                          << " | Diff: " << inconsistency << "\n"
                          << "                 AND IT IS NOT IN THE QUEUE!\n";
            }
        }

        nodes_checked++;
        if (inconsistency > max_local_inconsistency) {
            max_local_inconsistency = inconsistency;
            max_node = node->getIndex();
        }
    }

    std::cout << "-> Checked " << nodes_checked << " settled nodes.\n";
    std::cout << "-> Safely ignored " << queue_waiting << " inconsistent nodes correctly waiting in the queue.\n";
    std::cout << "-> Maximum local inconsistency of SETTLED nodes: " << max_local_inconsistency;
    if (max_node != -1) {
        std::cout << " (at Node " << max_node << ")";
    }
    std::cout << "\n";

    if (violations > 0) {
        std::cout << "[FMTx FORENSICS] --- \033[1;31mFAILED\033[0m: Found " << violations 
                  << " strictly inconsistent nodes missing from the queue! ---\n";
        return false;
    } else {
        std::cout << "[FMTx FORENSICS] --- \033[1;32mPASSED\033[0m: All FMTX nodes maintain perfect local Bellman consistency! ---\n";
        return true;
    }
}




void KinodynamicFMTX::runFMT(SuboptimalityMetrics& metrics) {

    struct FMTShadow {
        double cost = std::numeric_limits<double>::infinity();
        bool in_unvisited = true;
        bool in_queue = false;
        int parent_idx = -1;
        std::shared_ptr<Trajectory> parent_traj;
    };

    const int N = static_cast<int>(tree_.size());
    if (N == 0) return;

    std::vector<FMTShadow> shadow(N);

    if (root_state_index_ < 0 || root_state_index_ >= N) {
        FMTX_WARN("[runFMT] root_state_index_ invalid");
        return;
    }

    // ---------------------------------------------------------------
    // Proxy wrapper for shadow nodes (compatible with PriorityQueue)
    // ---------------------------------------------------------------
    struct ShadowProxy {
        int idx;
        std::vector<FMTShadow>* shadow;
        mutable bool in_queue_;   // must match shadow->in_queue
        size_t heap_index_;

        ShadowProxy(int i, std::vector<FMTShadow>* s)
            : idx(i), shadow(s), in_queue_(false), heap_index_(0) {}
    };

    struct ShadowProxyCompare {
        bool operator()(const std::pair<double, ShadowProxy*>& a,
                        const std::pair<double, ShadowProxy*>& b) const {
            if (std::abs(a.first - b.first) > 1e-9) return a.first < b.first;
            return a.second->idx < b.second->idx;
        }
    };

    PriorityQueue<ShadowProxy, ShadowProxyCompare> fmt_queue;
    std::vector<std::unique_ptr<ShadowProxy>> proxies; // automatic cleanup

auto addProxy = [&](int idx, double cost) {
    auto p = std::make_unique<ShadowProxy>(idx, &shadow);
    // p->in_queue_ is false initially
    fmt_queue.add(p.get(), cost);          // this will set p->in_queue_ = true
    shadow[idx].in_queue = p->in_queue_;   // sync the shadow flag
    proxies.push_back(std::move(p));
};

    // Seed root
    shadow[root_state_index_].cost = 0.0;
    shadow[root_state_index_].in_unvisited = false;
    // shadow[root_state_index_].in_queue = true;
    addProxy(root_state_index_, 0.0);
    // for (int idx : time_pillar_indices_) {
    //     if (idx < 0 || idx >= N) continue;
    //     shadow[idx].cost = 0.0;
    //     shadow[idx].in_unvisited = false;
    //     addProxy(idx, 0.0);
    // }


    int checks = 0;
    const double fmt_radius = neighborhood_radius_;

    auto start_time = std::chrono::steady_clock::now();
    // ---------------------------------------------------------------
    // FMT* wavefront using live neighbor maps
    // ---------------------------------------------------------------
    while (!fmt_queue.empty()) {
        auto top_pair = fmt_queue.top();
        double z_cost = top_pair.first;
        ShadowProxy* z_proxy = top_pair.second;
        int z_idx = z_proxy->idx;

        // (Optional consistency check)
        if (!shadow[z_idx].in_queue) {
            fmt_queue.pop();
            continue;
        }

        FMTNode* z_node = tree_[z_idx].get();
        if (!z_node) {
            fmt_queue.pop();
            continue;
        }


// // #if VIS  // Assuming you use a macro to toggle heavy visualization
//         if (visualization_) {
//             // 1. Visualize the current expanding 'z' node (e.g., in bright green)
//             visualization_->visualizeNodes(
//                 {z_node->getStateValue()}, 
//                 "map", 
//                 std::vector<float>{0.0f, 1.0f, 0.0f}, // Green
//                 "current_z_node"
//             );

//             // 2. Visualize the rest of the V_open queue (e.g., in red)
//             std::vector<Eigen::VectorXd> open_set_nodes;
//             const auto& heap_data = fmt_queue.getHeap();
//             open_set_nodes.reserve(heap_data.size()); // minor optimization
            
//             for (const auto& element : heap_data) {
//                 // element.second is a ShadowProxy* in this custom queue
//                 int open_idx = element.second->idx;
//                 if (tree_[open_idx]) {
//                     open_set_nodes.push_back(tree_[open_idx]->getStateValue());
//                 }
//             }

//             if (!open_set_nodes.empty()) {
//                 visualization_->visualizeNodes(
//                     open_set_nodes, 
//                     "map", 
//                     std::vector<float>{1.0f, 0.0f, 0.0f}, // Red
//                     "v_open_heap_nodes"
//                 );
//             }
            
//             // Publish the markers to RViz
//             visualization_->triggerPublish();
            
//             // Short delay so you can actually watch the wavefront expand in RViz
//             std::this_thread::sleep_for(std::chrono::milliseconds(5000)); 
//         }
// // #endif


        std::vector<int> newly_open;

        // Iterate over backward neighbors of z (nodes x that can reach z)
        for (const auto& [x_node, edge_info] : z_node->backwardNeighbors()) {
            // if (edge_info.distance > fmt_radius) continue;
            int x_idx = x_node->getIndex();
            if (x_idx < 0 || x_idx >= N) continue;
            FMTShadow& x_shadow = shadow[x_idx];
            if (!x_shadow.in_unvisited) continue;

            ++checks;

            // Find best parent for x among its forward neighbors that are in V_open
            double best_cost = std::numeric_limits<double>::infinity();
            int best_parent_idx = -1;
            std::shared_ptr<Trajectory> best_traj;

            for (const auto& [y_node, edge_info_xy] : x_node->forwardNeighbors()) {
                // if (edge_info_xy.distance > fmt_radius) continue;
                int y_idx = y_node->getIndex();
                if (y_idx < 0 || y_idx >= N) continue;
                const FMTShadow& y_shadow = shadow[y_idx];
                if (!y_shadow.in_queue) continue; // y must be in V_open

                double cost_via_y = y_shadow.cost + edge_info_xy.distance;
                if (cost_via_y < best_cost) {
                    best_cost = cost_via_y;
                    best_parent_idx = y_idx;
                    best_traj = edge_info_xy.cached_trajectory;
                }
            }

            if (best_parent_idx < 0) continue;

            // Lazy collision check
            bool obstacle_free = true;
            for (const auto& [name, ob] : previous_obstacles_) {
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*best_traj, ob)) {
                    obstacle_free = false;
                    break;
                }
            }
            if (obstacle_free) {
                x_shadow.cost = best_cost;
                x_shadow.parent_idx = best_parent_idx;
                x_shadow.parent_traj = best_traj;
                x_shadow.in_unvisited = false;
                newly_open.push_back(x_idx);
            }
        }

        // Add newly connected nodes to the queue
        for (int x_idx : newly_open) {
            addProxy(x_idx, shadow[x_idx].cost);
        }

        // Remove current z from queue
        fmt_queue.pop();
        shadow[z_idx].in_queue = false;


    }

    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    RCLCPP_INFO(rclcpp::get_logger("SOHEIL"), "FMT* took: %.2f ms", duration);
    FMTX_INFO("[runFMT] FMT* expansion complete. checks=" << checks << "  radius=" << fmt_radius);

    // Store shadow results for later visualization
    fmt_shadow_cost_.assign(N, std::numeric_limits<double>::infinity());
    fmt_shadow_parent_.assign(N, -1);
    for (int i = 0; i < N; ++i) {
        fmt_shadow_cost_[i] = shadow[i].cost;
        fmt_shadow_parent_[i] = shadow[i].parent_idx;
    }
    fmt_shadow_valid_ = true;


    // ---------------------------------------------------------------
    // 5. COMPARISON & PARENT DIAGNOSTIC
    // ---------------------------------------------------------------
    FMTX_INFO("[runFMT] ===== FMT* vs FMTX LMC Comparison =====");

    int    total_nodes          = 0;
    int    reachable_by_fmt     = 0;
    int    reachable_by_fmtx    = 0;
    int    both_reachable       = 0;
    int    fmt_strictly_better  = 0;
    int    fmtx_strictly_better = 0;
    int    cost_matches         = 0;
    double max_gap_fmt_better   = 0.0;
    double max_gap_fmtx_better  = 0.0;

    for (const auto& node_ptr : tree_) {
        if (!node_ptr) continue;
        int idx = node_ptr->getIndex();
        if (idx < 0 || idx >= N) continue;

        ++total_nodes;

        double fmt_cost  = shadow[idx].cost;
        double fmtx_lmc  = node_ptr->getLMC();

        bool fmt_reached  = std::isfinite(fmt_cost);
        bool fmtx_reached = std::isfinite(fmtx_lmc);

        if (fmt_reached)                     ++reachable_by_fmt;
        if (fmtx_reached)                    ++reachable_by_fmtx;
        if (fmt_reached && fmtx_reached)     ++both_reachable;

        if (!fmt_reached && !fmtx_reached) continue;

        if (fmt_reached && fmtx_reached) {
            double gap = std::abs(fmt_cost - fmtx_lmc);
            if (gap <= std::numeric_limits<double>::epsilon()) {
                ++cost_matches;
            } else if (fmt_cost < fmtx_lmc) {
                ++fmt_strictly_better;
                max_gap_fmt_better = std::max(max_gap_fmt_better, gap);

                // --- PARENT DIAGNOSTIC ---
                int best_fmt_parent_idx = shadow[idx].parent_idx;
                double parent_fmtx_lmc = -1.0;
                bool is_parent_in_queue = false;
                bool is_edge_in_fmtx_map = false;
                bool is_edge_collision_free = false;

                if (best_fmt_parent_idx >= 0) {
                    FMTNode* fmt_parent = tree_[best_fmt_parent_idx].get();
                    parent_fmtx_lmc = fmt_parent->getLMC();
                    is_parent_in_queue = fmt_parent->in_queue_;
                    
                    // Check if FMTX knows about this edge (Note: child -> parent is backward)
                    auto it = node_ptr->forwardNeighbors().find(fmt_parent);
                    if (it != node_ptr->forwardNeighbors().end()) {
                        is_edge_in_fmtx_map = true;
                        
                        // Check if FMTX thinks the edge is collision-free
                        is_edge_collision_free = true;
                        for (const auto& [name, ob] : previous_obstacles_) {
                            if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(it->second.cached_trajectory), ob)) {
                                is_edge_collision_free = false;
                                break;
                            }
                        }
                    }
                }

                int fmtx_parent_idx = -1;
                if (node_ptr->getParent() != nullptr) {
                    fmtx_parent_idx = node_ptr->getParent()->getIndex();
                }

                FMTX_WARN("[runFMT] Node " << idx <<" state: "<<tree_[idx]->getStateValue()<<","
                    << " | FMT*=" << fmt_cost << " < FMTX=" << fmtx_lmc << " | Gap=" << gap
                    << "\n      -> FMT* used Parent " << best_fmt_parent_idx <<" with lmc of: "<<shadow[best_fmt_parent_idx].cost<<", "
                    << " (Parent's FMTX LMC=" << parent_fmtx_lmc << ", in_queue=" << is_parent_in_queue << ")"
                    << "\n      -> FMTX used Parent " << fmtx_parent_idx
                    << "\n      -> Edge in FMTX map? " << (is_edge_in_fmtx_map ? "YES" : "NO")
                    << " | Collision-free? " << (is_edge_collision_free ? "YES" : "NO"));
                // ---------------------------

                ++metrics.nodes_with_cost_gap;
                metrics.total_cost_gap += gap;
                metrics.max_cost_gap = std::max(metrics.max_cost_gap, gap);
            } else {
                ++fmtx_strictly_better;
                max_gap_fmtx_better = std::max(max_gap_fmtx_better, fmt_cost - fmtx_lmc); // note: fmt_cost > fmtx_lmc

                // // --- PARENT DIAGNOSTIC FOR FMTX BETTER ---
                // int best_fmt_parent_idx = shadow[idx].parent_idx;
                // double parent_fmtx_lmc = -1.0;
                // bool is_parent_in_queue = false;
                // bool is_edge_in_fmtx_map = false;
                // bool is_edge_collision_free = false;

                // // Get FMT* parent info (if any)
                // if (best_fmt_parent_idx >= 0) {
                //     FMTNode* fmt_parent = tree_[best_fmt_parent_idx].get();
                //     parent_fmtx_lmc = fmt_parent->getLMC(); // cost of that parent in dynamic tree
                //     is_parent_in_queue = fmt_parent->in_queue_;

                //     // Check if FMTX knows about this edge (child -> parent)
                //     auto it = node_ptr->forwardNeighbors().find(fmt_parent);
                //     if (it != node_ptr->forwardNeighbors().end()) {
                //         is_edge_in_fmtx_map = true;
                //         is_edge_collision_free = true;
                //         for (const auto& [name, ob] : previous_obstacles_) {
                //             if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(it->second.cached_trajectory), ob)) {
                //                 is_edge_collision_free = false;
                //                 break;
                //             }
                //         }
                //     }
                // }

                // int fmtx_parent_idx = -1;
                // if (node_ptr->getParent() != nullptr) {
                //     fmtx_parent_idx = node_ptr->getParent()->getIndex();
                // }

                // // Log the improvement
                // FMTX_INFO("[runFMT] Node " << idx << " state: " << tree_[idx]->getStateValue().transpose()
                //     << " | FMTX=" << fmtx_lmc << " < FMT*=" << fmt_cost << " | Gap=" << (fmt_cost - fmtx_lmc)
                //     << "\n      -> FMTX used Parent " << fmtx_parent_idx
                //     << "\n      -> FMT* would have used Parent " << best_fmt_parent_idx
                //     << " (FMT* parent cost in FMTX tree = " << parent_fmtx_lmc << ", in_queue=" << is_parent_in_queue << ")"
                //     << "\n      -> Edge (FMT* parent) in FMTX map? " << (is_edge_in_fmtx_map ? "YES" : "NO")
                //     << " | Collision‑free? " << (is_edge_collision_free ? "YES" : "NO"));
            }
        } else if (fmt_reached && !fmtx_reached) {
            ++fmt_strictly_better;
            FMTX_WARN("[runFMT] Node " << idx << " Reached by FMT* but INF in FMTX!");
        } else {
            ++fmtx_strictly_better;
        }
    }

    FMTX_INFO("[runFMT] FMT* strictly better : " << fmt_strictly_better << " max_gap=" << max_gap_fmt_better);
    FMTX_INFO("[runFMT] FMTX strictly better : " << fmtx_strictly_better << " max_gap=" << max_gap_fmtx_better);



}


void KinodynamicFMTX::plan() {
  
#if DEBUG
    SuboptimalityMetrics dbg_metrics;
#endif

    // while (!v_open_heap_.empty() &&
    //        (partial_update ? (robot_node_== nullptr || v_open_heap_.top().first < robot_node_->getLMC() + bridge_cost_||
    //                            robot_node_->getLMC() == std::numeric_limits<double>::infinity() || robot_node_->in_queue_ == true) : true)) {

    while (!v_open_heap_.empty() &&
          (!partial_update || 
                robot_node_ == nullptr || 
                robot_node_->in_queue_ ||
                robot_node_->getLMC() == std::numeric_limits<double>::infinity() ||
                // v_open_heap_.top().first < robot_node_->getLMC() + bridge_cost_))
                v_open_heap_.top().first < robot_node_->getLMC()))
    {

    
        auto top_element = v_open_heap_.top();
        double cost = top_element.first;
        FMTNode* z = top_element.second;
        int zIndex = z->getIndex();
        // VISUALIZATION: Visualize all nodes currently in the Open Set (V_open)
#if VIS
        if (visualization_) {
            std::vector<Eigen::VectorXd> open_set_nodes;
            // const auto& heap_data = v_open_heap_.getHeap();
            // for (const auto& element : heap_data) {
            //     FMTNode* node = element.second;
            //     if (node) {
            //         open_set_nodes.push_back(node->getStateValue());
            //     }
            // }
            open_set_nodes.push_back(z->getStateValue());
            if (!open_set_nodes.empty()) {
                visualization_->visualizeNodes(open_set_nodes, "map", std::vector<float>{1.0f, 0.0f, 0.0f}, "v_open_heap_nodes");
            }
        }
#endif
        /*
            NOTE ON THE 'g' VALUE IN NON-ANYTIME FMTX:
            In standard dynamic algorithms (like D* Lite or RRTx), the 'g' variable acts as a 
            historical tracker. The inconsistency between 'g' and 'lmc' (g != lmc) is exactly 
            what triggers a node to propagate its cost to neighbors (lazy propagation).

            However, in this non-anytime version of FMTX, the 'g' variable is fundamentally optional:
            
            1. EAGER TOPOLOGICAL PROPAGATION: Instead of relying on the (g != lmc) condition to 
            lazily trigger updates, this architecture uses an aggressive, eager approach. The 
            moment node 'z' is popped, we strictly push its 'lmc' down to all its topological 
            children (via the children_ array). Thus, we don't need 'g' to remind the algorithm 
            whether a node has propagated its information or not.
            
            2. NO EPSILON BOUNCER REQUIRED: In anytime algorithms, 'g' is heavily used to calculate 
            |g - lmc| > epsilon to prevent infinite floating-point update loops caused by continuous 
            rewiring. Since this version executes discrete, single-shot replanning cycles 
            (non-anytime), this floating-point filtering is unnecessary.

            We keep `z->setG(z->getLMC());` here strictly for architectural consistency with the 
            anytime variant of the codebase and to maintain valid data for debugging forensics.
        */
        z->setG(z->getLMC());





        /*
            TOPOLOGICAL TREE PROPAGATION (The Strict Bellman Update)
            This phase strictly enforces cost propagation down the shortest-path tree (g(x) = g(P) + c(P,x)).
            If z's cost drops, ALL of its children must inherit the improvement. 
            regardless of whether they have been geometrically culled!
            
            This loop guarantees the survival of the anytime dynamic loop by solving TWO major 
            causes of "Cascade Death" (where a node is permanently stranded with a stale cost):
            
            1. THE FMTX ASYMMETRIC CULLING PROBLEM:
                'cullNeighbors' aggressively deletes long geometric edges to maintain O(log N) density.
                If a parent 'z' culled child 'x' from its outgoing edges, the standard geometric 
                wavefront physically cannot reach 'x'. This topological loop explicitly bridges that 
                severed gap, ensuring 'x' receives the update.
            
            2. THE INHERENT FMT* LAZY COLLISION PROBLEM:
                In standard FMT*, if a node tries to rewire to a new parent but fails a lazy collision 
                check, the node is simply ignored. If its *current* parent had also dropped in cost, 
                ignoring the node permanently strands it, breaking the optimal substructure for its 
                entire subtree. 
                By pushing updates blindly down the 'children_' array (with ZERO collision checks, 
                since the parent-child edge is already known to be safe), we guarantee the Bellman 
                equation survives even if the node later fails to find a better shortcut.
            
            PHILOSOPHY: 
            We accept that a node might miss a shortcut due to lazy collision checking in Phase 2, 
            but we REFUSE to let the tree's mathematical cost structure break. Because there is a
            chance that z cannot propagate its improvement in Phase 2 
            
            NO REDUNDANT WORK:
            Phase 1 updates the cost of z's children immediately. When Phase 2 (the geometric wavefront) 
            later loops over z's neighbors, the core condition `if (x->getLMC() > cost_via_z)` naturally 
            evaluates to FALSE for z's own children. Therefore, Phase 2 completely skips them, saving 
            expensive collision checks. Rewiring a child to a *different* parent naturally happens 
            later when that competing parent expands (i.e., later queue expansions)



            the only concern is do we need collision check here or not? the plan cycles only happen after the updateObstacleSample function gets triggered due to obstalce's turnaround
            in addNewObstacle we make nodes with trajectory in the obstacle's tube orphan and make their cost inf and sever their parent relationship! but we keep the tree nodes that are collision free even though they are in the tube
            so this cant even trigger the else if because there is no parent!
            but how about removeObstacle? this frees up some nodes that was severed in the addNewObstacle so they are still having inf cost with no parent.
            in both cases we need to check the collision in the standard plan cycle! but i dont think when a node is already part of the tree and has parent is caused by addNewObstacle or removeObstacle! and i think its safe to just update the cost
            of that node with its current parent! but if the parent is something other than the current parent we need to collision check which happens in the standard part above!
            though i might be wrong and need to further investigate!  
        */

        for (FMTNode* child : z->children_) {
            auto traj = child->getParentTrajectory();
            if (!traj) continue;

            double cost_via_z = z->getG() + traj->cost;
            
            // If the parent brings a better cost, push it down to the child
            if (child->getLMC() > cost_via_z) {
                child->setLMC(cost_via_z);
                
                // Wake the child up so it can propagate the cost to its own children
                if (child->in_queue_) {
                    v_open_heap_.update(child, cost_via_z);
                } else {
                    v_open_heap_.add(child, cost_via_z); 
                }
            }
        }


        // if (!neighbor_precache)
        //     near(z->getIndex());
        for (auto& [x, edge_info_x_to_z] : z->backwardNeighbors()) { // Backward means incoming . Forward is outgoing
            // if (!neighbor_precache)
            //     near(x->getIndex());
            const Trajectory& traj_xz = *(edge_info_x_to_z.cached_trajectory);
            if (!traj_xz.is_valid) {
                continue;
            }
       
            double cost_via_z = z->getG() + edge_info_x_to_z.distance;
            /*
                This condition is the core of FMTX. It serves two purposes:
                If x has not been connected yet (cost is INF), this is always true, triggering its initial connection.
                If x is already connected, this condition acts as a "witness" that a better path might exist.
                It proves x's current cost is suboptimal and justifies the more expensive search that follows.
                This adds implicit rewiring to FMTX. There is no explicit rewiring here (like RRTX). It repairs the graph
                as wavefront expands.  

                So this condtions is just an Alarm Bell Saying:
                Hey! The cost landscape around node x has dropped! x is currently overpriced!
            */
            if (x->getLMC() > cost_via_z) {

#if DEBUG
                // Check if we've already updated this node in this plan() cycle
                if (dbg_metrics.costUpdated.find(x) != dbg_metrics.costUpdated.end()) {
                    dbg_metrics.revisits++;
                }
#endif


                /*
                    'x' is suboptimal. We now search for its true best parent among ALL its neighbors
                    that are currently in the open set.
                */
                // double min_cost_for_x = std::numeric_limits<double>::infinity();
                double min_cost_for_x = x->getLMC();
                FMTNode* best_parent_for_x = nullptr;
                std::shared_ptr<Trajectory> best_traj_for_x;
                for (auto& [y, edge_info_xy] : x->forwardNeighbors()) {
                    if (y->in_queue_) { // We only consider parents that are in V_open.
                        auto traj_xy = edge_info_xy.cached_trajectory;
                        if (traj_xy->is_valid) {
                            double cost_via_y = y->getLMC() + traj_xy->cost;
                            if (cost_via_y < min_cost_for_x) {
                                min_cost_for_x = cost_via_y;
                                best_parent_for_x = y;
                                best_traj_for_x = traj_xy;
                            }
                        }
                    }
                }

//                 // If a better parent was found
//                 if (best_parent_for_x != nullptr) {
//                     bool obstacle_free = true;
                
//                     if (best_parent_for_x != x->getParent()) {
// #if USE_THREAT_SET_STRATEGY
//                         // Context-Aware
//                         if (!x->threats.empty()) {
//                             for (const Obstacle* ob_ptr : x->threats) {
//                                 last_replan_metrics_.obstacle_checks++;
//                                 if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*best_traj_for_x, *ob_ptr)) {
//                                     obstacle_free = false;
//                                     break; 
//                                 }
//                             }
//                         } else {
//                             obstacle_free = true;
//                         }
// #else
//                         // Default
//                         for (const auto& [obs_name, ob] : previous_obstacles_) {
//                             last_replan_metrics_.obstacle_checks++;
//                             if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*best_traj_for_x, ob)) {
//                                 obstacle_free = false;
//                                 break;
//                             }
//                         }
// #endif
//                     }

               if (best_parent_for_x != nullptr ) { 

                    bool obstacle_free = true;
                    if (best_parent_for_x != x->getParent()) {

                        // --- 1. STATIC CACHE BYPASS ---
                        // Fetch the specific edge struct for this connection
                        auto& edge_to_check = x->forwardNeighbors().at(best_parent_for_x);
                        if (edge_to_check.permanently_blocked) {
                            // We already know this edge hits a static wall. It will never be valid.
                            // We act as if the collision check failed, and skip adopting this parent.
                            obstacle_free = false;
                        } 
                        else {
                            // --- 2. PERFORM COLLISION CHECK ---
#if USE_THREAT_SET_STRATEGY
                            if (!x->threats.empty()){
                                for (const Obstacle* ob_ptr : x->threats) {
                                    last_replan_metrics_.obstacle_checks++;
                                    if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*best_traj_for_x, *ob_ptr)) {
                                        obstacle_free = false;
                                        // Cache if the threat is static
                                        if (!ob_ptr->is_dynamic) {
                                            edge_to_check.permanently_blocked = true;
                                        }
                                        break;
                                    }
                                }
                            } else {
                                obstacle_free = true;
                            }
#else
                            // Use previous_obstacles_ so the brute-force mode actually sees the tubes!
                            for (const auto& [name, ob] : previous_obstacles_) {
                                last_replan_metrics_.obstacle_checks++;
                                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*best_traj_for_x, ob)) {
                                    obstacle_free = false;
                                    // Cache if the threat is static
                                    if (!ob.is_dynamic) {
                                        edge_to_check.permanently_blocked = true;
                                    }
                                    break;
                                }
                            }
#endif
                        }
                    }



 
                    if (obstacle_free) {
                        
                        x->setLMC(min_cost_for_x);
                        x->setParent(best_parent_for_x, best_traj_for_x);

#if DEBUG
                        dbg_metrics.costUpdated[x] = true;
                        // Oracle!
                        analyzeSuboptimality(x, best_parent_for_x, z, dbg_metrics);
#endif
                        double priorityCost = min_cost_for_x;
                        if (x->in_queue_) {
                            v_open_heap_.update(x, priorityCost);
                        } else {
                            v_open_heap_.add(x, priorityCost); // add() also sets in_queue_ = true
                        }
                    }
                }
            }
        }


        v_open_heap_.pop();
        // visualizeTree();
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

    } 


#if DEBUG
    runFMT(dbg_metrics);
    // compareHeapLogs(shadow_log);

    // printDebugSummary(dbg_metrics); // Experimental proof that increasing sample counts would reduce the average suboptimality cost which happens in FMTX (Inherited from FMT*)
    // runCollisionForensics(); // To Recheck All the trajectories again with all the obstacles to validify our repair!
    // runCostForensics();
    // runGlobalCostForensics();
#endif


}


// void KinodynamicFMTX::near(int node_index) {
//     auto node = tree_[node_index].get();
//     if (node->neighbors_cached_) return;

//     std::vector<size_t> candidate_indices;
//     if (use_knn) {
//         if (k_neighbors_ > 0) {
//             candidate_indices = kdtree_->knnSearch(node->getStateValue().head(kd_dim), k_neighbors_);
//         }
//     } else {
//         if (neighborhood_radius_ > 0) {
//             candidate_indices = kdtree_->radiusSearch(node->getStateValue().head(kd_dim), neighborhood_radius_ + std::numeric_limits<double>::epsilon());
//         }
//     }

//     for (int idx : candidate_indices) {
//         if (idx == node_index) continue;
//         FMTNode* neighbor = tree_[idx].get();

//         // Test FORWARD connection
//         Trajectory traj_forward = statespace_->steer(node->getStateValue(), neighbor->getStateValue());
        
//         if (traj_forward.is_valid && (use_knn || traj_forward.cost <= neighborhood_radius_ + std::numeric_limits<double>::epsilon())) {
            
//             auto shared_traj_forward = std::make_shared<Trajectory>(std::move(traj_forward));
            
//             EdgeInfo edge_fwd;
//             edge_fwd.distance = shared_traj_forward->cost;
//             edge_fwd.distance_original = shared_traj_forward->cost;
//             edge_fwd.is_initial = true;
//             edge_fwd.cached_trajectory = shared_traj_forward;
//             edge_fwd.is_trajectory_computed = true;

//             node->forwardNeighbors()[neighbor] = edge_fwd;
//             neighbor->backwardNeighbors()[node] = edge_fwd;

//             // OPTIMIZATION FOR GEOMETRIC CASE 
//             if (is_geometric_mode_) {
//                 // In geometric mode, backward is identical to forward.
//                 // We reuse the exact same EdgeInfo (and the same shared_ptr)
//                 node->backwardNeighbors()[neighbor] = edge_fwd;
//                 neighbor->forwardNeighbors()[node] = edge_fwd;
//             } else {
//                 // KINODYNAMIC CASE
//                 // Test BACKWARD connection (Neighbor -> Node)
//                 Trajectory traj_backward = statespace_->steer(neighbor->getStateValue(), node->getStateValue());
//                 if (traj_backward.is_valid && (use_knn || traj_backward.cost <= neighborhood_radius_ + std::numeric_limits<double>::epsilon())) {
                    
//                     auto shared_traj_backward = std::make_shared<Trajectory>(std::move(traj_backward));
                    
//                     EdgeInfo edge_bwd;
//                     edge_bwd.distance = shared_traj_backward->cost;
//                     edge_bwd.distance_original = shared_traj_backward->cost;
//                     edge_bwd.is_initial = true;
//                     edge_bwd.cached_trajectory = shared_traj_backward;
//                     edge_bwd.is_trajectory_computed = true;
                    
//                     node->backwardNeighbors()[neighbor] = edge_bwd;
//                     neighbor->forwardNeighbors()[node] = edge_bwd;
//                 }
//             }
//         }
//     }
    
//     node->neighbors_cached_ = true;
// }


void KinodynamicFMTX::near(int node_index) {
    auto node = tree_[node_index].get();
    if (node->neighbors_cached_) return;

    std::vector<size_t> candidate_indices;
    if (use_knn) {
        if (k_neighbors_ > 0) {
            candidate_indices = kdtree_->knnSearch(node->getStateValue().head(kd_dim), k_neighbors_);
        }
    } else {
        if (neighborhood_radius_ > 0) {
            candidate_indices = kdtree_->radiusSearch(node->getStateValue().head(kd_dim), neighborhood_radius_ + std::numeric_limits<double>::epsilon());
        }
    }

    for (int idx : candidate_indices) {
        if (idx == node_index) continue;
        FMTNode* neighbor = tree_[idx].get();

        // 1. Test FORWARD connection (Node -> Neighbor)
        Trajectory traj_forward = statespace_->steer(node->getStateValue(), neighbor->getStateValue());
        std::shared_ptr<Trajectory> shared_traj_forward;
        
        bool is_forward_valid = traj_forward.is_valid && 
                                (use_knn || traj_forward.cost <= neighborhood_radius_ + std::numeric_limits<double>::epsilon());

        if (is_forward_valid) {
            shared_traj_forward = std::make_shared<Trajectory>(std::move(traj_forward));
            
            EdgeInfo edge_fwd;
            edge_fwd.distance = shared_traj_forward->cost;
            edge_fwd.distance_original = shared_traj_forward->cost;
            edge_fwd.is_initial = true;
            edge_fwd.cached_trajectory = shared_traj_forward;
            edge_fwd.is_trajectory_computed = true;

            node->forwardNeighbors()[neighbor] = edge_fwd;
            neighbor->backwardNeighbors()[node] = edge_fwd;
        }

        // 2. Test BACKWARD connection (Neighbor -> Node)
        if (is_geometric_mode_) {
            // OPTIMIZATION FOR GEOMETRIC CASE 
            // In geometric mode, backward is identical to forward.
            // Only assign if forward was valid!
            if (is_forward_valid && shared_traj_forward) {
                EdgeInfo edge_bwd;
                edge_bwd.distance = shared_traj_forward->cost;
                edge_bwd.distance_original = shared_traj_forward->cost;
                edge_bwd.is_initial = true;
                edge_bwd.cached_trajectory = shared_traj_forward; // Reuse same shared_ptr
                edge_bwd.is_trajectory_computed = true;

                node->backwardNeighbors()[neighbor] = edge_bwd;
                neighbor->forwardNeighbors()[node] = edge_bwd;
            }
        } else {
            // KINODYNAMIC CASE - INDEPENDENT CHECK
            // We check this regardless of whether the forward connection succeeded or failed!
            Trajectory traj_backward = statespace_->steer(neighbor->getStateValue(), node->getStateValue());
            
            if (traj_backward.is_valid && (use_knn || traj_backward.cost <= neighborhood_radius_ + std::numeric_limits<double>::epsilon())) {
                
                auto shared_traj_backward = std::make_shared<Trajectory>(std::move(traj_backward));
                
                EdgeInfo edge_bwd;
                edge_bwd.distance = shared_traj_backward->cost;
                edge_bwd.distance_original = shared_traj_backward->cost;
                edge_bwd.is_initial = true;
                edge_bwd.cached_trajectory = shared_traj_backward;
                edge_bwd.is_trajectory_computed = true;
                
                node->backwardNeighbors()[neighbor] = edge_bwd;
                neighbor->forwardNeighbors()[node] = edge_bwd;
            }
        }
    }
    
    node->neighbors_cached_ = true;
}



std::vector<Eigen::VectorXd> KinodynamicFMTX::getPathPositions() const
{
    // Check if the planner has a valid anchor point for the robot (setRobotState should have found one)
    if (!robot_node_ || robot_node_->getLMC() == std::numeric_limits<double>::infinity()) {
        FMTX_ERROR("FMTX_Path_Assembly: Robot has no valid anchor node. Cannot build path");
        return {}; // Return empty path
    }

    // // Generate the "bridge" trajectory from the robot's continuous state to the anchor node on the fly
    // Trajectory bridge_traj = statespace_->steer(robot_continuous_state_, robot_node_->getStateValue());

    // if (!bridge_traj.is_valid) {
    //     FMTX_ERROR("FMTX_Path_Assembly: Failed to steer from robot's continuous state to the anchor node.");
    //     return {};
    // }

    // Safety check on the cached bridge
    if (!current_bridge_trajectory_.is_valid || current_bridge_trajectory_.path_points.empty()) {
        FMTX_ERROR("FMTX_Path_Assembly: Cached bridge trajectory is invalid. Cannot build path");
        return {};
    }

    // Start the final path with the CACHED bridge trajectory! (Zero computation time)
    std::vector<Eigen::VectorXd> final_executable_path = current_bridge_trajectory_.path_points;
    // // Start the final path with this bridge trajectory.
    // std::vector<Eigen::VectorXd> final_executable_path = bridge_traj.path_points;

    // Traverse the rest of the tree from the anchor node using parent pointers.
    FMTNode* child = robot_node_;
    FMTNode* parent = child->getParent();

    while (parent) {
        auto cached_traj = child->getParentTrajectory();
        if (cached_traj->is_valid && cached_traj->path_points.size() > 1) {
            // Append all points from the segment except the first one to avoid duplicates.
            final_executable_path.insert(final_executable_path.end(),
                                         cached_traj->path_points.begin() + 1,
                                         cached_traj->path_points.end());
        } else {
            // If a valid cached trajectory doesn't exist, the path is broken.
            FMTX_WARN("[FMTX_Path_Assembly] Path reconstruction failed. Invalid cached trajectory between nodes " 
                << child->getIndex() << " and " << parent->getIndex() << ".");
            break;
        }
        child = parent;
        parent = child->getParent();
    }

    return final_executable_path;
}


// std::vector<Eigen::VectorXd> KinodynamicFMTX::getPathPositions() const
// {
//     // Check if the planner has a valid anchor point for the robot 
//     if (!robot_node_) {
//         // Fallback: If no anchor exists but we have an emergency trajectory, execute it!
//         if (current_bridge_trajectory_.is_valid && !current_bridge_trajectory_.path_points.empty()) {
//             return current_bridge_trajectory_.path_points;
//         }
//         FMTX_ERROR("[FMTX_Path_Assembly] No anchor node and no safe emergency trajectory.");
//         return {}; 
//     }

//     // Safety check on the cached bridge
//     if (!current_bridge_trajectory_.is_valid || current_bridge_trajectory_.path_points.empty()) {
//         FMTX_ERROR("[FMTX_Path_Assembly] Cached bridge trajectory is invalid. Cannot build path.");
//         return {};
//     }

//     // Start the final path with the CACHED bridge trajectory! (Zero computation time)
//     std::vector<Eigen::VectorXd> path = current_bridge_trajectory_.path_points;

//     // Traverse the rest of the tree from the anchor node using parent pointers.
//     FMTNode* current_node = robot_node_;
    
//     // Cycle Detection: Counter-based approach (faster than unordered_set hashing)
//     int steps = 0;
//     const int max_steps = tree_.size();

//     while (current_node) {
//         if (steps++ > max_steps) {
//             FMTX_WARN("[FMTX_Path_Assembly] Cycle detected (max steps exceeded). Aborting further reconstruction.");
//             break;
//         }

//         FMTNode* next_node = current_node->getParent();
//         if (!next_node) {
//             break; // Reached the root/goal of the tree
//         }

//         auto traj = current_node->getParentTrajectory();
        
//         // Check if cached_traj exists and is valid
//         if (traj && traj->is_valid && traj->path_points.size() > 1) {
//             // Skip the first point to avoid duplicate overlapping points at the nodes
//             path.insert(path.end(), traj->path_points.begin() + 1, traj->path_points.end());
//         } else {
//             // Fallback: If trajectory details are missing, just push the node's state
//             path.push_back(next_node->getStateValue());
//         }
        
//         current_node = next_node;
//     }

//     return path;
// }


// std::vector<Eigen::VectorXd> KinodynamicFMTX::getPathPositions() const
// {
//     // Check if the planner has a valid anchor point for the robot
//     if (!robot_node_) {  // ← ONLY check for null pointer, not LMC!
//         FMTX_ERROR("FMTX_Path_Assembly: Robot has no valid anchor node. Cannot build path");
//         return {}; 
//     }

//     // Safety check on the cached bridge
//     if (!current_bridge_trajectory_.is_valid || current_bridge_trajectory_.path_points.empty()) {
//         FMTX_ERROR("FMTX_Path_Assembly: Cached bridge trajectory is invalid. Cannot build path");
//         return {};
//     }

//     // Start the final path with the CACHED bridge trajectory!
//     std::vector<Eigen::VectorXd> final_executable_path = current_bridge_trajectory_.path_points;

//     // Traverse the rest of the tree from the anchor node using parent pointers.
//     FMTNode* child = robot_node_;
//     FMTNode* parent = child->getParent();

//     while (parent) {
//         auto cached_traj = child->getParentTrajectory();
//         if (cached_traj->is_valid && cached_traj->path_points.size() > 1) {
//             final_executable_path.insert(final_executable_path.end(),
//                                          cached_traj->path_points.begin() + 1,
//                                          cached_traj->path_points.end());
//         } else {
//             // Recovery nodes have no parent chain, so just stop here.
//             // This is expected behavior!
//             break;
//         }
//         child = parent;
//         parent = child->getParent();
//     }

//     return final_executable_path;
// }



void KinodynamicFMTX::setStart(const Eigen::VectorXd& start) {
    root_state_index_ = statespace_->getNumStates();
    auto node = std::make_unique<FMTNode>(statespace_->addState(start),tree_.size());
    node->setLMC(0);
    node->setTimeToGoal(0);
    v_open_heap_.add(node.get(),0);
    tree_.push_back(std::move(node));
    std::cout << "KinodynamicFMTX: Start node created on Index: " << root_state_index_ << "\n";
}
void KinodynamicFMTX::setGoal(const Eigen::VectorXd& goal) {
    robot_state_index_ = statespace_->getNumStates();
    auto node = std::make_unique<FMTNode>(statespace_->addState(goal),tree_.size());
    node->setTimeToGoal(std::numeric_limits<double>::infinity());
    robot_node_ = node.get();
    tree_.push_back(std::move(node));
    std::cout << "KinodynamicFMTX: Goal node created on Index: " << robot_state_index_ << "\n";
}


void KinodynamicFMTX::visualizeTreeGradient() {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    std::vector<double> edge_costs;

    if (!tree_.empty()) {
        edges.reserve(tree_.size());
        edge_costs.reserve(tree_.size());
    }
    
    // Find max cost in the tree to normalize the gradient
    double max_tree_cost = 0.001; 
    for (const auto& node_ptr : tree_) {
        if (node_ptr->getLMC() > max_tree_cost && !std::isinf(node_ptr->getLMC())) {
            max_tree_cost = node_ptr->getLMC();
        }
    }

    for (const auto& node_ptr : tree_) {
        FMTNode* child_node = node_ptr.get();
        FMTNode* parent_node = child_node->getParent();

        if (parent_node && !std::isinf(child_node->getLMC())) {
            // Use the full states so visualization has access to Z or time if needed
            edges.emplace_back(parent_node->getStateValue(), child_node->getStateValue());
            
            // Normalize the cost between 0.0 and 1.0
            edge_costs.push_back(child_node->getLMC() / max_tree_cost); 
        }
    }
    
    if (robot_node_) {
        // Visualize the anchor node itself as a big dot
        std::vector<Eigen::VectorXd> anchor_pt = { robot_node_->getStateValue().head<2>() };
        visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");
    } 
    visualization_->visualizeTreeGradient(edges, edge_costs, "map");
}


void KinodynamicFMTX::visualizeFMTtree() {
    if (!visualization_ || !fmt_shadow_valid_) return;

    const int N = static_cast<int>(tree_.size());

    // --- Full FMT* tree (green) ---
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> fmt_tree_edges;
    for (int idx = 0; idx < N; ++idx) {
        if (!tree_[idx]) continue;
        double cost = fmt_shadow_cost_[idx];
        if (!std::isfinite(cost)) continue;
        int parent = fmt_shadow_parent_[idx];
        if (parent >= 0 && parent < N && tree_[parent]) {
            fmt_tree_edges.emplace_back(
                tree_[parent]->getStateValue().head(2),
                tree_[idx]->getStateValue().head(2)
            );
        }
    }
    // visualization_->visualizeEdges(
    //     fmt_tree_edges, "map",
    //     std::array<float,3>{0.0f, 1.0f, 0.0f}, 1.0f, 0.15f,
    //     "fmt_full_tree", 300, true, 0.5
    // );

    // --- Disagreement edges ---
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> fmt_better_edges_red;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> fmtx_worse_edges_blue;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> fmtx_better_edges_magenta;

    for (int idx = 0; idx < N; ++idx) {
        auto node = tree_[idx].get();
        if (!node) continue;

        double fmt_cost = fmt_shadow_cost_[idx];
        double fmtx_lmc = node->getLMC();
        bool fmt_reached = std::isfinite(fmt_cost);
        bool fmtx_reached = std::isfinite(fmtx_lmc);
        if (!(fmt_reached && fmtx_reached)) continue;

        const double eps = std::numeric_limits<double>::epsilon();
        if (fmt_cost + eps < fmtx_lmc) {
            // FMT* strictly better
            int fmt_parent = fmt_shadow_parent_[idx];
            if (fmt_parent >= 0 && fmt_parent < N && tree_[fmt_parent]) {
                fmt_better_edges_red.emplace_back(
                    tree_[fmt_parent]->getStateValue().head(2),
                    node->getStateValue().head(2)
                );
            }
            if (node->getParent()) {
                fmtx_worse_edges_blue.emplace_back(
                    node->getParent()->getStateValue().head(2),
                    node->getStateValue().head(2)
                );
            }
        } else if (fmtx_lmc + eps < fmt_cost) {
            // FMTX strictly better
            if (node->getParent()) {
                fmtx_better_edges_magenta.emplace_back(
                    node->getParent()->getStateValue().head(2),
                    node->getStateValue().head(2)
                );
            }
        }
    }

    // Publish red edges (FMT* better)
    visualization_->visualizeEdges(
        fmt_better_edges_red, "map",
        std::array<float,3>{1.0f, 0.0f, 0.0f}, 1.0f, 0.25f,
        "fmt_better_edges", 301, true, 0.5
    );

    // Publish blue edges (FMTX suboptimal when FMT* is better)
    visualization_->visualizeEdges(
        fmtx_worse_edges_blue, "map",
        std::array<float,3>{0.0f, 0.35f, 1.0f}, 1.0f, 0.25f,
        "fmtx_worse_edges", 302, true, 0.5
    );

    // // Publish magenta edges (FMTX better)
    // visualization_->visualizeEdges(
    //     fmtx_better_edges_magenta, "map",
    //     std::array<float,3>{1.0f, 0.0f, 1.0f}, 1.0f, 0.25f,
    //     "fmtx_better_edges", 303, true, 0.5
    // );


    // --- Sleep only if there is at least one red edge (FMT* strictly better) ---
    if (!fmt_better_edges_red.empty()) {
        visualization_->triggerPublish();
        FMTX_ERROR("[visualizeFMTtree] FMT* strictly better detected — sleeping 5 seconds for RViz inspection.");
        // std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

// straight lines for all the statespaces
void KinodynamicFMTX::visualizeTree() {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    if (!tree_.empty()) {
        edges.reserve(tree_.size());
    }
    
    std::vector<Eigen::VectorXd> tree_nodes;
    tree_nodes.reserve(tree_.size());

    
    for (const auto& node_ptr : tree_) {
        FMTNode* child_node = node_ptr.get();
        FMTNode* parent_node = child_node->getParent();

        tree_nodes.push_back(node_ptr->getStateValue().head(2));

        if (parent_node) {
            edges.emplace_back(parent_node->getStateValue().head(2), child_node->getStateValue().head(2));
        }
    }


    // visualization_->visualizeNodes(tree_nodes, "map", 
    //                         std::vector<float>{0.0f, 1.0f, 0.0f},  // Green color
    //                         "tree_nodes");
    
    if (robot_node_) {
        // Visualize the anchor node itself as a big dot
        std::vector<Eigen::VectorXd> anchor_pt = { robot_node_->getStateValue().head<2>() };
        visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");
    } 
    visualization_->visualizeEdges(edges, "map");

#if DEBUG
    visualizeFMTtree();
#endif

}

// Curvy!
void KinodynamicFMTX::visualizeTreeReal() {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    if (!tree_.empty()) {
        edges.reserve(tree_.size() * 50); 
    }
    
    std::vector<Eigen::VectorXd> tree_nodes;
    tree_nodes.reserve(tree_.size());

    int connected_nodes_count = 0;
    
    for (const auto& node_ptr : tree_) {
        FMTNode* child_node = node_ptr.get();
        FMTNode* parent_node = child_node->getParent();

        tree_nodes.push_back(child_node->getStateValue().head(3));

        if (child_node->getLMC() != std::numeric_limits<double>::infinity()) {
            connected_nodes_count++;
        }

        if (parent_node) {

            auto traj = child_node->getParentTrajectory();

            if (traj->is_valid && traj->path_points.size() > 1) {
                
                for (size_t i = 0; i < traj->path_points.size() - 1; ++i) {
                    edges.emplace_back(traj->path_points[i].head(3), traj->path_points[i+1].head(3));
                }
            } else {
                edges.emplace_back(parent_node->getStateValue().head(3), child_node->getStateValue().head(3));
            }
        }
    }

    
    // Visualization calls
    // visualization_->visualizeNodes(tree_nodes, "map", {0.0f, 1.0f, 0.0f}, "tree_nodes");
    // visualization_->visualizeEdges(edges, "map", "1.0,1.0,1.0", "tree_edges");
    if (robot_node_) {
        // Visualize the anchor node itself as a big dot
        std::vector<Eigen::VectorXd> anchor_pt = { robot_node_->getStateValue().head<2>() };
        visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");
    } 
    visualization_->visualizeEdges(edges, "map");
}


void KinodynamicFMTX::visualizePathGradient(const std::vector<Eigen::VectorXd>& path_waypoints) {
    if (path_waypoints.size() < 2 || !visualization_ || robot_node_ == nullptr) {
        return;
    }
    
    double robot_lmc = robot_node_->getLMC();
    if (std::isinf(robot_lmc)) return;

    double current_robot_cost = robot_lmc + bridge_cost_;
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

void KinodynamicFMTX::visualizePath(const std::vector<Eigen::VectorXd>& path_waypoints) {
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


void KinodynamicFMTX::dumpTreeToCSV(const std::string& filename) const {
    std::ofstream fout(filename);
    if (!fout.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing\n";
        return;
    }
    if (tree_.empty()) {
        std::cerr << "Tree is empty. Nothing to dump.\n";
        return;
    }
    size_t dim = tree_[0]->getStateValue().size();
    fout << "node_id";
    for (size_t d = 0; d < dim; ++d) {
        fout << ",x" << d;
    }
    fout << ",parent_id\n";

    for (const auto& node_ptr : tree_) {
        int nid = node_ptr->getIndex(); 
        auto coords = node_ptr->getStateValue();
        FMTNode* parent = node_ptr->getParent();
        int pid = (parent ? parent->getIndex() : -1);

        fout << nid;
        for (size_t d = 0; d < dim; ++d) {
            fout << "," << std::setprecision(10) << coords[d];
        }
        fout << "," << pid << "\n";
    }
    fout.close();
    std::cout << "Tree dumped to " << filename << "\n";
}




void KinodynamicFMTX::analyzeSuboptimality(FMTNode* x, FMTNode* best_parent_for_x, FMTNode* z, SuboptimalityMetrics& metrics) {
    metrics.total_nodes_updated++;
    int missed_for_this_node = 0;
    double chosen_cost = x->getLMC();
    FMTNode* best_missed = nullptr;
    double best_missed_cost = chosen_cost;
    double witness_cost = z->getLMC();

    // Cache chosen dt/dx
    double chosen_dt = 0.0; double chosen_dx = 0.0;
    if (!is_geometric_mode_ && best_parent_for_x) {
        chosen_dt = x->getTimeToGoal() - best_parent_for_x->getTimeToGoal();
        chosen_dx = (x->getStateValue().head<2>() - best_parent_for_x->getStateValue().head<2>()).norm();
    }

    for (const auto& [y, edge_info_xy] : x->forwardNeighbors()) {
        if (y == best_parent_for_x) continue;

        auto traj_xy = edge_info_xy.cached_trajectory;
        if (!traj_xy || !traj_xy->is_valid) continue;

        double candidate_cost = y->getLMC() + traj_xy->cost;
        if (candidate_cost >= chosen_cost) continue;

        // Collision check
        bool is_safe = true;
#if USE_THREAT_SET_STRATEGY
        if (!x->threats.empty()) {
            for (const Obstacle* ob_ptr : x->threats) {
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*traj_xy, *ob_ptr)) {
                    is_safe = false; break;
                }
            }
        }
#else
        for (const auto& [obs_name, ob] : previous_obstacles_) {
            if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*traj_xy, ob)) {
                is_safe = false; break;
            }
        }
#endif

        if (is_safe) {
            missed_for_this_node++;
            if (!is_geometric_mode_) {
                double missed_dt = x->getTimeToGoal() - y->getTimeToGoal();
                double missed_dx = (x->getStateValue().head<2>() - y->getStateValue().head<2>()).norm();
                std::cout << "[SUBOPT_DEBUG] Node " << x->getIndex()
                        << " | Missed y=" << y->getIndex() << " | Δcost=" << (chosen_cost - candidate_cost) << "\n";
            }
            if (candidate_cost < best_missed_cost) {
                best_missed_cost = candidate_cost;
                best_missed = y;
            }
        }
    }

    if (missed_for_this_node > 0) {
        metrics.nodes_with_missed_better++;
        metrics.total_missed_better_parents += missed_for_this_node;
        double cost_penalty = chosen_cost - best_missed_cost;
        metrics.total_cost_gap += cost_penalty;
        metrics.nodes_with_cost_gap++;
        if (cost_penalty > metrics.max_cost_gap) metrics.max_cost_gap = cost_penalty;
    }
}

void KinodynamicFMTX::printDebugSummary(const SuboptimalityMetrics& metrics) {
    std::cout << "REVISITS: " << metrics.revisits << "\n";
    
    // Final summary
    std::cout << "\n=== Suboptimality summary (this replan) ===\n"
            << "Nodes successfully updated: " << metrics.total_nodes_updated << "\n";

    if (metrics.nodes_with_missed_better > 0) {
        double pct_missed = (metrics.total_nodes_updated > 0) ? 
            (100.0 * metrics.nodes_with_missed_better / metrics.total_nodes_updated) : 0.0;

        std::cout << "Nodes that missed better collision-free parents: "
                << metrics.nodes_with_missed_better
                << " (" << pct_missed << "% of updated nodes)\n";

        std::cout << "Total missed better parents across all nodes: "
                << metrics.total_missed_better_parents << "\n";

        double avg_missed = static_cast<double>(metrics.total_missed_better_parents) /
                            metrics.nodes_with_missed_better;
        std::cout << "Average missed better parents per suboptimal node: "
                << avg_missed << "\n";
    } else {
        std::cout << "No suboptimal connections detected — all updated nodes appear locally optimal\n";
    }

    // Cost-gap suboptimality report
    std::cout << "\n=== Cost-Gap Suboptimality Report ===\n";

    if (metrics.nodes_with_cost_gap > 0) {
        double avg_gap = metrics.total_cost_gap / metrics.nodes_with_cost_gap;
        
        double pct_gap = (metrics.total_nodes_updated > 0) ? 
            (100.0 * metrics.nodes_with_cost_gap / metrics.total_nodes_updated) : 0.0;

        std::cout << "Nodes with measurable cost penalty: " << metrics.nodes_with_cost_gap
                << " (" << pct_gap << "% of updated nodes)\n"
                << "Total accumulated cost penalty: " << metrics.total_cost_gap << "\n"
                << "Average cost gap per affected node: " << avg_gap << "\n"
                << "Worst single cost gap observed: " << metrics.max_cost_gap << "\n";
    } else {
        std::cout << "No measurable cost penalties detected — all updated nodes appear locally optimal in cost\n";
    }
}

bool KinodynamicFMTX::runCollisionForensics() {
    std::cout << "\n[FMTx FORENSICS] --- STARTING GOAL-ROOTED TREE VERIFICATION ---" << std::endl;
    int illegal_connections = 0;
    int checked_nodes = 0;

    for (size_t i = 0; i < tree_.size(); ++i) { 
        FMTNode* node = tree_[i].get();

        if (node->getParent() != nullptr && node->getLMC() != std::numeric_limits<double>::infinity()) {
            checked_nodes++;
            bool edge_collides = false;
            std::string guilty_obstacle = "";
            FMTNode* parent = node->getParent();
            
            Trajectory edge_traj = statespace_->steer(node->getStateValue(), parent->getStateValue());

            // // Verify against Ground Truth
            // for (const auto& [name, ob] : previous_obstacles_) {
            //     if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(edge_traj, ob)) {
            //         edge_collides = true;
            //         guilty_obstacle = name;
            //         break;
            //     }
            // }
            const ObstacleVector& current_obstacles = obs_checker_->getObstacles();
            // Iterate directly over the obstacle objects
            for (const auto& ob : current_obstacles) {
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(edge_traj, ob)) {
                    edge_collides = true;
                    guilty_obstacle = ob.name; // Access name via ob.name
                    break;
                }
            }

            if (edge_collides) {
                illegal_connections++;
                std::cout << "\033[1;31m[VIOLATION]\033[0m Node " << node->getIndex() 
                          << " -> Parent " << parent->getIndex()
                          << " | Cost: " << node->getLMC()
                          << " | \033[1;35mFATAL: Edge hits [" << guilty_obstacle << "]\033[0m\n";
                
                bool in_node_threats = false;
                for (const Obstacle* threat_ptr : node->threats) {
                    if (threat_ptr->name == guilty_obstacle) {
                        in_node_threats = true;
                        break;
                    }
                }

                if (!in_node_threats) {
                    std::cout << "   -> \033[1;33m[!] ANALYSIS:\033[0m Strategy missed it! "
                              << "Obstacle [" << guilty_obstacle << "] was not in the child node's threat set.\n"
                              << "      Check if KD-Tree radius search covers delta + inflation correctly.\n";
                } else {
                    std::cout << "   -> \033[1;31m[!] ANALYSIS:\033[0m Logic failure! "
                              << "Threat was detected, but the edge was still allowed to connect during plan().\n";
                }
            }
        }
    }
    
    if (illegal_connections > 0) {
        std::cout << "[FMTx FORENSICS] --- \033[1;31mFAILED\033[0m: Found " << illegal_connections 
                  << " violations ---\n\n";
        return false;
    } else {
        std::cout << "[FMTx FORENSICS] --- \033[1;32mPASSED\033[0m: Checked " << checked_nodes 
                  << " connections. Tree is 100% Collision-Free ---\n\n";
        return true;
    }

    return true;
}

// The Manager
void KinodynamicFMTX::updateObstacles(const ObstacleVector& turned_obstacles) {
    if (turned_obstacles.empty()) return;

    // if (robot_continuous_state_.size() == 0) {
    //     FMTX_WARN("Planner_Obstacle_Update: Robot state not set.");
    //     return;
    // }


    double T_robot = 0.0;
    if (!is_geometric_mode_) {
        // Only extract T_robot if we are in kinodynamic mode
        if (robot_continuous_state_.size() > 0) {
            T_robot = robot_continuous_state_(robot_continuous_state_.size() - 1);
        }
    }


    for (const auto& incoming_ob : turned_obstacles) {
        
        // Retrieve stored obstacle
        Obstacle& stored_ob = previous_obstacles_[incoming_ob.name];
        // for (auto p: stored_ob.predicted_path){
        //     std::cout<<"old: " <<p<<"\n";
        // }

        // REMOVE OLD TUBE (Wake up neighbors in the freed region)
        // We do this BEFORE updating the object so we use the OLD path.
        if (!stored_ob.predicted_path.empty()) {
            removeObstacle(stored_ob); 
        }

        // UPDATE STATE
        stored_ob = incoming_ob; 

        // GENERATE NEW DENSE TUBE
        // stored_ob.predicted_path = obs_checker_->generatePrediction(stored_ob, T_robot);
        // for (auto p: stored_ob.predicted_path){
        //     std::cout<<"new: "<<p<<"\n";
        // }

        // ADD NEW TUBE (Invalidate nodes in the blocked region)
        addNewObstacle(stored_ob);
    }

    plan();



    // // ==========================================================
    // // PROOF OF TIME PILLAR USAGE: Trace the actual computed path
    // // ==========================================================
    // if (!is_geometric_mode_ && robot_node_) {
    //     std::cout << "\n========== PLANNED PATH TRACE ==========\n";
    //     std::cout << "Robot's Path Cost (getLMC()): " << robot_node_->getLMC() << "\n";
        
    //     FMTNode* current = robot_node_;
        
    //     // In backward search, the problem's 'start' is the physical destination
    //     Eigen::VectorXd goal_spatial = problem_->getStart().head(2); 
        
    //     if (std::isinf(robot_node_->getLMC())) {
    //         std::cout << "NO PATH FOUND (getLMC() = inf)\n";
    //     } else {
    //         while (current != nullptr) {
    //             double curr_t = current->getStateValue()(statespace_->getDimension() - 1);
    //             double curr_x = current->getStateValue()(0);
    //             double curr_y = current->getStateValue()(1);
                
    //             std::cout << "Node " << current->getIndex() 
    //                       << " at [" << std::fixed << std::setprecision(1) << curr_x << ", " << curr_y << "]"
    //                       << " T=" << std::fixed << std::setprecision(2) << curr_t;
                
    //             FMTNode* next_node = current->getParent();
                
    //             // If there is no parent, we have reached a root of the tree!
    //             if (!next_node) {
    //                 if (current->getLMC() == 0.0) {
    //                     double dist_to_goal = (current->getStateValue().head(2) - goal_spatial).norm();
                        
    //                     // Time pillars are injected exactly at the goal location
    //                     if (dist_to_goal <= 1e-3) {
    //                         std::cout << " (MISSION GOAL / TIME PILLAR)\n";
    //                         std::cout << "   ==[ SUCCESSFULLY ARRIVED AT T=" << curr_t << " ]==> \n";
    //                     } else {
    //                         std::cout << " (UNKNOWN ROOT)\n";
    //                     }
    //                 } else {
    //                     std::cout << " -> [BROKEN PATH: No parent but LMC != 0]\n";
    //                 }
    //                 break;
    //             }
                
    //             std::cout << "\n   --[ physical steer ]--> \n";
    //             current = next_node;
    //         }
    //     }
    //     std::cout << "========================================\n\n";
    // }


}


void KinodynamicFMTX::addNewObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    // Calculate Search Radius (as before)
    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    

    /*
        We want the search circle around Center 1 to reach the midpoint (R,R). 
        because the dt calculation in generateprediction creates gaps in between obstalces!
        so to fix this we need to use  Gap Coverage Inflation
        If circles center points are spaced by diameter (2*R_eff), we need sqrt(2) * R_eff to cover the corners.
        If you used the adaptive DT from the previous step, points are spaced by 2*R_eff.
        so we add (sqrt(2)-1)*R because the base radius is already R. 
        Total = R + 0.414R = 1.414R.
    */


    double search_radius;
    if (is_geometric_mode_) {
        // In geometric mode, we just need to cover the obstacle size + robot size + delta
        search_radius = obs_r + ob.inflation + neighborhood_radius_;
    } else {
        // Kinodynamic mode: Add gap coverage for the "tube" samples
        double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
        search_radius = obs_r + ob.inflation + neighborhood_radius_ + gap_coverage_inflation;
    }


    std::unordered_set<int> orphan_indices;

    // Tube Search: Find all nodes inside the new obstacle tube
    for (const auto& point_3d : ob.predicted_path) {
        Eigen::VectorXd query(kd_dim);
        
        if (kd_dim == 3) {
            query << point_3d.x(), point_3d.y(), point_3d.z();  // z is time
        } else if (kd_dim == 2) {
            query << point_3d.x(), point_3d.y();
        } else if (kd_dim == 4) {
            query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
        } else if (kd_dim == 5) {
            query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z(); 
        }

        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) {
            // Protect all Time Pillars and the orignal root
            if (time_pillar_indices_.find(static_cast<int>(idx)) != time_pillar_indices_.end()) {
                continue; 
            }

            orphan_indices.insert(static_cast<int>(idx));
        }
    }
    // if (kd_dim == 4)
    //     search_radius += M_PI;

    /*
        Filter Orphan Indices using isTrajectorySafeAgainstSingleObstacle
        we keep the tree edges that are not in collision. This procedure
        doesn't violate the order of complexity of the collision check inherited from FMT*
        Its just good filter to not invalidate blindly
    */ 
    std::vector<int> filtered_orphan_indices;
    for (int idx : orphan_indices) {
        FMTNode* node = tree_[idx].get();
        // node->setG(INFINITY);
#if USE_THREAT_SET_STRATEGY
        // Mark this node as being under threat, avoiding duplicates
        if (std::find(node->threats.begin(), node->threats.end(), &ob) == node->threats.end()) {
            node->threats.push_back(&ob);
        }

#endif

        // Skip root or nodes with no parent
        if (node->getParent() == nullptr) continue; 
        last_replan_metrics_.obstacle_checks++;        
        // This checks the edge connecting the node to its parent
        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(node->getParentTrajectory()), ob)) {
            filtered_orphan_indices.push_back(idx);
        }
    }

    orphan_indices.clear(); 
    for (int idx : filtered_orphan_indices) {
        orphan_indices.insert(idx);
    }

    // Propagate Orphanhood to Descendants
    std::vector<int> initial_hit_list(orphan_indices.begin(), orphan_indices.end());
    std::queue<FMTNode*> propagation_queue;

    for (int idx : initial_hit_list) {
        propagation_queue.push(tree_[idx].get());
    }

    while (!propagation_queue.empty()) {
        FMTNode* current = propagation_queue.front();
        propagation_queue.pop();

        for (FMTNode* child : current->getChildren()) {
            if (orphan_indices.insert(child->getIndex()).second) {
                propagation_queue.push(child);
            }
        }
    }


    // Invalidate Nodes & Queue Boundary Parents
    std::unordered_set<FMTNode*> boundary_nodes_to_requeue;
    for (int node_index : orphan_indices) {
        auto node = tree_[node_index].get();
        // Remove from Open Set (it's invalid now)
        if (node->in_queue_ && node->getIndex() != root_state_index_) {
            v_open_heap_.remove(node);
        }

        // Invalidate Cost (but keep Root valid)
        if (node->getIndex() != root_state_index_) {
            node->setLMC(std::numeric_limits<double>::infinity()); 
            node->setG(std::numeric_limits<double>::infinity());
        }
        
        // Sever Parent Connection
        node->setParent(nullptr, std::shared_ptr<Trajectory>{});

        // Find Boundary (Valid Parents). We look at neighbors. If a neighbor is NOT an orphan, it's a valid candidate parent
        auto check_boundary = [&](const auto& neighbors) {
            for (const auto& [neighbor_ptr, edge_data] : neighbors) {
                if (orphan_indices.find(neighbor_ptr->getIndex()) == orphan_indices.end()) {
                    boundary_nodes_to_requeue.insert(neighbor_ptr);
                }
            }
        };

        check_boundary(node->forwardNeighbors());
        // check_boundary(node->backwardNeighbors());
    }

    // Add Boundary to Open Heap
    for (FMTNode* valid_node : boundary_nodes_to_requeue) {
        if (!valid_node->in_queue_ && valid_node->getLMC() != std::numeric_limits<double>::infinity()) {
            v_open_heap_.add(valid_node, valid_node->getLMC());
        }
    }


    // // VISUALIZATION: ORPHANS (RED) & BOUNDARY (MAGENTA)
    // if (visualization_) {
    //     std::vector<Eigen::VectorXd> orphan_positions;
    //     orphan_positions.reserve(orphan_indices.size());
    //     for (int idx : orphan_indices) {
    //         // Extract 2D position (x, y) for visualization
    //         orphan_positions.push_back(tree_[idx]->getStateValue().head<2>());
    //     }

    //     std::vector<Eigen::VectorXd> boundary_positions;
    //     boundary_positions.reserve(boundary_nodes_to_requeue.size());
    //     for (FMTNode* node : boundary_nodes_to_requeue) {
    //         boundary_positions.push_back(node->getStateValue().head<2>());
    //     }

    //     visualization_->visualizeNodes(orphan_positions, "map", 
    //                                  {1.0f, 0.0f, 0.0f}, 
    //                                  "fmtx_orphans");

    //     visualization_->visualizeNodes(boundary_positions, "map", 
    //                                  {1.0f, 0.0f, 1.0f}, 
    //                                  "fmtx_boundary");
    // }

}



// Wake Up Neighbors
void KinodynamicFMTX::removeObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);

    double search_radius;
    
    if (is_geometric_mode_) {
        // In geometric mode, we just need to cover the obstacle size + robot size + delta
        search_radius = obs_r + ob.inflation + neighborhood_radius_;
    } else {
        // Kinodynamic mode: Add gap coverage for the "tube" samples
        double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
        search_radius = obs_r + ob.inflation + neighborhood_radius_ + gap_coverage_inflation;
    }


    std::unordered_set<int> freed_indices;

    // Tube Search: Find nodes that were near the OLD path
    for (const auto& point_3d : ob.predicted_path) {
        Eigen::VectorXd query(kd_dim);
        if (kd_dim == 3) query << point_3d.x(), point_3d.y(), point_3d.z(); //z is time!
        else if (kd_dim == 2) query << point_3d.x(), point_3d.y();
        else if (kd_dim == 4) {
            query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
        }else if (kd_dim == 5) {
            query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z(); 
        }

        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) {
            /*
                Don't put protection for pillar nodes because I directly put kdtree nodes into vopen heap (as oppsed to addNewObstalce where I orphan the kdtree nodes)
            */
            freed_indices.insert(static_cast<int>(idx));
        }
    }

    // if (kd_dim == 4)
    //     search_radius += M_PI;

    /*
        Queue The Freed Nodes
        We don't change costs here. We just put valid nodes in Rn vicinity of Obstacle into the queue
        to trigger the planner to explore this newly opened space to either reduce the lmc from Inf to finite or a from a high
        finite to low finite. some 'WILL BE UNTOUCHED' edges/nodes are also queued which FMTX's  cost_via_z criteria will ignore them nice and easy
    */
    std::unordered_set<FMTNode*> neighbors_to_requeue;
    for (int node_index : freed_indices) {
        auto node = tree_.at(node_index).get();
#if USE_THREAT_SET_STRATEGY
        // O(1) SWAP-AND-POP THREAT REMOVAL
        auto it = std::find(node->threats.begin(), node->threats.end(), &ob);
        if (it != node->threats.end()) {
            *it = node->threats.back(); // Overwrite with the last element
            node->threats.pop_back();   // Delete the duplicate at the end
        }

#endif

        if (node->getLMC()!= std::numeric_limits<double>::infinity() && !node->in_queue_) {
            v_open_heap_.add(node, node->getLMC());
        }
    }
}






// // Wake Up Neighbors
// void KinodynamicFMTX::removeObstacle(const Obstacle& ob) {
//     if (ob.predicted_path.empty()) return;

//     double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
//                    std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);

//     double search_radius;
    
//     if (is_geometric_mode_) {
//         // In geometric mode, we just need to cover the obstacle size + robot size + delta
//         search_radius = obs_r + ob.inflation + neighborhood_radius_;
//     } else {
//         // Kinodynamic mode: Add gap coverage for the "tube" samples
//         double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
//         search_radius = obs_r + ob.inflation + neighborhood_radius_ + gap_coverage_inflation;
//     }


//     std::unordered_set<int> freed_indices;

//     // Tube Search: Find nodes that were near the OLD path
//     for (const auto& point_3d : ob.predicted_path) {
//         Eigen::VectorXd query(kd_dim);
//         if (kd_dim == 3) query << point_3d.x(), point_3d.y(), point_3d.z(); //z is time!
//         else if (kd_dim == 2) query << point_3d.x(), point_3d.y();
//         else if (kd_dim == 4) {
//             query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
//         }else if (kd_dim == 5) {
//             query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z(); 
//         }

//         std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
//         for (size_t idx : indices) {
            
//             // Protect all Time Pillars and the original root
//             if (time_pillar_indices_.find(static_cast<int>(idx)) != time_pillar_indices_.end()) {
//                 continue; 
//             }
//             freed_indices.insert(static_cast<int>(idx));
//         }
//     }

//     // if (kd_dim == 4)
//     //     search_radius += M_PI;

//     /*
//         Queue Neighbors of Freed Nodes
//         We don't change costs here. We just put valid neighbors into the queue
//         to trigger the planner to explore this newly opened space.
//     */
//     std::unordered_set<FMTNode*> neighbors_to_requeue;
//     for (int node_index : freed_indices) {
//         auto node = tree_.at(node_index).get();
// #if USE_THREAT_SET_STRATEGY
//         // O(1) SWAP-AND-POP THREAT REMOVAL
//         auto it = std::find(node->threats.begin(), node->threats.end(), &ob);
//         if (it != node->threats.end()) {
//             *it = node->threats.back(); // Overwrite with the last element
//             node->threats.pop_back();   // Delete the duplicate at the end
//         }

// #endif

//         // if (node->getLMC()!= std::numeric_limits<double>::infinity()) {
//         //     continue; // If the node already is on the graph then its already free!
//         // }
//         // if (!neighbor_precache) near(node_index);

//         if (node->getLMC()!= std::numeric_limits<double>::infinity() && !node->in_queue_) {
//             // continue; // If the node already is on the graph then its already free!  // THIS IS INCORRECT! what if all the nodes have finite lmc but obstalce removal opens a cheap edge between these socalled finite lmc nodes! so we shouldnt filter based on finite lmc!
//             neighbors_to_requeue.insert(node);
//         }

//         auto check_neighbors = [&](const auto& neighbors) {
//             for (const auto& [neighbor_ptr, edge_data] : neighbors) {
//                 // If neighbor is valid (has cost) and not in queue, add it.
//                 if (neighbor_ptr->getLMC() != std::numeric_limits<double>::infinity() && !neighbor_ptr->in_queue_) {
//                     neighbors_to_requeue.insert(neighbor_ptr);
//                 }
//             }
//         };

//         check_neighbors(node->forwardNeighbors());
//         // check_neighbors(node->backwardNeighbors());
//     }
    

//     // Add to Open Heap
//     for (FMTNode* neighbor : neighbors_to_requeue) {
//         v_open_heap_.add(neighbor, neighbor->getLMC());
//     }
// }






// void KinodynamicFMTX::setRobotState(const Eigen::VectorXd& robot_state) {
//     robot_continuous_state_ = robot_state;
//     double robot_sim_time = robot_continuous_state_(robot_continuous_state_.size() - 1);

//     Eigen::VectorXd query_point = Eigen::VectorXd::Zero(kd_dim);
//     if (robot_continuous_state_.size() >= 2) {
//         query_point(0) = robot_continuous_state_(0);
//         query_point(1) = robot_continuous_state_(1);
//     }
//     if (kd_dim == 3) {
//         query_point(2) = robot_sim_time;
//     } else if (kd_dim == 4) {
//         query_point(2) = robot_continuous_state_(2); 
//         query_point(3) = robot_sim_time;
//     } else if (kd_dim == 5) {
//         query_point = robot_continuous_state_; 
//     }

//     const double hysteresis_factor = 0.98;
//     double cost_of_current_path = std::numeric_limits<double>::infinity();
//     Trajectory bridge;
//     bool safe = true;

//     // Check existing anchor
//     if (robot_node_ && robot_node_->getLMC() != std::numeric_limits<double>::infinity()) {
//         bridge = statespace_->steer(robot_continuous_state_, robot_node_->getStateValue());
//         if (bridge.is_valid) {
//             const auto& obstacles = obs_checker_->getObstacles();
//             for (const auto& ob : obstacles) {
//                 last_replan_metrics_.obstacle_checks++;
//                 if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(bridge, ob)) {
//                     safe = false;
//                     break;
//                 }
//             }
//             if (safe) {
//                 cost_of_current_path = bridge.cost + robot_node_->getLMC();
//                 robot_current_time_to_goal_ = bridge.time_duration + robot_node_->getTimeToGoal();
//             }
//         }
//     }

//     FMTNode* best_candidate_node = nullptr;
//     Trajectory best_candidate_bridge;
//     double best_candidate_cost = std::numeric_limits<double>::infinity();
    
//     FMTNode* best_fallback_node = nullptr;
//     Trajectory best_fallback_bridge;
//     double best_fallback_cost = std::numeric_limits<double>::infinity();

//     double current_search_radius = neighborhood_radius_; 
//     const int max_attempts = 5; 
//     const double radius_multiplier = 2.0;
//     std::unordered_set<size_t> tested_indices;
//     for (int attempt = 1; attempt <= max_attempts; ++attempt) {
//         auto nearby_indices = kdtree_->radiusSearch(query_point, current_search_radius);
//         for (auto idx : nearby_indices) {
//             if (!tested_indices.insert(idx).second) {
//                 continue;
//             }
//             FMTNode* candidate = tree_[idx].get();
//             Trajectory temp_bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());
//             if (!temp_bridge.is_valid) continue;

//             bool safe = true;
//             const auto& obstacles = obs_checker_->getObstacles();
//             for (const auto& ob : obstacles) {
//                 last_replan_metrics_.obstacle_checks++;
//                 if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(temp_bridge, ob)) {
//                     safe = false;
//                     break;
//                 }
//             }
//             if (!safe) continue;

// #if USE_RECOVERY
//             if (temp_bridge.cost < best_fallback_cost) {
//                 best_fallback_node = candidate;
//                 best_fallback_bridge = temp_bridge;
//                 best_fallback_cost = temp_bridge.cost;
//             }
// #endif

//             if (candidate->getLMC() != std::numeric_limits<double>::infinity()) {
//                 double cost = temp_bridge.cost + candidate->getLMC();
//                 if (cost < best_candidate_cost) {
//                     best_candidate_node = candidate;
//                     best_candidate_bridge = temp_bridge;
//                     best_candidate_cost = cost;
//                 }
//             }
//         }
//         if (best_candidate_node) break;
//         current_search_radius *= radius_multiplier;
//     }

//     // ASSIGNMENT PRIORITY
//     // 1) Better connected anchor
//     if (best_candidate_node &&
//         best_candidate_cost < cost_of_current_path * hysteresis_factor) {

//         robot_node_ = best_candidate_node;
//         robot_current_time_to_goal_ =
//             best_candidate_bridge.time_duration + best_candidate_node->getTimeToGoal();
//         last_replan_metrics_.path_cost = best_candidate_cost;
//         current_bridge_trajectory_ = best_candidate_bridge;
//         bridge_cost_ = best_candidate_bridge.cost;
//     }

//     // 2) Keep current anchor with fresh bridge
//     else if (safe && robot_node_ &&
//              cost_of_current_path != std::numeric_limits<double>::infinity() &&
//              bridge.is_valid) {

//         robot_current_time_to_goal_ =
//             bridge.time_duration + robot_node_->getTimeToGoal();
//         last_replan_metrics_.path_cost = cost_of_current_path;
//         current_bridge_trajectory_ = bridge;
//         bridge_cost_ = bridge.cost;
//     }

//     // 3) Recovery: go to nearest safe tree node
// #if USE_RECOVERY
//     else if (best_fallback_node) {
//         // robot_node_ = best_fallback_node;
//         robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
//         bridge_cost_ = best_fallback_cost;
//         current_bridge_trajectory_ = best_fallback_bridge;
//         last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();

//         FMTX_WARN("[Set Robot STate] No connected anchor found. Falling back to nearest safe tree node.");
//     }
// #endif

//     // 4) Keep current anchor, reuse previous cached bridge
//     //    This is the near-root / numerical-steer-failure case.
//     else if (robot_node_ &&
//              current_bridge_trajectory_.is_valid &&
//              !current_bridge_trajectory_.path_points.empty()) {

//         robot_current_time_to_goal_ =
//             current_bridge_trajectory_.time_duration + robot_node_->getTimeToGoal();

//         // Keep finite so we do NOT enter trapped logic.
//         const double anchor_tail_cost =
//             (robot_node_->getLMC() != std::numeric_limits<double>::infinity())
//                 ? robot_node_->getLMC()
//                 : 0.0;

//         cost_of_current_path = current_bridge_trajectory_.cost + anchor_tail_cost;
//         last_replan_metrics_.path_cost = cost_of_current_path;
//         bridge_cost_ = current_bridge_trajectory_.cost;

//         FMTX_WARN("[Set Robot STate] Fresh steer failed near anchor/root. Reusing cached bridge.");
//     }

//     // 5) Truly trapped
//     else {
//         robot_node_ = nullptr;
//         robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
//         bridge_cost_ = std::numeric_limits<double>::infinity();
//         current_bridge_trajectory_ = Trajectory();
//         last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();

//         FMTX_WARN("[Set Robot STate] LOST SAFE ANCHOR. TRULY TRAPPED.");
//     }
// }

bool KinodynamicFMTX::isRobotSafe() {
    return (robot_node_ != nullptr) && (robot_node_->getLMC() != std::numeric_limits<double>::infinity());
}




void KinodynamicFMTX::addStaticObstacles(const ObstacleVector& obstacles) {

    for (const auto& ob : obstacles) {
        previous_obstacles_[ob.name] = ob;
        addNewStaticObstacle(previous_obstacles_[ob.name]);
    }

    plan();
}

void KinodynamicFMTX::removeStaticObstacles(const ObstacleVector& obstacles) {
    for (const auto& ob : obstacles) {
        auto it = previous_obstacles_.find(ob.name);
        if (it != previous_obstacles_.end()) {
            removeStaticObstacle(it->second);
            previous_obstacles_.erase(it);
        }
    }
    plan();
}

void KinodynamicFMTX::addNewStaticObstacle(const Obstacle& ob) {
    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    
    double search_radius;
    search_radius = obs_r + ob.inflation + neighborhood_radius_;

    std::unordered_set<int> orphan_indices;

    // STATIC ONLY: Use spatial KD-tree (x,y), NO TIME
    Eigen::Vector2d obs_pos(ob.position.x(), ob.position.y());
    std::vector<size_t> indices = spatial_kdtree_->radiusSearch(obs_pos, search_radius);
    
    for (size_t idx : indices) {
        if (time_pillar_indices_.find(static_cast<int>(idx)) != time_pillar_indices_.end()) {
            continue; 
        }
        orphan_indices.insert(static_cast<int>(idx));
    }

    // EXACT SAME LOGIC AS addNewObstacle from here
    std::vector<int> filtered_orphan_indices;
    for (int idx : orphan_indices) {
        FMTNode* node = tree_[idx].get();
#if USE_THREAT_SET_STRATEGY
        if (std::find(node->threats.begin(), node->threats.end(), &ob) == node->threats.end()) {
            node->threats.push_back(&ob);
        }
#endif
        if (node->getParent() == nullptr) continue; 
        last_replan_metrics_.obstacle_checks++;        
        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(node->getParentTrajectory()), ob)) {
            filtered_orphan_indices.push_back(idx);
        }
    }

    orphan_indices.clear(); 
    for (int idx : filtered_orphan_indices) {
        orphan_indices.insert(idx);
    }

    // Propagate
    std::vector<int> initial_hit_list(orphan_indices.begin(), orphan_indices.end());
    std::queue<FMTNode*> propagation_queue;
    for (int idx : initial_hit_list) {
        propagation_queue.push(tree_[idx].get());
    }
    while (!propagation_queue.empty()) {
        FMTNode* current = propagation_queue.front(); propagation_queue.pop();
        for (FMTNode* child : current->getChildren()) {
            if (orphan_indices.insert(child->getIndex()).second) {
                propagation_queue.push(child);
            }
        }
    }

    // Invalidate & requeue boundaries
    std::unordered_set<FMTNode*> boundary_nodes_to_requeue;
    for (int node_index : orphan_indices) {
        auto node = tree_[node_index].get();
        if (node->in_queue_ && node->getIndex() != root_state_index_) {
            v_open_heap_.remove(node);
        }
        if (node->getIndex() != root_state_index_) {
            node->setLMC(std::numeric_limits<double>::infinity()); 
            node->setG(std::numeric_limits<double>::infinity());
        }
        node->setParent(nullptr, std::shared_ptr<Trajectory>{});

        auto check_boundary = [&](const auto& neighbors) {
            for (const auto& [neighbor_ptr, edge_data] : neighbors) {
                if (orphan_indices.find(neighbor_ptr->getIndex()) == orphan_indices.end()) {
                    boundary_nodes_to_requeue.insert(neighbor_ptr);
                }
            }
        };
        check_boundary(node->forwardNeighbors());
    }

    for (FMTNode* valid_node : boundary_nodes_to_requeue) {
        if (!valid_node->in_queue_ && valid_node->getLMC() != std::numeric_limits<double>::infinity()) {
            v_open_heap_.add(valid_node, valid_node->getLMC());
        }
    }
}

void KinodynamicFMTX::removeStaticObstacle(const Obstacle& ob) {
    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);

    double search_radius;
    search_radius = obs_r + ob.inflation + neighborhood_radius_;

    std::unordered_set<int> freed_indices;

    // STATIC ONLY: Use spatial KD-tree (x,y), NO TIME  
    Eigen::Vector2d obs_pos(ob.position.x(), ob.position.y());
    std::vector<size_t> indices = spatial_kdtree_->radiusSearch(obs_pos, search_radius);
    
    for (size_t idx : indices) {
        if (time_pillar_indices_.find(static_cast<int>(idx)) != time_pillar_indices_.end()) {
            continue; 
        }
        freed_indices.insert(static_cast<int>(idx));
    }

    // EXACT SAME LOGIC AS removeObstacle from here
    std::unordered_set<FMTNode*> neighbors_to_requeue;
    for (int node_index : freed_indices) {
        auto node = tree_.at(node_index).get();
#if USE_THREAT_SET_STRATEGY
        auto it = std::find(node->threats.begin(), node->threats.end(), &ob);
        if (it != node->threats.end()) {
            *it = node->threats.back();
            node->threats.pop_back();
        }
#endif

        auto check_neighbors = [&](const auto& neighbors) {
            for (const auto& [neighbor_ptr, edge_data] : neighbors) {
                if (neighbor_ptr->getLMC() != std::numeric_limits<double>::infinity() && !neighbor_ptr->in_queue_) {
                    neighbors_to_requeue.insert(neighbor_ptr);
                }
            }
        };
        check_neighbors(node->forwardNeighbors());
    }
    
    for (FMTNode* neighbor : neighbors_to_requeue) {
        v_open_heap_.add(neighbor, neighbor->getLMC());
    }
}






void KinodynamicFMTX::setRobotState(const Eigen::VectorXd& robot_state) {
    robot_continuous_state_ = robot_state;
    double robot_sim_time = robot_continuous_state_(robot_continuous_state_.size() - 1);

    Eigen::VectorXd query_point = Eigen::VectorXd::Zero(kd_dim);
    if (robot_continuous_state_.size() >= 2) {
        query_point(0) = robot_continuous_state_(0);
        query_point(1) = robot_continuous_state_(1);
    }
    if (kd_dim == 3) {
        query_point(2) = robot_sim_time;
    } else if (kd_dim == 4) {
        query_point(2) = robot_continuous_state_(2); 
        query_point(3) = robot_sim_time;
    } else if (kd_dim == 5) {
        query_point = robot_continuous_state_; 
    }

    FMTNode* best_candidate_node = nullptr;
    Trajectory best_candidate_bridge;
    double best_candidate_cost = std::numeric_limits<double>::infinity();
    
    FMTNode* best_fallback_node = nullptr;
    Trajectory best_fallback_bridge;
    double best_fallback_cost = std::numeric_limits<double>::infinity();

    double current_search_radius = neighborhood_radius_; 
    const int max_attempts = 5; 
    const double radius_multiplier = 2.0;

    std::unordered_set<size_t> tested_indices;

    // 1. PERFORM RADIUS SEARCH FOR A NEW ANCHOR
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        auto nearby_indices = kdtree_->radiusSearch(query_point, current_search_radius);
        for (auto idx : nearby_indices) {
            if (!tested_indices.insert(idx).second) {
                continue;
            }
            FMTNode* candidate = tree_[idx].get();
            Trajectory temp_bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());
            if (!temp_bridge.is_valid) continue;

            bool safe = true;
            for (const auto& [name, ob] : previous_obstacles_) {
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(temp_bridge, ob)) {
                    safe = false;
                    break;
                }
            }
            if (!safe) continue;

#if USE_RECOVERY
            if (temp_bridge.cost < best_fallback_cost) {
                best_fallback_node = candidate;
                best_fallback_bridge = temp_bridge;
                best_fallback_cost = temp_bridge.cost;
            }
#endif

            if (candidate->getLMC() != std::numeric_limits<double>::infinity()) {
                double cost = temp_bridge.cost + candidate->getLMC();
                if (cost < best_candidate_cost) {
                    best_candidate_node = candidate;
                    best_candidate_bridge = temp_bridge;
                    best_candidate_cost = cost;
                }
            }
        }
        if (best_candidate_node) break;
        current_search_radius *= radius_multiplier;
    }

    // 2. SIMPLE ASSIGNMENT LOGIC (We only get here if we MUST switch anchors)
    if (best_candidate_node) {
        robot_node_ = best_candidate_node;
        robot_current_time_to_goal_ = best_candidate_bridge.time_duration + best_candidate_node->getTimeToGoal();
        last_replan_metrics_.path_cost = best_candidate_cost;
        current_bridge_trajectory_ = best_candidate_bridge;
        bridge_cost_ = best_candidate_bridge.cost;
    }
#if USE_RECOVERY
    else if (best_fallback_node) {
        robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
        bridge_cost_ = best_fallback_cost;
        current_bridge_trajectory_ = best_fallback_bridge;
        last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
        FMTX_WARN("[Set Robot State] No connected anchor found. Falling back to nearest safe tree node.");
    }
#endif
    else {
        robot_node_ = nullptr;
        robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
        bridge_cost_ = std::numeric_limits<double>::infinity();
        current_bridge_trajectory_ = Trajectory();
        last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
        FMTX_WARN("[Set Robot State] LOST SAFE ANCHOR. TRULY TRAPPED.");
    }
}
// bool KinodynamicFMTX::isCurrentBridgeSafe(const ObstacleVector& obstacles) const {
//     // If we don't have a valid bridge, it's definitely not safe.
//     if (!current_bridge_trajectory_.is_valid || current_bridge_trajectory_.path_points.empty()) {
//         return false; 
//     }

//     // Check the current bridge against all newly turned obstacles
//     for (const auto& ob : obstacles) {
//         if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(current_bridge_trajectory_, ob)) {
//             return false; // The dynamic obstacle crashed into our trajectory!
//         }
//     }
//     return true;
// }
bool KinodynamicFMTX::isCurrentBridgeSafe(const ObstacleVector& obstacles) const {
    if (!current_bridge_trajectory_.is_valid || current_bridge_trajectory_.path_points.empty()) {
        // FMTX_WARN("[Bridge Check] Bridge is empty or invalid!");
        return false; 
    }

    // 1. Check immediate bridge collision
    for (const auto& ob : obstacles) {
        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(current_bridge_trajectory_, ob)) {
            // FMTX_WARN("[Bridge Check] IMMEDIATE COLLISION! Bridge intersects with obstacle: " << ob.name);
            return false; 
        }
    }

    // 2. THE ROOT CAUSE CHECK: Is the anchor node still safely connected to the goal?
    // In FMTX, plan() might have orphaned our robot_node_ or set its LMC to INF!
    if (robot_node_ != nullptr) {
        if (robot_node_->getLMC() == std::numeric_limits<double>::infinity()) {
            // FMTX_WARN("[Bridge Check] GRAPH DESYNC! The immediate bridge is safe, but the anchor node (LMC=INF) has been cut off by plan()!");
            return false; // The bridge leads to a dead end!
        }
    } else {
        // FMTX_WARN("[Bridge Check] GRAPH DESYNC! robot_node_ is NULL!");
        return false;
    }

    return true;
}

bool KinodynamicFMTX::hasReachedAnchor(const Eigen::VectorXd& current_sim_state) const {
    // If we don't have an anchor, trigger a new search
    if (!robot_node_) return true; 
    if (robot_node_->getLMC()==INFINITY) return true; 

    // Calculate the time remaining on the current edge.
    // T_robot is the time left in the simulation budget (the z-axis in your kd-tree).
    double current_T_robot = current_sim_state(current_sim_state.size() - 1);
    
    // The anchor node's T_robot
    double anchor_T_robot = robot_node_->getStateValue()(current_sim_state.size() - 1);

    // We consider the anchor "reached" if we are within a small temporal threshold 
    // of the node's timestamp (e.g., 0.1 seconds away from the node).
    if (current_T_robot <= anchor_T_robot + 1e-6) {
        return true;
    }

    return false;
}

std::vector<Eigen::VectorXd> KinodynamicFMTX::getLivePathPositions(const Eigen::VectorXd& current_state) const
{
    if (!robot_node_ || robot_node_->getLMC() == std::numeric_limits<double>::infinity()) {
        return {}; 
    }

    // 1. Create a temporary, real-time bridge from the robot to the anchor
    Trajectory live_bridge = statespace_->steer(current_state, robot_node_->getStateValue());
    
    if (!live_bridge.is_valid || live_bridge.path_points.empty()) {
        // If the live steer fails (e.g. numerical issue very close to the node),
        // fallback to the cached path
        return getPathPositions(); 
    }

    // 2. Start the path with the live bridge
    std::vector<Eigen::VectorXd> final_executable_path = live_bridge.path_points;

    // 3. Traverse the rest of the tree from the anchor node
    FMTNode* child = robot_node_;
    FMTNode* parent = child->getParent();

    while (parent) {
        auto cached_traj = child->getParentTrajectory();
        if (cached_traj->is_valid && cached_traj->path_points.size() > 1) {
            final_executable_path.insert(final_executable_path.end(),
                                         cached_traj->path_points.begin() + 1,
                                         cached_traj->path_points.end());
        } else {
            break;
        }
        child = parent;
        parent = child->getParent();
    }

    return final_executable_path;
}


bool KinodynamicFMTX::hasShortcut(const Eigen::VectorXd& robot_state, double threshold) {
    if (!robot_node_ || robot_node_->getLMC() == std::numeric_limits<double>::infinity()) {
        return false; // Cannot evaluate a shortcut if we are trapped or have no path
    }

    // UNIFIED LOGIC: Current Cost = Bridge Cost + Anchor Cost
    double current_cost = bridge_cost_ + robot_node_->getLMC();

    // Protect against division by zero
    if (current_cost <= 0.001 || current_cost == std::numeric_limits<double>::infinity()) {
        return false; 
    }

    double robot_sim_time = robot_state(robot_state.size() - 1);
    Eigen::VectorXd query_point = Eigen::VectorXd::Zero(kd_dim);
    if (robot_state.size() >= 2) {
        query_point(0) = robot_state(0);
        query_point(1) = robot_state(1);
    }
    if (kd_dim == 3) {
        query_point(2) = robot_sim_time;
    } else if (kd_dim == 4) {
        query_point(2) = robot_state(2); 
        query_point(3) = robot_sim_time;
    } else if (kd_dim == 5) {
        query_point = robot_state; 
    }

    // Force a larger search radius so we can jump gaps!
    double current_search_radius = neighborhood_radius_ * 2.0; 
    const int max_attempts = 5; 
    const double radius_multiplier = 2.0;

    std::unordered_set<size_t> tested_indices;

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        auto nearby_indices = kdtree_->radiusSearch(query_point, current_search_radius);
        for (auto idx : nearby_indices) {
            if (!tested_indices.insert(idx).second) continue;

            FMTNode* candidate = tree_[idx].get();
            if (candidate->getLMC() == std::numeric_limits<double>::infinity()) continue;

            Trajectory temp_bridge = statespace_->steer(robot_state, candidate->getStateValue());
            if (!temp_bridge.is_valid) continue;

            // CANDIDATE COST = SteerCost + Candidate Cost
            double new_total_cost = temp_bridge.cost + candidate->getLMC();
            double improvement = (current_cost - new_total_cost) / current_cost;
            
            // FAST REJECTION: Only check collisions if it meets the threshold
            if (improvement >= threshold) {
                bool safe = true;
                // ANYFMTX stores obstacles in a map called previous_obstacles_
                for (const auto& [name, ob] : previous_obstacles_) {
                    if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(temp_bridge, ob)) {
                        safe = false;
                        break;
                    }
                }
                if (safe) {
                    return true; // We found a valid shortcut!
                }
            }
        }
        current_search_radius *= radius_multiplier;
    }
    return false;
}