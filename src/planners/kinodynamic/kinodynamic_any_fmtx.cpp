// Copyright 2025 Soheil E.nia

// TODO: Later implement KNN. with knn you wouldnt need cullNeighbor! use if (use_knn) return in cullNeighbor
#define DEBUG 0 // Debugs included are a full "Collision/Cost propagation/Espsilon consistency/Suboptimality average cost" recheck
#define VIS 0 // For visualizing open heap node to see the partial update in play
#define USE_THREAT_SET_STRATEGY 0 // Context-aware Threat set: The Threat Set is the bridge that allows a lazy algorithm (like FMTx) to behave with the same spatial intelligence as an eager one (Eager like RRTx)
#define USE_RECOVERY 0 // Emergency Fallback
#define STATIC 0
// Epoch-based "soft-block" of edges that fail a parent-search collision check, so the same
// edge is not re-selected & re-checked again within the SAME obstacle cycle. The mark lives in
// EdgeInfo::last_eval_epoch (unused by FMTX otherwise) and auto-expires when updateObstacles
// bumps plan_epoch_ -> NO restoration loop needed. Set to 0 to disable (no caching).
#define USE_CACHE_FAILURE 1
#include "motion_planning/planners/kinodynamic/kinodynamic_any_fmtx.hpp"
#include "motion_planning/planners/kinodynamic/time_cone_prune.hpp"  // TIME_CONE_PRUNED + master switch

KinodynamicANYFMTX::KinodynamicANYFMTX(std::shared_ptr<StateSpace> statespace ,std::shared_ptr<ProblemDefinition> problem_def, std::shared_ptr<ObstacleChecker> obs_checker) :  statespace_(statespace), problem_(problem_def), obs_checker_(obs_checker) {
    std::cout<< "KinodynamicANYFMTX Constructor \n";
}

void KinodynamicANYFMTX::clearPlannerState() {
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

void KinodynamicANYFMTX::setCurrentRobotTime(double robot_time_) {
    T_robot = robot_time_;
}



// void KinodynamicANYFMTX::injectTimePillarNodes(const Eigen::VectorXd& goal_state_val, int num_pillar_nodes) {
//     double max_time = upper_bounds_(statespace_->getDimension() - 1); 

//     for (int i = 1; i <= num_pillar_nodes; ++i) {
//         Eigen::VectorXd pillar_state = goal_state_val;

//         // Safely zero velocities ONLY for 5D (x, y, vx, vy, t)
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
        
//         node->setLMC(0.0);
//         node->setG(0.0);
//         v_open_heap_.add(node.get(), 0.0);

//         time_pillar_indices_.insert(node->getIndex()); 

//         tree_.push_back(std::move(node));
//     }
// }


void KinodynamicANYFMTX::injectTimePillarNodes(const Eigen::VectorXd& goal_state_val, int num_pillar_nodes) {
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



void KinodynamicANYFMTX::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    std::cout << "------------------------------------------------------------\n";
    auto start = std::chrono::high_resolution_clock::now();
    clearPlannerState();
    visualization_ = visualization;
    num_of_samples_ = params.getParam<int>("num_of_samples");
    partial_update = params.getParam<bool>("partial_update");
#if DEBUG
    partial_update = false;
#endif
    is_geometric_mode_ = params.getParam<bool>("is_geometric_mode", false);
    epsilon = params.getParam<double>("epsilon", 1e-4);
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
        kdtree_ = std::make_shared<DynamicWeightedNanoFlann>(kd_dim, weights);
    } else if (kdtree_type == "LieKDTree"){
        kdtree_ = std::make_unique<LieSplittingKDTree>(statespace_->getDimension(), statespace_);
    } else {
        throw std::runtime_error("FMTX requires a KD-Tree.");
    }
    std::cout << "num_of_samples per batch=" << num_of_samples_
                << ", bounds=[" << lower_bounds_ << ", " << upper_bounds_ << "]\n";


    setStart(problem_->getStart());
    // setGoal(problem_->getGoal()); // IMPORTANT: this alongside with saturate helps the tree to be beiased toward the root which needs more samples at the hard late stages of dynamic replanning 
    // Protect the original main root (T=0)
    time_pillar_indices_.insert(root_state_index_); 
    // Inject the rest of the Time Pillars (Backward search: start is the destination)
    if(!is_geometric_mode_) injectTimePillarNodes(problem_->getStart(), num_pillar_nodes_);

    // KDTREE
    Eigen::MatrixXd all_samples = statespace_->getSamplesCopy();
    Eigen::MatrixXd spatial_samples_only = all_samples.leftCols(kd_dim).eval();
    // kdtree_->addPoints(spatial_samples_only);
    kdtree_->addPoints(all_samples);
    kdtree_->buildTree(); // Empty function in "DynamicWeightedNanoFlann" class
    std::cout << "KDTree is Initialized: \n\n";



#if STATIC
    // SPATIAL KD-tree (x,y only)
    spatial_kdtree_ = std::make_shared<DynamicWeightedNanoFlann>(2, Eigen::Vector2d(1.0,1.0));
    Eigen::MatrixXd spatial_samples = all_samples.leftCols(2);  // Just (x,y)
    spatial_kdtree_->addPoints(spatial_samples);
    spatial_kdtree_->buildTree();
#endif


    dimension_ = statespace_->getDimension();
    factor = params.getParam<double>("factor");
    delta = params.getParam<double>("delta");

    // GAMMA CALC
    int d = kd_dim;
    Eigen::VectorXd range = upper_bounds_ - lower_bounds_;
    // double mu = range.prod();
    double mu = 1.0;
    for(int i = 0; i < d; ++i) {
        mu *= range(i);
    }
    double zetaD = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1.0);
    // gamma_ = std::pow(1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); // FMT star gamma which is smaller than RRT* which makes the neighborhood size less than RRT*
    gamma_ = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); // RRT* gamma


    // Calculate initial radius based on N=2 (Start + Goal)
    shrinkingBallRadius();
    std::cout << "Setup complete. Ready for incremental sampling.\n";
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "------------------------------------------------------------\n";
}

// void KinodynamicANYFMTX::shrinkingBallRadius() {
//     int N = tree_.size();
//     if (N <= 1) return;
//     // int d = statespace_->getDimension();
//     int d = kd_dim;
//     neighborhood_radius_ = factor * gamma_ * std::pow(std::log(N) / N, 1.0 / d);
//     neighborhood_radius_ = std::min(delta, neighborhood_radius_);
// }

void KinodynamicANYFMTX::shrinkingBallRadius() {
    // tree_.size() includes the root + injected time pillars, which are
    // co-located at the goal position and are NOT space-filling. Counting
    // them inflates N and shrinks r_n as if the space were denser than it is,
    // degrading connectivity. Exclude them so N reflects only the random,
    // space-filling samples.
    int N = static_cast<int>(tree_.size()) - num_pillar_nodes_;
    if (N == 0) return; // No real samples yet



    // Safe N prevents log(1) = 0. We treat the tree as having at least 2 nodes 
    // to give the initial radius a mathematically valid starting volume.
    int safe_N = std::max(2, N); 
    
    int d = kd_dim;
    neighborhood_radius_ = factor * gamma_ * std::pow(std::log(safe_N) / safe_N, 1.0 / d);
    neighborhood_radius_ = std::min(delta, neighborhood_radius_);
}


void KinodynamicANYFMTX::analyzeSuboptimality(FMTNode* x, FMTNode* best_parent_for_x, FMTNode* z, SuboptimalityMetrics& metrics) {
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
                // std::cout << "[SUBOPT_DEBUG] Node " << x->getIndex()
                //         << " | Missed y=" << y->getIndex() << " | Δcost=" << (chosen_cost - candidate_cost) << "\n";
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

void KinodynamicANYFMTX::printDebugSummary(const SuboptimalityMetrics& metrics) {
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

bool KinodynamicANYFMTX::runCollisionForensics() {
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
            
            // // VERIFY: Absolute Ground Truth check
            // for (const auto& [name, ob] : previous_obstacles_) {
            //     if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(edge_traj, ob)) {
            //         edge_collides = true;
            //         guilty_obstacle = name;
            //         break;
            //     }
            // }
            const ObstacleVector& current_obstacles = obs_checker_->getObstacles();
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
}

bool KinodynamicANYFMTX::runGlobalCostForensics() {
    // std::cout << "\n[FMTx FORENSICS] --- STARTING K*EPSILON COST DRIFT VERIFICATION ---" << std::endl;
    int nodes_checked = 0;
    int drift_violations = 0;
    int optimistic_violations = 0;
    double max_drift = 0.0;
    FMTNode* worst_node = nullptr;

    std::vector<double> true_costs(tree_.size(), -2.0);
    std::vector<int> true_depths(tree_.size(), -2);
    
    std::function<std::pair<double, int>(FMTNode*)> computeTrueCostAndDepth = [&](FMTNode* node) -> std::pair<double, int> {
        int idx = node->getIndex();
        
        if (idx == root_state_index_) return {0.0, 0};
        
        if (node->getParent() == nullptr || node->getLMC() == std::numeric_limits<double>::infinity()) {
            return {std::numeric_limits<double>::infinity(), 0};
        }
        
        if (true_costs[idx] == -1.0) {
            return {std::numeric_limits<double>::infinity(), 0};
        }
        
        if (true_costs[idx] >= 0.0) {
            return {true_costs[idx], true_depths[idx]};
        }

        true_costs[idx] = -1.0;

        auto parent_result = computeTrueCostAndDepth(node->getParent());
        double parent_true_cost = parent_result.first;
        int parent_depth = parent_result.second;
        
        if (parent_true_cost == std::numeric_limits<double>::infinity()) {
            true_costs[idx] = std::numeric_limits<double>::infinity();
            return {std::numeric_limits<double>::infinity(), 0};
        }

        double edge_cost = 0.0;
        if (node->getParentTrajectory() != nullptr) {
            edge_cost = node->getParentTrajectory()->cost;
        } else {
            true_costs[idx] = std::numeric_limits<double>::infinity();
            return {std::numeric_limits<double>::infinity(), 0};
        }

        true_costs[idx] = parent_true_cost + edge_cost;
        true_depths[idx] = parent_depth + 1; 
        
        return {true_costs[idx], true_depths[idx]};
    };

    // Evaluate all nodes
    for (size_t i = 0; i < tree_.size(); ++i) {
        FMTNode* node = tree_[i].get();
        double stored_cost = node->getLMC(); 

        if (stored_cost != std::numeric_limits<double>::infinity() && node->getIndex() != root_state_index_) {
            nodes_checked++;
            auto result = computeTrueCostAndDepth(node);
            double actual_true_cost = result.first;
            int k_depth = result.second;

            if (actual_true_cost != std::numeric_limits<double>::infinity()) {
                double drift = stored_cost - actual_true_cost;
                
                if (drift > max_drift) {
                    max_drift = drift;
                    worst_node = node;
                }

                double k_epsilon_bound = k_depth * epsilon;

                if (drift > k_epsilon_bound + 1e-5) { 
                    drift_violations++;
                } else if (drift < -1e-5) {
                    optimistic_violations++;
                }
            }
        }
    }

    // std::cout << "-> Checked " << nodes_checked << " valid nodes.\n";
    
    if (worst_node != nullptr) {
        auto result = computeTrueCostAndDepth(worst_node);
        double k_epsilon_bound = result.second * epsilon;
        std::cout << "-> Maximum pessimistic drift observed: " << max_drift 
                  << " (at Node " << worst_node->getIndex() << ") | k*eps limit: " << k_epsilon_bound << "\n";
                  
        if (max_drift > k_epsilon_bound + 1e-5) {
            std::cout << "\n\033[1;31m[X-RAY TRACE OF WORST VIOLATOR: NODE " << worst_node->getIndex() << "]\033[0m\n";
            std::cout << "------------------------------------------------------\n";
            FMTNode* curr = worst_node;
            while (curr != nullptr && curr->getIndex() != root_state_index_) {
                FMTNode* parent = curr->getParent();
                double edge_cost = (curr->getParentTrajectory() != nullptr) ? curr->getParentTrajectory()->cost : 0.0;
                
                std::cout << "Node " << curr->getIndex() 
                          << " | Stored g(v): " << curr->getLMC() 
                          << " | Broadcast g: " << curr->getG() << "\n";
                std::cout << "   -> Parent " << parent->getIndex() 
                          << " | Stored g(v): " << parent->getLMC() 
                          << " | Broadcast g: " << parent->getG() << "\n";
                std::cout << "   -> Trajectory Edge Cost: " << edge_cost << "\n";
                
                // Check the exact math
                double expected_lmc = parent->getG() + edge_cost;
                std::cout << "   => Expected lmc(v): " << expected_lmc 
                          << " | Actual Drift at this step: " << (curr->getLMC() - expected_lmc) << "\n";
                std::cout << "------------------------------------------------------\n";
                
                curr = parent;
            }
            std::cout << "Root Node " << root_state_index_ << " reached.\n\n";
        }
    }
    
    bool passed = true;
    if (optimistic_violations > 0) passed = false;
    if (drift_violations > 0) passed = false;
    
    if (passed) {
        std::cout << "[FMTx FORENSICS] --- \033[1;32mPASSED\033[0m: All nodes respect the k*epsilon suboptimality bound! ---\n\n";
    } else {
        std::cout << "[FMTx FORENSICS] --- \033[1;31mFAILED\033[0m: Theoretical limits broken! Check trace above. ---\n\n";
    }

    return passed;
}

bool KinodynamicANYFMTX::runCostForensics() {
    std::cout << "\n[FMTx FORENSICS] --- STARTING STRICT EPSILON-CONSISTENCY VERIFICATION ---" << std::endl;
    int nodes_checked = 0;
    int violations = 0;
    int queue_waiting = 0;
    double max_local_inconsistency = 0.0;
    int max_node = -1;

    for (size_t i = 0; i < tree_.size(); ++i) {
        FMTNode* node = tree_[i].get();
        
        // In FMTX:
        // -> getLMC() acts as LMC (the instantly updated best-known path)
        // -> broadcast_cost_ acts as 'g' (the stale cost frozen until queue pops)
        double lmc = node->getLMC();
        double g = node->getG();

        // Skip completely unconnected nodes
        if (g == std::numeric_limits<double>::infinity() && 
            lmc == std::numeric_limits<double>::infinity()) {
            continue;
        }

        // Calculate inconsistency
        double inconsistency = g - lmc;

        if (inconsistency > epsilon + 1e-5) {
            if (node->in_queue_) {
                // Legally inconsistent and waiting its turn in the wavefront
                queue_waiting++;
                continue; 
            } else {
                // REAL BUG: It exceeded epsilon but was forgotten!
                violations++;
                std::cout << "\033[1;31m[REAL VIOLATION]\033[0m Node " << node->getIndex() 
                          << " | g(broadcast): " << g << " | lmc(cost): " << lmc 
                          << " | Diff: " << inconsistency << " > eps(" << epsilon << ")\n"
                          << "                 AND IT IS NOT IN THE QUEUE!\n";
            }
        }

        // Settled node
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
                  << " nodes violating the epsilon bound and missing from the queue! ---\n";
        return false;
    } else {
        std::cout << "[FMTx FORENSICS] --- \033[1;32mPASSED\033[0m: All FMTX nodes are locally consistent or safely queued! ---\n";
        return true;
    }
}

bool KinodynamicANYFMTX::runTreePropagationForensics() {
    int nodes_checked = 0;
    int stranded_children = 0;
    int epsilon_saved_cascades = 0;

    for (size_t i = 0; i < tree_.size(); ++i) {
        FMTNode* child = tree_[i].get();
        
        if (child->getIndex() == root_state_index_ || 
            child->getLMC() == std::numeric_limits<double>::infinity() || 
            child->getParent() == nullptr) {
            continue;
        }

        FMTNode* parent = child->getParent();
        auto traj = child->getParentTrajectory();
        if (!traj || !traj->is_valid) continue; 

        double expected_cost = parent->getLMC() + traj->cost;
        double actual_cost = child->getLMC();
        double difference = actual_cost - expected_cost;

        nodes_checked++;

        if (difference > 1e-5) {
            if (parent->in_queue_) continue; 
            if (child->in_queue_) continue;

            // THE EPSILON FIX:
            // If the lag is less than epsilon, it was intentionally suppressed by the algorithm
            // to save queue operations.
            if (difference <= epsilon + 1e-5) {
                epsilon_saved_cascades++;
                continue; 
            }

            stranded_children++;
            std::cout << "\033[1;31m[FATAL PROPAGATION FAILURE]\033[0m Node " << child->getIndex() 
                      << " is illegally stranded!\n";
            std::cout << "   -> Child Cost: " << actual_cost << "\n";
            std::cout << "   -> Expected Cost via Parent: " << expected_cost << " (Diff: " << difference << " > " << epsilon << ")\n";
        }
    }

    std::cout << "-> Suppressed " << epsilon_saved_cascades << " tiny cascades due to Epsilon optimization.\n";

    if (stranded_children > 0) {
        std::cout << "[FMTx FORENSICS] --- \033[1;31mFAILED\033[0m: Found " << stranded_children 
                  << " illegally stranded children! ---\n\n";
        return false;
    } else {
        std::cout << "[FMTx FORENSICS] --- \033[1;32mPASSED\033[0m: " << nodes_checked 
                  << " tree edges verified. All nodes are Bellman-consistent or safely within epsilon! ---\n\n";
        return true;
    }
}




void KinodynamicANYFMTX::runFMT(SuboptimalityMetrics& metrics) {
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

    // Proxy wrapper for shadow nodes (compatible with PriorityQueue)
    struct ShadowProxy {
        int idx;
        std::vector<FMTShadow>* shadow;
        mutable bool in_queue_; 
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
    fmt_queue.add(p.get(), cost);          // this will set p->in_queue_ = true
    shadow[idx].in_queue = p->in_queue_;   // sync the shadow flag
    proxies.push_back(std::move(p));
};

    // Seed root
    shadow[root_state_index_].cost = 0.0;
    shadow[root_state_index_].in_unvisited = false;
    addProxy(root_state_index_, 0.0);

    int checks = 0;
    const double fmt_radius = neighborhood_radius_;

    // FMT* wavefront using live neighbor maps
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


        std::vector<int> newly_open;

        // Iterate over backward neighbors of z (nodes x that can reach z)
        for (const auto& [x_node, edge_info] : z_node->backwardNeighbors()) {
            if (edge_info.distance > fmt_radius) continue;
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
                if (edge_info_xy.distance > fmt_radius) continue;
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

    FMTX_INFO("[runFMT] FMT* expansion complete. checks=" << checks << "  radius=" << fmt_radius);


    // Store shadow results for later visualization
    fmt_shadow_cost_.assign(N, std::numeric_limits<double>::infinity());
    fmt_shadow_parent_.assign(N, -1);
    for (int i = 0; i < N; ++i) {
        fmt_shadow_cost_[i] = shadow[i].cost;
        fmt_shadow_parent_[i] = shadow[i].parent_idx;
    }
    fmt_shadow_valid_ = true;


    // COMPARISON & PARENT DIAGNOSTIC
    FMTX_INFO("[runFMT] ===== FMT* vs FMTX LMC Comparison =====");

    // ADDED: collect asymmetric-reachability nodes for visualization
    std::vector<Eigen::VectorXd> fmt_only_nodes;   // FMT* reached, FMTX INF
    std::vector<Eigen::VectorXd> fmtx_only_nodes;  // FMTX reached, FMT* INF

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

                // FMTX_WARN("[runFMT] Node " << idx <<" state: "<<tree_[idx]->getStateValue()<<","
                //     << " | FMT*=" << fmt_cost << " < FMTX=" << fmtx_lmc << " | Gap=" << gap
                //     << "\n      -> FMT* used Parent " << best_fmt_parent_idx <<" with lmc of: "<<shadow[best_fmt_parent_idx].cost<<", "
                //     << " (Parent's FMTX LMC=" << parent_fmtx_lmc << ", in_queue=" << is_parent_in_queue << ")"
                //     << "\n      -> FMTX used Parent " << fmtx_parent_idx
                //     << "\n      -> Edge in FMTX map? " << (is_edge_in_fmtx_map ? "YES" : "NO")
                //     << " | Collision-free? " << (is_edge_collision_free ? "YES" : "NO"));
                // // // ---------------------------

                ++metrics.nodes_with_cost_gap;
                metrics.total_cost_gap += gap;
                metrics.max_cost_gap = std::max(metrics.max_cost_gap, gap);
            } else {
                ++fmtx_strictly_better;
                max_gap_fmtx_better = std::max(max_gap_fmtx_better, fmt_cost - fmtx_lmc); // note: fmt_cost > fmtx_lmc

                // --- PARENT DIAGNOSTIC FOR FMTX BETTER ---
                int best_fmt_parent_idx = shadow[idx].parent_idx;
                double parent_fmtx_lmc = -1.0;
                bool is_parent_in_queue = false;
                bool is_edge_in_fmtx_map = false;
                bool is_edge_collision_free = false;

                // Get FMT* parent info (if any)
                if (best_fmt_parent_idx >= 0) {
                    FMTNode* fmt_parent = tree_[best_fmt_parent_idx].get();
                    parent_fmtx_lmc = fmt_parent->getLMC(); // cost of that parent in dynamic tree
                    is_parent_in_queue = fmt_parent->in_queue_;

                    // Check if FMTX knows about this edge (child -> parent)
                    auto it = node_ptr->forwardNeighbors().find(fmt_parent);
                    if (it != node_ptr->forwardNeighbors().end()) {
                        is_edge_in_fmtx_map = true;
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

            fmt_only_nodes.push_back(node_ptr->getStateValue().head(2));

            // FMTX_WARN("[runFMT] Node " << idx << " Reached by FMT* but INF in FMTX!");
        } else {
            ++fmtx_strictly_better;

            fmtx_only_nodes.push_back(node_ptr->getStateValue().head(2));

            // FMTX_WARN("[runFMT] Node " << idx << " Reached by FMTX but INF in FMT*!");
        }
    }

    FMTX_INFO("[runFMT] FMT* strictly better : " << fmt_strictly_better << " max_gap=" << max_gap_fmt_better);
    FMTX_INFO("[runFMT] FMTX strictly better : " << fmtx_strictly_better << " max_gap=" << max_gap_fmtx_better);




}




Eigen::VectorXd KinodynamicANYFMTX::saturate(const Eigen::VectorXd& newPoint, const Eigen::VectorXd& closestPoint, double delta) {
    int dimension = newPoint.size();
    Eigen::VectorXd saturatedPoint = newPoint;

    switch (dimension) {
        case 2: { // State: (x, y)
            double dist = (newPoint - closestPoint).norm();
            if (dist > delta) {
                saturatedPoint = closestPoint + (newPoint - closestPoint) * (delta / dist);
            }
            break;
        }

        case 3: { // State: (x, y, time)
            double dist = (newPoint - closestPoint).norm();
            if (dist > delta) {
                saturatedPoint = closestPoint + (newPoint - closestPoint) * (delta / dist);
            }
            break;
        }

        case 4: { // State: (x, y, theta, time)
            constexpr int X = 0, Y = 1, THETA = 2, TIME = 3;

            // Distance is calculated on Euclidean components (x, y, time)
            Eigen::Vector3d p1(newPoint(X), newPoint(Y), newPoint(TIME));
            Eigen::Vector3d p2(closestPoint(X), closestPoint(Y), closestPoint(TIME));
            double dist = (p1 - p2).norm();

            if (dist > delta) {
                double scale = delta / dist;
                // Saturate Euclidean parts
                saturatedPoint(X) = closestPoint(X) + (newPoint(X) - closestPoint(X)) * scale;
                saturatedPoint(Y) = closestPoint(Y) + (newPoint(Y) - closestPoint(Y)) * scale;
                saturatedPoint(TIME) = closestPoint(TIME) + (newPoint(TIME) - closestPoint(TIME)) * scale;

                // Saturate angle along the shortest path
                double thetaDiff = normalizeAngle(newPoint(THETA) - closestPoint(THETA));
                saturatedPoint(THETA) = closestPoint(THETA) + thetaDiff * scale;
            }
            // Always normalize the final angle
            saturatedPoint(THETA) = normalizeAngle(saturatedPoint(THETA));
            break;
        }

        case 5: { // State: (x, y, vx, vy, time)
            // For this state space, all components are Euclidean.
            // We calculate distance and saturate across all 5 dimensions.
            double dist = (newPoint - closestPoint).norm();
            if (dist > delta) {
                saturatedPoint = closestPoint + (newPoint - closestPoint) * (delta / dist);
            }
            break;
        }

        default:
            // Handle unsupported dimensions
            throw std::runtime_error("Unsupported state dimension for saturate function: " + std::to_string(dimension));
    }

    return saturatedPoint;
}



#if 0  // ===== DISABLED: dormant/accept-all variant (kept for reference; saturate+discard is active below) =====
void KinodynamicANYFMTX::updateNeighbors(const Eigen::VectorXd& sample_val, FMTNode* new_node) {

#if !STATIC
    std::vector<size_t> candidate_indices = kdtree_->radiusSearch(sample_val.head(kd_dim), neighborhood_radius_ + std::numeric_limits<double>::epsilon());
#endif
#if STATIC
    std::vector<size_t> candidate_indices = spatial_kdtree_->radiusSearch(sample_val.head(2), neighborhood_radius_ + std::numeric_limits<double>::epsilon());
#endif

    // OUTGOING VALIDATION & DIRECT COMMIT
    for (size_t idx : candidate_indices) {
        FMTNode* neighbor = tree_[idx].get();
        Trajectory traj_outgoing = statespace_->steer(sample_val, neighbor->getStateValue());
        
        if (traj_outgoing.is_valid && traj_outgoing.cost <= neighborhood_radius_ + std::numeric_limits<double>::epsilon()) {
            auto shared_traj_outgoing = std::make_shared<Trajectory>(std::move(traj_outgoing));
            
            EdgeInfo edge_info;
            edge_info.distance = shared_traj_outgoing->cost;
            edge_info.distance_original = shared_traj_outgoing->cost;
            edge_info.is_trajectory_computed = true;
            edge_info.cached_trajectory = shared_traj_outgoing;
            
            // Commit Forward 
            edge_info.is_initial = true;
            new_node->forwardNeighbors()[neighbor] = edge_info;
            edge_info.is_initial = false;
            neighbor->backwardNeighbors()[new_node] = edge_info;
            
            // Geometric Optimization
            if (is_geometric_mode_) {
                edge_info.is_initial = true;
                new_node->backwardNeighbors()[neighbor] = edge_info;
                edge_info.is_initial = false;
                neighbor->forwardNeighbors()[new_node] = edge_info;
            }
        }
    }

    // INCOMING VALIDATION
    if (!is_geometric_mode_) {
        for (size_t idx : candidate_indices) {
            FMTNode* neighbor = tree_[idx].get();
            Trajectory traj_incoming = statespace_->steer(neighbor->getStateValue(), sample_val);
            
            if (traj_incoming.is_valid && traj_incoming.cost <= neighborhood_radius_ + std::numeric_limits<double>::epsilon()) {
                
                auto shared_traj_incoming = std::make_shared<Trajectory>(std::move(traj_incoming));
                
                EdgeInfo edge_info;
                edge_info.distance = shared_traj_incoming->cost;
                edge_info.distance_original = shared_traj_incoming->cost;
                edge_info.is_trajectory_computed = true;
                edge_info.cached_trajectory = shared_traj_incoming;
                
                edge_info.is_initial = true;
                new_node->backwardNeighbors()[neighbor] = edge_info;
                edge_info.is_initial = false;
                neighbor->forwardNeighbors()[new_node] = edge_info;
            }
        }
    }
}


// /*
//   Eager new sample insertion, later in plan we do lazy propagation
//   In anytime fmtx eager approach we still dont care about all the collision checking connections from new node to neighbors or neighbors to new node
//   except maybe the new sampled node to its forward neighbors! so we only collision check to find the parent here! but in anytime rrtx we have to check all of the
//   edges and make them to have edge.distance of INF because that how rrtx works! but here because of lazy collision checking we dont have to do this!
//   Even this collision checking was not necessary if i could've found a way to make the extend to not become O(n log^2 (n))
//   But here the O(n log(n)) will be preserved because we only do one heap push if the new node finds the best parent!
// */
void KinodynamicANYFMTX::extend(int num_samples) {
    if (num_samples <= 0) return;

    // if (!is_geometric_mode_ && dimension_ >= 3) {
    //     const double R = T_robot * statespace_->getMaxVelocity();
    //     // Degenerate footprint near goal: adding nodes in a collapsing start-disk
    //     // buys nothing and costs everything. Skip the batch.
    //     if (R < 2*neighborhood_radius_) return;   // tune threshold to taste
    // }
    
    std::vector<int> added_node_indices;

    int successfully_added = 0;


    // Raw-attempt cap: bounds the loop when the footprint is degenerate
    // (near goal, T_robot <= t_reach rejects almost every draw).
    // Tune the multiplier; large enough that it never bites in normal regions.
    const int max_attempts = num_samples * 20;
    int attempts = 0;


    // std::cout<<"T_ROBOT: "<<T_robot<<"\n";
    // for (int i = 0; i < num_samples; ++i) {
    while (successfully_added < num_samples && attempts < max_attempts){
        ++attempts;   // every iteration consumes an attempt, including the ones that `continue`



        // Generate Sample (centralized strategy: Halton low-dispersion or i.i.d. uniform)
        Eigen::VectorXd sample_val = statespace_->sampleUnregistered(lower_bounds_, upper_bounds_);
        
        // if (!is_geometric_mode_ && dimension_ >= 3) {
        //     Eigen::Vector2d root_position_ = problem_->getStart().head(2);
        //     const int t_idx = dimension_ - 1;
        //     auto dist = (root_position_ - sample_val.head(2)).norm();
        //     const double t_reach = dist / statespace_->getMaxVelocity(); // minTimeToReachNode (time-to-goal model, no S.start offset)
        //     const double t_samp  = sample_val[t_idx];

        //     const bool too_early = (t_samp < t_reach);
        //     const bool behind    = (t_samp > T_robot);
        //     if ((too_early || behind) && T_robot > t_reach) {
        //         // Remap the ALREADY-DRAWN time into [t_reach, t_robot]
        //         // u is uniform in [0,1)
        //         const double u = (t_samp - lower_bounds_[t_idx]) /
        //                         (upper_bounds_[t_idx] - lower_bounds_[t_idx]);
        //         auto before = sample_val[t_idx];
        //         sample_val[t_idx] = t_reach + u * (T_robot - t_reach);
        //         // std::cout<<"BEFORE: "<<before<<", AFTER: "<<sample_val[t_idx]<<"\n";
        //     }
        // }


        // if (!is_geometric_mode_ && dimension_ >= 3) {
        //     Eigen::Vector2d root_position_ = problem_->getStart().head(2);
        //     const int t_idx = dimension_ - 1;
        //     const double dist   = (root_position_ - sample_val.head(2)).norm();
        //     const double t_reach = dist / statespace_->getMaxVelocity();

        //     // Footprint feasibility: empty interval -> reject and redraw position.
        //     if (T_robot <= t_reach) {
        //         continue;                 // do NOT increment successfully_added
        //     }
        //     auto before = sample_val[t_idx];
        //     // UNCONDITIONAL remap: u is uniform on [0,1), so t_new ~ Unif[t_reach, T_robot].
        //     const double u = (sample_val[t_idx] - lower_bounds_[t_idx]) /
        //                     (upper_bounds_[t_idx] - lower_bounds_[t_idx]);
        //     sample_val[t_idx] = t_reach + u * (T_robot - t_reach);
        //     // std::cout<<"BEFORE: "<<before<<", AFTER: "<<sample_val[t_idx]<<"\n";
        // }

        if (!is_geometric_mode_ && dimension_ >= 3) {
            const int t_idx = dimension_ - 1;
            // One-sided goal-reachability cone (shared StateSpace helper). root_xy = goal
            // (backward search: the problem's "start" IS the goal).
            if (!statespace_->remapTimeToGoalCone(sample_val, Eigen::Vector2d(problem_->getStart().head(2)),
                                                  T_robot, lower_bounds_[t_idx], upper_bounds_[t_idx])) {
                continue;                 // do NOT increment successfully_added
            }
        }




        // NOT USED IN FMTX, I ONLY PUT IT FOR DEBUG PURPOSES
        // // Find Nearest
        // std::vector<size_t> nearest_indices = kdtree_->knnSearch(sample_val.head(kd_dim), 1);
        // FMTNode* nearest_node = tree_[nearest_indices[0]].get();
        // Eigen::VectorXd nearest_state = nearest_node->getStateValue();
        
        // // Saturate
        // sample_val = saturate(sample_val, nearest_state, delta);


        // This is just for fmtx to be complete! in case we have static obstalces!
        if (!obs_checker_->isObstacleFree(sample_val)) {
            continue;
        }

        // Create Node Object (Temporarily)
        // We create the node to get the pointer, but we don't push it to tree_ yet.
        auto node = std::make_unique<FMTNode>(statespace_->addState(sample_val), tree_.size());
        
    
#if USE_THREAT_SET_STRATEGY
        // INITIALIZE THREATS FOR NEW NODES
        // Use previous_obstacles_ because it contains the fully populated predicted_path!
        /*
            We immediately update the threat set of the new sample because even though we collision check later at this function to find it a collision free parent
            but later a new sample maybe generated and rewire that previous new sample! so new samples need to update their threat set immediately and we cant ait for the 
            addNewObstacle function to update the threat set!
        */
        for (const auto& [name, ob] : previous_obstacles_) {
            if (obs_checker_->isNodeInObstacleTube(node->getStateValue(), ob, delta)) {
                node->threats.push_back(&ob); // Pointer insertion
            }
        }
        
#endif


        // POPULATE NEIGHBORS (no discard)
        // updateNeighbors performs the steering and populates the neighbor maps in both
        // directions. A node with no valid edge yet is kept dormant; the wavefront connects
        // it once a later sample provides an edge.
        updateNeighbors(sample_val, node.get());

        // COMMIT TO TREE
        // If we reach here, the node is good.
        int node_index = tree_.size();
        
        if (!is_geometric_mode_ && node->getStateValue().size() > 2) {
            double absolute_t = node->getStateValue().tail<1>()[0];
            node->setTimeToGoal(absolute_t);
        } else {
            node->setTimeToGoal(0.0);
        }

        kdtree_->addPoint(sample_val.head(kd_dim)); 
        kdtree_->buildTree(); // BUILD KD-TREE --> BUILD TREE function is empty in case you use DynamicWeightedNanoFlann
#if STATIC
        spatial_kdtree_->addPoint(sample_val.head(2)); 
        spatial_kdtree_->buildTree();
#endif

        tree_.push_back(std::move(node));
        added_node_indices.push_back(node_index);

        // WE SUCCEEDED! Increment the actual counter.
        successfully_added++;

    }

    if (added_node_indices.empty()) return;


    // UPDATE NEIGHBORHOOD RADIUS
    shrinkingBallRadius();
    // SEED V_OPEN (EAGER INSERTION + LAZY PROPAGATION WITH THREATS)
    /*
        Mention this in the paper: Mind that there is no immediate rewiring like rrtx here!
        the propagation is lazy! but in rrtx there is an immediate rewiring because the neighbors 
        need to know if they are getting better through the new sample and then go into the queue
        for triggering the propagation (when needed!) but here the new node goes into the queue and
        triggers the new propagation when needed so we dont have to check the neighbors right away!
        its ineherent to the main fmt expand function
    */ 
    for (int idx : added_node_indices) {
        FMTNode* new_node = tree_[idx].get();

        std::vector<std::pair<double, FMTNode*>> candidate_parents;
        candidate_parents.reserve(new_node->forwardNeighbors().size());

        for (auto& [neighbor, edge_info] : new_node->forwardNeighbors()) {
            if (neighbor->getLMC() == std::numeric_limits<double>::infinity()) {
                continue; 
            }
            double potential_cost = neighbor->getLMC() + edge_info.distance; 
            candidate_parents.emplace_back(potential_cost, neighbor); // emplace_back is slightly faster than push_back
        }

        if (candidate_parents.empty()) continue;

        // Build a Min-Heap in strictly O(K) time instead of O(K log K)
        // We use std::greater to make the lowest cost bubble to the top
        std::make_heap(candidate_parents.begin(), candidate_parents.end(), std::greater<std::pair<double, FMTNode*>>());

        bool connected = false;

        // Eagerly check candidates by popping from the Min-Heap
        while (!candidate_parents.empty()) {
            // Pop the lowest cost element from the heap in O(log K) time
            std::pop_heap(candidate_parents.begin(), candidate_parents.end(), std::greater<std::pair<double, FMTNode*>>());
            auto candidate = candidate_parents.back();
            candidate_parents.pop_back();

            FMTNode* potential_parent = candidate.second;
            auto& edge_info = new_node->forwardNeighbors().at(potential_parent);

            auto traj_xy = edge_info.cached_trajectory;
            if (!traj_xy->is_valid) continue;
            bool collision_free = true;
#if USE_THREAT_SET_STRATEGY
            // We ONLY check the exact obstacle pointers in memory!
            for (const Obstacle* ob_ptr : new_node->threats) {
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*traj_xy, *ob_ptr)) {
                    collision_free = false;

                    // CACHE STATIC WALLS FOR LATER
                    if (!ob_ptr->is_dynamic) {
                        edge_info.permanently_blocked = true;
                    }
                    

                    break; // Short-circuit
                }
            }
#else
            // Default Blind strategy
            // Again, brute-force must check everything in previous_obstacles_
            for (const auto& [name, ob] : previous_obstacles_) {
                last_replan_metrics_.obstacle_checks++;
                collision_checked_++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*traj_xy, ob)) {
                    
                    // CACHE STATIC WALLS FOR LATER
                    if (!ob.is_dynamic) {
                        edge_info.permanently_blocked = true;
                    }

                    collision_free = false;
                    break;
                }
            }
#endif

            if (collision_free) {
                // SUCCESS!
                new_node->setLMC(candidate.first);
                new_node->setParent(potential_parent, traj_xy);
                connected = true;
                break; // Stop checking candidates.
            }
            /*
                If NOT collision free, do nothing! Just let the while loop 
                pop the next candidate from the heap. The edge remains intact 
                for future dynamic repairs.
                To me this is like Bidirectional Approach because at setup we have start and goal node so the way FMTX preserves samples is like a bidirectional approach 
                Although the tree is created from root but since FMTX is a dynamic replanner we cache the neighbors any way so as soon as the two sides reach each other, voilla! 
            
            */
        }

        /*
            Important Insight on AO with epsilon usage: as you can see the new samples is added to the tree and now is in vopen heap! we do not immediately improve the neighbors
            which can use this! but if nothing happens and this new samples didnt get removed from the heap the neighbors would use this to connect. epsilon is not in the picture 
            until the queue operations in plan function!
            now if x's cost is improved little then we wouldn't cascade! 
            so the argument that when we use epsilon we have epsilon bounded AO is incorrect because those children that we didnt improve will get better by new samples!
            so all in all the new sample will eventually betters their immediate neighbors if partial update allows (i.e., its useful) because the new sample is in the vopen and when
            plan function runs then immediate neighbors will get improved anyway! and only then afterward the epsilon comes into picture to decide if the immeidate neighbor should also
            get into queue or not!
        */
        // THE SINGLE HEAP INSERTION
        if (connected) {
            v_open_heap_.add(new_node, new_node->getLMC());
        } 
    }
}




#endif // ===== end disabled dormant/accept-all variant =====


// NOTE: updateNeighbors() has been INLINED into extend() so the
// outgoing (forward) steers can run BEFORE the parent/discard decision and the
// expensive incoming (reverse) steers only AFTER a node survives (discarded nodes
// skip them entirely). The standalone version is preserved (disabled) in the
// #if 0 block above for reference. Header declaration kept (now unused).
/*
  While ANYFMTX is mathematically proven to generate the same candidate graph 
  richness as ANYRRTX, achieving 100% byte-for-byte spatial parity of all nodes 
  over a long run is fundamentally impossible by design.
  The discprency between RRTX and FMTX sample points is expected becuase 
  cullNeighbor ins FMTX is not as aggressive as RRTX so KNN in saturate might results in difference 
  places in FMTX compared to RRTX! 

*/
void KinodynamicANYFMTX::extend(int num_samples) {
    if (num_samples <= 0) return;
    int successfully_added = 0;
    const int max_attempts = num_samples * 20; // Keep the buffer!
    int attempts = 0;

    while (successfully_added < num_samples && attempts < max_attempts){
        ++attempts;

        // 1. Generate Sample (centralized strategy: i.i.d. or Halton via toggle)
        Eigen::VectorXd sample_val = statespace_->sampleUnregistered(lower_bounds_, upper_bounds_);

        if (!is_geometric_mode_ && dimension_ >= 3) {
            const int t_idx = dimension_ - 1;
            // One-sided goal-reachability cone (shared StateSpace helper).
            if (!statespace_->remapTimeToGoalCone(sample_val, Eigen::Vector2d(problem_->getStart().head(2)),
                                                  T_robot, lower_bounds_[t_idx], upper_bounds_[t_idx])) {
                continue;
            }
        }

        // ==========================================
        // 2. SATURATE TO TREE (RRTX Parity)
        // ==========================================
        std::vector<size_t> nearest_indices = kdtree_->knnSearch(sample_val.head(kd_dim), 1);
        FMTNode* nearest_node = tree_[nearest_indices[0]].get();
        Eigen::VectorXd nearest_state = nearest_node->getStateValue();

        sample_val = saturate(sample_val, nearest_state, delta);
        // ==========================================


        if (!obs_checker_->isObstacleFree(sample_val)) continue;

        // 3. Create Node (but DO NOT add to tree_ or kdtree_ yet)
        auto node = std::make_unique<FMTNode>(statespace_->addState(sample_val), tree_.size());

#if USE_THREAT_SET_STRATEGY
        for (const auto& [name, ob] : previous_obstacles_) {
            if (obs_checker_->isNodeInObstacleTube(node->getStateValue(), ob, delta)) {
                node->threats.push_back(&ob);
            }
        }
#endif

        // 4. NEIGHBOR SEARCH (shared by the outgoing pass here and, if the node
        //    survives the discard check, the deferred incoming pass below).
#if !STATIC
        std::vector<size_t> candidate_indices = kdtree_->radiusSearch(sample_val.head(kd_dim), neighborhood_radius_ + std::numeric_limits<double>::epsilon());
#endif
#if STATIC
        std::vector<size_t> candidate_indices = spatial_kdtree_->radiusSearch(sample_val.head(2), neighborhood_radius_ + std::numeric_limits<double>::epsilon());
#endif

        // 4a. OUTGOING VALIDATION (verify-then-wire: writes ONLY into the new node's
        //     own maps; existing nodes' maps stay untouched until the commit step).
        //     Only outgoing (forward) edges decide parent feasibility, so these are
        //     the ONLY steers we pay for BEFORE the discard check. The expensive
        //     reverse (incoming) steers are deferred until the node is actually kept
        //     (mirrors RRTX::extend: find a parent from outgoing edges first, only
        //     then evaluate the incoming edges).
        for (size_t idx : candidate_indices) {
            FMTNode* neighbor = tree_[idx].get();
            Trajectory traj_outgoing = statespace_->steer(sample_val, neighbor->getStateValue());

            if (traj_outgoing.is_valid && traj_outgoing.cost <= neighborhood_radius_ + std::numeric_limits<double>::epsilon()) {
                auto shared_traj_outgoing = std::make_shared<Trajectory>(std::move(traj_outgoing));

                EdgeInfo edge_info;
                edge_info.distance = shared_traj_outgoing->cost;
                edge_info.distance_original = shared_traj_outgoing->cost;
                edge_info.is_trajectory_computed = true;
                edge_info.cached_trajectory = shared_traj_outgoing;
                edge_info.is_initial = true;

                node->forwardNeighbors()[neighbor] = edge_info;
                if (is_geometric_mode_) {
                    node->backwardNeighbors()[neighbor] = edge_info;
                }
            }
        }

        bool connected = false;

        // 5. EAGER PARENT SEARCH (Uses your exact existing logic)
        std::vector<std::pair<double, FMTNode*>> candidate_parents;
        candidate_parents.reserve(node->forwardNeighbors().size());

        for (auto& [neighbor, edge_info] : node->forwardNeighbors()) {
            if (neighbor->getLMC() == std::numeric_limits<double>::infinity()) continue;
            double potential_cost = neighbor->getLMC() + edge_info.distance;
            candidate_parents.emplace_back(potential_cost, neighbor);
        }

        if (!candidate_parents.empty()) {
            std::make_heap(candidate_parents.begin(), candidate_parents.end(), std::greater<std::pair<double, FMTNode*>>());

            while (!candidate_parents.empty()) {
                std::pop_heap(candidate_parents.begin(), candidate_parents.end(), std::greater<std::pair<double, FMTNode*>>());
                auto candidate = candidate_parents.back();
                candidate_parents.pop_back();

                FMTNode* potential_parent = candidate.second;
                auto& edge_info = node->forwardNeighbors().at(potential_parent);
                auto traj_xy = edge_info.cached_trajectory;

                if (!traj_xy->is_valid) continue;

                bool collision_free = true;

#if USE_THREAT_SET_STRATEGY
                for (const Obstacle* ob_ptr : node->threats) {
                    last_replan_metrics_.obstacle_checks++;
                    if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*traj_xy, *ob_ptr)) {
                        collision_free = false;
                        if (!ob_ptr->is_dynamic) edge_info.permanently_blocked = true;
                        break;
                    }
                }
#else
                for (const auto& [name, ob] : previous_obstacles_) {
                    last_replan_metrics_.obstacle_checks++;
                    collision_checked_++;
                    if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*traj_xy, ob)) {
                        collision_free = false;
                        if (!ob.is_dynamic) edge_info.permanently_blocked = true;
                        break;
                    }
                }
#endif

                if (collision_free) {
                    node->setLMC(candidate.first);
                    node->setParent(potential_parent, traj_xy);
                    connected = true;
                    break; // Parent found!
                }
            }
        }

        // 6. FAILURE PATH: trivial discard. No existing node's maps were mutated,
        //    so there is nothing to unlink; the unique_ptr just goes out of scope.
        //    Crucially this is BEFORE the reverse steers, so a discarded node never
        //    pays for its (expensive, high-D) incoming trajectories.
        if (!connected) {
            continue;
        }

        // 6a. INCOMING VALIDATION (deferred to here: only KEPT nodes pay for it).
        //     Reuses the same candidate_indices as the outgoing pass. Geometric mode
        //     already filled backwardNeighbors during the outgoing pass (incoming ==
        //     outgoing), so it is skipped here.
        if (!is_geometric_mode_) {
            for (size_t idx : candidate_indices) {
                FMTNode* neighbor = tree_[idx].get();
                // Time-cone prune: an incoming edge neighbor->new_node only lets `neighbor`
                // (which has tau > tau(new_node)) adopt new_node as a parent. If neighbor is
                // already beyond the robot's reachable cone it will never be on the robot's
                // path, so skip its (expensive, high-D) reverse steer. EXACT prune.
                // if (TIME_CONE_PRUNED(neighbor, T_robot)) continue;  // DISABLED (test): graph-construction prune permanently omits an incoming edge
                Trajectory traj_incoming = statespace_->steer(neighbor->getStateValue(), sample_val);

                if (traj_incoming.is_valid && traj_incoming.cost <= neighborhood_radius_ + std::numeric_limits<double>::epsilon()) {
                    auto shared_traj_incoming = std::make_shared<Trajectory>(std::move(traj_incoming));

                    EdgeInfo edge_info;
                    edge_info.distance = shared_traj_incoming->cost;
                    edge_info.distance_original = shared_traj_incoming->cost;
                    edge_info.is_trajectory_computed = true;
                    edge_info.cached_trajectory = shared_traj_incoming;
                    edge_info.is_initial = true;

                    node->backwardNeighbors()[neighbor] = edge_info;
                }
            }
        }

        // ==========================================
        // 6b. COMMIT WIRING (Verify-then-Wire)
        //     Parent is confirmed, so it is now safe to write the back-edges
        //     into the existing neighbors' maps. No new collision checks here.
        // ==========================================
        {
            // Forward edges of new_node -> back-edges into neighbors.
            for (auto& [neighbor, edge_info] : node->forwardNeighbors()) {
                EdgeInfo back = edge_info;
                back.is_initial = false;
                neighbor->backwardNeighbors()[node.get()] = back;

                if (is_geometric_mode_) {
                    // symmetric forward edge into neighbor
                    neighbor->forwardNeighbors()[node.get()] = back;
                }
            }

            // Backward edges of new_node -> forward-edges into neighbors.
            if (!is_geometric_mode_) {
                for (auto& [neighbor, edge_info] : node->backwardNeighbors()) {
                    EdgeInfo fwd = edge_info;
                    fwd.is_initial = false;
                    neighbor->forwardNeighbors()[node.get()] = fwd;
                }
            }
        }

        // 7. COMMIT TO KD-TREE (Only reaches here if safely connected)
        if (!is_geometric_mode_ && node->getStateValue().size() > 2) {
            node->setTimeToGoal(node->getStateValue().tail<1>()[0]);
        } else {
            node->setTimeToGoal(0.0);
        }

        kdtree_->addPoint(sample_val.head(kd_dim));
        kdtree_->buildTree();
#if STATIC
        spatial_kdtree_->addPoint(sample_val.head(2));
        spatial_kdtree_->buildTree();
#endif

        /*
            Important Insight on AO with epsilon usage: as you can see the new samples is added to the tree and now is in vopen heap! we do not immediately improve the neighbors
            which can use this! but if nothing happens and this new samples didnt get removed from the heap the neighbors would use this to connect. epsilon is not in the picture 
            until the queue operations in plan function!
            now if x's cost is improved little then we wouldn't cascade! 
            so the argument that when we use epsilon we have epsilon bounded AO is incorrect because those children that we didnt improve will get better by new samples!
            so all in all the new sample will eventually betters their immediate neighbors if partial update allows (i.e., its useful) because the new sample is in the vopen and when
            plan function runs then immediate neighbors will get improved anyway! and only then afterward the epsilon comes into picture to decide if the immeidate neighbor should also
            get into queue or not!
        */


        v_open_heap_.add(node.get(), node->getLMC());
        tree_.push_back(std::move(node));

        successfully_added++;
    }

    if (successfully_added > 0) {
        shrinkingBallRadius(); // Update radius at the very end
    }
}




/*
    Insights on CullNeighbor: even though we put parent filter in the cullNeighbor this doesnt mean later in time a child of a parent will
    still be in the parent's backward neighbors. because that child can alway choose the parent that is is_initial (even though far away wrt to rn) 
    if node 623 is child and node 503 is parent there is chance 503 forgot 623 but 623 didnt! (thats why 503 become parent!)
    in this case 503 cant notify 623 of its updated cost (in case of improvement) because fmtx is not like rrtx 
    so in rrtx 623 in updateLMC goes through the outgoing and can update lmc by using 503! but in fmtx 
    we loop through the backward (incoming) of 503 first to find the 623! but 623 got culled from 503's incoming neighbors so 623 never improves even if its parent (503) 
    has improved 
    The perfect Solution which i found for the cost propagation cascade death also fixes the above problem and we just need to decouple the current tree update (current parent to children)
    from the wavefront expansion.
    because 623 eventually gets notified by using its current parent which is 503 in the first for loop of the plan function
    Mind that the wavefront expansion can partly do this job but its not guranteed but the loop over children of popped z gurantees propagation never gets interupted
*/


void KinodynamicANYFMTX::cullNeighbors(FMTNode* v) {

    // static long long cull_count_ = 0;  // total successful culls this run
    // Skip loop if radius hasn't shrunk
    if (v->last_culled_radius_ > 0 && 
        (v->last_culled_radius_ / neighborhood_radius_) < 1.0001) {
        return;
    }

    auto& outgoing = v->forwardNeighbors();
    auto it = outgoing.begin();

    while (it != outgoing.end()) {
        auto neighbor = it->first;
        auto& edge = it->second;
        double edge_cost = edge.cached_trajectory->cost;

        // An edge is only considered for culling if it is longer than the current radius
        // AND the node is not the current parent in the shortest-path tree
        if (edge_cost > (neighborhood_radius_ + std::numeric_limits<double>::epsilon()) && neighbor != v->getParent()) {

            // SYMMETRIC CULL (Neighbor's Side)
            // Remove 'v' from the neighbor's backward list if it wasn't an 'initial' birth-neighbor.
            auto& incoming = neighbor->backwardNeighbors();
            if (auto incoming_it = incoming.find(v); incoming_it != incoming.end()) {
                if (!incoming_it->second.is_initial) {
                    incoming.erase(incoming_it);
                }
            }

            // SOURCE CULL (v's Side)
            // Remove the neighbor from the active forward set if it wasn't an 'initial' birth-neighbor.
            // Preservation of N_0 neighbors is sacred for optimality
            if (!edge.is_initial) {
                it = outgoing.erase(it);
                // outgoing.erase(it++); //for absl
                // ++cull_count_;  // a real cull happened
                continue; // Move to next neighbor
            }
        }
        ++it;
    }
    v->last_culled_radius_ = neighborhood_radius_;
    // std::cout<<"count: "<<cull_count_<<"\n";
}



// void KinodynamicANYFMTX::cullNeighbors(FMTNode* v) {

//     // Track both radius and time
//     bool radius_changed = (v->last_culled_radius_ == 0 || 
//                            (v->last_culled_radius_ / neighborhood_radius_) > 1.0001);
//     bool time_advanced = (v->last_culled_T_robot < T_robot - 1e-6);


//     if (!radius_changed && !time_advanced) {
//         return;  // Nothing to cull
//     }


//     auto& outgoing = v->forwardNeighbors();
//     auto it = outgoing.begin();

//     while (it != outgoing.end()) {
//         auto neighbor = it->first;
//         auto& edge = it->second;


//         // ← ADD: Time-cone prune
//         if (TIME_CONE_PRUNED(neighbor, T_robot)) {
//             // SYMMETRIC CULL
//             auto& incoming = neighbor->backwardNeighbors();
//             if (auto incoming_it = incoming.find(v); incoming_it != incoming.end()) {
//                 if (!incoming_it->second.is_initial) {
//                     incoming.erase(incoming_it);
//                 }
//             }

//             // SOURCE CULL
//             if (!edge.is_initial) {
//                 it = outgoing.erase(it);
//                 continue;
//             }
//         }


//         double edge_cost = edge.cached_trajectory->cost;

//         // An edge is only considered for culling if it is longer than the current radius
//         // AND the node is not the current parent in the shortest-path tree
//         if (edge_cost > (neighborhood_radius_ + std::numeric_limits<double>::epsilon()) && neighbor != v->getParent()) {

//             // SYMMETRIC CULL (Neighbor's Side)
//             // Remove 'v' from the neighbor's backward list if it wasn't an 'initial' birth-neighbor.
//             auto& incoming = neighbor->backwardNeighbors();
//             if (auto incoming_it = incoming.find(v); incoming_it != incoming.end()) {
//                 if (!incoming_it->second.is_initial) {
//                     incoming.erase(incoming_it);
//                 }
//             }

//             // SOURCE CULL (v's Side)
//             // Remove the neighbor from the active forward set if it wasn't an 'initial' birth-neighbor.
//             // Preservation of N_0 neighbors is sacred for optimality
//             if (!edge.is_initial) {
//                 it = outgoing.erase(it);
//                 // outgoing.erase(it++); //for absl
//                 // ++cull_count_;  // a real cull happened
//                 continue; // Move to next neighbor
//             }
//         }
//         ++it;
//     }
//     v->last_culled_radius_ = neighborhood_radius_;
//     v->last_culled_T_robot = T_robot;  // ← ADD THIS FIELD

//     // std::cout<<"count: "<<cull_count_<<"\n";
// }




/*
    * SEMANTICS OF COST: 'g' vs 'lmc' (Inspired by D* Lite / RRTX)
    * To maintain strict mathematical correctness during dynamic replanning, we decouple 
    * a node's cost into two distinct values:
    * 
    * 1. 'lmc' (Locally Minimal Cost): The absolute best cost known *right now*. It updates 
    *    instantly the moment a better path is found.
    * 2. 'g' (Broadcast/Frozen Cost): The cost the node had the last time it popped from 
    *    the priority queue. It represents the "official" cost that has been fully 
    *    propagated to the node's subtree.
    * 
    * GOLDEN RULES FOR USAGE:
    * 
    * RULE 1: The Broadcaster (z) uses 'g'.
    * When the active node 'z' pops from the queue, it commits its live cost to history 
    * ( z->setG(z->getLMC()) ). It must then broadcast `z->getG()` to its children and 
    * neighbors to ensure the wavefront uses the strictly committed cost.
    *   -> Example: double cost_via_z = z->getG() + edge_distance;
    * 
    * RULE 2: The Receiver (x or child) protects its 'lmc'.
    * When a node checks if an incoming path is better than its current best, it evaluates 
    * the offer against its live 'lmc'.
    *   -> Example: if (x->getLMC() > cost_via_z) { x->setLMC(cost_via_z); }
    * 
    * RULE 3: The Parent Searcher (x) looks at the 'lmc' of queued parents (y).
    * When 'x' is suboptimal and searches for a new parent 'y' that is waiting in the queue, 
    * it MUST read `y->getLMC()`. Because 'y' hasn't popped yet, its 'g' is stale, but 
    * its 'lmc' holds the newly discovered optimal promise.
    *   -> Example: double cost_via_y = y->getLMC() + traj_xy->cost;
    * 
    * RULE 4: The Epsilon Queue Bouncer compares 'g' vs 'lmc'.
    * To decide if a node needs to enter the priority queue, we check if the new reality 
    * (lmc) is significantly better than the frozen history (g). If the gap exceeds epsilon, 
    * it enters the queue to sync its descendants.
    *   -> Example: if (x->getG() - x->getLMC() > epsilon) { v_open_heap_.add(x); }
*/



void KinodynamicANYFMTX::plan() {
#if DEBUG
    SuboptimalityMetrics dbg_metrics;
#endif

    extend(num_of_samples_); // Add a small batch (e.g., 10) instead of 1

    // // ← ADD THESE LINES
    // int iteration = 0;
    // int total_neighbors_considered = 0;
    // int total_cone_pruned = 0;
    // // ← END ADD


#if VIS
    // VISUALIZATION: Visualize all nodes currently in the Open Set (V_open)
    if (visualization_) {
        std::vector<Eigen::VectorXd> open_set_nodes;
        const auto& heap_data = v_open_heap_.getHeap();
        for (const auto& element : heap_data) {
            FMTNode* node = element.second;
            if (node) {
                open_set_nodes.push_back(node->getStateValue());
            }
        }
        if (!open_set_nodes.empty()) {
            visualization_->visualizeNodes(open_set_nodes, "map", std::vector<float>{1.0f, 0.0f, 0.0f}, "v_open_heap_nodes");
        }
    }
#endif
    /*
        If heap is empty, we can't plan. 
        Check if we should stop based on robot cost
        If the robot node (the node that the robot is reaching to) is valid,
        and the best node in heap is worse than robot's current cost, stop.
    */
    while (!v_open_heap_.empty() &&
          (!partial_update || 
                robot_node_ == nullptr || 
                robot_node_->in_queue_ ||
                robot_node_->getLMC() == std::numeric_limits<double>::infinity() ||
                // v_open_heap_.top().first < robot_node_->getLMC() + bridge_cost_))
                v_open_heap_.top().first < robot_node_->getLMC()))
    {


        // // ← ADD THIS
        // iteration++;
        // int neighbors_this_iteration = 0;
        // int cone_pruned_this_iteration = 0;
        // // ← END ADD

        auto top_element = v_open_heap_.top();
        double cost = top_element.first;
        FMTNode* z = top_element.second;
        int zIndex = z->getIndex();



        // -------- SKIPPING USELESS NODES (time-cone reachability prune) ----------------
        // If this node's time-to-goal exceeds the robot's remaining time budget, it can
        // never lie on any feasible path the robot can still execute (tau strictly
        // decreases along every edge, T_robot is monotone), so expanding it cannot help
        // any relevant node. EXACT prune -> AO preserved. See time_cone_prune.hpp.
            if (TIME_CONE_PRUNED(z, T_robot)) {
                // col_stats_.plan_ignored++;  // ← COUNT PRUNE (z)
                v_open_heap_.pop();
                continue;
            }
        // ------------------------------------------------

        /*
            when a node becomes z (meaning it is acting as a parent and expanding the wavefront), 
            it broadcasts its cost. Right after we read z, we lock its broadcast cost
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
            or in the intial phase where the first propagation is involved (which is not of concern here)
            in addNewObstacle we make nodes with trajectory in the obstacle's tube orphan and make their cost inf and sever their parent relationship! but we keep the tree nodes that are collision free even though they are in the tube
            but how about removeObstacle? this frees up some nodes that was severed in the addNewObstacle so they are still having inf cost with no parent.
            in both cases we need to check the collision in the standard plan cycle! but i dont think when a node is already part of the tree and has parent is caused by addNewObstacle or removeObstacle! and i think its safe to just update the cost
            of that node with its current parent! but if the parent is something other than the current parent we need to collision check which happens in the standard implicit rewiring below!
        */

        for (FMTNode* child : z->children_) {

            // // ← ADD THIS
            if (TIME_CONE_PRUNED(child, T_robot)) {
                continue;  // Don't wake children beyond the cone (A child beyond the cone can never help the robot’s path. Don’t wake it.)
            }
            // // ← END

            auto traj = child->getParentTrajectory();
            if (!traj) continue;

            // // Diagnostic: check if the tree edge is marked as blocked in the Phase 2 cache --> THIS WILL NEVER HAPPEN!
            // auto& edge_info = child->forwardNeighbors().at(z);
            // if (edge_info.distance == std::numeric_limits<double>::infinity()) {
            //     FMTX_WARN("[FMTX_Phase1] Tree edge (" << z->getIndex() << " → " << child->getIndex()
            //             << ") is marked BLOCKED in cache (distance = inf) but is in z->children_. "
            //             << "This violates the assumption that tree edges are collision-free.");
            // }

            

            /*
                For the cost via z below in the loop and bellman i used edge.distance instead of traj->cost because here its not needed because 
                this edge already collision checked in the addnewobstacle!
                but maybe its needed i dont know!
                use traj->cost in Phase 1 because it represents the true, immutable edge cost, whereas edge.distance is now a mutable cache that tracks blocked edges within the current cycle. Phase 1 is about propagating mathematically correct Bellman costs down known-safe tree edges, so relying on the ground truth rather than a Phase 2 optimization artifact avoids fragile coupling between the two phases.


                For any valid edge, edge.distance == traj->cost by construction (you initialize distance = trajectory->cost). The only time they diverge is when Phase 2 overwrites distance with $\infty$ to cache a collision failure. So the question is really: do you want that $\infty$ poison to be visible in Phase 1?
                In Phase 2 the answer is yes. The whole point of cost_via_y = y->getLMC() + edge_info_xy.distance is that a blocked edge evaluates to $\infty$ and is automatically never selected as best parent. That’s the optimization working as intended. edge.distance is the right value there precisely because it carries the “blocked this cycle” semantics.
                In Phase 1 the answer is no, for two reasons:
                Contract. Phase 1 propagates the strict Bellman cost down edges that are already known safe (the parent-child tree edges). There is no “skip blocked edge” decision to make here. You’re not choosing among candidates, you’re pushing g(P) + c(P,x) down a fixed safe edge. The correct quantity is the unconditional true cost, which is traj->cost.
                Decoupling. edge.distance is now a mutable, Phase-2-owned cache. If you read it in Phase 1 you’re coupling your Bellman correctness to Phase 2’s caching state and to whatever removeObstacle did or didn’t reset. Even if you can argue the parent edge can never be $\infty$ (a blocked edge can never be selected as parent, so the parent edge’s distance stays valid), that’s a fragile invariant to lean on for the one phase whose entire job is keeping the cost structure mathematically intact. traj->cost has no such coupling.


                Why can’t the cache be used in Phase 1?
                Because Phase 1 propagates cost down known-safe tree edges. The collision status is irrelevant there.
                The Key Insight: Phase 1 edges CAN’T be in collision
            */
            double cost_via_z = z->getG() + traj->cost;
            
            // If the parent brings a better cost, push it down to the child
            if (child->getLMC() > cost_via_z) {
                child->setLMC(cost_via_z);

                // Wake the child up so it can propagate the cost to its own children
                if (child->in_queue_) {
                    v_open_heap_.update(child, cost_via_z);
                } else if (child->getG() == std::numeric_limits<double>::infinity() || (child->getG() - cost_via_z > epsilon)) {
                    v_open_heap_.add(child, cost_via_z); 
                }
            }
        }



        /*
            Before we expand z, we remove any temporary neighbors that are now outside the shrinking radius.
            We have no worries on cascade death because we already propagated the cost improvment of z to its children
        */ 
        cullNeighbors(z);



        // IDENTIFY POTENTIALLY SUBOPTIMAL NEIGHBORS
        // Iterate through all neighbors 'x' of the expanding node 'z'.
        auto& backward_neighbors = z->backwardNeighbors();
        for (auto it = backward_neighbors.begin(); it != backward_neighbors.end(); ) {
            auto x = it->first;
            auto& edge_info_x_to_z = it->second; 
            ++it; // advance the iterator BEFORE any potential deletion occurs;



            // -------- SKIPPING USELESS NODES (time-cone reachability prune) ----------------
            // x lies beyond the robot's reachable cone -> it can never be on the robot's
            // path and its optimal parent (lower tau) is also beyond reach, so wiring it
            // here helps nobody relevant. EXACT prune. See time_cone_prune.hpp.
            if (TIME_CONE_PRUNED(x, T_robot)) {
                // col_stats_.plan_ignored++;  // ← COUNT PRUNE (x)

                // // ← ADD THIS
                // cone_pruned_this_iteration++;
                // total_cone_pruned++;
                // // ← END ADD

                continue;
            }
            // ------------------------------------------------

            // // ← ADD THIS
            // neighbors_this_iteration++;
            // total_neighbors_considered++;
            // // ← END ADD

            const Trajectory& traj_xz = *(edge_info_x_to_z.cached_trajectory);
            if (!traj_xz.is_valid) {
                continue;
            }
            double cost_via_z = z->getG() + edge_info_x_to_z.distance;
            /*
                This condition is the core of FMTX. It serves two purposes:
                If x has not been connected yet (cost is INF), this is always true, triggering its initial connection.
                If x is already connected, this condition acts as a "witness" or an "Alarm bell" that a better path might exist for x.
                It proves x's current cost is suboptimal and justifies the more expensive search that follows.
                This adds implicit rewiring to FMTX. There is no explicit rewiring here (like RRTX). It repairs the graph
                as wavefront expands.  
            */

            if (x->getLMC() > cost_via_z) {


#if DEBUG
                // Check if we've already updated this node in this plan() cycle
                if (dbg_metrics.costUpdated.find(x) != dbg_metrics.costUpdated.end()) {
                    dbg_metrics.revisits++;
                }
#endif

                /*
                    Since x might be not visited (never popped), its neighbors 
                    might contain stale temporary edges from an older, larger radius.
                    We must clean them now to ensure we pick a valid parent.
                    Safe to call now! If this deletes 'x' from z's map, 
                    our iterator 'it' has already safely moved past it. --> so for this reason we couldnt use the range based for loop!
                    because cullNeihgbor(x) could've deleted the incoming node from x to z from the z's perpective, the one that we are already using in this region of code!
                */
                cullNeighbors(x);
                /*
                    Optional step: Did 'z' survive the cull?
                    In this step we can decide to ignore this witness or not!
                    This step is a proof that z it self cannot help x but x might find another y
                    and we can eagerly connect x!
                    but mind that, that y is in queue and will later be popped from the queue
                    we can trust the queue to pop y and connect x to it or we can connect eagerly in this step
                    the difference is its best to wait instead of eagerly connect because number of collision checks 
                    will be little bit higher in eager connection specially in high number of batches
                    who know what happens if we delay it a little! some new node may come and change the picture or something else!
                    So its best to wait (skip for now) and no 'panic rewires'. we wait for the priority to pop the best parent because 
                    in the process that leads to that pop alot could happen.
                */
                if (x->forwardNeighbors().count(z) == 0) {
                    continue; // Skip entirely! but its optional
                }



                // 'x' is suboptimal. We now search for its true best parent among ALL its neighbors that are currently in the open set.
                // double min_cost_for_x = std::numeric_limits<double>::infinity();
                double min_cost_for_x = x->getLMC();
                FMTNode* best_parent_for_x = nullptr;
                std::shared_ptr<Trajectory> best_traj_for_x;

                                
                //////////////////////////////BELLMAN/////////////////////////////////

                // // Faster version of bellman
                for (auto& [y, edge_info_xy] : x->forwardNeighbors()) {
                    if (!y->in_queue_) continue;                              // only open parents
                    // NOTE: cached per-epoch failures are intentionally NOT skipped here. Removing a
                    // blocked edge from the argmin would make x fall through to its next-cheapest
                    // parent -- "looking for other connections in the neighborhood" -- which FMT*
                    // explicitly forbids (Janson et al., Sec 3.1: a blocked locally-optimal connection
                    // means the sample is "simply skipped and left for later"). The failure cache is
                    // consumed at the collision-check site below instead: a cached-blocked argmin is
                    // treated as a failed check (no recompute) and x DEFERS -- identical decisions to
                    // the no-cache path, just without the redundant re-checks.
                    const double cost_via_y = y->getLMC() + edge_info_xy.distance;  // distance==inf ⇒ excluded automatically
                    if (cost_via_y < min_cost_for_x) {
                        min_cost_for_x = cost_via_y;
                        best_parent_for_x = y;
                    }
                }
                if (best_parent_for_x)
                    best_traj_for_x = x->forwardNeighbors().at(best_parent_for_x).cached_trajectory;  // ONE shared_ptr copy, for the winner only

                /////////////////////////////////////////////////////////////////////////////



                if (best_parent_for_x != nullptr ) { 

                    bool obstacle_free = true;
                    if (best_parent_for_x != x->getParent()) {

                        // --- 1. CACHE BYPASS (static wall + per-epoch dynamic failure) ---
                        // Fetch the specific edge struct for this connection
                        auto& edge_to_check = x->forwardNeighbors().at(best_parent_for_x);
                        bool cached_block = edge_to_check.permanently_blocked;   // static wall: blocked forever
#if USE_CACHE_FAILURE
                        // This exact edge already failed a collision check THIS obstacle cycle. Treat it
                        // as blocked WITHOUT recomputing (the repair-time saving), but do NOT advance to
                        // another parent: x defers, exactly as the no-cache path would. Auto-expires when
                        // updateObstacles bumps plan_epoch_.
                        cached_block = cached_block || (edge_to_check.last_eval_epoch == plan_epoch_);
#endif
                        if (cached_block) {
                            // We already know this edge is blocked. Act as if the collision check failed,
                            // and skip adopting this parent (x defers, keeps its current parent).
                            obstacle_free = false;
                        }
                        else {
                            // col_stats_.plan_checked++;  // ← COUNT CHECK
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
                                collision_checked_++;
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


                        // THE EPSILON QUEUE BOUNCER
                        if (x->in_queue_) {
                            // If it is already scheduled to be processed, give it the best cost
                            v_open_heap_.update(x, priorityCost);
                        } 
                        else if (x->getG() == std::numeric_limits<double>::infinity() || (x->getG() - min_cost_for_x > epsilon)) {
                            // If it is NOT in the queue, ONLY wake it up if the improvement is > epsilon 
                            // or if it is a brand new node or an orphan node that must expand the wavefront
                            v_open_heap_.add(x, priorityCost); 
                        }
                    }
                    else { // I'm not gonna put this else in the pseudo code because failure caching is implmentation dependent. Maybe later i should utilize the edgeInfo struct!
#if USE_CACHE_FAILURE
                        // Cache the failure for THIS obstacle cycle: stamp the edge with the current
                        // epoch. The parent search above MAY re-select this edge as the argmin, but the
                        // collision-check site will treat it as blocked WITHOUT recomputing (x defers),
                        // so we never re-run the geometry on a known-blocked edge until the obstacle set
                        // changes. Auto-expires when updateObstacles bumps plan_epoch_ (no restoration
                        // loop needed, unlike the old distance=inf hack).
                        x->forwardNeighbors().at(best_parent_for_x).last_eval_epoch = plan_epoch_;
#endif
                    }
                }
            }
        }
        v_open_heap_.pop();

      

        // visualizeTree();
        // std::this_thread::sleep_for(std::chrono::milliseconds(5));


    } 
    

#if DEBUG 
    runFMT(dbg_metrics); 
    printDebugSummary(dbg_metrics);
    runCollisionForensics();
    runCostForensics();
    runGlobalCostForensics();
    runTreePropagationForensics();
#endif


}

std::vector<Eigen::VectorXd> KinodynamicANYFMTX::getPathPositions() const
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



void KinodynamicANYFMTX::setStart(const Eigen::VectorXd& start) {
    root_state_index_ = statespace_->getNumStates();
    auto node = std::make_unique<FMTNode>(statespace_->addState(start),tree_.size());
    node->setLMC(0);
    node->setTimeToGoal(0);
    v_open_heap_.add(node.get(),0);
    std::cout << "KinodynamicANYFMTX: Start node created on Index: " << root_state_index_ << "\n";
    tree_.push_back(std::move(node));
}
void KinodynamicANYFMTX::setGoal(const Eigen::VectorXd& goal) {
    robot_state_index_ = statespace_->getNumStates();
    auto node = std::make_unique<FMTNode>(statespace_->addState(goal),tree_.size());
    node->in_unvisited_ = true;
    node->setTimeToGoal(std::numeric_limits<double>::infinity());
    robot_node_ = node.get();
    std::cout << "KinodynamicANYFMTX: Goal node created on Index: " << robot_state_index_ << "\n";
    tree_.push_back(std::move(node));
}

void KinodynamicANYFMTX::visualizeTreeGradient() {
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
        std::vector<Eigen::VectorXd> anchor_pt = { robot_node_->getStateValue().head<2>() };
        visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");
    } 
    visualization_->visualizeTreeGradient(edges, edge_costs, "map");
}



void KinodynamicANYFMTX::visualizeFMTtree() {
    if (!visualization_ || !fmt_shadow_valid_) return;

    const int N = static_cast<int>(tree_.size());

    // Full FMT* tree (green)
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
    visualization_->visualizeEdges(
        fmt_tree_edges, "map",
        std::array<float,3>{0.0f, 1.0f, 0.0f}, 1.0f, 0.15f,
        "fmt_full_tree", 300, true, 0.5
    );

    // Disagreement edges
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

    // Publish magenta edges (FMTX better)
    visualization_->visualizeEdges(
        fmtx_better_edges_magenta, "map",
        std::array<float,3>{1.0f, 0.0f, 1.0f}, 1.0f, 0.25f,
        "fmtx_better_edges", 303, true, 0.5
    );


    if (!fmt_better_edges_red.empty()) {
        visualization_->triggerPublish();
        FMTX_ERROR("[visualizeFMTtree] FMT* strictly better detected — sleeping 5 seconds for RViz inspection.");
        // std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}



// straight line
void KinodynamicANYFMTX::visualizeTree() {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    if (!tree_.empty()) {
        edges.reserve(tree_.size());
    }
    
    std::vector<Eigen::VectorXd> tree_nodes;
    tree_nodes.reserve(tree_.size());
    
    for (const auto& node_ptr : tree_) {
        FMTNode* child_node = node_ptr.get();
        FMTNode* parent_node = child_node->getParent();

        tree_nodes.push_back(node_ptr->getStateValue().head(2)); // TODO: For min snap it needs to be 3!!! I need spatial dim variable!

        if (parent_node) {
            edges.emplace_back(parent_node->getStateValue().head(2), child_node->getStateValue().head(2));
        }
    }
    // visualization_->visualizeNodes(tree_nodes, "map", 
    //                         std::vector<float>{0.0f, 0.0f, 1.0f},  // Green color
    //                         "tree_nodes");
    

    // if (robot_node_) {
    //     std::vector<Eigen::VectorXd> anchor_pt = { robot_node_->getStateValue().head<2>() };
    //     visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");
    // } 
    


    visualization_->visualizeEdges(edges, "map");

    #if DEBUG
        visualizeFMTtree();
    #endif
}



// Curvy!
void KinodynamicANYFMTX::visualizeTreeReal() {
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
        std::vector<Eigen::VectorXd> anchor_pt = { robot_node_->getStateValue().head<2>() };
        visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");
    } 
    visualization_->visualizeEdges(edges, "map");
}

void KinodynamicANYFMTX::visualizePathGradient(const std::vector<Eigen::VectorXd>& path_waypoints) {
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

void KinodynamicANYFMTX::visualizePath(const std::vector<Eigen::VectorXd>& path_waypoints) {
    if (path_waypoints.size() < 2) {
        return;
    }

    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    for (size_t i = 0; i < path_waypoints.size() - 1; ++i) {
        const Eigen::VectorXd& start_point = path_waypoints[i];
        const Eigen::VectorXd& end_point = path_waypoints[i+1];
        edges.emplace_back(start_point.head(2), end_point.head(2));
    }

    if (visualization_) {
        visualization_->visualizeEdges(edges, "map", "0.13,0.59,0.15", "executable_path");
    }
}

void KinodynamicANYFMTX::visualizeSearchArea() {
    if (!visualization_) return;

    // RATE LIMITING (30 FPS MAX)
    static auto last_vis_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_vis_time).count() < 33) {
        return; 
    }
    last_vis_time = now;

    if (robot_node_ == nullptr || std::isinf(robot_node_->getLMC())) {
        return; 
    }

    double current_robot_cost = robot_node_->getLMC() + bridge_cost_;
    
    // Check if we need to calibrate the color gradient on this frame
    bool is_first_run = (global_max_cost_ < 0.0);
    double max_tree_cost = current_robot_cost;

    std::vector<Eigen::VectorXd> surface_points;
    std::vector<double> surface_costs;
    surface_points.reserve(tree_.size());
    surface_costs.reserve(tree_.size());

    for (const auto& node_ptr : tree_) {
        if (node_ptr) {
            double cost = node_ptr->getLMC();
            
            if (!std::isinf(cost)) {
                
                // Find the absolute highest cost in the tree (only on the first run)
                if (is_first_run && cost > max_tree_cost) {
                    max_tree_cost = cost;
                }
                
                // Collect the points for rendering
                if (cost <= current_robot_cost) {
                    surface_points.push_back(node_ptr->getStateValue());
                    surface_costs.push_back(cost);
                }
            }
        }
    }

    // Lock in the globally calibrated max cost so the colors stay stable
    if (is_first_run) {
        global_max_cost_ = max_tree_cost;
    }

    visualization_->visualizeContinuousMesh(
        surface_points, surface_costs, global_max_cost_, getNeighborhoodRadius(), 
        lower_bounds_, upper_bounds_, "map"
    );
}

void KinodynamicANYFMTX::dumpTreeToCSV(const std::string& filename) const {
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

// The Manager
void KinodynamicANYFMTX::updateObstacles(const ObstacleVector& turned_obstacles) {
    // col_stats_.reset();  // ← RESET
    // just_updated_ = true;

    if (turned_obstacles.empty()) return;

#if USE_CACHE_FAILURE
    // New obstacle configuration => every per-edge failure soft-block from the previous cycle
    // is now stale. Bump the epoch to invalidate them all at once (O(1), no restoration loop).
    ++plan_epoch_;
#endif

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
        
        // REMOVE OLD TUBE (Wake up neighbors in the freed region)
        // We do this BEFORE updating the object so we use the OLD path.
        if (!stored_ob.predicted_path.empty()) {
            removeObstacle(stored_ob); 
        }


        // FOR REMOVED STATIC OBS
        // PURGE: visible obstacle the robot reached. Free edges, erase, never re-add. --> Remove static obstalce!
        /*
            Disclaimer! --> The permanently_blocked variable in edgeinfo needs to be reset too or else the old edges that were permanently blocked are
            still blocked even after removing this static obstacle!
            The new sampled nodes and edges are fine though after this deletion! --> So we might not even need to take care of anything!
            Either use permanaenly blocked for absolutely fixed static obstalces (and not uncertain ones!) or some how restore the permanently blocked variable
            
            no worries overall because for benchmark i dont even use static obstalces and this is just making the planner complete!
        */
        if (incoming_ob.is_removed) {
            previous_obstacles_.erase(incoming_ob.name);
            continue;
        }
        //////////////////


        // UPDATE STATE
        stored_ob = incoming_ob; 

        // GENERATE NEW DENSE TUBE
        // stored_ob.predicted_path = obs_checker_->generatePrediction(stored_ob, T_robot);

        // ADD NEW TUBE (Invalidate nodes in the blocked region)
        addNewObstacle(stored_ob);
    }
}

void KinodynamicANYFMTX::addNewObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    // Calculate Search Radius
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
        search_radius = obs_r + ob.inflation + delta;
    } else {
        // Kinodynamic mode: Add gap coverage for the "tube" samples
        double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
        search_radius = obs_r + ob.inflation + delta + gap_coverage_inflation;
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
            // Protect ALL Time Pillars and the main root instantly!
            if (time_pillar_indices_.find(static_cast<int>(idx)) != time_pillar_indices_.end()) {
                continue;
            }
            // (Time-cone prune MOVED DOWN to the filter loop below, so it sits at FMTX's
            //  collision-check site and visually parallels RRTX addNewObstacle 2630.
            //  Behavior-identical: orphan_indices is rebuilt from the filtered set, so
            //  collecting dead nodes here and pruning them there is the same.)
            orphan_indices.insert(static_cast<int>(idx));
        }
    }
    // if (kd_dim == 4)
    //     search_radius += M_PI;


    /*
        Optional Filter
        Filter Orphan Indices using isTrajectorySafeAgainstSingleObstacle
        we keep the tree edges that are not in collision. This procedure
        doesn't violate the order of complexity of the collision check inherited from FMT*
        Its just good filter to not invalidate blindly
        Another Important Side Effect of this filter is it reduces the decpetion happens in bellman for loop (suboptimal connection)
        because nodes within rn distance to the obstalce are prone to connect to suboptimal parent in finite sampling case
        it actually changes the behaviour of the heap poping and bellman update wrt fmt* not entiely to the better because sometime fmt* gets lucky
        but in AO they both should reach the same answer but all in all we preserve the correct parent-child relationship
        many mini wavefront propagation VS the big wavefront of FMT*
    */

    std::vector<int> filtered_orphan_indices;
    for (int idx : orphan_indices) {
        FMTNode* node = tree_[idx].get();
        // Time-cone prune (moved here from the kd-tree collection loop so it sits at FMTX's
        // collision-check site, visually parallel to RRTX addNewObstacle 2630): a node beyond
        // the robot's reachable cone can never be on the robot's path, so skip its parent-edge
        // collision check and all downstream orphan bookkeeping. EXACT prune. See time_cone_prune.hpp.
        if (TIME_CONE_PRUNED(node, T_robot)) continue;
#if USE_THREAT_SET_STRATEGY
        // Mark this node as being under threat, ensuring no duplicates
        if (std::find(node->threats.begin(), node->threats.end(), &ob) == node->threats.end()) {
            node->threats.push_back(&ob);
        }

#endif
        // Skip root or nodes with no parent
        if (node->getParent() == nullptr) continue; 
        // col_stats_.add_checked++;  // ← COUNT CHECK
        last_replan_metrics_.obstacle_checks++;
        collision_checked_++;
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
            // if (TIME_CONE_PRUNED(child, T_robot)) continue;
            if (orphan_indices.insert(child->getIndex()).second) {
                propagation_queue.push(child);
            }
        }
    }


    

    // Invalidate Nodes & Queue Boundary Parents
    std::unordered_set<FMTNode*> boundary_nodes_to_requeue;
    for (int node_index : orphan_indices) {
        auto node = tree_[node_index].get();
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

        // After setting LMC=INF and parent=nullptr for orphans
        if (node->getIndex() == root_state_index_) {
            std::cout << "[WARN] ROOT NODE BECAME ORPHAN! This should never happen.\n";
        }


        // Find Boundary (Valid Parents)
        // We look at neighbors. If a neighbor is NOT an orphan, it's a valid candidate parent.
        
        auto check_boundary = [&](const auto& neighbors) {
            for (const auto& [neighbor_ptr, edge_data] : neighbors) {
                if (orphan_indices.find(neighbor_ptr->getIndex()) == orphan_indices.end()) {
                    boundary_nodes_to_requeue.insert(neighbor_ptr);
                }
            }
        };

        check_boundary(node->forwardNeighbors());
    }

    // Add Boundary to Open Heap
    for (FMTNode* valid_node : boundary_nodes_to_requeue) {
        if (!valid_node->in_queue_ && valid_node->getLMC() != std::numeric_limits<double>::infinity()) {
            v_open_heap_.add(valid_node, valid_node->getLMC());
        }
    }
}



// Wake Up Neighbors
void KinodynamicANYFMTX::removeObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);

    double search_radius;
    
    if (is_geometric_mode_) {
        // In geometric mode, we just need to cover the obstacle size + robot size + delta
        search_radius = obs_r + ob.inflation + delta;
    } else {
        // Kinodynamic mode: Add gap coverage for the "tube" samples
        double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
        search_radius = obs_r + ob.inflation + delta + gap_coverage_inflation;
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


        // Time-cone prune: re-queuing a node beyond the robot's reachable cone only feeds
        // the heap a node that plan() will immediately pop-and-discard. Skip it. EXACT prune.
        if (TIME_CONE_PRUNED(node, T_robot)) continue;

        // (Removed) the per-node "reset temporarily blocked edges" neighbor scan: nothing writes
        // edge.distance = inf anymore. Parent-search failures are soft-blocked via
        // EdgeInfo::last_eval_epoch (USE_CACHE_FAILURE) and invalidated wholesale by the epoch
        // bump in updateObstacles; addNewObstacle orphans NODES, not edges. So there is nothing
        // to restore here.
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
        /*
            Do we need to put the neihgbors of these nodes in the queue too? theoretically no! because the kd tree search radius is large enough
            So finite lmc can be found when an obstalce disappears
            But what if no node in the Rfree has finite lmc??? it means it was blocked by addNewObstacle at some point and some vopen node is there to back
            it up ofcourse (at this case even we put the neighbors of these node to the queue in removeObstalce here then they wouldnt connect!)
            At every removeobstalce we put finite lmc node into the queue to give this freed region a chance if there is no vopen node in Rfree then 
            there is no chance! even if we put their neighbor in the queue it wouldnt help them! some other removeObstalce is responsible for those helpless nodes!
        */
    }
}




// void KinodynamicANYFMTX::setRobotState(const Eigen::VectorXd& robot_state) {
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
//             for (const auto& [name, ob] : previous_obstacles_) {
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
//             for (const auto& [name, ob] : previous_obstacles_) {
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



//     bool cached_bridge_safe = false;                // will be set inside the cached‑bridge block
//     std::string first_unsafe_obstacle_name_;        // will be set inside the cached‑bridge block
//     // ----- TRAP DIAGNOSTIC LAMBDA -----
//     auto trap_reason = [&]() -> std::string {
//         std::ostringstream oss;
//         oss << "TRAP DIAGNOSTIC: ";
//         if (best_candidate_node) {
//             oss << "candidate exists (cost " << best_candidate_cost
//                 << ") but not better than current (" << cost_of_current_path
//                 << ") or hysteresis (factor " << hysteresis_factor << "). ";
//         } else {
//             oss << "no candidate with safe bridge + finite LMC found within "
//                 << (current_search_radius / radius_multiplier) << " m radius. "
//                 << "Searched " << tested_indices.size() << " nodes. ";
//         }
//         if (robot_node_) {
//             oss << "Current anchor LMC=" << robot_node_->getLMC() << ", ";
//         } else {
//             oss << "No current anchor. ";
//         }
//         oss << "Fresh steer safe=" << safe << ", bridge valid=" << bridge.is_valid;
//         if (!bridge.is_valid && robot_node_) {
//             oss << " (steer failed from " << robot_continuous_state_.head<2>().transpose()
//                 << " to " << robot_node_->getStateValue().head<2>().transpose() << ")";
//         }
//         if (current_bridge_trajectory_.is_valid) {
//             // We’ll set cached_bridge_safe before entering the trap branches
//             oss << ", cached bridge exists, re‑stamped and checked="
//                 << (cached_bridge_safe ? "safe" : "unsafe");
//             if (!cached_bridge_safe) {
//                 oss << " (collision with " << first_unsafe_obstacle_name_ << ")";
//             }
//         } else {
//             oss << ", no cached bridge";
//         }
//         oss << ", recovery=" << (USE_RECOVERY ? "enabled" : "disabled");
//         return oss.str();
//     };
//     // ----- END LAMBDA -----



// // // After choosing best_candidate_node, verify every edge on its path to goal
// // if (best_candidate_node) {
// //     FMTNode* cur = best_candidate_node;
// //     // Stop when we reach a pillar (LMC == 0). The main root and all time pillars have LMC 0.
// //     while (cur && cur->getLMC() > 0) {
// //         FMTNode* parent = cur->getParent();
// //         if (!parent) break;
// //         auto it = cur->forwardNeighbors().find(parent);
// //         if (it == cur->forwardNeighbors().end()) break;
// //         bool safe = true;
// //         for (const auto& [name, ob] : previous_obstacles_) {
// //             if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*it->second.cached_trajectory, ob)) {
// //                 safe = false;
// //                 std::cerr << "[FMTX] Anchor " << best_candidate_node->getIndex()
// //                           << " has blocked edge " << cur->getIndex() << "->" << parent->getIndex()
// //                           << " against " << ob.name << " – REJECTING" << std::endl;
// //                 break;
// //             }
// //         }
// //         if (!safe) {
// //             best_candidate_node = nullptr;   // force next candidate
// //             break;
// //         }
// //         cur = parent;
// //     }
// // }
    

//     // ASSIGNMENT PRIORITY
//     // Better connected anchor
//     if (best_candidate_node &&
//         best_candidate_cost < cost_of_current_path * hysteresis_factor) {

//         robot_node_ = best_candidate_node;
//         robot_current_time_to_goal_ =
//             best_candidate_bridge.time_duration + best_candidate_node->getTimeToGoal();
//         last_replan_metrics_.path_cost = best_candidate_cost;
//         current_bridge_trajectory_ = best_candidate_bridge;
//         bridge_cost_ = best_candidate_bridge.cost;
//     }

//     // Keep current anchor with fresh bridge
//     else if (safe && robot_node_ &&
//              cost_of_current_path != std::numeric_limits<double>::infinity() &&
//              bridge.is_valid) {

//         robot_current_time_to_goal_ =
//             bridge.time_duration + robot_node_->getTimeToGoal();
//         last_replan_metrics_.path_cost = cost_of_current_path;
//         current_bridge_trajectory_ = bridge;
//         bridge_cost_ = bridge.cost;
//     }

//     // Recovery: go to nearest safe tree node
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

//     // // Keep current anchor, reuse previous cached bridge
//     // // This is the near-root / numerical-steer-failure case.
//     // else if (robot_node_ &&
//     //          current_bridge_trajectory_.is_valid &&
//     //          !current_bridge_trajectory_.path_points.empty()) {

//     //     robot_current_time_to_goal_ =
//     //         current_bridge_trajectory_.time_duration + robot_node_->getTimeToGoal();

//     //     // Keep finite so we do NOT enter trapped logic.
//     //     const double anchor_tail_cost =
//     //         (robot_node_->getLMC() != std::numeric_limits<double>::infinity())
//     //             ? robot_node_->getLMC()
//     //             : 0.0;

//     //     cost_of_current_path = current_bridge_trajectory_.cost + anchor_tail_cost;
//     //     last_replan_metrics_.path_cost = cost_of_current_path;
//     //     bridge_cost_ = current_bridge_trajectory_.cost;

//     //     FMTX_WARN("[Set Robot STate] Fresh steer failed near anchor/root. Reusing cached bridge.");
//     // }


//     else if (robot_node_ &&
//          robot_node_->getLMC() != std::numeric_limits<double>::infinity() &&
//          current_bridge_trajectory_.is_valid &&
//          !current_bridge_trajectory_.path_points.empty()) {

//         // 1. Build a re‑stamped version of the cached bridge
//         const auto& old_pts = current_bridge_trajectory_.path_points;
//         std::vector<Eigen::VectorXd> new_pts = old_pts;  // copy spatial points

//         // Original time step (assuming uniform spacing)
//         double dt_old = (old_pts.front()(2) - old_pts.back()(2)) / (old_pts.size() - 1);

//         // New time values: current robot_sim_time down to robot_sim_time - duration
//         double new_start = robot_sim_time;                // <-- use robot_sim_time
//         for (size_t i = 0; i < new_pts.size(); ++i) {
//             new_pts[i](2) = new_start - i * dt_old;
//         }

//         // Create a temporary trajectory for collision checking
//         Trajectory re_stamped_bridge = current_bridge_trajectory_;
//         re_stamped_bridge.path_points = new_pts;

//         // 2. Re‑verify against ALL current obstacles
//         // bool cached_bridge_safe = true;
//         // std::string first_unsafe_obstacle_name_;   // local, but we'll copy to the member for logging later
//         cached_bridge_safe = true;                 // reuse the outer variable
//         first_unsafe_obstacle_name_.clear();       // reset to empty

//         for (const auto& [name, ob] : previous_obstacles_) {
//             last_replan_metrics_.obstacle_checks++;
//             if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(
//                     re_stamped_bridge, ob)) {
//                 cached_bridge_safe = false;
//                 first_unsafe_obstacle_name_ = ob.name;   // store the name
//                 break;
//             }
//         }

//         if (!cached_bridge_safe) {
//             // The old bridge is now unsafe – truly trapped.
//             robot_node_ = nullptr;
//             robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
//             bridge_cost_ = std::numeric_limits<double>::infinity();
//             current_bridge_trajectory_ = Trajectory();
//             last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
//             FMTX_WARN("[Set Robot State] " << trap_reason());


//             FMTX_WARN("[Set Robot State] Cached bridge became unsafe. TRULY TRAPPED.");
//         } else {
//             // The cached bridge is still safe – reuse it.
//             robot_current_time_to_goal_ =
//                 current_bridge_trajectory_.time_duration + robot_node_->getTimeToGoal();
//             const double anchor_tail_cost =
//                 (robot_node_->getLMC() != std::numeric_limits<double>::infinity())
//                     ? robot_node_->getLMC()
//                     : 0.0;
//             // cost_of_current_path = current_bridge_trajectory_.cost + anchor_tail_cost;
//             // last_replan_metrics_.path_cost = cost_of_current_path;
//             // bridge_cost_ = current_bridge_trajectory_.cost;

//             current_bridge_trajectory_ = re_stamped_bridge;   // <-- important
//             bridge_cost_ = re_stamped_bridge.cost;
//             robot_current_time_to_goal_ = re_stamped_bridge.time_duration + robot_node_->getTimeToGoal();
//             cost_of_current_path = re_stamped_bridge.cost + anchor_tail_cost;
//             last_replan_metrics_.path_cost = cost_of_current_path;

//             FMTX_WARN("[Set Robot State] Fresh steer failed near anchor/root. Reusing cached bridge (verified safe).");
//         }
//     }


//     // Truly trapped
//     else {
//         robot_node_ = nullptr;
//         robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
//         bridge_cost_ = std::numeric_limits<double>::infinity();
//         current_bridge_trajectory_ = Trajectory();
//         last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();

//         FMTX_WARN("[Set Robot STate] LOST SAFE ANCHOR. TRULY TRAPPED.");
//     }



//     // // <------ ADD LOGGING HERE
//     // if (robot_node_ && current_bridge_trajectory_.is_valid) {
//     //     RCLCPP_INFO(rclcpp::get_logger("SetRobotState"),
//     //         "Chosen anchor: node %d at (%.2f, %.2f)  bridge length %.2f  bridge duration %.2f",
//     //         robot_node_->getIndex(),
//     //         robot_node_->getStateValue()(0), robot_node_->getStateValue()(1),
//     //         current_bridge_trajectory_.cost,
//     //         current_bridge_trajectory_.time_duration);

//     //     for (const auto& [name, ob] : previous_obstacles_) {
//     //         bool safe = obs_checker_->isTrajectorySafeAgainstSingleObstacle(
//     //                         current_bridge_trajectory_, ob);
//     //         if (!safe) {
//     //             RCLCPP_ERROR(rclcpp::get_logger("SetRobotState"),
//     //                 "   *** Bridge collides with %s!  Obs pos (%.2f,%.2f) radius %.2f",
//     //                 ob.name.c_str(), ob.position.x(), ob.position.y(),
//     //                 ob.dimensions.radius + ob.inflation);
//     //         }
//     //     }
//     // }


// // if (robot_node_ && current_bridge_trajectory_.is_valid && !current_bridge_trajectory_.path_points.empty()) {
// //     double t_start = current_bridge_trajectory_.path_points.front()(2);
// //     double t_end = current_bridge_trajectory_.path_points.back()(2);
// //     double expected_anchor_time = robot_node_->getStateValue()(2);
    
// //     // 1. Check for Time Desync between Bridge and Tree
// //     if (std::abs(t_end - expected_anchor_time) > 1e-4) {
// //         RCLCPP_ERROR(rclcpp::get_logger("SetRobotState_Debug"),
// //             "[TIME DESYNC] Bridge ends at T=%.4f but anchor expects T=%.4f! "
// //             "Difference: %.4f seconds. Tree edges are being validated at the wrong time!",
// //             t_end, expected_anchor_time, t_end - expected_anchor_time);
// //     }

// //     // 2. Track the specific crashing obstacles exactly where they failed in your logs
// //     for (const auto& [name, ob] : previous_obstacles_) {
// //         if (name == "moving_cylinder_10" || name == "moving_box_2") {
// //             bool safe = obs_checker_->isTrajectorySafeAgainstSingleObstacle(current_bridge_trajectory_, ob);
            
// //             // Ask the obstacle tube where it thinks it is at the start and end of this bridge
// //             // (Replace 'evaluateTubeAt' with whatever method your tube uses to get spatial position at time T)
// //             // auto pos_start = ob.evaluateTubeAt(t_start);
// //             // auto pos_end = ob.evaluateTubeAt(t_end);
            
// //             RCLCPP_INFO(rclcpp::get_logger("SetRobotState_Debug"),
// //                 "Evaluating [%s] | Bridge T: [%.2f to %.2f] | Deemed Safe? %s",
// //                 name.c_str(), t_start, t_end, safe ? "YES" : "NO");
// //         }
// //     }
// // }


// // if (robot_node_ && current_bridge_trajectory_.is_valid && !current_bridge_trajectory_.path_points.empty()) {
// //     double t_start = current_bridge_trajectory_.path_points.front()(2);
// //     double t_end = current_bridge_trajectory_.path_points.back()(2);

// //     // 1. TUBE CHECK (What the planner currently relies on)
// //     for (const auto& [name, ob] : previous_obstacles_) {
// //         if (name == "moving_cylinder_10" || name == "moving_box_2") {
// //             bool safe_tube = obs_checker_->isTrajectorySafeAgainstSingleObstacle(current_bridge_trajectory_, ob);
// //             RCLCPP_INFO(rclcpp::get_logger("Diagnostic"),
// //                 "TUBE CHECK [%s] | T:[%.2f to %.2f] | Safe? %s",
// //                 name.c_str(), t_start, t_end, safe_tube ? "YES" : "NO");
// //         }
// //     }

// //     // 2. REAL CHECK (Using the instantaneous simulator obstacle states)
// //     const auto& current_sim_obstacles = obs_checker_->getObstacles();
// //     for (const auto& ob : current_sim_obstacles) {
// //         if (ob.name == "moving_cylinder_10" || ob.name == "moving_box_2") {
// //             bool safe_real = obs_checker_->isTrajectorySafeAgainstSingleObstacle(current_bridge_trajectory_, ob);
// //             RCLCPP_INFO(rclcpp::get_logger("Diagnostic"),
// //                 "REAL CHECK [%s] | T:[%.2f to %.2f] | Safe? %s",
// //                 ob.name.c_str(), t_start, t_end, safe_real ? "YES" : "NO");
// //         }
// //     }
// // }


// // auto evalTubeAt = [](const Obstacle& ob, double Tq, Eigen::Vector2d& pos_out) -> bool {
// //     const auto& pts = ob.predicted_path;
// //     if (pts.size() < 2) return false;

// //     for (size_t i = 0; i + 1 < pts.size(); ++i) {
// //         double t1 = pts[i](2);
// //         double t2 = pts[i + 1](2);

// //         if ((t1 >= Tq && Tq >= t2) || (t2 >= Tq && Tq >= t1)) {
// //             double alpha = (Tq - t1) / (t2 - t1);
// //             pos_out = pts[i].head<2>() + alpha * (pts[i + 1].head<2>() - pts[i].head<2>());
// //             return true;
// //         }
// //     }
// //     return false;
// // };
// // auto it = previous_obstacles_.find("moving_cylinder_10");
// // if (it != previous_obstacles_.end()) {
// //     Eigen::Vector2d tube_pos;
// //     if (evalTubeAt(it->second, robot_sim_time, tube_pos)) {
// //         const auto& live_obs = obs_checker_->getObstacles();
// //         for (const auto& ob : live_obs) {
// //             if (ob.name == "moving_cylinder_10") {
// //                 RCLCPP_ERROR(rclcpp::get_logger("TubeCompare"),
// //                     "T=%.4f | LIVE=(%.3f, %.3f) | TUBE@T=(%.3f, %.3f) | DY=%.3f",
// //                     robot_sim_time,
// //                     ob.position.x(), ob.position.y(),
// //                     tube_pos.x(), tube_pos.y(),
// //                     ob.position.y() - tube_pos.y());
// //             }
// //         }
// //     }
// // }


// }



// void KinodynamicANYFMTX::setRobotState(const Eigen::VectorXd& robot_state) {
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

//     // 1. PERFORM RADIUS SEARCH FOR A NEW ANCHOR
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
//             for (const auto& [name, ob] : previous_obstacles_) {
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

//     // 2. SIMPLE ASSIGNMENT LOGIC (We only get here if we MUST switch anchors)
//     if (best_candidate_node) {
//         robot_node_ = best_candidate_node;
//         robot_current_time_to_goal_ = best_candidate_bridge.time_duration + best_candidate_node->getTimeToGoal();
//         last_replan_metrics_.path_cost = best_candidate_cost;
//         current_bridge_trajectory_ = best_candidate_bridge;
//         bridge_cost_ = best_candidate_bridge.cost;

//          // =========================================================================
//         // STRICT FORENSIC TRACE: Expose upstream planner failures
//         // =========================================================================
//         FMTX_WARN("======================================================");
//         FMTX_WARN("[setRobotState] Selected Anchor: " << robot_node_->getIndex() << " | LMC: " << robot_node_->getLMC());
        
//         FMTNode* curr = robot_node_;
//         int steps = 0;
        
//         while (curr != nullptr) {
//             // Stop if we hit the goal or a time pillar
//             if (curr->getIndex() == 1 || time_pillar_indices_.count(curr->getIndex()) > 0) {
//                 FMTX_WARN("  -> [OK] Traced safely to Root/Pillar " << curr->getIndex() << " in " << steps << " steps.");
//                 break;
//             }
            
//             FMTNode* parent = curr->getParent();
            
//             if (parent == nullptr) {
//                 FMTX_WARN("  -> [ERROR] Broken chain! Node " << curr->getIndex() << " has a NULL parent but is not a root/pillar!");
//                 break;
//             }

//             // Test the cached trajectory going to the parent against CURRENT obstacles
//             auto cached_traj = curr->getParentTrajectory();
//             if (cached_traj && cached_traj->is_valid) {
//                 for (const auto& [name, ob] : previous_obstacles_) {
//                     if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*cached_traj, ob)) {
//                         FMTX_WARN("  -> [SMOKING GUN] Edge " << curr->getIndex() << " -> " << parent->getIndex() << " physically collides with " << name << "!");
//                         FMTX_WARN("     (The planner failed to sever this edge!)");
//                     }
//                 }
//             } else {
//                  FMTX_WARN("  -> [ERROR] Cached trajectory between " << curr->getIndex() << " and " << parent->getIndex() << " is INVALID/EMPTY!");
//             }

//             curr = parent;
//             steps++;
//             if (steps > 1000) { FMTX_WARN("  -> [ERROR] Cycle detected in parent pointers!"); break; }
//         }
//         FMTX_WARN("======================================================");
//         // =========================================================================

//     }
// #if USE_RECOVERY
//     else if (best_fallback_node) {
//         robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
//         bridge_cost_ = best_fallback_cost;
//         current_bridge_trajectory_ = best_fallback_bridge;
//         last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
//         FMTX_WARN("[Set Robot State] No connected anchor found. Falling back to nearest safe tree node.");
//     }
// #endif
//     else {
//         robot_node_ = nullptr;
//         robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
//         bridge_cost_ = std::numeric_limits<double>::infinity();
//         current_bridge_trajectory_ = Trajectory();
//         last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
//         FMTX_WARN("[Set Robot State] LOST SAFE ANCHOR. TRULY TRAPPED.");
//     }
// }




// // Wake Up Neighbors (Conservative version!)
// void KinodynamicANYFMTX::removeObstacle(const Obstacle& ob) {
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



// void KinodynamicANYFMTX::setRobotState(const Eigen::VectorXd& robot_state) {
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



//     bool cached_bridge_safe = false;                // will be set inside the cached‑bridge block
//     std::string first_unsafe_obstacle_name_;        // will be set inside the cached‑bridge block
//     // ----- TRAP DIAGNOSTIC LAMBDA -----
//     auto trap_reason = [&]() -> std::string {
//         std::ostringstream oss;
//         oss << "TRAP DIAGNOSTIC: ";
//         if (best_candidate_node) {
//             oss << "candidate exists (cost " << best_candidate_cost
//                 << ") but not better than current (" << cost_of_current_path
//                 << ") or hysteresis (factor " << hysteresis_factor << "). ";
//         } else {
//             oss << "no candidate with safe bridge + finite LMC found within "
//                 << (current_search_radius / radius_multiplier) << " m radius. "
//                 << "Searched " << tested_indices.size() << " nodes. ";
//         }
//         if (robot_node_) {
//             oss << "Current anchor LMC=" << robot_node_->getLMC() << ", ";
//         } else {
//             oss << "No current anchor. ";
//         }
//         oss << "Fresh steer safe=" << safe << ", bridge valid=" << bridge.is_valid;
//         if (!bridge.is_valid && robot_node_) {
//             oss << " (steer failed from " << robot_continuous_state_.head<2>().transpose()
//                 << " to " << robot_node_->getStateValue().head<2>().transpose() << ")";
//         }
//         if (current_bridge_trajectory_.is_valid) {
//             // We’ll set cached_bridge_safe before entering the trap branches
//             oss << ", cached bridge exists, re‑stamped and checked="
//                 << (cached_bridge_safe ? "safe" : "unsafe");
//             if (!cached_bridge_safe) {
//                 oss << " (collision with " << first_unsafe_obstacle_name_ << ")";
//             }
//         } else {
//             oss << ", no cached bridge";
//         }
//         oss << ", recovery=" << (USE_RECOVERY ? "enabled" : "disabled");
//         return oss.str();
//     };
//     // ----- END LAMBDA -----



// // After choosing best_candidate_node, verify every edge on its path to goal
// if (best_candidate_node) {
//     FMTNode* cur = best_candidate_node;
//     // Stop when we reach a pillar (LMC == 0). The main root and all time pillars have LMC 0.
//     while (cur && cur->getLMC() > 0) {
//         FMTNode* parent = cur->getParent();
//         if (!parent) break;
//         auto it = cur->forwardNeighbors().find(parent);
//         if (it == cur->forwardNeighbors().end()) break;
//         bool safe = true;
//         for (const auto& ob : obs_checker_->getObstacles()) {
//             if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*it->second.cached_trajectory, ob)) {
//                 safe = false;
//                 std::cerr << "[FMTX] Anchor " << best_candidate_node->getIndex()
//                           << " has blocked edge " << cur->getIndex() << "->" << parent->getIndex()
//                           << " against " << ob.name << " – REJECTING" << std::endl;
//                 break;
//             }
//         }
//         if (!safe) {
//             best_candidate_node = nullptr;   // force next candidate
//             break;
//         }
//         cur = parent;
//     }
// }
    

//     // ASSIGNMENT PRIORITY
//     // Better connected anchor
//     if (best_candidate_node &&
//         best_candidate_cost < cost_of_current_path * hysteresis_factor) {

//         robot_node_ = best_candidate_node;
//         robot_current_time_to_goal_ =
//             best_candidate_bridge.time_duration + best_candidate_node->getTimeToGoal();
//         last_replan_metrics_.path_cost = best_candidate_cost;
//         current_bridge_trajectory_ = best_candidate_bridge;
//         bridge_cost_ = best_candidate_bridge.cost;
//     }

//     // Keep current anchor with fresh bridge
//     else if (safe && robot_node_ &&
//              cost_of_current_path != std::numeric_limits<double>::infinity() &&
//              bridge.is_valid) {

//         robot_current_time_to_goal_ =
//             bridge.time_duration + robot_node_->getTimeToGoal();
//         last_replan_metrics_.path_cost = cost_of_current_path;
//         current_bridge_trajectory_ = bridge;
//         bridge_cost_ = bridge.cost;
//     }

//     // Recovery: go to nearest safe tree node
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

//     // // Keep current anchor, reuse previous cached bridge
//     // // This is the near-root / numerical-steer-failure case.
//     // else if (robot_node_ &&
//     //          current_bridge_trajectory_.is_valid &&
//     //          !current_bridge_trajectory_.path_points.empty()) {

//     //     robot_current_time_to_goal_ =
//     //         current_bridge_trajectory_.time_duration + robot_node_->getTimeToGoal();

//     //     // Keep finite so we do NOT enter trapped logic.
//     //     const double anchor_tail_cost =
//     //         (robot_node_->getLMC() != std::numeric_limits<double>::infinity())
//     //             ? robot_node_->getLMC()
//     //             : 0.0;

//     //     cost_of_current_path = current_bridge_trajectory_.cost + anchor_tail_cost;
//     //     last_replan_metrics_.path_cost = cost_of_current_path;
//     //     bridge_cost_ = current_bridge_trajectory_.cost;

//     //     FMTX_WARN("[Set Robot STate] Fresh steer failed near anchor/root. Reusing cached bridge.");
//     // }


//     else if (robot_node_ &&
//          robot_node_->getLMC() != std::numeric_limits<double>::infinity() &&
//          current_bridge_trajectory_.is_valid &&
//          !current_bridge_trajectory_.path_points.empty()) {

//         // 1. Build a re‑stamped version of the cached bridge
//         const auto& old_pts = current_bridge_trajectory_.path_points;
//         std::vector<Eigen::VectorXd> new_pts = old_pts;  // copy spatial points

//         // Original time step (assuming uniform spacing)
//         double dt_old = (old_pts.front()(2) - old_pts.back()(2)) / (old_pts.size() - 1);

//         // New time values: current robot_sim_time down to robot_sim_time - duration
//         double new_start = robot_sim_time;                // <-- use robot_sim_time
//         for (size_t i = 0; i < new_pts.size(); ++i) {
//             new_pts[i](2) = new_start - i * dt_old;
//         }

//         // Create a temporary trajectory for collision checking
//         Trajectory re_stamped_bridge = current_bridge_trajectory_;
//         re_stamped_bridge.path_points = new_pts;

//         // 2. Re‑verify against ALL current obstacles
//         // bool cached_bridge_safe = true;
//         // std::string first_unsafe_obstacle_name_;   // local, but we'll copy to the member for logging later
//         cached_bridge_safe = true;                 // reuse the outer variable
//         first_unsafe_obstacle_name_.clear();       // reset to empty

//         const auto& obstacles = obs_checker_->getObstacles();
//         for (const auto& ob : obstacles) {
//             last_replan_metrics_.obstacle_checks++;
//             if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(
//                     re_stamped_bridge, ob)) {
//                 cached_bridge_safe = false;
//                 first_unsafe_obstacle_name_ = ob.name;   // store the name
//                 break;
//             }
//         }

//         if (!cached_bridge_safe) {
//             // The old bridge is now unsafe – truly trapped.
//             robot_node_ = nullptr;
//             robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
//             bridge_cost_ = std::numeric_limits<double>::infinity();
//             current_bridge_trajectory_ = Trajectory();
//             last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
//             FMTX_WARN("[Set Robot State] " << trap_reason());


//             FMTX_WARN("[Set Robot State] Cached bridge became unsafe. TRULY TRAPPED.");
//         } else {
//             // The cached bridge is still safe – reuse it.
//             robot_current_time_to_goal_ =
//                 current_bridge_trajectory_.time_duration + robot_node_->getTimeToGoal();
//             const double anchor_tail_cost =
//                 (robot_node_->getLMC() != std::numeric_limits<double>::infinity())
//                     ? robot_node_->getLMC()
//                     : 0.0;
//             // cost_of_current_path = current_bridge_trajectory_.cost + anchor_tail_cost;
//             // last_replan_metrics_.path_cost = cost_of_current_path;
//             // bridge_cost_ = current_bridge_trajectory_.cost;

//             current_bridge_trajectory_ = re_stamped_bridge;   // <-- important
//             bridge_cost_ = re_stamped_bridge.cost;
//             robot_current_time_to_goal_ = re_stamped_bridge.time_duration + robot_node_->getTimeToGoal();
//             cost_of_current_path = re_stamped_bridge.cost + anchor_tail_cost;
//             last_replan_metrics_.path_cost = cost_of_current_path;

//             FMTX_WARN("[Set Robot State] Fresh steer failed near anchor/root. Reusing cached bridge (verified safe).");
//         }
//     }


//     // Truly trapped
//     else {
//         robot_node_ = nullptr;
//         robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
//         bridge_cost_ = std::numeric_limits<double>::infinity();
//         current_bridge_trajectory_ = Trajectory();
//         last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();

//         FMTX_WARN("[Set Robot STate] LOST SAFE ANCHOR. TRULY TRAPPED.");
//     }



//     // // <------ ADD LOGGING HERE
//     // if (robot_node_ && current_bridge_trajectory_.is_valid) {
//     //     RCLCPP_INFO(rclcpp::get_logger("SetRobotState"),
//     //         "Chosen anchor: node %d at (%.2f, %.2f)  bridge length %.2f  bridge duration %.2f",
//     //         robot_node_->getIndex(),
//     //         robot_node_->getStateValue()(0), robot_node_->getStateValue()(1),
//     //         current_bridge_trajectory_.cost,
//     //         current_bridge_trajectory_.time_duration);

//     //     const auto& obstacles = obs_checker_->getObstacles();
//     //     for (const auto& ob : obstacles) {
//     //         bool safe = obs_checker_->isTrajectorySafeAgainstSingleObstacle(
//     //                         current_bridge_trajectory_, ob);
//     //         if (!safe) {
//     //             RCLCPP_ERROR(rclcpp::get_logger("SetRobotState"),
//     //                 "   *** Bridge collides with %s!  Obs pos (%.2f,%.2f) radius %.2f",
//     //                 ob.name.c_str(), ob.position.x(), ob.position.y(),
//     //                 ob.dimensions.radius + ob.inflation);
//     //         }
//     //     }
//     // }


// }


bool KinodynamicANYFMTX::isRobotSafe() {
    return (robot_node_ != nullptr) && (robot_node_->getLMC() != std::numeric_limits<double>::infinity());
}


void KinodynamicANYFMTX::addStaticObstacles(const ObstacleVector& obstacles) {

    for (const auto& ob : obstacles) {
        previous_obstacles_[ob.name] = ob;
        addNewStaticObstacle(previous_obstacles_[ob.name]);
    }

}

void KinodynamicANYFMTX::removeStaticObstacles(const ObstacleVector& obstacles) {
    for (const auto& ob : obstacles) {
        auto it = previous_obstacles_.find(ob.name);
        if (it != previous_obstacles_.end()) {
            removeStaticObstacle(it->second);
            previous_obstacles_.erase(it);
        }
    }
}

void KinodynamicANYFMTX::addNewStaticObstacle(const Obstacle& ob) {
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

void KinodynamicANYFMTX::removeStaticObstacle(const Obstacle& ob) {
    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);

    double search_radius;
    search_radius = obs_r + ob.inflation + neighborhood_radius_;

    std::unordered_set<int> freed_indices;

    // Use spatial KD-tree (x,y), NO TIME  
    Eigen::Vector2d obs_pos(ob.position.x(), ob.position.y());
    std::vector<size_t> indices = spatial_kdtree_->radiusSearch(obs_pos, search_radius);
    
    for (size_t idx : indices) {
        if (time_pillar_indices_.find(static_cast<int>(idx)) != time_pillar_indices_.end()) {
            continue; 
        }
        freed_indices.insert(static_cast<int>(idx));
    }

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







void KinodynamicANYFMTX::setRobotState(const Eigen::VectorXd& robot_state, bool anchor_was_reached = false) {
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


    if (!is_geometric_mode_) {
        // Phase 1: If anchor reached, try using cached parent trajectory
        if (anchor_was_reached  // <-- replace time_diff < 1e-2
            && robot_node_ 
            && robot_node_->getParent() != nullptr 
            && robot_node_->getLMC() != std::numeric_limits<double>::infinity()) 
        {
            FMTNode* next_anchor = robot_node_->getParent();
            auto cached_traj_ptr = robot_node_->getParentTrajectory();
            
            if (cached_traj_ptr && cached_traj_ptr->is_valid) {
                Trajectory cached_traj = *cached_traj_ptr;
                
                bool safe = true;
                for (const auto& [name, ob] : previous_obstacles_) {
                    if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(cached_traj, ob)) {
                        safe = false; break;
                    }
                }
                
                if (safe) {
                    robot_node_ = next_anchor;
                    current_bridge_trajectory_ = cached_traj;
                    bridge_cost_ = cached_traj.cost;
                    robot_current_time_to_goal_ = cached_traj.time_duration + next_anchor->getTimeToGoal();
                    last_replan_metrics_.path_cost = bridge_cost_ + next_anchor->getLMC();
                    return;
                }
            }
        }
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


    int steer_feasible_count = 0;   // <-- ADD: nodes reachable by valid steer
    int safe_feasible_count  = 0;   // <-- ADD: reachable AND collision-free
    int connected_count = 0;   // safe AND finite LMC



    // 1. PERFORM RADIUS SEARCH FOR A NEW ANCHOR
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        auto nearby_indices = kdtree_->radiusSearch(query_point, current_search_radius);
        for (auto idx : nearby_indices) {
            if (!tested_indices.insert(idx).second) {
                continue;
            }
            FMTNode* candidate = tree_[idx].get();
            // Time-cone prune: the bridge steer(robot -> candidate) needs tau(candidate) <
            // robot_sim_time, so a candidate beyond the robot's remaining budget can never be a
            // valid anchor. Skip the steer attempt (steer would reject it anyway). EXACT prune.
            if (TIME_CONE_PRUNED(candidate, robot_sim_time)) continue;
            Trajectory temp_bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());
            if (!temp_bridge.is_valid) continue;
            steer_feasible_count++;   // <-- ADD


            bool safe = true;
            for (const auto& [name, ob] : previous_obstacles_) {
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(temp_bridge, ob)) {
                    safe = false;
                    break;
                }
            }
            if (!safe) continue;
            safe_feasible_count++;    // <-- ADD


#if USE_RECOVERY
            if (temp_bridge.cost < best_fallback_cost) {
                best_fallback_node = candidate;
                best_fallback_bridge = temp_bridge;
                best_fallback_cost = temp_bridge.cost;
            }
#endif

            if (candidate->getLMC() != std::numeric_limits<double>::infinity()) {
                connected_count++;     // <-- ADD
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
        // diagnoseLostAnchor(robot_continuous_state_, query_point, current_search_radius);
    }



    // // one line per replan
    // double t_rem = upper_bounds_(upper_bounds_.size() - 1) - robot_sim_time;
    // FMTX_INFO("[Traj]"
    //     << " t_rem="          << t_rem
    //     << " x="              << robot_continuous_state_(0)
    //     << " y="              << robot_continuous_state_(1)
    //     << " theta="          << robot_continuous_state_(2)
    //     << " steer_feasible=" << steer_feasible_count
    //     << " safe_feasible="  << safe_feasible_count
    //     << " connected="      << connected_count
    //     << " radius="         << current_search_radius
    //     << " anchor="         << (best_candidate_node ? "OK" :
    //                               (best_fallback_node ? "FALLBACK" : "TRAP")));


}
// bool KinodynamicANYFMTX::isCurrentBridgeSafe(const ObstacleVector& obstacles) const {
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
bool KinodynamicANYFMTX::isCurrentBridgeSafe(const ObstacleVector& obstacles) const {
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

bool KinodynamicANYFMTX::hasReachedAnchor(const Eigen::VectorXd& current_sim_state) const {
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

// bool KinodynamicANYFMTX::hasReachedAnchor(const Eigen::VectorXd& current_sim_state) const {
//     if (!robot_node_) return false;
//     if (std::isinf(robot_node_->getLMC())) return false;

//     const Eigen::VectorXd& node = robot_node_->getStateValue();
//     const Eigen::VectorXd& rob  = current_sim_state;   // live pose, NOT robot_continuous_state_

//     // position gate (always required)
//     double pos_err = (rob.head(2) - node.head(2)).norm();

//     // heading gate when the state carries orientation:
//     //   size 4 -> (x, y, theta, t),  size 5 -> (x, y, theta, v, t)
//     //   size 3 -> (x, y, t) has no heading
//     double head_err = 0.0;
//     const bool has_heading = (rob.size() >= 4);
//     if (has_heading) {
//         double d = rob(2) - node(2);
//         head_err = std::abs(std::atan2(std::sin(d), std::cos(d)));  // wrap to [-pi, pi]
//     }

//     // TODO: tie these to your sim step (see caveat below)
//     const double anchor_pos_tol  = 0.5;   // meters
//     const double anchor_head_tol = 0.2;   // radians

//     return pos_err < anchor_pos_tol && head_err < anchor_head_tol;
// }



std::vector<Eigen::VectorXd> KinodynamicANYFMTX::getLivePathPositions(
    const Eigen::VectorXd& current_state) const
{
    if (!robot_node_ || robot_node_->getLMC() == std::numeric_limits<double>::infinity()) {
        return {};
    }

    std::vector<Eigen::VectorXd> final_path;

    // --- 1) Remaining bridge: trim cached trajectory, do not re-steer ---
    const Trajectory& bridge = current_bridge_trajectory_;
    if (bridge.is_valid && bridge.path_points.size() >= 2) {
        // Prefer time (last dim) if present; else nearest (x,y)
        const int t_idx = static_cast<int>(current_state.size()) - 1;
        const bool use_time = (current_state.size() >= 3);

        size_t start_i = 0;
        double best = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < bridge.path_points.size(); ++i) {
            double d;
            if (use_time && bridge.path_points[i].size() == current_state.size()) {
                // remaining-time / absolute-time consistency: match last component
                d = std::abs(bridge.path_points[i](t_idx) - current_state(t_idx));
            } else {
                d = (bridge.path_points[i].head<2>() - current_state.head<2>()).squaredNorm();
            }
            if (d < best) {
                best = d;
                start_i = i;
            }
        }

        // Path should start at the robot (smooth tip of the gradient)
        final_path.push_back(current_state);
        for (size_t i = start_i; i < bridge.path_points.size(); ++i) {
            // skip duplicate of current if very close
            if (i == start_i &&
                (bridge.path_points[i].head<2>() - current_state.head<2>()).norm() < 1e-3) {
                continue;
            }
            final_path.push_back(bridge.path_points[i]);
        }
    } else {
        // No cached bridge: optional steer, else empty
        Trajectory live = statespace_->steer(current_state, robot_node_->getStateValue());
        if (!live.is_valid || live.path_points.empty()) {
            return getPathPositions();  // last resort only
        }
        final_path = live.path_points;
    }

    // --- 2) Rest of tree from anchor (unchanged) ---
    FMTNode* child = robot_node_;
    FMTNode* parent = child->getParent();

    while (parent) {
        auto cached_traj = child->getParentTrajectory();
        if (cached_traj && cached_traj->is_valid && cached_traj->path_points.size() > 1) {
            final_path.insert(final_path.end(),
                              cached_traj->path_points.begin() + 1,
                              cached_traj->path_points.end());
        } else {
            break;
        }
        child = parent;
        parent = child->getParent();
    }

    return final_path;
}


// std::vector<Eigen::VectorXd> KinodynamicANYFMTX::getLivePathPositions(const Eigen::VectorXd& current_state) const
// {
//     if (!robot_node_ || robot_node_->getLMC() == std::numeric_limits<double>::infinity()) {
//         return {}; 
//     }

//     // 1. Create a temporary, real-time bridge from the robot to the anchor
//     Trajectory live_bridge = statespace_->steer(current_state, robot_node_->getStateValue());
    
//     if (!live_bridge.is_valid || live_bridge.path_points.empty()) {
//         // If the live steer fails (e.g. numerical issue very close to the node),
//         // fallback to the cached path
//         return getPathPositions(); 
//     }

//     // 2. Start the path with the live bridge
//     std::vector<Eigen::VectorXd> final_executable_path = live_bridge.path_points;

//     // 3. Traverse the rest of the tree from the anchor node
//     FMTNode* child = robot_node_;
//     FMTNode* parent = child->getParent();

//     while (parent) {
//         auto cached_traj = child->getParentTrajectory();
//         if (cached_traj->is_valid && cached_traj->path_points.size() > 1) {
//             final_executable_path.insert(final_executable_path.end(),
//                                          cached_traj->path_points.begin() + 1,
//                                          cached_traj->path_points.end());
//         } else {
//             break;
//         }
//         child = parent;
//         parent = child->getParent();
//     }

//     return final_executable_path;
// }

bool KinodynamicANYFMTX::hasShortcut(const Eigen::VectorXd& robot_state, double threshold) {
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



void KinodynamicANYFMTX::diagnoseLostAnchor(const Eigen::VectorXd& robot_state,
                                            const Eigen::VectorXd& query_point,
                                            double max_radius) {
    const double INF = std::numeric_limits<double>::infinity();

    auto free_edge = [&](const Trajectory& tr) {
        if (!tr.is_valid) return false;
        for (const auto& [name, ob] : previous_obstacles_) {
            if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(tr, ob)) return false;
        }
        return true;
    };

    // time coordinate = last dimension of the state vector
    const double t_robot = robot_state[robot_state.size() - 1];

    // ---- A: nodes the robot could anchor to IF their LMC were finite ----
    //        while scanning, categorize WHY each candidate fails.
    std::vector<size_t> reachable;
    size_t reachable_finite = 0;

    int n_total       = 0;   // candidates in radius
    int n_steer_fail  = 0;   // steer() returned no valid trajectory (dynamics/time infeasible)
    int n_collision   = 0;   // steer() ok but trajectory hit an obstacle
    int n_behind_time = 0;   // candidate time <= robot time (in robot's past)

    for (auto idx : kdtree_->radiusSearch(query_point, max_radius)) {
        ++n_total;
        FMTNode* cand = tree_[idx].get();
        const Eigen::VectorXd s = cand->getStateValue();

        if (s[s.size() - 1] <= t_robot) ++n_behind_time;

        Trajectory tr = statespace_->steer(robot_state, s);
        if (!tr.is_valid) { ++n_steer_fail; continue; }

        bool safe = true;
        for (const auto& [name, ob] : previous_obstacles_) {
            if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(tr, ob)) {
                safe = false; break;
            }
        }
        if (!safe) { ++n_collision; continue; }

        reachable.push_back(idx);
        if (cand->getLMC() != INF) ++reachable_finite;
    }

    if (reachable.empty()) {
        FMTX_WARN("[Trap Dx] 0/" << n_total << " candidates within " << max_radius
            << " are reachable. Breakdown: steer-infeasible=" << n_steer_fail
            << " (dynamics/time), collision-blocked=" << n_collision
            << ", behind-robot-in-time=" << n_behind_time
            << ". If steer-infeasible/behind-time dominate => FORWARD-TIME-CONE "
               "exhaustion (FMTX-relevant). If collision-blocked dominates => "
               "GENUINE GEOMETRIC TRAP.");
        diagnoseTimeCone(robot_state, query_point, max_radius);
        return;
    }
    if (reachable_finite > 0) {
        FMTX_WARN("[Trap Dx] " << reachable_finite
                  << " reachable nodes have FINITE LMC but anchoring still failed. "
                     "Bug in anchor selection, NOT graph repair.");
        return;
    }

    // ---- B: 1-hop orphaning. Reachable LMC=inf node one free edge from a finite node? ----
    size_t one_hop = 0;
    double best_recoverable = INF;
    for (auto xidx : reachable) {
        FMTNode* x = tree_[xidx].get();
        if (x->getLMC() != INF) continue;
        for (auto uidx : kdtree_->radiusSearch(x->getStateValue(), neighborhood_radius_)) {
            if (uidx == xidx) continue;
            FMTNode* u = tree_[uidx].get();
            if (u->getLMC() == INF) continue;
            Trajectory e = statespace_->steer(x->getStateValue(), u->getStateValue());
            if (!free_edge(e)) continue;
            ++one_hop;
            best_recoverable = std::min(best_recoverable, e.cost + u->getLMC());
            break;
        }
    }
    if (one_hop > 0) {
        FMTX_WARN("[Trap Dx] LAZY-CC ORPHANING CONFIRMED. " << one_hop
                  << " robot-reachable nodes sit at LMC=inf yet have a collision-free edge "
                     "to a finite-LMC neighbor (best recoverable cost=" << best_recoverable
                  << "). The wavefront skipped a valid connection.");
        return;
    }

    // ---- C: multi-hop. Flood the free graph from the finite (goal) component. ----
    std::unordered_set<size_t> reach_set(reachable.begin(), reachable.end());
    std::unordered_set<size_t> visited;
    std::queue<size_t> q;
    for (size_t i = 0; i < tree_.size(); ++i)
        if (tree_[i]->getLMC() != INF) { visited.insert(i); q.push(i); }

    const size_t budget = 200000;
    size_t exp = 0, hit = 0;
    bool reached = false;
    while (!q.empty() && exp < budget && !reached) {
        size_t cur = q.front(); q.pop();
        FMTNode* c = tree_[cur].get();
        for (auto nidx : kdtree_->radiusSearch(c->getStateValue(), neighborhood_radius_)) {
            ++exp;
            if (visited.count(nidx)) continue;
            FMTNode* n = tree_[nidx].get();
            // child(inf) -> parent(connected), toward goal
            if (!free_edge(statespace_->steer(n->getStateValue(), c->getStateValue()))) continue;
            visited.insert(nidx);
            if (reach_set.count(nidx)) { reached = true; hit = nidx; break; }
            q.push(nidx);
        }
    }

    if (reached) {
        FMTX_WARN("[Trap Dx] MULTI-HOP ORPHANING. A free-edge corridor connects the goal "
                  "component to robot-reachable node " << hit
                  << ", but every node on it stayed at LMC=inf. The wavefront failed to "
                     "propagate along an existing free corridor.");
        return;
    }

    FMTX_WARN("[Trap Dx] No free-edge path from the goal component reaches the robot's "
              "neighborhood (" << exp << " expansions). GENUINE TOPOLOGICAL TRAP. "
              "RRTX would fail here too.");
}


void KinodynamicANYFMTX::diagnoseTimeCone(const Eigen::VectorXd& robot_state,
                                          const Eigen::VectorXd& query_point,
                                          double max_radius) {
    const int TI = robot_state.size() - 1;          // time = last dim
    const double t_robot = robot_state[TI];

    // spatially-near nodes that are AHEAD in time, sorted by spatial distance
    std::vector<std::pair<double,size_t>> ahead;
    double t_max = t_robot;
    for (auto idx : kdtree_->radiusSearch(query_point, max_radius)) {
        const Eigen::VectorXd s = tree_[idx]->getStateValue();
        if (s[TI] <= t_robot) continue;
        ahead.push_back({(s.head(TI) - robot_state.head(TI)).squaredNorm(), idx});
        t_max = std::max(t_max, s[TI]);
    }
    std::sort(ahead.begin(), ahead.end());

    const int probes = std::min<int>(5, (int)ahead.size());
    const int steps  = 200;
    const double lo = t_robot, hi = t_max + (t_max - t_robot);
    const double step = (hi - lo) / steps;

    int recoverable = 0;
    for (int k = 0; k < probes; ++k) {
        Eigen::VectorXd probe = tree_[ahead[k].second]->getStateValue();
        const double own_t = probe[TI];
        bool found = false; double found_t = 0;
        for (int i = 1; i <= steps && !found; ++i) {
            probe[TI] = lo + i * step;
            if (statespace_->steer(robot_state, probe).is_valid) {
                found = true; found_t = probe[TI];
            }
        }
        if (found) {
            ++recoverable;
            FMTX_WARN("[Cone Dx] node " << ahead[k].second
                << " unreachable at its own time " << own_t
                << " but steer SUCCEEDS at time " << found_t
                << " (gap " << (found_t - t_robot) << "). Missing TIME slice.");
        }
    }

    if (recoverable > 0) {
        FMTX_WARN("[Cone Dx] " << recoverable << "/" << probes
            << " near nodes are reachable at SOME forward time, just not at the time "
               "FMTX sampled them. RRTX wins by placing a node in the robot's forward "
               "cone. Fix: inject samples / densify time, or allow dwell in steer.");
    } else {
        FMTX_WARN("[Cone Dx] 0/" << probes << " near nodes reachable at any forward time "
               "in sweep. Genuine dynamic infeasibility, not a missing slice.");
    }
}
