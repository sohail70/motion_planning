// Copyright 2025 Soheil E.nia

// TODO: Later implement KNN. with knn you wouldnt need cullNeighbor! use if (use_knn) return in cullNeighbor
#define DEBUG 0 // Debugs included are a full "Collision/Cost propagation/Espsilon consistency/Suboptimality average cost" recheck
#define VIS 0 // For visualizing open heap node to see the partial update in play
#define USE_THREAT_SET_STRATEGY 0 // Context-aware Threat set: The Threat Set is the bridge that allows a lazy algorithm (like FMTx) to behave with the same spatial intelligence as an eager one (Eager like RRTx)

#include "motion_planning/planners/kinodynamic/kinodynamic_any_fmtx.hpp"

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
    setGoal(problem_->getGoal());

    // KDTREE
    Eigen::MatrixXd all_samples = statespace_->getSamplesCopy();
    Eigen::MatrixXd spatial_samples_only = all_samples.leftCols(kd_dim).eval();
    // kdtree_->addPoints(spatial_samples_only);
    kdtree_->addPoints(all_samples);
    kdtree_->buildTree(); // Empty function in "DynamicWeightedNanoFlann" class
    std::cout << "KDTree is Initialized: \n\n";

    dimension_ = statespace_->getDimension();
    factor = params.getParam<double>("factor");
    delta = params.getParam<double>("delta");
    // Calculate initial radius based on N=2 (Start + Goal)
    shrinkingBallRadius();
    std::cout << "Setup complete. Ready for incremental sampling.\n";
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "------------------------------------------------------------\n";
}

void KinodynamicANYFMTX::shrinkingBallRadius() {
    int N = tree_.size();
    if (N <= 1) return;
    // int d = statespace_->getDimension();
    int d = kd_dim;
    Eigen::VectorXd range = upper_bounds_ - lower_bounds_;
    // double mu = range.prod();
    double mu = 1.0;
    for(int i = 0; i < d; ++i) {
        mu *= range(i);
    }
    double zetaD = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1.0);
    // double gamma = std::pow(1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); // FMT star gamma which is smaller than RRT* which makes the neighborhood size less than RRT*
    double gamma = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); // RRT* gamma

    neighborhood_radius_ = factor * gamma * std::pow(std::log(N) / N, 1.0 / d);
    neighborhood_radius_ = std::min(delta, neighborhood_radius_);
}

bool KinodynamicANYFMTX::updateNeighbors(const Eigen::VectorXd& sample_val, FMTNode* new_node) {
    std::vector<size_t> candidate_indices = kdtree_->radiusSearch(sample_val.head(kd_dim), neighborhood_radius_ + 0.01);
    bool is_valid_sample = false;

    // OUTGOING VALIDATION & DIRECT COMMIT
    for (size_t idx : candidate_indices) {
        FMTNode* neighbor = tree_[idx].get();
        Trajectory traj_outgoing = statespace_->steer(sample_val, neighbor->getStateValue());
        
        if (traj_outgoing.is_valid && traj_outgoing.cost <= neighborhood_radius_ + 0.01) {
            is_valid_sample = true; 
            
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

    // Bail early, no expensive steer checks wasted
    if (!is_valid_sample) {
        return false; 
    }

    // INCOMING VALIDATION
    if (!is_geometric_mode_) {
        for (size_t idx : candidate_indices) {
            FMTNode* neighbor = tree_[idx].get();
            Trajectory traj_incoming = statespace_->steer(neighbor->getStateValue(), sample_val);
            
            if (traj_incoming.is_valid && traj_incoming.cost <= neighborhood_radius_ + 0.01) {
                
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
    
    return true;
}


void KinodynamicANYFMTX::analyzeSuboptimality(FMTNode* x, FMTNode* best_parent_for_x, FMTNode* z, SuboptimalityMetrics& metrics) {
    metrics.total_nodes_updated++;
    int missed_for_this_node = 0;
    double chosen_cost = x->getCost();
    FMTNode* best_missed = nullptr;
    double best_missed_cost = chosen_cost;
    double witness_cost = z->getCost();

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

        double candidate_cost = y->getCost() + traj_xy->cost;
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
                std::cout << "[R2T_SUBOPT_DEBUG] Node " << x->getIndex()
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

        if (node->getParent() != nullptr && node->getCost() != std::numeric_limits<double>::infinity()) {
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
                          << " | Cost: " << node->getCost()
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
        
        if (node->getParent() == nullptr || node->getCost() == std::numeric_limits<double>::infinity()) {
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
        double stored_cost = node->getCost(); 

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
                          << " | Stored g(v): " << curr->getCost() 
                          << " | Broadcast g: " << curr->broadcast_cost_ << "\n";
                std::cout << "   -> Parent " << parent->getIndex() 
                          << " | Stored g(v): " << parent->getCost() 
                          << " | Broadcast g: " << parent->broadcast_cost_ << "\n";
                std::cout << "   -> Trajectory Edge Cost: " << edge_cost << "\n";
                
                // Check the exact math
                double expected_lmc = parent->broadcast_cost_ + edge_cost;
                std::cout << "   => Expected lmc(v): " << expected_lmc 
                          << " | Actual Drift at this step: " << (curr->getCost() - expected_lmc) << "\n";
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
        // -> getCost() acts as LMC (the instantly updated best-known path)
        // -> broadcast_cost_ acts as 'g' (the stale cost frozen until queue pops)
        double lmc = node->getCost();
        double g = node->broadcast_cost_;

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
            child->getCost() == std::numeric_limits<double>::infinity() || 
            child->getParent() == nullptr) {
            continue;
        }

        FMTNode* parent = child->getParent();
        auto traj = child->getParentTrajectory();
        if (!traj || !traj->is_valid) continue; 

        double expected_cost = parent->getCost() + traj->cost;
        double actual_cost = child->getCost();
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
/*
  Eager new sample insertion, later in plan we do lazy propagation
  In anytime fmtx eager approach we still dont care about all the collision checking connections from new node to neighbors or neighbors to new node
  except maybe the new sampled node to its forward neighbors! so we only collision check to find the parent here! but in anytime rrtx we have to check all of the
  edges and make them to have edge.distance of INF because that how rrtx works! but here because of lazy collision checking we dont have to do this!
  Even this collision checking was not necessary if i could've found a way to make the addBatchOfSamplesLazy to not become O(n log^2 (n))
  But here the O(n log(n)) will be preserved because we only do one heap push if the new node finds the best parent!
*/
void KinodynamicANYFMTX::addBatchOfSamplesEager(int num_samples) {
    if (num_samples <= 0) return;

    std::vector<int> added_node_indices;

    for (int i = 0; i < num_samples; ++i) {
        // Generate Sample
        // Eigen::VectorXd sample_val = statespace_->sampleUniform(lower_bounds_, upper_bounds_)->getValue();
        Eigen::VectorXd sample_val = Eigen::VectorXd::Random(dimension_);
        
        // Scale from [-1, 1] to [lower_bounds, upper_bounds]
        sample_val = lower_bounds_.array() + 
                    (upper_bounds_ - lower_bounds_).array() * ((sample_val.array() + 1.0) / 2.0);


        // This is just for fmtx to be complete! in case we have static obstalces!
        if (!obs_checker_->isObstacleFree(sample_val)) {
            continue;
        }

        // Create Node Object (Temporarily)
        // We create the node to get the pointer, but we don't push it to tree_ yet.
        auto node = std::make_shared<FMTNode>(statespace_->addState(sample_val), tree_.size());
        
    
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


        // CHECK FEASIBILITY & CONNECT
        // updateNeighbors will perform the steering and populate the neighbor maps.
        // It returns TRUE if at least one valid steer was found.
        if (!updateNeighbors(sample_val, node.get())) {
            // DISCARD: The node is kinematically unreachable.
            // It is NOT added to tree_, and the shared_ptr will go out of scope and be destroyed.
            continue; 
        }

        // COMMIT TO TREE
        // If we reach here, the node is good.
        int node_index = tree_.size();
        tree_.push_back(node);
        
        if (!is_geometric_mode_ && node->getStateValue().size() > 2) {
            double absolute_t = node->getStateValue().tail<1>()[0];
            node->setTimeToGoal(absolute_t);
        } else {
            node->setTimeToGoal(0.0);
        }
        kdtree_->addPoint(sample_val.head(kd_dim)); 
        
        added_node_indices.push_back(node_index);
    }

    if (added_node_indices.empty()) return;

    // BUILD KD-TREE --> BUILD TREE function is empty in case you use DynamicWeightedNanoFlann
    kdtree_->buildTree();

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
            if (neighbor->getCost() == INFINITY) {
                continue; 
            }
            double potential_cost = neighbor->getCost() + edge_info.distance; 
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
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*traj_xy, *ob_ptr)) {
                    collision_free = false;
                    break; // Short-circuit
                }
            }
#else
            // Default Blind strategy
            // Again, brute-force must check everything in previous_obstacles_
            for (const auto& [name, ob] : previous_obstacles_) {
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*traj_xy, ob)) {
                    collision_free = false;
                    break;
                }
            }
#endif

            if (collision_free) {
                // SUCCESS!
                new_node->setCost(candidate.first);
                new_node->setParent(potential_parent, traj_xy);
                connected = true;
                break; // Stop checking candidates.
            }
            // If NOT collision free, do nothing! Just let the while loop 
            // pop the next candidate from the heap. The edge remains intact 
            // for future dynamic repairs.
        }

        /*
            Important Insight on AO with epsilon usage: as you can see the new samples is added to the tree and now is in vopen heap! we do not immediately improve the neighbors
            which can use this! but if nothing happens and this new samples didnt get removed from the heap the neighbors would use this to connect. epsilon is not in the picture 
            until the queue operations in plan function!
            now if x's cost is improved little then we wouldn't cascade! 
            so the argument that when we use epsilon we have epsilon bounded AO is incorrect because those children that we didnt improve will get better by new samples!
            so all in all the new sample will eventually betters their immediate neighbors if partial update allows (i.e., its useful)
        */
        // THE SINGLE HEAP INSERTION
        if (connected) {
            v_open_heap_.add(new_node, new_node->getCost());
            last_replan_metrics_.queue_operations++;
        } 
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
        if (edge_cost > (neighborhood_radius_ + 0.01) && neighbor != v->getParent()) {

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
                continue; // Move to next neighbor
            }
        }
        ++it;
    }
    v->last_culled_radius_ = neighborhood_radius_;
}

void KinodynamicANYFMTX::plan() {
#if DEBUG
    SuboptimalityMetrics dbg_metrics;
#endif
    addBatchOfSamplesEager(num_of_samples_); // Add a small batch (e.g., 10) instead of 1


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
                robot_node_->getCost() == INFINITY ||
                // v_open_heap_.top().first < robot_node_->getCost() + bridge_cost_))
                v_open_heap_.top().first < robot_node_->getCost()))
    {

        auto top_element = v_open_heap_.top();
        double cost = top_element.first;
        FMTNode* z = top_element.second;
        int zIndex = z->getIndex();

        /*
            when a node becomes z (meaning it is acting as a parent and expanding the wavefront), 
            it broadcasts its cost. Right after we read z, we lock its broadcast cost
        */
        z->broadcast_cost_ = z->getCost(); 



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
            later loops over z's neighbors, the core condition `if (x->getCost() > cost_via_z)` naturally 
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

            double cost_via_z = z->getCost() + traj->cost;
            
            // If the parent brings a better cost, push it down to the child
            if (child->getCost() > cost_via_z) {
                child->setCost(cost_via_z);
                
                // Wake the child up so it can propagate the cost to its own children
                if (child->in_queue_) {
                    v_open_heap_.update(child, cost_via_z);
                    last_replan_metrics_.queue_operations++;
                } else if (child->broadcast_cost_ == INFINITY || (child->broadcast_cost_ - cost_via_z > epsilon)) {
                    v_open_heap_.add(child, cost_via_z); 
                    last_replan_metrics_.queue_operations++;
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
            const Trajectory& traj_xz = *(edge_info_x_to_z.cached_trajectory);
            if (!traj_xz.is_valid) {
                continue;
            }
            double cost_via_z = z->getCost() + edge_info_x_to_z.distance;
            /*
                This condition is the core of FMTX. It serves two purposes:
                If x has not been connected yet (cost is INF), this is always true, triggering its initial connection.
                If x is already connected, this condition acts as a "witness" or an "Alarm bell" that a better path might exist for x.
                It proves x's current cost is suboptimal and justifies the more expensive search that follows.
                This adds implicit rewiring to FMTX. There is no explicit rewiring here (like RRTX). It repairs the graph
                as wavefront expands.  
            */
            if (x->getCost() > cost_via_z) {

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
                double min_cost_for_x = x->getCost();
                FMTNode* best_parent_for_x = nullptr;
                std::shared_ptr<Trajectory> best_traj_for_x;

                                
                for (auto& [y, edge_info_xy] : x->forwardNeighbors()) {
                    if (y->in_queue_) { // We only consider parents that are in V_open.
                        auto traj_xy = edge_info_xy.cached_trajectory;
                        if (traj_xy->is_valid) {
                            double cost_via_y = y->getCost() + traj_xy->cost;
                            if (cost_via_y < min_cost_for_x) {
                                min_cost_for_x = cost_via_y;
                                best_parent_for_x = y;
                                best_traj_for_x = traj_xy;
                            }
                        }
                    }
                }



                // if (best_parent_for_x != nullptr && min_cost_for_x < (x->getCost())) { // Depend on min_cost_for_x initialization!
                if (best_parent_for_x != nullptr ) { 
                    bool obstacle_free = true;
                    if (best_parent_for_x != x->getParent()) {
#if USE_THREAT_SET_STRATEGY
                        if (!x->threats.empty()){
                            for (const Obstacle* ob_ptr : x->threats) {
                                last_replan_metrics_.obstacle_checks++;
                                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*best_traj_for_x, *ob_ptr)) {
                                    obstacle_free = false;
                                    break;
                                }
                            }
                        } else {
                            // Safe by default.
                            obstacle_free = true;
                        }
#else
                        // Use previous_obstacles_ so the brute-force mode actually sees the tubes!
                        for (const auto& [name, ob] : previous_obstacles_) {
                            last_replan_metrics_.obstacle_checks++;
                            if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*best_traj_for_x, ob)) {
                                obstacle_free = false;
                                break;
                            }
                        }
#endif
                    }

      
                    if (obstacle_free) {
                        x->setCost(min_cost_for_x);
                        x->setParent(best_parent_for_x, best_traj_for_x);
#if DEBUG
                        dbg_metrics.costUpdated[x] = true;
                        // Oracle!
                        analyzeSuboptimality(x, best_parent_for_x, z, dbg_metrics);
#endif
                        last_replan_metrics_.nodes_updated++;
                        double priorityCost = min_cost_for_x;


                        // THE EPSILON QUEUE BOUNCER
                        if (x->in_queue_) {
                            // If it is already scheduled to be processed, give it the best cost
                            v_open_heap_.update(x, priorityCost);
                            last_replan_metrics_.queue_operations++;
                        } 
                        else if (x->broadcast_cost_ == INFINITY || (x->broadcast_cost_ - min_cost_for_x > epsilon)) {
                            // If it is NOT in the queue, ONLY wake it up if the improvement is > epsilon 
                            // or if it is a brand new node or an orphan node that must expand the wavefront
                            v_open_heap_.add(x, priorityCost); 
                            last_replan_metrics_.queue_operations++;
                        }
                    }
                }
            }
        }
        v_open_heap_.pop();
        last_replan_metrics_.queue_operations++;
        // visualizeTree();
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } 




#if DEBUG 
    printDebugSummary(dbg_metrics);
    runCollisionForensics();
    runCostForensics();
    runGlobalCostForensics();
    runTreePropagationForensics();
#endif


}

std::vector<Eigen::VectorXd> KinodynamicANYFMTX::getPathPositions() const
{
    // Check if the planner has a valid anchor point for the robot (setRobotState should have found one).
    if (!robot_node_ || robot_node_->getCost() == INFINITY) {
        FMTX_ERROR("FMTX_Path_Assembly: Robot has no valid anchor node. Cannot build path");
        return {}; // Return empty path
    }

    // Generate the "bridge" trajectory from the robot's continuous state to the anchor node on the fly.
    Trajectory bridge_traj = statespace_->steer(robot_continuous_state_, robot_node_->getStateValue());

    if (!bridge_traj.is_valid) {
        FMTX_ERROR("FMTX_Path_Assembly: Failed to steer from robot's continuous state to the anchor node.");
        return {};
    }

    // Start the final path with this bridge trajectory.
    std::vector<Eigen::VectorXd> final_executable_path = bridge_traj.path_points;

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
    auto node = std::make_shared<FMTNode>(statespace_->addState(start),tree_.size());
    node->setCost(0);
    node->setTimeToGoal(0);
    v_open_heap_.add(node.get(),0);
    last_replan_metrics_.queue_operations++;
    tree_.push_back(node);
    std::cout << "KinodynamicANYFMTX: Start node created on Index: " << root_state_index_ << "\n";
}
void KinodynamicANYFMTX::setGoal(const Eigen::VectorXd& goal) {
    robot_state_index_ = statespace_->getNumStates();
    auto node = std::make_shared<FMTNode>(statespace_->addState(goal),tree_.size());
    node->in_unvisited_ = true;
    node->setTimeToGoal(std::numeric_limits<double>::infinity());
    robot_node_ = node.get();
    tree_.push_back(node);
    std::cout << "KinodynamicANYFMTX: Goal node created on Index: " << robot_state_index_ << "\n";
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
    //                         std::vector<float>{0.0f, 1.0f, 0.0f},  // Green color
    //                         "tree_nodes");
    
    visualization_->visualizeEdges(edges, "map");
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

        if (child_node->getCost() != std::numeric_limits<double>::infinity()) {
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
    visualization_->visualizeEdges(edges, "map");
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
        visualization_->visualizeEdges(edges, "map", "0.0,1.0,0.0", "executable_path");
    }
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
void KinodynamicANYFMTX::updateObstacleSamples(const ObstacleVector& turned_obstacles) {
    if (turned_obstacles.empty()) return;

    if (robot_continuous_state_.size() == 0) {
        FMTX_WARN("Planner_Obstacle_Update: Robot state not set.");
        return;
    }

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

        // UPDATE STATE
        stored_ob = incoming_ob; 

        // GENERATE NEW DENSE TUBE
        stored_ob.predicted_path = obs_checker_->generatePrediction(stored_ob, T_robot);

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
            if (idx != root_state_index_)
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
    */ 

    std::vector<int> filtered_orphan_indices;
    for (int idx : orphan_indices) {
        FMTNode* node = tree_[idx].get();
#if USE_THREAT_SET_STRATEGY
        // Mark this node as being under threat, ensuring no duplicates
        if (std::find(node->threats.begin(), node->threats.end(), &ob) == node->threats.end()) {
            node->threats.push_back(&ob);
        }

#endif
        // Skip root or nodes with no parent
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

    last_replan_metrics_.orphaned_nodes += orphan_indices.size();

    // Invalidate Nodes & Queue Boundary Parents
    std::unordered_set<FMTNode*> boundary_nodes_to_requeue;
    for (int node_index : orphan_indices) {
        auto node = tree_[node_index].get();
        if (node->in_queue_ && node->getIndex() != root_state_index_) {
            v_open_heap_.remove(node);
            last_replan_metrics_.queue_operations++;
        }

        // Invalidate Cost (but keep Root valid)
        if (node->getIndex() != root_state_index_) {
            node->setCost(INFINITY); 
            node->broadcast_cost_ = INFINITY;
            last_replan_metrics_.nodes_updated++;
        }
        
        // Sever Parent Connection
        node->setParent(nullptr, std::shared_ptr<Trajectory>{});

        // Find Boundary (Valid Parents)
        // We look at neighbors. If a neighbor is NOT an orphan, it's a valid candidate parent.
        // if (!neighbor_precache) near(node_index);
        
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
        if (!valid_node->in_queue_ && valid_node->getCost() != INFINITY) {
            v_open_heap_.add(valid_node, valid_node->getCost());
            last_replan_metrics_.queue_operations++;
        }
    }
}



// Wake Up Neighbors
void KinodynamicANYFMTX::removeObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);


    double search_radius;
    
    // GEOMETRIC VS KINODYNAMIC RADIUS
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
        } else if (kd_dim == 5) {
            query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z(); 
        }

        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) {
            if (idx == root_state_index_) continue; 
            freed_indices.insert(static_cast<int>(idx));
        }
    }

    // if (kd_dim == 4)
    //     search_radius += M_PI;

    // Queue Neighbors of Freed Nodes
    // We don't change costs here. We just put valid neighbors into the queue
    // to trigger the planner to explore this newly opened space.
    std::unordered_set<FMTNode*> neighbors_to_requeue;
    for (int node_index : freed_indices) {
        auto node = tree_.at(node_index).get();
#if USE_THREAT_SET_STRATEGY
        // O(1) SWAP-AND-POP THREAT REMOVAL
        auto it = std::find(node->threats.begin(), node->threats.end(), &ob);
        if (it != node->threats.end()) {
            *it = node->threats.back(); // Overwrite with the last element
            node->threats.pop_back();   // Delete the duplicate at the back
        }
#endif

        if (node->getCost()!= INFINITY) {
            continue; // If the node already is on the graph then its already free!
        }

        // if (!neighbor_precache) near(node_index);

        auto check_neighbors = [&](const auto& neighbors) {
            for (const auto& [neighbor_ptr, edge_data] : neighbors) {
                // If neighbor is valid (has cost) and not in queue, add it.
                if (neighbor_ptr->getCost() != INFINITY && !neighbor_ptr->in_queue_) {
                    neighbors_to_requeue.insert(neighbor_ptr);
                }
            }
        };

        check_neighbors(node->forwardNeighbors());
    }

    // Add to Open Heap
    for (FMTNode* neighbor : neighbors_to_requeue) {
        v_open_heap_.add(neighbor, neighbor->getCost());
        last_replan_metrics_.queue_operations++;
    }
}



void KinodynamicANYFMTX::setRobotState(const Eigen::VectorXd& robot_state) {
    robot_continuous_state_ = robot_state;
    // Extract actual planner-time from the state (last element)
    double robot_sim_time = robot_continuous_state_(robot_continuous_state_.size() - 1);

    // QUERY POINT CONSTRUCTION
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

    // HYSTERESIS LOGIC
    const double hysteresis_factor = 0.98;
    double cost_of_current_path = std::numeric_limits<double>::infinity();

    if (robot_node_ && robot_node_->getCost() != INFINITY) {
        Trajectory bridge = statespace_->steer(robot_continuous_state_, robot_node_->getStateValue());
        if (bridge.is_valid) {
            bool safe = true;
            const auto& obstacles = obs_checker_->getObstacles();
            for (const auto& ob : obstacles) {
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(bridge, ob)) {
                    safe = false;
                    break;
                }
            }
            if (safe) {
                cost_of_current_path = bridge.cost + robot_node_->getCost();
                robot_current_time_to_goal_ = bridge.time_duration + robot_node_->getTimeToGoal();
                // return;
            }
        }
    }

    FMTNode* best_candidate_node = nullptr;
    Trajectory best_candidate_bridge;
    double best_candidate_cost = std::numeric_limits<double>::infinity();
    
    // Radius Expansion to handle sparse graphs
    double current_search_radius = neighborhood_radius_; 
    const int max_attempts = 5; 
    const double radius_multiplier = 2.0;

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        auto nearby_indices = kdtree_->radiusSearch(query_point, current_search_radius);

        for (auto idx : nearby_indices) {
            FMTNode* candidate = tree_[idx].get();
            if (candidate->getCost() == INFINITY) continue;

            Trajectory bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());

            if (!bridge.is_valid) continue;
            

            bool safe = true;
            const auto& obstacles = obs_checker_->getObstacles();
            for (const auto& ob : obstacles) {
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(bridge, ob)) {
                    safe = false;
                    break;
                }
            }
            if (!safe) continue;



            double cost = bridge.cost + candidate->getCost();
            if (cost < best_candidate_cost) {
                best_candidate_node = candidate;
                best_candidate_bridge = bridge;
                best_candidate_cost = cost;
                bridge_cost_ = bridge.cost;
            }
        }
        if (best_candidate_node) break;
        current_search_radius *= radius_multiplier;
    }

    // ASSIGNMENT
    if (best_candidate_node && best_candidate_cost < cost_of_current_path * hysteresis_factor) {
        robot_node_ = best_candidate_node;
        robot_current_time_to_goal_ = best_candidate_bridge.time_duration + best_candidate_node->getTimeToGoal();
        last_replan_metrics_.path_cost = best_candidate_cost;
    } else if (robot_node_ && cost_of_current_path != std::numeric_limits<double>::infinity()) {
        Trajectory bridge = statespace_->steer(robot_continuous_state_, robot_node_->getStateValue());
        robot_current_time_to_goal_ = bridge.time_duration + robot_node_->getTimeToGoal();
        // The cost changes slightly every frame as the robot moves towards the anchor.
        last_replan_metrics_.path_cost = cost_of_current_path;
    } else {
        // We are trapped. No nodes in radius are safe.
        robot_node_ = nullptr;
        robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
        bridge_cost_ = std::numeric_limits<double>::infinity();
        last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
        FMTX_WARN("Set Robot State: LOST SAFE ANCHOR!");
    }


    // INTERNAL DEBUG VISUALIZATION
    if (visualization_) {
        if (robot_node_) {
            std::vector<Eigen::VectorXd> anchor_pt = { robot_node_->getStateValue().head<2>() };
            visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");
        } 
    }
}

bool KinodynamicANYFMTX::isRobotSafe() {
    return (robot_node_ != nullptr) && (robot_node_->getCost() != INFINITY);
}