// Copyright 2025 Soheil E.nia
// TODO: culled neighbor edges doesnt need to be in the orphan or edge dist inf! they are there just to find the incoming!
#include "motion_planning/planners/kinodynamic/kinodynamic_any_rrtx.hpp"
/*
    The Threat Set (Node-level)
    The Invalidating Set (Edge-level)

*/
#define USE_INVALIDATING_SET_STRATEGY 0
#define USE_THREAT_SET_STRATEGY 0
// If both are 0, it falls back to the Default/Brute-Force Strategy

#define DEBUG 0

KinodynamicANYRRTX::KinodynamicANYRRTX(std::shared_ptr<StateSpace> statespace, 
    std::shared_ptr<ProblemDefinition> problem_def,
    std::shared_ptr<ObstacleChecker> obs_checker): statespace_(statespace), problem_(problem_def), obs_checker_(obs_checker){
        std::cout<<"KinodynamicANYRRTX constructor \n";
}

// It sets the root of the tree in backward search
void KinodynamicANYRRTX::setStart(const Eigen::VectorXd& start) {
    auto index = statespace_->getNumStates();
    auto node = std::make_shared<RRTxNode>(statespace_->addState(start) ,  tree_.size());
    tree_.push_back(node);
    node->setTimeToGoal(0);
    node->setG(0);
    node->setLMC(0);
    std::cout << "KinodynamicANYRRTX: Start node created on Index: " << index << " with value: " << node->getStateValue() << "\n";
}
// It sets the robot's location in backward search
void KinodynamicANYRRTX::setGoal(const Eigen::VectorXd& goal) {
    auto index = statespace_->getNumStates();
    auto node = std::make_shared<RRTxNode>(statespace_->addState(goal) ,  tree_.size());
    vbot_node_ = node.get();
    node->setTimeToGoal(goal(goal.size() - 1));
    tree_.push_back(node);
    std::cout << "KinodynamicANYRRTX: Goal node created on Index: " << index << " with value: "<< node->getStateValue() << "\n";
}

std::vector<Eigen::VectorXd> KinodynamicANYRRTX::getPathPositions() const {
    // Check if the planner has a valid anchor point for the robot.
    if (!vbot_node_ || vbot_node_->getG() == std::numeric_limits<double>::infinity()) {
        RRTX_ERROR("[RRTX_Path_Assembly] Robot has no valid anchor node in the tree. Cannot build path.");
        return {}; // Return empty path
    }

    // // Generate the "bridge" trajectory from the robot's continuous state
    // //    to the anchor node on the fly.
    // Trajectory bridge_traj = statespace_->steer(robot_continuous_state_, vbot_node_->getStateValue());

    // if (!bridge_traj.is_valid) {
    //     RRTX_ERROR("[RRTX_Path_Assembly] Failed to steer from robot's continuous state to the anchor node.");
    //     return {};
    // }


    // Safety check on the cached bridge
    if (!current_bridge_trajectory_.is_valid || current_bridge_trajectory_.path_points.empty()) {
        RRTX_ERROR("RRTX_Path_Assembly: Cached bridge trajectory is invalid. Cannot build path");
        return {};
    }

    // Start the final path with the CACHED bridge trajectory! (Zero computation time)
    std::vector<Eigen::VectorXd> final_executable_path = current_bridge_trajectory_.path_points;


    // // Start the final path with this bridge trajectory.
    // std::vector<Eigen::VectorXd> final_executable_path = bridge_traj.path_points;

    // Traverse the rest of the tree from the anchor node using parent pointers.
    RRTxNode* child = vbot_node_;
    RRTxNode* parent = child->getParent();

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



void KinodynamicANYRRTX::clearPlannerState() {
    inconsistency_queue_.clear();
    for (auto& node : tree_) {
        node->disconnectFromGraph();
        node.reset();
    }
    tree_.clear();
    statespace_->reset();
    kdtree_.reset();
    Vc_T_.clear();
}


void KinodynamicANYRRTX::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    auto start = std::chrono::high_resolution_clock::now();
    clearPlannerState();
    visualization_ = visualization;
    num_of_samples_ = params.getParam<int>("num_of_samples");
    partial_update = params.getParam<bool>("partial_update");
    is_geometric_mode_ = params.getParam<bool>("is_geometric_mode", false);
    lower_bounds_ = problem_->getLowerBound();
    upper_bounds_ = problem_->getUpperBound();
    std::string kdtree_type = params.getParam<std::string>("kdtree_type");
    epsilon_ = params.getParam<double>("epsilon", 1e-4);
    kd_dim = params.getParam<int>("kd_dim",2);


    if (kdtree_type == "NanoFlann"){
        Eigen::VectorXd weights(kd_dim);
        switch (kd_dim) {
            case 2: // (x, y)
                weights << 1.0, 1.0; // Weights for x, y,
                break;
            case 3: // (x, y, time)
                {
                    weights << 1.0, 1.0, 1.0; // Weights for x, y, time
                }
                break;
            case 4: // (x, y, theta, time)
                {
                    weights << 1.0, 1.0, 1.0, 1.0; // Weights for x, y, theta, time
                }
                break;
            case 5: // (x, y, vx, vy, time)
                {
                    weights << 1.0, 1.0, 1.0, 1.0, 1.0; 
                }
                break;
            default: 
                RRTX_ERROR("Unsupported k-d tree dimension: " << kd_dim);
        }
        kdtree_ = std::make_shared<DynamicWeightedNanoFlann>(kd_dim, weights);
    } else if (kdtree_type == "LieKDTree"){
        kdtree_ = std::make_unique<LieSplittingKDTree>(statespace_->getDimension(), statespace_);
    } else {
        throw std::runtime_error("FMTX requires a KD-Tree.");
    }

    std::cout << "KinodynamicANYRRTX setup complete: num_of_samples per iteration=" << num_of_samples_
                << ", bounds=[" << lower_bounds_ << ", " << upper_bounds_ << "]\n";


    setStart(problem_->getStart());
    setGoal(problem_->getGoal()); //robots current position

    // KD-TREE
    Eigen::MatrixXd all_samples = statespace_->getSamplesCopy();
    int spatial_dimension = kd_dim;
    Eigen::MatrixXd spatial_samples_only = all_samples.leftCols(spatial_dimension).eval();
    // kdtree_->addPoints(spatial_samples_only);
    kdtree_->addPoints(all_samples);
    kdtree_->buildTree(); // Empty function in "DynamicWeightedNanoFlann" class

    ///////////////////Neighborhood Radius//////////////////////////
    dimension_ = statespace_->getDimension();
    int d = kd_dim;
    // double mu = std::pow(problem_->getUpperBound()[0] - problem_->getLowerBound()[0] , 2);
    Eigen::VectorXd range = upper_bounds_ - lower_bounds_;
    // double mu = range.prod(); // .prod() computes the product of all coefficients
    double mu = 1.0;
    for(int i = 0; i < d; ++i) {
        mu *= range(i);
    }
    std::cout<<"mu "<<mu<<"\n";
    double zetaD = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
    gamma_ = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); //RRT star gamma
    factor = params.getParam<double>("factor");
    std::cout<<"factor: "<<factor<<"\n";
    delta = params.getParam<double>("delta");
    std::cout << "delta: " << delta << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "---\n";
}

Eigen::VectorXd KinodynamicANYRRTX::saturate(const Eigen::VectorXd& newPoint, const Eigen::VectorXd& closestPoint, double delta) {
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

bool KinodynamicANYRRTX::runCollisionForensics() {
    std::cout << "\n[RRTx FORENSICS] --- STARTING POST-PLAN GRAPH VERIFICATION ---" << std::endl;
    int illegal_connections = 0;
    int checked_nodes = 0;

    // Start at 1 to skip the root/goal node which has no parent
    for (size_t i = 0; i < tree_.size(); ++i) { 
        RRTxNode* node = tree_[i].get();

        // Only check nodes that have an active parent connection
        if (node->getParent() != nullptr && node->getG() != std::numeric_limits<double>::infinity()) {
            checked_nodes++;
            bool edge_collides = false;
            std::string guilty_obstacle = "";
            RRTxNode* parent = node->getParent();
            
            // Re-steer from Child back to Parent (matching RRTX shortest-path direction)
            Trajectory edge_traj = statespace_->steer(node->getStateValue(), parent->getStateValue());



            // // Verify against Absolute Ground Truth (previous_obstacles_ map)
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
                          << " | Cost: " << node->getG()
                          << " | \033[1;35mFATAL: Edge hits [" << guilty_obstacle << "]\033[0m\n";
                
                // Root Cause Analysis: Is the graph aware of the break?
                if (node->outgoingEdges().count(parent)) {
                    double stored_dist = node->outgoingEdges().at(parent).distance;
                    std::cout << "   -> Stored Graph Distance (node->parent): " << stored_dist;
                    
                    if (stored_dist == std::numeric_limits<double>::infinity()) {
                        std::cout << " \033[1;31m(GRAPH KNEW IT WAS BLOCKED, BUT CONNECTED ANYWAY!)\033[0m\n";
                        std::cout << "      [!] Bug in RRTX Logic: Orphan/Inconsistency propagation failed.\n";
                    } else {
                        std::cout << " \033[1;33m(GRAPH THOUGHT IT WAS SAFE! Strategy missed it!)\033[0m\n";
                        std::cout << "      [!] Bug in Strategy Logic: Search radius or local filtering failed.\n";
                    }
                } else {
                    std::cout << "   -> \033[1;31mEdge doesn't even exist in child's outgoing map!\033[0m\n";
                }
            }


        }
    }
    
    if (illegal_connections > 0) {
        std::cout << "[RRTx FORENSICS] --- \033[1;31mFAILED\033[0m: Found " << illegal_connections 
                  << " violations out of " << checked_nodes << " connections ---\n\n";
        return false;
    } else {
        std::cout << "[RRTx FORENSICS] --- \033[1;32mPASSED\033[0m: Checked " << checked_nodes 
                  << " connections. Tree is 100% Collision-Free ---\n\n";
        return true;
    }
}

bool KinodynamicANYRRTX::runCostForensics() {
    std::cout << "\n[RRTx FORENSICS] --- STARTING STRICT EPSILON-CONSISTENCY VERIFICATION ---" << std::endl;
    int nodes_checked = 0;
    int violations = 0;
    int queue_waiting = 0;
    double max_local_inconsistency = 0.0;
    int max_node = -1;

    for (size_t i = 0; i < tree_.size(); ++i) {
        RRTxNode* node = tree_[i].get();
        
        double g = node->getG();
        double lmc = node->getLMC();

        // 1. Skip completely unconnected nodes
        if (g == std::numeric_limits<double>::infinity() && 
            lmc == std::numeric_limits<double>::infinity()) {
            continue;
        }

        // Calculate inconsistency
        double inconsistency = g - lmc;

        // 2. Is this node violating the epsilon bound?
        if (inconsistency > epsilon_ + 1e-5) {
            
            // ------------------------------------------------------------------
            // REPLACE THIS with whatever method you use to check if a node 
            // is currently in your priority queue (Q)
            bool is_in_queue = node->in_queue_; // <--- UPDATE THIS LINE
            // ------------------------------------------------------------------

            if (is_in_queue) {
                // This is perfectly legal! RRTX knows it's inconsistent 
                // and has queued it for lazy evaluation.
                queue_waiting++;
                continue; 
            } else {
                // THIS IS A REAL BUG! It is inconsistent and the algorithm forgot about it!
                violations++;
                std::cout << "\033[1;31m[REAL VIOLATION]\033[0m Node " << node->getIndex() 
                          << " | g: " << g << " | lmc: " << lmc 
                          << " | Diff: " << inconsistency << " > eps(" << epsilon_ << ")\n"
                          << "                 AND IT IS NOT IN THE QUEUE!\n";
            }
        }

        // If we reach here, the node is considered "settled"
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
        std::cout << "[RRTx FORENSICS] --- \033[1;31mFAILED\033[0m: Found " << violations 
                  << " nodes that are inconsistent AND missing from the queue! ---\n";
        return false;
    } else {
        std::cout << "[RRTx FORENSICS] --- \033[1;32mPASSED\033[0m: All nodes are either locally epsilon-consistent, or safely queued! ---\n";
        return true;
    }
}

void KinodynamicANYRRTX::plan() {
    for (int i = 0; i < num_of_samples_; ++i) {
        // Calculate Radius
        neighborhood_radius_ = shrinkingBallRadius();

        // Sample a point
        Eigen::VectorXd sample = Eigen::VectorXd::Random(dimension_);
        sample = (lower_bounds_.array() + (upper_bounds_ - lower_bounds_).array() * ((sample.array() + 1.0) / 2.0)).matrix();
        
        
        // Find Nearest
        std::vector<size_t> nearest_indices = kdtree_->knnSearch(sample.head(kd_dim), 1);
        RRTxNode* nearest_node = tree_[nearest_indices[0]].get();
        Eigen::VectorXd nearest_state = nearest_node->getStateValue();
        
        // Saturate
        sample = saturate(sample, nearest_state, delta);

        // Attempt to extend tree
        bool node_added = false;
        if (obs_checker_->isObstacleFree(sample)) {
            node_added = extend(sample);
        }
            
        // If added, rewire and reduce inconsistency
        if (node_added) {
            RRTxNode* new_node = tree_.back().get();
            // Update node costs and neighbors
            /*
                Is rewireNeighbors Necessary here? isnt reduceInconsistency function has rewire neighbor in it? Its is necessary here for new samples!
                imagine the robot is near root and we added a new sample to the envrionment far away 
                from robot then if we dont rewire neighbors then those neighbors wont get into the queue
                to later be processed by reducinconsistency.
            */
            rewireNeighbors(new_node); 
            // verifyQueue(new_node);
            new_node->setG(new_node->getLMC());
            reduceInconsistency();
        }
    }

    #if DEBUG
        runCollisionForensics();
        runCostForensics();
    #endif
    
}
// STRATEGY 1: INVALIDATING SET (Edge-Level Caching + Local Broad-Phase)
#if USE_INVALIDATING_SET_STRATEGY
bool KinodynamicANYRRTX::extend(Eigen::VectorXd v) {
    auto new_node = std::make_shared<RRTxNode>(statespace_->addState(v), tree_.size());
    auto neighbors = kdtree_->radiusSearch(new_node->getStateValue().head(kd_dim), neighborhood_radius_ + std::numeric_limits<double>::epsilon());
    
    double min_lmc = std::numeric_limits<double>::infinity();
    RRTxNode* best_parent = nullptr;
    std::shared_ptr<Trajectory> best_traj;

    // 1. LOCAL THREAT SET FOR EXTEND
    std::vector<const Obstacle*> local_threats;
    for (const auto& [name, ob] : previous_obstacles_) {
        if (obs_checker_->isNodeInObstacleTube(new_node->getStateValue(), ob, neighborhood_radius_ + std::numeric_limits<double>::epsilon())) {
            local_threats.push_back(&ob);
        }
    }

    if (!is_geometric_mode_) {
        double absolute_t = new_node->getStateValue().tail<1>()[0];
        new_node->setTimeToGoal(absolute_t);
    } else {
        new_node->setTimeToGoal(0.0);
    }

    evaluated_edges.resize(neighbors.size());
    for (size_t i = 0; i < neighbors.size(); ++i) {
        evaluated_edges[i].fwd_exists = false;
        evaluated_edges[i].rev_exists = false;
        evaluated_edges[i].fwd_blockers.clear();
        evaluated_edges[i].rev_blockers.clear();
    }

    // PASS 1: Evaluate OUTGOING edges (v -> u) & Find Parent
    for (size_t i = 0; i < neighbors.size(); ++i) {
        auto& candidate = tree_[neighbors[i]];
        if (candidate.get() == new_node.get()) continue;
        
        RRTxNode* u = candidate.get();
        evaluated_edges[i].neighbor = u;

        Trajectory temp_traj = statespace_->steer(new_node->getStateValue(), u->getStateValue());
        
        if (temp_traj.is_valid && temp_traj.cost <= neighborhood_radius_ + std::numeric_limits<double>::epsilon()) {
            auto shared_fwd_traj = std::make_shared<Trajectory>(std::move(temp_traj));
            
            evaluated_edges[i].fwd_exists = true;
            evaluated_edges[i].fwd_traj = shared_fwd_traj; // Store pointer
            evaluated_edges[i].fwd_safe = true;
            
            // Check against local threats
            for (const Obstacle* ob_ptr : local_threats) {
                const Obstacle& ob = *ob_ptr;
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*shared_fwd_traj, ob)) {
                    evaluated_edges[i].fwd_safe = false;
                    evaluated_edges[i].fwd_blockers.push_back(ob_ptr); 
                }
            }

            if (evaluated_edges[i].fwd_safe) {
                const double candidate_lmc = u->getLMC() + shared_fwd_traj->cost;
                if (candidate_lmc < min_lmc) {
                    min_lmc = candidate_lmc;
                    best_parent = u;
                    best_traj = shared_fwd_traj; // Reuse pointer
                }
            }
        }
    }

    // EARLY BAILOUT
    if (!best_parent) {
        return false; 
    }

    // PASS 2: Evaluate INCOMING edges (u -> v)
    for (size_t i = 0; i < neighbors.size(); ++i) {
        if (!evaluated_edges[i].neighbor) continue; 
        RRTxNode* u = evaluated_edges[i].neighbor;

        if (is_geometric_mode_) {
            // Geometric: Share the pointer
            evaluated_edges[i].rev_exists = evaluated_edges[i].fwd_exists;
            evaluated_edges[i].rev_traj = evaluated_edges[i].fwd_traj;
            evaluated_edges[i].rev_safe = evaluated_edges[i].fwd_safe;
            evaluated_edges[i].rev_blockers = evaluated_edges[i].fwd_blockers;
        } else {
            Trajectory temp_rev_traj = statespace_->steer(u->getStateValue(), new_node->getStateValue());
            if (temp_rev_traj.is_valid && temp_rev_traj.cost <= neighborhood_radius_ + std::numeric_limits<double>::epsilon()) {
                auto shared_rev_traj = std::make_shared<Trajectory>(std::move(temp_rev_traj));
                
                evaluated_edges[i].rev_exists = true;
                evaluated_edges[i].rev_traj = shared_rev_traj;
                evaluated_edges[i].rev_safe = true;
                
                for (const Obstacle* ob_ptr : local_threats) {
                    const Obstacle& ob = *ob_ptr;
                    last_replan_metrics_.obstacle_checks++;
                    if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*shared_rev_traj, ob)) {
                        evaluated_edges[i].rev_safe = false;
                        evaluated_edges[i].rev_blockers.push_back(ob_ptr);
                    }
                }
            }
        }
    }

    // COMMIT GRAPH CHANGES
    new_node->setParent(best_parent, best_traj);
    new_node->setLMC(min_lmc);
    tree_.push_back(new_node);
    kdtree_->addPoint(new_node->getStateValue().head(kd_dim));
    kdtree_->buildTree(); // Build tree is an empty function in DynamicKdtree
    last_replan_metrics_.nodes_updated++;

    for (auto& eval : evaluated_edges) {
        if (!eval.neighbor) continue;
        if (eval.fwd_exists) {
            new_node->addNeighbor(eval.neighbor, true, false, eval.fwd_traj);
            if (!eval.fwd_safe) {
                new_node->outgoingEdges().at(eval.neighbor).distance = std::numeric_limits<double>::infinity();
                new_node->outgoingEdges().at(eval.neighbor).invalidating_obstacles = eval.fwd_blockers;
                if (eval.neighbor->incomingEdges().count(new_node.get())) {
                    eval.neighbor->incomingEdges().at(new_node.get()).distance = std::numeric_limits<double>::infinity();
                    eval.neighbor->incomingEdges().at(new_node.get()).invalidating_obstacles = eval.fwd_blockers;
                }
            }
        }
        if (eval.rev_exists) {
            eval.neighbor->addNeighbor(new_node.get(), false, true, eval.rev_traj);
            if (!eval.rev_safe) {
                eval.neighbor->outgoingEdges().at(new_node.get()).distance = std::numeric_limits<double>::infinity();
                eval.neighbor->outgoingEdges().at(new_node.get()).invalidating_obstacles = eval.rev_blockers;
                if (new_node->incomingEdges().count(eval.neighbor)) {
                    new_node->incomingEdges().at(eval.neighbor).distance = std::numeric_limits<double>::infinity();
                    new_node->incomingEdges().at(eval.neighbor).invalidating_obstacles = eval.rev_blockers;
                }
            }
        }
    }
    return true;
}
// ==============================================================================================
// STRATEGY 2: THREAT SET (Node-Level Filtering)
// ==============================================================================================

#elif USE_THREAT_SET_STRATEGY
bool KinodynamicANYRRTX::extend(Eigen::VectorXd v) {
    auto new_node = std::make_shared<RRTxNode>(statespace_->addState(v), tree_.size());
    auto neighbors = kdtree_->radiusSearch(new_node->getStateValue().head(kd_dim), neighborhood_radius_ + std::numeric_limits<double>::epsilon());
    
    double min_lmc = std::numeric_limits<double>::infinity();
    RRTxNode* best_parent = nullptr;
    std::shared_ptr<Trajectory> best_traj; // Use shared_ptr

    if (!is_geometric_mode_) {
        double absolute_t = new_node->getStateValue().tail<1>()[0];
        new_node->setTimeToGoal(absolute_t);
    } else {
        new_node->setTimeToGoal(0.0);
    }

    // INITIALIZE NODE-BASED THREAT SET
    for (const auto& [name, ob] : previous_obstacles_) {
        if (obs_checker_->isNodeInObstacleTube(new_node->getStateValue(), ob, delta)) {
            new_node->threats_.push_back(&ob);
        }
    }

    // Resize uses existing capacity, ZERO heap allocations!
    evaluated_edges.resize(neighbors.size());
    for (size_t i = 0; i < neighbors.size(); ++i) {
        evaluated_edges[i].fwd_exists = false;
        evaluated_edges[i].rev_exists = false;
        evaluated_edges[i].fwd_blockers.clear();
        evaluated_edges[i].rev_blockers.clear();
    }

    // PASS 1: Evaluate OUTGOING edges (v -> u) & Find Parent
    for (size_t i = 0; i < neighbors.size(); ++i) {
        auto& candidate = tree_[neighbors[i]];
        if (candidate.get() == new_node.get()) continue;
        
        RRTxNode* u = candidate.get();
        evaluated_edges[i].neighbor = u;

        Trajectory temp_traj = statespace_->steer(new_node->getStateValue(), u->getStateValue());
        
        if (temp_traj.is_valid && temp_traj.cost <= neighborhood_radius_ + std::numeric_limits<double>::epsilon()) {
            auto shared_fwd_traj = std::make_shared<Trajectory>(std::move(temp_traj));
            
            evaluated_edges[i].fwd_exists = true;
            evaluated_edges[i].fwd_traj = shared_fwd_traj;
            evaluated_edges[i].fwd_safe = true;
            
            for (const Obstacle* threat_ptr : new_node->threats_) {
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*shared_fwd_traj, *threat_ptr)) {
                    evaluated_edges[i].fwd_safe = false;
                    break; 
                }
            }

            if (evaluated_edges[i].fwd_safe) {
                const double candidate_lmc = u->getLMC() + shared_fwd_traj->cost;
                if (candidate_lmc < min_lmc) {
                    min_lmc = candidate_lmc;
                    best_parent = u;
                    best_traj = shared_fwd_traj; // Reuse pointer
                }
            }
        }
    }

    if (!best_parent) {
        return false; 
    }

    // PASS 2: Evaluate INCOMING edges (u -> v)
    for (size_t i = 0; i < neighbors.size(); ++i) {
        if (!evaluated_edges[i].neighbor) continue; 
        RRTxNode* u = evaluated_edges[i].neighbor;

        if (is_geometric_mode_) {
            evaluated_edges[i].rev_exists = evaluated_edges[i].fwd_exists;
            evaluated_edges[i].rev_traj = evaluated_edges[i].fwd_traj;
            evaluated_edges[i].rev_safe = evaluated_edges[i].fwd_safe;
        } else {
            Trajectory temp_rev_traj = statespace_->steer(u->getStateValue(), new_node->getStateValue());
            if (temp_rev_traj.is_valid && temp_rev_traj.cost <= neighborhood_radius_ + std::numeric_limits<double>::epsilon()) {
                auto shared_rev_traj = std::make_shared<Trajectory>(std::move(temp_rev_traj));
                
                evaluated_edges[i].rev_exists = true;
                evaluated_edges[i].rev_traj = shared_rev_traj;
                evaluated_edges[i].rev_safe = true;
                
                for (const Obstacle* threat_ptr : u->threats_) {
                    last_replan_metrics_.obstacle_checks++;
                    if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*shared_rev_traj, *threat_ptr)) {
                        evaluated_edges[i].rev_safe = false;
                        break;
                    }
                }
            }
        }
    }

    // COMMIT GRAPH CHANGES
    new_node->setParent(best_parent, best_traj);
    new_node->setLMC(min_lmc);
    tree_.push_back(new_node);
    kdtree_->addPoint(new_node->getStateValue().head(kd_dim));
    kdtree_->buildTree(); 
    last_replan_metrics_.nodes_updated++;

    for (auto& eval : evaluated_edges) {
        if (!eval.neighbor) continue;
        if (eval.fwd_exists) {
            new_node->addNeighbor(eval.neighbor, true, false, eval.fwd_traj);
            if (!eval.fwd_safe) {
                new_node->outgoingEdges().at(eval.neighbor).distance = std::numeric_limits<double>::infinity();
                if (eval.neighbor->incomingEdges().count(new_node.get())) {
                    eval.neighbor->incomingEdges().at(new_node.get()).distance = std::numeric_limits<double>::infinity();
                }
            }
        }
        if (eval.rev_exists) {
            eval.neighbor->addNeighbor(new_node.get(), false, true, eval.rev_traj);
            if (!eval.rev_safe) {
                eval.neighbor->outgoingEdges().at(new_node.get()).distance = std::numeric_limits<double>::infinity();
                if (new_node->incomingEdges().count(eval.neighbor)) {
                    new_node->incomingEdges().at(eval.neighbor).distance = std::numeric_limits<double>::infinity();
                }
            }
        }
    }
    return true;
}
// ==============================================================================================
// STRATEGY 3: DEFAULT (Brute-Force Fallback)
// ==============================================================================================
#else

// bool KinodynamicANYRRTX::extend(Eigen::VectorXd v) {
//     auto new_node = std::make_shared<RRTxNode>(statespace_->addState(v), tree_.size());
//     auto neighbors = kdtree_->radiusSearch(new_node->getStateValue().head(kd_dim), neighborhood_radius_ + std::numeric_limits<double>::epsilon());
    
//     double min_lmc = std::numeric_limits<double>::infinity();
//     RRTxNode* best_parent = nullptr;
//     Trajectory best_traj;

//     if (!is_geometric_mode_) {
//         double absolute_t = new_node->getStateValue().tail<1>()[0];
//         new_node->setTimeToGoal(absolute_t);
//     } else {
//         new_node->setTimeToGoal(0.0);
//     }

//     // Fetch globally tracked obstacles for brute-force checking
//     const ObstacleVector& all_obstacles = obs_checker_->getObstacles();

//     // Resize uses existing capacity, ZERO heap allocations!
//     evaluated_edges.resize(neighbors.size());
//     for (size_t i = 0; i < neighbors.size(); ++i) {
//         evaluated_edges[i].fwd_exists = false;
//         evaluated_edges[i].rev_exists = false;
//         // evaluated_edges[i].fwd_blockers.clear(); // Clears data, keeps memory capacity
//         // evaluated_edges[i].rev_blockers.clear(); // Clears data, keeps memory capacity
//     }

//     // PASS 1: Evaluate OUTGOING edges (v -> u) & Find Parent
//     for (size_t i = 0; i < neighbors.size(); ++i) {
//         auto& candidate = tree_[neighbors[i]];
//         if (candidate.get() == new_node.get()) continue;
        
//         RRTxNode* u = candidate.get();
//         evaluated_edges[i].neighbor = u;

//         Trajectory fwd_traj = statespace_->steer(new_node->getStateValue(), u->getStateValue());
//         if (fwd_traj.is_valid && fwd_traj.cost <= neighborhood_radius_ + std::numeric_limits<double>::epsilon()) {
//             evaluated_edges[i].fwd_exists = true;
//             evaluated_edges[i].fwd_traj = fwd_traj;
//             evaluated_edges[i].fwd_safe = true;
            
//             // Brute force against all known obstacles
//             for (const auto& ob : all_obstacles) {
//                 last_replan_metrics_.obstacle_checks++;
//                 if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(fwd_traj, ob)) {
//                     evaluated_edges[i].fwd_safe = false;
//                     break; 
//                 }
//             }

//             if (evaluated_edges[i].fwd_safe) {
//                 const double candidate_lmc = u->getLMC() + fwd_traj.cost;
//                 if (candidate_lmc < min_lmc) {
//                     min_lmc = candidate_lmc;
//                     best_parent = u;
//                     best_traj = std::move(fwd_traj); 
//                 }
//             }
//         }
//     }

//     if (!best_parent) {
//         return false; 
//     }

//     // PASS 2: Evaluate INCOMING edges (u -> v)
//     for (size_t i = 0; i < neighbors.size(); ++i) {
//         if (!evaluated_edges[i].neighbor) continue; 
//         RRTxNode* u = evaluated_edges[i].neighbor;

//         if (is_geometric_mode_) {
//             evaluated_edges[i].rev_exists = evaluated_edges[i].fwd_exists;
//             evaluated_edges[i].rev_traj = evaluated_edges[i].fwd_traj;
//             evaluated_edges[i].rev_safe = evaluated_edges[i].fwd_safe;
//         } else {
//             Trajectory rev_traj = statespace_->steer(u->getStateValue(), new_node->getStateValue());
//             if (rev_traj.is_valid && rev_traj.cost <= neighborhood_radius_ + std::numeric_limits<double>::epsilon()) {
//                 evaluated_edges[i].rev_exists = true;
//                 evaluated_edges[i].rev_traj = rev_traj;
//                 evaluated_edges[i].rev_safe = true;
                
//                 // Brute force reverse edge
//                 for (const auto& ob : all_obstacles) {
//                     last_replan_metrics_.obstacle_checks++;
//                     if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(rev_traj, ob)) {
//                         evaluated_edges[i].rev_safe = false;
//                         break;
//                     }
//                 }
//             }
//         }
//     }

//     // COMMIT GRAPH CHANGES
//     new_node->setParent(best_parent, best_traj);
//     new_node->setLMC(min_lmc);

//     tree_.push_back(new_node);
//     kdtree_->addPoint(new_node->getStateValue().head(kd_dim));
//     kdtree_->buildTree(); 

//     for (auto& eval : evaluated_edges) {
//         if (!eval.neighbor) continue;

//         if (eval.fwd_exists) {
//             new_node->addNeighbor(eval.neighbor, true, false, eval.fwd_traj);
//             if (!eval.fwd_safe) {
//                 new_node->outgoingEdges().at(eval.neighbor).distance = std::numeric_limits<double>::infinity();
//                 if (eval.neighbor->incomingEdges().count(new_node.get())) {
//                     eval.neighbor->incomingEdges().at(new_node.get()).distance = std::numeric_limits<double>::infinity();
//                 }
//             }
//         }

//         if (eval.rev_exists) {
//             eval.neighbor->addNeighbor(new_node.get(), false, true, eval.rev_traj);
//             if (!eval.rev_safe) {
//                 eval.neighbor->outgoingEdges().at(new_node.get()).distance = std::numeric_limits<double>::infinity();
//                 if (new_node->incomingEdges().count(eval.neighbor)) {
//                     new_node->incomingEdges().at(eval.neighbor).distance = std::numeric_limits<double>::infinity();
//                 }
//             }
//         }
//     }

//     return true;
// }
bool KinodynamicANYRRTX::extend(Eigen::VectorXd v) {
    auto new_node = std::make_shared<RRTxNode>(statespace_->addState(v), tree_.size());
    auto neighbors = kdtree_->radiusSearch(new_node->getStateValue().head(kd_dim), neighborhood_radius_ + std::numeric_limits<double>::epsilon());
    
    double min_lmc = std::numeric_limits<double>::infinity();
    RRTxNode* best_parent = nullptr;
    std::shared_ptr<Trajectory> best_traj; // Use shared_ptr to hold the best trajectory

    if (!is_geometric_mode_) {
        double absolute_t = new_node->getStateValue().tail<1>()[0];
        new_node->setTimeToGoal(absolute_t);
    } else {
        new_node->setTimeToGoal(0.0);
    }
    double v_time = new_node->getTimeToGoal();

    // Fetch globally tracked obstacles for brute-force checking
    const ObstacleVector& all_obstacles = obs_checker_->getObstacles();

    // Resize uses existing capacity, ZERO heap allocations!
    evaluated_edges.resize(neighbors.size());
    for (size_t i = 0; i < neighbors.size(); ++i) {
        evaluated_edges[i].fwd_exists = false;
        evaluated_edges[i].rev_exists = false;
    }

    // PASS 1: Evaluate OUTGOING edges (v -> u) & Find Parent
    for (size_t i = 0; i < neighbors.size(); ++i) {
        auto& candidate = tree_[neighbors[i]];
        if (candidate.get() == new_node.get()) continue;
        RRTxNode* u = candidate.get();
        evaluated_edges[i].neighbor = u;

        // 1. Steer
        Trajectory temp_traj = statespace_->steer(new_node->getStateValue(), u->getStateValue());

        if (temp_traj.is_valid && temp_traj.cost <= neighborhood_radius_ + std::numeric_limits<double>::epsilon()) {
            // 2. Wrap in shared_ptr immediately (Optimization: Avoids copy inside EdgeEval)
            auto shared_fwd_traj = std::make_shared<Trajectory>(std::move(temp_traj));
            
            evaluated_edges[i].fwd_exists = true;
            evaluated_edges[i].fwd_traj = shared_fwd_traj; // Store pointer
            evaluated_edges[i].fwd_safe = true;

            // Brute force against all known obstacles
            for (const auto& ob : all_obstacles) {
                last_replan_metrics_.obstacle_checks++;
                // Dereference pointer for the checker
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*shared_fwd_traj, ob)) {
                    evaluated_edges[i].fwd_safe = false;
                    break;
                }
            }

            if (evaluated_edges[i].fwd_safe) {
                const double candidate_lmc = u->getLMC() + shared_fwd_traj->cost;
                if (candidate_lmc < min_lmc) {
                    min_lmc = candidate_lmc;
                    best_parent = u;
                    best_traj = shared_fwd_traj; // Reuse the pointer, no copy
                }
            }
        }
    }

    if (!best_parent) {
        return false;
    }

    // PASS 2: Evaluate INCOMING edges (u -> v)
    for (size_t i = 0; i < neighbors.size(); ++i) {
        if (!evaluated_edges[i].neighbor) continue;
        RRTxNode* u = evaluated_edges[i].neighbor;

        if (is_geometric_mode_) {
            // Geometric: Incoming is identical to Outgoing
            evaluated_edges[i].rev_exists = evaluated_edges[i].fwd_exists;
            evaluated_edges[i].rev_traj = evaluated_edges[i].fwd_traj; // Share pointer
            evaluated_edges[i].rev_safe = evaluated_edges[i].fwd_safe;
        } else {
            // Kinodynamic: Must steer separately
            Trajectory temp_rev_traj = statespace_->steer(u->getStateValue(), new_node->getStateValue());
            if (temp_rev_traj.is_valid && temp_rev_traj.cost <= neighborhood_radius_ + std::numeric_limits<double>::epsilon()) {
                auto shared_rev_traj = std::make_shared<Trajectory>(std::move(temp_rev_traj));
                
                evaluated_edges[i].rev_exists = true;
                evaluated_edges[i].rev_traj = shared_rev_traj;
                evaluated_edges[i].rev_safe = true;

                // Brute force reverse edge
                for (const auto& ob : all_obstacles) {
                    last_replan_metrics_.obstacle_checks++;
                    if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*shared_rev_traj, ob)) {
                        evaluated_edges[i].rev_safe = false;
                        break;
                    }
                }
            }
        }
    }

    // COMMIT GRAPH CHANGES
    new_node->setParent(best_parent, best_traj); 
    new_node->setLMC(min_lmc);
    tree_.push_back(new_node);
    kdtree_->addPoint(new_node->getStateValue().head(kd_dim));
    kdtree_->buildTree();
    last_replan_metrics_.nodes_updated++;

    for (auto& eval : evaluated_edges) {
        if (!eval.neighbor) continue;

        if (eval.fwd_exists) {
            new_node->addNeighbor(eval.neighbor, true, false, eval.fwd_traj);
            
            if (!eval.fwd_safe) {
                new_node->outgoingEdges().at(eval.neighbor).distance = std::numeric_limits<double>::infinity();
                if (eval.neighbor->incomingEdges().count(new_node.get())) {
                    eval.neighbor->incomingEdges().at(new_node.get()).distance = std::numeric_limits<double>::infinity();
                }
            }
        }

        if (eval.rev_exists) {
            eval.neighbor->addNeighbor(new_node.get(), false, true, eval.rev_traj);
            
            if (!eval.rev_safe) {
                eval.neighbor->outgoingEdges().at(new_node.get()).distance = std::numeric_limits<double>::infinity();
                if (new_node->incomingEdges().count(eval.neighbor)) {
                    new_node->incomingEdges().at(eval.neighbor).distance = std::numeric_limits<double>::infinity();
                }
            }
        }
    }

    return true;
}

#endif



void KinodynamicANYRRTX::rewireNeighbors(RRTxNode* v) {
    const double inconsistency = v->getG() - v->getLMC();
    if (inconsistency <= epsilon_) return;
    cullNeighbors(v);
    for (auto& [u, edge] : v->incomingEdges()) {
        if (u == v->getParent() ) continue;
        const double candidate_lmc = v->getLMC() + edge.distance;
        if (u->getLMC() > candidate_lmc) {
            u->setLMC(candidate_lmc);
            u->setParent(v, edge.cached_trajectory);
            last_replan_metrics_.nodes_updated++;
            if (u->getG() - candidate_lmc > epsilon_) {
                verifyQueue(u);
            }
        }
    }
}

void KinodynamicANYRRTX::reduceInconsistency() {
    while (!inconsistency_queue_.empty()) {
        auto top_element = inconsistency_queue_.top();
        double min_key = top_element.first;
        if (partial_update && vbot_node_) {
            // Check if Robot is already consistent (Cost == LMC)
            bool robot_consistent = (vbot_node_->getG() == vbot_node_->getLMC());
            // Check if the queue has passed the robot
            // We stop if the smallest key in the queue is greater than the robot's cost.
            // bool queue_past_robot = (min_key > vbot_node_->getG() + bridge_cost_);
            bool queue_past_robot = (min_key > vbot_node_->getG());
            // STOP CONDITION:
            // If the robot is consistent AND the queue only contains nodes more expensive than the robot,
            // then we have successfully repaired the path up to the robot.
            if (robot_consistent && queue_past_robot) {
                break;
            }
        }
        inconsistency_queue_.pop();
        last_replan_metrics_.queue_operations++;
        RRTxNode* node = top_element.second;
        // Standard RRTx logic: if Cost > LMC + eps, we need to update
        if (node->getG() > node->getLMC() + epsilon_) {
            updateLMC(node);
            rewireNeighbors(node);
        }

        /*
            Synchronize Cost and LMC
            Mind that some nodes have g and lmc with less than epsilon apart. 
            The above filter keeps this improvement a secret from its children but the node it self uses that little improvement

        */ 
        if (node->getG() != node->getLMC()) {
            node->setG(node->getLMC());
            last_replan_metrics_.nodes_updated++;
        }
    }
    // Debugging
    // std::cout << "Queue size after reduce: " << inconsistency_queue_.getHeap().size() << "\n";
}



double KinodynamicANYRRTX::shrinkingBallRadius() const {
    int d = kd_dim;
    auto rad = factor * gamma_ * pow(log(tree_.size()) / tree_.size(), 1.0/d);
    return std::min(rad, delta);
}

/*
    Why not start the min_lmc variable from INF? 
    This function is not for clearing LMC and recalculate it because RRTx speicifically has orphan handling 
    As opposed to D star which doesnt explicitly hadnles orphans. This function only updates to better LMC
    this difference between recomputeRHS in D starlite and updateLMC in RRTx
*/
void KinodynamicANYRRTX::updateLMC(RRTxNode* v) {
    cullNeighbors(v);
    double min_lmc = v->getLMC(); 
    RRTxNode* best_parent = nullptr;
    double best_edge_distance = std::numeric_limits<double>::infinity();
    std::shared_ptr<Trajectory> best_traj; 
    // Iterate over outgoing edges (v → u)
    for (auto& [u, edge] : v->outgoingEdges()) {
        if (Vc_T_.count(u->getIndex()) || edge.distance == std::numeric_limits<double>::infinity()) continue;
        const double candidate_lmc = u->getLMC() + edge.distance;
        if (candidate_lmc < min_lmc) {
            min_lmc = candidate_lmc;
            best_parent = u;
            best_edge_distance = edge.distance;
            best_traj = edge.cached_trajectory;
        }
    }
    if (best_parent) {
        v->setParent(best_parent, best_traj);
        v->setLMC(min_lmc);
        last_replan_metrics_.nodes_updated++;
    } 
}


void KinodynamicANYRRTX::cullNeighbors(RRTxNode* v) {
    if (v->last_culled_radius_ > 0 && 
        (v->last_culled_radius_ / neighborhood_radius_) < 1.0001) {
        return; 
    }

    auto& outgoing = v->outgoingEdges();
    auto it = outgoing.begin();

    while (it != outgoing.end()) {
        auto neighbor = it->first;
        auto& edge = it->second;
        double edge_cost = edge.cached_trajectory->cost;

        // A node ONLY evaluates culling if the edge is longer than the current radius
        // AND the neighbor is not the current parent in the shortest-path tree.
        if (edge_cost > (neighborhood_radius_ + std::numeric_limits<double>::epsilon()) && neighbor != v->getParent()) {

            // SYMMETRIC CULL (Neighbor's Side)
            // Remove 'v' from the neighbor's incoming list if it wasn't an 'initial' birth-neighbor.
            auto& incoming = neighbor->incomingEdges();
            if (auto incoming_it = incoming.find(v); incoming_it != incoming.end()) {
                if (!incoming_it->second.is_initial) {
                    incoming.erase(incoming_it);
                }
            }

            // SOURCE CULL (v's Side)
            // Move the neighbor to the passive map if it wasn't an 'initial' birth-neighbor.
            if (!edge.is_initial) {
                v->culled_outgoing_edges_[neighbor] = edge;
                it = outgoing.erase(it);
                // outgoing.erase(it++); /for absl
                continue; // Node is culled; move to next iterator
            }
        }
        // Move to next neighbor if no cull occurred
        ++it;
    }
    v->last_culled_radius_ = neighborhood_radius_;
}




void KinodynamicANYRRTX::verifyQueue(RRTxNode* node) {
    const double min_key = std::min(node->getLMC(), node->getG());
    const double g_value = node->getG();
    

    if (node->in_queue_) {
        // Update both the priority and maintains g_value through node pointer
        inconsistency_queue_.update(node, min_key);
        last_replan_metrics_.queue_operations++;
    } else {
        inconsistency_queue_.add(node, min_key);
        last_replan_metrics_.queue_operations++;
        // node->in_queue_ = true;
    }
}


void KinodynamicANYRRTX::verifyOrphan(RRTxNode* node) {
    if(node->in_queue_==true){
        inconsistency_queue_.remove(node);
        last_replan_metrics_.queue_operations++;
        // node->in_queue_=false;
    }
    Vc_T_.insert(node->getIndex());
}

void KinodynamicANYRRTX::propagateDescendants() {
    std::queue<RRTxNode*> to_process;

    // Propagate descendants through the tree
    for (int idx : Vc_T_) {
        to_process.push(tree_[idx].get());
    }

    while (!to_process.empty()) {
        RRTxNode* current = to_process.front();
        to_process.pop();

        // Propagate to children using successors()
        for (RRTxNode* child : current->getChildren()) {
            int child_idx = child->getIndex();
            if (Vc_T_.count(child_idx)) continue;
            // Vc_T_.insert(child_idx);
            verifyOrphan(child);

            to_process.push(child);
        }
    }


    // if (visualization_ && !Vc_T_.empty()) {
    //     // Create a vector to hold the 2D positions of the orphaned nodes.
    //     std::vector<Eigen::VectorXd> orphan_positions;
    //     orphan_positions.reserve(Vc_T_.size());

    //     // Iterate through the indices of the nodes in the Vc_T_ set.
    //     for (int node_index : Vc_T_) {
    //         // Get the full state of the node from the tree.
    //         const Eigen::VectorXd& state = tree_.at(node_index)->getStateValue();
    //         // Extract the 2D spatial part (x, y) for visualization.
    //         orphan_positions.push_back(state.head<2>());
    //     }

    //     // Call the visualization function to draw these nodes in RViz.
    //     // We use a bright red color and a unique namespace to distinguish them.
    //     visualization_->visualizeNodes(orphan_positions, "map",
    //                                  {1.0f, 0.0f, 0.0f},  // Red color
    //                                  "orphaned_nodes");
    // }




    // Invalidate costs for neighbors of affected nodes
    for (int idx : Vc_T_) {
        auto node = tree_[idx].get();

        // Process outgoing neighbors (N⁺(v) \ Vc_T)
        for (const auto& [neighbor, edge] : node->outgoingEdges()) {
            // Skip invalid edges
            int neighbor_idx = neighbor->getIndex();
            if (Vc_T_.count(neighbor_idx)) continue;
            // neighbor->setG(std::numeric_limits<double>::infinity());
            // verifyQueue(neighbor);
            if (neighbor->getG() != std::numeric_limits<double>::infinity()) {
                neighbor->setG(std::numeric_limits<double>::infinity());
                verifyQueue(neighbor);
            }
        }

        // Process parent (p⁺_T(v) \ Vc_T)
        if (RRTxNode* parent = node->getParent()) {
            // Find the edge from parent to node
            const auto& parent_edges = parent->outgoingEdges();
            auto it = parent_edges.find(node);

            // Validate the edge if it exists
            if (it != parent_edges.end() ) {
                int parent_idx = parent->getIndex();
                // if (!Vc_T_.count(parent_idx)) {
                //     parent->setG(std::numeric_limits<double>::infinity());
                //     verifyQueue(parent);
                // }
                if (!Vc_T_.count(parent_idx) && parent->getG() != std::numeric_limits<double>::infinity()) {
                    parent->setG(std::numeric_limits<double>::infinity());
                    verifyQueue(parent);
                }
            }
        }
    }

    // Reset orphaned nodes using new parent API
    for (int idx : Vc_T_) {
        auto node = tree_[idx].get();
        node->setG(std::numeric_limits<double>::infinity());
        node->setLMC(std::numeric_limits<double>::infinity());
        last_replan_metrics_.nodes_updated++;
        /*
            An orphaned node doesn't move in time; it just has no path to the goal. 
            By keeping its time_to_goal at the original sampled value, the collision checker can still accurately check if it is blocked or clear
        */
        node->setParent(nullptr, std::shared_ptr<Trajectory>{});
    }

    last_replan_metrics_.orphaned_nodes = Vc_T_.size();

    // std::cout<<"orphans size"<<Vc_T_.size()<<"\n";

    Vc_T_.clear();
}





void KinodynamicANYRRTX::visualizeTree() {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    if (!tree_.empty()) {
        edges.reserve(tree_.size());
    }
    
    std::vector<Eigen::VectorXd> tree_nodes;
    tree_nodes.reserve(tree_.size());

    // --- Variables for statistics ---
    long long total_forward_neighbors = 0;
    size_t max_forward_neighbors = 0;
    int connected_nodes_count = 0;
    
    for (const auto& node_ptr : tree_) {
        RRTxNode* child_node = node_ptr.get();
        RRTxNode* parent_node = child_node->getParent();

        tree_nodes.push_back(node_ptr->getStateValue().head(2)); // TODO: For min snap it needs to be 3!!! I need spatial dim variable!

        if (child_node->getG() != std::numeric_limits<double>::infinity()) {
            connected_nodes_count++;
            size_t current_neighbors = child_node->outgoingEdges().size();
            total_forward_neighbors += current_neighbors;
            if (current_neighbors > max_forward_neighbors) {
                max_forward_neighbors = current_neighbors;
            }
        }

        if (parent_node) {
            edges.emplace_back(parent_node->getStateValue().head(2), child_node->getStateValue().head(2));
        }
    }

    double average_neighbors = (connected_nodes_count > 0) 
                             ? static_cast<double>(total_forward_neighbors) / connected_nodes_count 
                             : 0.0;
    
    // std::cout << "[FMTX INFO] Total nodes: " << tree_.size()
    //           << " | Connected: " << connected_nodes_count
    //           << " | Max Neighbors: " << max_forward_neighbors
    //           << " | Avg Neighbors: " << std::fixed << std::setprecision(2) << average_neighbors << std::endl;
    
    // visualization_->visualizeNodes(tree_nodes, "map", 
    //                         std::vector<float>{0.0f, 1.0f, 0.0f},  // Green color
    //                         "tree_nodes");
    
    // std::cout<<"Tree Nodes: "<<tree_nodes.size()<<"\n";
    visualization_->visualizeEdges(edges, "map");
}


void KinodynamicANYRRTX::visualizePath(const std::vector<Eigen::VectorXd>& path_waypoints) {
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
        edges.emplace_back(start_point, end_point);
    }

    // Use your existing visualization class to draw the edges.
    // We'll use a distinct namespace and color (e.g., green and thick) to see it clearly.
    if (visualization_) {
        // The last argument is a namespace to keep it separate from the main tree visualization.
        visualization_->visualizeEdges(edges, "map", "0.0,1.0,0.0", "executable_path");
    }
}



void KinodynamicANYRRTX::dumpTreeToCSV(const std::string& filename) const {
    std::ofstream fout(filename);
    if (!fout.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing\n";
        return;
    }
    // Determine the dimension of the states from the first node
    if (tree_.empty()) {
        std::cerr << "Tree is empty. Nothing to dump.\n";
        return;
    }
    size_t dim = tree_[0]->getStateValue().size();

    // Write the CSV header
    fout << "node_id";
    for (size_t d = 0; d < dim; ++d) {
        fout << ",x" << d;
    }
    fout << ",parent_id\n";

    // Iterate through each node and write its data
    for (const auto& node_ptr : tree_) {
        int nid = node_ptr->getIndex(); 
        const auto& coords = node_ptr->getStateValue();
        RRTxNode* parent = node_ptr->getParent();
        int pid = (parent ? parent->getIndex() : -1); // Use -1 for nodes without a parent

        fout << nid;
        for (size_t d = 0; d < dim; ++d) {
            fout << "," << std::setprecision(10) << coords[d];
        }
        fout << "," << pid << "\n";
    }
    fout.close();
    std::cout << "RRTX tree with " << tree_.size() << " nodes dumped to " << filename << "\n";
}



void KinodynamicANYRRTX::setRobotState(const Eigen::VectorXd& robot_state) {
    robot_continuous_state_ = robot_state;
    // Extract actual planner-time (Time-to-Go) from the state (last element)
    double robot_time_to_go = robot_continuous_state_(robot_continuous_state_.size() - 1);

    // --- QUERY POINT CONSTRUCTION ---
    Eigen::VectorXd query_point = Eigen::VectorXd::Zero(kd_dim);
    if (robot_continuous_state_.size() >= 2) {
        query_point(0) = robot_continuous_state_(0);
        query_point(1) = robot_continuous_state_(1);
    }
    if (kd_dim == 3) {
        query_point(2) = robot_time_to_go;
    } else if (kd_dim == 4) {
        query_point(2) = robot_continuous_state_(2); 
        query_point(3) = robot_time_to_go;
    } else if (kd_dim == 5) {
        query_point = robot_continuous_state_; 
    }
    // --- HYSTERESIS LOGIC ---
    const double hysteresis_factor = 0.98;
    double cost_of_current_path = std::numeric_limits<double>::infinity();
    Trajectory bridge;
    if (vbot_node_ && vbot_node_->getG() != std::numeric_limits<double>::infinity()) {
        bridge = statespace_->steer(robot_continuous_state_, vbot_node_->getStateValue());
        // Use robot_time_to_go so collision check is synced with the world
        if (bridge.is_valid) {

            bool safe = true;
            const auto& obstacles = obs_checker_->getObstacles();

            for (const auto& ob : obstacles) {
                last_replan_metrics_.obstacle_checks++;

                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(
                        bridge, ob)) {
                    safe = false;
                    break;
                }
            }

            if (safe) {
                cost_of_current_path = bridge.cost + vbot_node_->getG();
                robot_current_time_to_goal_ = bridge.time_duration + vbot_node_->getTimeToGoal();
                // return;
            }
        }
    }
    RRTxNode* best_candidate_node = nullptr;
    Trajectory best_candidate_bridge;
    double best_candidate_cost = std::numeric_limits<double>::infinity();
    
    // Radius Expansion to handle sparse graphs
    double current_search_radius = neighborhood_radius_; 
    const int max_attempts = 5; 
    const double radius_multiplier = 2.0;
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        auto nearby_indices = kdtree_->radiusSearch(query_point, current_search_radius);
        for (auto idx : nearby_indices) {
            RRTxNode* candidate = tree_[idx].get();
            if (candidate->getG() == std::numeric_limits<double>::infinity()) continue;
            Trajectory bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());
            if (!bridge.is_valid) continue;

            bool safe = true;
            const auto& obstacles = obs_checker_->getObstacles();
            for (const auto& ob : obstacles) {
                last_replan_metrics_.obstacle_checks++;

                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(bridge, ob)) {
                    safe = false;
                    break;  // stop checking remaining obstacles for this bridge
                }
            }

            if (!safe) continue;

            double cost = bridge.cost + candidate->getG();
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
    // --- ASSIGNMENT ---
    if (best_candidate_node && best_candidate_cost < cost_of_current_path * hysteresis_factor) {
        vbot_node_ = best_candidate_node;
        robot_current_time_to_goal_ = best_candidate_bridge.time_duration + best_candidate_node->getTimeToGoal();
        last_replan_metrics_.path_cost = best_candidate_cost;
        current_bridge_trajectory_ = best_candidate_bridge;
    } else if (vbot_node_ && cost_of_current_path != std::numeric_limits<double>::infinity()) {
        Trajectory bridge = statespace_->steer(robot_continuous_state_, vbot_node_->getStateValue());
        robot_current_time_to_goal_ = bridge.time_duration + vbot_node_->getTimeToGoal();
        // The cost changes slightly every frame as the robot moves towards the anchor.
        last_replan_metrics_.path_cost = cost_of_current_path;
        current_bridge_trajectory_ = bridge;
    } 
    else if (vbot_node_&& !bridge.is_valid && current_bridge_trajectory_.is_valid) {
        /*
            When steer() mathematically fails near an anchor, we fallback to recycling 
            the 'current_bridge_trajectory_' from a previous control loop. One might worry 
            that the 2nd or 3rd points of this old array are "stale" and will cause the 
            robot to jump backwards. This will NOT happen due to the time-driven pipeline:
            1. setPath(): Overwrites index [0]'s position/velocity to match the robot's 
            ACTUAL current physical state, preventing any theoretical first-frame jump.
            2. stepSimulation(): Interpolation is strictly driven by 'current_sim_time_'. 
            The std::lower_bound function automatically "fast-forwards" through the array. 
            Because time has passed since the trajectory was cached, std::lower_bound 
            in ros2manager completely ignores the stitched index [0] and any other passed points.
            It skips directly to the exact time-segment where the robot belongs chronologically, 
            smoothly riding the rest of the valid curve into the anchor.
        */
        // Steer failed mathematically, so we try to recycle the previous valid bridge.
        // BUT we must verify it is still safe against current dynamic obstacles!
        
        bool cached_is_safe = true;
        const auto& obstacles = obs_checker_->getObstacles();
        for (const auto& ob : obstacles) {
            last_replan_metrics_.obstacle_checks++;
            if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(current_bridge_trajectory_, ob)) {
                cached_is_safe = false;
                break;
            }
        }

        if (cached_is_safe) {
            robot_current_time_to_goal_ = current_bridge_trajectory_.time_duration + vbot_node_->getTimeToGoal();
            last_replan_metrics_.path_cost = current_bridge_trajectory_.cost + vbot_node_->getG();
            // current_bridge_trajectory_ remains unchanged
        } else {
            // The cached trajectory is blocked by a new dynamic obstacle. We are trapped.
            vbot_node_= nullptr;
            robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
            bridge_cost_ = std::numeric_limits<double>::infinity();
            current_bridge_trajectory_ = Trajectory();
            last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
            RRTX_WARN("Set Robot State: CACHED BRIDGE BLOCKED! LOST SAFE ANCHOR!");
        }
    } 
    else {
        // We are trapped. No nodes in radius are safe.
        vbot_node_ = nullptr;
        robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
        bridge_cost_ = std::numeric_limits<double>::infinity();
        current_bridge_trajectory_ = bridge;
        last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
        RRTX_WARN("[RRTX_Anchor] Status: NULL (Robot is lost or searching...)");
    }

    // INTERNAL DEBUG VISUALIZATION
    if (visualization_) {
        // std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> debug_edges;
        // If we have a valid anchor, recalculate the bridge path for visualization
        if (vbot_node_) {
            // // Recalculate strictly for visualization
            // Trajectory viz_bridge = statespace_->steer(robot_continuous_state_, vbot_node_->getStateValue());
            
            // // Convert points to edges (segments)
            // if (viz_bridge.path_points.size() >= 2) {
            //     for (size_t i = 0; i < viz_bridge.path_points.size() - 1; ++i) {
            //         debug_edges.emplace_back(viz_bridge.path_points[i], viz_bridge.path_points[i+1]);
            //     }
            // } else {
            //     // Fallback: simple straight line from robot to anchor node
                 
            //     debug_edges.emplace_back(robot_continuous_state_, vbot_node_->getStateValue());
            // }
            // // Visualize in CYAN (0, 1, 1) so it stands out from the path (Green) and Tree (Red/Green)
            // // Using a unique namespace "debug_anchor_trajectory" ensures it overwrites the previous frame
            // visualization_->visualizeEdges(debug_edges, "map", "0.0,1.0,1.0", "debug_anchor_trajectory");
            
            // // OPTIONAL: Visualize the anchor node itself as a big dot
            std::vector<Eigen::VectorXd> anchor_pt = { vbot_node_->getStateValue().head<2>() };
            visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");
        } else {
            // // CLEAR THE VISUALIZATION
            // // Sending an empty list to the same namespace effectively deletes the markers
            // visualization_->visualizeEdges({}, "map", "0.0,0.0,0.0", "debug_anchor_trajectory");
            // visualization_->visualizeNodes({}, "map", {0.0f, 0.0f, 0.0f}, "debug_anchor_point");
        }
    }
}



bool KinodynamicANYRRTX::isRobotSafe() {
    // If vbot_node_ is null, we have no anchor.
    // If cost is std::numeric_limits<double>::infinity(), the anchor is invalid (trapped).
    return (vbot_node_ != nullptr) && (vbot_node_->getG() != std::numeric_limits<double>::infinity());
}

//////////////////////////////////////EVENT BASED!!!////////////////////////////////////////
void KinodynamicANYRRTX::updateObstacleSamples(const ObstacleVector& turned_obstacles) {
    if (turned_obstacles.empty()) return;

    // last_replan_metrics_ = ReplanMetrics();
    
    // Get the exact planning time for synchronization
    double T_robot = 0.0;
    if (!is_geometric_mode_) {
        // Only extract T_robot if we are in kinodynamic mode
        if (robot_continuous_state_.size() > 0) {
            T_robot = robot_continuous_state_(robot_continuous_state_.size() - 1);
        }
    }



    for (const auto& incoming_ob : turned_obstacles) {
        
        // Retrieve the stored obstacle (or create a blank one)
        Obstacle& stored_ob = previous_obstacles_[incoming_ob.name];
        
        // REMOVE OLD TUBE
        // We must do this BEFORE overwriting stored_ob, because we need the OLD path to find nodes to repair.
        if (!stored_ob.predicted_path.empty()) {
            removeObstacle(stored_ob); 
        }

        // UPDATE STATE 
        stored_ob = incoming_ob; 

        // GENERATE NEW DENSE TUBE
        // We regenerate this here to ensure it uses the Planner's exact 'T_robot'.
        // Since we copied the object in step above, 'stored_ob' now has the valid physics data to do this.
        stored_ob.predicted_path = obs_checker_->generatePrediction(stored_ob, T_robot);

        // --- DEBUG LOGGING ---
        /*
        std::cout << "\n=== DEBUG TUBE FOR " << stored_ob.name << " ===" << std::endl;
        for (size_t i = 0; i < stored_ob.predicted_path.size(); i += 5) {
            const auto& pt = stored_ob.predicted_path[i];
            printf("[%zu] X:%.2f  Y:%.2f  T:%.2f\n", i, pt.x(), pt.y(), pt.z());
        }
        std::cout << "========================================\n" << std::endl;
        */

        // ADD NEW TUBE
        // Now that the path is populated, this will successfully invalidate blocked nodes.
        addNewObstacle(stored_ob);
        
        // RCLCPP_INFO(rclcpp::get_logger("RRTx"), 
        //     "Obstacle [%s]: Removed %zu candidates, Added %zu candidates. Tube Size: %zu", 
        //     stored_ob.name.c_str(), debug_repaired_nodes.size(), 
        //     debug_invalidated_nodes.size(), stored_ob.predicted_path.size());
    }

    // PROPAGATE CHANGES
    propagateDescendants();
    if (vbot_node_) verifyQueue(vbot_node_);
    reduceInconsistency();
}




// STRATEGY 1: INVALIDATING SET (Edge-Level Caching)
#if USE_INVALIDATING_SET_STRATEGY

void KinodynamicANYRRTX::addNewObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    double search_radius;
    if (is_geometric_mode_) {
        search_radius = obs_r + ob.inflation + delta;
    } else {
        double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
        search_radius = obs_r + ob.inflation + delta + gap_coverage_inflation;
    }

    std::unordered_set<int> unique_node_indices;
    for (const auto& point_3d : ob.predicted_path) {
        Eigen::VectorXd query(kd_dim);
        if (kd_dim == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
        else if (kd_dim == 2) query << point_3d.x(), point_3d.y();
        else if (kd_dim == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
        else if (kd_dim == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
    }

    auto checkAndBlockEdge = [&](RRTxNode* node, RRTxNode* neighbor, EdgeInfo& edge) {
        last_replan_metrics_.obstacle_checks++;

        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ob)) {
            edge.distance = std::numeric_limits<double>::infinity();
            if (std::find(edge.invalidating_obstacles.begin(), edge.invalidating_obstacles.end(), &ob) == edge.invalidating_obstacles.end()) {
                edge.invalidating_obstacles.push_back(&ob);
            }
            if (neighbor->incomingEdges().count(node)) {
                auto& inc_edge = neighbor->incomingEdges().at(node);
                inc_edge.distance = std::numeric_limits<double>::infinity();
                if (std::find(inc_edge.invalidating_obstacles.begin(), inc_edge.invalidating_obstacles.end(), &ob) == inc_edge.invalidating_obstacles.end()) {
                    inc_edge.invalidating_obstacles.push_back(&ob);
                }
            }
            
            // if (neighbor->getParent() == node) verifyOrphan(neighbor);
            if (node->getParent() == neighbor) verifyOrphan(node);

            if (is_geometric_mode_ && neighbor->outgoingEdges().count(node)) {
                auto& rev_edge = neighbor->outgoingEdges().at(node);
                if (rev_edge.distance != std::numeric_limits<double>::infinity()) {
                    rev_edge.distance = std::numeric_limits<double>::infinity();

                    if (node->incomingEdges().count(neighbor)) {
                        node->incomingEdges().at(neighbor).distance =
                            std::numeric_limits<double>::infinity();
                    }
                    // if (node->getParent() == neighbor) verifyOrphan(node);
                    if (neighbor->getParent() == node) verifyOrphan(neighbor);
                }
            }
        }
    };

    for (int idx : unique_node_indices) {
        RRTxNode* node = tree_[idx].get();
        for (auto& [neighbor, edge] : node->outgoingEdges()) checkAndBlockEdge(node, neighbor, edge);
        for (auto& [neighbor, edge] : node->culled_outgoing_edges_) checkAndBlockEdge(node, neighbor, edge);
    }
}

void KinodynamicANYRRTX::removeObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    double search_radius;
    if (is_geometric_mode_) {
        search_radius = obs_r + ob.inflation + delta;
    } else {
        double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
        search_radius = obs_r + ob.inflation + delta + gap_coverage_inflation;
    }

    std::unordered_set<int> unique_node_indices;
    for (const auto& point_3d : ob.predicted_path) {
        Eigen::VectorXd query(kd_dim);
        if (kd_dim == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
        else if (kd_dim == 2) query << point_3d.x(), point_3d.y();
        else if (kd_dim == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
        else if (kd_dim == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
    }
    auto checkAndRestoreEdge = [&](RRTxNode* node, RRTxNode* neighbor, EdgeInfo& edge, bool& neighborsWereBlocked) {
        if (edge.distance == std::numeric_limits<double>::infinity()) {
            
            // SWAP-AND-POP for the outgoing edge
            auto it = std::find(edge.invalidating_obstacles.begin(), edge.invalidating_obstacles.end(), &ob);
            if (it != edge.invalidating_obstacles.end()) {
                *it = edge.invalidating_obstacles.back();
                edge.invalidating_obstacles.pop_back();
                
                // If the edge has zero blocking obstacles left, it is fully restored
                if (edge.invalidating_obstacles.empty()) {
                    edge.distance = edge.distance_original;
                    
                    if (neighbor->incomingEdges().count(node)) {
                        auto& inc_edge = neighbor->incomingEdges().at(node);
                        inc_edge.distance = edge.distance_original;
                        
                        // Clean up the incoming edge as well
                        auto inc_it = std::find(inc_edge.invalidating_obstacles.begin(), inc_edge.invalidating_obstacles.end(), &ob);
                        if (inc_it != inc_edge.invalidating_obstacles.end()) {
                            *inc_it = inc_edge.invalidating_obstacles.back();
                            inc_edge.invalidating_obstacles.pop_back();
                        }

                    }
                    neighborsWereBlocked = true;
                    if (is_geometric_mode_ && neighbor->outgoingEdges().count(node)) {
                        auto& rev_edge = neighbor->outgoingEdges().at(node);

                        if (rev_edge.distance == std::numeric_limits<double>::infinity()) {

                            rev_edge.distance = rev_edge.distance_original;

                            if (node->incomingEdges().count(neighbor)) {
                                node->incomingEdges().at(neighbor).distance =
                                    rev_edge.distance_original;
                            }
                        }
                    }
                }
            }
        }
    };


    for (int idx : unique_node_indices) {
        RRTxNode* node = tree_[idx].get();
        bool neighborsWereBlocked = false;
        
        for (auto& [neighbor, edge] : node->outgoingEdges()) checkAndRestoreEdge(node, neighbor, edge, neighborsWereBlocked);
        for (auto& [neighbor, edge] : node->culled_outgoing_edges_) checkAndRestoreEdge(node, neighbor, edge, neighborsWereBlocked);
        
        if (neighborsWereBlocked) {
            updateLMC(node);
            if (node->getG() != node->getLMC()) verifyQueue(node);
        }
    }
}

// STRATEGY 2: THREAT SET (Node-Level Filtering)
#elif USE_THREAT_SET_STRATEGY

void KinodynamicANYRRTX::addNewObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    double search_radius;
    if (is_geometric_mode_) {
        search_radius = obs_r + ob.inflation + delta;
    } else {
        double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
        search_radius = obs_r + ob.inflation + delta + gap_coverage_inflation;
    }

    std::unordered_set<int> unique_node_indices;
    for (const auto& point_3d : ob.predicted_path) {
        Eigen::VectorXd query(kd_dim);
        if (kd_dim == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
        else if (kd_dim == 2) query << point_3d.x(), point_3d.y();
        else if (kd_dim == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
        else if (kd_dim == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
    }

    auto checkAndBlockEdge = [&](RRTxNode* node, RRTxNode* neighbor, EdgeInfo& edge) {
        if (edge.distance == std::numeric_limits<double>::infinity()) return; 

        last_replan_metrics_.obstacle_checks++;

        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ob)) {
            edge.distance = std::numeric_limits<double>::infinity();
            
            if (neighbor->incomingEdges().count(node)) {
                neighbor->incomingEdges().at(node).distance = std::numeric_limits<double>::infinity();
            }
            
            // if (neighbor->getParent() == node) verifyOrphan(neighbor);
            if (node->getParent() == neighbor) verifyOrphan(node);

            if (is_geometric_mode_ && neighbor->outgoingEdges().count(node)) {
                auto& rev_edge = neighbor->outgoingEdges().at(node);
                if (rev_edge.distance != std::numeric_limits<double>::infinity()) {
                    rev_edge.distance = std::numeric_limits<double>::infinity();

                    if (node->incomingEdges().count(neighbor)) {
                        node->incomingEdges().at(neighbor).distance =
                            std::numeric_limits<double>::infinity();
                    }
                    // if (node->getParent() == neighbor) verifyOrphan(node);
                    if (neighbor->getParent() == node) verifyOrphan(neighbor);
                }
            }
        }
    };

    for (int idx : unique_node_indices) {
        RRTxNode* node = tree_[idx].get();
        if (std::find(node->threats_.begin(), node->threats_.end(), &ob) == node->threats_.end()) {
            node->threats_.push_back(&ob);
        }
        
        for (auto& [neighbor, edge] : node->outgoingEdges()) checkAndBlockEdge(node, neighbor, edge);
        for (auto& [neighbor, edge] : node->culled_outgoing_edges_) checkAndBlockEdge(node, neighbor, edge);
    }
}

void KinodynamicANYRRTX::removeObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    double search_radius;
    if (is_geometric_mode_) {
        search_radius = obs_r + ob.inflation + delta;
    } else {
        double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
        search_radius = obs_r + ob.inflation + delta + gap_coverage_inflation;
    }

    std::unordered_set<int> unique_node_indices;
    for (const auto& point_3d : ob.predicted_path) {
        Eigen::VectorXd query(kd_dim);
        if (kd_dim == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
        else if (kd_dim == 2) query << point_3d.x(), point_3d.y();
        else if (kd_dim == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
        else if (kd_dim == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
    }
    auto checkAndRestoreEdge = [&](RRTxNode* node, RRTxNode* neighbor, EdgeInfo& edge, bool& neighborsWereBlocked) {
        if (edge.distance == std::numeric_limits<double>::infinity()) {
            bool is_safe = true;
            
            // Iteration through contiguous pointers
            for (const Obstacle* threat_ptr : node->threats_) {
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), *threat_ptr)) {
                    is_safe = false;
                    break;
                }
            }

            if (is_safe) {
                edge.distance = edge.distance_original;
                if (neighbor->incomingEdges().count(node)) {
                    neighbor->incomingEdges().at(node).distance = edge.distance_original;
                }
                neighborsWereBlocked = true;

                if (is_geometric_mode_ && neighbor->outgoingEdges().count(node)) {
                    auto& rev_edge = neighbor->outgoingEdges().at(node);

                    if (rev_edge.distance == std::numeric_limits<double>::infinity()) {

                        rev_edge.distance = rev_edge.distance_original;

                        if (node->incomingEdges().count(neighbor)) {
                            node->incomingEdges().at(neighbor).distance =
                                rev_edge.distance_original;
                        }
                    }
                }
            }
        }
    };

    for (int idx : unique_node_indices) {
        RRTxNode* node = tree_[idx].get();

        // SWAP-AND-POP Node Threat Removal
        auto it = std::find(node->threats_.begin(), node->threats_.end(), &ob);
        if (it != node->threats_.end()) {
            *it = node->threats_.back();
            node->threats_.pop_back();
        }
        bool neighborsWereBlocked = false;
        for (auto& [neighbor, edge] : node->outgoingEdges()) checkAndRestoreEdge(node, neighbor, edge, neighborsWereBlocked);
        for (auto& [neighbor, edge] : node->culled_outgoing_edges_) checkAndRestoreEdge(node, neighbor, edge, neighborsWereBlocked);
        if (neighborsWereBlocked) {
            updateLMC(node);
            if (node->getG() != node->getLMC()) verifyQueue(node);
        }
    }
}

// STRATEGY 3: DEFAULT (Brute-Force)
#else

void KinodynamicANYRRTX::addNewObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    double search_radius;
    if (is_geometric_mode_) {
        search_radius = obs_r + ob.inflation + delta;
    } else {
        double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
        search_radius = obs_r + ob.inflation + delta + gap_coverage_inflation;
    }

    std::unordered_set<int> unique_node_indices;
    for (const auto& point_3d : ob.predicted_path) {
        Eigen::VectorXd query(kd_dim);
        if (kd_dim == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
        else if (kd_dim == 2) query << point_3d.x(), point_3d.y();
        else if (kd_dim == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
        else if (kd_dim == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
    }

    auto checkAndBlockEdge = [&](RRTxNode* node, RRTxNode* neighbor, EdgeInfo& edge) {
        if (edge.distance == std::numeric_limits<double>::infinity()) return; 

        last_replan_metrics_.obstacle_checks++;

        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ob)) {
            edge.distance = std::numeric_limits<double>::infinity();
            
            if (neighbor->incomingEdges().count(node)) {
                neighbor->incomingEdges().at(node).distance = std::numeric_limits<double>::infinity();
            }
            
            // if (neighbor->getParent() == node) verifyOrphan(neighbor);
            if (node->getParent() == neighbor) verifyOrphan(node);

            if (is_geometric_mode_ && neighbor->outgoingEdges().count(node)) {
                auto& rev_edge = neighbor->outgoingEdges().at(node);
                if (rev_edge.distance != std::numeric_limits<double>::infinity()) {
                    rev_edge.distance = std::numeric_limits<double>::infinity();

                    if (node->incomingEdges().count(neighbor)) {
                        node->incomingEdges().at(neighbor).distance =
                            std::numeric_limits<double>::infinity();
                    }
                    // if (node->getParent() == neighbor) verifyOrphan(node);
                    if (neighbor->getParent() == node) verifyOrphan(neighbor);
                }
            }
        }
    };

    for (int idx : unique_node_indices) {
        RRTxNode* node = tree_[idx].get();
        for (auto& [neighbor, edge] : node->outgoingEdges()) checkAndBlockEdge(node, neighbor, edge);
        for (auto& [neighbor, edge] : node->culled_outgoing_edges_) checkAndBlockEdge(node, neighbor, edge);
    }
}

void KinodynamicANYRRTX::removeObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    double search_radius;
    if (is_geometric_mode_) {
        search_radius = obs_r + ob.inflation + delta;
    } else {
        double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
        search_radius = obs_r + ob.inflation + delta + gap_coverage_inflation;
    }

    std::unordered_set<int> unique_node_indices;
    for (const auto& point_3d : ob.predicted_path) {
        Eigen::VectorXd query(kd_dim);
        if (kd_dim == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
        else if (kd_dim == 2) query << point_3d.x(), point_3d.y();
        else if (kd_dim == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
        else if (kd_dim == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
    }

    const ObstacleVector& all_obstacles = obs_checker_->getObstacles();

    auto checkAndRestoreEdge = [&](RRTxNode* node, RRTxNode* neighbor, EdgeInfo& edge, bool& neighborsWereBlocked) {
        if (edge.distance == std::numeric_limits<double>::infinity()) {
            bool should_restore = false;
            last_replan_metrics_.obstacle_checks++;
            if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ob)) {
                bool conflicts_with_other = false;
                for (const auto& other_ob : all_obstacles) {
                    if (other_ob.name == ob.name) continue; 
                    last_replan_metrics_.obstacle_checks++;
                    if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), other_ob)) {
                        conflicts_with_other = true;
                        break; 
                    }
                }
                if (!conflicts_with_other) {
                    should_restore = true;
                }
            }

            if (should_restore) {
                edge.distance = edge.distance_original;
                if (neighbor->incomingEdges().count(node)) {
                    neighbor->incomingEdges().at(node).distance = edge.distance_original;
                }
                neighborsWereBlocked = true;

                if (is_geometric_mode_ && neighbor->outgoingEdges().count(node)) {
                    auto& rev_edge = neighbor->outgoingEdges().at(node);

                    if (rev_edge.distance == std::numeric_limits<double>::infinity()) {

                        rev_edge.distance = rev_edge.distance_original;

                        if (node->incomingEdges().count(neighbor)) {
                            node->incomingEdges().at(neighbor).distance =
                                rev_edge.distance_original;
                        }
                    }
                }

            }
        }
    };

    for (int idx : unique_node_indices) {
        RRTxNode* node = tree_[idx].get();
        bool neighborsWereBlocked = false;
        
        for (auto& [neighbor, edge] : node->outgoingEdges()) checkAndRestoreEdge(node, neighbor, edge, neighborsWereBlocked);
        for (auto& [neighbor, edge] : node->culled_outgoing_edges_) checkAndRestoreEdge(node, neighbor, edge, neighborsWereBlocked);
        
        if (neighborsWereBlocked) {
            updateLMC(node);
            if (node->getG() != node->getLMC()) verifyQueue(node);
        }
    }
}

#endif


// // STRATEGY 3: DEFAULT (Brute-Force)
// #else
// void KinodynamicANYRRTX::addNewObstacle(const Obstacle& ob) {
//     if (ob.predicted_path.empty()) return;
    
//     // Log entry
//     std::cout << "[DEBUG] addNewObstacle called for: " << ob.name << std::endl;

//     double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius :
//                    std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
//     double search_radius;
//     if (is_geometric_mode_) {
//         search_radius = obs_r + ob.inflation + delta;
//     } else {
//         double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0);
//         search_radius = obs_r + ob.inflation + delta + gap_coverage_inflation;
//     }

//     std::unordered_set<int> unique_node_indices;
//     for (const auto& point_3d : ob.predicted_path) {
//         Eigen::VectorXd query(kd_dim);
//         if (kd_dim == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
//         else if (kd_dim == 2) query << point_3d.x(), point_3d.y();
//         else if (kd_dim == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
//         else if (kd_dim == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
//         std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
//         for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
//     }

// // Debug: check specific indices presence
// int child_idx = 328;
// int parent_idx = 259;
// bool child_found = (unique_node_indices.count(child_idx) > 0);
// bool parent_found = (unique_node_indices.count(parent_idx) > 0);
// std::cout << "[DBG] unique_node_indices contains child " << child_idx << "? " << (child_found ? "YES" : "NO") 
//           << " parent " << parent_idx << "? " << (parent_found ? "YES" : "NO") << "\n";


//     std::cout << "[DEBUG] Obstacle " << ob.name << " affects " << unique_node_indices.size() << " nodes." << std::endl;

//     // auto checkAndBlockEdge = [&](RRTxNode* node, RRTxNode* neighbor, EdgeInfo& edge) {
//     //     if (edge.distance == std::numeric_limits<double>::infinity()) return;

//     //     last_replan_metrics_.obstacle_checks++;
        
//     //     bool is_safe = obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ob);

//     //     // --- FORENSIC LOGGING FOR THE PROBLEMATIC EDGE ---
//     //     // Based on logs, the violation is often Node 233 -> Parent 359 or 63
//     //     if ((node->getIndex() == 233 && (neighbor->getIndex() == 359 || neighbor->getIndex() == 63)) ||
//     //         (neighbor->getIndex() == 233 && (node->getIndex() == 359 || node->getIndex() == 63))) {
            
//     //         std::cout << "\n[FORENSIC] Checking Edge " << node->getIndex() << " -> " << neighbor->getIndex() 
//     //                   << " against Obstacle [" << ob.name << "]" << std::endl;
//     //         std::cout << "   Edge Start TTG: " << edge_start_ttg << std::endl;
//     //         std::cout << "   Cached Trajectory Valid: " << (edge.cached_trajectory ? "Yes" : "NULL") << std::endl;
//     //         if (edge.cached_trajectory) {
//     //             std::cout << "   Cached Traj Cost: " << edge.cached_trajectory->cost << std::endl;
//     //         }
//     //         std::cout << "   Collision Check Result: " << (is_safe ? "SAFE" : "COLLISION") << std::endl;
            
//     //         if (!is_safe) {
//     //             std::cout << "   >>> ACTION: Blocking edge (Setting dist to std::numeric_limits<double>::infinity()) <<<" << std::endl;
//     //         }
//     //     }
//     //     // --------------------------------------------------

//     //     if (!is_safe) {
//     //         edge.distance = std::numeric_limits<double>::infinity();
//     //         if (neighbor->incomingEdges().count(node)) {
//     //             neighbor->incomingEdges().at(node).distance = std::numeric_limits<double>::infinity();
//     //         }
//     //         if (neighbor->getParent() == node) verifyOrphan(neighbor);
//     //         if (node->getParent() == neighbor) verifyOrphan(node);
//     //     }
//     // };

// auto checkAndBlockEdge = [&](RRTxNode* node, RRTxNode* neighbor, EdgeInfo& edge) {
//     // fast path
//     if (edge.distance == std::numeric_limits<double>::infinity()) return;

//     const double edge_start_ttg = node->getTimeToGoal();
//     last_replan_metrics_.obstacle_checks++;

//     // FORENSIC: print everything for the pair of interest (or print for all if you want)
//     bool is_interesting = ( (node->getIndex() == 328 && neighbor->getIndex() == 259)
//                          || (node->getIndex() == 259 && neighbor->getIndex() == 328) );

//     if (is_interesting) {
//         std::cout << "[DBG_EDGE] checking edge " << node->getIndex() << " -> " << neighbor->getIndex()
//                   << " prior_dist=" << edge.distance << " node_ttg=" << edge_start_ttg << "\n";
//         std::cout << "         edge.cached_trajectory? " << (edge.cached_trajectory ? "YES":"NO");
//         if (edge.cached_trajectory) {
//             std::cout << " cost=" << edge.cached_trajectory->cost
//                       << " pts=" << edge.cached_trajectory->path_points.size()
//                       << " first_pt_t=" << (edge.cached_trajectory->path_points.front())( (int)edge.cached_trajectory->path_points.front().size() - 1 )
//                       << " last_pt_t="  << (edge.cached_trajectory->path_points.back())( (int)edge.cached_trajectory->path_points.back().size() - 1 );
//         }
//         std::cout << "\n";

//         // If you also store obstacle bounds in Obstacle, print them:
//         std::cout << "         obstacle bbox x[" << ob.min_x << "," << ob.max_x << "] y[" << ob.min_y << "," << ob.max_y << "]\n";
//     }
//     if (is_interesting) {
//     std::cout << "[DBG_EDGE_PATH] addNewObstacle vs " << ob.name 
//               << " path_size=" << ob.predicted_path.size()
//               << " first_t=" << (ob.predicted_path.empty() ? 0 : ob.predicted_path.front().z())
//               << " last_t=" << (ob.predicted_path.empty() ? 0 : ob.predicted_path.back().z())
//               << " bbox y[" << ob.min_y << "," << ob.max_y << "]\n";
//     }

//     bool is_safe = true;
//     if (!edge.cached_trajectory) {
//         // defensive: recompute a fresh steer (this matches your forensic re-steer)
//         Trajectory fresh = statespace_->steer(node->getStateValue(), neighbor->getStateValue());
//         if (!fresh.is_valid) {
//             is_safe = false; // or treat invalid as unsafe
//         } else {
//             is_safe = obs_checker_->isTrajectorySafeAgainstSingleObstacle(fresh, ob);
//             if (is_interesting) {
//                 std::cout << "[DBG_EDGE] fresh steer computed valid=" << fresh.is_valid << " cost=" << fresh.cost
//                           << " fresh_safe=" << (is_safe ? "YES":"NO") << "\n";
//             }
//         }
//     } else {
//         is_safe = obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ob);
//     }

//     if (is_interesting) {
//         std::cout << "[DBG_EDGE] checker returned safe=" << (is_safe ? "YES":"NO") << " for edge "
//                   << node->getIndex() << "->" << neighbor->getIndex() << "\n";
//     }

//     if (!is_safe) {
//         // block the edge
//         edge.distance = std::numeric_limits<double>::infinity();



//         if (neighbor->incomingEdges().count(node)) {
//             neighbor->incomingEdges().at(node).distance = std::numeric_limits<double>::infinity();
//         } else {
//             if (is_interesting) {
//                 std::cout << "[DBG_EDGE] WARNING: neighbor->incomingEdges() does NOT contain node pointer!\n";
//             }
//         }
//         // // after you set edge.distance = std::numeric_limits<double>::infinity();
//         // if (node->outgoingEdges().count(neighbor)) {
//         //     std::cout << "[DBG_VERIFY_IMMEDIATE] node->outgoingEdges().at(neighbor).distance = "
//         //             << node->outgoingEdges().at(neighbor).distance << "\n";
//         // } else {
//         //     std::cout << "[DBG_VERIFY_IMMEDIATE] WARNING: node->outgoingEdges() does not contain neighbor pointer!\n";
//         // }
//         // if (neighbor->incomingEdges().count(node)) {
//         //     std::cout << "[DBG_VERIFY_IMMEDIATE] neighbor->incomingEdges().at(node).distance = "
//         //             << neighbor->incomingEdges().at(node).distance << "\n";
//         // } else {
//         //     std::cout << "[DBG_VERIFY_IMMEDIATE] neighbor->incomingEdges() does NOT contain node pointer!\n";
//         // }
//         // if this was tree parent, orphan notifications
//         if (neighbor->getParent() == node) verifyOrphan(neighbor);
//         if (node->getParent() == neighbor) verifyOrphan(node);

//         if (is_interesting) {
//             std::cout << "[DBG_EDGE] BLOCKED edge " << node->getIndex() << "->" << neighbor->getIndex() << "\n";
//         }
//     }
// };


//     for (int idx : unique_node_indices) {
//         RRTxNode* node = tree_[idx].get();
//         for (auto& [neighbor, edge] : node->outgoingEdges()) checkAndBlockEdge(node, neighbor, edge);
//         for (auto& [neighbor, edge] : node->culled_outgoing_edges_) checkAndBlockEdge(node, neighbor, edge);
//     }


// // right after processing unique_node_indices (end of addNewObstacle)
// for (int idx : unique_node_indices) {
//     RRTxNode* n = tree_[idx].get();
//     RRTxNode* p = n->getParent();
//     if (!p) continue;
//     auto it = n->outgoingEdges().find(p);
//     if (it == n->outgoingEdges().end()) {
//         std::cout << "[DBG_POST_ADD] Node " << n->getIndex() << " -> parent " << p->getIndex()
//                   << " : NOT FOUND in outgoingEdges()\n";
//     } else {
//         std::cout << "[DBG_POST_ADD] Node " << n->getIndex() << " -> parent " << p->getIndex()
//                   << " stored dist=" << it->second.distance << " original=" << it->second.distance_original << "\n";
//     }
// }

// }
// void KinodynamicANYRRTX::removeObstacle(const Obstacle& ob) {
//     if (ob.predicted_path.empty()) return;
//     // Log entry
//     std::cout << "[DEBUG] removeObstacle called for: " << ob.name << std::endl;
//     double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius :
//                    std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
//     double search_radius;
//     if (is_geometric_mode_) {
//         search_radius = obs_r + ob.inflation + delta;
//     } else {
//         double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0);
//         search_radius = obs_r + ob.inflation + delta + gap_coverage_inflation;
//     }
//     std::unordered_set<int> unique_node_indices;
//     for (const auto& point_3d : ob.predicted_path) {
//         Eigen::VectorXd query(kd_dim);
//         if (kd_dim == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
//         else if (kd_dim == 2) query << point_3d.x(), point_3d.y();
//         else if (kd_dim == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
//         else if (kd_dim == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
//         std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
//         for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
//     }
//     const ObstacleVector& all_obstacles = obs_checker_->getObstacles();
   

// auto checkAndRestoreEdge = [&](RRTxNode* node, RRTxNode* neighbor, EdgeInfo& edge, bool& neighborsWereBlocked) {
//         if (edge.distance == std::numeric_limits<double>::infinity()) {
//             bool should_restore = false;
            
//             bool is_target_edge = (node->getIndex() == 328 && neighbor->getIndex() == 259);
            
//             if (is_target_edge) {
//                 std::cout << "\n[FORENSIC] Attempting Restore Edge " << node->getIndex() 
//                           << " -> " << neighbor->getIndex() 
//                           << " (Removing Obstacle [" << ob.name << "])" << std::endl;
//                 std::cout << "   Edge Start TTG: " << ttg << std::endl;
//                 std::cout << "   Total obstacles in check list: " << all_obstacles.size() << std::endl;
//             }
            
//             // Check if the edge is safe against the REMOVED obstacle
//             if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ob)) {
//                 if (is_target_edge) {
//                     std::cout << "   >>> WARNING: Edge still collides with [" << ob.name << "] despite removal! <<<" << std::endl;
//                 }
                
//                 bool conflicts_with_other = false;
//                 bool found_moving_box_10 = false;

//                 for (const auto& other_ob : all_obstacles) {
//                     if (other_ob.name == ob.name) continue;
                    
//                     if (is_target_edge && other_ob.name == "moving_box_10") {
//                         found_moving_box_10 = true;
//                     }

//                     last_replan_metrics_.obstacle_checks++;
//                     if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), other_ob)) {
//                         conflicts_with_other = true;
//                         if (is_target_edge) {
//                             std::cout << "   >>> Conflict found with: " << other_ob.name << std::endl;
//                             std::cout << "[RESTORE_PATH] CONFLICT with " << other_ob.name 
//                                       << " during remove of " << ob.name
//                                       << " path_size=" << other_ob.predicted_path.size()
//                                       << " first_t=" << (other_ob.predicted_path.empty() ? 0 : other_ob.predicted_path.front().z())
//                                       << " last_t="  << (other_ob.predicted_path.empty() ? 0 : other_ob.predicted_path.back().z())
//                                       << "\n";
//                         }
//                         break;
//                     } 
//                     else if (is_target_edge && other_ob.name == "moving_box_10") {
//                         std::cout << "[RESTORE_PATH] NO CONFLICT with moving_box_10 (this is the bug!) "
//                                   << "during remove of " << ob.name
//                                   << " path_size=" << other_ob.predicted_path.size()
//                                   << " first_t=" << (other_ob.predicted_path.empty() ? 0 : other_ob.predicted_path.front().z())
//                                   << " last_t="  << (other_ob.predicted_path.empty() ? 0 : other_ob.predicted_path.back().z())
//                                   << "\n";
//                     }
//                 }

//                 if (is_target_edge && !found_moving_box_10) {
//                     std::cout << "\033[1;31m   [CRITICAL BUG] 'moving_box_10' was NOT found in all_obstacles list!\033[0m" << std::endl;
//                 }

//                 if (!conflicts_with_other) {
//                     should_restore = true;
//                     if (is_target_edge) std::cout << "   >>> ACTION: Restoring edge (No other conflicts found) <<<" << std::endl;
//                 }
//             } 
//             else {
//                 // It is safe against the removed obstacle. Check others.
//                 bool conflicts_with_other = false;
//                 bool found_moving_box_10 = false;

//                 for (const auto& other_ob : all_obstacles) {
//                     if (other_ob.name == ob.name) continue;

//                     if (is_target_edge && other_ob.name == "moving_box_10") {
//                         found_moving_box_10 = true;
//                     }

//                     last_replan_metrics_.obstacle_checks++;
//                     if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), other_ob)) {
//                         conflicts_with_other = true;
//                         if (is_target_edge) {
//                             std::cout << "   >>> Conflict found with: " << other_ob.name << std::endl;
//                             std::cout << "[RESTORE_PATH] CONFLICT with " << other_ob.name 
//                                       << " during remove of " << ob.name
//                                       << " path_size=" << other_ob.predicted_path.size()
//                                       << " first_t=" << (other_ob.predicted_path.empty() ? 0 : other_ob.predicted_path.front().z())
//                                       << " last_t="  << (other_ob.predicted_path.empty() ? 0 : other_ob.predicted_path.back().z())
//                                       << "\n";
//                         }
//                         break;
//                     } 
//                     else if (is_target_edge && other_ob.name == "moving_box_10") {
//                         std::cout << "[RESTORE_PATH] NO CONFLICT with moving_box_10 (this is the bug!) "
//                                   << "during remove of " << ob.name
//                                   << " path_size=" << other_ob.predicted_path.size()
//                                   << " first_t=" << (other_ob.predicted_path.empty() ? 0 : other_ob.predicted_path.front().z())
//                                   << " last_t="  << (other_ob.predicted_path.empty() ? 0 : other_ob.predicted_path.back().z())
//                                   << "\n";
//                     }
//                 }

//                 if (is_target_edge && !found_moving_box_10) {
//                     std::cout << "\033[1;31m   [CRITICAL BUG] 'moving_box_10' was NOT found in all_obstacles list!\033[0m" << std::endl;
//                 }

//                 if (!conflicts_with_other) {
//                     should_restore = true;
//                     if (is_target_edge) std::cout << "   >>> ACTION: Restoring edge (Safe against all others) <<<" << std::endl;
//                 }
//             }
            
//             if (should_restore) {
//                 edge.distance = edge.distance_original;
//                 if (neighbor->incomingEdges().count(node)) {
//                     neighbor->incomingEdges().at(node).distance = edge.distance_original;
//                 }
//                 neighborsWereBlocked = true;
//             }
//         }
//     };
    
//     for (int idx : unique_node_indices) {
//         RRTxNode* node = tree_[idx].get();
//         bool neighborsWereBlocked = false;
//         for (auto& [neighbor, edge] : node->outgoingEdges()) checkAndRestoreEdge(node, neighbor, edge, neighborsWereBlocked);
//         for (auto& [neighbor, edge] : node->culled_outgoing_edges_) checkAndRestoreEdge(node, neighbor, edge, neighborsWereBlocked);
//         if (neighborsWereBlocked) {
//             updateLMC(node);
//             if (node->getG() != node->getLMC()) verifyQueue(node);
//         }
//     }
// }
// #endif