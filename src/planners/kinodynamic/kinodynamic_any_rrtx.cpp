// Copyright 2025 Soheil E.nia

#include "motion_planning/planners/kinodynamic/kinodynamic_any_rrtx.hpp"
/*
    The Threat Set (Node-level)
    The Invalidating Set (Edge-level)
    The Threat Set is the bridge that allows a lazy algorithm to behave with the same spatial intelligence as an eager one

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


void KinodynamicANYRRTX::setStart(const Eigen::VectorXd& start) {
    robot_state_index_ = statespace_->getNumStates();
    auto node = std::make_shared<RRTxNode>(statespace_->addState(start) ,  tree_.size());
    tree_.push_back(node);
    node->setTimeToGoal(0);

    std::cout << "RRTX: Start node created on Index: " << robot_state_index_ << "\n";
}
void KinodynamicANYRRTX::setGoal(const Eigen::VectorXd& goal) {
    root_state_index_ = statespace_->getNumStates();
    auto node = std::make_shared<RRTxNode>(statespace_->addState(goal) ,  tree_.size());
    vbot_index_ = 1;
    vbot_node_ = node.get();
    node->setTimeToGoal(goal(goal.size() - 1));

    
    tree_.push_back(node); // Fixed parenthesis
    std::cout << "KinodynamicANYRRTX: Goal node created on Index: " << root_state_index_ << "\n";
}

std::vector<Eigen::VectorXd> KinodynamicANYRRTX::getPathPositions() const {
    // Check if the planner has a valid anchor point for the robot.
    if (!vbot_node_ || vbot_node_->getCost() == INFINITY) {
        RRTX_ERROR("[RRTX_Path_Assembly] Robot has no valid anchor node in the tree. Cannot build path.");
        return {}; // Return empty path
    }

    // Generate the "bridge" trajectory from the robot's continuous state
    //    to the anchor node on the fly.
    Trajectory bridge_traj = statespace_->steer(robot_continuous_state_, vbot_node_->getStateValue());

    if (!bridge_traj.is_valid) {
        RRTX_ERROR("[RRTX_Path_Assembly] Failed to steer from robot's continuous state to the anchor node.");
        return {};
    }

    // Start the final path with this bridge trajectory.
    std::vector<Eigen::VectorXd> final_executable_path = bridge_traj.path_points;

    // Traverse the rest of the tree from the anchor node using parent pointers.
    RRTxNode* child = vbot_node_;
    RRTxNode* parent = child->getParent();

    while (parent) {
        // No more map lookup! Just get the stored trajectory.
        const Trajectory& cached_traj = child->getParentTrajectory();
        
        if (cached_traj.is_valid && cached_traj.path_points.size() > 1) {
            final_executable_path.insert(final_executable_path.end(),
                                         cached_traj.path_points.begin() + 1,
                                         cached_traj.path_points.end());
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

    tree_.clear();  // Clear the vector

    // Reset the StateSpace
    statespace_->reset();
    kdtree_.reset();




    // Reset indices
    vbot_index_ = -1;
    vgoal_index_ = -1;
    root_state_index_ = -1;
    robot_state_index_ = -1;

    Vc_T_.clear();

    // Clear the sample counter
    sample_counter = 0;



}


void KinodynamicANYRRTX::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    auto start = std::chrono::high_resolution_clock::now();
    clearPlannerState();
    sample_counter = 0;
    visualization_ = visualization;
    num_of_samples_ = params.getParam<int>("num_of_samples");
    partial_update = params.getParam<bool>("partial_update");
    is_geometric_mode_ = params.getParam<bool>("is_geometric_mode", false);
    lower_bounds_ = problem_->getLowerBound();
    upper_bounds_ = problem_->getUpperBound();
    use_kdtree = params.getParam<bool>("use_kdtree");
    std::string kdtree_type = params.getParam<std::string>("kdtree_type");
    epsilon_ = params.getParam<double>("epsilon", 1e-4);
    kd_dim = params.getParam<int>("kd_dim",2);


    if (use_kdtree == true && kdtree_type == "NanoFlann"){
        Eigen::VectorXd weights(kd_dim);
        // weights << 1.0, 1.0, 1.0; // Weights for x, y, time
        switch (kd_dim) {
            case 2: // (x, y)
                weights << 1.0, 1.0; // Weights for x, y,
                break;
            case 3: // (x, y, time)
                {
                    weights << 1.0, 1.0, 1.0; // Weights for x, y, time
                }
                break;
            case 4: // (x, y, theta, time) - From your Dubins example
                {
                    weights << 1.0, 1.0, 1.0, 1.0; // Weights for x, y, theta, time
                }
                break;
            case 5: // (x, y, vx, vy, time) - From your Dubins example
                {
                    weights << 1.0, 1.0, 1.0, 1.0, 1.0; 
                }
                break;
            default: 
                RRTX_ERROR("Unsupported k-d tree dimension: " << kd_dim);
        }
        kdtree_ = std::make_shared<DynamicWeightedNanoFlann>(kd_dim, weights);
    } else if (use_kdtree == true && kdtree_type == "LieKDTree"){
        kdtree_ = std::make_unique<LieSplittingKDTree>(statespace_->getDimension(), statespace_);
    } else {
        throw std::runtime_error("FMTX requires a KD-Tree.");
    }

    std::cout << "KinodynamicANYRRTX setup complete: num_of_samples=" << num_of_samples_
                << ", bounds=[" << lower_bounds_ << ", " << upper_bounds_ << "]\n";


    std::cout << "--- \n";
    std::cout << "Taking care of the samples: \n \n";
    setStart(problem_->getStart());
    std::cout << "--- \n";
    setGoal(problem_->getGoal()); //robots current position

    // put the start and goal node in kdtree
    if (use_kdtree == true) {
        // kdtree_->addPoints(statespace_->getSamplesCopy());
        // kdtree_->buildTree();

        // Get the full 3D (or 4D) samples from the state space.
        Eigen::MatrixXd all_samples = statespace_->getSamplesCopy();

        // Define how many spatial dimensions you have.
        //    This makes the code robust for future changes (e.g., to 3D space).
        //    Assuming (x, y, time), the spatial dimension is 2.
        int spatial_dimension = kd_dim; // For (x, y)
        // For a future Dubins (x, y, theta, time) planner, this would still be 2.

        // Use .leftCols() to create a new matrix with only the spatial data.
        //    .eval() is used to ensure we pass a concrete matrix, not a temporary expression.
        Eigen::MatrixXd spatial_samples_only = all_samples.leftCols(spatial_dimension).eval();
        
        // Pass the 2D spatial matrix to the KD-tree.
        kdtree_->addPoints(spatial_samples_only);
        
        // Build the tree all at once after we fill the data.
        kdtree_->buildTree();
    }

    /////////////////////////SETTING UP DS//////////////


    tree_.at(0)->setCost(0);
    tree_.at(0)->setLMC(0);



    v_indices_.insert(0);
    // v_indices_.insert(1); 
    ///////////////////Neighborhood Radius////////////////////////////////
    dimension_ = statespace_->getDimension();
    // dimension_ = kd_dim;
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
    // gamma_ = 2 * std::pow(1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); //FMT star gamma
    gamma_ = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); //RRT star gamma



    // Since i want to put a cap on the number of samples and i want RRTX to be as close as to FMTX im gonna set step size (delta) to this:
    factor = params.getParam<double>("factor");
    std::cout<<"factor: "<<factor<<"\n";
    // delta = factor * gamma_ * std::pow(std::log(num_of_samples_) / num_of_samples_, 1.0 / d);
    delta = params.getParam<double>("delta");
    std::cout << "Computed value of delta: " << delta << std::endl;



    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "---\n";

    sample_counter = 1;
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

bool KinodynamicANYRRTX::runForensics() {
    std::cout << "\n[RRTx FORENSICS] --- STARTING POST-PLAN GRAPH VERIFICATION ---" << std::endl;
    int illegal_connections = 0;
    int checked_nodes = 0;

    // Start at 1 to skip the root/goal node which has no parent
    for (size_t i = 0; i < tree_.size(); ++i) { 
        RRTxNode* node = tree_[i].get();

        // Only check nodes that have an active parent connection
        if (node->getParent() != nullptr && node->getCost() != std::numeric_limits<double>::infinity()) {
            checked_nodes++;
            bool edge_collides = false;
            std::string guilty_obstacle = "";
            RRTxNode* parent = node->getParent();
            
            // Re-steer from Child back to Parent (matching RRTX shortest-path direction)
            Trajectory edge_traj = statespace_->steer(node->getStateValue(), parent->getStateValue());
            double time_ref = node->getTimeToGoal(); 

            // Verify against Absolute Ground Truth (previous_obstacles_ map)
            for (const auto& [name, ob] : previous_obstacles_) {
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(edge_traj, time_ref, ob)) {
                    edge_collides = true;
                    guilty_obstacle = name;
                    break;
                }
            }

            if (edge_collides) {
                illegal_connections++;
                std::cout << "\033[1;31m[VIOLATION]\033[0m Node " << node->getIndex() 
                          << " -> Parent " << parent->getIndex()
                          << " | Cost: " << node->getCost()
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

// In KinodynamicANYRRTX.cpp

void KinodynamicANYRRTX::plan() {
    // 1. Calculate Radius (Do this every iteration as the tree grows)
    for (int i = 0; i < num_of_samples_; ++i) {
        neighborhood_radius_ = shrinkingBallRadius();

        // 2. Sample a point
        Eigen::VectorXd sample = Eigen::VectorXd::Random(dimension_);
        sample = (lower_bounds_.array() + (upper_bounds_ - lower_bounds_).array() * ((sample.array() + 1.0) / 2.0)).matrix();
        
        sample_counter++;
        
        // 3. Find Nearest
        std::vector<size_t> nearest_indices = kdtree_->knnSearch(sample.head(kd_dim), 1);
        RRTxNode* nearest_node = tree_[nearest_indices[0]].get();
        Eigen::VectorXd nearest_state = nearest_node->getStateValue();
        
        // 4. Saturate (Steer)
        sample = saturate(sample, nearest_state, delta);

        // 5. Attempt to extend tree
        bool node_added = false;
        if (obs_checker_->isObstacleFree(sample)) {
            node_added = extend(sample);
        }
            
        // 6. If added, rewire and reduce inconsistency
        if (node_added) {
            RRTxNode* new_node = tree_.back().get();
            
            // Add to active node set
            v_indices_.insert(tree_.size()-1);
            
            // Update node costs and neighbors
            /*
                Is rewireNeighbors Necessary here? isnt reduceInconsistency function has rewire neighbor in it? Its is necessary here for new samples!
                imagine the robot is near root and we added a new sample to the envrionment far away 
                from robot then if we dont rewire neighbors then those neighbors wont get into the queue
                to later be processed by reducinconsistency.
            */
            rewireNeighbors(new_node); 
            // verifyQueue(new_node);
            new_node->setCost(new_node->getLMC());
            reduceInconsistency();
        }
    }

    #if DEBUG
        runForensics();
    #endif
    
}

// ==============================================================================================
// STRATEGY 1: INVALIDATING SET (Edge-Level Caching + Local Broad-Phase)
// ==============================================================================================
#if USE_INVALIDATING_SET_STRATEGY

bool KinodynamicANYRRTX::extend(Eigen::VectorXd v) {
    auto new_node = std::make_shared<RRTxNode>(statespace_->addState(v), tree_.size());
    auto neighbors = kdtree_->radiusSearch(new_node->getStateValue().head(kd_dim), neighborhood_radius_ + 0.01);
    
    double min_lmc = INFINITY;
    RRTxNode* best_parent = nullptr;
    Trajectory best_traj;

    // 1. LOCAL THREAT SET FOR EXTEND
    // Broad-phase optimization to avoid checking the whole world
    std::vector<const Obstacle*> local_threats;
    for (const auto& [name, ob] : previous_obstacles_) {
        if (obs_checker_->isNodeInObstacleTube(new_node->getStateValue(), ob, neighborhood_radius_ + 0.01)) {
            local_threats.push_back(&ob);
        }
    }

    if (!is_geometric_mode_) {
        double absolute_t = new_node->getStateValue().tail<1>()[0];
        new_node->setTimeToGoal(absolute_t);
    } else {
        new_node->setTimeToGoal(0.0);
    }
    double v_time = new_node->getTimeToGoal();

    // struct EdgeEval {
    //     RRTxNode* neighbor;
    //     bool fwd_exists = false; Trajectory fwd_traj; bool fwd_safe = false; std::unordered_set<std::string> fwd_blockers;
    //     bool rev_exists = false; Trajectory rev_traj; bool rev_safe = false; std::unordered_set<std::string> rev_blockers;
    // };

    struct EdgeEval {
        RRTxNode* neighbor;
        bool fwd_exists = false; Trajectory fwd_traj; bool fwd_safe = false; 
        std::vector<const Obstacle*> fwd_blockers; // UPDATED
        bool rev_exists = false; Trajectory rev_traj; bool rev_safe = false; 
        std::vector<const Obstacle*> rev_blockers; // UPDATED
    };
    
    std::vector<EdgeEval> evaluated_edges(neighbors.size());

    // PASS 1: Evaluate OUTGOING edges (v -> u) & Find Parent
    for (size_t i = 0; i < neighbors.size(); ++i) {
        auto& candidate = tree_[neighbors[i]];
        if (candidate.get() == new_node.get()) continue;
        
        RRTxNode* u = candidate.get();
        evaluated_edges[i].neighbor = u;

        Trajectory fwd_traj = statespace_->steer(new_node->getStateValue(), u->getStateValue());
        if (fwd_traj.is_valid && fwd_traj.cost <= neighborhood_radius_ + 0.01) {
            evaluated_edges[i].fwd_exists = true;
            evaluated_edges[i].fwd_traj = fwd_traj;
            evaluated_edges[i].fwd_safe = true;
            
            // Check against local threats
            for (const Obstacle* ob_ptr : local_threats) {
                const Obstacle& ob = *ob_ptr;
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(fwd_traj, v_time, ob)) {
                    evaluated_edges[i].fwd_safe = false;
                    // NO BREAK HERE! We must collect ALL invalidating obstacles
                    // evaluated_edges[i].fwd_blockers.insert(ob.name);
                    evaluated_edges[i].fwd_blockers.push_back(ob_ptr); // Push Pointer
                }
            }

            if (evaluated_edges[i].fwd_safe) {
                const double candidate_lmc = u->getLMC() + fwd_traj.cost;
                if (candidate_lmc < min_lmc && candidate_lmc < new_node->getLMC()) {
                    min_lmc = candidate_lmc;
                    best_parent = u;
                    best_traj = fwd_traj; 
                }
            }
        }
    }

    // EARLY BAILOUT
    if (!best_parent) {
        sample_counter--;
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
            evaluated_edges[i].rev_blockers = evaluated_edges[i].fwd_blockers;
        } else {
            Trajectory rev_traj = statespace_->steer(u->getStateValue(), new_node->getStateValue());
            if (rev_traj.is_valid && rev_traj.cost <= neighborhood_radius_ + 0.01) {
                evaluated_edges[i].rev_exists = true;
                evaluated_edges[i].rev_traj = rev_traj;
                evaluated_edges[i].rev_safe = true;
                
                for (const Obstacle* ob_ptr : local_threats) {
                    const Obstacle& ob = *ob_ptr;
                    last_replan_metrics_.obstacle_checks++;
                    if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(rev_traj, u->getTimeToGoal(), ob)) {
                        evaluated_edges[i].rev_safe = false;
                        // NO BREAK HERE! Must collect ALL blockers
                        // evaluated_edges[i].rev_blockers.insert(ob.name);
                        evaluated_edges[i].rev_blockers.push_back(ob_ptr); // Push Pointer
                    }
                }
            }
        }
    }

    // COMMIT GRAPH CHANGES
    new_node->setParent(best_parent, best_traj);
    new_node->setTimeToGoal(v_time);
    new_node->setLMC(min_lmc);

    tree_.push_back(new_node);
    kdtree_->addPoint(new_node->getStateValue().head(kd_dim));
    kdtree_->buildTree(); 

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
    auto neighbors = kdtree_->radiusSearch(new_node->getStateValue().head(kd_dim), neighborhood_radius_ + 0.01);
    
    double min_lmc = INFINITY;
    RRTxNode* best_parent = nullptr;
    Trajectory best_traj;

    if (!is_geometric_mode_) {
        double absolute_t = new_node->getStateValue().tail<1>()[0];
        new_node->setTimeToGoal(absolute_t);
    } else {
        new_node->setTimeToGoal(0.0);
    }
    double v_time = new_node->getTimeToGoal();

    // INITIALIZE NODE-BASED THREAT SET
    for (const auto& [name, ob] : previous_obstacles_) {
        if (obs_checker_->isNodeInObstacleTube(new_node->getStateValue(), ob, delta)) {
            // new_node->threats_.insert(name);
            new_node->threats_.push_back(&ob);
        }
    }

    struct EdgeEval {
        RRTxNode* neighbor;
        bool fwd_exists = false; Trajectory fwd_traj; bool fwd_safe = false; 
        bool rev_exists = false; Trajectory rev_traj; bool rev_safe = false; 
    };
    
    std::vector<EdgeEval> evaluated_edges(neighbors.size());

    // PASS 1: Evaluate OUTGOING edges (v -> u) & Find Parent
    for (size_t i = 0; i < neighbors.size(); ++i) {
        auto& candidate = tree_[neighbors[i]];
        if (candidate.get() == new_node.get()) continue;
        
        RRTxNode* u = candidate.get();
        evaluated_edges[i].neighbor = u;

        Trajectory fwd_traj = statespace_->steer(new_node->getStateValue(), u->getStateValue());
        if (fwd_traj.is_valid && fwd_traj.cost <= neighborhood_radius_ + 0.01) {
            evaluated_edges[i].fwd_exists = true;
            evaluated_edges[i].fwd_traj = fwd_traj;
            evaluated_edges[i].fwd_safe = true;
            
            // for (const std::string& threat_name : new_node->threats_) {
            //     const Obstacle& ob = previous_obstacles_.at(threat_name);
            //     last_replan_metrics_.obstacle_checks++;
            //     if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(fwd_traj, v_time, ob)) {
            //         evaluated_edges[i].fwd_safe = false;
            //         break; // Just need one threat to block it
            //     }
            // }
            for (const Obstacle* threat_ptr : new_node->threats_) {
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(fwd_traj, v_time, *threat_ptr)) {
                    evaluated_edges[i].fwd_safe = false;
                    break; 
                }
            }

            if (evaluated_edges[i].fwd_safe) {
                const double candidate_lmc = u->getLMC() + fwd_traj.cost;
                if (candidate_lmc < min_lmc && candidate_lmc < new_node->getLMC()) {
                    min_lmc = candidate_lmc;
                    best_parent = u;
                    best_traj = fwd_traj; 
                }
            }
        }
    }

    if (!best_parent) {
        sample_counter--;
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
            Trajectory rev_traj = statespace_->steer(u->getStateValue(), new_node->getStateValue());
            if (rev_traj.is_valid && rev_traj.cost <= neighborhood_radius_ + 0.01) {
                evaluated_edges[i].rev_exists = true;
                evaluated_edges[i].rev_traj = rev_traj;
                evaluated_edges[i].rev_safe = true;
                
                // // Use Source Node's perfectly maintained threat set!
                // for (const std::string& threat_name : u->threats_) {
                //     const Obstacle& ob = previous_obstacles_.at(threat_name);
                //     last_replan_metrics_.obstacle_checks++;
                //     if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(rev_traj, u->getTimeToGoal(), ob)) {
                //         evaluated_edges[i].rev_safe = false;
                //         break;
                //     }
                // }
                for (const Obstacle* threat_ptr : u->threats_) {
                    last_replan_metrics_.obstacle_checks++;
                    if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(rev_traj, u->getTimeToGoal(), *threat_ptr)) {
                        evaluated_edges[i].rev_safe = false;
                        break;
                    }
                }
            }
        }
    }

    // COMMIT GRAPH CHANGES
    new_node->setParent(best_parent, best_traj);
    new_node->setTimeToGoal(v_time);
    new_node->setLMC(min_lmc);

    tree_.push_back(new_node);
    kdtree_->addPoint(new_node->getStateValue().head(kd_dim));
    kdtree_->buildTree(); 

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

bool KinodynamicANYRRTX::extend(Eigen::VectorXd v) {
    auto new_node = std::make_shared<RRTxNode>(statespace_->addState(v), tree_.size());
    auto neighbors = kdtree_->radiusSearch(new_node->getStateValue().head(kd_dim), neighborhood_radius_ + 0.01);
    
    double min_lmc = INFINITY;
    RRTxNode* best_parent = nullptr;
    Trajectory best_traj;

    if (!is_geometric_mode_) {
        double absolute_t = new_node->getStateValue().tail<1>()[0];
        new_node->setTimeToGoal(absolute_t);
    } else {
        new_node->setTimeToGoal(0.0);
    }
    double v_time = new_node->getTimeToGoal();

    // Fetch globally tracked obstacles for brute-force checking
    ObstacleVector all_obstacles = obs_checker_->getObstacles();

    struct EdgeEval {
        RRTxNode* neighbor;
        bool fwd_exists = false; Trajectory fwd_traj; bool fwd_safe = false; 
        bool rev_exists = false; Trajectory rev_traj; bool rev_safe = false; 
    };
    
    std::vector<EdgeEval> evaluated_edges(neighbors.size());

    // PASS 1: Evaluate OUTGOING edges (v -> u) & Find Parent
    for (size_t i = 0; i < neighbors.size(); ++i) {
        auto& candidate = tree_[neighbors[i]];
        if (candidate.get() == new_node.get()) continue;
        
        RRTxNode* u = candidate.get();
        evaluated_edges[i].neighbor = u;

        Trajectory fwd_traj = statespace_->steer(new_node->getStateValue(), u->getStateValue());
        if (fwd_traj.is_valid && fwd_traj.cost <= neighborhood_radius_ + 0.01) {
            evaluated_edges[i].fwd_exists = true;
            evaluated_edges[i].fwd_traj = fwd_traj;
            evaluated_edges[i].fwd_safe = true;
            
            // Brute force against all known obstacles
            for (const auto& ob : all_obstacles) {
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(fwd_traj, v_time, ob)) {
                    evaluated_edges[i].fwd_safe = false;
                    break; 
                }
            }

            if (evaluated_edges[i].fwd_safe) {
                const double candidate_lmc = u->getLMC() + fwd_traj.cost;
                if (candidate_lmc < min_lmc && candidate_lmc < new_node->getLMC()) {
                    min_lmc = candidate_lmc;
                    best_parent = u;
                    best_traj = fwd_traj; 
                }
            }
        }
    }

    if (!best_parent) {
        sample_counter--;
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
            Trajectory rev_traj = statespace_->steer(u->getStateValue(), new_node->getStateValue());
            if (rev_traj.is_valid && rev_traj.cost <= neighborhood_radius_ + 0.01) {
                evaluated_edges[i].rev_exists = true;
                evaluated_edges[i].rev_traj = rev_traj;
                evaluated_edges[i].rev_safe = true;
                
                // Brute force reverse edge
                for (const auto& ob : all_obstacles) {
                    last_replan_metrics_.obstacle_checks++;
                    if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(rev_traj, u->getTimeToGoal(), ob)) {
                        evaluated_edges[i].rev_safe = false;
                        break;
                    }
                }
            }
        }
    }

    // COMMIT GRAPH CHANGES
    new_node->setParent(best_parent, best_traj);
    new_node->setTimeToGoal(v_time);
    new_node->setLMC(min_lmc);

    tree_.push_back(new_node);
    kdtree_->addPoint(new_node->getStateValue().head(kd_dim));
    kdtree_->buildTree(); 

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
    const double inconsistency = v->getCost() - v->getLMC();
    if (inconsistency <= epsilon_) return;

    cullNeighbors(v);
    // metrics_.total_rewire_edges += v->incomingEdges().size(); // TRACKING

    last_replan_metrics_.rewire_neighbor_searches += v->incomingEdges().size();

    for (auto& [u, edge] : v->incomingEdges()) {
        // if (u == v->getParent() || !isValidEdge(u, v, edge)) continue;

        // if (v->getIndex() == 538 && u->getIndex() == 260 || 
        //     u->getIndex() == 260 && v->getIndex() == 538  
        //     )
        //     std::cout<<edge.distance <<"\n";
        if (u == v->getParent() ) continue;


    // // === INJECT THIS LOG ===
    //     if ((v->getIndex() == 288 && u->getIndex() == 548) || 
    //         (v->getIndex() == 548 && u->getIndex() == 288)) {
    //         std::cout << "\n[LOG - REWIRE] Node " << u->getIndex() << " evaluating connection to Node " << v->getIndex() << "\n";
    //         std::cout << "   -> edge.distance seen by rewire: " << edge.distance << "\n";
    //         std::cout << "   -> candidate_lmc: " << v->getLMC() + edge.distance << "\n";
    //     }
    //     // =======================



        const double candidate_lmc = v->getLMC() + edge.distance;
        if (u->getLMC() > candidate_lmc) {
            // The trajectory for the u -> v edge is what's stored in 'edge_from_u_on_v'
            // const Trajectory& trajectory_to_parent = v->outgoing_edges_.at(u).cached_trajectory;

            u->setLMC(candidate_lmc);
            // makeParentOf(u, v, edge.distance);
            u->setParent(v,*(edge.cached_trajectory));
            // edge_length_[u->getIndex()] = edge.distance;
            if (u->getCost() - candidate_lmc > epsilon_) {
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
            // 1. Check if Robot is already consistent (Cost == LMC)
            bool robot_consistent = (vbot_node_->getCost() == vbot_node_->getLMC());
            
            // 2. Check if the queue has passed the robot
            // We stop if the smallest key in the queue is greater than the robot's cost.
            bool queue_past_robot = (min_key > vbot_node_->getCost() + bridge_cost_);
            
            // STOP CONDITION:
            // If the robot is consistent AND the queue only contains nodes more expensive than the robot,
            // then we have successfully repaired the path up to the robot.
            if (robot_consistent && queue_past_robot) {
                break;
            }
        }
        
        inconsistency_queue_.pop();
        // metrics_.total_reduce_iterations++;

        RRTxNode* node = top_element.second;
        int node_idx = node->getIndex();
        
        // Safety check (ensure node is valid and not an orphan)
        if (node_idx == -1 || Vc_T_.count(node_idx)) continue;
        
        // Standard RRTx logic: if Cost > LMC, we need to update
        if (node->getCost() > node->getLMC() + epsilon_) {
            updateLMC(node);
            rewireNeighbors(node);
        }
        
        // Synchronize Cost and LMC
        node->setCost(node->getLMC());
    }
    
    // Debugging
    // std::cout << "Queue size after reduce: " << inconsistency_queue_.getHeap().size() << "\n";
}



double KinodynamicANYRRTX::shrinkingBallRadius() const {
    int d = kd_dim;
    auto rad = factor * gamma_ * pow(log(tree_.size()) / tree_.size(), 1.0/d);
    // std::cout<<"current rad: "<<std::min(rad,delta)<<"\n";
    return std::min(rad, delta);
    // return 15.0;

}


void KinodynamicANYRRTX::updateLMC(RRTxNode* v) {
    cullNeighbors(v);
    // metrics_.total_lmc_edges += v->outgoingEdges().size(); // TRACKING

    double min_lmc = v->getLMC();
    RRTxNode* best_parent = nullptr;
    double best_edge_distance = INFINITY;  // Track the distance of the best edge
    // Trajectory best_traj = v->getParentTrajectory();
    Trajectory* best_traj = nullptr;

    last_replan_metrics_.rewire_neighbor_searches += v->outgoingEdges().size();

    // Iterate over outgoing edges (v → u)
    for (auto& [u, edge] : v->outgoingEdges()) {
        // if (v->getIndex() == 538 && u->getIndex() == 260 || 
        //     u->getIndex() == 260 && v->getIndex() == 538  
        //     )
        //     std::cout<<edge.distance <<"\n";


        if (Vc_T_.count(u->getIndex()) || edge.distance == INFINITY) continue;
        const double candidate_lmc = u->getLMC() + edge.distance;
        if (candidate_lmc < min_lmc) {
            min_lmc = candidate_lmc;
            best_parent = u;
            best_edge_distance = edge.distance;  // Capture the distance here
            best_traj = edge.cached_trajectory.get();
        }
    }

    if (best_parent) {
        // std::vector<Eigen::VectorXd> positions4;
        // std::string color_str = "0.0,0.0,1.0"; // Blue color
        // Eigen::VectorXd vec(2);
        // vec << v->getStateValue();
        // positions4.push_back(vec);
        // Eigen::VectorXd vec2(2);
        // vec2 << best_parent->getStateValue();
        // positions4.push_back(vec2);
        // visualization_->visualizeNodes(positions4,"map",color_str);


        v->setParent(best_parent, *best_traj);  // Use the captured distance
        v->setLMC(min_lmc);
    } 
}


void KinodynamicANYRRTX::cullNeighbors(RRTxNode* v) {
    // 1. PERFORMANCE TRIGGER
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

        // --- CONSOLIDATED RRTX CULL PROTECTION ---
        // A node ONLY evaluates culling if the edge is longer than the current radius
        // AND the neighbor is not the current parent in the shortest-path tree. [cite: 1066]
        if (edge_cost > (neighborhood_radius_ + 0.01) && neighbor != v->getParent()) {

            // 1. SYMMETRIC CULL (Neighbor's Side)
            // Remove 'v' from the neighbor's incoming list if it wasn't an 'initial' birth-neighbor. [cite: 766]
            auto& incoming = neighbor->incomingEdges();
            if (auto incoming_it = incoming.find(v); incoming_it != incoming.end()) {
                if (!incoming_it->second.is_initial) {
                    incoming.erase(incoming_it);
                }
            }

            // 2. SOURCE CULL (v's Side)
            // Move the neighbor to the passive map if it wasn't an 'initial' birth-neighbor. [cite: 1084]
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
    const double min_key = std::min(node->getLMC(), node->getCost());
    const double g_value = node->getCost();
    

    if (node->in_queue_) {
        // Update both the priority and maintains g_value through node pointer
        inconsistency_queue_.update(node, min_key);
    } else {
        inconsistency_queue_.add(node, min_key);
        // node->in_queue_ = true;
    }
}







void KinodynamicANYRRTX::verifyOrphan(RRTxNode* node) {
    if(node->in_queue_==true){
        inconsistency_queue_.remove(node);
        // node->in_queue_=false;
    }

    int idx = node->getIndex();
    if (idx != -1) {
        Vc_T_.insert(idx);
    }
}

void KinodynamicANYRRTX::propagateDescendants() {
    std::queue<RRTxNode*> to_process;

    // Step 1: Propagate descendants through the tree
    for (int idx : Vc_T_) {
        if (idx >= 0 && idx < tree_.size()) {
            to_process.push(tree_[idx].get());
        }
    }

    while (!to_process.empty()) {
        RRTxNode* current = to_process.front();
        to_process.pop();

        // Propagate to children using successors()
        for (RRTxNode* child : current->getChildren()) {
            int child_idx = child->getIndex();
            if (child_idx == -1 || Vc_T_.count(child_idx)) continue;

            Vc_T_.insert(child_idx);
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
        if (idx < 0 || idx >= tree_.size()) continue;
        auto node = tree_[idx].get();

        // Process outgoing neighbors (N⁺(v) \ Vc_T)
        for (const auto& [neighbor, edge] : node->outgoingEdges()) {
            // Skip invalid edges
            // if (!isValidEdge(node, neighbor, edge)) continue; 

            int neighbor_idx = neighbor->getIndex();
            if (neighbor_idx == -1 || Vc_T_.count(neighbor_idx)) continue;

            // edge_length_[neighbor_idx] = -INFINITY;
            neighbor->setCost(INFINITY);
            verifyQueue(neighbor);
        }

        // Process parent (p⁺_T(v) \ Vc_T)
        if (RRTxNode* parent = node->getParent()) {
            // Find the edge from parent to node
            const auto& parent_edges = parent->outgoingEdges();
            auto it = parent_edges.find(node);

            // Validate the edge if it exists
            // if (it != parent_edges.end() && isValidEdge(parent, node, it->second)) {
            if (it != parent_edges.end() ) {
                int parent_idx = parent->getIndex();
                if (parent_idx != -1 && !Vc_T_.count(parent_idx)) {
                    // edge_length_[parent_idx] = -INFINITY;
                    parent->setCost(INFINITY);
                    verifyQueue(parent);
                }
            }
        }
    }

    // Reset orphaned nodes using new parent API
    for (int idx : Vc_T_) {
        if (idx < 0 || idx >= tree_.size()) continue;
        auto node = tree_[idx].get();

        node->setCost(INFINITY);
        node->setLMC(INFINITY);


        /*
            An orphaned node doesn't move in time; it just has no path to the goal. 
            By keeping its time_to_goal at the original sampled value, the collision checker can still accurately check if it is blocked or clear
        */
        // node->setTimeToGoal(std::numeric_limits<double>::infinity()); // TTG doesnt change! its sampled and fixed for a node!
        node->setParent(nullptr, Trajectory{});
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

        if (child_node->getCost() != std::numeric_limits<double>::infinity()) {
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

    // --- 1. QUERY POINT CONSTRUCTION ---
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
    // --- 2. HYSTERESIS LOGIC ---
    const double hysteresis_factor = 0.98;
    double cost_of_current_path = std::numeric_limits<double>::infinity();
    if (vbot_node_ && vbot_node_->getCost() != INFINITY) {
        Trajectory bridge = statespace_->steer(robot_continuous_state_, vbot_node_->getStateValue());
        // Use robot_time_to_go so collision check is synced with the world
        if (bridge.is_valid && obs_checker_->isTrajectorySafe(bridge, robot_time_to_go)) {
            last_replan_metrics_.obstacle_checks += obs_checker_->getObstaclesSize();
            cost_of_current_path = bridge.cost + vbot_node_->getCost();
            robot_current_time_to_goal_ = bridge.time_duration + vbot_node_->getTimeToGoal();
            // return;
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
            if (candidate->getCost() == INFINITY) continue;
            Trajectory bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());
            if (!bridge.is_valid) continue;

            last_replan_metrics_.obstacle_checks += obs_checker_->getObstaclesSize();

            // CRITICAL: Check if this candidate connection is safe
            if (!obs_checker_->isTrajectorySafe(bridge, robot_time_to_go)) continue;

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
    // --- 3. ASSIGNMENT ---
    if (best_candidate_node && best_candidate_cost < cost_of_current_path * hysteresis_factor) {
        vbot_node_ = best_candidate_node;
        robot_current_time_to_goal_ = best_candidate_bridge.time_duration + best_candidate_node->getTimeToGoal();
        last_replan_metrics_.path_cost = best_candidate_cost;
    } else if (vbot_node_ && cost_of_current_path != std::numeric_limits<double>::infinity()) {
        Trajectory bridge = statespace_->steer(robot_continuous_state_, vbot_node_->getStateValue());
        robot_current_time_to_goal_ = bridge.time_duration + vbot_node_->getTimeToGoal();
        // The cost changes slightly every frame as the robot moves towards the anchor.
        last_replan_metrics_.path_cost = cost_of_current_path;
    } else {
        // We are trapped. No nodes in radius are safe.
        vbot_node_ = nullptr;
        robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
        bridge_cost_ = std::numeric_limits<double>::infinity();
        last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
        RRTX_WARN("[RRTX_Anchor] Status: NULL (Robot is lost or searching...)");
    }

    // 4. INTERNAL DEBUG VISUALIZATION
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
    // If cost is INFINITY, the anchor is invalid (trapped).
    return (vbot_node_ != nullptr) && (vbot_node_->getCost() != INFINITY);
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
        
        // 1. Retrieve the stored obstacle (or create a blank one)
        Obstacle& stored_ob = previous_obstacles_[incoming_ob.name];
        
        // 2. REMOVE OLD TUBE (Pruning)
        // We must do this BEFORE overwriting stored_ob, because we need the OLD path to find nodes to repair.
        if (!stored_ob.predicted_path.empty()) {
            removeObstacle(stored_ob); 
        }

        // 3. UPDATE STATE (The Critical Fix)
        // Instead of copying fields manually, copy the WHOLE object.
        // This ensures 'has_ground_truth', 'motion_axis', etc. are all transferred correctly.
        stored_ob = incoming_ob; 

        // 4. GENERATE NEW DENSE TUBE
        // We regenerate this here to ensure it uses the Planner's exact 'T_robot'.
        // Since we copied the object in step 3, 'stored_ob' now has the valid physics data to do this.
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

        // 5. ADD NEW TUBE (Rewiring)
        // Now that the path is populated, this will successfully invalidate blocked nodes.
        
        addNewObstacle(stored_ob);
        
        // RCLCPP_INFO(rclcpp::get_logger("RRTx"), 
        //     "Obstacle [%s]: Removed %zu candidates, Added %zu candidates. Tube Size: %zu", 
        //     stored_ob.name.c_str(), debug_repaired_nodes.size(), 
        //     debug_invalidated_nodes.size(), stored_ob.predicted_path.size());
    }

    // 6. PROPAGATE CHANGES
    propagateDescendants();
    if (vbot_node_) verifyQueue(vbot_node_);
    reduceInconsistency();
}




// ==============================================================================================
// STRATEGY 1: INVALIDATING SET (Edge-Level Caching)
// ==============================================================================================
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
        const double edge_start_ttg = node->getTimeToGoal();
        last_replan_metrics_.obstacle_checks++;

        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), edge_start_ttg, ob)) {
            edge.distance = std::numeric_limits<double>::infinity();

            // edge.invalidating_obstacles.insert(ob.name);

            // Pointer insertion without duplicates
            if (std::find(edge.invalidating_obstacles.begin(), edge.invalidating_obstacles.end(), &ob) == edge.invalidating_obstacles.end()) {
                edge.invalidating_obstacles.push_back(&ob);
            }
            
            // if (neighbor->incomingEdges().count(node)) {
            //     auto& inc_edge = neighbor->incomingEdges().at(node);
            //     inc_edge.distance = std::numeric_limits<double>::infinity();
            //     inc_edge.invalidating_obstacles.insert(ob.name);
            // }
            if (neighbor->incomingEdges().count(node)) {
                auto& inc_edge = neighbor->incomingEdges().at(node);
                inc_edge.distance = std::numeric_limits<double>::infinity();
                
                // CORRECT: Pointer insertion without duplicates
                if (std::find(inc_edge.invalidating_obstacles.begin(), inc_edge.invalidating_obstacles.end(), &ob) == inc_edge.invalidating_obstacles.end()) {
                    inc_edge.invalidating_obstacles.push_back(&ob);
                }
            }
            
            if (neighbor->getParent() == node) verifyOrphan(neighbor);
            if (node->getParent() == neighbor) verifyOrphan(node);
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

    // auto checkAndRestoreEdge = [&](RRTxNode* node, RRTxNode* neighbor, EdgeInfo& edge, bool& neighborsWereBlocked) {
    //     if (edge.distance == std::numeric_limits<double>::infinity()) {
    //         if (edge.invalidating_obstacles.erase(ob.name) > 0) {
    //             if (edge.invalidating_obstacles.empty()) {
    //                 edge.distance = edge.distance_original;
    //                 if (neighbor->incomingEdges().count(node)) {
    //                     auto& inc_edge = neighbor->incomingEdges().at(node);
    //                     inc_edge.distance = edge.distance_original;
    //                     inc_edge.invalidating_obstacles.erase(ob.name);
    //                 }
    //                 neighborsWereBlocked = true;
    //             }
    //         }
    //     }
    // };

    auto checkAndRestoreEdge = [&](RRTxNode* node, RRTxNode* neighbor, EdgeInfo& edge, bool& neighborsWereBlocked) {
        if (edge.distance == std::numeric_limits<double>::infinity()) {
            
            // O(1) SWAP-AND-POP for the outgoing edge
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
            if (node->getCost() != node->getLMC()) verifyQueue(node);
        }
    }
}

// ==============================================================================================
// STRATEGY 2: THREAT SET (Node-Level Filtering)
// ==============================================================================================
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

        const double edge_start_ttg = node->getTimeToGoal();
        last_replan_metrics_.obstacle_checks++;

        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), edge_start_ttg, ob)) {
            edge.distance = std::numeric_limits<double>::infinity();
            
            if (neighbor->incomingEdges().count(node)) {
                neighbor->incomingEdges().at(node).distance = std::numeric_limits<double>::infinity();
            }
            
            if (neighbor->getParent() == node) verifyOrphan(neighbor);
            if (node->getParent() == neighbor) verifyOrphan(node);
        }
    };

    for (int idx : unique_node_indices) {
        RRTxNode* node = tree_[idx].get();
        // node->threats_.insert(ob.name);
        // Pointer insertion without duplicates
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

    // auto checkAndRestoreEdge = [&](RRTxNode* node, RRTxNode* neighbor, EdgeInfo& edge, bool& neighborsWereBlocked) {
    //     if (edge.distance == std::numeric_limits<double>::infinity()) {
    //         const double ttg = node->getTimeToGoal();
    //         bool is_safe = true;
    //         for (const std::string& threat_name : node->threats_) {
    //             last_replan_metrics_.obstacle_checks++;
    //             const Obstacle& threat_ob = previous_obstacles_.at(threat_name); 
    //             if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ttg, threat_ob)) {
    //                 is_safe = false;
    //                 break;
    //             }
    //         }

    //         if (is_safe) {
    //             edge.distance = edge.distance_original;
    //             if (neighbor->incomingEdges().count(node)) {
    //                 neighbor->incomingEdges().at(node).distance = edge.distance_original;
    //             }
    //             neighborsWereBlocked = true;
    //         }
    //     }
    // };


    auto checkAndRestoreEdge = [&](RRTxNode* node, RRTxNode* neighbor, EdgeInfo& edge, bool& neighborsWereBlocked) {
        if (edge.distance == std::numeric_limits<double>::infinity()) {
            const double ttg = node->getTimeToGoal();
            bool is_safe = true;
            
            // Instant iteration through contiguous pointers
            for (const Obstacle* threat_ptr : node->threats_) {
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ttg, *threat_ptr)) {
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
            }
        }
    };

    for (int idx : unique_node_indices) {
        RRTxNode* node = tree_[idx].get();
        // node->threats_.erase(ob.name);

        // O(1) SWAP-AND-POP Node Threat Removal
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
            if (node->getCost() != node->getLMC()) verifyQueue(node);
        }
    }
}

// ==============================================================================================
// STRATEGY 3: DEFAULT (Brute-Force Fallback)
// ==============================================================================================
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

        const double edge_start_ttg = node->getTimeToGoal();
        last_replan_metrics_.obstacle_checks++;

        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), edge_start_ttg, ob)) {
            edge.distance = std::numeric_limits<double>::infinity();
            
            if (neighbor->incomingEdges().count(node)) {
                neighbor->incomingEdges().at(node).distance = std::numeric_limits<double>::infinity();
            }
            
            if (neighbor->getParent() == node) verifyOrphan(neighbor);
            if (node->getParent() == neighbor) verifyOrphan(node);
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

    ObstacleVector all_obstacles = obs_checker_->getObstacles();

    auto checkAndRestoreEdge = [&](RRTxNode* node, RRTxNode* neighbor, EdgeInfo& edge, bool& neighborsWereBlocked) {
        if (edge.distance == std::numeric_limits<double>::infinity()) {
            const double ttg = node->getTimeToGoal();
            bool should_restore = false;
            
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
            if (node->getCost() != node->getLMC()) verifyQueue(node);
        }
    }
}

#endif