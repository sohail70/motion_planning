// Copyright 2025 Soheil E.nia

#include "motion_planning/planners/kinodynamic/kinodynamic_rrtx.hpp"
// -----------------------------------------------------------------------------------------
// STRATEGY SWITCH
// 1 = Use Set Optimization (Eager Add, Fast Remove). Requires 'invalidating_obstacles' in Edge.
// 0 = Use Standard RRTx (Lazy Add, Check-All Remove). Matches Julia reference.
// -----------------------------------------------------------------------------------------
/*
    The Threat Set (Node-level)
    The Invalidating Set (Edge-level)
    The Threat Set is the bridge that allows a lazy algorithm to behave with the same spatial intelligence as an eager one

*/
#define USE_INVALIDATING_SET_STRATEGY 0
#define USE_THREAT_SET_STRATEGY 0
// If both are 0, it falls back to the Default/Brute-Force Strategy


#define DEBUG 0


KinodynamicRRTX::KinodynamicRRTX(std::shared_ptr<StateSpace> statespace, 
    std::shared_ptr<ProblemDefinition> problem_def,
    std::shared_ptr<ObstacleChecker> obs_checker): statespace_(statespace), problem_(problem_def), obs_checker_(obs_checker){
        std::cout<<"KinodynamicRRTX constructor \n";
}


void KinodynamicRRTX::setStart(const Eigen::VectorXd& start) {
    auto index = statespace_->getNumStates();
    auto node = std::make_shared<RRTxNode>(statespace_->addState(start) ,  tree_.size());
    tree_.push_back(node);
    node->setTimeToGoal(0);
    node->setG(0);
    node->setLMC(0);
    std::cout << "KinodynamicRRTX: Start node created on Index: " << index << "\n";
}
void KinodynamicRRTX::setGoal(const Eigen::VectorXd& goal) {
    auto index = statespace_->getNumStates();
    auto node = std::make_shared<RRTxNode>(statespace_->addState(goal) ,  tree_.size());
    vbot_node_ = node.get();
    node->setTimeToGoal(goal(goal.size() - 1));
    tree_.push_back(node);
    std::cout << "KinodynamicRRTX: Goal node created on Index: " << index << "\n";
}


std::vector<Eigen::VectorXd> KinodynamicRRTX::getPathPositions() const {
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
    // // else {
    // //     RRTX_INFO("[DSTARLITE_Path_Assembly] Robot has found a valid anchor node in the tree.");
    // // }

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



void KinodynamicRRTX::clearPlannerState() {
    inconsistency_queue_.clear();
    for (auto& node : tree_) {
        node->disconnectFromGraph();
        node.reset();
    }
    tree_.clear();
    statespace_->reset();
    kdtree_.reset();
    Vc_T_.clear();
    sample_counter = 0;
}


void KinodynamicRRTX::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    auto start = std::chrono::high_resolution_clock::now();
    clearPlannerState();

    sample_counter = 0;


    visualization_ = visualization;



    num_of_samples_ = params.getParam<int>("num_of_samples");
    partial_update = params.getParam<bool>("partial_update");
    is_geometric_mode_ = params.getParam<bool>("is_geometric_mode", false);
    lower_bounds_ = problem_->getLowerBound();
    upper_bounds_ = problem_->getUpperBound();
    std::string kdtree_type = params.getParam<std::string>("kdtree_type");

    kd_dim = params.getParam<int>("kd_dim",2);


    if (kdtree_type == "NanoFlann"){
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

    std::cout << "KinodynamicRRTX setup complete: num_of_samples=" << num_of_samples_
                << ", bounds=[" << lower_bounds_ << ", " << upper_bounds_ << "]\n";


    std::cout << "--- \n";
    std::cout << "Taking care of the samples: \n \n";
    setStart(problem_->getStart());
    std::cout << "--- \n";
    setGoal(problem_->getGoal()); //robots current position

    // KD-TREE
    Eigen::MatrixXd all_samples = statespace_->getSamplesCopy();
    Eigen::MatrixXd spatial_samples_only = all_samples.leftCols(kd_dim).eval();
    // kdtree_->addPoints(spatial_samples_only);
    kdtree_->addPoints(all_samples);
    kdtree_->buildTree();
    ///////////////////Neighborhood Radius////////////////////////////////
    dimension_ = statespace_->getDimension();
    int d = dimension_;
    Eigen::VectorXd range = upper_bounds_ - lower_bounds_;
    double mu = range.prod(); // .prod() computes the product of all coefficients
    std::cout<<"mu "<<mu<<"\n";
    double zetaD = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
    gamma_ = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); //RRT star gamma

    // Since i want to put a cap on the number of samples and i want RRTX to be as close as to FMTX im gonna set step size (delta) to this:
    factor = params.getParam<double>("factor");
    std::cout<<"factor: "<<factor<<"\n";
    delta = params.getParam<double>("delta");
    delta = factor * gamma_ * std::pow(std::log(num_of_samples_) / num_of_samples_, 1.0 / d); // FOR COMPARISON WITH FMTX
    // delta = 15.0;
    std::cout << "delta: " << delta << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "---\n";

    // One is already added at goal and another at start location
    sample_counter = 2;
}

Eigen::VectorXd KinodynamicRRTX::saturate(const Eigen::VectorXd& newPoint, const Eigen::VectorXd& closestPoint, double delta) {
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

void KinodynamicRRTX::plan() {

    auto start = std::chrono::high_resolution_clock::now();
    while (sample_counter < num_of_samples_) {
        neighborhood_radius_ = shrinkingBallRadius();
        Eigen::VectorXd sample = Eigen::VectorXd::Random(dimension_);
        sample = (lower_bounds_.array() + (upper_bounds_ - lower_bounds_).array() * ((sample.array() + 1.0) / 2.0)).matrix();
        sample_counter++;
        std::vector<size_t> nearest_indices = kdtree_->knnSearch(sample.head(kd_dim), 1);
        RRTxNode* nearest_node = tree_[nearest_indices[0]].get();
        Eigen::VectorXd nearest_state = nearest_node->getStateValue();
        sample = saturate(sample, nearest_state, delta);
        bool node_added = false;
        if (obs_checker_->isObstacleFree(sample)) {
            node_added = extend(sample);
        }
        if (node_added) {
            RRTxNode* new_node = tree_.back().get();
            rewireNeighbors(new_node);
            new_node->setG(new_node->getLMC()); // RRTx julia implementation puts the new node in the queue instead so that reduce function would set it but that causes redundant rewiring and update lmc again!
            reduceInconsistency();
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(150));
        // visualizeTree();
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Planning time: " << duration.count() << " ms" << std::endl;
}


// STRATEGY 1: INVALIDATING SET (Edge-Level Caching + Local Broad-Phase)
#if USE_INVALIDATING_SET_STRATEGY
bool KinodynamicRRTX::extend(Eigen::VectorXd v) {
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
                    best_traj = shared_fwd_traj;
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
// STRATEGY 2: THREAT SET (Node-Level Filtering)

#elif USE_THREAT_SET_STRATEGY
bool KinodynamicRRTX::extend(Eigen::VectorXd v) {
    auto new_node = std::make_shared<RRTxNode>(statespace_->addState(v), tree_.size());
    auto neighbors = kdtree_->radiusSearch(new_node->getStateValue().head(kd_dim), neighborhood_radius_ + std::numeric_limits<double>::epsilon());
    
    double min_lmc = std::numeric_limits<double>::infinity();
    RRTxNode* best_parent = nullptr;
    std::shared_ptr<Trajectory> best_traj;

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
            new_node->threats_.push_back(&ob);
        }
    }

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

// bool KinodynamicRRTX::extend(Eigen::VectorXd v) {
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
//     double v_time = new_node->getTimeToGoal();

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
//         sample_counter--;
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
bool KinodynamicRRTX::extend(Eigen::VectorXd v) {
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
        sample_counter--;
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
    // Dereference best_traj to pass const Trajectory& to existing setParent function
    new_node->setParent(best_parent, best_traj); 
    new_node->setLMC(min_lmc);
    tree_.push_back(new_node);
    kdtree_->addPoint(new_node->getStateValue().head(kd_dim));
    kdtree_->buildTree();
    last_replan_metrics_.nodes_updated++;

    for (auto& eval : evaluated_edges) {
        if (!eval.neighbor) continue;

        if (eval.fwd_exists) {
            // FIX: Dereference the shared_ptr to match existing function signature
            // This creates a reference, avoiding a deep copy.
            new_node->addNeighbor(eval.neighbor, true, false, eval.fwd_traj);
            
            if (!eval.fwd_safe) {
                new_node->outgoingEdges().at(eval.neighbor).distance = std::numeric_limits<double>::infinity();
                if (eval.neighbor->incomingEdges().count(new_node.get())) {
                    eval.neighbor->incomingEdges().at(new_node.get()).distance = std::numeric_limits<double>::infinity();
                }
            }
        }

        if (eval.rev_exists) {
            // FIX: Dereference the shared_ptr
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




void KinodynamicRRTX::rewireNeighbors(RRTxNode* v) {
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

void KinodynamicRRTX::reduceInconsistency() {
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
        // Synchronize Cost and LMC 
        if (node->getG() != node->getLMC()) {
            node->setG(node->getLMC());
            last_replan_metrics_.nodes_updated++;
        }
    }
    // Debugging
    // std::cout << "Queue size after reduce: " << inconsistency_queue_.getHeap().size() << "\n";
}


double KinodynamicRRTX::shrinkingBallRadius() const {
    auto rad = factor * gamma_ * pow(log(tree_.size()) / tree_.size(), 1.0/dimension_);
    return std::min(rad, delta);
}

void KinodynamicRRTX::updateLMC(RRTxNode* v) {
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


/*
    IMPORTANT: imagine a scenario where C is a new node and just added to the graph and A is an old node neighbor to it!
    then A will be initial(original) to C
    and C will be temporary to A
    so eventually A will forget if it had any connection to C when we call cullNeighbors(A). but the most important thing is afterward
    when we call rewireNeighbors(C), C might become A's parent! there is not problem with that but you should be careful that there is no outgoing 
    edge from A to C (it got deleted in cullNeighbor) so that's why in setParent function we set the parent_trajectory as a separate variable since there is no
    finding it through A's outgoingNeighbors list. so in getPathPositions do not loop through A's (which is a child to C) outgoingNeighbors list. Just use the saved trajecotyr
    which we saved using C's incomingNeighbors list.  

*/


/*
    Imagine u is old node and v is new node so u would be original neighbor of v and v would be u's running
    when the rn shrinks and we call cullNeighbor for u for example the temp outgoing edge from u to v gets erased 
    but since it still exists in v's incoming neighbors this could cause an issue in the addnewobstalce process of making it inf 
    incase its on obstacle! imagine we are looping through u's outgoing neighbor in addNewObstacle function then we cant find v because   
    it got deleted in the cullNeighbor then we cant do a symmetric lines of code we do in the for loop to make the v's incoming edge to inf
    then in rewire process planner thinks that edge is safe but it isnt!   
    we either have to use a new flat map container in the rrtx node which i used this method or do another loop on incoming edges in the          
    addnewobstalce and removeobstacle. 
    I also think the julia implmentation of rrtx has this bug! but the pseudo code in the paper is fine.
    so all in all if we didnt amend like this then addNewObstacle couldnt find some dangered edges!
*/

void KinodynamicRRTX::cullNeighbors(RRTxNode* v) {
    // No existing edges can grow longer than the current radius.
    if (sample_counter >= num_of_samples_ - 1) {
        return; 
    }

    auto& outgoing = v->outgoingEdges();
    auto it = outgoing.begin();
    
    while (it != outgoing.end()) {
        RRTxNode* neighbor = it->first;
        auto& edge = it->second;
        double edge_cost = edge.cached_trajectory->cost;

        // Only evaluate culling if the edge is too long AND not the parent.
        if (edge_cost > (neighborhood_radius_ + std::numeric_limits<double>::epsilon()) && neighbor != v->getParent()) {

            // SYMMETRIC CULL: Evaluate removing 'v' from the neighbor's incoming list.
            auto& incoming = neighbor->incomingEdges();
            if (auto incoming_it = incoming.find(v); incoming_it != incoming.end()) {
                // Preserving birth-right connectivity
                if (!incoming_it->second.is_initial) {
                    incoming.erase(incoming_it);
                }
            }

            // SOURCE CULL: Evaluate removing the neighbor from v's active set.
            if (!edge.is_initial) {
                // Move it to the passive map (required for RRTX's obstacle repair)
                v->culled_outgoing_edges_[neighbor] = edge;
                // Remove from active map and advance iterator
                it = outgoing.erase(it);
                // outgoing.erase(it++); //for absl
                continue; 
            }
        }
        // If the edge was not culled, move to next
        ++it;
    }
}

void KinodynamicRRTX::verifyQueue(RRTxNode* node) {
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


void KinodynamicRRTX::verifyOrphan(RRTxNode* node) {
    if(node->in_queue_==true){
        inconsistency_queue_.remove(node);
        last_replan_metrics_.queue_operations++;
        // node->in_queue_=false;
    }
    Vc_T_.insert(node->getIndex());
}

void KinodynamicRRTX::propagateDescendants() {
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


void KinodynamicRRTX::visualizeTree() {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    if (!tree_.empty()) {
        edges.reserve(tree_.size());
    }
    std::vector<Eigen::VectorXd> tree_nodes;

    for (const auto& node_ptr : tree_) {
        RRTxNode* child_node = node_ptr.get();
        RRTxNode* parent_node = child_node->getParent();

        tree_nodes.push_back(node_ptr->getStateValue());

        // If a node has a parent, it forms a valid edge in the tree.
        if (parent_node) {
            
            // Add a single, straight-line edge from the parent's state to the child's state.
            // No need to check for intermediate points.
            edges.emplace_back(parent_node->getStateValue(), child_node->getStateValue());
        }
    }
    

    // visualization_->visualizeNodes(tree_nodes, "map", 
    //                         std::vector<float>{0.0f, 1.0f, 0.0f},  // Red for tree
    //                         "tree_nodes");
    
    visualization_->visualizeEdges(edges, "map");
}



void KinodynamicRRTX::visualizeTreeReal() {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    if (!tree_.empty()) {
        edges.reserve(tree_.size() * 50); 
    }
    
    std::vector<Eigen::VectorXd> tree_nodes;
    tree_nodes.reserve(tree_.size());

    int connected_nodes_count = 0;
    
    for (const auto& node_ptr : tree_) {
        RRTxNode* child_node = node_ptr.get();
        RRTxNode* parent_node = child_node->getParent();

        tree_nodes.push_back(child_node->getStateValue().head(3));

        if (child_node->getG() != std::numeric_limits<double>::infinity()) {
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



void KinodynamicRRTX::visualizePath(const std::vector<Eigen::VectorXd>& path_waypoints) {
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

////////////////////////////////////////////////////////////////////////////////////////
void KinodynamicRRTX::dumpTreeToCSV(const std::string& filename) const {
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

void KinodynamicRRTX::setRobotState(const Eigen::VectorXd& robot_state) {
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
    // HYSTERESIS LOGIC
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
        RRTX_WARN("[RRTx_Anchor] Status: NULL (Robot is lost or searching...)");
    }

    // INTERNAL DEBUG VISUALIZATION
    if (visualization_) {
        if (vbot_node_) {
            std::vector<Eigen::VectorXd> anchor_pt = { vbot_node_->getStateValue().head<2>() };
            visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");
        } 
    }

}

bool KinodynamicRRTX::isRobotSafe() {
    return (vbot_node_ != nullptr) && (vbot_node_->getG() != std::numeric_limits<double>::infinity());
}



bool KinodynamicRRTX::runCollisionForensics() {
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

//////////////////////////////////////EVENT BASED!!!////////////////////////////////////////
void KinodynamicRRTX::updateObstacleSamples(const ObstacleVector& turned_obstacles) {
    if (turned_obstacles.empty()) return;

    // last_replan_metrics_ = ReplanMetrics();
    
    // // Get the exact planning time for synchronization
    // double T_robot = robot_continuous_state_(robot_continuous_state_.size() - 1);

    // --- ROBOT TIME HANDLING ---
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
        
        // REMOVE OLD TUBE (Pruning)
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




    #if DEBUG
        runForensics();
        std::string cycle_report;
        if (hasCycleInParentGraph(cycle_report)) {
            RCLCPP_ERROR(rclcpp::get_logger("RRTX_DEBUG"), "CYCLE DETECTED IN PARENT GRAPH!\n%s", cycle_report.c_str());
            // Optionally: throw, pause simulation, visualize, etc.
        } else {
            RCLCPP_INFO(rclcpp::get_logger("RRTX_DEBUG"), "%s", cycle_report.c_str());
        }
    #endif




}

// STRATEGY 1: INVALIDATING SET (Edge-Level Caching)
#if USE_INVALIDATING_SET_STRATEGY

void KinodynamicRRTX::addNewObstacle(const Obstacle& ob) {
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

void KinodynamicRRTX::removeObstacle(const Obstacle& ob) {
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

void KinodynamicRRTX::addNewObstacle(const Obstacle& ob) {
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

void KinodynamicRRTX::removeObstacle(const Obstacle& ob) {
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

void KinodynamicRRTX::addNewObstacle(const Obstacle& ob) {
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

void KinodynamicRRTX::removeObstacle(const Obstacle& ob) {
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




bool KinodynamicRRTX::hasCycleInParentGraph(std::string& report) {
    std::unordered_set<int> visited;
    std::unordered_set<int> rec_stack;  // recursion stack for cycle detection

    report.clear();
    std::ostringstream oss;
    bool found_cycle = false;
    int cycle_count = 0;

    auto dfs = [&](auto&& self, RRTxNode* node, std::vector<int>& path) -> bool {
        if (!node) return false;

        int idx = node->getIndex();

        // Already in recursion stack → cycle found
        if (rec_stack.count(idx)) {
            found_cycle = true;
            cycle_count++;
            oss << "CYCLE DETECTED (length " << path.size() << "):\n";
            oss << "  Path: ";
            for (int p : path) {
                oss << p << " -> ";
            }
            oss << idx << " (back to " << idx << ")\n";

            // Also show the actual parent chain that closes the cycle
            oss << "  Closing edge: " << node->getParent()->getIndex() << " -> " << idx << "\n\n";
            return true;
        }

        if (visited.count(idx)) return false;

        visited.insert(idx);
        rec_stack.insert(idx);
        path.push_back(idx);

        bool cycle_found = false;
        if (RRTxNode* parent = node->getParent()) {
            if (self(self, parent, path)) {
                cycle_found = true;
            }
        }

        path.pop_back();
        rec_stack.erase(idx);

        return cycle_found;
    };

    // Check every node in the tree
    for (const auto& node_ptr : tree_) {
        RRTxNode* node = node_ptr.get();
        if (!node) continue;

        if (!visited.count(node->getIndex())) {
            std::vector<int> path;
            dfs(dfs, node, path);
        }
    }

    if (!found_cycle) {
        report = "No cycles detected in parent-pointer graph. Checked " + 
                 std::to_string(visited.size()) + " nodes.";
        return false;
    }

    report = "Found " + std::to_string(cycle_count) + " cycle(s):\n" + oss.str() +
             "\nTotal nodes checked: " + std::to_string(visited.size());
    return true;
}