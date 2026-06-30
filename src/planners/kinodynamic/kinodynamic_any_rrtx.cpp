// Copyright 2025 Soheil E.nia
// TODO: culled neighbor edges doesnt need to be in the orphan or edge dist inf! they are there just to find the incoming!
#include "motion_planning/planners/kinodynamic/kinodynamic_any_rrtx.hpp"
#include "motion_planning/planners/kinodynamic/time_cone_prune.hpp"  // TIME_CONE_PRUNED + master switch
/*
    The Threat Set (Node-level)
    The Invalidating Set (Edge-level)

*/
#define USE_INVALIDATING_SET_STRATEGY 0
#define USE_THREAT_SET_STRATEGY 0
// If both are 0, it falls back to the Default/Brute-Force Strategy

#define DEBUG 0
#define USE_RECOVERY 0 // Emergency Fallback

KinodynamicANYRRTX::KinodynamicANYRRTX(std::shared_ptr<StateSpace> statespace,
    std::shared_ptr<ProblemDefinition> problem_def,
    std::shared_ptr<ObstacleChecker> obs_checker): statespace_(statespace), problem_(problem_def), obs_checker_(obs_checker){
        std::cout<<"KinodynamicANYRRTX constructor \n";
}

// It sets the root of the tree in backward search
void KinodynamicANYRRTX::setStart(const Eigen::VectorXd& start) {
    root_state_index_ = statespace_->getNumStates();
    auto index = statespace_->getNumStates();
    auto node = std::make_unique<RRTxNode>(statespace_->addState(start) ,  tree_.size());
    node->setTimeToGoal(0);
    node->setG(0);
    node->setLMC(0);
    std::cout << "KinodynamicANYRRTX: Start node created on Index: " << index << " with value: " << node->getStateValue() << "\n";
    tree_.push_back(std::move(node));
}
// It sets the robot's location in backward search
void KinodynamicANYRRTX::setGoal(const Eigen::VectorXd& goal) {
    auto index = statespace_->getNumStates();
    auto node = std::make_unique<RRTxNode>(statespace_->addState(goal) ,  tree_.size());
    vbot_node_ = node.get();
    node->setTimeToGoal(goal(goal.size() - 1));
    std::cout << "KinodynamicANYRRTX: Goal node created on Index: " << index << " with value: "<< node->getStateValue() << "\n";
    tree_.push_back(std::move(node));
}



void KinodynamicANYRRTX::setCurrentRobotTime(double robot_time_) {
    T_robot = robot_time_;
}

std::vector<Eigen::VectorXd> KinodynamicANYRRTX::getPathPositions() const {
    if (!vbot_node_ || vbot_node_->getLMC() == std::numeric_limits<double>::infinity()) {
        RRTX_ERROR("[RRTX_Path_Assembly] Robot has no valid anchor node. Cannot build path.");
        return {}; // Return empty path
    }

    // Safety check on the cached bridge
    if (!current_bridge_trajectory_.is_valid || current_bridge_trajectory_.path_points.empty()) {
        RRTX_ERROR("RRTX_Path_Assembly: Cached bridge trajectory is invalid. Cannot build path");
        return {};
    }

    // Start the final path with the CACHED bridge trajectory! (Zero computation time)
    std::vector<Eigen::VectorXd> final_executable_path = current_bridge_trajectory_.path_points;

    // Traverse the rest of the tree from the anchor node using parent pointers.
    RRTxNode* child = vbot_node_;
    RRTxNode* parent = child->getParent();

    int steps = 0;
    const int max_steps = tree_.size();

    while (parent) {
        if (steps++ > max_steps) {
            RRTX_WARN("[RRTX_Path_Assembly] Cycle detected. Aborting.");
            break;
        }
        
        auto cached_traj = child->getParentTrajectory();
        if (cached_traj && cached_traj->is_valid && cached_traj->path_points.size() > 1) {
            final_executable_path.insert(final_executable_path.end(),
                                         cached_traj->path_points.begin() + 1,
                                         cached_traj->path_points.end());
        } else {
            // Recovery nodes have no parent chain, so just stop here.
            // This is expected behavior!
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


// void KinodynamicANYRRTX::injectTimePillarNodes(const Eigen::VectorXd& goal_state_val, int num_pillar_nodes) {
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
//         auto node = std::make_unique<RRTxNode>(state_ptr, tree_.size());
        
//         node->setTimeToGoal(t_val);
        
//         node->setLMC(0.0);
//         node->setG(0.0);
        
//         inconsistency_queue_.add(node.get(), 0.0);

//         time_pillar_indices_.insert(node->getIndex()); 

//         tree_.push_back(std::move(node));
//     }
// }

void KinodynamicANYRRTX::injectTimePillarNodes(const Eigen::VectorXd& goal_state_val, int num_pillar_nodes) {
    if (num_pillar_nodes <= 0) return;

    auto start_state_val_ = problem_->getGoal();
    // 1. Calculate minimum required time (using only X and Y coordinates)
    double dist = (goal_state_val.head(2) - start_state_val_.head(2)).norm();
    double min_arrival_time = dist / statespace_->getMaxVelocity();

    // 2. Get the maximum time budget from the problem bounds
    int time_dim_index = statespace_->getDimension() - 1;
    double max_time = upper_bounds_(time_dim_index); 

    // If max_time is less than min_arrival_time, the goal is physically unreachable.
    if (max_time <= min_arrival_time) {
        std::cerr << "Time budget is too strict. Goal is physically unreachable at max velocity.\n";
        return; 
    }

    // 3. Define the reachable time window
    double time_window = max_time - min_arrival_time;

    // 4. Inject the Time Pillars
    for (int i = 1; i <= num_pillar_nodes; ++i) {
        // Copy the goal state (This inherently copies vx=0, vy=0 for 5D if they are in the config)
        Eigen::VectorXd pillar_state = goal_state_val;

        // Distribute time evenly across the REACHABLE window
        // double t_val = min_arrival_time + (time_window / num_pillar_nodes) * i; 

        // 1. Calculate how long the journey takes (distributed between min_arrival and max_time)
        double travel_time = min_arrival_time + (time_window / num_pillar_nodes) * i; 
        
        // 2. Subtract travel_time from max_time because time decreases from the robot to the goal
        double t_val = max_time - travel_time; 

        pillar_state(time_dim_index) = t_val;

        auto state_ptr = statespace_->addState(pillar_state);
        auto node = std::make_unique<RRTxNode>(state_ptr, tree_.size());
        
        node->setTimeToGoal(t_val);
        
        node->setLMC(0.0);
        node->setG(0.0);
        
        // Use RRTx's specific queue
        inconsistency_queue_.add(node.get(), 0.0);

        time_pillar_indices_.insert(node->getIndex()); 

        tree_.push_back(std::move(node));
    }
}

void KinodynamicANYRRTX::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    std::cout << "------------------------------------------------------------\n";
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
    num_pillar_nodes_ = params.getParam<int>("num_pillar_nodes", 50);
    if (is_geometric_mode_) num_pillar_nodes_ = 0;

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
    // setGoal(problem_->getGoal()); //robots current position
    // Protect the original main root (T=0)
    time_pillar_indices_.insert(root_state_index_); 
    // Inject the rest of the Time Pillars (Backward search: start is the destination)
    if(!is_geometric_mode_) injectTimePillarNodes(problem_->getStart(), num_pillar_nodes_);

    time_pillar_indices_.insert(root_state_index_);
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
    neighborhood_radius_ = shrinkingBallRadius();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "------------------------------------------------------------\n";
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
    int successfully_added = 0;


    // if (!is_geometric_mode_ && dimension_ >= 3) {
    //     const double R = T_robot * statespace_->getMaxVelocity();
    //     // Degenerate footprint near goal: adding nodes in a collapsing start-disk
    //     // buys nothing and costs everything. Skip the batch.
    //     if (R < 2*neighborhood_radius_) return;   // tune threshold to taste
    // }

    // Raw-attempt cap: bounds the loop when the footprint is degenerate
    // (near goal, T_robot <= t_reach rejects almost every draw).
    // Tune the multiplier; large enough that it never bites in normal regions.
    const int max_attempts = num_of_samples_ * 20;
    int attempts = 0;

    // for (int i = 0; i < num_of_samples_; ++i) {
    while (successfully_added < num_of_samples_ && attempts < max_attempts){
        ++attempts;   // every iteration consumes an attempt, including the ones that `continue`

        // Calculate Radius
        neighborhood_radius_ = shrinkingBallRadius();

        // Sample a point (centralized strategy: Halton low-dispersion or i.i.d. uniform)
        Eigen::VectorXd sample = statespace_->sampleUnregistered(lower_bounds_, upper_bounds_);
        // if (!is_geometric_mode_ && dimension_ >= 3) {
        //     Eigen::Vector2d root_position_ = problem_->getStart().head(2);
        //     const int t_idx = dimension_ - 1;
        //     auto dist = (root_position_ - sample.head(2)).norm();
        //     const double t_reach = dist / statespace_->getMaxVelocity(); // minTimeToReachNode (time-to-goal model
        //     const double t_samp  = sample[t_idx];

        //     const bool too_early = (t_samp < t_reach);
        //     const bool behind    = (t_samp > T_robot);
        //     if ((too_early || behind) && T_robot > t_reach) {
        //         // Remap the ALREADY-DRAWN time into [t_reach, t_robot]
        //         // u is uniform in [0,1)
        //         const double u = (t_samp - lower_bounds_[t_idx]) /
        //                         (upper_bounds_[t_idx] - lower_bounds_[t_idx]);
        //         auto before = sample[t_idx];
        //         sample[t_idx] = t_reach + u * (T_robot - t_reach);
        //         // std::cout<<"BEFORE: "<<before<<", AFTER: "<<sample[t_idx]<<"\n";
        //     }
        // }
        if (!is_geometric_mode_ && dimension_ >= 3) {
            const int t_idx = dimension_ - 1;
            // One-sided goal-reachability cone (shared StateSpace helper).
            if (!statespace_->remapTimeToGoalCone(sample, Eigen::Vector2d(problem_->getStart().head(2)),
                                                  T_robot, lower_bounds_[t_idx], upper_bounds_[t_idx])) {
                continue;                 // do NOT increment successfully_added
            }
        }



        
        
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
                So you either do rewireNeighbor or do verifyQueue()
                mind that you shouldnt use the setG for the new sample node if you use the verify queue! or else you'd lose RRT* subgraph realization
                but if you do rewireNeighbor then its better to set the new samples G to avoid going to updateLMC and rewire neighbor again in reduceInconsistency
                becuse we are already updated the lmc of the new sample in extend and we already did a rewireNeighbor call for it here! so its a waste of cycles

            */
            rewireNeighbors(new_node); 
            new_node->setG(new_node->getLMC());


            // verifyQueue(new_node);

            reduceInconsistency();

            // We successfully added a node! Increment counter.
            successfully_added++;

        }
    }

    #if DEBUG
        runCollisionForensics();
        runCostForensics();
    #endif
    
}
// INVALIDATING SET (Edge-Level Caching + Local Broad-Phase)
#if USE_INVALIDATING_SET_STRATEGY
bool KinodynamicANYRRTX::extend(Eigen::VectorXd v) {
    auto new_node = std::make_unique<RRTxNode>(statespace_->addState(v), tree_.size());
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
    kdtree_->addPoint(new_node->getStateValue().head(kd_dim));
    kdtree_->buildTree(); // Build tree is an empty function in DynamicKdtree
    

    // for (auto& eval : evaluated_edges) {
    //     if (!eval.neighbor) continue;
    //     if (eval.fwd_exists) {
    //         new_node->addNeighbor(eval.neighbor, true, false, eval.fwd_traj);
    //         if (!eval.fwd_safe) {
    //             new_node->outgoingEdges().at(eval.neighbor).distance = std::numeric_limits<double>::infinity();
    //             new_node->outgoingEdges().at(eval.neighbor).invalidating_obstacles = eval.fwd_blockers;
    //             if (eval.neighbor->incomingEdges().count(new_node.get())) {
    //                 eval.neighbor->incomingEdges().at(new_node.get()).distance = std::numeric_limits<double>::infinity();
    //                 eval.neighbor->incomingEdges().at(new_node.get()).invalidating_obstacles = eval.fwd_blockers;
    //             }
    //         }
    //     }
    //     if (eval.rev_exists) {
    //         eval.neighbor->addNeighbor(new_node.get(), false, true, eval.rev_traj);
    //         if (!eval.rev_safe) {
    //             eval.neighbor->outgoingEdges().at(new_node.get()).distance = std::numeric_limits<double>::infinity();
    //             eval.neighbor->outgoingEdges().at(new_node.get()).invalidating_obstacles = eval.rev_blockers;
    //             if (new_node->incomingEdges().count(eval.neighbor)) {
    //                 new_node->incomingEdges().at(eval.neighbor).distance = std::numeric_limits<double>::infinity();
    //                 new_node->incomingEdges().at(eval.neighbor).invalidating_obstacles = eval.rev_blockers;
    //             }
    //         }
    //     }
    // }

    auto hitStatic = [](const std::vector<const Obstacle*>& blockers) {
        for (const Obstacle* ob_ptr : blockers) {
            if (!ob_ptr->is_dynamic) return true;
        }
        return false;
    };

    for (auto& eval : evaluated_edges) {
        if (!eval.neighbor) continue;

        if (eval.fwd_exists) {
            new_node->addNeighbor(eval.neighbor, true, false, eval.fwd_traj);

            if (!eval.fwd_safe) {
                bool fwd_static = hitStatic(eval.fwd_blockers);

                auto& out_edge = new_node->outgoingEdges().at(eval.neighbor);
                out_edge.distance = std::numeric_limits<double>::infinity();

                if (fwd_static) {
                    out_edge.permanently_blocked = true;
                    out_edge.invalidating_obstacles.clear();
                } else {
                    out_edge.invalidating_obstacles = eval.fwd_blockers;
                }

                if (eval.neighbor->incomingEdges().count(new_node.get())) {
                    auto& in_edge = eval.neighbor->incomingEdges().at(new_node.get());
                    in_edge.distance = std::numeric_limits<double>::infinity();

                    if (fwd_static) {
                        in_edge.permanently_blocked = true;
                        in_edge.invalidating_obstacles.clear();
                    } else {
                        in_edge.invalidating_obstacles = eval.fwd_blockers;
                    }
                }
            }
        }

        if (eval.rev_exists) {
            eval.neighbor->addNeighbor(new_node.get(), false, true, eval.rev_traj);

            if (!eval.rev_safe) {
                bool rev_static = hitStatic(eval.rev_blockers);

                auto& out_edge = eval.neighbor->outgoingEdges().at(new_node.get());
                out_edge.distance = std::numeric_limits<double>::infinity();

                if (rev_static) {
                    out_edge.permanently_blocked = true;
                    out_edge.invalidating_obstacles.clear();
                } else {
                    out_edge.invalidating_obstacles = eval.rev_blockers;
                }

                if (new_node->incomingEdges().count(eval.neighbor)) {
                    auto& in_edge = new_node->incomingEdges().at(eval.neighbor);
                    in_edge.distance = std::numeric_limits<double>::infinity();

                    if (rev_static) {
                        in_edge.permanently_blocked = true;
                        in_edge.invalidating_obstacles.clear();
                    } else {
                        in_edge.invalidating_obstacles = eval.rev_blockers;
                    }
                }
            }
        }
    }


    tree_.push_back(std::move(new_node));

    return true;
}
// THREAT SET (Node-Level Filtering)

#elif USE_THREAT_SET_STRATEGY
bool KinodynamicANYRRTX::extend(Eigen::VectorXd v) {
    auto new_node = std::make_unique<RRTxNode>(statespace_->addState(v), tree_.size());
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
    std::vector<bool> fwd_hit_static(neighbors.size(), false);
    std::vector<bool> rev_hit_static(neighbors.size(), false);

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
                    if (!threat_ptr->is_dynamic) {
                        fwd_hit_static[i] = true;
                    }
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
            rev_hit_static[i] = fwd_hit_static[i];
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
                        if (!threat_ptr->is_dynamic) {
                            rev_hit_static[i] = true;
                        }
                        break;
                    }
                }
            }
        }
    }

    // COMMIT GRAPH CHANGES
    new_node->setParent(best_parent, best_traj);
    new_node->setLMC(min_lmc);
    kdtree_->addPoint(new_node->getStateValue().head(kd_dim));
    kdtree_->buildTree(); 
    

    // for (auto& eval : evaluated_edges) {
    //     if (!eval.neighbor) continue;
    //     if (eval.fwd_exists) {
    //         new_node->addNeighbor(eval.neighbor, true, false, eval.fwd_traj);
    //         if (!eval.fwd_safe) {
    //             new_node->outgoingEdges().at(eval.neighbor).distance = std::numeric_limits<double>::infinity();
    //             if (eval.neighbor->incomingEdges().count(new_node.get())) {
    //                 eval.neighbor->incomingEdges().at(new_node.get()).distance = std::numeric_limits<double>::infinity();
    //             }
    //         }
    //     }
    //     if (eval.rev_exists) {
    //         eval.neighbor->addNeighbor(new_node.get(), false, true, eval.rev_traj);
    //         if (!eval.rev_safe) {
    //             eval.neighbor->outgoingEdges().at(new_node.get()).distance = std::numeric_limits<double>::infinity();
    //             if (new_node->incomingEdges().count(eval.neighbor)) {
    //                 new_node->incomingEdges().at(eval.neighbor).distance = std::numeric_limits<double>::infinity();
    //             }
    //         }
    //     }
    // }
    for (size_t i = 0; i < evaluated_edges.size(); ++i) {
        auto& eval = evaluated_edges[i];
        if (!eval.neighbor) continue;

        if (eval.fwd_exists) {
            new_node->addNeighbor(eval.neighbor, true, false, eval.fwd_traj);

            if (!eval.fwd_safe) {
                auto& out_edge = new_node->outgoingEdges().at(eval.neighbor);
                out_edge.distance = std::numeric_limits<double>::infinity();
                out_edge.permanently_blocked = fwd_hit_static[i];

                if (eval.neighbor->incomingEdges().count(new_node.get())) {
                    auto& in_edge = eval.neighbor->incomingEdges().at(new_node.get());
                    in_edge.distance = std::numeric_limits<double>::infinity();
                    in_edge.permanently_blocked = fwd_hit_static[i];
                }
            }
        }

        if (eval.rev_exists) {
            eval.neighbor->addNeighbor(new_node.get(), false, true, eval.rev_traj);

            if (!eval.rev_safe) {
                auto& out_edge = eval.neighbor->outgoingEdges().at(new_node.get());
                out_edge.distance = std::numeric_limits<double>::infinity();
                out_edge.permanently_blocked = rev_hit_static[i];

                if (new_node->incomingEdges().count(eval.neighbor)) {
                    auto& in_edge = new_node->incomingEdges().at(eval.neighbor);
                    in_edge.distance = std::numeric_limits<double>::infinity();
                    in_edge.permanently_blocked = rev_hit_static[i];
                }
            }
        }
    }


    tree_.push_back(std::move(new_node));

    return true;
}
// DEFAULT (Brute-Force Fallback)
#else

bool KinodynamicANYRRTX::extend(Eigen::VectorXd v) {
    auto new_node = std::make_unique<RRTxNode>(statespace_->addState(v), tree_.size());
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

    std::vector<bool> fwd_hit_static(neighbors.size(), false); // NEW
    std::vector<bool> rev_hit_static(neighbors.size(), false); // NEW

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
                    // CACHE STATIC WALL ---
                    if (!ob.is_dynamic) {
                        fwd_hit_static[i] = true;
                    }
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

        // Time-cone prune: an incoming edge u->v only lets u (tau(u) > tau(v)) adopt v as a
        // parent. If u is already beyond the robot's reachable cone it can never be on the
        // robot's path, so skip its (expensive) reverse steer AND the full obstacle-set
        // collision check below. EXACT prune; no-op in geometric mode. See time_cone_prune.hpp.
        // [Option B — behavior-preserving] DISABLED: this incoming-edge wiring is the relay
        // FEEDER (= FMTX addBatch 1601). It populates dead nodes' neighbour lists that
        // propagateDescendants later scans to seed live repair; pruning it perturbs the
        // realized tree (FMTX seed-88 root cause). Keep OFF.
        // if (TIME_CONE_PRUNED(u, T_robot)) continue;

        if (is_geometric_mode_) {
            // Geometric: Incoming is identical to Outgoing
            evaluated_edges[i].rev_exists = evaluated_edges[i].fwd_exists;
            evaluated_edges[i].rev_traj = evaluated_edges[i].fwd_traj; // Share pointer
            evaluated_edges[i].rev_safe = evaluated_edges[i].fwd_safe;
            rev_hit_static[i] = fwd_hit_static[i];
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
                        // --- CACHE STATIC WALL ---
                        if (!ob.is_dynamic) {
                            rev_hit_static[i] = true;
                        }

                        break;
                    }
                }
            }
        }
    }

    // COMMIT GRAPH CHANGES
    new_node->setParent(best_parent, best_traj); 
    new_node->setLMC(min_lmc);
    kdtree_->addPoint(new_node->getStateValue().head(kd_dim));
    kdtree_->buildTree();
    

    // for (auto& eval : evaluated_edges) {
    //     if (!eval.neighbor) continue;

    //     if (eval.fwd_exists) {
    //         new_node->addNeighbor(eval.neighbor, true, false, eval.fwd_traj);
            
    //         if (!eval.fwd_safe) {
    //             new_node->outgoingEdges().at(eval.neighbor).distance = std::numeric_limits<double>::infinity();
    //             if (eval.neighbor->incomingEdges().count(new_node.get())) {
    //                 eval.neighbor->incomingEdges().at(new_node.get()).distance = std::numeric_limits<double>::infinity();
    //             }
    //         }
    //     }

    //     if (eval.rev_exists) {
    //         eval.neighbor->addNeighbor(new_node.get(), false, true, eval.rev_traj);
            
    //         if (!eval.rev_safe) {
    //             eval.neighbor->outgoingEdges().at(new_node.get()).distance = std::numeric_limits<double>::infinity();
    //             if (new_node->incomingEdges().count(eval.neighbor)) {
    //                 new_node->incomingEdges().at(eval.neighbor).distance = std::numeric_limits<double>::infinity();
    //             }
    //         }
    //     }
    // }
    for (size_t i = 0; i < evaluated_edges.size(); ++i) {
        auto& eval = evaluated_edges[i];
        if (!eval.neighbor) continue;

        if (eval.fwd_exists) {
            new_node->addNeighbor(eval.neighbor, true, false, eval.fwd_traj);
            
            if (!eval.fwd_safe) {
                auto& out_edge = new_node->outgoingEdges().at(eval.neighbor);
                out_edge.distance = std::numeric_limits<double>::infinity();
                out_edge.permanently_blocked = fwd_hit_static[i]; // APPLY CACHE

                if (eval.neighbor->incomingEdges().count(new_node.get())) {
                    auto& in_edge = eval.neighbor->incomingEdges().at(new_node.get());
                    in_edge.distance = std::numeric_limits<double>::infinity();
                    in_edge.permanently_blocked = fwd_hit_static[i]; // APPLY CACHE
                }
            }
        }

        if (eval.rev_exists) {
            eval.neighbor->addNeighbor(new_node.get(), false, true, eval.rev_traj);
            
            if (!eval.rev_safe) {
                auto& out_edge = eval.neighbor->outgoingEdges().at(new_node.get());
                out_edge.distance = std::numeric_limits<double>::infinity();
                out_edge.permanently_blocked = rev_hit_static[i]; // APPLY CACHE

                if (new_node->incomingEdges().count(eval.neighbor)) {
                    auto& in_edge = new_node->incomingEdges().at(eval.neighbor);
                    in_edge.distance = std::numeric_limits<double>::infinity();
                    in_edge.permanently_blocked = rev_hit_static[i]; // APPLY CACHE
                }
            }
        }
    }


    tree_.push_back(std::move(new_node));

    return true;
}

#endif



void KinodynamicANYRRTX::rewireNeighbors(RRTxNode* v) {
    const double inconsistency = v->getG() - v->getLMC();
    if (inconsistency <= epsilon_) return;
    cullNeighbors(v);
    for (auto& [u, edge] : v->incomingEdges()) {
        if (u == v->getParent() ) continue;
        // Time-cone prune: offering v as a parent to a neighbor u beyond the robot's
        // reachable cone helps nobody on the robot's path and only churns the queue. Skip.
        if (TIME_CONE_PRUNED(u, T_robot)) continue;
        const double candidate_lmc = v->getLMC() + edge.distance;
        if (u->getLMC() > candidate_lmc) {
            u->setLMC(candidate_lmc);
            u->setParent(v, edge.cached_trajectory);
            
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
        RRTxNode* node = top_element.second;

        // Time-cone prune: a node beyond the robot's reachable cone can never affect the
        // robot's path (its cost is only ever read by even-higher-tau nodes), so leave it
        // inconsistent and skip its updateLMC/rewire. NEVER prune the robot anchor itself,
        // so the partial-update termination below stays well-defined. EXACT prune.
        if (node != vbot_node_ && TIME_CONE_PRUNED(node, T_robot)) {
            continue;
        }

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
            
        }
    }
    // Debugging
    // std::cout << "Queue size after reduce: " << inconsistency_queue_.getHeap().size() << "\n";
}



// double KinodynamicANYRRTX::shrinkingBallRadius() const {
//     int d = kd_dim;
//     auto rad = factor * gamma_ * pow(log(tree_.size()) / tree_.size(), 1.0/d);
//     return std::min(rad, delta);
// }
double KinodynamicANYRRTX::shrinkingBallRadius() const {

    // tree_.size() includes the root + injected time pillars, which are
    // co-located at the goal position and are NOT space-filling. Counting
    // them inflates N and shrinks r_n as if the space were denser than it is,
    // degrading connectivity. Exclude them so N reflects only the random,
    // space-filling samples.
    int N = static_cast<int>(tree_.size()) - num_pillar_nodes_;
    if (N == 0) return delta; // No real samples yet


    
    // Prevent log(1) = 0 by enforcing a minimum N of 2.
    // This gives the tree a valid initial volume to start growing.
    double safe_N = std::max(2, N);
    
    int d = kd_dim;
    double rad = factor * gamma_ * std::pow(std::log(safe_N) / safe_N, 1.0 / d);
    
    return std::min(rad, delta);
}


/*
    Why not start the min_lmc variable from INF? 
    This function is not for clearing LMC and recalculate it because RRTx speicifically has orphan handling 
    As opposed to D star which doesnt explicitly hadnles orphans. This function only updates to better LMC
    this difference between recomputeRHS in D starlite and updateLMC in RRTx
*/
void KinodynamicANYRRTX::updateLMC(RRTxNode* v) {
    // ROOT PROTECTION
    // The main goal and all Time Pillars are mathematical sinks.
    // They must NEVER recalculate their LMC. Their cost is eternally 0.0.
    if (time_pillar_indices_.find(v->getIndex()) != time_pillar_indices_.end()) {
        return;
    }
    // Time-cone prune (defensive): a node beyond the robot's reachable cone never needs a
    // recomputed parent. It should already be filtered upstream (reduceInconsistency /
    // removeObstacle), so this just documents and enforces the invariant. EXACT prune.
    if (TIME_CONE_PRUNED(v, T_robot)) {
        return;
    }
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
        
    } 
}


void KinodynamicANYRRTX::cullNeighbors(RRTxNode* v) {
    // static long long cull_count_ = 0;  // total successful culls this run


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
                // ++cull_count_;  // a real cull happened

                continue; // Node is culled; move to next iterator
            }
        }
        // Move to next neighbor if no cull occurred
        ++it;
    }
    v->last_culled_radius_ = neighborhood_radius_;
    // std::cout<<"count: "<<cull_count_<<"\n";
}




void KinodynamicANYRRTX::verifyQueue(RRTxNode* node) {
    // Time-cone prune: never enqueue a node beyond the robot's reachable cone; it would only
    // be popped and discarded by reduceInconsistency. The robot anchor (vbot_node_) is always
    // allowed in so the search can repair the path up to it. EXACT prune.
    if (node != vbot_node_ && TIME_CONE_PRUNED(node, T_robot)) {
        return;
    }

    const double min_key = std::min(node->getLMC(), node->getG());
    const double g_value = node->getG();


    if (node->in_queue_) {
        // Update both the priority and maintains g_value through node pointer
        inconsistency_queue_.update(node, min_key);
    } else {
        inconsistency_queue_.add(node, min_key);
        // node->in_queue_ = true;
    }
}


void KinodynamicANYRRTX::verifyOrphan(RRTxNode* node) {
    // ROOT PROTECTION
    // Pillars are the finish line. They cannot be "orphaned" because they
    // do not have parents. They must never be added to the invalidation set.
    if (time_pillar_indices_.find(node->getIndex()) != time_pillar_indices_.end()) {
        return;
    }
    if(node->in_queue_==true){
        inconsistency_queue_.remove(node);
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
            // Time-cone prune: descendants have strictly higher tau than their parent, so a
            // child beyond the robot's reachable cone (and its whole subtree) can never be on
            // the robot's path. Don't orphan it and don't descend into it. EXACT prune.
            // (Relevant descendants have tau <= T_robot and are still reached.)
            // [Option B — behavior-preserving] DISABLED: this is the orphan-cascade RELAY
            // (= FMTX addNewObstacle 2981). Skipping dead descendants drops the boundary
            // verifyQueue() seeding of their live neighbours, perturbing the tree. Keep OFF.
            // if (TIME_CONE_PRUNED(child, T_robot)) continue;
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
        
        /*
            An orphaned node doesn't move in time; it just has no path to the goal. 
            By keeping its time_to_goal at the original sampled value, the collision checker can still accurately check if it is blocked or clear
        */
        node->setParent(nullptr, std::shared_ptr<Trajectory>{});
    }


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
    //                         std::vector<float>{0.0f, 0.0f, 1.0f},  // Green color
    //                         "tree_nodes");
    

    if (vbot_node_) {
        
        std::vector<Eigen::VectorXd> anchor_pt = { vbot_node_->getStateValue().head<2>() };
        visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");
    }
    // std::cout<<"Tree Nodes: "<<tree_nodes.size()<<"\n";
    visualization_->visualizeEdges(edges, "map");
}



void KinodynamicANYRRTX::visualizeTreeReal() {
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
    if (vbot_node_) {
        std::vector<Eigen::VectorXd> anchor_pt = { vbot_node_->getStateValue().head<2>() };
        visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");
    } 
    visualization_->visualizeEdges(edges, "map");
}


void KinodynamicANYRRTX::visualizeTreeGradient() {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    std::vector<double> edge_costs;

    if (!tree_.empty()) {
        edges.reserve(tree_.size());
        edge_costs.reserve(tree_.size());
    }
    
    // Find max cost in the tree to normalize the gradient
    double max_tree_cost = 0.001; 
    for (const auto& node_ptr : tree_) {
        // RRTX uses getG() instead of getLMC()
        if (node_ptr->getG() > max_tree_cost && !std::isinf(node_ptr->getG())) {
            max_tree_cost = node_ptr->getG();
        }
    }

    for (const auto& node_ptr : tree_) {
        RRTxNode* child_node = node_ptr.get();
        RRTxNode* parent_node = child_node->getParent();

        if (parent_node && !std::isinf(child_node->getG())) {
            // Use the full states so visualization has access to Z or time if needed
            edges.emplace_back(parent_node->getStateValue(), child_node->getStateValue());
            
            // Normalize the cost between 0.0 and 1.0
            edge_costs.push_back(child_node->getG() / max_tree_cost); 
        }
    }
    if (vbot_node_) {
        std::vector<Eigen::VectorXd> anchor_pt = { vbot_node_->getStateValue().head<2>() };
        visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");
    } 
    
    visualization_->visualizeTreeGradient(edges, edge_costs, "map");
}

void KinodynamicANYRRTX::visualizePathGradient(const std::vector<Eigen::VectorXd>& path_waypoints) {
    // RRTX uses vbot_node_ as the anchor
    if (path_waypoints.size() < 2 || !visualization_ || vbot_node_ == nullptr) {
        return;
    }
    
    double robot_g = vbot_node_->getG();
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


// void KinodynamicANYRRTX::setRobotState(const Eigen::VectorXd& robot_state) {
//     robot_continuous_state_ = robot_state;
//     // Extract actual planner-time (Time-to-Go) from the state (last element)
//     double robot_time_to_go = robot_continuous_state_(robot_continuous_state_.size() - 1);

//     // QUERY POINT CONSTRUCTION
//     Eigen::VectorXd query_point = Eigen::VectorXd::Zero(kd_dim);
//     if (robot_continuous_state_.size() >= 2) {
//         query_point(0) = robot_continuous_state_(0);
//         query_point(1) = robot_continuous_state_(1);
//     }
//     if (kd_dim == 3) {
//         query_point(2) = robot_time_to_go;
//     } else if (kd_dim == 4) {
//         query_point(2) = robot_continuous_state_(2); 
//         query_point(3) = robot_time_to_go;
//     } else if (kd_dim == 5) {
//         query_point = robot_continuous_state_; 
//     }

//     // HYSTERESIS LOGIC
//     const double hysteresis_factor = 0.98;
//     double cost_of_current_path = std::numeric_limits<double>::infinity();
//     Trajectory bridge;
//     bool safe = true;
//     if (vbot_node_ && vbot_node_->getLMC() != std::numeric_limits<double>::infinity()) {
//         bridge = statespace_->steer(robot_continuous_state_, vbot_node_->getStateValue());

//         // Use robot_time_to_go so collision check is synced with the world
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
//                 cost_of_current_path = bridge.cost + vbot_node_->getLMC();
//                 robot_current_time_to_goal_ = bridge.time_duration + vbot_node_->getTimeToGoal();
//             }
//         }
//     }

//     RRTxNode* best_candidate_node = nullptr;
//     Trajectory best_candidate_bridge;
//     double best_candidate_cost = std::numeric_limits<double>::infinity();
    
//     // Radius Expansion fallback tracking (for unexplored nodes)
//     RRTxNode* best_fallback_node = nullptr;
//     Trajectory best_fallback_bridge;
//     double best_fallback_cost = std::numeric_limits<double>::infinity();

//     // Radius Expansion to handle sparse graphs
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
//             RRTxNode* candidate = tree_[idx].get();

//             // DO NOT SKIP based on getG() == infinity yet! We need it for the fallback check.

//             Trajectory temp_bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());
            
//             if (!temp_bridge.is_valid) continue;

//             bool safe = true;
//             const auto& obstacles = obs_checker_->getObstacles();
//             for (const auto& ob : obstacles) {
//                 last_replan_metrics_.obstacle_checks++;
//                 if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(temp_bridge, ob)) {
//                     safe = false;
//                     break;  // stop checking remaining obstacles for this bridge
//                 }
//             }

//             if (!safe) continue;

// #if USE_RECOVERY
//             // Fallback Tracking: Easiest safe physical node to reach
//             if (temp_bridge.cost < best_fallback_cost) {
//                 best_fallback_node = candidate;
//                 best_fallback_bridge = temp_bridge;
//                 best_fallback_cost = temp_bridge.cost;
//             }
// #endif

//             // Connected Tracking: Best node that ALREADY has a finite path
//             if (candidate->getLMC() != std::numeric_limits<double>::infinity()) {
//                 double cost = temp_bridge.cost + candidate->getLMC();
//                 if (cost < best_candidate_cost) {
//                     best_candidate_node = candidate;
//                     best_candidate_bridge = temp_bridge;
//                     best_candidate_cost = cost;
//                 }
//             }
//         }
        
//         // Only break early if we found a CONNECTED node.
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
//         if (vbot_node_) {
//             oss << "Current anchor LMC=" << vbot_node_->getLMC() << ", ";
//         } else {
//             oss << "No current anchor. ";
//         }
//         oss << "Fresh steer safe=" << safe << ", bridge valid=" << bridge.is_valid;
//         if (!bridge.is_valid && vbot_node_) {
//             oss << " (steer failed from " << robot_continuous_state_.head<2>().transpose()
//                 << " to " << vbot_node_->getStateValue().head<2>().transpose() << ")";
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


//     // ASSIGNMENT PRIORITY (Fixed order)
//     // 1) Better connected anchor
//     if (best_candidate_node &&
//         best_candidate_cost < cost_of_current_path * hysteresis_factor) {

//         vbot_node_ = best_candidate_node;
//         robot_current_time_to_goal_ = best_candidate_bridge.time_duration + best_candidate_node->getTimeToGoal();
//         last_replan_metrics_.path_cost = best_candidate_cost;
//         current_bridge_trajectory_ = best_candidate_bridge;
//         bridge_cost_ = best_candidate_bridge.cost;
//     } 
//     // 2) Keep current anchor with fresh bridge
//     else if (safe && vbot_node_ &&
//              cost_of_current_path != std::numeric_limits<double>::infinity() &&
//              bridge.is_valid) {

//         robot_current_time_to_goal_ = bridge.time_duration + vbot_node_->getTimeToGoal();
//         last_replan_metrics_.path_cost = cost_of_current_path;
//         current_bridge_trajectory_ = bridge;
//         bridge_cost_ = bridge.cost;
//     } 
//     // 3) Recovery: go to nearest safe tree node (HIGHER PRIORITY THAN CACHED REUSE)
// #if USE_RECOVERY
//     else if (best_fallback_node) {
//         // vbot_node_ = best_fallback_node;
//         robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
//         bridge_cost_ = best_fallback_cost;
//         current_bridge_trajectory_ = best_fallback_bridge;
//         last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();

//         RRTX_WARN("[RRTX_SetRobotState] USE_RECOVERY: Falling back to nearest safe tree node.");
//     }
// #endif
//     // // 4) Keep current anchor, reuse previous cached bridge
//     // //    This is the near-root / numerical-steer-failure case.
//     // else if (vbot_node_ &&
//     //          current_bridge_trajectory_.is_valid &&
//     //          !current_bridge_trajectory_.path_points.empty()) {

//     //     robot_current_time_to_goal_ =
//     //         current_bridge_trajectory_.time_duration + vbot_node_->getTimeToGoal();

//     //     // Keep finite so we do NOT enter trapped logic.
//     //     const double anchor_tail_cost =
//     //         (vbot_node_->getLMC() != std::numeric_limits<double>::infinity())
//     //             ? vbot_node_->getLMC()
//     //             : 0.0;

//     //     cost_of_current_path = current_bridge_trajectory_.cost + anchor_tail_cost;
//     //     last_replan_metrics_.path_cost = cost_of_current_path;
//     //     bridge_cost_ = current_bridge_trajectory_.cost;

//     //     RRTX_WARN("[RRTX_SetRobotState] Fresh steer failed near anchor/root. Reusing cached bridge.");
//     // }


//     else if (vbot_node_ &&
//          vbot_node_->getLMC() != std::numeric_limits<double>::infinity() &&
//          current_bridge_trajectory_.is_valid &&
//          !current_bridge_trajectory_.path_points.empty()) {

//         // 1. Build a re‑stamped version of the cached bridge
//         const auto& old_pts = current_bridge_trajectory_.path_points;
//         std::vector<Eigen::VectorXd> new_pts = old_pts;  // copy spatial points

//         // Original time step (assuming uniform spacing)
//         double dt_old = (old_pts.front()(2) - old_pts.back()(2)) / (old_pts.size() - 1);

//         // New time values: current robot_sim_time down to robot_sim_time - duration
//         double new_start = robot_time_to_go;                // <-- use robot_sim_time
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
//             vbot_node_ = nullptr;
//             robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
//             bridge_cost_ = std::numeric_limits<double>::infinity();
//             current_bridge_trajectory_ = Trajectory();
//             last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
//             RRTX_WARN("[Set Robot State] " << trap_reason());


//             RRTX_WARN("[Set Robot State] Cached bridge became unsafe. TRULY TRAPPED.");
//         } else {
//             // The cached bridge is still safe – reuse it.
//             robot_current_time_to_goal_ =
//                 current_bridge_trajectory_.time_duration + vbot_node_->getTimeToGoal();
//             const double anchor_tail_cost =
//                 (vbot_node_->getLMC() != std::numeric_limits<double>::infinity())
//                     ? vbot_node_->getLMC()
//                     : 0.0;
//             // cost_of_current_path = current_bridge_trajectory_.cost + anchor_tail_cost;
//             // last_replan_metrics_.path_cost = cost_of_current_path;
//             // bridge_cost_ = current_bridge_trajectory_.cost;

//             current_bridge_trajectory_ = re_stamped_bridge;   // <-- important
//             bridge_cost_ = re_stamped_bridge.cost;
//             robot_current_time_to_goal_ = re_stamped_bridge.time_duration + vbot_node_->getTimeToGoal();
//             cost_of_current_path = re_stamped_bridge.cost + anchor_tail_cost;
//             last_replan_metrics_.path_cost = cost_of_current_path;

//             RRTX_WARN("[Set Robot State] Fresh steer failed near anchor/root. Reusing cached bridge (verified safe).");
//         }
//     }


//     // 5) Truly trapped
//     else {
//         vbot_node_ = nullptr;
//         robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
//         bridge_cost_ = std::numeric_limits<double>::infinity();
//         current_bridge_trajectory_ = Trajectory();
//         last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();

//         RRTX_WARN("[RRTX_SetRobotState] LOST SAFE ANCHOR. TRULY TRAPPED.");
//     }

// }


bool KinodynamicANYRRTX::isRobotSafe() {
    // If vbot_node_ is null, we have no anchor.
    // If cost is std::numeric_limits<double>::infinity(), the anchor is invalid (trapped).
    return (vbot_node_ != nullptr) && (vbot_node_->getG() != std::numeric_limits<double>::infinity());
}

//////////////////////////////////////EVENT BASED!!!////////////////////////////////////////
void KinodynamicANYRRTX::updateObstacles(const ObstacleVector& turned_obstacles) {
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

        // FOR REMOVED STATIC OBS
        // PURGE: visible obstacle the robot reached. Free edges, erase, never re-add. --> Remove static obstalce!
        if (incoming_ob.is_removed) {
            previous_obstacles_.erase(incoming_ob.name);
            continue;
        }
        //////////////////

        // UPDATE STATE 
        stored_ob = incoming_ob; 

        // GENERATE NEW DENSE TUBE
        // We regenerate this here to ensure it uses the Planner's exact 'T_robot'.
        // Since we copied the object in step above, 'stored_ob' now has the valid physics data to do this.
        // stored_ob.predicted_path = obs_checker_->generatePrediction(stored_ob, T_robot);

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
        if (edge.permanently_blocked) return;
        last_replan_metrics_.obstacle_checks++;

        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ob)) {
            edge.distance = std::numeric_limits<double>::infinity();

            if (!ob.is_dynamic) {
                edge.permanently_blocked = true;
                edge.invalidating_obstacles.clear();
            } else {
                if (std::find(edge.invalidating_obstacles.begin(),
                            edge.invalidating_obstacles.end(), &ob) == edge.invalidating_obstacles.end()) {
                    edge.invalidating_obstacles.push_back(&ob);
                }
            }

            if (neighbor->incomingEdges().count(node)) {
                auto& inc_edge = neighbor->incomingEdges().at(node);
                inc_edge.distance = std::numeric_limits<double>::infinity();
                if (!ob.is_dynamic) {
                    inc_edge.permanently_blocked = true;
                    inc_edge.invalidating_obstacles.clear();
                } else {
                    if (std::find(inc_edge.invalidating_obstacles.begin(),
                                inc_edge.invalidating_obstacles.end(), &ob) == inc_edge.invalidating_obstacles.end()) {
                        inc_edge.invalidating_obstacles.push_back(&ob);
                    }
                }
            }
            
            // if (neighbor->getParent() == node) verifyOrphan(neighbor);
            if (node->getParent() == neighbor) verifyOrphan(node);

            if (is_geometric_mode_ && neighbor->outgoingEdges().count(node)) {
                auto& rev_edge = neighbor->outgoingEdges().at(node);
                if (rev_edge.distance != std::numeric_limits<double>::infinity()) {
                    rev_edge.distance = std::numeric_limits<double>::infinity();
                    if (!ob.is_dynamic) {
                        rev_edge.permanently_blocked = true;
                        rev_edge.invalidating_obstacles.clear();
                    } else {
                        if (std::find(rev_edge.invalidating_obstacles.begin(),
                                    rev_edge.invalidating_obstacles.end(), &ob) == rev_edge.invalidating_obstacles.end()) {
                            rev_edge.invalidating_obstacles.push_back(&ob);
                        }
                    }

                    if (node->incomingEdges().count(neighbor)) {
                        auto& rev_inc = node->incomingEdges().at(neighbor);
                        rev_inc.distance = std::numeric_limits<double>::infinity();
                        if (!ob.is_dynamic) {
                            rev_inc.permanently_blocked = true;
                            rev_inc.invalidating_obstacles.clear();
                        } else {
                            if (std::find(rev_inc.invalidating_obstacles.begin(),
                                        rev_inc.invalidating_obstacles.end(), &ob) == rev_inc.invalidating_obstacles.end()) {
                                rev_inc.invalidating_obstacles.push_back(&ob);
                            }
                        }
                    }

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
        if (edge.permanently_blocked) return;
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
        if (edge.permanently_blocked) return;
        if (edge.distance == std::numeric_limits<double>::infinity()) return; 

        last_replan_metrics_.obstacle_checks++;

        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ob)) {
            edge.distance = std::numeric_limits<double>::infinity();
            if (!ob.is_dynamic) {
                edge.permanently_blocked = true;
            } 

            if (neighbor->incomingEdges().count(node)) {
                auto& inc_edge = neighbor->incomingEdges().at(node);
                inc_edge.distance = std::numeric_limits<double>::infinity();
                if (!ob.is_dynamic) {
                    inc_edge.permanently_blocked = true;
                }
            }
            
            // if (neighbor->getParent() == node) verifyOrphan(neighbor);
            if (node->getParent() == neighbor) verifyOrphan(node);

            if (is_geometric_mode_ && neighbor->outgoingEdges().count(node)) {
                auto& rev_edge = neighbor->outgoingEdges().at(node);
                if (rev_edge.distance != std::numeric_limits<double>::infinity()) {
                    rev_edge.distance = std::numeric_limits<double>::infinity();
                    if (!ob.is_dynamic) {
                        rev_edge.permanently_blocked = true;
                    }

                    if (node->incomingEdges().count(neighbor)) {
                        auto& rev_inc = node->incomingEdges().at(neighbor);
                        rev_inc.distance = std::numeric_limits<double>::infinity();
                        if (!ob.is_dynamic) {
                            rev_inc.permanently_blocked = true;
                        }
                    }

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
        if (edge.permanently_blocked) return;
        if (edge.distance == std::numeric_limits<double>::infinity()) {
            bool is_safe = true;
            
            // Iteration through contiguous pointers
            for (const Obstacle* threat_ptr : node->threats_) {
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), *threat_ptr)) {
                    is_safe = false;
                    if (!threat_ptr->is_dynamic) {
                        edge.permanently_blocked = true;

                        if (neighbor->incomingEdges().count(node)) {
                            neighbor->incomingEdges().at(node).permanently_blocked = true;
                        }

                        if (is_geometric_mode_ && neighbor->outgoingEdges().count(node)) {
                            auto& rev_edge = neighbor->outgoingEdges().at(node);
                            rev_edge.permanently_blocked = true;

                            if (node->incomingEdges().count(neighbor)) {
                                node->incomingEdges().at(neighbor).permanently_blocked = true;
                            }
                        }
                    }
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
        if (edge.permanently_blocked) return;
        if (edge.distance == std::numeric_limits<double>::infinity()) return; 

        last_replan_metrics_.obstacle_checks++;

        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ob)) {
            edge.distance = std::numeric_limits<double>::infinity();
            // --- CACHE NEW STATIC WALLS ---
            if (!ob.is_dynamic) {
                edge.permanently_blocked = true;
            }

            
            if (neighbor->incomingEdges().count(node)) {
                auto& inc_edge = neighbor->incomingEdges().at(node);
                inc_edge.distance = std::numeric_limits<double>::infinity();
                if (!ob.is_dynamic) {
                    inc_edge.permanently_blocked = true;
                }
            }
            
            // if (neighbor->getParent() == node) verifyOrphan(neighbor);
            if (node->getParent() == neighbor) verifyOrphan(node);

            if (is_geometric_mode_ && neighbor->outgoingEdges().count(node)) {
                auto& rev_edge = neighbor->outgoingEdges().at(node);
                if (rev_edge.distance != std::numeric_limits<double>::infinity()) {
                    rev_edge.distance = std::numeric_limits<double>::infinity();
                    if (!ob.is_dynamic) {
                        rev_edge.permanently_blocked = true;
                    }

                    if (node->incomingEdges().count(neighbor)) {
                        auto& rev_inc = node->incomingEdges().at(neighbor);
                        rev_inc.distance = std::numeric_limits<double>::infinity();
                        if (!ob.is_dynamic) {
                            rev_inc.permanently_blocked = true;
                        }
                    }

                    if (neighbor->getParent() == node) verifyOrphan(neighbor);
                }
            }
        }
    };
    // int count = 0;
    for (int idx : unique_node_indices) {
        RRTxNode* node = tree_[idx].get();
        // Time-cone prune: every edge out of `node` originates at tau(node); if that exceeds
        // the robot's budget the edge can never be on the robot's path (and `node` can never
        // be a relevant node's parent), so skip all its collision checks. EXACT prune.
        if (TIME_CONE_PRUNED(node, T_robot)) {
            // count++;
            continue;
        }
        for (auto& [neighbor, edge] : node->outgoingEdges()) checkAndBlockEdge(node, neighbor, edge);
        for (auto& [neighbor, edge] : node->culled_outgoing_edges_) checkAndBlockEdge(node, neighbor, edge);
    }
    // std::cout<<"ADD OBS -- "<<"checked: "<<unique_node_indices.size()-count << " , ignored: "<<count<<"\n";
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

            // --- 1. STATIC CACHE BYPASS (Instant CPU Savings) ---
            if (edge.permanently_blocked) return; 

            bool should_restore = false;
            last_replan_metrics_.obstacle_checks++;
            if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), ob)) {
                bool conflicts_with_other = false;
                for (const auto& other_ob : all_obstacles) {
                    if (other_ob.name == ob.name) continue; 
                    last_replan_metrics_.obstacle_checks++;
                    if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(edge.cached_trajectory), other_ob)) {
                        conflicts_with_other = true;
                        // --- 2. CACHE STATIC WALLS FOUND DURING RESTORE ---
                        if (!other_ob.is_dynamic) {
                            edge.permanently_blocked = true;

                            if (neighbor->incomingEdges().count(node)) {
                                neighbor->incomingEdges().at(node).permanently_blocked = true;
                            }

                            if (is_geometric_mode_ && neighbor->outgoingEdges().count(node)) {
                                auto& rev_edge = neighbor->outgoingEdges().at(node);
                                rev_edge.permanently_blocked = true;

                                if (node->incomingEdges().count(neighbor)) {
                                    node->incomingEdges().at(neighbor).permanently_blocked = true;
                                }
                            }
                        }

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
    // int count = 0;
    for (int idx : unique_node_indices) {
        RRTxNode* node = tree_[idx].get();
        // Time-cone prune: restoring edges and recomputing the cost of a node beyond the
        // robot's reachable cone helps nobody on the robot's path. Skip its collision checks.
        // Relevant nodes in the freed region (tau <= T_robot) are still restored. EXACT prune.
        if (TIME_CONE_PRUNED(node, T_robot)) {
            // count++;
            continue;
        }
        bool neighborsWereBlocked = false;

        for (auto& [neighbor, edge] : node->outgoingEdges()) checkAndRestoreEdge(node, neighbor, edge, neighborsWereBlocked);
        for (auto& [neighbor, edge] : node->culled_outgoing_edges_) checkAndRestoreEdge(node, neighbor, edge, neighborsWereBlocked);

        if (neighborsWereBlocked) {
            updateLMC(node);
            if (node->getG() != node->getLMC()) verifyQueue(node);
        }
    }
    // std::cout<<"REM OBS -- "<<"checked: "<<unique_node_indices.size()-count << " , ignored: "<<count<<"\n";
}

#endif



void KinodynamicANYRRTX::setRobotState(const Eigen::VectorXd& robot_state) {
    robot_continuous_state_ = robot_state;
    double robot_time_to_go = robot_continuous_state_(robot_continuous_state_.size() - 1);

    // QUERY POINT CONSTRUCTION
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

    RRTxNode* best_candidate_node = nullptr;
    Trajectory best_candidate_bridge;
    double best_candidate_cost = std::numeric_limits<double>::infinity();
    
    RRTxNode* best_fallback_node = nullptr;
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
            if (!tested_indices.insert(idx).second) continue;

            RRTxNode* candidate = tree_[idx].get();

            // Time-cone prune: the bridge steer(robot -> candidate) needs tau(candidate) <
            // robot_time_to_go (time strictly decreases), so a candidate beyond the robot's
            // remaining budget can never be a valid anchor. Skip the steer attempt entirely.
            // (Conservative: steer would reject it anyway; this just saves the call.)
            if (TIME_CONE_PRUNED(candidate, robot_time_to_go)) continue;

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
            // Fallback Tracking: Easiest safe physical node to reach
            if (temp_bridge.cost < best_fallback_cost) {
                best_fallback_node = candidate;
                best_fallback_bridge = temp_bridge;
                best_fallback_cost = temp_bridge.cost;
            }
#endif

            // Connected Tracking: Best node that ALREADY has a finite path
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
        
        // Break early if we found a strictly CONNECTED node.
        if (best_candidate_node) break;
        current_search_radius *= radius_multiplier;
    }

    // 2. SIMPLE ASSIGNMENT LOGIC
    if (best_candidate_node) {
        vbot_node_ = best_candidate_node;
        robot_current_time_to_goal_ = best_candidate_bridge.time_duration + best_candidate_node->getTimeToGoal();
        last_replan_metrics_.path_cost = best_candidate_cost;
        current_bridge_trajectory_ = best_candidate_bridge;
        bridge_cost_ = best_candidate_bridge.cost;
    } 
#if USE_RECOVERY
    else if (best_fallback_node) {
        // vbot_node_ = best_fallback_node; // Uncomment if RRTX architecture requires assigning vbot_node_ here
        robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
        bridge_cost_ = best_fallback_cost;
        current_bridge_trajectory_ = best_fallback_bridge;
        last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
        RRTX_WARN("[RRTX_SetRobotState] USE_RECOVERY: Falling back to nearest safe tree node.");
    }
#endif
    else {
        vbot_node_ = nullptr;
        robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
        bridge_cost_ = std::numeric_limits<double>::infinity();
        current_bridge_trajectory_ = Trajectory();
        last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
        RRTX_WARN("[RRTX_SetRobotState] LOST SAFE ANCHOR. TRULY TRAPPED.");
    }




    // // one line per replan
    // double t_rem = upper_bounds_(upper_bounds_.size() - 1) - robot_time_to_go;
    // RRTX_INFO("[Traj]"
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


bool KinodynamicANYRRTX::isCurrentBridgeSafe(const ObstacleVector& obstacles) const {
    if (!current_bridge_trajectory_.is_valid || current_bridge_trajectory_.path_points.empty()) {
        // RRTX_WARN("[Bridge Check] Bridge is empty or invalid!");
        return false; 
    }

    // 1. Check immediate bridge collision
    for (const auto& ob : obstacles) {
        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(current_bridge_trajectory_, ob)) {
            // RRTX_WARN("[Bridge Check] IMMEDIATE COLLISION! Bridge intersects with obstacle: " << ob.name);
            return false; 
        }
    }

    // 2. THE ROOT CAUSE CHECK: Is the anchor node still safely connected to the goal?
    // In RRTX, rewire/cascade updates might have set our vbot_node_ LMC to INF!
    if (vbot_node_ != nullptr) {
        if (vbot_node_->getLMC() == std::numeric_limits<double>::infinity()) {
            // RRTX_WARN("[Bridge Check] GRAPH DESYNC! The immediate bridge is safe, but the anchor node (LMC=INF) has been cut off!");
            return false; // The bridge leads to a dead end!
        }
    } else {
        // RRTX_WARN("[Bridge Check] GRAPH DESYNC! vbot_node_ is NULL!");
        return false;
    }

    return true;
}

bool KinodynamicANYRRTX::hasReachedAnchor(const Eigen::VectorXd& current_sim_state) const {
    // If we don't have an anchor, trigger a new search
    if (!vbot_node_) return true; 
    if (vbot_node_->getLMC() == std::numeric_limits<double>::infinity()) return true; 

    // Calculate the time remaining on the current edge.
    // T_robot is the time left in the simulation budget.
    double current_T_robot = current_sim_state(current_sim_state.size() - 1);
    
    // The anchor node's T_robot
    double anchor_T_robot = vbot_node_->getStateValue()(current_sim_state.size() - 1);

    // We consider the anchor "reached" if we are within a small temporal threshold 
    // of the node's timestamp (e.g., 0.1 seconds away from the node).
    if (current_T_robot <= anchor_T_robot + 1e-6) {
        return true;
    }

    return false;
}

bool KinodynamicANYRRTX::hasShortcut(const Eigen::VectorXd& robot_state, double threshold) {
    if (!vbot_node_ || vbot_node_->getLMC() == std::numeric_limits<double>::infinity()) {
        return false;
    }

    double current_cost = bridge_cost_ + vbot_node_->getLMC();
    if (current_cost <= 1e-9 || current_cost == std::numeric_limits<double>::infinity()) {
        return false;
    }

    double robot_time_to_go = robot_state(robot_state.size() - 1);

    Eigen::VectorXd query_point = Eigen::VectorXd::Zero(kd_dim);
    if (robot_state.size() >= 2) {
        query_point(0) = robot_state(0);
        query_point(1) = robot_state(1);
    }
    if (kd_dim == 3) {
        query_point(2) = robot_time_to_go;
    } else if (kd_dim == 4) {
        query_point(2) = robot_state(2);
        query_point(3) = robot_time_to_go;
    } else if (kd_dim == 5) {
        query_point = robot_state;
    }

    double current_search_radius = neighborhood_radius_ * 2.0;
    const int max_attempts = 5;
    const double radius_multiplier = 2.0;

    std::unordered_set<size_t> tested_indices;

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        auto nearby_indices = kdtree_->radiusSearch(query_point, current_search_radius);

        for (auto idx : nearby_indices) {
            if (!tested_indices.insert(idx).second) continue;

            RRTxNode* candidate = tree_[idx].get();
            if (candidate->getLMC() == std::numeric_limits<double>::infinity()) continue;

            Trajectory temp_bridge = statespace_->steer(robot_state, candidate->getStateValue());
            if (!temp_bridge.is_valid) continue;

            double new_total_cost = temp_bridge.cost + candidate->getLMC();
            double improvement = (current_cost - new_total_cost) / current_cost;

            if (improvement >= threshold) {
                bool safe = true;
                for (const auto& [name, ob] : previous_obstacles_) {
                    if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(temp_bridge, ob)) {
                        safe = false;
                        break;
                    }
                }

                if (safe) {
                    return true;
                }
            }
        }

        current_search_radius *= radius_multiplier;
    }

    return false;
}

std::vector<Eigen::VectorXd> KinodynamicANYRRTX::getLivePathPositions(const Eigen::VectorXd& current_state) const
{
    if (!vbot_node_ || vbot_node_->getLMC() == std::numeric_limits<double>::infinity()) {
        return {}; 
    }

    // 1. Create a temporary, real-time bridge from the robot to the anchor
    Trajectory live_bridge = statespace_->steer(current_state, vbot_node_->getStateValue());
    
    if (!live_bridge.is_valid || live_bridge.path_points.empty()) {
        // If the live steer fails (e.g. numerical issue very close to the node),
        // fallback to the cached path
        return getPathPositions(); 
    }

    // 2. Start the path with the live bridge
    std::vector<Eigen::VectorXd> final_executable_path = live_bridge.path_points;

    // 3. Traverse the rest of the tree from the anchor node
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
