// Copyright 2025 Soheil E.nia

// TODO: Later implement KNN. with knn you wouldnt need cullNeighbor! use if (use_knn) return in cullNeighbor
#define DEBUG 1

// Set to 1 to use my context-aware Threat Set.
// Set to 0 to use the Default/Blind exhaustive checking.
#define USE_THREAT_SET_STRATEGY 0
// The Threat Set is the bridge that allows a lazy algorithm (like FMTx) to behave with the same spatial intelligence as an eager one (Eager like RRTx)

#include "motion_planning/planners/kinodynamic/kinodynamic_any_fmtx.hpp"

KinodynamicANYFMTX::KinodynamicANYFMTX(std::shared_ptr<StateSpace> statespace ,std::shared_ptr<ProblemDefinition> problem_def, std::shared_ptr<ObstacleChecker> obs_checker) :  statespace_(statespace), problem_(problem_def), obs_checker_(obs_checker) {
    std::cout<< "KinodynamicANYFMTX Constructor \n";

}

void KinodynamicANYFMTX::clearPlannerState() {

    v_open_heap_.clear(); //This function also makes the in_queue false so remember to clean it before deleting the node by node.reset()
    // Nullify all raw pointers --> or else if you only use tree_.clear() you have dangling pointers for parent_ and children_ that do not exist now! --> I'm using sharedptr now
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
    auto start = std::chrono::high_resolution_clock::now();
    clearPlannerState();
    visualization_ = visualization;
    num_of_samples_ = params.getParam<int>("num_of_samples");
    partial_update = params.getParam<bool>("partial_update");
    use_heuristic= params.getParam<bool>("use_heuristic");
    is_geometric_mode_ = params.getParam<bool>("is_geometric_mode", false);
    epsilon = params.getParam<double>("epsilon", 1e-4);
    lower_bounds_ = problem_->getLowerBound();
    upper_bounds_ = problem_->getUpperBound();
    use_kdtree = params.getParam<bool>("use_kdtree");
    kd_dim = params.getParam<int>("kd_dim", 2);
    std::string kdtree_type = params.getParam<std::string>("kdtree_type");
    use_knn = params.getParam<bool>("use_knn", false);

    if (use_kdtree == true && kdtree_type == "NanoFlann"){
        Eigen::VectorXd weights(kd_dim);
        // weights << 1.0, 1.0, 1.0; // Weights for x, y, time
        switch (kd_dim) {
            case 2: // (x, y)
                weights << 1.0, 1.0; // Weights for x, y,
                break;
            case 3: // (x, y, time)
                weights << 1.0, 1.0, 1.0; // Weights for x, y, time
                break;
            case 4: // (x, y, theta, time) - From your Dubins example
                weights << 1.0, 1.0, 1.0, 1.0; // Weights for x, y, theta, time
                break;
            case 5:
                weights << 1.0, 1.0, 1.0, 1.0, 1.0; // Weights for x, y, vx, vy, time
                break;
            default: 
                FMTX_ERROR("Unsupported k-d tree dimension: " << kd_dim);
        }
        kdtree_ = std::make_shared<DynamicWeightedNanoFlann>(kd_dim, weights);
    } else if (use_kdtree == true && kdtree_type == "LieKDTree"){
        kdtree_ = std::make_unique<LieSplittingKDTree>(statespace_->getDimension(), statespace_);
    } else {
        throw std::runtime_error("FMTX requires a KD-Tree.");
    }
    std::cout << "num_of_samples=" << num_of_samples_
                << ", bounds=[" << lower_bounds_ << ", " << upper_bounds_ << "]\n";


    std::cout << "Taking care of the samples: \n \n";
    setStart(problem_->getStart());
    setGoal(problem_->getGoal());






    std::cout << "KDTree: \n\n";
    if (use_kdtree == true) {
        // // Put all the points at once because fmtx doesnt need incremental addition
        // kdtree_->addPoints(statespace_->getSamplesCopy());
        // // Build the tree all at once after we fill the data_ in the KDTree
        // kdtree_->buildTree();

        // Get the full 3D (or 4D) samples from the state space.
        Eigen::MatrixXd all_samples = statespace_->getSamplesCopy();

        // // Use .leftCols() to create a new matrix with only the kd_dim data.
        // //    .eval() is used to ensure we pass a concrete matrix, not a temporary expression.
        Eigen::MatrixXd spatial_samples_only = all_samples.leftCols(kd_dim).eval();
        
        // Pass the 2D spatial matrix to the KD-tree.
        kdtree_->addPoints(spatial_samples_only);
        // kdtree_->addPoints(all_samples);
        
        // Build the tree all at once after we fill the data.
        kdtree_->buildTree();
        // kdtree_->printData();

    }

    dimension_ = statespace_->getDimension();
    factor = params.getParam<double>("factor");
    delta = params.getParam<double>("delta");
    // Calculate initial radius based on N=2 (Start + Goal)
    updateNeighborhoodRadius();

    // Enable incremental mode

    std::cout << "Setup complete. Ready for incremental sampling.\n";



    // ///////////////////Neighborhood Radius////////////////////////////////
    // /*
    //     One thing i notices in R2T case Specifically is the neighbor serach that happens in plan function is alot! well thats what fmt* is about as opposed to rrt*
    //     on the other hand it gives us an advantage on number of obstacle checks. 
    //     In R2T since the possiblities of connections are much more than dubins or thrusters then these big number of loops could take a hit since we also do not filter the 
    //     conflicting nodes (findSamplesNearObstalces function) with isTrajectorySafe, which ofcourse we can as a feature but its not inherent from fmt core logics!
    //     so we can utilize the knn approach to put a theoretical cap on the number of neighbors! another thing to mention is that maybe when you use rrtx samples in fmtx
    //     the number of neighbors get a bit biased toward the root! (because thats how rrt star's saturate works) and maybe if we use the natural fmt star based connection 
    //     this problem can be avoided. but all in all knn is a nice tool in our arsenal which rrtx doesn't seem to have. for instance the whole cullNeighbor Idea
    //     works based on Neighborhood radius.
    //     So all in all the TOTAL_NEIGHBOR_ITERATIONS is expected to reduce when we use knn
    // */
    // if (use_knn) {
    //     int d = statespace_->getDimension();
    //     factor = params.getParam<double>("factor");
    //     // Practical k-NN parameter from the FMT* paper's experiments 
    //     double k0_fmt_star_practical = std::pow(2.0, d) * (M_E / d);
    //     k_neighbors_ = static_cast<int>(std::ceil(factor * k0_fmt_star_practical * std::log(statespace_->getNumStates())));
    //     // // // Standard k-NN parameter for RRT*
    //     // double k0_rrt_star = M_E * (1.0 + 1.0 / d);
    //     // k_neighbors_ = static_cast<int>(std::ceil(factor * k0_rrt_star * std::log(statespace_->getNumStates())));
    //     std::cout << "k-NN formula. k = " << k_neighbors_ << "\n";
    // } else {
    //     int d = statespace_->getDimension();
    //     // mu = std::pow(problem_->getUpperBound()[0] - problem_->getLowerBound()[0] , 2);
    //     Eigen::VectorXd range = upper_bounds_ - lower_bounds_;
    //     double mu = range.prod(); // .prod() computes the product of all coefficients
    //     std::cout<<"mu "<<mu<<"\n";
    //     zetaD = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
    //     // gamma = 2 * std::pow(1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); //Real FMT star gamma which is smaller than rrt star which makes the neighborhood size less than rrt star hence so much faster performance
    //     gamma = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d);
    //     factor = params.getParam<double>("factor");
    //     std::cout<<"factor: "<<factor<<"\n";
    //     neighborhood_radius_ = factor * gamma * std::pow(std::log(statespace_->getNumStates()) / statespace_->getNumStates(), 1.0 / d);
    //     // // neighborhood_radius_ = 15.0;
    // }

//     ////////////////////////////////////////////////////////////////////////
// // 1) State dims & controllability indices
// int n = statespace_->getDimension();   // e.g. 3 (x,y,t)
// double D = 2.0;                        // single integrator on x and y

// // 2) Hybrid dim
// double tildeD = 0.5 * (n + D);         // (3 + 2) / 2 = 2.5

// // // 3) Volume of the *controllable* subspace (x,y)
// // Eigen::Vector2d range_xy = upper_bounds_.head<2>() - lower_bounds_.head<2>();
// // double mu = range_xy.prod();           // area
// Eigen::Vector3d range = upper_bounds_ - lower_bounds_;  // includes time range
// double mu = range.prod();  // e.g. (x_max–x_min)*(y_max–y_min)*(t_max–t_min)

// // 4) Unit‐ball volume in 2D
// double zeta = M_PI;                    // π·1²

// // 5) Constant C = (µ / ζ)^(1/2)
// double C = std::pow(mu / zeta, 1.0 / 2.0);

// // 6) Tuning factor (1+η)^(1/˜D)
// double eta = 0.5;                      // e.g. 10% inflation
// double factor = std::pow(1.0 + eta, 1.0 / tildeD);

// // 7) Sample count
// double N = double(statespace_->getNumStates());

// // 8) Final radius
// neighborhood_radius_ =
//     factor
//   * C
//   * std::pow(std::log(N) / N, 1.0 / tildeD);


    /////////////////////////////////////////////////////////////////////////
    // std::cout << "Computed value of rn: " << neighborhood_radius_ << std::endl;

    // neighbor_precache = params.getParam<bool>("precache_neighbors");
    // // In complex state spaces with complex steer function its better to cache before leaving the robot in the wild!
    // if (params.getParam<bool>("precache_neighbors")){
    //     std::cout << "Forcing neighbor caching for all " << tree_.size() << " nodes..." << std::endl;
    //     auto cache_start = std::chrono::high_resolution_clock::now();

    //     for (size_t i = 0; i < tree_.size(); ++i) {
    //         near(i);
    //     }

    //     auto cache_end = std::chrono::high_resolution_clock::now();
    //     auto cache_duration = std::chrono::duration_cast<std::chrono::milliseconds>(cache_end - cache_start);
    //     std::cout << "Neighbor caching complete. Time taken: " << cache_duration.count() << " ms." << std::endl;
    // }


    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "---\n";




}

void KinodynamicANYFMTX::updateNeighborhoodRadius() {
    // int A = statespace_->getNumStates();
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
    
    // Gamma calculation (consistent with your setup)
    double gamma = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d);
    // double gamma = std::pow(1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); //Real FMT star gamma which is smaller than rrt star which makes the neighborhood size less than rrt star hence so much faster performance

    
    neighborhood_radius_ = factor * gamma * std::pow(std::log(N) / N, 1.0 / d);
    neighborhood_radius_ = std::min(delta, neighborhood_radius_);
    // neighborhood_radius_ = 15;
    // std::cout<<"Currenr rad: "<<neighborhood_radius_<<"\n";
}

// // We need to separate decision making! i.,e the above updateNeighbors will give seg fault because you update backward map but we we ignore it because there was not forward map!!! but that pointer in the backward map still exists!!! DANGEROUS!
bool KinodynamicANYFMTX::updateNeighbors(const Eigen::VectorXd& sample_val, FMTNode* new_node) {
    std::vector<size_t> candidate_indices = kdtree_->radiusSearch(sample_val.head(kd_dim), neighborhood_radius_ + 0.01);
    
    bool is_valid_sample = false;

    // --- PASS 1: OUTGOING VALIDATION & DIRECT COMMIT ---
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

    // --- PASS 2: INCOMING VALIDATION (Kinodynamic Only) ---
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
    
    // new_node->neighbors_cached_ = true;
    return true;
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

bool KinodynamicANYFMTX::runForensics() {
    std::cout << "\n[FMTx FORENSICS] --- STARTING GOAL-ROOTED TREE VERIFICATION ---" << std::endl;
    int illegal_connections = 0;
    int checked_nodes = 0;

    // Iterate through all nodes
    for (size_t i = 0; i < tree_.size(); ++i) { 
        FMTNode* node = tree_[i].get();

        // Check nodes that are actively part of the tree (have a parent and finite cost)
        if (node->getParent() != nullptr && node->getCost() != std::numeric_limits<double>::infinity()) {
            checked_nodes++;
            bool edge_collides = false;
            std::string guilty_obstacle = "";
            FMTNode* parent = node->getParent();
            
            // 1. DIRECTION: Steer from Child (node) to Parent (closer to goal)
            Trajectory edge_traj = statespace_->steer(node->getStateValue(), parent->getStateValue());
            
            // 2. TIME: Use the Child's time as the start of the trajectory
            double time_ref = node->getTimeToGoal(); 

            // // 3. VERIFY: Absolute Ground Truth check
            // for (const auto& [name, ob] : previous_obstacles_) {
            //     if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(edge_traj, time_ref, ob)) {
            //         edge_collides = true;
            //         guilty_obstacle = name;
            //         break;
            //     }
            // }
            const ObstacleVector& current_obstacles = obs_checker_->getObstacles();
            // Iterate directly over the obstacle objects
            for (const auto& ob : current_obstacles) {
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(edge_traj, time_ref, ob)) {
                    edge_collides = true;
                    guilty_obstacle = ob.name; // Access name via ob.name
                    break;
                }
            }

            // 4. REPORT: Detailed error logs
            if (edge_collides) {
                illegal_connections++;
                std::cout << "\033[1;31m[VIOLATION]\033[0m Node " << node->getIndex() 
                          << " -> Parent " << parent->getIndex()
                          << " | Cost: " << node->getCost()
                          << " | \033[1;35mFATAL: Edge hits [" << guilty_obstacle << "]\033[0m\n";
                
                // Threat Set Diagnostics
                // bool in_node_threats = node->threats.count(guilty_obstacle);
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

// Lazy insertion, later in plan we do lazy propagation
void KinodynamicANYFMTX::addBatchOfSamplesLazy(int num_samples) {
    if (num_samples <= 0) return;

    std::vector<int> added_node_indices;
    // std::vector<Eigen::VectorXd> new_samples_viz;

    for (int i = 0; i < num_samples; ++i) {
        // 1. Generate Sample
        // Eigen::VectorXd sample_val = statespace_->sampleUniform(lower_bounds_, upper_bounds_)->getValue();
        Eigen::VectorXd sample_val = Eigen::VectorXd::Random(dimension_);
        
        // Scale from [-1, 1] to [lower_bounds, upper_bounds]
        sample_val = lower_bounds_.array() + 
                    (upper_bounds_ - lower_bounds_).array() * ((sample_val.array() + 1.0) / 2.0);


    //    // //////// INCASE YOU WANNA USE SATURATE TO GUIDE THE SAMPLES --> THIS IS NOT NECESSARY UNLESS YOU ONLY WANNA ADD ONE SAMPLE OR YOU WANNA COMPARE WITH RRTX!///////////
    //     std::vector<size_t> nearest_indices = kdtree_->knnSearch(sample_val.head(kd_dim), 1);
    //     FMTNode* nearest_node = tree_[nearest_indices[0]].get();
    //     Eigen::VectorXd nearest_state = nearest_node->getStateValue();
        
    //     // 4. Saturate (Steer)
    //     sample_val = saturate(sample_val, nearest_state, delta);
    //     // ///////////////////////////////////////////////////////
        // This is just for fmtx to be complete! in case we have static obstalces!
        if (!obs_checker_->isObstacleFree(sample_val)) {
            continue;
        }


        // 2. Create Node Object (Temporarily)
        // We create the node to get the pointer, but we don't push it to tree_ yet.
        auto node = std::make_shared<FMTNode>(statespace_->addState(sample_val), tree_.size());
        node->is_new = true; 
        
        // 3. CHECK FEASIBILITY & CONNECT
        // We pass the sample_val and the node pointer.
        // updateNeighbors will perform the steering and populate the neighbor maps.
        // It returns TRUE if at least one valid steer was found.
        if (!updateNeighbors(sample_val, node.get())) {
            // DISCARD: The node is kinematically unreachable.
            // It is NOT added to tree_, and the shared_ptr will go out of scope and be destroyed.
            continue; 
        }

        // 4. COMMIT TO TREE
        // If we reach here, the node is good.
        int node_index = tree_.size();
        tree_.push_back(node);
        
        if (!is_geometric_mode_ && node->getStateValue().size() > 2) {
            double absolute_t = node->getStateValue().tail<1>()[0];
            node->setTimeToGoal(absolute_t);
        } else {
            node->setTimeToGoal(0.0);
        }



#if USE_THREAT_SET_STRATEGY
        // ====================================================================
        // NEW: BROAD-PHASE THREAT DISCOVERY
        // ====================================================================
        // We use the spatial "Swept Volume" check to identify threats.
        // This ensures symmetry with addNewObstacle/removeObstacle.

        // for (const auto& [name, ob] : previous_obstacles_) {
        //     // neighborhood_radius_ acts as the current max_edge_length
        //     if (obs_checker_->isNodeInObstacleTube(node->getStateValue(), ob, delta)) {
        //         node->threats.insert(name);
        //     }
        // }
        for (const auto& [name, ob] : previous_obstacles_) {
            if (obs_checker_->isNodeInObstacleTube(node->getStateValue(), ob, delta)) {
                // Grab the exact memory address of the obstacle in the map
                node->threats.push_back(&ob); 
            }
        }
        
        // Since we've pre-populated threats, the plan() function will handle 
        // the actual collision checks lazily. We can clear the "is_new" flag.
        // node->is_new = false;  // NO! we shouldnt set this here because cullNeighbor also uses itasa filter!
        // ====================================================================
#endif


        // Add to KD-Tree --> BUT WE ONLY PUT THE ONES WE SPECIFIED IN THE PARAMETER
        kdtree_->addPoint(sample_val.head(kd_dim)); 
        
        added_node_indices.push_back(node_index);
        // new_samples_viz.push_back(sample_val);
    }

    if (added_node_indices.empty()) return;

    // 5. BUILD KD-TREE --> BUILDT TREE function is empty in case you use DynamicWeightedNanoFlann
    if (use_kdtree) {
        kdtree_->buildTree();
    }

    // 6. UPDATE NEIGHBORHOOD RADIUS
    updateNeighborhoodRadius();

    /*
        For newly sampled node can we use the union of the threats of the neighbors? it might work but i need to think if the removeObstalce later will remove 
        those obstalces from the new node or not. I can think of a configuration where those obstalces might linger in the threat set of this new node so for now I will
        check the collision with all the obstalces in the plan function for the new added node with is_new flag! then later in the nex turn around events the threat set for this
        new node would be updated naturally in the addNewObstalce function and we are good! 
        even using KNN to use the nearest node's threat seems like its not exact because the nearest node might be in the obstalces tube but new node might be safe outside the tube!
        I will think of an optimized solution later!
    
    */
    // 7. SEED V_OPEN
    for (int idx : added_node_indices) {
        FMTNode* new_node = tree_[idx].get();
        // std::unordered_set<std::string> accumulated_threats;

        for (const auto& [neighbor, edge_info] : new_node->forwardNeighbors()) {
            // // Accumulate threats from all connected neighbors
            // for (const auto& threat_name : neighbor->threats) {
            //     accumulated_threats.insert(threat_name);
            // }

            if (neighbor->in_queue_ || neighbor->getCost() == INFINITY) {
                continue;
            }
            v_open_heap_.add(neighbor, neighbor->getCost());
            last_replan_metrics_.queue_operations++;
        }
        // // Assign the union of threats to the new node
        // new_node->threats = accumulated_threats;

    }
    
    // // ======================================================
    // // NEW: VISUALIZE V_OPEN NODES
    // // ======================================================
    // if (visualization_) {
    //     // 1. Visualize the newly added samples (e.g., Green)
    //     visualization_->visualizeNodes(new_samples_viz);

    //     // 2. Extract and Visualize nodes currently in V_Open (e.g., Blue)
    //     std::vector<Eigen::VectorXd> v_open_nodes;
    //     v_open_nodes.reserve(v_open_heap_.getHeap().size());
        
    //     // Access the underlying heap data. 
    //     // Note: This depends on your BinaryHeap implementation. 
    //     // If it has a .getHeap() or .data() method returning a vector of pairs/elements:
    //     const auto& heap_data = v_open_heap_.getHeap(); 
        
    //     for (const auto& element : heap_data) {
    //         // element is likely {cost, node*} or similar
    //         FMTNode* node = element.second; 
    //         if (node) {
    //             v_open_nodes.push_back(node->getStateValue().head(2));
    //         }
    //     }
        
    //     // Visualize V_Open in Blue (0.0, 0.0, 1.0)
    //     if (!v_open_nodes.empty()) {
    //         visualization_->visualizeNodes(v_open_nodes, "map", 
    //                                 std::vector<float>{0.0f, 0.0f, 1.0f}, // Blue
    //                                 "v_open_nodes");
    //     }
    // }
    // // ======================================================


    // if (visualization_) {
    //     visualization_->visualizeNodes(new_samples_viz);
    // }
    
    // std::cout << "Added " << added_node_indices.size() << " feasible samples." << std::endl;
}


// Eager new sample insertion, later in plan we do lazy propagation
// In anytime fmtx eager approach we still dont care about all the collision checking connections from new node to neighbors or neighbors to new node
// except maybe the new sampled node to its forward neighbors! so we only collision check to find the parent here! but in anytime rrtx we have to check all of the
// edges and make them to have edge.distance of INF because that how rrtx works! but here because of lazy collision checking we dont have to do this!
// Even this collision checking was not necessary if i could've found a way to make the addBatchOfSamplesLazy to not become O(n log^2 (n))
// But here the O(n log(n)) will be preserved because we only do one heap push if the new node finds the best parent!
void KinodynamicANYFMTX::addBatchOfSamplesEager(int num_samples) {
    if (num_samples <= 0) return;

    std::vector<int> added_node_indices;
    // std::vector<Eigen::VectorXd> new_samples_viz;

    for (int i = 0; i < num_samples; ++i) {
        // 1. Generate Sample
        // Eigen::VectorXd sample_val = statespace_->sampleUniform(lower_bounds_, upper_bounds_)->getValue();
        Eigen::VectorXd sample_val = Eigen::VectorXd::Random(dimension_);
        
        // Scale from [-1, 1] to [lower_bounds, upper_bounds]
        sample_val = lower_bounds_.array() + 
                    (upper_bounds_ - lower_bounds_).array() * ((sample_val.array() + 1.0) / 2.0);


    //    // //////// INCASE YOU WANNA USE SATURATE TO GUIDE THE SAMPLES --> THIS IS NOT NECESSARY UNLESS YOU ONLY WANNA ADD ONE SAMPLE OR YOU WANNA COMPARE WITH RRTX!///////////
    //     std::vector<size_t> nearest_indices = kdtree_->knnSearch(sample_val.head(kd_dim), 1);
    //     FMTNode* nearest_node = tree_[nearest_indices[0]].get();
    //     Eigen::VectorXd nearest_state = nearest_node->getStateValue();
        
    //     // 4. Saturate (Steer)
    //     sample_val = saturate(sample_val, nearest_state, delta);
    //     // ///////////////////////////////////////////////////////
        // This is just for fmtx to be complete! in case we have static obstalces! so just 
        if (!obs_checker_->isObstacleFree(sample_val)) {
            continue;
        }


        // 2. Create Node Object (Temporarily)
        // We create the node to get the pointer, but we don't push it to tree_ yet.
        auto node = std::make_shared<FMTNode>(statespace_->addState(sample_val), tree_.size());
        node->is_new = true; 
        
    
#if USE_THREAT_SET_STRATEGY
        // ====================================================================
        // THE TRUE FIX: INITIALIZE THREATS FOR NEW NODES
        // Use previous_obstacles_ because it contains the fully populated predicted_path!
        /*
            the reason we do the following is this:

            Tree Nodes: 529
            Tree Nodes: 530
            Tree Nodes: 530
            Tree Nodes: 530
            Tree Nodes: 531
            Tree Nodes: 532
            [DEBUG] THREAT SET BYPASS DETECTED!!
            -> CHILD Node: 531
                State: [0.578503 -49.4121  -2.4241 -1.08705  24.4029]
                Cost: inf
                Threats: { EMPTY }
            -> PARENT Node: 532
                State: [ 1.62292 -23.1327  8.06325 0.589202  19.6487]
                Cost: 69.8477
                Threats: { EMPTY }
            -> OBSTACLE: [moving_box_8]
                Current Pos: [0, -40.44]
                Distance to Child: 8.99075
                Distance to Parent: 17.3832
            -> FATAL: Edge slices through [moving_box_8]

            during the above run the updateobstalcesample has updated once before the node 500 started working
            so even though we obstalce check to connect the node 531 to its best collision free parent but what if the
            next sampled node 532 would be a better parent!? then the child node (531) threat set needed to be updated
            or it will falsky gets connected to 532 without any valid collision check in parent!!
        
        */


        // // ====================================================================
        // for (const auto& [name, ob] : previous_obstacles_) {
        //     if (obs_checker_->isNodeInObstacleTube(node->getStateValue(), ob, delta)) {
        //         node->threats.insert(name);
        //     }
        // }
        for (const auto& [name, ob] : previous_obstacles_) {
            if (obs_checker_->isNodeInObstacleTube(node->getStateValue(), ob, delta)) {
                node->threats.push_back(&ob); // Pointer insertion
            }
        }

        
        // ====================================================================
#endif


        // 3. CHECK FEASIBILITY & CONNECT
        // We pass the sample_val and the node pointer.
        // updateNeighbors will perform the steering and populate the neighbor maps.
        // It returns TRUE if at least one valid steer was found.
        if (!updateNeighbors(sample_val, node.get())) {
            // DISCARD: The node is kinematically unreachable.
            // It is NOT added to tree_, and the shared_ptr will go out of scope and be destroyed.
            continue; 
        }

        // 4. COMMIT TO TREE
        // If we reach here, the node is good.
        int node_index = tree_.size();
        tree_.push_back(node);
        
        if (!is_geometric_mode_ && node->getStateValue().size() > 2) {
            double absolute_t = node->getStateValue().tail<1>()[0];
            node->setTimeToGoal(absolute_t);
        } else {
            node->setTimeToGoal(0.0);
        }
        // Add to KD-Tree --> BUT WE ONLY PUT THE ONES WE SPECIFIED IN THE PARAMETER
        kdtree_->addPoint(sample_val.head(kd_dim)); 
        
        added_node_indices.push_back(node_index);
        // new_samples_viz.push_back(sample_val);
    }

    if (added_node_indices.empty()) return;

    // 5. BUILD KD-TREE --> BUILDT TREE function is empty in case you use DynamicWeightedNanoFlann
    if (use_kdtree) {
        kdtree_->buildTree();
    }

    // 6. UPDATE NEIGHBORHOOD RADIUS
    updateNeighborhoodRadius();

    /*
        For newly sampled node can we use the union of the threats of the neighbors? it might work but i need to think if the removeObstalce later will remove 
        those obstalces from the new node or not. I can think of a configuration where those obstalces might linger in the threat set of this new node so for now I will
        check the collision with all the obstalces in the plan function for the new added node with is_new flag! then later in the nex turn around events the threat set for this
        new node would be updated naturally in the addNewObstalce function and we are good! 
        even using KNN to use the nearest node's threat seems like its not exact because the nearest node might be in the obstalces tube but new node might be safe outside the tube!
        I will think of an optimized solution later!
    
    */
    // ========================================================================
    // 7. SEED V_OPEN (EAGER INSERTION + LAZY PROPAGATION WITH THREATS) --> Mention this in the paper: Mind that there is no immediate rewiring like rrtx here! the propagation is lazy! but in rrtx there is an immediate rewiring because the neighbors need to know if they are getting better through the new sample and then go into the queue for triggering the propagation (when needed!) but here the new node goes into the queue and triggers the new propagation when needed so we dont have to check the neighbors right away! its ineherent to the main fmt expand function
    // ========================================================================
    for (int idx : added_node_indices) {
        FMTNode* new_node = tree_[idx].get();

        // [OPTIMIZATION 1]: Preallocate memory to avoid dynamic resizing
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

        // [OPTIMIZATION 2]: Build a Min-Heap in strictly O(K) time instead of O(K log K)
        // We use std::greater to make the lowest cost bubble to the top
        std::make_heap(candidate_parents.begin(), candidate_parents.end(), std::greater<std::pair<double, FMTNode*>>());

        // [OPTIMIZATION 3]: Hoist loop invariants. 
        // new_node->getTimeToGoal() is constant for all candidate parents!
        double node_time_to_goal = 0.0;
        if (!is_geometric_mode_ && new_node->getStateValue().size() > 2) {
            node_time_to_goal = new_node->getTimeToGoal();
        }

        bool connected = false;

        // Eagerly check candidates by popping from the Min-Heap
        while (!candidate_parents.empty()) {
            // Pop the lowest cost element from the heap in O(log K) time
            std::pop_heap(candidate_parents.begin(), candidate_parents.end(), std::greater<std::pair<double, FMTNode*>>());
            auto candidate = candidate_parents.back();
            candidate_parents.pop_back();

            FMTNode* potential_parent = candidate.second;
            auto& edge_info = new_node->forwardNeighbors().at(potential_parent);

            // if (!edge_info.is_trajectory_computed) {
            //     // lazy steering logic 
            // }
            auto traj_xy = edge_info.cached_trajectory;
            if (!traj_xy->is_valid) continue;
            bool collision_free = true;
#if USE_THREAT_SET_STRATEGY
            // We ONLY check the exact obstacle pointers in memory!
            for (const Obstacle* ob_ptr : new_node->threats) {
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*traj_xy, node_time_to_goal, *ob_ptr)) {
                    collision_free = false;
                    break; // Short-circuit
                }
            }
#else
            // Default Blind strategy
            // FIX: Again, brute-force must check everything in previous_obstacles_
            for (const auto& [name, ob] : previous_obstacles_) {
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*traj_xy, node_time_to_goal, ob)) {
                    collision_free = false;
                    break;
                }
            }
#endif

            if (collision_free) {
                // SUCCESS!
                new_node->is_new = false; 
                // new_node->threats.clear(); 

                new_node->setCost(candidate.first);
                new_node->setParent(potential_parent, traj_xy);
                new_node->setFinalDerivatives(traj_xy->final_velocity, traj_xy->final_acceleration);

                connected = true;
                break; // Stop checking candidates.
            }
            // If NOT collision free, do nothing! Just let the while loop 
            // pop the next candidate from the heap. The edge remains intact 
            // for future dynamic repairs.
        }

        // ======================================================
        // THE SINGLE HEAP INSERTION
        // ======================================================
        if (connected) {
            v_open_heap_.add(new_node, new_node->getCost());
            last_replan_metrics_.queue_operations++;
        } 
    }


    // // ======================================================
    // // NEW: VISUALIZE V_OPEN NODES
    // // ======================================================
    // if (visualization_) {
    //     // 1. Visualize the newly added samples (e.g., Green)
    //     visualization_->visualizeNodes(new_samples_viz);

    //     // 2. Extract and Visualize nodes currently in V_Open (e.g., Blue)
    //     std::vector<Eigen::VectorXd> v_open_nodes;
    //     v_open_nodes.reserve(v_open_heap_.getHeap().size());
        
    //     // Access the underlying heap data. 
    //     // Note: This depends on your BinaryHeap implementation. 
    //     // If it has a .getHeap() or .data() method returning a vector of pairs/elements:
    //     const auto& heap_data = v_open_heap_.getHeap(); 
        
    //     for (const auto& element : heap_data) {
    //         // element is likely {cost, node*} or similar
    //         FMTNode* node = element.second; 
    //         if (node) {
    //             v_open_nodes.push_back(node->getStateValue().head(2));
    //         }
    //     }
        
    //     // Visualize V_Open in Blue (0.0, 0.0, 1.0)
    //     if (!v_open_nodes.empty()) {
    //         visualization_->visualizeNodes(v_open_nodes, "map", 
    //                                 std::vector<float>{0.0f, 0.0f, 1.0f}, // Blue
    //                                 "v_open_nodes");
    //     }
    // }
    // // ======================================================


    // if (visualization_) {
    //     visualization_->visualizeNodes(new_samples_viz);
    // }
    
    // std::cout << "Added " << added_node_indices.size() << " feasible samples." << std::endl;
}

void KinodynamicANYFMTX::cullNeighbors(FMTNode* v) {
    if (v->is_new) return;

    // 1. PERFORMANCE TRIGGER: Skip loop if radius hasn't shrunk significantly
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

        // --- CONSOLIDATED CULL PROTECTION ---
        // An edge is only considered for culling if it is longer than the current radius
        // AND the node is not the current parent in the shortest-path tree[cite: 1066].
        if (edge_cost > (neighborhood_radius_ + 0.01) && neighbor != v->getParent()) {

            // 1. SYMMETRIC CULL (Neighbor's Side)
            // Remove 'v' from the neighbor's backward list if it wasn't an 'initial' birth-neighbor.
            // This mirrors Algorithm 3 in the paper[cite: 1068].
            auto& incoming = neighbor->backwardNeighbors();
            if (auto incoming_it = incoming.find(v); incoming_it != incoming.end()) {
                if (!incoming_it->second.is_initial) {
                    incoming.erase(incoming_it);
                }
            }

            // 2. SOURCE CULL (v's Side)
            // Remove the neighbor from the active forward set if it wasn't an 'initial' birth-neighbor.
            // Preservation of N_0 neighbors is sacred for optimality[cite: 1084].
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

// void KinodynamicANYFMTX::cullNeighbors(FMTNode* v) {
//     if (v->is_new) return;

//     // 1. PERFORMANCE TRIGGER
//     if (v->last_culled_radius_ > 0 && 
//         (v->last_culled_radius_ / neighborhood_radius_) < 1.0001) {
//         return;
//     }

//     auto& outgoing = v->forwardNeighbors();
    
//     // Create a temporary container for elements we want to KEEP
//     // This is much faster for flat_map than multiple erases.
//     FMTNode::NeighborMap keepers;
//     keepers.reserve(outgoing.size()); 

//     std::vector<FMTNode*> culled_neighbors;

//     for (auto& pair : outgoing) {
//         FMTNode* neighbor = pair.first;
//         EdgeInfo& edge = pair.second;
//         double edge_cost = edge.cached_trajectory->cost;

//         // logic: Should we cull this?
//         bool is_too_long = edge_cost > (neighborhood_radius_ + 0.01);
//         bool is_not_parent = (neighbor != v->getParent());
//         bool is_not_initial = !edge.is_initial;

//         if (is_too_long && is_not_parent && is_not_initial) {
//             // This neighbor is being culled. Record it for the symmetric side.
//             culled_neighbors.push_back(neighbor);
//         } else {
//             // Keep this edge! Move it to the keepers list.
//             // Using insert with hint (end) is O(1) because the source is already sorted.
//             keepers.insert(keepers.end(), std::move(pair));
//         }
//     }

//     // 2. SYMMETRIC CLEANUP (The Side Effect Zone)
//     // We do this BEFORE swapping the local map to be safe.
//     for (FMTNode* neighbor : culled_neighbors) {
//         auto& incoming = neighbor->backwardNeighbors();
//         // This is still an O(N) operation on the neighbor side,
//         // but it's necessary for graph consistency.
//         incoming.erase(v); 
//     }

//     // 3. LOCAL SWAP
//     // The old map is destroyed, and the 'keepers' map becomes the new forwardNeighbors.
//     outgoing = std::move(keepers);

//     v->last_culled_radius_ = neighborhood_radius_;
// }


void KinodynamicANYFMTX::plan() {
  
    // // while (!v_open_heap_.empty() &&
    // //        (partial_update ? (robot_node_== nullptr || v_open_heap_.top().first < robot_node_->getCost() ||
    // //                            robot_node_->getCost() == INFINITY || robot_node_->in_queue_ == true) : true)) {

    // std::unordered_map<FMTNode*, bool> costUpdated;
    // int revisits = 0;

    // // --- NEW: Track the specific number of updates per node ---
    // std::unordered_map<FMTNode*, int> node_update_counts;
    // int max_updates_single_node = 0;


    // // --- SCALING METRICS ---
    // long long m_pops = 0;           // Total expansions (N)
    // long long m_l1_edges = 0;       // Backward neighborhood checks (N * log N)
    // long long m_triggers = 0;       // Number of times the "Whistleblower" fired
    // long long m_l2_edges = 0;       // Neighborhood searches triggered by re-evaluations
    // long long m_rewires = 0;        // Successful cost/parent updates
    // // -----------------------


    // addBatchOfSamplesLazy(num_of_samples_); // Add a small batch (e.g., 10) instead of 1
    addBatchOfSamplesEager(num_of_samples_); // Add a small batch (e.g., 10) instead of 1
    while (true) {
        // // 1. ANYTIME LOGIC: Refill heap if empty or low
        // // if ( tree_.size() < num_of_samples_) {
        //     std::cout << "[Anytime] Open set low (" << v_open_heap_.getHeap().size() << "). Adding batch of samples..." << std::endl;
        //     addBatchOfSamples(1); 
        // // }

        // if (v_open_heap_.getHeap().size() < 50) { // Increased threshold to 50 for better stability
        //     // std::cout << "[Anytime] Open set low (" << v_open_heap_.getHeap().size() << "). Adding batch of samples..." << std::endl;
        //     addBatchOfSamples(1); // Add a small batch (e.g., 10) instead of 1
        // }

        // // 2. EXIT CONDITION: If heap is empty AND we can't add more samples, we are done.
        // if (v_open_heap_.empty()) {
        //     if (tree_.size() >= num_of_samples_) {
        //         break; // Planning complete
        //     } else {
        //         // Heap is empty but we haven't reached max samples.
        //         // This means addBatchOfSamples failed to add connectable nodes.
        //         // We continue the loop to try adding more samples.
        //         continue; 
        //     }
        // }
        if (v_open_heap_.empty()) {
            // If heap is empty, we can't plan. 
            // Since we have no sample cap, we just break to avoid freezing.
            break; 
        }

        // 3. PARTIAL UPDATE CONDITION: Check if we should stop based on robot cost
        // We can safely access .top() here because we checked !empty() above.
        if (partial_update) {
            double top_cost = v_open_heap_.top().first;
            // If robot is valid, and the best node in heap is worse than robot's current cost, stop.
            if (robot_node_ != nullptr && 
                robot_node_->getCost() != INFINITY && 
                top_cost >= robot_node_->getCost() + bridge_cost_) {
                break;
            }
        }

    


        auto top_element = v_open_heap_.top();
        double cost = top_element.first;  // Changed .min_key to .first
        FMTNode* z = top_element.second;  // Changed .index to .second
        int zIndex = z->getIndex();

        // Before we expand z, we remove any temporary neighbors 
        // that are now outside the shrinking radius.
        cullNeighbors(z);

        // // ======================================================
        // // NEW: VISUALIZE 'z' (The Expanding Node)
        // // ======================================================
        // if (visualization_) {
        //     std::vector<Eigen::VectorXd> z_node_viz;
        //     z_node_viz.push_back(z->getStateValue().head(2));
        //     // Visualize 'z' in RED
        //     visualization_->visualizeNodes(z_node_viz, "map", 
        //                             std::vector<float>{1.0f, 0.0f, 0.0f}, 
        //                             "current_z_node");
        // }
        // // ======================================================


        // // Find neighbors for z if they haven't been found yet.
        // if (!neighbor_precache)
        //     near(z->getIndex());

        // --- STAGE 1: IDENTIFY POTENTIALLY SUBOPTIMAL NEIGHBORS ---
        // Iterate through all neighbors 'x' of the expanding node 'z'.
        // for (auto& [x, edge_info_from_z] : z->backwardNeighbors()) { //backward means incoming . forward is outgoing
        auto& backward_neighbors = z->backwardNeighbors();
        for (auto it = backward_neighbors.begin(); it != backward_neighbors.end(); ) {
            auto x = it->first;
            // 3. Make a COPY of the edge info (remove the '&'). 
            // If the map entry gets erased, a reference would become a dangling pointer!
            const auto& edge_info_from_z = it->second; 
            // 4. SAFELY advance the iterator BEFORE any potential deletion occurs
            ++it;

            // m_l1_edges++; // Stage 1 edge check

            // // ======================================================
            // // NEW: VISUALIZE 'x' (The Neighbor Being Analyzed)
            // // ======================================================
            // if (visualization_) {
            //     std::vector<Eigen::VectorXd> x_node_viz;
            //     x_node_viz.push_back(x->getStateValue().head(2));
            //     // Visualize 'x' in YELLOW
            //     visualization_->visualizeNodes(x_node_viz, "map", 
            //                             std::vector<float>{1.0f, 1.0f, 0.0f}, 
            //                             "analyzing_x_node");
            // }
            // // ======================================================

            // The edge we care about is from child 'x' to parent 'z' in our backward search.
            // The authoritative trajectory is stored in the child's (x's) map for that edge.
            // if (!neighbor_precache)
            //     near(x->getIndex()); // Ensure x's neighbor map is initialized.

            // auto& edge_info_from_x = x->forwardNeighbors().at(z);
            auto& edge_info_from_x = edge_info_from_z;

            // // --- LAZY STEERING WITH SYMMETRIC CACHING ---
            // if (!edge_info_from_x.is_trajectory_computed) {
            //     // Compute the trajectory on-demand
            //     edge_info_from_x.cached_trajectory = statespace_->steer(x->getStateValue(), z->getStateValue());
            //     edge_info_from_x.is_trajectory_computed = true;

            //     // This ensures that if we later check the edge from y->x, we know it's handled.
            //     if (z->backwardNeighbors().count(x)) {
            //         z->backwardNeighbors().at(x).is_trajectory_computed = true;
            //         z->backwardNeighbors().at(x).cached_trajectory = edge_info_from_x.cached_trajectory;
            //     }
            // }

            // // LAZY COMPUTATION  ---
            // if (statespace_->prefersLazyNear() && !edge_info_from_x.is_trajectory_computed) {
            //     // If this is a lazy state space, compute the true trajectory from x to z now.
            //     edge_info_from_x.cached_trajectory = statespace_->steer(
            //         x->getStateValue(), z->getStateValue(), 
            //         z->getFinalVelocity(), z->getFinalAcceleration()
            //     );

            //     edge_info_from_x.is_trajectory_computed = true;
            //     // Update the distance with the true cost
            //     edge_info_from_x.distance = edge_info_from_x.cached_trajectory.cost;

            //     // Add symmetric caching ---
            //     // Update the corresponding forward neighbor entry in node 'x' to prevent re-computation.
            //     if (x->forwardNeighbors().count(z)) {
            //         // Assign the entire updated EdgeInfo struct to ensure all fields are synchronized.
            //         x->forwardNeighbors().at(z) = edge_info_from_x;
            //     }
            // }



            const Trajectory& traj_xz = *(edge_info_from_x.cached_trajectory);
            if (!traj_xz.is_valid) {
                continue;
            }
       
            double cost_via_z = z->getCost() + edge_info_from_x.distance;

            // This condition is the core of FMTX. It serves two purposes:
            // If x has not been connected yet (cost is INF), this is always true, triggering its initial connection.
            // If x is already connected, this condition acts as a "witness" that a better path *might* exist.
            //    It proves x's current cost is suboptimal and justifies the more expensive search that follows.
            // if (x->getCost() > cost_via_z + epsilon) {
            if (x->getCost() > cost_via_z) {
                // m_triggers++; // Suboptimal proof found

                // // // checks++;
                // if (costUpdated[x]) {
                //     // std::cout<<"Node " << x->getIndex() 
                //     //     << " is about to be updated a second time! "
                //     //     "previous cost = " << x->getCost() << "\n";
                    
                //     revisits++;

                // } 



                // Since x might be unvisited (never popped), its neighbors 
                // might contain stale temporary edges from an older, larger radius.
                // We must clean them now to ensure we pick a valid parent.
                // Safe to call now! If this deletes 'x' from z's map, 
                // our iterator 'it' has already safely moved past it. --> so for this reason we couldnt use the range based for loop! because cullNeihgbor(x) could've deleted the incoming node from x to z from the z's perpective, the one that we are already using in this region of code!
                cullNeighbors(x);

                // 2. THE BOUNCER: Did 'z' survive the cull?
                // If 'x' deleted 'z', the edge is illegal and 'cost_via_z' was a lie.
                // even if we do not use this if contion it will reach the "if (best_parent_for_x != nullptr && min_cost_for_x < x->getCost()) " line
                // and it will see that x's new parent is not better than its previous one but mind that if we dont use these two lines of defenses 
                // then x will connect to a suboptimal parent and cant find its previous parent because of in_queue if condtion in the bellman update
                if (x->forwardNeighbors().count(z) == 0) {
                    continue; // Skip Stage 2 and Stage 3 entirely!
                }


                // total_neighbor_iterations += x->forwardNeighbors().size();

                // STAGE 2: SEARCH FOR THE TRUE BEST PARENT ---
                // 'x' is suboptimal. We now search for its true best parent among ALL its neighbors
                // that are currently in the open set.
                double min_cost_for_x = std::numeric_limits<double>::infinity();
                FMTNode* best_parent_for_x = nullptr;
                // Trajectory best_traj_for_x;
                std::shared_ptr<Trajectory> best_traj_for_x;

                                
                for (auto& [y, edge_info_xy] : x->forwardNeighbors()) {
                    // m_l2_edges++; // [THE TAX]: Re-scanning neighbors
                    if (y->in_queue_) { // We only consider parents that are in V_open.
                        // // Check if the trajectory is already computed ---
                        // if (statespace_->prefersLazyNear() && !edge_info_xy.is_trajectory_computed) {
                        //     // If not, compute it ONCE and cache it symmetrically.
                        //     edge_info_xy.cached_trajectory = statespace_->steer(
                        //         x->getStateValue(), y->getStateValue(), 
                        //         y->getFinalVelocity(), y->getFinalAcceleration()
                        //     );
                        //     edge_info_xy.is_trajectory_computed = true;

                        //     // Symmetrically cache for the other direction to prevent re-computation later.
                        //     if (y->backwardNeighbors().count(x)) {
                        //         auto& symmetric_edge = y->backwardNeighbors().at(x);
                        //         symmetric_edge.cached_trajectory = edge_info_xy.cached_trajectory;
                        //         symmetric_edge.is_trajectory_computed = true;
                        //     }
                        // }
                        
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

                double node_time_to_goal = 0.0;
                if (!is_geometric_mode_) {
                    // Only access time if we are in kinodynamic mode
                    if (x->getStateValue().size() > 2) {
                        node_time_to_goal = x->getTimeToGoal();
                    }
                }



                // if (costUpdated[x]) {
                //     // std::cout<<"Node " << x->getIndex() 
                //     //     << "  updated a second time! "
                //     //     "new cost = " << min_cost_for_x << "\n";
                // }

                // --- STAGE 3: UPDATE (if a better parent was found) ---
                // if (best_parent_for_x != nullptr) {
                // MIND THAT EPSILON HERE IS NOT THEORETICALLY NEEDED 
                /*
                    $C_z < C_x - \epsilon$
                    Because the whistleblower $z$ is one of the nodes it checks, the minimum cost found is guaranteed to be less than or equal to the whistleblower's cost:
                    $C_{min} \le C_z$
                    $C_{min} \le C_z < C_x - \epsilon
                    Therefore, it is mathematically impossible for $C_{min}$ to fail the commit condition ($C_{min} < C_x - \epsilon$).

                */
                // if (best_parent_for_x != nullptr && min_cost_for_x < (x->getCost()-epsilon)) { // The second && is the second line of defense if the cullNeighbor(x) deletes z and we are here for somereason! because the if condtion right after cullNiehgbor(x) would caught the lie and we never reach here but still its good to be safe!
                if (best_parent_for_x != nullptr && min_cost_for_x < (x->getCost())) { // The second && is the second line of defense if the cullNeighbor(x) deletes z and we are here for somereason! because the if condtion right after cullNiehgbor(x) would caught the lie and we never reach here but still its good to be safe!
                    bool obstacle_free = true;

// #if USE_THREAT_SET_STRATEGY
//                     // ======================================================
//                     // HYBRID COLLISION CHECKING (Context-Aware)
//                     // ======================================================
//                     if (!x->threats.empty()){
//                         for (const std::string& obs_name : x->threats) {
//                             if (previous_obstacles_.count(obs_name)) {
//                                 const Obstacle& ob = previous_obstacles_[obs_name];
//                                 last_replan_metrics_.obstacle_checks++;
//                                 if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(best_traj_for_x, node_time_to_goal, ob)) {
//                                     obstacle_free = false;
//                                     break;
//                                 }
//                             }
//                         }
//                     } else {
//                         // Safe by default.
//                         obstacle_free = true;
//                     }
// #else
//                     // ======================================================
//                     // BLIND COLLISION CHECKING (Default approach)
//                     // ======================================================
//                     for (const auto& ob : obs_checker_->getObstacles()) {
//                         last_replan_metrics_.obstacle_checks++;
//                         if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(best_traj_for_x, node_time_to_goal, ob)) {
//                             obstacle_free = false;
//                             break;
//                         }
//                     }
// #endif
      
#if USE_THREAT_SET_STRATEGY
                    if (!x->threats.empty()){
                        // The CPU blasts through these contiguous pointers
                        for (const Obstacle* ob_ptr : x->threats) {
                            last_replan_metrics_.obstacle_checks++;
                            
                            // Dereference the pointer directly. Zero lookups.
                            if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*best_traj_for_x, node_time_to_goal, *ob_ptr)) {
                                obstacle_free = false;
                                break;
                            }
                        }
                    } else {
                        // Safe by default.
                        obstacle_free = true;
                    }
#else
                    // FIX: Use previous_obstacles_ so the brute-force mode actually sees the tubes!
                    for (const auto& [name, ob] : previous_obstacles_) {
                        last_replan_metrics_.obstacle_checks++;
                        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*best_traj_for_x, node_time_to_goal, ob)) {
                            obstacle_free = false;
                            break;
                        }
                    }
#endif

                    
 
                    if (obstacle_free) {

                        // // --- DEBUG COUNTERS ---
                        // static long long stage3_entered = 0;
                        // static long long queue_updates = 0;
                        // static long long queue_new_additions = 0;
                        // static long long queue_cascade_additions = 0;
                        // static long long queue_bypassed_epsilon = 0;
                        
                        // stage3_entered++;
                        // // ----------------------


                        // if (costUpdated[x]) {
                        //     revisits++;
                        // }


                        // costUpdated[x] = true;   // mark “done once”
                        // m_rewires++; // Successful repair
                        // // best_traj_for_x.from_node_index = x->getIndex(); 
                        // // best_traj_for_x.to_node_index = best_parent_for_x->getIndex(); 

                        // // --- NEW: Increment the specific node counter ---
                        // node_update_counts[x]++;
                        // if (node_update_counts[x] > max_updates_single_node) {
                        //     max_updates_single_node = node_update_counts[x];
                        // }
                        // // ------------------------------------------------


                        
                        // costUpdated[x] = true;   // mark “done once”

                        // CAPTURE OLD COST BEFORE OVERWRITING --> Need it for the epsilon consistency approach
                        double old_cost = x->getCost();
                        x->setCost(min_cost_for_x);
                        x->setParent(best_parent_for_x, best_traj_for_x);
                        last_replan_metrics_.nodes_updated++;
                        // We can now safely allow culling in future iterations.
                        x->is_new = false; 


                        ///////////////// IMPORTANT --> THIS SETTING IS TOO LATE --> I NEED TO DO THIS IN addBatchOfSamples!!!///////////
                        // // x->setTimeToGoal(min_time_for_x);
                        // double absolute_t = x->getStateValue().tail<1>()[0];
                        // x->setTimeToGoal(absolute_t);
                        if (!is_geometric_mode_) {
                            // Kinodynamic: Set time from state vector
                            if (x->getStateValue().size() > 2) {
                                double absolute_t = x->getStateValue().tail<1>()[0];
                                x->setTimeToGoal(absolute_t);
                            }
                        } else {
                            // Geometric: Time is irrelevant, set to 0 or leave default
                            x->setTimeToGoal(0.0);
                        }
                        //////////////////

                        x->setFinalDerivatives(best_traj_for_x->final_velocity, best_traj_for_x->final_acceleration);


                        // double h_value = use_heuristic ? heuristic(x->getIndex()) : 0.0;
                        // double priorityCost = min_cost_for_x + h_value;

                        double priorityCost = min_cost_for_x;

                        // if (x->in_queue_) {
                        //     v_open_heap_.update(x, priorityCost);
                        // } else {
                        //     v_open_heap_.add(x, priorityCost); // add() also sets in_queue_ = true
                        // }

                        // //////////////////////////
                        // THE EPSILON QUEUE BOUNCER --> we just dont put the 
                        if (x->in_queue_) {
                            // If it is already scheduled to be processed, give it the best cost
                            v_open_heap_.update(x, priorityCost);
                            last_replan_metrics_.queue_operations++;
                        } else if (old_cost == INFINITY || (old_cost - min_cost_for_x > epsilon)) {
                            // If it is NOT in the queue, ONLY wake it up if the improvement is > epsilon 
                            // (or if it is a brand new node that must expand the wavefront)
                            v_open_heap_.add(x, priorityCost); 
                            last_replan_metrics_.queue_operations++;
                        }
                        // /////////////////////////////

                        // // THE EPSILON QUEUE BOUNCER WITH LOGS
                        // if (x->in_queue_) {
                        //     v_open_heap_.update(x, priorityCost);
                        //     queue_updates++;
                        // } else if (old_cost == INFINITY) {
                        //     v_open_heap_.add(x, priorityCost); 
                        //     queue_new_additions++;
                        // } else if ((old_cost - min_cost_for_x) > epsilon) {
                        //     v_open_heap_.add(x, priorityCost); 
                        //     queue_cascade_additions++;
                        // } else {
                        //     // THE BOUNCER BLOCKED IT! 
                        //     // The node improved, but not enough to wake up its neighbors.
                        //     queue_bypassed_epsilon++;
                        // }

                        // // Print an update every 5000 times we enter Stage 3
                        // if (stage3_entered % 100 == 0) {
                        //     std::cout << "\033[1;35m[FMTX DEBUG] Stage 3: " << stage3_entered
                        //               << " | Updates: " << queue_updates
                        //               << " | New Additions: " << queue_new_additions
                        //               << " | Cascades (> eps): " << queue_cascade_additions
                        //               << " | Bypassed (< eps): " << queue_bypassed_epsilon << "\033[0m\n";
                        // }



                    }
                }
            } // End of STAGE 2/3 trigger
        } // End of STAGE 1 loop
        v_open_heap_.pop();
        last_replan_metrics_.queue_operations++;
        // m_pops++; // Expansion count
        // visualizeTree();
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

    } 

    // // std::cout<<"REVISITS: "<<revisits<<"\n";
    // std::cout << "[COMPLEXITY DATA] Tree: " << tree_.size() 
    //             << " | Pops: " << m_pops 
    //             << " | S1 Edges: " << m_l1_edges 
    //             << " | Whistleblowers: " << m_triggers 
    //             << " | S2 Search Edges: " << m_l2_edges 
    //             << " | Revisits: " << revisits 
    //             << " | Max Updates/Node: " << max_updates_single_node << "\n";

    #if DEBUG 
        runForensics();
    #endif


}

void KinodynamicANYFMTX::printCacheStatus() const {
    if (tree_.empty()) {
        std::cout << "[CACHE STATUS] Tree is empty." << std::endl;
        return;
    }

    size_t cached_count = 0;
    for (const auto& node_ptr : tree_) {
        if (node_ptr->isNeighborsCached()) {
            cached_count++;
        }
    }

    double percentage = 100.0 * static_cast<double>(cached_count) / tree_.size();
    std::cout << "[CACHE STATUS] Cached Nodes: " << cached_count << " / " << tree_.size()
              << " (" << std::fixed << std::setprecision(1) << percentage << "%)" << std::endl;
}

double KinodynamicANYFMTX::heuristic(int current_index) {
    Eigen::VectorXd current_position = tree_.at(current_index)->getStateValue();
    Eigen::VectorXd goal_position = tree_.at(robot_state_index_)->getStateValue();
    return (goal_position-current_position).norm();
}


std::vector<Eigen::VectorXd> KinodynamicANYFMTX::getPathPositions() const
{
    // Check if the planner has a valid anchor point for the robot.
    //    (setRobotState should have found one).
    if (!robot_node_ || robot_node_->getCost() == INFINITY) {
        FMTX_ERROR("FMTX_Path_Assembly: Robot has no valid anchor node. Cannot build path");
        return {}; // Return empty path
    }

    // Generate the "bridge" trajectory from the robot's continuous state
    //    to the anchor node on the fly.
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
        // Use the pre-computed trajectories cached in the graph during the `plan()` phase.
        // const auto& cached_traj = child->forwardNeighbors().at(parent).cached_trajectory;
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
    // QueueElement2 new_element ={0,0};

    // The root of the tree (our destination) is at rest.
    // Initialize its derivatives to zero vectors of the correct size.
    // Assuming 4 axes (x, y, z, yaw) for MinSnap.
    int num_axes = 4;
    node->setFinalDerivatives(Eigen::VectorXd::Zero(num_axes), Eigen::VectorXd::Zero(num_axes));

    v_open_heap_.add(node.get(),0);
    last_replan_metrics_.queue_operations++;
    // node->in_queue_ = true;

    tree_.push_back(node);
    std::cout << "KinodynamicANYFMTX: Start node created on Index: " << robot_state_index_ << "\n";
}
void KinodynamicANYFMTX::setGoal(const Eigen::VectorXd& goal) {
    robot_state_index_ = statespace_->getNumStates();
    auto node = std::make_shared<FMTNode>(statespace_->addState(goal),tree_.size());
    node->in_unvisited_ = true;
    node->setTimeToGoal(std::numeric_limits<double>::infinity());
    robot_node_ = node.get(); // Management of the node variable above will be done by the unique_ptr i'll send to tree_ below so robot_node_ is just using it!
    tree_.push_back(node);
    std::cout << "KinodynamicANYFMTX: Goal node created on Index: " << root_state_index_ << "\n";
}


// Edge + Nodes --> straight line
void KinodynamicANYFMTX::visualizeTree() {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    if (!tree_.empty()) {
        edges.reserve(tree_.size());
    }
    
    std::vector<Eigen::VectorXd> tree_nodes;
    tree_nodes.reserve(tree_.size());

    // --- Variables for statistics ---
    long long total_active_forward = 0;
    long long total_backward = 0;
    int connected_nodes_count = 0;
    
    for (const auto& node_ptr : tree_) {
        FMTNode* child_node = node_ptr.get();
        FMTNode* parent_node = child_node->getParent();

        tree_nodes.push_back(node_ptr->getStateValue().head(2)); // TODO: For min snap it needs to be 3!!! I need spatial dim variable!

        FMTNode* node = node_ptr.get();
        
        if (node->getCost() != std::numeric_limits<double>::infinity()) {
            connected_nodes_count++;
            total_active_forward += node->forwardNeighbors().size();
            total_backward += node->backwardNeighbors().size();
        }

        if (parent_node) {
            edges.emplace_back(parent_node->getStateValue().head(2), child_node->getStateValue().head(2));
        }
    }

    // double avg_fwd = (connected_nodes_count > 0) ? (double)total_active_forward / connected_nodes_count : 0.0;
    // double avg_bwd = (connected_nodes_count > 0) ? (double)total_backward / connected_nodes_count : 0.0;

    // std::cout << "--- Planner Stats ---" << std::endl;
    // std::cout << "Radius: " << neighborhood_radius_ << " (Delta: " << delta << ")" << std::endl;
    // std::cout << "Avg Forward (Active): " << avg_fwd << std::endl;
    // std::cout << "Avg Backward (Active): " << avg_bwd << std::endl;
        
    // visualization_->visualizeNodes(tree_nodes, "map", 
    //                         std::vector<float>{0.0f, 1.0f, 0.0f},  // Green color
    //                         "tree_nodes");
    
    // std::cout<<"Tree Nodes: "<<tree_nodes.size()<<"\n";
    visualization_->visualizeEdges(edges, "map");
}




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


void KinodynamicANYFMTX::visualizePath(const std::vector<Trajectory>& path_segments) {
    if (path_segments.empty()) {
        return;
    }

    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    
    // Iterate through each trajectory segment in the path
    for (const auto& segment : path_segments) {
        // The 'path_points' member of a Trajectory object contains the
        // finely-sampled points from the true polynomial curve.
        if (segment.path_points.size() < 2) {
            continue; // Skip segments with too few points
        }

        // Create edges between the finely-sampled points within this segment
        for (size_t i = 0; i < segment.path_points.size() - 1; ++i) {
            const Eigen::VectorXd& start_point = segment.path_points[i];
            const Eigen::VectorXd& end_point = segment.path_points[i+1];
            edges.emplace_back(start_point, end_point);
        }
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
    // figure out dimension of states
    if (tree_.empty()) {
        std::cerr << "Tree is empty. Nothing to dump.\n";
        return;
    }
    size_t dim = tree_[0]->getStateValue().size();
    // write CSV header
    fout << "node_id";
    for (size_t d = 0; d < dim; ++d) {
        fout << ",x" << d;
    }
    fout << ",parent_id\n";

    // for each node in tree_, write: node_id, coords..., parent_id
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

////////////////////////////EVENT BASED/////////////////////////////////
// ============================================================================
// 1. updateObstacleSamples (The Manager)
// ============================================================================
void KinodynamicANYFMTX::updateObstacleSamples(const ObstacleVector& turned_obstacles) {
    // Return true to satisfy the Planner interface
    if (turned_obstacles.empty()) return;

    // last_replan_metrics_ = ReplanMetrics();

    if (robot_continuous_state_.size() == 0) {
        FMTX_WARN("Planner_Obstacle_Update: Robot state not set.");
        return;
    }

    // Get exact planning time for tube generation
    double T_robot = 0.0;
    if (!is_geometric_mode_) {
        // Only extract T_robot if we are in kinodynamic mode
        if (robot_continuous_state_.size() > 0) {
            T_robot = robot_continuous_state_(robot_continuous_state_.size() - 1);
        }
    }


    for (const auto& incoming_ob : turned_obstacles) {
        
        // A. Retrieve stored obstacle
        Obstacle& stored_ob = previous_obstacles_[incoming_ob.name];
        
        // B. REMOVE OLD TUBE (Wake up neighbors in the freed region)
        // We do this BEFORE updating the object so we use the OLD path.
        if (!stored_ob.predicted_path.empty()) {
            removeObstacle(stored_ob); 
        }

        // C. UPDATE STATE
        stored_ob = incoming_ob; 

        // D. GENERATE NEW DENSE TUBE
        // Generate the tube [x, y, z, t] synchronized to T_robot
        stored_ob.predicted_path = obs_checker_->generatePrediction(stored_ob, T_robot);

        // E. ADD NEW TUBE (Invalidate nodes in the blocked region)
        addNewObstacle(stored_ob);
    }

    // if (robot_node_) {
    //     robot_node_ = nullptr;
    // }


    // plan();
}

void KinodynamicANYFMTX::addNewObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    // 1. Calculate Search Radius (as before)
    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    
    // // We want the search circle around Center 1 to reach the midpoint (R,R). because the dt calculation in generateprediction creates gaps in between obstalces!
    // // 4. THE FIX: Gap Coverage Inflation
    // // If samples are spaced by diameter (2*R_eff), we need sqrt(2) * R_eff to cover the corners.
    // // If you used the adaptive DT from the previous step, your samples are spaced by 2*R_eff.
    // double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
    // // Note: We add (sqrt(2)-1)*R because the base radius is already R. 
    // // Total = R + 0.414R = 1.414R.


    // // Use tube radius + small buffer for edges
    // // double search_radius = obs_r + ob.inflation + (max_length_); 
    // double search_radius = obs_r + ob.inflation + gap_coverage_inflation + delta; 

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
    // std::set<int> orphan_indices;


    // ======================================================
    // VISUALIZATION HELPERS
    // ======================================================
    std::vector<Eigen::VectorXd> viz_potential_orphans;
    std::vector<Eigen::VectorXd> viz_obstacle_path;
    // ======================================================

    // 2. Tube Search: Find all nodes inside the new obstacle tube
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


        // Collect obstacle path for visualization
        viz_obstacle_path.push_back(point_3d.head(2));

        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) {
            if (idx != 0)
                orphan_indices.insert(static_cast<int>(idx));
                // Collect node for visualization
                // viz_potential_orphans.push_back(tree_[idx]->getStateValue().head(2));
        }
    }
    // if (kd_dim == 4)
    //     search_radius += M_PI;




    // // 3. Filter Orphan Indices using isTrajectorySafeAgainstSingleObstacle
    std::vector<int> filtered_orphan_indices;
    for (int idx : orphan_indices) {
        FMTNode* node = tree_[idx].get();
#if USE_THREAT_SET_STRATEGY
        // // ======================================================
        // // NEW: ADD THREAT --> EVEN THOUGHT THIS NODE MAYNOT BE ORPHANED BUT ITS CLEARLY IN THREAT OF THIS OBSTALCES!
        // // ======================================================
        // // Mark this node as being under threat from this specific obstacle
        // node->threats.insert(ob.name);
        // // ======================================================

        // Mark this node as being under threat, ensuring no duplicates
        if (std::find(node->threats.begin(), node->threats.end(), &ob) == node->threats.end()) {
            node->threats.push_back(&ob);
        }

#endif
        // Skip root or nodes with no parent (shouldn't happen in tree except root, but good to be safe)
        if (node->getParent() == nullptr) continue; 

        last_replan_metrics_.obstacle_checks++;

        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(node->getParentTrajectory()), node->getTimeToGoal(), ob)) {
            filtered_orphan_indices.push_back(idx);
        }
    }

    orphan_indices.clear(); // Clear the original set
    for (int idx : filtered_orphan_indices) {
        orphan_indices.insert(idx);
        viz_potential_orphans.push_back(tree_[idx]->getStateValue().head(2));
    }


    // // ======================================================
    // // NEW: VISUALIZE KD-TREE SEARCH RESULTS
    // // ======================================================
    // if (visualization_) {
    //     // // 1. Visualize the Obstacle's Predicted Path (Yellow)
    //     // visualization_->visualizeNodes(viz_obstacle_path, "map", 
    //     //                         std::vector<float>{1.0f, 1.0f, 0.0f}, // Yellow
    //     //                         "debug_obs_path_" + ob.name);

    //     // 2. Visualize the Nodes found by KD-Tree (Red)
    //     // These are the nodes that are WITHIN the search radius.
    //     // If the robot crashes, we expect to see RED nodes where the crash happens.
    //     if (ob.name==std::string("moving_cylinder_5"))
    //         visualization_->visualizeNodes(viz_potential_orphans, "map", 
    //                                 std::vector<float>{1.0f, 0.0f, 0.0f}, // Red
    //                                 "debug_potential_orphans_" + ob.name);
    // }
    // // ======================================================
    // 4. Propagate Orphanhood to Descendants
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

    // 5. Invalidate Nodes & Queue Boundary Parents
    std::unordered_set<FMTNode*> boundary_nodes_to_requeue;
    // std::set<FMTNode*> boundary_nodes_to_requeue;
    int counter = 0;
    for (int node_index : orphan_indices) {
        auto node = tree_[node_index].get();


        // OBS CHECKING THE TREE EDGES IS GOOD BUT THE GRAPH GETS DICONNECTED! WHAT SHOULD WE DO?? When we obs check and we go into the if we need to consider the descendants or maybe take care of stuff from above loop! or maybe do stuff in nested loop?! but is that gonna make collision checking more than O(n)?
        // if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(node->getParentTrajectory(),node->getTimeToGoal(), ob)) { // JAN 9 NEW! --> Its still O(n)
            // Remove from Open Set (it's invalid now)
            if (node->in_queue_ && node->getIndex() != root_state_index_) {
                v_open_heap_.remove(node);
                last_replan_metrics_.queue_operations++;
            }

            // Invalidate Cost (but keep Root valid)
            if (node->getIndex() != root_state_index_) {
                node->setCost(INFINITY); 
                last_replan_metrics_.nodes_updated++;
                // NOTE: We do NOT set time_to_goal to INF, preserving heuristic.
            }
            
            // Sever Parent Connection
            // node->setParent(nullptr, Trajectory{});
            node->setParent(nullptr, std::shared_ptr<Trajectory>{});

            // 5. Find Boundary (Valid Parents)
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
            check_boundary(node->backwardNeighbors());
        // }
        // else{
            // counter++;
        // }
    }
    // std::cout<<"ADD OBS NO NEED COUNTER:" <<counter<<"\n";

    // 6. Add Boundary to Open Heap
    for (FMTNode* valid_node : boundary_nodes_to_requeue) {
        if (!valid_node->in_queue_ && valid_node->getCost() != INFINITY) {
            double h_value = use_heuristic ? heuristic(valid_node->getIndex()) : 0.0;
            v_open_heap_.add(valid_node, valid_node->getCost() + h_value);
            last_replan_metrics_.queue_operations++;
        }
    }
}



// ============================================================================
// 3. removeObstacle (Wake Up Neighbors)
// ============================================================================
void KinodynamicANYFMTX::removeObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);

    // // 4. THE FIX: Gap Coverage Inflation
    // // If samples are spaced by diameter (2*R_eff), we need sqrt(2) * R_eff to cover the corners.
    // // If you used the adaptive DT from the previous step, your samples are spaced by 2*R_eff.
    // double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
    // // Note: We add (sqrt(2)-1)*R because the base radius is already R. 
    // // Total = R + 0.414R = 1.414R.


    // // double search_radius = obs_r + ob.inflation + (max_length_ ); 
    // double search_radius = obs_r + ob.inflation + gap_coverage_inflation + delta;


    double search_radius;
    
    // --- GEOMETRIC VS KINODYNAMIC RADIUS ---
    if (is_geometric_mode_) {
        // In geometric mode, we just need to cover the obstacle size + robot size + delta
        search_radius = obs_r + ob.inflation + delta;
    } else {
        // Kinodynamic mode: Add gap coverage for the "tube" samples
        double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
        search_radius = obs_r + ob.inflation + delta + gap_coverage_inflation;
    }


    std::unordered_set<int> freed_indices;
    // std::set<int> freed_indices;

    // 1. Tube Search: Find nodes that were near the OLD path
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
            // Optional: Skip root if you never want it in the open set
            if (idx == root_state_index_) continue; 
            freed_indices.insert(static_cast<int>(idx));
        }
    }

    // if (kd_dim == 4)
    //     search_radius += M_PI;

    // 2. Queue Neighbors of Freed Nodes
    // We don't change costs here. We just put valid neighbors into the queue
    // to trigger the planner to explore this newly opened space.
    std::unordered_set<FMTNode*> neighbors_to_requeue;
    // std::set<FMTNode*> neighbors_to_requeue;
    int counter = 0;
    for (int node_index : freed_indices) {
        auto node = tree_.at(node_index).get();
#if USE_THREAT_SET_STRATEGY
        // // ======================================================
        // // NEW: REMOVE THREAT & RE-QUEUE
        // // ======================================================
        // // Remove the obstacle from the threat set
        // node->threats.erase(ob.name);
        // // ======================================================


        // ======================================================
        // O(1) SWAP-AND-POP THREAT REMOVAL
        // ======================================================
        auto it = std::find(node->threats.begin(), node->threats.end(), &ob);
        if (it != node->threats.end()) {
            *it = node->threats.back(); // Overwrite with the last element
            node->threats.pop_back();   // Delete the duplicate at the back
        }

#endif

        if (node->getCost()!= INFINITY) {
            counter ++; 
            continue; //JAN 9 NEW --> If the node already is on the graph then its already free!
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
        check_neighbors(node->backwardNeighbors()); 
    }
    // std::cout<< "REMOVE OBS NO NEED COUNTER: "<<counter<<"\n";

    // 3. Add to Open Heap
    for (FMTNode* neighbor : neighbors_to_requeue) {
        double h_value = use_heuristic ? heuristic(neighbor->getIndex()) : 0.0;
        v_open_heap_.add(neighbor, neighbor->getCost() + h_value);
        last_replan_metrics_.queue_operations++;
    }
}



void KinodynamicANYFMTX::setRobotState(const Eigen::VectorXd& robot_state) {
    robot_continuous_state_ = robot_state;
    // Extract actual planner-time from the state (last element)
    double robot_sim_time = robot_continuous_state_(robot_continuous_state_.size() - 1);

    // --- QUERY POINT CONSTRUCTION ---
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

    // --- HYSTERESIS LOGIC ---
    const double hysteresis_factor = 0.98;
    double cost_of_current_path = std::numeric_limits<double>::infinity();

    if (robot_node_ && robot_node_->getCost() != INFINITY) {
        Trajectory bridge = statespace_->steer(robot_continuous_state_, robot_node_->getStateValue());
        // Use robot_sim_time so collision check is synced with the world
        if (bridge.is_valid) {
            bool safe = true;
            const auto& obstacles = obs_checker_->getObstacles();
            for (const auto& ob : obstacles) {
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(bridge, robot_sim_time, ob)) {
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
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(bridge, robot_sim_time, ob)) {
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

    // --- ASSIGNMENT ---
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
        // std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> debug_edges;

        // If we have a valid anchor, recalculate the bridge path for visualization
        if (robot_node_) {
            // // Recalculate strictly for visualization
            // Trajectory viz_bridge = statespace_->steer(robot_continuous_state_, robot_node_->getStateValue());
            
            // // Convert points to edges (segments)
            // if (viz_bridge.path_points.size() >= 2) {
            //     for (size_t i = 0; i < viz_bridge.path_points.size() - 1; ++i) {
            //         debug_edges.emplace_back(viz_bridge.path_points[i], viz_bridge.path_points[i+1]);
            //     }
            // } else {
            //     // Fallback: simple straight line from robot to anchor node
                 
            //     debug_edges.emplace_back(robot_continuous_state_, robot_node_->getStateValue());
            // }

            // Visualize in CYAN (0, 1, 1) so it stands out from the path (Green) and Tree (Red/Green)
            // Using a unique namespace "debug_anchor_trajectory" ensures it overwrites the previous frame
            // visualization_->visualizeEdges(debug_edges, "map", "0.0,1.0,1.0", "debug_anchor_trajectory");
            
            // // OPTIONAL: Visualize the anchor node itself as a big dot
            std::vector<Eigen::VectorXd> anchor_pt = { robot_node_->getStateValue().head<2>() };
            visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");

        } else {
            // // CLEAR THE VISUALIZATION
            // // Sending an empty list to the same namespace effectively deletes the markers
            // visualization_->visualizeEdges({}, "map", "0.0,0.0,0.0", "debug_anchor_trajectory");
            // visualization_->visualizeNodes({}, "map", {0.0f, 0.0f, 0.0f}, "debug_anchor_point");
        }
    }
}

bool KinodynamicANYFMTX::isRobotSafe() {
    // If vbot_node_ is null, we have no anchor.
    // If cost is INFINITY, the anchor is invalid (trapped).
    return (robot_node_ != nullptr) && (robot_node_->getCost() != INFINITY);
}