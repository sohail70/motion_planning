// Copyright 2025 Soheil E.nia
// TODO : fix the KNN usage because with knn there is not neighborhood radisu constraints (check near function)
#define DEBUG 1

// Set to 1 to use your novel context-aware Threat Set.
// Set to 0 to use the Default/Blind exhaustive checking.
#define USE_THREAT_SET_STRATEGY 0


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

void KinodynamicFMTX::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    auto start = std::chrono::high_resolution_clock::now();
    clearPlannerState();
    visualization_ = visualization;
    num_of_samples_ = params.getParam<int>("num_of_samples");
    partial_update = params.getParam<bool>("partial_update");
    use_heuristic= params.getParam<bool>("use_heuristic");
    is_geometric_mode_ = params.getParam<bool>("is_geometric_mode", false);
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
        kdtree_ = std::make_shared<WeightedNanoFlann>(kd_dim, weights);
    } else if (use_kdtree == true && kdtree_type == "LieKDTree"){
        kdtree_ = std::make_unique<LieSplittingKDTree>(statespace_->getDimension(), statespace_);
    } else {
        throw std::runtime_error("FMTX requires a KD-Tree.");
    }
    std::cout << "num_of_samples=" << num_of_samples_
                << ", bounds=[" << lower_bounds_ << ", " << upper_bounds_ << "]\n";


    std::cout << "Taking care of the samples: \n \n";
    bool use_rrtx_saved_samples_ = true;
    if (use_rrtx_saved_samples_) {
        std::string filepath = "/home/sohail/motion_planning/build/rrtx_tree_nodes.csv";
               std::cout << "Loading nodes from file: " << filepath << "\n";
        std::ifstream fin(filepath);
        if (!fin.is_open()) {
            throw std::runtime_error("Failed to open node file: " + filepath);
        }

        std::string line;
        // Skip header line
        std::getline(fin, line); 

        std::string cell;
        while (std::getline(fin, line)) {
            std::stringstream lineStream(line);
            std::vector<double> state_values;
            
            // Skip node_id
            std::getline(lineStream, cell, ','); 

            // Read the state vector (x0, x1, x2...)
            // This assumes state is 3D for R2T space. Adjust if needed.
            for(int i = 0; i < statespace_->getDimension(); ++i) {
                std::getline(lineStream, cell, ',');
                state_values.push_back(std::stod(cell));
            }

            // Create an Eigen vector and then the FMTNode
            Eigen::Map<Eigen::VectorXd> state_vec(state_values.data(), state_values.size());
            auto node = std::make_shared<FMTNode>(statespace_->addState(state_vec), tree_.size());
            node->in_unvisited_ = true;
            tree_.push_back(node);
        }
        fin.close();
        std::cout << "Loaded " << tree_.size() << " nodes from file.\n";

        const Eigen::VectorXd& start_state_val = problem_->getStart();
        const Eigen::VectorXd& goal_state_val = problem_->getGoal();

        FMTNode* root_node_ptr = nullptr;
        FMTNode* robot_node_ptr = nullptr;
        double min_dist_to_start = std::numeric_limits<double>::infinity();
        double min_dist_to_goal = std::numeric_limits<double>::infinity();

        // Iterate through all loaded nodes to find the closest matches
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
        
        // Assign the found nodes to the member variables
        robot_node_ = robot_node_ptr;
        root_state_index_ = root_node_ptr->getIndex();
        robot_state_index_ = robot_node_ptr->getIndex();

        // Configure the root node (the destination of the backward search)
        root_node_ptr->setCost(0);
        root_node_ptr->setTimeToGoal(0);
        v_open_heap_.add(root_node_ptr, 0.0);

        // Configure the goal node (the robot's starting position)
        robot_node_ptr->setTimeToGoal(goal_state_val(goal_state_val.size() - 1));

        std::cout<<"Successfully identified start  and goal  nodes."<<"\n";
    }
    else{
        setStart(problem_->getStart());
        for (int i = 0 ; i < num_of_samples_ - 2; i++) {  // BUT THIS DOESNT CREATE A TREE NODE FOR START AND GOAL !!!
            auto node = std::make_shared<FMTNode>(statespace_->sampleUniform(lower_bounds_ , upper_bounds_),tree_.size());
            node->in_unvisited_ = true;
            if (!is_geometric_mode_) {
                // Kinodynamic: Set time from state vector
                if (node->getStateValue().size() > 2) {
                    double absolute_t = node->getStateValue().tail<1>()[0];
                    node->setTimeToGoal(absolute_t);
                }
            } else {
                // Geometric: Time is irrelevant, set to 0 or leave default
                node->setTimeToGoal(0.0);
            }
            tree_.push_back(node);
        }
        setGoal(problem_->getGoal());
    }






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



    ///////////////////Neighborhood Radius////////////////////////////////
    /*
        One thing i notices in R2T case Specifically is the neighbor serach that happens in plan function is alot! well thats what fmt* is about as opposed to rrt*
        on the other hand it gives us an advantage on number of obstacle checks. 
        In R2T since the possiblities of connections are much more than dubins or thrusters then these big number of loops could take a hit since we also do not filter the 
        conflicting nodes (findSamplesNearObstalces function) with isTrajectorySafe, which ofcourse we can as a feature but its not inherent from fmt core logics!
        so we can utilize the knn approach to put a theoretical cap on the number of neighbors! another thing to mention is that maybe when you use rrtx samples in fmtx
        the number of neighbors get a bit biased toward the root! (because thats how rrt star's saturate works) and maybe if we use the natural fmt star based connection 
        this problem can be avoided. but all in all knn is a nice tool in our arsenal which rrtx doesn't seem to have. for instance the whole cullNeighbor Idea
        works based on Neighborhood radius.
        So all in all the TOTAL_NEIGHBOR_ITERATIONS is expected to reduce when we use knn
    */
    if (use_knn) {
        int d = statespace_->getDimension();
        factor = params.getParam<double>("factor");
        // Practical k-NN parameter from the FMT* paper's experiments 
        double k0_fmt_star_practical = std::pow(2.0, d) * (M_E / d);
        k_neighbors_ = static_cast<int>(std::ceil(factor * k0_fmt_star_practical * std::log(statespace_->getNumStates())));
        // // // Standard k-NN parameter for RRT*
        // double k0_rrt_star = M_E * (1.0 + 1.0 / d);
        // k_neighbors_ = static_cast<int>(std::ceil(factor * k0_rrt_star * std::log(statespace_->getNumStates())));
        std::cout << "k-NN formula. k = " << k_neighbors_ << "\n";
    } else {
        int d = statespace_->getDimension();
        // mu = std::pow(problem_->getUpperBound()[0] - problem_->getLowerBound()[0] , 2);
        Eigen::VectorXd range = upper_bounds_ - lower_bounds_;
        double mu = range.prod(); // .prod() computes the product of all coefficients
        std::cout<<"mu "<<mu<<"\n";
        double zetaD = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
        // gamma = 2 * std::pow(1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); //Real FMT star gamma which is smaller than rrt star which makes the neighborhood size less than rrt star hence so much faster performance
        double gamma = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d);
        factor = params.getParam<double>("factor");
        std::cout<<"factor: "<<factor<<"\n";
        // neighborhood_radius_ = factor * gamma * std::pow(std::log(statespace_->getNumStates()) / statespace_->getNumStates(), 1.0 / d);
        neighborhood_radius_ = factor * gamma * std::pow(std::log(num_of_samples_) / num_of_samples_, 1.0 / d);
        // neighborhood_radius_ = 15.0;
    }

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
    std::cout << "Computed value of rn: " << neighborhood_radius_ << std::endl;

    neighbor_precache = params.getParam<bool>("precache_neighbors");
    // In complex state spaces with complex steer function its better to cache before leaving the robot in the wild!
    if (params.getParam<bool>("precache_neighbors")){
        std::cout << "Forcing neighbor caching for all " << tree_.size() << " nodes..." << std::endl;
        auto cache_start = std::chrono::high_resolution_clock::now();

        for (size_t i = 0; i < tree_.size(); ++i) {
            near(i);
        }

        auto cache_end = std::chrono::high_resolution_clock::now();
        auto cache_duration = std::chrono::duration_cast<std::chrono::milliseconds>(cache_end - cache_start);
        std::cout << "Neighbor caching complete. Time taken: " << cache_duration.count() << " ms." << std::endl;
    }



    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "---\n";




}

void KinodynamicFMTX::plan() {
  
    while (!v_open_heap_.empty() &&
           (partial_update ? (robot_node_== nullptr || v_open_heap_.top().first < robot_node_->getCost() + bridge_cost_||
                               robot_node_->getCost() == INFINITY || robot_node_->in_queue_ == true) : true)) {


        auto top_element = v_open_heap_.top();
        double cost = top_element.first;  // Changed .min_key to .first
        FMTNode* z = top_element.second;  // Changed .index to .second
        int zIndex = z->getIndex();


        // // =========================================================================================
        // // VISUALIZATION: CURRENT 'Z' NODE (BLUE)
        // // =========================================================================================
        // if (visualization_) {
        //     // Visualize the node we just popped from the Open set
        //     visualization_->visualizeNodes({z->getStateValue().head<2>()}, "map", 
        //                                  {0.0f, 0.0f, 1.0f}, // Blue
        //                                  "fmtx_current_z");
        // }
        // // =========================================================================================



        // Find neighbors for z if they haven't been found yet.
        if (!neighbor_precache)
            near(z->getIndex());
        // --- STAGE 1: IDENTIFY POTENTIALLY SUBOPTIMAL NEIGHBORS ---
        // Iterate through all neighbors 'x' of the expanding node 'z'.
        for (auto& [x, edge_info_from_z] : z->backwardNeighbors()) { //backward means incoming . forward is outgoing

                    // if (visualization_) {
                    //     // Visualize the neighbor 'x' being considered for update
                    //     visualization_->visualizeNodes({x->getStateValue().head<2>()}, "map", 
                    //                                  {1.0f, 1.0f, 0.0f}, // Yellow
                    //                                  "fmtx_current_x");
                    // }



            // The edge we care about is from child 'x' to parent 'z' in our backward search.
            // The authoritative trajectory is stored in the child's (x's) map for that edge.
            if (!neighbor_precache)
                near(x->getIndex()); // Ensure x's neighbor map is initialized.

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
            if (x->getCost() > cost_via_z) {
                // // checks++;
                // if (costUpdated[x]) {
                //     // std::cout<<"Node " << x->getIndex() 
                //     //     << " is about to be updated a second time! "
                //     //     "previous cost = " << x->getCost() << "\n";
                    
                //     revisits++;

                // } 

                last_replan_metrics_.rewire_neighbor_searches += x->forwardNeighbors().size();

                // total_neighbor_iterations += x->forwardNeighbors().size();

                // STAGE 2: SEARCH FOR THE TRUE BEST PARENT ---
                // 'x' is suboptimal. We now search for its true best parent among ALL its neighbors
                // that are currently in the open set.
                double min_cost_for_x = std::numeric_limits<double>::infinity();
                FMTNode* best_parent_for_x = nullptr;
                // Trajectory best_traj_for_x;
                std::shared_ptr<Trajectory> best_traj_for_x;

                                
                for (auto& [y, edge_info_xy] : x->forwardNeighbors()) {
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
                // In geometric mode, node_time_to_goal remains 0.0, which is fine 
                // because isTrajectorySafeAgainstSingleObstacle ignores it.




                // if (costUpdated[x]) {
                //     // std::cout<<"Node " << x->getIndex() 
                //     //     << "  updated a second time! "
                //     //     "new cost = " << min_cost_for_x << "\n";
                // }

                // --- STAGE 3: UPDATE (if a better parent was found) ---
                if (best_parent_for_x != nullptr) {
                    bool obstacle_free = true;

#if USE_THREAT_SET_STRATEGY
                    // ======================================================
                    // HYBRID COLLISION CHECKING (Context-Aware)
                    // ======================================================
                    if (!x->threats.empty()) {
                        // The CPU cache blasts through these contiguous pointers
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
                    // ======================================================
                    // BLIND COLLISION CHECKING (Default Exhaustive)
                    // ======================================================
                    for (const auto& [obs_name, ob] : previous_obstacles_) {
                        last_replan_metrics_.obstacle_checks++;
                        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*best_traj_for_x, node_time_to_goal, ob)) {
                            obstacle_free = false;
                            break;
                        }
                    }
#endif


                    
 
                    if (obstacle_free) {
                        // costUpdated[x] = true;   // mark “done once”
                        x->setCost(min_cost_for_x);
                        x->setParent(best_parent_for_x, best_traj_for_x);

                        ////////////////////////////
                        // // x->setTimeToGoal(min_time_for_x);
                        // double absolute_t = x->getStateValue().tail<1>()[0];
                        // x->setTimeToGoal(absolute_t);


                        ////////////////////////////



                        x->setFinalDerivatives(best_traj_for_x->final_velocity, best_traj_for_x->final_acceleration);


                        // double h_value = use_heuristic ? heuristic(x->getIndex()) : 0.0;
                        // double priorityCost = min_cost_for_x + h_value;

                        double priorityCost = min_cost_for_x;

                        if (x->in_queue_) {
                            v_open_heap_.update(x, priorityCost);
                        } else {
                            v_open_heap_.add(x, priorityCost); // add() also sets in_queue_ = true
                        }
                    }
                }
            } // End of STAGE 2/3 trigger
        } // End of STAGE 1 loop
        v_open_heap_.pop();
        // visualizeTree();
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

    } 

    // // =========================================================================
    // // FMTX DEBUG: PRINT FULL GRAPH CONNECTIVITY
    // // =========================================================================
    // std::cout << "\n================ FMTX GRAPH CONNECTIVITY ================\n";
    // for (const auto& node_ptr : tree_) {
    //     if (!node_ptr) continue;
    //     FMTNode* node = node_ptr.get();
    //     FMTNode* parent = node->getParent();
    //     int parent_idx = parent ? parent->getIndex() : -1;

    //     std::cout << "Node [" << node->getIndex() << "] | Cost: " << node->getCost() 
    //               << " | Parent: [" << parent_idx << "]\n";
    //     std::cout << "  Outgoing Neighbors:\n";
        
    //     if (node->forwardNeighbors().empty()) {
    //         std::cout << "    (none)\n";
    //     } else {
    //         for (const auto& [neighbor, edge] : node->forwardNeighbors()) {
    //             std::cout << "    -> Node [" << neighbor->getIndex() 
    //                       << "] | Edge Cost: " << edge.distance 
    //                       << " | Geom Dist: " << edge.cached_trajectory.geometric_distance << "\n";
    //         }
    //     }
    // }
    // std::cout << "=========================================================\n\n";


    #if DEBUG 
        runForensics();
    #endif

}









void KinodynamicFMTX::near(int node_index) {
    auto node = tree_[node_index].get();
    if (node->neighbors_cached_) return;

    // Get candidate neighbors (common to both strategies)
    std::vector<size_t> candidate_indices;
    if (use_knn) {
        if (k_neighbors_ > 0) {
            candidate_indices = kdtree_->knnSearch(node->getStateValue().head(kd_dim), k_neighbors_);
        }
    } else {
        if (neighborhood_radius_ > 0) {
            candidate_indices = kdtree_->radiusSearch(node->getStateValue().head(kd_dim), neighborhood_radius_ + 0.01);
        }
    }

    for (int idx : candidate_indices) {
        if (idx == node_index) continue;
        FMTNode* neighbor = tree_[idx].get();

        // Test FORWARD connection
        Trajectory traj_forward = statespace_->steer(node->getStateValue(), neighbor->getStateValue());
        
        if (traj_forward.is_valid && (use_knn || traj_forward.cost < neighborhood_radius_ + 0.01)) {
            
            // Allocate trajectory to heap ONCE
            auto shared_traj_forward = std::make_shared<Trajectory>(std::move(traj_forward));
            
            EdgeInfo edge_fwd;
            edge_fwd.distance = shared_traj_forward->cost;
            edge_fwd.distance_original = shared_traj_forward->cost;
            edge_fwd.is_initial = true;
            edge_fwd.cached_trajectory = shared_traj_forward;
            edge_fwd.is_trajectory_computed = true;

            // Add Forward Edge (Node -> Neighbor)
            node->forwardNeighbors()[neighbor] = edge_fwd;
            neighbor->backwardNeighbors()[node] = edge_fwd;

            // --- OPTIMIZATION FOR GEOMETRIC CASE ---
            if (is_geometric_mode_) {
                // In geometric mode, backward is identical to forward.
                // We reuse the exact same EdgeInfo (and the same shared_ptr)
                node->backwardNeighbors()[neighbor] = edge_fwd;
                neighbor->forwardNeighbors()[node] = edge_fwd;
            } else {
                // --- KINODYNAMIC CASE ---
                // Test BACKWARD connection (Neighbor -> Node)
                Trajectory traj_backward = statespace_->steer(neighbor->getStateValue(), node->getStateValue());
                if (traj_backward.is_valid && (use_knn || traj_backward.cost < neighborhood_radius_ + 0.01)) {
                    
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
    }
    
    node->neighbors_cached_ = true;
}


void KinodynamicFMTX::printCacheStatus() const {
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



// //////////////////////////////////////////////////////////////////////////////////////////////////////

double KinodynamicFMTX::heuristic(int current_index) {
    Eigen::VectorXd current_position = tree_.at(current_index)->getStateValue();
    Eigen::VectorXd goal_position = tree_.at(robot_state_index_)->getStateValue();
    return (goal_position-current_position).norm();
}



std::vector<size_t> KinodynamicFMTX::getPathIndex() const {
    int idx = robot_state_index_;
    std::vector<size_t> path_index;
    while (idx != -1) {
        path_index.push_back(idx);
        idx = tree_.at(idx)->getParent()->getIndex();
    }
    return path_index;
}

std::vector<Eigen::VectorXd> KinodynamicFMTX::getPathPositions() const
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


void KinodynamicFMTX::setStart(const Eigen::VectorXd& start) {
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
    // node->in_queue_ = true;

    tree_.push_back(node);
    std::cout << "KinodynamicFMTX: Start node created on Index: " << robot_state_index_ << "\n";
}
void KinodynamicFMTX::setGoal(const Eigen::VectorXd& goal) {
    robot_state_index_ = statespace_->getNumStates();
    auto node = std::make_shared<FMTNode>(statespace_->addState(goal),tree_.size());
    node->in_unvisited_ = true;
    node->setTimeToGoal(std::numeric_limits<double>::infinity());
    robot_node_ = node.get(); // Management of the node variable above will be done by the unique_ptr i'll send to tree_ below so robot_node_ is just using it!
    tree_.push_back(node);
    std::cout << "KinodynamicFMTX: Goal node created on Index: " << root_state_index_ << "\n";
}




// Edge + Nodes --> straight line
void KinodynamicFMTX::visualizeTree() {
    // std::cout<<"--------------------------- \n";
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
        FMTNode* child_node = node_ptr.get();
        FMTNode* parent_node = child_node->getParent();

        tree_nodes.push_back(node_ptr->getStateValue().head(2)); // TODO: For min snap it needs to be 3!!! I need spatial dim variable!

        if (child_node->getCost() != std::numeric_limits<double>::infinity()) {
            // std::cout<<"INDEX: "<<child_node->getIndex()<<" , COST: "<<child_node->getCost()<<"\n";
            connected_nodes_count++;
            size_t current_neighbors = child_node->forwardNeighbors().size();
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
    
    visualization_->visualizeEdges(edges, "map");
}

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


void KinodynamicFMTX::visualizePath(const std::vector<Trajectory>& path_segments) {
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

void KinodynamicFMTX::dumpTreeToCSV(const std::string& filename) const {
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




bool KinodynamicFMTX::runForensics() {
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
            double time_ref = node->getTimeToGoal(); 

            // // Verify against Ground Truth
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

            if (edge_collides) {
                illegal_connections++;
                std::cout << "\033[1;31m[VIOLATION]\033[0m Node " << node->getIndex() 
                          << " -> Parent " << parent->getIndex()
                          << " | Cost: " << node->getCost()
                          << " | \033[1;35mFATAL: Edge hits [" << guilty_obstacle << "]\033[0m\n";
                
                // --- FIXED: Threat Set Diagnostics for Pointer Vector ---
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

    return true; // Added final safety return for non-void function
}
////////////////////////////EVENT BASED/////////////////////////////////
// ============================================================================
// 1. updateObstacleSamples (The Manager)
// ============================================================================
void KinodynamicFMTX::updateObstacleSamples(const ObstacleVector& turned_obstacles) {
    // Return true to satisfy the Planner interface
    if (turned_obstacles.empty()) return;

    // last_replan_metrics_ = ReplanMetrics();


    if (robot_continuous_state_.size() == 0) {
        FMTX_WARN("Planner_Obstacle_Update: Robot state not set.");
        return;
    }

    // // Get exact planning time for tube generation
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


    plan();
}


void KinodynamicFMTX::addNewObstacle(const Obstacle& ob) {
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

    // double search_radius = obs_r + ob.inflation + neighborhood_radius_ + gap_coverage_inflation;

    double search_radius;
    
    // --- GEOMETRIC VS KINODYNAMIC RADIUS ---
    if (is_geometric_mode_) {
        // In geometric mode, we just need to cover the obstacle size + robot size + delta
        search_radius = obs_r + ob.inflation + neighborhood_radius_;
    } else {
        // Kinodynamic mode: Add gap coverage for the "tube" samples
        double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
        search_radius = obs_r + ob.inflation + neighborhood_radius_ + gap_coverage_inflation;
    }


    std::unordered_set<int> orphan_indices;
    // std::set<int> orphan_indices;

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

        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) {
            if (idx != 0)
                orphan_indices.insert(static_cast<int>(idx));

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
        // // NEW: ADD THREAT
        // // ======================================================
        // // Mark this node as being under threat from this specific obstacle
        // node->threats.insert(ob.name);

        // Mark this node as being under threat, avoiding duplicates
        if (std::find(node->threats.begin(), node->threats.end(), &ob) == node->threats.end()) {
            node->threats.push_back(&ob);
        }

#endif
        // ======================================================
        // Skip root or nodes with no parent (shouldn't happen in tree except root, but good to be safe)
        if (node->getParent() == nullptr) continue; 

        // This checks the edge connecting the node to its parent
        last_replan_metrics_.obstacle_checks++;        

        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*(node->getParentTrajectory()), node->getTimeToGoal(), ob)) {
            filtered_orphan_indices.push_back(idx);
        }
    }

    orphan_indices.clear(); // Clear the original set
    for (int idx : filtered_orphan_indices) {
        orphan_indices.insert(idx);
    }

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
            }

            // Invalidate Cost (but keep Root valid)
            if (node->getIndex() != root_state_index_) {
                node->setCost(INFINITY); 
                // NOTE: We do NOT set time_to_goal to INF, preserving heuristic.
            }
            
            // Sever Parent Connection
            // node->setParent(nullptr, Trajectory{});
            node->setParent(nullptr, std::shared_ptr<Trajectory>{});

            // 5. Find Boundary (Valid Parents)
            // We look at neighbors. If a neighbor is NOT an orphan, it's a valid candidate parent.
            if (!neighbor_precache) near(node_index);
            
            auto check_boundary = [&](const auto& neighbors) {
                for (const auto& [neighbor_ptr, edge_data] : neighbors) {
                    if (orphan_indices.find(neighbor_ptr->getIndex()) == orphan_indices.end()) {
                        boundary_nodes_to_requeue.insert(neighbor_ptr);
                    }
                }
            };

            check_boundary(node->forwardNeighbors());
            // check_boundary(node->backwardNeighbors());
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
        }
    }

    // // =========================================================================================
    // // 7. [NEW] VISUALIZATION: ORPHANS (RED) & BOUNDARY (MAGENTA)
    // // =========================================================================================
    // if (visualization_) {
    //     // A. Prepare Orphaned Nodes (Red)
    //     std::vector<Eigen::VectorXd> orphan_positions;
    //     orphan_positions.reserve(orphan_indices.size());
    //     for (int idx : orphan_indices) {
    //         // Extract 2D position (x, y) for visualization
    //         orphan_positions.push_back(tree_[idx]->getStateValue().head<2>());
    //     }

    //     // B. Prepare Boundary Nodes (Magenta)
    //     std::vector<Eigen::VectorXd> boundary_positions;
    //     boundary_positions.reserve(boundary_nodes_to_requeue.size());
    //     for (FMTNode* node : boundary_nodes_to_requeue) {
    //         boundary_positions.push_back(node->getStateValue().head<2>());
    //     }

    //     // C. Publish
    //     // Visualize Orphans in Red (1, 0, 0)
    //     visualization_->visualizeNodes(orphan_positions, "map", 
    //                                  {1.0f, 0.0f, 0.0f}, 
    //                                  "fmtx_orphans");

    //     // Visualize Boundary in Magenta (1, 0, 1) - High visibility against Green/Red
    //     visualization_->visualizeNodes(boundary_positions, "map", 
    //                                  {1.0f, 0.0f, 1.0f}, 
    //                                  "fmtx_boundary");
    // }
    // // =========================================================================================

}



// ============================================================================
// 3. removeObstacle (Wake Up Neighbors)
// ============================================================================
void KinodynamicFMTX::removeObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);

    // // We want the search circle around Center 1 to reach the midpoint (R,R). because the dt calculation in generateprediction creates gaps in between obstalces!
    // // 4. THE FIX: Gap Coverage Inflation
    // // If samples are spaced by diameter (2*R_eff), we need sqrt(2) * R_eff to cover the corners.
    // // If you used the adaptive DT from the previous step, your samples are spaced by 2*R_eff.
    // double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
    // // Note: We add (sqrt(2)-1)*R because the base radius is already R. 
    // // Total = R + 0.414R = 1.414R.

    // double search_radius = obs_r + ob.inflation + neighborhood_radius_+ gap_coverage_inflation;                   

    double search_radius;
    
    // --- GEOMETRIC VS KINODYNAMIC RADIUS ---
    if (is_geometric_mode_) {
        // In geometric mode, we just need to cover the obstacle size + robot size + delta
        search_radius = obs_r + ob.inflation + neighborhood_radius_;
    } else {
        // Kinodynamic mode: Add gap coverage for the "tube" samples
        double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
        search_radius = obs_r + ob.inflation + neighborhood_radius_ + gap_coverage_inflation;
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
        }else if (kd_dim == 5) {
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
            node->threats.pop_back();   // Delete the duplicate at the end
        }

#endif

        if (node->getCost()!= INFINITY) {
            counter ++; 
            continue; //JAN 9 NEW --> If the node already is on the graph then its already free!
        }
        if (!neighbor_precache) near(node_index);

        auto check_neighbors = [&](const auto& neighbors) {
            for (const auto& [neighbor_ptr, edge_data] : neighbors) {
                // If neighbor is valid (has cost) and not in queue, add it.
                if (neighbor_ptr->getCost() != INFINITY && !neighbor_ptr->in_queue_) {
                    neighbors_to_requeue.insert(neighbor_ptr);
                }
            }
        };

        check_neighbors(node->forwardNeighbors());
        // check_neighbors(node->backwardNeighbors());
    }
    // std::cout<< "REMOVE OBS NO NEED COUNTER: "<<counter<<"\n";

    // 3. Add to Open Heap
    for (FMTNode* neighbor : neighbors_to_requeue) {
        double h_value = use_heuristic ? heuristic(neighbor->getIndex()) : 0.0;
        v_open_heap_.add(neighbor, neighbor->getCost() + h_value);
    }
}


void KinodynamicFMTX::setRobotState(const Eigen::VectorXd& robot_state) {
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
        if (bridge.is_valid && obs_checker_->isTrajectorySafe(bridge, robot_sim_time)) {
            last_replan_metrics_.obstacle_checks += obs_checker_->getObstaclesSize();
            cost_of_current_path = bridge.cost + robot_node_->getCost();
            robot_current_time_to_goal_ = bridge.time_duration + robot_node_->getTimeToGoal();
            // return;
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

            last_replan_metrics_.obstacle_checks += obs_checker_->getObstaclesSize();
            // Check if this candidate connection is safe
            if (!obs_checker_->isTrajectorySafe(bridge, robot_sim_time)) continue;

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
        bridge_cost_= std::numeric_limits<double>::infinity();
        last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
        FMTX_WARN("[Set Robot STate] LOST SAFE ANCHOR.");
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

            // // Visualize in CYAN (0, 1, 1) so it stands out from the path (Green) and Tree (Red/Green)
            // // Using a unique namespace "debug_anchor_trajectory" ensures it overwrites the previous frame
            // visualization_->visualizeEdges(debug_edges, "map", "0.0,1.0,1.0", "debug_anchor_trajectory");
            
            // OPTIONAL: Visualize the anchor node itself as a big dot
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

bool KinodynamicFMTX::isRobotSafe() {
    // If vbot_node_ is null, we have no anchor.
    // If cost is INFINITY, the anchor is invalid (trapped).
    return (robot_node_ != nullptr) && (robot_node_->getCost() != INFINITY);
}