// Copyright 2025 Soheil E.nia
// R2T state space
#include "motion_planning/planners/kinodynamic/kinodynamic_fmtx.hpp"

KinodynamicFMTX::KinodynamicFMTX(std::shared_ptr<StateSpace> statespace ,std::shared_ptr<ProblemDefinition> problem_def, std::shared_ptr<ObstacleChecker> obs_checker) :  statespace_(statespace), problem_(problem_def), obs_checker_(obs_checker) {
    std::cout<< "KinodynamicFMTX Constructor \n";

}


void KinodynamicFMTX::setClock(rclcpp::Clock::SharedPtr clock) {
    clock_ = clock;
}

void KinodynamicFMTX::clearPlannerState() {

    v_open_heap_.clear(); //This function also makes the in_queue false so remember to clean it before deleting the node by node.reset()
    // // Step 1: Nullify all raw pointers --> or else if you only use tree_.clear() you have dangling pointers for parent_ and children_ that do not exist now!
    for (auto& node : tree_) {
        node->disconnectFromGraph();
        node.reset();  
    }
    tree_.clear();
    statespace_->reset();
    kdtree_.reset();
    // samples_in_obstacles_.clear();
    edge_length_.clear();
    max_length_edge_ind = -1;
    max_length = -std::numeric_limits<double>::infinity();
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
    partial_plot = params.getParam<bool>("partial_plot");
    static_obs_presence = params.getParam<bool>("static_obs_presence");
    ignore_sample = params.getParam<bool>("ignore_sample");
    prune = params.getParam<bool>("prune");
    obs_cache = params.getParam<bool>("obs_cache");
    lower_bounds_ = problem_->getLowerBound();
    upper_bounds_ = problem_->getUpperBound();
    use_kdtree = params.getParam<bool>("use_kdtree");
    kd_dim = params.getParam<int>("kd_dim",2);
    std::string kdtree_type = params.getParam<std::string>("kdtree_type");



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
            default: 
                RCLCPP_ERROR(rclcpp::get_logger("Planner_Obstacle_Update"), "Unsupported k-d tree dimension : %d", kd_dim);
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

        // --- START OF THE FIX ---
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
        // --- END OF THE FIX ---

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
        for (int i = 0 ; i < num_of_samples_; i++) {  // BUT THIS DOESNT CREATE A TREE NODE FOR START AND GOAL !!!
            auto node = std::make_shared<FMTNode>(statespace_->sampleUniform(lower_bounds_ , upper_bounds_),tree_.size());
            node->in_unvisited_ = true;
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

        // 1. Get the full 3D (or 4D) samples from the state space.
        Eigen::MatrixXd all_samples = statespace_->getSamplesCopy();

        // 2. Define how many spatial dimensions you have.
        //    This makes the code robust for future changes (e.g., to 3D space).
        //    Assuming (x, y, time), the spatial dimension is 2.
        int spatial_dimension = kd_dim; // For (x, y) or (x, y, time) or (x, y, theta, time)
        // For a future Dubins (x, y, theta, time) planner, this would still be 2.

        // 3. Use .leftCols() to create a new matrix with only the spatial data.
        //    .eval() is used to ensure we pass a concrete matrix, not a temporary expression.
        Eigen::MatrixXd spatial_samples_only = all_samples.leftCols(spatial_dimension).eval();
        
        // 4. Pass the 2D spatial matrix to the KD-tree.
        kdtree_->addPoints(spatial_samples_only);
        
        // 5. Build the tree all at once after we fill the data.
        kdtree_->buildTree();

    }



    ///////////////////Neighborhood Radius////////////////////////////////
    int d = statespace_->getDimension();
    // mu = std::pow(problem_->getUpperBound()[0] - problem_->getLowerBound()[0] , 2);
    Eigen::VectorXd range = upper_bounds_ - lower_bounds_;
    double mu = range.prod(); // .prod() computes the product of all coefficients
    std::cout<<"mu "<<mu<<"\n";
    zetaD = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
    // gamma = 2 * std::pow(1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); //Real FMT star gamma which is smaller than rrt star which makes the neighborhood size less than rrt star hence so much faster performance
    gamma = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d);


    
    factor = params.getParam<double>("factor");
    std::cout<<"factor: "<<factor<<"\n";
    neighborhood_radius_ = factor * gamma * std::pow(std::log(statespace_->getNumStates()) / statespace_->getNumStates(), 1.0 / d);
    // // neighborhood_radius_ = 15.0;

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


// void KinodynamicFMTX::plan() {

//     //
//     // ---> START OF NEW LOGIC <---
//     //
//     // 1. Get the current time ONCE at the beginning of the planning cycle.
//     const double t_now = clock_->now().seconds();

//     // 2. Get the current best time-to-go from the robot's anchor node.
//     //    Use a large fallback if the path is not yet found.
//     // const double best_known_time_to_goal = (robot_node_ && robot_node_->getTimeToGoal() != INFINITY)
//     //                                        ? robot_node_->getTimeToGoal()
//     //                                        : problem_->getGoal()(2); // Use initial time budget as fallback

//     const double best_known_time_to_goal = robot_current_time_to_goal_;

//     // 3. Calculate the single, predicted global time of arrival for this planning cycle.
//     //    This provides a stable anchor for all time calculations within this plan() call.
//     const double t_arrival_predicted = t_now + best_known_time_to_goal;
//     //
//     // ---> END OF NEW LOGIC <---
//     //

//     std::unordered_map<FMTNode*, bool> costUpdated;
//     int checks = 0;
//     while (!v_open_heap_.empty() &&
//            (partial_update ? (v_open_heap_.top().first < robot_node_->getCost() ||
//                                robot_node_->getCost() == INFINITY || robot_node_->in_queue_ == true) : true)) {

//         // visualizeHeapAndUnvisited();
//         // Get the node with the lowest cost from the priority queue.
//         // FMTNode* z = v_open_heap_.top(); // .pop() also sets z->in_queue_ = false;

//         auto top_element = v_open_heap_.top();
//         double cost = top_element.first;  // Changed .min_key to .first
//         FMTNode* z = top_element.second;  // Changed .index to .second
//         int zIndex = z->getIndex();

//         // Find neighbors for z if they haven't been found yet.
//         near(z->getIndex());
        
//         // --- STAGE 1: IDENTIFY POTENTIALLY SUBOPTIMAL NEIGHBORS ---
//         // Iterate through all neighbors 'x' of the expanding node 'z'.
//         for (auto& [x, edge_info_from_z] : z->neighbors()) {
//             // if (zIndex==2 && x->getIndex()==3){
//             //     std::cout<<"here \n";
//             //     std::cout<<"z: "<<z->getStateValue()<<"\n";
//             //     std::cout<<"x: "<<x->getStateValue()<<"\n";
//             // } 
//             // The edge we care about is from child 'x' to parent 'z' in our backward search.
//             // The authoritative trajectory is stored in the child's (x's) map for that edge.
//             near(x->getIndex()); // Ensure x's neighbor map is initialized.
//             auto& edge_info_from_x = x->neighbors().at(z);

//             // Compute the kinodynamic path only once and cache it.
//             if (!edge_info_from_x.is_trajectory_computed) {
//                 edge_info_from_x.cached_trajectory = statespace_->steer(x->getStateValue(), z->getStateValue());
//                 edge_info_from_x.is_trajectory_computed = true;
//             }
//             const Trajectory& traj_xz = edge_info_from_x.cached_trajectory;
//             if (!traj_xz.is_valid) {
//                 // std::cout<<"INVALID \n";
//                 continue;
//             }

//             // --- THE TRIGGER CONDITION ---
//             // Calculate the potential cost for 'x' if it were to connect through 'z'.
//             double cost_via_z = z->getCost() + traj_xz.cost;
//             // if (!traj_xz.is_valid){
//             //     cost_via_z = z->getCost() + edge_info_from_z.distance;
//             // } 
//             // This condition is the core of FMTX. It serves two purposes:
//             // 1. If x has not been connected yet (cost is INF), this is always true, triggering its initial connection.
//             // 2. If x is already connected, this condition acts as a "witness" that a better path *might* exist.
//             //    It proves x's current cost is suboptimal and justifies the more expensive search that follows.
//             if (x->getCost() > cost_via_z) {
//                 if (costUpdated[x]) {
//                     // std::cout<<"Node " << x->getIndex() 
//                     //     << " is about to be updated a second time! "
//                     //     "previous cost = " << x->getCost() << "\n";
                    
//                     checks++;

//                 } 
//                 // --- STAGE 2: SEARCH FOR THE TRUE BEST PARENT ---
//                 // 'x' is suboptimal. We now search for its true best parent among ALL its neighbors
//                 // that are currently in the open set.
//                 double min_cost_for_x = std::numeric_limits<double>::infinity();
//                 FMTNode* best_parent_for_x = nullptr;
//                 Trajectory best_traj_for_x;
//                 // std::cout<<x->getIndex()<<"\n";
//                 // if(x->getIndex()==3)
//                 //     std::cout<<"\n";
//                 // std::cout<<"----\n";
//                 //////////////////////////NOT PARALLEL////////////////////////
//                 for (auto& [y, edge_info_xy] : x->neighbors()) {
//                     // std::cout<<y->getIndex()<<"\n";
//                     // std::cout<<"----\n";
//                     if (y->in_queue_) { // We only consider parents that are in V_open.
//                         // Steer from child 'x' to potential parent 'y'. Reuse cached trajectory if possible.
//                         if (!edge_info_xy.is_trajectory_computed) {
//                             edge_info_xy.cached_trajectory = statespace_->steer(x->getStateValue(), y->getStateValue());
//                             edge_info_xy.is_trajectory_computed = true;
//                         }

//                         if (edge_info_xy.cached_trajectory.is_valid) {
//                             double cost_via_y = y->getCost() + edge_info_xy.cached_trajectory.cost;
//                             if (cost_via_y < min_cost_for_x) {
//                                 min_cost_for_x = cost_via_y;
//                                 best_parent_for_x = y;
//                                 best_traj_for_x = edge_info_xy.cached_trajectory;
//                             }
//                         }
//                     }
//                 }
//                 // ////////////////////////PARALLEL/////////////////////////////////
            


//                 ////////////////////////////////////////////////////////////////




//                 if (costUpdated[x]) {
//                     // std::cout<<"Node " << x->getIndex() 
//                     //     << "  updated a second time! "
//                     //     "new cost = " << min_cost_for_x << "\n";
//                 }

//                 // --- STAGE 3: UPDATE (if a better parent was found) ---
//                 if (best_parent_for_x != nullptr) {
                    
//                     double min_time_for_x = best_parent_for_x->getTimeToGoal() + best_traj_for_x.time_duration;



//                     ///////////////////------------------
//                     // Calculate the global time this edge is SCHEDULED to start, based on our fixed prediction.
//                     const double global_edge_start_time = t_arrival_predicted - min_time_for_x;



//                     // The global start time for the edge x->y is based on the PURE time-to-goal
//                     // double global_edge_start_time = t_arrival - min_time_for_x;

//                     // // --- DEBUG BLOCK ---
//                     // // You can uncomment this to see the data going into the check
//                     // std::cout << "\n--- Checking Trajectory ---\n"
//                     //           << "Edge: " << x->getIndex() << " -> " << best_parent_for_x->getIndex() << "\n"
//                     //           << "Current Sim Time (t_now): " << t_now << "s\n"
//                     //           << "Predicted Goal Arrival (t_arrival): " << t_arrival << "s\n"
//                     //           << "Edge Time Duration: " << best_traj_for_x.time_duration << "s\n"
//                     //           << "New Total Time-to-Goal for Node x: " << min_time_for_x << "s\n"
//                     //           << "Calculated Edge Start Time (Global): " << global_edge_start_time << "s\n"
//                     //           << "---------------------------\n";


//                     // Perform the full predictive check with the CORRECT time context.
//                     bool obstacle_free = obs_checker_->isTrajectorySafe(best_traj_for_x, global_edge_start_time);
//                     // bool obstacle_free = obs_checker_->isTrajectorySafe(best_traj_for_x, t_now);

//                     // // // /////////////////////////////////--------------

//                     // bool obstacle_free = true; // ✅ Default to true

//                     // // ✅ --- START OF THE OPTIMIZATION ---
//                     // // Only perform the expensive predictive check if the node 'x' is on the relevant
//                     // // future path of the robot. The robot_node_ stores the robot's current progress.
//                     // // ✅ --- CORRECTED OPTIMIZATION ---
//                     // // First, check if the robot has a valid, finite time-to-go.
//                     // // Then, check if the node 'x' is on the relevant future path.
//                     // if (robot_current_time_to_goal_ != std::numeric_limits<double>::infinity()) {
                        
//                     //     // Compare the node's potential time with the robot's ACTUAL current time
//                     //     if (min_time_for_x < robot_current_time_to_goal_) {
//                     //         // This is a relevant future edge, so we must check it for collisions.
//                     //         const double global_edge_start_time = t_arrival_predicted - min_time_for_x;
//                     //         obstacle_free = obs_checker_->isTrajectorySafe(best_traj_for_x, global_edge_start_time);
//                     //     }
//                     //     // ELSE: The node is "behind" the robot in time.
//                     //     // We skip the expensive check, and `obstacle_free` remains true.
                        
//                     // } else {
//                     //     // Fallback: The robot has no valid time (e.g., at the very start).
//                     //     // We must check all potential paths for safety.
//                     //     const double global_edge_start_time = t_arrival_predicted - min_time_for_x;
//                     //     obstacle_free = obs_checker_->isTrajectorySafe(best_traj_for_x, global_edge_start_time);
//                     // }

//                     // // ELSE: The node 'x' is "behind" the robot in the time-to-go timeline.
//                     // // We can skip the safety check, effectively treating the edge as safe
//                     // // for the purpose of maintaining the graph structure, even though
//                     // // the robot will never actually traverse it.
//                     // // --- END OF THE OPTIMIZATION ---




//                     // // ////////////////////------------------------


                    

//                     if (obstacle_free) {
//                         costUpdated[x] = true;   // mark “done once”
//                         // // The connection is valid and locally optimal. Update the tree and priority queue.
//                         // std::cout<<"-------\n";
//                         // std::cout<<"index: "<<x->getIndex()<<"\n";
//                         // std::cout<<"state: "<<x->getStateValue()<<"\n";
//                         // std::cout<<"cost: "<<min_cost_for_x<<"\n";
//                         // std::cout<<"-------\n";
//                         x->setCost(min_cost_for_x);
//                         x->setParent(best_parent_for_x, best_traj_for_x.cost);
//                         x->setTimeToGoal(min_time_for_x);

//                         double h_value = use_heuristic ? heuristic(x->getIndex()) : 0.0;
//                         double priorityCost = x->getCost() + h_value;

//                         if (x->in_queue_) {
//                             v_open_heap_.update(x, priorityCost);
//                         } else {
//                             v_open_heap_.add(x, priorityCost); // add() also sets in_queue_ = true
//                         }
//                     }
//                 }
//             } // End of STAGE 2/3 trigger
//             // visualizeTree();
//             // std::this_thread::sleep_for(std::chrono::milliseconds(200));
//         } // End of STAGE 1 loop
//         v_open_heap_.pop();
//         // visualizeTree();
//         // std::this_thread::sleep_for(std::chrono::milliseconds(500));

//     } // End of while loop
//     std::cout<<"REVISITS: "<<checks<<"\n";
// }








void KinodynamicFMTX::plan() {

    auto start = std::chrono::steady_clock::now();
    //
    // ---> START OF NEW LOGIC <---
    //
    // 1. Get the current time ONCE at the beginning of the planning cycle.
    const double t_now = clock_->now().seconds();

    // 2. Get the current best time-to-go from the robot's anchor node.
    //    Use a large fallback if the path is not yet found.
    // const double best_known_time_to_goal = (robot_node_ && robot_node_->getTimeToGoal() != INFINITY)
    //                                        ? robot_node_->getTimeToGoal()
    //                                        : problem_->getGoal()(2); // Use initial time budget as fallback

    const double best_known_time_to_goal = robot_current_time_to_goal_;

    // // Check if the robot has a valid time-to-go. If not, use the initial budget as a fallback.
    // const double best_known_time_to_goal = (robot_current_time_to_goal_ != std::numeric_limits<double>::infinity())
    //                                        ? robot_current_time_to_goal_
    //                                        : problem_->getGoal()(problem_->getGoal().size() - 1); // Fallback to initial time budget

    // 3. Calculate the single, predicted global time of arrival for this planning cycle.
    //    This provides a stable anchor for all time calculations within this plan() call.
    const double t_arrival_predicted = t_now + best_known_time_to_goal;
    //
    // ---> END OF NEW LOGIC <---
    //

    // std::unordered_map<FMTNode*, bool> costUpdated;
    // int checks = 0;
    // int revisits = 0;
    int obs_check = 0;
    // long long total_neighbor_iterations = 0; // The new, more accurate counter

    while (!v_open_heap_.empty() &&
           (partial_update ? (v_open_heap_.top().first < robot_node_->getCost() ||
                               robot_node_->getCost() == INFINITY || robot_node_->in_queue_ == true) : true)) {

        // visualizeHeapAndUnvisited();


        auto top_element = v_open_heap_.top();
        double cost = top_element.first;  // Changed .min_key to .first
        FMTNode* z = top_element.second;  // Changed .index to .second
        int zIndex = z->getIndex();

        // Find neighbors for z if they haven't been found yet.
        if (!neighbor_precache)
            near(z->getIndex());
        // --- STAGE 1: IDENTIFY POTENTIALLY SUBOPTIMAL NEIGHBORS ---
        // Iterate through all neighbors 'x' of the expanding node 'z'.
        for (auto& [x, edge_info_from_z] : z->backwardNeighbors()) { //backward means incoming . forward is outgoing


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




            const Trajectory& traj_xz = edge_info_from_x.cached_trajectory;
            if (!traj_xz.is_valid) {
                // std::cout<<"INVALID \n";
                continue;
            }

            // --- THE TRIGGER CONDITION ---
            // Calculate the potential cost for 'x' if it were to connect through 'z'.
            double cost_via_z = z->getCost() + traj_xz.cost;
            // if (!traj_xz.is_valid){
            //     cost_via_z = z->getCost() + edge_info_from_z.distance;
            // } 
            // This condition is the core of FMTX. It serves two purposes:
            // 1. If x has not been connected yet (cost is INF), this is always true, triggering its initial connection.
            // 2. If x is already connected, this condition acts as a "witness" that a better path *might* exist.
            //    It proves x's current cost is suboptimal and justifies the more expensive search that follows.
            if (x->getCost() > cost_via_z) {
                // checks++;
                // if (costUpdated[x]) {
                //     // std::cout<<"Node " << x->getIndex() 
                //     //     << " is about to be updated a second time! "
                //     //     "previous cost = " << x->getCost() << "\n";
                    
                //     revisits++;

                // } 

                // total_neighbor_iterations += x->forwardNeighbors().size();

                // --- STAGE 2: SEARCH FOR THE TRUE BEST PARENT ---
                // 'x' is suboptimal. We now search for its true best parent among ALL its neighbors
                // that are currently in the open set.
                double min_cost_for_x = std::numeric_limits<double>::infinity();
                FMTNode* best_parent_for_x = nullptr;
                // Trajectory best_traj_for_x;
                const Trajectory* best_traj_for_x = nullptr;

                
                for (auto& [y, edge_info_xy] : x->forwardNeighbors()) {
                    // std::cout<<y->getIndex()<<"\n";
                    // std::cout<<"----\n";
                    if (y->in_queue_) { // We only consider parents that are in V_open.
                        // // --- LAZY STEERING WITH SYMMETRIC CACHING ---
                        // if (!edge_info_xy.is_trajectory_computed) {
                        //     // Compute the trajectory on-demand
                        //     edge_info_xy.cached_trajectory = statespace_->steer(x->getStateValue(), y->getStateValue());
                        //     edge_info_xy.is_trajectory_computed = true;

                        //     // *** THE FIX: Update the neighbor's map to prevent re-computation ***
                        //     // This ensures that if we later check the edge from y->x, we know it's handled.
                        //     if (y->backwardNeighbors().count(x)) {
                        //         y->backwardNeighbors().at(x).is_trajectory_computed = true;
                        //         y->backwardNeighbors().at(x).cached_trajectory = edge_info_xy.cached_trajectory;
                        //     }
                        // }
                        if (edge_info_xy.cached_trajectory.is_valid) {
                            double cost_via_y = y->getCost() + edge_info_xy.cached_trajectory.cost;
                            if (cost_via_y < min_cost_for_x) {
                                min_cost_for_x = cost_via_y;
                                best_parent_for_x = y;
                                best_traj_for_x = &edge_info_xy.cached_trajectory;
                            }
                        }
                    }
                    
                }




                // if (costUpdated[x]) {
                //     // std::cout<<"Node " << x->getIndex() 
                //     //     << "  updated a second time! "
                //     //     "new cost = " << min_cost_for_x << "\n";
                // }

                // --- STAGE 3: UPDATE (if a better parent was found) ---
                if (best_parent_for_x != nullptr) {
                    double min_time_for_x = best_parent_for_x->getTimeToGoal() + best_traj_for_x->time_duration;



                    ///////////////////------------------
                    // Calculate the global time this edge is SCHEDULED to start, based on our fixed prediction.
                    const double global_edge_start_time = t_arrival_predicted - min_time_for_x;



                    // The global start time for the edge x->y is based on the PURE time-to-goal
                    // double global_edge_start_time = t_arrival - min_time_for_x;

                    // // --- DEBUG BLOCK ---
                    // // You can uncomment this to see the data going into the check
                    // std::cout << "\n--- Checking Trajectory ---\n"
                    //           << "Edge: " << x->getIndex() << " -> " << best_parent_for_x->getIndex() << "\n"
                    //           << "Current Sim Time (t_now): " << t_now << "s\n"
                    //           << "Predicted Goal Arrival (t_arrival): " << t_arrival << "s\n"
                    //           << "Edge Time Duration: " << best_traj_for_x.time_duration << "s\n"
                    //           << "New Total Time-to-Goal for Node x: " << min_time_for_x << "s\n"
                    //           << "Calculated Edge Start Time (Global): " << global_edge_start_time << "s\n"
                    //           << "---------------------------\n";


                    // Perform the full predictive check with the CORRECT time context.
                    bool obstacle_free = true;
                    // if(!prune)
                        obstacle_free = obs_checker_->isTrajectorySafe(*best_traj_for_x, global_edge_start_time);
                    // else if (!in_dynamic)
                    //     obstacle_free = obs_checker_->isTrajectorySafe(best_traj_for_x, global_edge_start_time);

                    // bool obstacle_free = obs_checker_->isTrajectorySafe(best_traj_for_x, t_now);
                    obs_check++;

                    // // // /////////////////////////////////--------------

                    // bool obstacle_free = true; // ✅ Default to true

                    // // ✅ --- START OF THE OPTIMIZATION ---
                    // // Only perform the expensive predictive check if the node 'x' is on the relevant
                    // // future path of the robot. The robot_node_ stores the robot's current progress.
                    // // ✅ --- CORRECTED OPTIMIZATION ---
                    // // First, check if the robot has a valid, finite time-to-go.
                    // // Then, check if the node 'x' is on the relevant future path.
                    // if (robot_current_time_to_goal_ != std::numeric_limits<double>::infinity()) {
                        
                    //     // Compare the node's potential time with the robot's ACTUAL current time
                    //     if (min_time_for_x < robot_current_time_to_goal_) {
                    //         // This is a relevant future edge, so we must check it for collisions.
                    //         const double global_edge_start_time = t_arrival_predicted - min_time_for_x;
                    //         obstacle_free = obs_checker_->isTrajectorySafe(best_traj_for_x, global_edge_start_time);
                    //     }
                    //     // ELSE: The node is "behind" the robot in time.
                    //     // We skip the expensive check, and `obstacle_free` remains true.
                        
                    // } else {
                    //     // Fallback: The robot has no valid time (e.g., at the very start).
                    //     // We must check all potential paths for safety.
                    //     const double global_edge_start_time = t_arrival_predicted - min_time_for_x;
                    //     obstacle_free = obs_checker_->isTrajectorySafe(best_traj_for_x, global_edge_start_time);
                    // }

                    // // ELSE: The node 'x' is "behind" the robot in the time-to-go timeline.
                    // // We can skip the safety check, effectively treating the edge as safe
                    // // for the purpose of maintaining the graph structure, even though
                    // // the robot will never actually traverse it.
                    // // --- END OF THE OPTIMIZATION ---




                    // // ////////////////////------------------------


                    

                    if (obstacle_free) {
                        // costUpdated[x] = true;   // mark “done once”

                        // // The connection is valid and locally optimal. Update the tree and priority queue.
                        // std::cout<<"-------\n";
                        // std::cout<<"index: "<<x->getIndex()<<"\n";
                        // std::cout<<"state: "<<x->getStateValue()<<"\n";
                        // std::cout<<"cost: "<<min_cost_for_x<<"\n";
                        // std::cout<<"-------\n";
                        x->setCost(min_cost_for_x);
                        x->setParent(best_parent_for_x, *best_traj_for_x);
                        x->setTimeToGoal(min_time_for_x);

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
            // visualizeTree();
            // std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } // End of STAGE 1 loop
        v_open_heap_.pop();
        // visualizeTree();
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

    } // End of while loop
    // std::cout<<"checks: "<<checks<<"\n";
    // std::cout<<"REVISITS: "<<revisits<<"\n";
    // std::cout << "TOTAL NEIGHBOR ITERATIONS: " << total_neighbor_iterations << "\n"; // <-- Print the new metric
    std::cout<<"OBS CHECK: "<<obs_check<<"\n";

    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    if (duration.count() > 0) {
        std::cout << "time taken for the plan func: " << duration.count() 
                << " milliseconds\n";
    }

}














/*
    Near function at first uses kd tree and after that it caches node so it doesnt need to spend time on cache
    WHEN KD TREE PROVIDES DISTANCE WHY DO YOU CALC DISTS AGAIN IN BELOW! --> ALSO DO THIS FOR RRTX  ---->
    Im not gonna use distance of kd tree for now because i don't know about other kd tree implementations
*/
// void KinodynamicFMTX::near(int node_index) {
//     auto node = tree_[node_index].get();
//     if (!node->neighbors().empty()) return;

//     auto indices = kdtree_->radiusSearch(node->getStateValue(), neighborhood_radius_); 
//     for(int idx : indices) {
//         if(idx == node->getIndex()) continue;
//         FMTNode* neighbor = tree_[idx].get();
//         auto dist = (node->getStateValue() - neighbor->getStateValue()).norm();
//         node->neighbors()[neighbor] = EdgeInfo{dist,dist}; // The first is for distance and the second is for distance_original so that we have a cache incase we want to reset the obsoleted edges to correct distance (used in prune==true case)
//     }
// }


// // Finds potential neighbors but does not calculate final cost.
// void KinodynamicFMTX::near(int node_index) {
//     auto node = tree_[node_index].get();
//     if (node->neighbors_cached_) return;

//     auto indices = kdtree_->radiusSearch(node->getStateValue().head(2), neighborhood_radius_);
//     for (int idx : indices) {
//         if (idx == node_index) continue;
//         FMTNode* neighbor = tree_[idx].get();
//         // Initialize EdgeInfo without a computed trajectory.
//         // The geometric distance is just a placeholder.
//         double geometric_dist = (node->getStateValue().head<2>() - neighbor->getStateValue().head<2>()).norm();
        
//         //IMPORTANT: mind that the geometric_dist we are saving to edge info is useless and also know that in RRTX IT SHOULD BE THE REAL COST NOT JUST THE GEOMETRIC
//         // Symmetrically add neighbor relationship
//         node->neighbors()[neighbor] = EdgeInfo{geometric_dist, geometric_dist, false, Trajectory(), false};
//         neighbor->neighbors()[node] = EdgeInfo{geometric_dist, geometric_dist, false, Trajectory(), false};

//     }
//     node->neighbors_cached_ = true;
// }


void KinodynamicFMTX::near(int node_index) {
    auto node = tree_[node_index].get();
    if (node->neighbors_cached_) return;
    auto candidate_indices = kdtree_->radiusSearch(node->getStateValue().head(kd_dim), neighborhood_radius_);
    for (int idx : candidate_indices) {
        if (idx == node_index) continue;
        FMTNode* neighbor = tree_[idx].get();

        // Test FORWARD connection: node -> neighbor
        Trajectory traj_forward = statespace_->steer(node->getStateValue(), neighbor->getStateValue());
        if (traj_forward.is_valid && traj_forward.cost < neighborhood_radius_) {
        // if (traj_forward.is_valid ) { // I think we already doing radiusSeach (even though d is an approximation on d_pi) so no need for traj cost <= rn check
        // if (traj_forward.is_valid) {
            // If we can go from node to neighbor:
            // - neighbor is in node's forward set
            // - node is in neighbor's backward set
            
            // FIX: Use brace initialization {} instead of parentheses ()
            node->forwardNeighbors()[neighbor] = {traj_forward.cost, traj_forward.cost, false, traj_forward, true};
            neighbor->backwardNeighbors()[node] = {traj_forward.cost, traj_forward.cost, false, traj_forward, true};
        }

        // Test BACKWARD connection: neighbor -> node
        Trajectory traj_backward = statespace_->steer(neighbor->getStateValue(), node->getStateValue());

        if (traj_backward.is_valid && traj_backward.cost < neighborhood_radius_) {
        // if (traj_backward.is_valid ) {
            // If we can go from neighbor to node:
            // - node is in neighbor's backward set
            // - neighbor is in node's forward set
            
            // FIX: Use brace initialization {} instead of parentheses ()
            node->backwardNeighbors()[neighbor] = {traj_backward.cost, traj_backward.cost, false, traj_backward, true};
            neighbor->forwardNeighbors()[node] = {traj_backward.cost, traj_backward.cost, false, traj_backward, true};
        }
    }
    node->neighbors_cached_ = true;
}


// // for lazy steer eval because caching does call steer twice as much! but caching gives faster time performance 
// void KinodynamicFMTX::near(int node_index) {
//     auto node = tree_[node_index].get();
//     if (node->neighbors_cached_) return;

//     auto indices = kdtree_->radiusSearch(node->getStateValue().head(kd_dim), neighborhood_radius_);
//     for (int idx : indices) {
//         if (idx == node_index) continue;
//         FMTNode* neighbor = tree_[idx].get();

//         // Create a default placeholder EdgeInfo.
//         // Using infinity is more robust than 0 for uncalculated costs.
//         EdgeInfo placeholder_edge;
//         placeholder_edge.distance = std::numeric_limits<double>::infinity();
//         placeholder_edge.distance_original = std::numeric_limits<double>::infinity();
//         placeholder_edge.is_trajectory_computed = false;

//         // Establish the FULL symmetric relationship
//         node->forwardNeighbors()[neighbor] = placeholder_edge;
//         node->backwardNeighbors()[neighbor] = placeholder_edge;
        
//         neighbor->forwardNeighbors()[node] = placeholder_edge;
//         neighbor->backwardNeighbors()[node] = placeholder_edge;
//     }
//     node->neighbors_cached_ = true;
// }



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

/*
    When an obstalce appears on some node we rely on the position of the obstalce and its radius to (D-ball containing that obstalce)
    to find the nodes in the tree that are inside of that obstalce and we use the following formula to handle worst case scenraio that i explained in the plan function i guess
    std::sqrt(std::pow(obstacle.radius + obstacle.inflation , 2) + std::pow(max_length / 2.0, 2))
    this is minimum amount needed to cover all the potential colliding edges and it lower than rrtx paper and better
*/

// std::unordered_set<int> KinodynamicFMTX::findSamplesNearObstacles(
//     const ObstacleVector& obstacles, 
//     double max_length
// ) {
//     std::unordered_set<int> conflicting_samples;
//     for (const auto& obstacle : obstacles) {
//         /*
//             when static_obs_presence is true. then use_range and all the static features also needs to be true
//             mind that obstalce check still sees all the static obstalces that are out of range because i design the gazeboObstalceChecker this way (presistent_static_obstalce==true)
//             but here we do not need to keep checking the static obstalces and put the surrounding neighbors in vopen!
//             i could've done it in the gazebo obstalce checker but since i separated the obstalce_positions from their snapshots since the obstalce need to be fixed during the process of plan function then its a bit conflicting to handle this in that class
//             these are all just handling corner cases in a simulation and separting dynamic and static obstalce and simulating a sensor range etc but not core to the algorthim it self.
//         */ 
//         if (static_obs_presence==true && !obstacle.is_dynamic) {
//             // static: only if we haven't handled it before
//             auto it = std::find(seen_statics_.begin(), seen_statics_.end(), obstacle);
//             if (it != seen_statics_.end()) {
//                 // already processed this static obstacle → skip
//                 continue;
//             }
//             // first time we see this static, remember it
//             seen_statics_.push_back(obstacle);
//         }



//         double obstacle_radius;
//         if (obstacle.type == Obstacle::CIRCLE) {
//             obstacle_radius = obstacle.dimensions.radius + obstacle.inflation;
//         } else { // BOX
//             // Calculate half diagonal of the box
//             double half_diagonal = std::sqrt(
//                 std::pow(obstacle.dimensions.width/2, 2) + 
//                 std::pow(obstacle.dimensions.height/2, 2)
//             );
//             obstacle_radius = half_diagonal + obstacle.inflation;
//         }
        
//         double search_radius = std::sqrt(
//             std::pow(obstacle_radius, 2) + 
//             std::pow(max_length / 2.0, 2)
//         );

//         // 1. Define the 3D state vector for the obstacle
//         // Eigen::VectorXd obs_state(3);

//         // // 2. Convert the time_point to a scalar double (e.g., seconds since epoch)
//         // double time_as_double = std::chrono::duration<double>(
//         //     obstacle.last_update_time.time_since_epoch()
//         // ).count();
//         // std::cout<<obstacle.last_update_time.time_since_epoch().count()<<"\n"; 
//         // // 3. Fill the vector with three explicit scalar values
//         // obs_state << obstacle.position(0), obstacle.position(1), time_as_double;
        

//         Eigen::VectorXd obs_state(2);
//         double time_as_double = obstacle.last_update_time.seconds();
//         // std::cout<<time_as_double<<"\n" ;
//         // 3. Fill the vector with three explicit scalar values
//         // obs_state << obstacle.position(0), obstacle.position(1), time_as_double;
//         obs_state << obstacle.position(0), obstacle.position(1);


//         auto sample_indices = kdtree_->radiusSearch(obs_state, search_radius);
//         conflicting_samples.insert(sample_indices.begin(), sample_indices.end());
//     }
//     return conflicting_samples;
// }
//////////////////////////////////////////////////////////

// // --------- SWEEP VERSION TO INCLUDE FUTURE POSITION OF OBSTACLES---------- //
// // TODO: Later implement a box query in weighted nano flann
// // This version is suited for 2D kd tree (x,y) --> Simple and Practical.
// std::unordered_set<int> KinodynamicFMTX::findSamplesNearObstacles(
//     const ObstacleVector& obstacles,
//     double max_length
// ) {
//     std::unordered_set<int> conflicting_samples;
//     const double PREDICTION_HORIZON_SECONDS = 3.0; // How far into the future to predict
    
//     // Controls how many steps we check between the start and end of the horizon.
//     // 0 = just start and end points. 1 = start, middle, and end points.
//     const int num_intermediate_steps = 3; 

//     for (const auto& obstacle : obstacles) {
//         // --- The existing logic for calculating search radius is good. ---
//         double obstacle_radius;
//         if (obstacle.type == Obstacle::CIRCLE) {
//             obstacle_radius = obstacle.dimensions.radius + obstacle.inflation;
//         } else { // BOX
//             double half_diagonal = std::sqrt(
//                 std::pow(obstacle.dimensions.width/2, 2) +
//                 std::pow(obstacle.dimensions.height/2, 2)
//             );
//             obstacle_radius = half_diagonal + obstacle.inflation;
//         }
//         // This search radius is a heuristic to find potentially colliding EDGES.
//         // It should be the radius of our "swept volume" check.
//         double edge_heuristic_radius = std::sqrt(
//             std::pow(obstacle_radius, 2) +
//             std::pow(max_length / 2.0, 2)
//         );
//         // double edge_heuristic_radius = obstacle_radius + max_length;

//         // --- LOGIC FOR DYNAMIC OBSTACLES ---
//         if (obstacle.is_dynamic && obstacle.velocity.norm() > 1e-6) {
            
//             // Perform searches at multiple points along the predicted path
//             for (int i = 0; i <= num_intermediate_steps + 1; ++i) {
//                 // Calculate the time for the current step
//                 double t = (static_cast<double>(i) / (num_intermediate_steps + 1)) * PREDICTION_HORIZON_SECONDS;
                
//                 // Predict the obstacle's position at time t
//                 Eigen::VectorXd predicted_pos = obstacle.position + obstacle.velocity * t;

//                 // Perform the radius search at this intermediate point
//                 auto indices = kdtree_->radiusSearch(predicted_pos, edge_heuristic_radius);
//                 conflicting_samples.insert(indices.begin(), indices.end());
//             }

//         } else {
//             // --- STATIC OBSTACLE LOGIC (Unchanged) ---
//             Eigen::VectorXd obs_state(2);
//             obs_state << obstacle.position(0), obstacle.position(1);
//             auto sample_indices = kdtree_->radiusSearch(obs_state, edge_heuristic_radius);
//             conflicting_samples.insert(sample_indices.begin(), sample_indices.end());
//         }
//     }
//     return conflicting_samples;
// }




std::unordered_set<int> KinodynamicFMTX::findSamplesNearObstacles(
    const ObstacleVector& obstacles,
    double max_length
) {
    std::unordered_set<int> conflicting_samples;

    // 1. Ensure the robot's state is valid before proceeding.
    if (robot_continuous_state_.size() == 0) {
        RCLCPP_WARN(rclcpp::get_logger("Planner_Obstacle_Update"), "Robot state not set. Skipping obstacle update.");
        return conflicting_samples;
    }

    // 2. Get the necessary information from the current state and k-d tree.
    const double robot_current_heading = (kd_dim == 4) ? robot_continuous_state_(2) : 0.0;
    const double robot_current_timestamp = (kd_dim >= 3) ? robot_continuous_state_(kd_dim - 1) : 0.0;

    for (const auto& obstacle : obstacles) {
        // --- Calculate the search radius for this obstacle ---
        double obstacle_radius;
        if (obstacle.type == Obstacle::CIRCLE) {
            obstacle_radius = obstacle.dimensions.radius + obstacle.inflation;
        } else { // BOX
            double half_diagonal = std::hypot(obstacle.dimensions.width / 2.0, obstacle.dimensions.height / 2.0);
            obstacle_radius = half_diagonal + obstacle.inflation;
        }
        double edge_heuristic_radius = std::sqrt(
            std::pow(obstacle_radius, 2) +
            std::pow(max_length / 2.0, 2)
        );

        // --- Handle DYNAMIC Obstacles ---
        if (obstacle.is_dynamic && obstacle.velocity.norm() > 1e-6) {
            /*
                // --- 1. Intelligently Calculate Horizon ---
                const double max_robot_speed = statespace_->getMaxVelocity(); // Assuming StateSpace exposes this
                const double distance_to_obstacle = (robot_continuous_state_.head<2>() - obstacle.position).norm();

                // Simplified closing speed calculation
                const double closing_speed = max_robot_speed + obstacle.velocity.norm();
                
                // Heuristic for time-to-impact
                double time_to_impact = distance_to_obstacle / std::max(1.0, closing_speed);

                // The adaptive horizon is the shortest of the time-to-impact or the robot's remaining plan time,
                // clamped within a reasonable min/max range.
                const double PREDICTION_HORIZON_SECONDS = std::min(
                    robot_current_timestamp, // Can't predict past the goal
                    std::clamp(time_to_impact, 1.5, 5.0) // Clamp to a sensible range (e.g., 1.5s to 5.0s)
                );

                // --- 2. Intelligently Calculate Step Count ---
                const double desired_time_resolution = 0.5; // Check every 0.5 seconds
                const int num_intermediate_steps = static_cast<int>(std::ceil(PREDICTION_HORIZON_SECONDS / desired_time_resolution));
            */

            const double PREDICTION_HORIZON_SECONDS = 3.0;
            const int num_intermediate_steps = 10;

            // Perform searches at multiple discrete time steps along the predicted path
            for (int i = 0; i <= num_intermediate_steps; ++i) {
                double delta_t = (static_cast<double>(i) / num_intermediate_steps) * PREDICTION_HORIZON_SECONDS;
                
                // Predict the obstacle's future 2D position
                Eigen::Vector2d predicted_pos_2d = obstacle.position + obstacle.velocity * delta_t;
                
                // Construct the full-dimensional query point for the k-d tree
                Eigen::VectorXd query_point(kd_dim);
                
                switch (kd_dim) {
                    case 2: // (x, y)
                        query_point = predicted_pos_2d;
                        break;
                    case 3: // (x, y, time)
                    case 4: // (x, y, theta, time)
                        {
                            // // Translate future time into the planner's "time-to-go" frame
                            // double query_timestamp = robot_current_timestamp - delta_t;
                            // if (query_timestamp < 0) {
                            //     // std::cout<<"skipped" << query_timestamp<<"\n";
                            //     query_timestamp = 0.0;
                            //     // continue;
                            // } // Skip if predicted time is "after" the goal

                            
                            // By not subtracting delta_t, we create a spatial query
                            // centered temporally near the robot's current plan.
                            double query_timestamp = robot_current_timestamp;
                            if (kd_dim == 3) {
                                query_point << predicted_pos_2d, query_timestamp;
                            } else { // kd_dim == 4
                                query_point << predicted_pos_2d, robot_current_heading, query_timestamp;
                            }
                        }
                        break;
                    default:
                        continue; // Skip unsupported dimensions
                }
                
                auto indices = kdtree_->radiusSearch(query_point, edge_heuristic_radius);
                conflicting_samples.insert(indices.begin(), indices.end());
            }
        } else {
            // --- Handle STATIC Obstacles ---
            // Optimization: If we have seen this static obstacle before, skip it.
            auto it = std::find(seen_statics_.begin(), seen_statics_.end(), obstacle);
            if (it != seen_statics_.end()) {
                continue;
            }
            seen_statics_.push_back(obstacle);

            Eigen::VectorXd query_point(kd_dim);
            
            switch (kd_dim) {
                case 2: // (x, y)
                    query_point = obstacle.position;
                    break;
                case 3: // (x, y, time)
                case 4: // (x, y, theta, time)
                    // For static obstacles, query at the robot's current timestamp
                    if (kd_dim == 3) {
                        query_point << obstacle.position, robot_current_timestamp;
                    } else { // kd_tree_dim == 4
                        query_point << obstacle.position, robot_current_heading, robot_current_timestamp;
                    }
                    break;
                default:
                    continue;
            }
            
            auto indices = kdtree_->radiusSearch(query_point, edge_heuristic_radius);
            conflicting_samples.insert(indices.begin(), indices.end());
        }
    }
    return conflicting_samples;
}



////////////////////////////////////////////////////////


/*
    This is using two circles with same center but different radius to find the nodes exactly on obstalce and nodes that are on surrounding using the above formula
    its still using the old api scale factor but since i don't wanna use it for now i kept the old api but you can use the above formula instead of scale factor
    the reason to use this is to make some part of the code faster because if you have distinction on what nodes are exactlyon obstalce and what are the nodes that are also on obstalce and surrouning then
    you can use that to your advantage
*/
std::pair<std::unordered_set<int>, std::unordered_set<int>> KinodynamicFMTX::findSamplesNearObstaclesDual(
    const ObstacleVector& obstacles, 
    double max_length
) {
    std::unordered_set<int> conflicting_samples_inflated;
    std::unordered_set<int> conflicting_samples;

    for (const auto& obstacle : obstacles) {
        double obstacle_radius, base_radius;
        if (obstacle.type == Obstacle::CIRCLE) {
            base_radius = obstacle.dimensions.radius;
            obstacle_radius = base_radius + obstacle.inflation;
        } else { // BOX
            // Calculate half diagonal of the box
            double half_diagonal = std::sqrt(
                std::pow(obstacle.dimensions.width/2, 2) + 
                std::pow(obstacle.dimensions.height/2, 2)
            );
            base_radius = half_diagonal;
            obstacle_radius = half_diagonal + obstacle.inflation;
        }

        double outer_radius = std::sqrt(
            std::pow(obstacle_radius, 2) + 
            std::pow(max_length / 2.0, 2)
        );
        
        double inner_radius = base_radius + obstacle.inflation;

        auto [sample_indices1, sample_indices2] = kdtree_->radiusSearchDual(
            obstacle.position, 
            outer_radius, 
            inner_radius
        );

        conflicting_samples_inflated.insert(sample_indices1.begin(), sample_indices1.end());
        conflicting_samples.insert(sample_indices2.begin(), sample_indices2.end());
    }

    return {conflicting_samples_inflated, conflicting_samples};
}







/*
    The procedure is like this depending on if you use ignore_sample to ignore all the samples around obstalce (based on the minimum amounf formula)
    if you ignore sample everything is easy and you don't need obstalce check in handle add and handle remove obstalce
    we can alse use added/removed wrt to the previous 
    imagine two circles that have an intersection the left one is previous position of the upstacle and the right is the new position
    also try to visulize them bigger because of the findSampleNewObstalce that has overestimate formula (which i emphasize agian is the minimum needed for the worst case scenraio)
    now the right crescent would be the "added" nodes and the left crescent would be the "removed" nodes and every thing in between is not necessary to be in update because they are 
    on samples_in_obstalces and we ignore them in the plan function

    the approach of ignoring samples is good but as the number of samples lowers the overestimate formula gets bigger because max_edge gets bigger so we end up ignoring lots of sample (so we are sacrificing optimality)
    even though its optimal for samples that we provide to the plan but since we ignore some samples so the plan() function doesnt see them
    but all in all we can say its much faster than to explicitly check and as the number of nodes increases that problem becomes negligible to non existent i would say although i need to provide some math maybe

    if i want to give some comparison between rrtx and fmtx in ignore sample case --> it would be a fight between incremental rewiring and batch repair! because if we use ignore_sample==true there would be absolutely no obstalce check
    and you can see as we lower the rate of the outer main while loop the batch update outperforms and as we increase the rate we see  they are both on par algorithms.
    to give more explanation lower rate (like 1hz) means the obstalce move alot and suddenly lots of nodes gets orphaned and fmtx shines

    but if we follow rrtx approach to explicitly check edges because why do we ignore useful samples! then we have to use complete current sample and complete previous samples 
    and we explicitly check obstalce for each node and its neighbor to find which of the overestimated node are indeed have an edge on obstalce and which don't

*/

bool KinodynamicFMTX::updateObstacleSamples(const ObstacleVector& obstacles) {
    in_dynamic = true;

    /*
        Now that im thinking about this the max_length's upper bound is neighborhood_radius_ in fmtx! this is not a rrt star based algorithm!
        I guess we don't need to track the max_edge! and we can easily use rn for this but for now i'll leave this as is!

        But maybe its best to calc it i don't know if the trade off in the findsamples and the cascading calculation after ward will help the overall performance 
        or not but i suspect it does. but for now i do not consider this!

        mind that in kinodynamic case this is not the case i guess unless you put some constraint that the cost of the traj is not more that neighborhood radius
    
    */
    max_length =  neighborhood_radius_; // At first Static plan we don't have max_length --> either do this or do a static plan
    // if (edge_length_[max_length_edge_ind] != max_length) // This condition also triggeres the first calculation os It's okay
    // {
    //     auto max_it = std::max_element(edge_length_.begin() , edge_length_.end() ,[](const std::pair<int, double>& a , const std::pair<int, double>& b){
    //         return a.second < b.second;
    //     });
    //     max_length = max_it->second;
    //     max_length_edge_ind = max_it->first;
    //     // std::cout<<max_it->first << "  " << max_it->second <<" \n"; 
    // }

    auto current = findSamplesNearObstacles(obstacles, max_length);
    // auto [current, direct] = findSamplesNearObstaclesDual(obstacles, max_length);


    // // --- START OF NEW FILTERING LOGIC ---
    // // This is the narrow-phase check to refine the 'current' set.
    // // We iterate through the potentially conflicting nodes and remove any whose
    // // connection to their parent is actually still safe.
    // for (auto it = current.begin(); it != current.end(); ) {
    //     int node_index = *it;
    //     auto node = tree_[node_index].get();
    //     auto parent = node->getParent();

    //     // If the node has no parent, it's an orphan or the root.
    //     // It's definitely in a state of conflict or change, so we must keep it.
    //     if (!parent) {
    //         ++it;
    //         continue;
    //     }

    //     // The trajectory from this node to its parent defines its connection to the tree.
    //     Trajectory traj_to_parent;
    //     // auto& neighbors = node->neighbors();
    //     auto& neighbors = node->forwardNeighbors(); // the trajectory from a child to its parent is stored in the child's forwardNeighbors
    //     auto neighbor_it = neighbors.find(parent);

    //     // Ensure the trajectory has been computed. If not, compute and cache it.
    //     if (neighbor_it != neighbors.end() && neighbor_it->second.is_trajectory_computed) {
    //         traj_to_parent = neighbor_it->second.cached_trajectory;
    //     } else {
    //         traj_to_parent = statespace_->steer(node->getStateValue(), parent->getStateValue());
    //         if (neighbor_it != neighbors.end()) {
    //             neighbor_it->second.cached_trajectory = traj_to_parent;
    //             neighbor_it->second.is_trajectory_computed = true;
    //         }
    //     }

    //     // If the trajectory is geometrically invalid, the edge is broken. Keep the node.
    //     if (!traj_to_parent.is_valid) {
    //         ++it;
    //         continue;
    //     }

    //     // Check if this specific trajectory is safe against dynamic obstacles.
    //     // We use a heuristic start time of 'now' since we are reacting to a current obstacle update.
    //     const double global_start_time_heuristic = clock_->now().seconds();
    //     if (obs_checker_->isTrajectorySafe(traj_to_parent, global_start_time_heuristic)) {
    //         // The connection to the parent is SAFE. This node is not in immediate conflict
    //         // via its tree connection. Therefore, we can filter it out (erase it).
    //         it = current.erase(it);
    //     } else {
    //         // The connection to the parent is NOT safe. Keep this node in the set.
    //         ++it;
    //     }
    // }
    // // --- END OF NEW FILTERING LOGIC ---
    

    //////////////////////////////////////////////////////////////////////////////////

    // // 'current' is the set of nodes from findSamplesNearObstacles
    // for (auto it = current.begin(); it != current.end(); /* no increment here */) {
    //     auto node = tree_[*it].get();
    //     auto parent = node->getParent();

    //     // A node with no parent must be processed.
    //     if (!parent) {
    //         ++it;
    //         continue;
    //     }

    //     // Use your existing, correct method to get the trajectory.
    //     const Trajectory& traj_to_parent = node->getParentTrajectory();

    //     // If the cached trajectory is invalid, the edge is broken. Keep the node for replanning.
    //     if (!traj_to_parent.is_valid) {
    //         ++it;
    //         continue;
    //     }

    //     bool is_edge_in_confirmed_conflict = false;
    //     // Check this specific edge against all dynamic obstacles.
    //     for (const auto& obstacle : obstacles) {
    //         if (!obstacle.is_dynamic) continue;

    //         // Stage 1: Time-Relevance Check.
    //         const double node_time_to_goal = node->getTimeToGoal();
    //         const double node_global_time = clock_->now().seconds() + (robot_current_time_to_goal_ - node_time_to_goal);

    //         if (std::abs(node_global_time - obstacle.last_update_time.seconds()) > 3.0) {
    //             continue; // This obstacle is not a threat at this time.
    //         }

    //         // Stage 2: The actual collision check.
    //         if (!obs_checker_->isTrajectorySafe(traj_to_parent, node_global_time)) {
    //             is_edge_in_confirmed_conflict = true;
    //             break; // Collision found; no need to check other obstacles for this edge.
    //         }
    //     }

    //     // Stage 3: Hysteresis logic.
    //     if (is_edge_in_confirmed_conflict) {
    //         node->bad_count++;
    //     } else {
    //         node->bad_count = 0;
    //     }

    //     // Only keep nodes that have failed consistently.
    //     if (node->bad_count >= 2) {
    //         ++it; // KEEP this node.
    //     } else {
    //         it = current.erase(it); // REMOVE this node; it's considered safe for now.
    //     }
    // }


//////////////////////////////////////////////////////
    // // ==============================================================================
    // // ================= CURRENT NODE VISUALIZATION CODE BLOCK ======================
    // // ==============================================================================
    // if (visualization_) {
    //     // Create a vector to hold the 2D positions of the nodes near obstacles.
    //     std::vector<Eigen::VectorXd> positions_to_visualize;
    //     positions_to_visualize.reserve(current.size());

    //     // Iterate through the indices of the nodes in the 'current' set.
    //     for (int node_index : current) {
    //         // Get the full state of the node from the tree.
    //         const Eigen::VectorXd& state = tree_.at(node_index)->getStateValue();
    //         // Extract the 2D spatial part (x, y) for visualization.
    //         positions_to_visualize.push_back(state.head<2>());
    //     }

    //     // Call the visualization function to draw these nodes in RViz.
    //     // We use a bright cyan color and a unique namespace to distinguish them.
    //     visualization_->visualizeNodes(positions_to_visualize, "map", 
    //                                  {0.0f, 1.0f, 1.0f},  // Cyan color
    //                                  "current_obstacle_nodes");
    // }
    // // ==============================================================================
    // // ======================= END OF VISUALIZATION CODE BLOCK ======================
    // ==============================================================================    



    if (current == samples_in_obstacles_ && current.size()!=tree_.size()) return false; // Early exit if nothing has changed

    bool force_repair = false;
    // Heuristic: If the set of nodes near obstacles contains nearly every node in the tree,
    // we should force a full re-check to handle potential obstacle movement within this large set.
    // Using a threshold like 90% is safer than an exact '==' check.
    /*
        My reason is if current is all of the nodes then samples_in_obstalces (prev) and current would make the addNewObstalce and etc to get skipped
    */
    if (current.size() >= tree_.size() * 0.9) {
        force_repair = true;
    }

    std::vector<int> added, removed;
    std::vector<int> cur, prev;

    if (ignore_sample) {
        for (int sample : current) {
            if (!samples_in_obstacles_.count(sample)) added.push_back(sample);
        }
        for (int sample : samples_in_obstacles_) {
            if (!current.count(sample)) removed.push_back(sample);
        }
    } else {
        
        for (int c : current) cur.push_back(c);
        for (int p : samples_in_obstacles_) prev.push_back(p);
    }

    if (ignore_sample) {
        if (!added.empty() || force_repair) handleAddedObstacleSamples(added);
        if (!removed.empty() || force_repair ) handleRemovedObstacleSamples(removed);
        samples_in_obstacles_ = std::move(current);


    } else {
        if (!cur.empty() || force_repair) handleAddedObstacleSamples(cur);
        if (!prev.empty() || force_repair) handleRemovedObstacleSamples(prev);
        samples_in_obstacles_ = current;

    }
    return true;
    // visualizeHeapAndUnvisited();


    // std::vector<Eigen::VectorXd> positions2;
    // for (const auto& y: v_open_set_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(y)->getStateValue();
    //     positions2.push_back(vec);
    // }
    // // std::string color_str = "0.0,0.0,1.0"; // Blue color
    // visualization_->visualizeNodes(positions2,"map");
    // v_open_set_.clear();
    



    /* THIS NOTE MAYBE OUTDATED!
        handling the problem of reconnecion to the inflation zone is not easy! you can't just keep track of the difference between added and current
        and you have to use current everytime in the handleAddedObstalceSamples because if you don't the added will only go forward (with a distance to the obstalce ofcourse)
        and the obstalce might move and end up on top of a edge but since the added is just a difference between previous and current iteration it doesnt cover the nodes on that edge
        so the timeline is this --> added --> then plan takes care of the nodes in the inflation zone --> obstalce move and end up on some long edge --> the added is far away and doesnt invalidate those long edge because it invalidated it in previous iteraion and moved on!
    */

}

// void FMTX::handleAddedObstacleSamples(const std::vector<int>& added) {
//     std::unordered_set<int> orphan_nodes;

//     /*
//         the 2 near function i put here because of rviz new goal feature i've added to the system and since the tree_ is cleared the neighbors need to be set again 
//     */
//     for (int idx : added) {
//         if (!ignore_sample && prune) {
//             auto node = tree_[idx].get();
//             near(idx);
//             for (auto& [neighbor, edge_info] : node->neighbors()) {
//                 if (edge_info.distance == INFINITY) continue;
//                 /*
//                     Note: i think we don't need to obstacle check for all the node->neighbor relatioship because i think we can utilize dualFindSample and instantly make nodes on samples_in_obstalce distances to INFINITY
//                           but for now since rrtx didn't do optimization in this part i keep the code this way
//                 */
//                 bool is_free_ = obs_checker_->isObstacleFree(node->getStateValue(), neighbor->getStateValue());
//                 if (is_free_) continue;

//                 edge_info.distance = INFINITY;
//                 near(neighbor->getIndex());
//                 neighbor->neighbors().at(node).distance = INFINITY;

//                 if (node->getParent() == neighbor){
//                     orphan_nodes.insert(node->getIndex()); // In the current tree_ we check if an edge connection the node and its parent is on obstalce or not, if it is, then we send it and its descendant to orphan list
//                     auto descendants = getDescendants(node->getIndex());
//                     orphan_nodes.insert(descendants.begin(), descendants.end());
//                 }
//                 if (neighbor->getParent() == node) { //We check bidirectionaly because we used "neighbor->neighbors().at(node).distance = INFINITY;" in above line
//                     orphan_nodes.insert(neighbor->getIndex());
//                     auto descendants = getDescendants(neighbor->getIndex());
//                     orphan_nodes.insert(descendants.begin(), descendants.end());
//                 }
//             }
//         }
//         else{
//             orphan_nodes.insert(idx);
//             auto descendants = getDescendants(idx);
//             orphan_nodes.insert(descendants.begin(), descendants.end());
//         }

//     }
//     /*
//         TO DO: Later i might create a hybrid approach to decide between prune true or false
//         Order of obstalce check for rrtx style is O(h ln(n)) --> h being the "added" variable and n being num of samples 
//         Order of obstalce check for fmt style is O(K) --> K being the number of orphan nodes
//     */
//     // std::cout << "Added samples: " << added.size()
//     //           << ", added.size() * ln(n): " << (added.size() * std::log(num_of_samples_))
//     //           << "\n";
//     // std::cout << "Orphan nodes count: " << orphan_nodes.size() << "\n";



//     /*
//         one might ask why do you put orphan nodes into v_unvisited_set when you have a mechanism in the main loop to find these automatically?! 
//         The reason is these help the finding of the v open nodes later in the update obstalce sample function
//         If we only rely on that mechansim we can't find connections to other branches because we are blind to see other branches! like on the other side of the tree
//         Imagine the one side of the plier and some nodes get better cost if they get connected to the other tip of the plier but since we didn't put the other side nodes into v open we never know!

//         (side note: Also imagine if the the two tips of the the plier is far apart so you can't rely on the neighborhood raidus of one side to get to the other!)

//         So that condtion in the main loop is just for one direction expansion and is good for the nodes that gor removed from the obstalce--> Although its a reasonable question here also to ask ourselves why its not the case
//         for the remove obstlace to now know their v open at first!
//         the difference between addObstalce and removeObstalce is adding and obstalce most certainly adds cost to orphan nodes
//         but removing an obstlace most certainly reduces cost of the neighbor nodes! and reducing happens in the current branch and direction of the expansion that happens in dijkstra like (like fmtx) algorithm 
//         so we don't need to worry about the other side of plier (per say!) because they are gonna connect to us! not us connecting to them (and by "us" i mean the current direction of the expansion)
//     */


//     // DOESNT MATTER IF THIS LOOP WOULD GO HIGHER OR LOWER THAN THE BELOW FOR LOOP BECAUSE THE VUNVISTED UPDATE LOOP IS GONNA HELP THE BELOW LOOP
//     for (auto node_index : orphan_nodes) {
//         // tree_.at(node_index)->in_unvisited_ = true;
//         auto node = tree_.at(node_index).get();
        
//         if (node->in_queue_) {
//             v_open_heap_.remove(node);
//             // node->in_queue_ = false;
//         }
//         if (!node->getIndex() == 0) // Root of the tree must keep its zero cost!
//             node->setCost(INFINITY); 
//         node->setParent(nullptr, INFINITY);
//         // node->getChildrenMutable().clear(); // We don't need to do this even though at this current iteration this node has children but they will be removed as we iterate by the setParent function
//         edge_length_[node_index] = -std::numeric_limits<double>::infinity();
//     }
  
//     /*
//         IMPORTNAT NOTE: My assumption is we do not need to update the queue here because we only need to ADD to queue. 
//         UPDATE IS WHEN A COST of node has changed and that happens only in the main plan function. here we only make the cost to inf and we removed. you may ask
//         how can you be sure we don't have any vopen heap nodes left? in partial update false the vopen heap gets fully cleaned because of the while loop but in partial update true
//         we early exit that loop so some vopen heap nodes are left! in this scenraio now imagine an obstalce is being added and at the worst case scenario it is being added to the region where there are already vopen heap nodes!
//         the result of adding obstalce means two things! ---> some vclosed (conceptually i mean because i don't use vclose in my algorithm!) nodes become vunvisted or some vopen nodes become vunvisted! and the previously vunvisited due to partial update
//         stays in vunvisted! mind that the cost of these per say messeup nodes will become infinity or stay infinity
//         for expansion we need CORRECT vopen heap nodes! (by correct i mean the correct cost that resembles the changes of the environment) and one rule you need to follow for safety is to not put any vunvisted node into vopen or if some vunvisted node is in vopen you need to remove it
//         so since the cost of nodes doesnt change to any numbers but inf! so we only need to remove them. the following addition to heap is also for covering the vunvisted node for the next batch of update in plan function
//         in the next for loop i do the heap removal

//         but in the plan function since i update the nodes cost because of the second condtion in the main if! i have to update the queue's priority --> and its only happening frequntly in obstalce removal(even though it might sometimes happens in the regular loop due to the main problem that fmt has in low sample number which is negligible when samples counters go to inf theoretically!)

//     */


//     /*
//         Time Complexity Comparison: Inserting k New Elements into a Heap of Size M
//         Method 1: Individual Insertions (add one by one)
//         Time Complexity:
//         O(klogM)
//         Each insertion takes O(logM) time (due to heapifyUp).

//         For k insertions: O(klogM).

//         Method 2: Bulk Insertion (bulkAdd)
//         Time Complexity: O(M+k)
//         Collecting elements: O(k).

//         Heap construction (Floyd’s method): 
//         O(M+k), where M is the existing heap size.

//         Which is Faster?
//         If k is small compared to M (k ≪ M)

//         O(klogM) vs.  O(M+k)≈O(M).
//         Since M might dominate klogM, Method 1 (individual insertions) could be faster.

//         If k is large (k ≈ M or k > M)
//         O(klogM) vs.  O(M+k)≈O(k).
//         Since k log ⁡ M ≫ k
//         klogM≫k, Method 2 (bulkAdd) wins.
//     */


//     // // Method 1

//     for (auto node_index : orphan_nodes) {
//         auto node = tree_.at(node_index).get();
//         near(node_index);
//         for (const auto& [neighbor, dist] : node->neighbors()){
//             int index = neighbor->getIndex();
//             if (neighbor->in_queue_ || neighbor->getCost()==INFINITY ) continue;
//             double h_value = use_heuristic ? heuristic(index) : 0.0;
//             v_open_heap_.add(neighbor , neighbor->getCost() + h_value);
//             // neighbor->in_queue_ = true;
//         }
//     }
// //////////////////////////////
//     // // // Method 2
//     // // 1) Gather & mark
//     // std::vector<std::pair<double, FMTNode*>> to_enqueue;
//     // // to_enqueue.reserve(orphan_nodes.size() * average_degree); // optional hint

//     // for (auto node_index : orphan_nodes) {
//     //     FMTNode* node = tree_.at(node_index).get();
//     //     near(node_index);  // ensure node->neighbors() is populated

//     //     for (const auto& [neighbor, dist] : node->neighbors()) {
//     //         // skip if already enqueued or not yet reachable
//     //         if (neighbor->in_queue_ || neighbor->getCost() == INFINITY) 
//     //             continue;

//     //         // mark so we don’t enqueue duplicates
//     //         neighbor->in_queue_ = true;

//     //         // compute priority (cost-to-come + optional heuristic)
//     //         double h_value = use_heuristic 
//     //                         ? heuristic(neighbor->getIndex()) 
//     //                         : 0.0;
//     //         double priority = neighbor->getCost() + h_value;

//     //         to_enqueue.emplace_back(priority, neighbor);
//     //     }
//     // }

//     // // 2) Bulk‐add into your custom priority queue in O(K)
//     // v_open_heap_.bulkAdd(to_enqueue);

//     // // Note: after bulkAdd, heap_index_ is set and in_queue_ remains true.
//     // // When you later pop/remove a node, your remove() method will reset in_queue_ = false.


// /////////////////////////////


  
// }

// void KinodynamicFMTX::handleAddedObstacleSamples(const std::vector<int>& added_indices) {
//     std::cout<<"added indices : "<<added_indices.size()<<"\n";


//     std::unordered_set<int> final_orphan_nodes;
//     for (int idx : added_indices) {
//         final_orphan_nodes.insert(idx);
//         auto descendants = getDescendants(idx); // Assuming getDescendants works correctly
//         final_orphan_nodes.insert(descendants.begin(), descendants.end());
//     }

//       // Process all final orphan nodes by resetting their state.
//     for (int node_index : final_orphan_nodes) {
//         auto node = tree_.at(node_index).get();
        
//         if (node->in_queue_) {
//             v_open_heap_.remove(node);
//         }
//         // Do not reset the root of the tree (index 0 is a common convention).
//         if (node->getIndex() != root_state_index_) {
//             node->setCost(INFINITY);
//             node->setTimeToGoal(std::numeric_limits<double>::infinity());
//         }
//         node->setParent(nullptr, Trajectory{});
//         edge_length_[node_index] = -std::numeric_limits<double>::infinity();
//     }
  
//     // Add valid neighbors of the now-orphaned region to the open heap to begin the repair.
//     for (int node_index : final_orphan_nodes) {
//         auto node = tree_.at(node_index).get();
//         if (!neighbor_precache)
//             near(node_index); // Ensure neighbors are loaded.

//         // Define a helper lambda to avoid repeating the queuing logic.
//         auto queue_if_valid = [&](FMTNode* neighbor_ptr) {
//             // Don't queue a neighbor if it's already in the heap or is an orphan itself.
//             if (neighbor_ptr->in_queue_ || neighbor_ptr->getCost() == INFINITY) return;
            
//             double h_value = use_heuristic ? heuristic(neighbor_ptr->getIndex()) : 0.0;
//             v_open_heap_.add(neighbor_ptr, neighbor_ptr->getCost() + h_value);
//         };

//         // Iterate over BOTH sets to find all valid neighbors on the boundary of the orphan set.
//         for (const auto& [neighbor_ptr, edge_data] : node->forwardNeighbors()){
//             queue_if_valid(neighbor_ptr);
//         }
//         for (const auto& [neighbor_ptr, edge_data] : node->backwardNeighbors()){
//             queue_if_valid(neighbor_ptr);
//         }
//     }
//     std::cout<<"orphans size"<<final_orphan_nodes.size()<<"\n";

// }


void KinodynamicFMTX::handleAddedObstacleSamples(const std::vector<int>& added_indices) {
    if (added_indices.empty()) {
        return;
    }
    std::cout << "added indices: " << added_indices.size() << "\n";

    // --- Step 1: Iteratively find all descendants (the orphan set) ---
    std::unordered_set<int> orphan_indices;
    std::queue<FMTNode*> propagation_queue;

    for (int idx : added_indices) {
        // The insert method returns a pair, the .second is a bool indicating if insertion happened.
        if (orphan_indices.insert(idx).second) {
            propagation_queue.push(tree_.at(idx).get());
        }
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

    // --- Step 2: Reset orphans and find the unique boundary for repair ---
    // Using a hash set is faster than adding to a vector and then sorting.
    std::unordered_set<FMTNode*> boundary_nodes_to_requeue;

    for (int node_index : orphan_indices) {
        auto node = tree_.at(node_index).get();

        // Reset the orphan node's state
        if (node->in_queue_) {
            v_open_heap_.remove(node);
        }
        if (node->getIndex() != root_state_index_) {
            node->setCost(INFINITY);
            node->setTimeToGoal(std::numeric_limits<double>::infinity());
        }
        node->setParent(nullptr, Trajectory{});
        edge_length_[node_index] = -std::numeric_limits<double>::infinity();

        // Check neighbors to find the boundary
        if (!neighbor_precache) {
            near(node_index);
        }
        
        auto find_boundary = [&](const auto& neighbors) {
            for (const auto& [neighbor_ptr, edge_data] : neighbors) {
                if (orphan_indices.find(neighbor_ptr->getIndex()) == orphan_indices.end()) {
                    // insert() automatically handles duplicates for us.
                    boundary_nodes_to_requeue.insert(neighbor_ptr);
                }
            }
        };

        find_boundary(node->forwardNeighbors());
        find_boundary(node->backwardNeighbors());
    }

    // --- Step 3: Add the unique boundary nodes to the open heap ---
    for (FMTNode* neighbor_ptr : boundary_nodes_to_requeue) {
        // This check is still useful as a safeguard, though the set ensures uniqueness.
        if (neighbor_ptr->in_queue_ || neighbor_ptr->getCost() == INFINITY) continue;
            
        double h_value = use_heuristic ? heuristic(neighbor_ptr->getIndex()) : 0.0;
        v_open_heap_.add(neighbor_ptr, neighbor_ptr->getCost() + h_value);
    }

    std::cout << "orphans size: " << orphan_indices.size() << "\n";
}




void KinodynamicFMTX::handleRemovedObstacleSamples(const std::vector<int>& removed_indices) {
    // --- PART 1: RE-VALIDATE PREVIOUSLY BLOCKED EDGES (GRAPH OPERATION) ---
    // This loop checks if any connections to the now-free nodes can be restored.
    for (const auto& node_index : removed_indices) {
        auto node = tree_[node_index].get();

        // The check for `in_queue_` is a good safeguard, though it's unlikely for a
        // node just removed from an obstacle to be in the queue with INF cost.
        if (node->in_queue_ && node->getCost() == INFINITY) {
            v_open_heap_.remove(node);
        }

        if (!ignore_sample && prune) {
            if (!neighbor_precache)
                near(node_index); // Ensure neighbor sets are populated.

            // Check incoming connections (neighbor -> node).
            for (auto& [neighbor, edge_info] : node->backwardNeighbors()) {
                if (edge_info.distance != INFINITY) continue; // Edge is already valid.

                // If the edge is now obstacle-free, restore its original cost.
                if (obs_checker_->isObstacleFree(neighbor->getStateValue(), node->getStateValue())) {
                    edge_info.distance = edge_info.distance_original;
                    // Restore the symmetric edge in the neighbor's forward set.
                    if (neighbor->forwardNeighbors().count(node)) {
                        neighbor->forwardNeighbors().at(node).distance = edge_info.distance_original;
                    }
                }
            }
            
            // Check outgoing connections (node -> neighbor).
            for (auto& [neighbor, edge_info] : node->forwardNeighbors()) {
                if (edge_info.distance != INFINITY) continue;

                if (obs_checker_->isObstacleFree(node->getStateValue(), neighbor->getStateValue())) {
                    edge_info.distance = edge_info.distance_original;
                    // Restore the symmetric edge in the neighbor's backward set.
                    if (neighbor->backwardNeighbors().count(node)) {
                        neighbor->backwardNeighbors().at(node).distance = edge_info.distance_original;
                    }
                }
            }
        }
    }

    // --- PART 2: QUEUE NEIGHBORS TO TRIGGER REWIRING (GRAPH OPERATION) ---
    // This loop adds the valid neighbors of the freed nodes to the open heap.
    // This prompts the planner to check if a better path now exists through this region.
    for (int node_index : removed_indices) {
        auto node = tree_[node_index].get();
        if (!neighbor_precache)
            near(node_index); // Ensure neighbors are loaded.

        // Helper lambda to avoid repeating the queuing logic.
        auto queue_if_valid = [&](FMTNode* neighbor_ptr) {
            // A neighbor is on the "boundary" if it's connected to the valid tree (cost != INF)
            // and not already scheduled for expansion (not in_queue_).
            if (neighbor_ptr->in_queue_ || neighbor_ptr->getCost() == INFINITY) {
                return;
            }
            
            double h_value = use_heuristic ? heuristic(neighbor_ptr->getIndex()) : 0.0;
            v_open_heap_.add(neighbor_ptr, neighbor_ptr->getCost() + h_value);
        };

        // Iterate over BOTH sets to find all valid neighbors on the boundary.
        for (const auto& [neighbor_ptr, edge_data] : node->forwardNeighbors()) {
            queue_if_valid(neighbor_ptr);
        }
        for (const auto& [neighbor_ptr, edge_data] : node->backwardNeighbors()) {
            queue_if_valid(neighbor_ptr);
        }
    }
}




//////////////////////////////////////////////////////////////////////////////////////////////////////

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

// std::vector<Eigen::VectorXd> KinodynamicFMTX::getPathPositions() const
// {
//     std::vector<Eigen::VectorXd> path_positions;
    
//     // Start from the node representing the robot's state in the tree.
//     FMTNode* current_node = robot_node_;

//     // Traverse the tree from the robot's node up to the root via parent pointers.
//     while (current_node != nullptr)
//     {
//         // Add the state of the current node to our path list.
//         // We know from our StateSpace that this will be a valid 3D vector.
//         path_positions.push_back(current_node->getStateValue());
        
//         // Move to the parent node for the next iteration.
//         current_node = current_node->getParent();
//     }

//     // Note: The path is constructed backward (from robot to root).
//     // The R2TROSManager is responsible for reversing it for execution.
//     return path_positions;
// }



// std::vector<Eigen::VectorXd> KinodynamicFMTX::getPathPositions() const
// {
//     std::vector<Eigen::VectorXd> final_executable_path;

//     // 1. Ensure a valid plan exists from our anchor node.
//     if (!robot_node_ || robot_node_->getCost() == INFINITY) {
//         return final_executable_path; // Return empty path if no solution.
//     }

//     // 2. Reconstruct the discrete part of the path, which runs BACKWARD from the anchor.
//     //    Path is [Leaf, Parent, ..., Root]
//     std::vector<Eigen::VectorXd> discrete_path_backward;
//     FMTNode* current_node = robot_node_;
//     while (current_node != nullptr) {
//         discrete_path_backward.push_back(current_node->getStateValue());
//         current_node = current_node->getParent();
//     }

//     if (discrete_path_backward.empty()) {
//         return final_executable_path;
//     }

//     // 3. Create the "bridge" trajectory from the robot's current continuous state
//     //    to the discrete anchor node (the leaf of the path).
//     const Eigen::VectorXd& connection_target = discrete_path_backward.front();
//     Trajectory bridge_traj = statespace_->steer(robot_continuous_state_, connection_target);

//     // 4. Stitch the path together in the correct FORWARD execution order.
//     if (bridge_traj.is_valid && !bridge_traj.path_points.empty()) {
//         // The bridge trajectory is already forward [Robot -> Leaf]. Add its points.
//         final_executable_path = bridge_traj.path_points;
//     } else {
//         // If steering fails, start the path from the robot's current state.
//         // A jump to the first discrete node will occur, but it prevents a crash.
//         std::cout<< "Bridge trajectory invalid. Path will jump to the discrete tree.\n";
//         final_executable_path.push_back(robot_continuous_state_);
//     }

//     // 5. Append the rest of the discrete path, avoiding duplicates.
//     //    The discrete path is backward [Leaf, Parent, ..., Root].
//     //    The bridge path already contains the Leaf, so we start from the Parent (index 1).
//     for (size_t i = 1; i < discrete_path_backward.size(); ++i) {
//         final_executable_path.push_back(discrete_path_backward[i]);
//     }

//     // The final path is now correctly ordered: [Robot_Continuous, ..., Leaf, Parent, ..., Root]
//     // No final reversal is needed.
//     return final_executable_path;
// }

// std::vector<Eigen::VectorXd> KinodynamicFMTX::getPathPositions() const
// {
//     std::vector<Eigen::VectorXd> final_executable_path;

//     // 1. Find the best node in the entire neighborhood to connect to.
//     // This combines the logic from the old setRobotState and the robust bridging loop.

//     const double search_radius = neighborhood_radius_; // Use the planner's radius
//     auto nearby_indices = kdtree_->radiusSearch(robot_continuous_state_.head<2>(), search_radius);

//     FMTNode* best_node_to_connect = nullptr;
//     Trajectory best_bridge_traj;
//     best_bridge_traj.is_valid = false;
//     double min_total_cost = std::numeric_limits<double>::infinity();

//     // Iterate through all nearby nodes that are part of a valid plan.
//     for (const auto& index : nearby_indices) {
//         FMTNode* candidate_node = tree_[index].get();
        
//         // Skip nodes that aren't connected to the goal.
//         if (candidate_node->getCost() == INFINITY) continue;

//         // Step A: Check for a kinodynamically valid bridge.
//         Trajectory bridge_candidate_traj = statespace_->steer(robot_continuous_state_, candidate_node->getStateValue());
        
//         if (bridge_candidate_traj.is_valid) {
//             // Step B: Check if the bridge is collision-free.
//             if (obs_checker_->isTrajectorySafe(bridge_candidate_traj, clock_->now().seconds())) {
                
//                 // This bridge is valid AND safe. Now, evaluate its total cost.
//                 double total_cost_via_candidate = bridge_candidate_traj.cost + candidate_node->getCost();

//                 if (total_cost_via_candidate < min_total_cost) {
//                     // This is the best connection found so far. Store it.
//                     min_total_cost = total_cost_via_candidate;
//                     best_node_to_connect = candidate_node;
//                     best_bridge_traj = bridge_candidate_traj;
//                 }
//             }
//         }
//     }

//     // 2. If a valid connection was found, build the final path.
//     if (best_node_to_connect) {
//         // The bridge portion of the path is from our best trajectory.
//         final_executable_path = best_bridge_traj.path_points;

//         // Now, reconstruct the discrete path starting from our chosen connection point.
//         FMTNode* current_node = best_node_to_connect->getParent();
//         while (current_node != nullptr) {
//             final_executable_path.push_back(current_node->getStateValue());
//             current_node = current_node->getParent();
//         }
//     } else {
//         // Fallback: If no valid, collision-free bridge can be found in the entire neighborhood.
//         RCLCPP_ERROR(rclcpp::get_logger("FMTX_Path"), "CRITICAL: Could not find any valid bridge to the plan. Robot is stranded.");
//         // Return an empty path to signal failure.
//     }

//     return final_executable_path;
// }





// std::vector<Eigen::VectorXd> KinodynamicFMTX::getPathPositions() const
// {
//     std::vector<Eigen::VectorXd> final_executable_path;
//     FMTNode* best_node_to_connect = nullptr;
//     Trajectory best_bridge_traj;
//     best_bridge_traj.is_valid = false;

//     // --- Search Parameters ---
//     double current_search_radius = neighborhood_radius_;
//     const int max_attempts = 4; // Number of times to expand the search radius
//     const double radius_multiplier = 1.8; // How much to increase the radius each time

//     // --- Iterative Broadening Search Loop ---
//     for (int attempt = 1; attempt <= max_attempts; ++attempt) {
//         auto nearby_indices = kdtree_->radiusSearch(robot_continuous_state_.head<2>(), current_search_radius);

//         double min_total_cost = std::numeric_limits<double>::infinity();

//         // This inner loop is the same as your original function, checking all candidates at the current radius
//         for (const auto& index : nearby_indices) {
//             FMTNode* candidate_node = tree_[index].get();
            
//             if (candidate_node->getCost() == INFINITY) continue;

//             Trajectory bridge_candidate_traj = statespace_->steer(robot_continuous_state_, candidate_node->getStateValue());
            
//             if (bridge_candidate_traj.is_valid) {
//                 if (obs_checker_->isTrajectorySafe(bridge_candidate_traj, clock_->now().seconds())) {
//                     double total_cost_via_candidate = bridge_candidate_traj.cost + candidate_node->getCost();
//                     if (total_cost_via_candidate < min_total_cost) {
//                         min_total_cost = total_cost_via_candidate;
//                         best_node_to_connect = candidate_node;
//                         best_bridge_traj = bridge_candidate_traj;
//                     }
//                 }
//             }
//         }

//         // --- Check if we found a connection in this attempt ---
//         if (best_node_to_connect) {
//             RCLCPP_INFO(rclcpp::get_logger("FMTX_Path"), "Found valid bridge on attempt %d with radius %.2f", attempt, current_search_radius);
//             break; // Success! Exit the iterative search loop.
//         }

//         // If no connection was found and we haven't reached the max attempts, expand the search.
//         if (attempt < max_attempts) {
//             RCLCPP_WARN(rclcpp::get_logger("FMTX_Path"), "Bridge search failed at radius %.2f. Expanding search.", current_search_radius);
//             current_search_radius *= radius_multiplier;
//         }
//     }


//     // // --- After all attempts, build the path or declare critical failure ---
//     // if (best_node_to_connect) {
//     //     final_executable_path = best_bridge_traj.path_points;

//     //     FMTNode* current_node = best_node_to_connect->getParent();
//     //     while (current_node != nullptr) {
//     //         final_executable_path.push_back(current_node->getStateValue());
//     //         current_node = current_node->getParent();
//     //     }
//     // } else {
//     //     // This error now only happens after we have exhausted all search attempts.
//     //     RCLCPP_ERROR(rclcpp::get_logger("FMTX_Path"), "CRITICAL: Could not find any valid bridge to the plan after %d attempts. Robot is stranded.", max_attempts);
//     // }

//         /*********************************************************************************/
//     /* --- THE FIX IS HERE: Correct Path Assembly ---                                */
//     /*********************************************************************************/
//     if (best_node_to_connect) {
//         // 1. Start the path with the smooth "bridge" from the robot to the tree.
//         final_executable_path = best_bridge_traj.path_points;

//         // 2. Iteratively generate and append the path for the rest of the tree segments.
//         FMTNode* child_node = best_node_to_connect;
//         FMTNode* parent_node = child_node->getParent();

//         while (parent_node != nullptr) {
//             // Generate the smooth path for the segment FROM the child TO its parent.
//             // (Remember, the plan is built backward from the goal).
//             Trajectory segment_traj = statespace_->steer(child_node->getStateValue(), parent_node->getStateValue());
            
//             if (segment_traj.is_valid && segment_traj.path_points.size() > 1) {
//                 // Append all points from this segment, *except the first one*,
//                 // to avoid duplicating the state of the child_node.
//                 final_executable_path.insert(
//                     final_executable_path.end(), 
//                     segment_traj.path_points.begin() + 1, 
//                     segment_traj.path_points.end()
//                 );
//             } else {
//                 // Fallback: If steering fails (shouldn't happen in a valid plan), 
//                 // just add the parent state to prevent a total failure.
//                 RCLCPP_WARN(rclcpp::get_logger("FMTX_Path"), "Steering failed during path reconstruction between nodes %d and %d.", child_node->getIndex(), parent_node->getIndex());
//                 final_executable_path.push_back(parent_node->getStateValue());
//             }

//             // Move up the tree for the next iteration.
//             child_node = parent_node;
//             parent_node = child_node->getParent();
//         }
//     } else {
//         RCLCPP_ERROR(rclcpp::get_logger("FMTX_Path"), "CRITICAL: Could not find any valid bridge to the plan after %d attempts. Robot is stranded.", max_attempts);
//     }
//     return final_executable_path;
// }


// std::vector<Eigen::VectorXd> KinodynamicFMTX::getPathPositions() const
// {
//     std::vector<Eigen::VectorXd> final_executable_path;
//     FMTNode* best_node_to_connect = nullptr;
//     Trajectory best_bridge_traj;
//     best_bridge_traj.is_valid = false;

//     // 1. Find closest feasible node
//     double current_search_radius = neighborhood_radius_;
//     const int max_attempts = 4;
//     const double radius_multiplier = 1.8;

//     for (int attempt = 1; attempt <= max_attempts; ++attempt) {
//         auto nearby_indices = kdtree_->radiusSearch(robot_continuous_state_.head<2>(), current_search_radius);
//         double min_total_cost = std::numeric_limits<double>::infinity();

//         for (auto idx : nearby_indices) {
//             FMTNode* candidate = tree_[idx].get();
//             if (candidate->getCost() == INFINITY) continue;

//             Trajectory bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());
//             if (!bridge.is_valid) continue;

//             double start_time = clock_->now().seconds();
//             if (!obs_checker_->isTrajectorySafe(bridge, start_time)) continue;

//             double cost = bridge.cost + candidate->getCost();
//             if (cost < min_total_cost) {
//                 min_total_cost = cost;
//                 best_node_to_connect = candidate;
//                 best_bridge_traj = bridge;
//             }
//         }

//         if (best_node_to_connect) break;
//         current_search_radius *= radius_multiplier;
//     }

//     // 2. If no connection found, abort
//     if (!best_node_to_connect || best_bridge_traj.path_points.empty()) {
//         RCLCPP_ERROR(rclcpp::get_logger("FMTX_Path_Assembly"),
//                      "No valid connection from robot to tree. Cannot build path.");
//         return {};
//     }

//     // 3. Start with bridge trajectory
//     final_executable_path = best_bridge_traj.path_points;

//     // 4. Traverse the tree using cached trajectories (not re-steering!)
//     FMTNode* child = best_node_to_connect;
//     FMTNode* parent = child->getParent();

//     while (parent) {
//         // Look up the cached trajectory from child to parent
//         const auto& neighbors = child->neighbors();
//         auto it = neighbors.find(parent);
//         if (it == neighbors.end()) {
//             RCLCPP_ERROR(rclcpp::get_logger("FMTX_Path_Assembly"),
//                          "Missing cached trajectory from node %d to parent %d.",
//                          child->getIndex(), parent->getIndex());
//             break;
//         }

//         const auto& edge_info = it->second;
//         const Trajectory& cached_traj = edge_info.cached_trajectory;

//         if (!edge_info.is_trajectory_computed || !cached_traj.is_valid || cached_traj.path_points.size() <= 1) {
//             RCLCPP_WARN(rclcpp::get_logger("FMTX_Path_Assembly"),
//                         "Invalid or trivial cached trajectory from node %d to parent %d.",
//                         child->getIndex(), parent->getIndex());
//             break;
//         }

//         // Append trajectory (excluding the first point to avoid duplicates)
//         for (size_t i = 1; i < cached_traj.path_points.size(); ++i) {
//             final_executable_path.push_back(cached_traj.path_points[i]);
//         }

//         // Move up the tree
//         child = parent;
//         parent = child->getParent();
//     }

//     return final_executable_path;
// }

std::vector<Eigen::VectorXd> KinodynamicFMTX::getPathPositions() const
{
    // 1. Check if the planner has a valid anchor point for the robot.
    //    (setRobotState should have found one).
    if (!robot_node_ || robot_node_->getCost() == INFINITY) {
        RCLCPP_ERROR(rclcpp::get_logger("FMTX_Path_Assembly"),
                     "Robot has no valid anchor node in the tree. Cannot build path.");
        return {}; // Return empty path
    }

    // 2. ✅ Generate the "bridge" trajectory from the robot's continuous state
    //    to the anchor node on the fly.
    Trajectory bridge_traj = statespace_->steer(robot_continuous_state_, robot_node_->getStateValue());

    if (!bridge_traj.is_valid) {
        RCLCPP_ERROR(rclcpp::get_logger("FMTX_Path_Assembly"),
                     "Failed to steer from robot's continuous state to the anchor node.");
        return {};
    }

    // 3. Start the final path with this bridge trajectory.
    std::vector<Eigen::VectorXd> final_executable_path = bridge_traj.path_points;

    // 4. Traverse the rest of the tree from the anchor node using parent pointers.
    FMTNode* child = robot_node_;
    FMTNode* parent = child->getParent();

    while (parent) {
        // Use the pre-computed trajectories cached in the graph during the `plan()` phase.
        // const auto& cached_traj = child->forwardNeighbors().at(parent).cached_trajectory;
        const Trajectory& cached_traj = child->getParentTrajectory();

        
        if (cached_traj.is_valid && cached_traj.path_points.size() > 1) {
            // Append all points from the segment except the first one to avoid duplicates.
            final_executable_path.insert(final_executable_path.end(),
                                         cached_traj.path_points.begin() + 1,
                                         cached_traj.path_points.end());
        } else {
            // If a valid cached trajectory doesn't exist, the path is broken.
            RCLCPP_WARN(rclcpp::get_logger("FMTX_Path_Assembly"), 
                        "Path reconstruction failed. Invalid cached trajectory between nodes %d and %d.", 
                        child->getIndex(), parent->getIndex());
            break;
        }
        child = parent;
        parent = child->getParent();
    }

    return final_executable_path;
}
void KinodynamicFMTX::setRobotState(const Eigen::VectorXd& robot_state) {
    // 1. Store the robot's continuous state
    robot_continuous_state_ = robot_state;

    // --- STABILIZATION FIX START ---

    // Define a hysteresis factor. A new path must be at least 5% cheaper to be adopted.
    // This prevents switching for negligible gains.
    const double hysteresis_factor = 0.80;
    double cost_of_current_path = std::numeric_limits<double>::infinity();

    // First, calculate the cost of sticking with the current anchor node, if it's valid.
    // This gives us a baseline to beat.
    if (robot_node_ && robot_node_->getCost() != INFINITY) {
        Trajectory bridge_to_current_anchor = statespace_->steer(robot_continuous_state_, robot_node_->getStateValue());
        if (bridge_to_current_anchor.is_valid && obs_checker_->isTrajectorySafe(bridge_to_current_anchor, clock_->now().seconds())) {
            cost_of_current_path = bridge_to_current_anchor.cost + robot_node_->getCost();
        }
    }
    // --- STABILIZATION FIX END ---

    // 2. Search for the best *potential* anchor node in the neighborhood.
    // This part of your logic remains unchanged.
    FMTNode* best_candidate_node = nullptr;
    Trajectory best_candidate_bridge;
    Trajectory current_bridge;
    double best_candidate_cost = std::numeric_limits<double>::infinity();
    double current_search_radius = neighborhood_radius_;
    const int max_attempts = 4;
    const double radius_multiplier = 1.2;

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        auto nearby_indices = kdtree_->radiusSearch(robot_continuous_state_.head(kd_dim), current_search_radius);
        
        double min_cost_in_radius = std::numeric_limits<double>::infinity();

        for (auto idx : nearby_indices) {
            FMTNode* candidate = tree_[idx].get();
            if (candidate->getCost() == INFINITY) continue;

            // CHANGED: Reuse 'current_bridge' object via assignment. No new object is constructed here.
            current_bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());
            
            if (!current_bridge.is_valid || !obs_checker_->isTrajectorySafe(current_bridge, clock_->now().seconds())) continue;

            double cost = current_bridge.cost + candidate->getCost();
            if (cost < min_cost_in_radius) {
                min_cost_in_radius = cost;
                best_candidate_node = candidate;
                
                // CHANGED: Use std::swap instead of copying. This is a very cheap move operation.
                std::swap(best_candidate_bridge, current_bridge);
                
                best_candidate_cost = cost; // Note: 'cost' was calculated with the old 'current_bridge' before the swap.
                                            // We need to re-calculate it with the new best, or better, use its own data.
                best_candidate_cost = best_candidate_bridge.cost + candidate->getCost(); // Correct way
            }
        }

        if (best_candidate_node) break;
        current_search_radius *= radius_multiplier;
    }

    // --- STABILIZATION FIX START ---

    // 3. Make a stable decision.
    // Only switch to the new candidate if it's significantly better than our current path.
    if (best_candidate_node && best_candidate_cost < cost_of_current_path * hysteresis_factor) {
        // The new node is significantly better. It's worth switching.
        robot_node_ = best_candidate_node;
        robot_current_time_to_goal_ = best_candidate_bridge.time_duration + best_candidate_node->getTimeToGoal();
    } else if (robot_node_) {
        // The new candidate is not significantly better, or none was found.
        // Stick with the old anchor node to maintain stability.
        // We still need to recalculate the time-to-go in case the tree costs updated.
        Trajectory bridge_to_kept_anchor = statespace_->steer(robot_continuous_state_, robot_node_->getStateValue());
        if (bridge_to_kept_anchor.is_valid) {
             robot_current_time_to_goal_ = bridge_to_kept_anchor.time_duration + robot_node_->getTimeToGoal();
        }
    } else {
        // This case handles when there was no previous anchor OR no valid new anchor.
        // If we found a candidate but didn't switch, we still need to set it for the first time.
        robot_node_ = best_candidate_node; // This will be nullptr if none found.
        if (robot_node_) {
             robot_current_time_to_goal_ = best_candidate_bridge.time_duration + best_candidate_node->getTimeToGoal();
        } else {
             robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
        }
    }
    // --- STABILIZATION FIX END ---
}

ExecutionTrajectory KinodynamicFMTX::getFinalExecutionTrajectory() const {
    ExecutionTrajectory final_traj;
    final_traj.is_valid = false; // Default to invalid

    // 1. Ensure a valid plan exists from the robot's anchor node.
    if (!robot_node_ || robot_node_->getCost() == INFINITY) {
        RCLCPP_ERROR(rclcpp::get_logger("FMTX_Path_Assembly"), "Robot has no valid anchor. Cannot build execution path.");
        return final_traj;
    }

    // 2. Generate the "bridge" trajectory from the robot's continuous state to the anchor.
    Trajectory bridge_traj = statespace_->steer(robot_continuous_state_, robot_node_->getStateValue());
    if (!bridge_traj.is_valid) {
        RCLCPP_ERROR(rclcpp::get_logger("FMTX_Path_Assembly"), "Failed to steer from robot to anchor node.");
        return final_traj;
    }

    // The bridge trajectory is the first piece of our final path.
    final_traj = bridge_traj.execution_data;
    final_traj.is_valid = true;

    // 3. Traverse the tree from the anchor node up to the root, appending each segment.
    FMTNode* child_node = robot_node_;
    FMTNode* parent_node = child_node->getParent();

    while (parent_node) {
        // Retrieve the pre-computed, cached trajectory for this tree edge.
        const auto& cached_traj = child_node->neighbors().at(parent_node).cached_trajectory;
        const auto& exec_data = cached_traj.execution_data;

        if (!cached_traj.is_valid || exec_data.Time.size() <= 1) {
            RCLCPP_WARN(rclcpp::get_logger("FMTX_Path_Assembly"), "Path reconstruction failed due to invalid cached segment.");
            final_traj.is_valid = false;
            return final_traj;
        }

        // --- Append matrices, avoiding duplicating the connection point ---
        // We take all but the first row of the new segment's data.
        long existing_rows = final_traj.Time.size();
        long new_rows = exec_data.Time.size() - 1;
        
        if (new_rows > 0) {
            final_traj.Time.conservativeResize(existing_rows + new_rows);
            final_traj.Time.tail(new_rows) = exec_data.Time.tail(new_rows);
            
            final_traj.X.conservativeResize(existing_rows + new_rows, Eigen::NoChange);
            final_traj.X.bottomRows(new_rows) = exec_data.X.bottomRows(new_rows);

            final_traj.V.conservativeResize(existing_rows + new_rows, Eigen::NoChange);
            final_traj.V.bottomRows(new_rows) = exec_data.V.bottomRows(new_rows);
            
            // Acceleration has one less row than states.
            long existing_a_rows = final_traj.A.rows();
            long new_a_rows = exec_data.A.rows(); // Append all accel rows for the new segment
            final_traj.A.conservativeResize(existing_a_rows + new_a_rows, Eigen::NoChange);
            final_traj.A.bottomRows(new_a_rows) = exec_data.A;
        }

        // Move up the tree for the next iteration.
        child_node = parent_node;
        parent_node = child_node->getParent();
    }
    
    // Update the total cost of the stitched trajectory
    if (final_traj.Time.size() > 0) {
        final_traj.total_cost = final_traj.Time.maxCoeff() - final_traj.Time.minCoeff();
    }

    return final_traj;
}




// bool KinodynamicFMTX::isPathStillValid(const std::vector<Eigen::VectorXd>& path, const Eigen::VectorXd& current_robot_state) const {
//     if (path.size() < 2) {
//         return true;
//     }

//     const double t_now = clock_->now().seconds();
//     const double robot_time_to_go = current_robot_state(current_robot_state.size() - 1);
//     const double t_arrival_predicted = t_now + robot_time_to_go;

//     // Find the segment the robot is currently on.
//     auto it = std::lower_bound(path.begin(), path.end(), robot_time_to_go,
//         [](const Eigen::VectorXd& point, double time_val) {
//             return point(point.size() - 1) > time_val;
//         });

//     size_t start_check_index = 0;
//     if (it != path.begin()) {
//         start_check_index = std::distance(path.begin(), std::prev(it));
//     }

//     // Check all future planned segments for safety.
//     for (size_t i = start_check_index; i < path.size() - 1; ++i) {
//         const Eigen::VectorXd& segment_start_state = path[i];
//         const Eigen::VectorXd& segment_end_state = path[i+1];

//         // --- FIX: Generate the TRUE kinodynamic trajectory for this segment ---
//         Trajectory segment_traj = statespace_->steer(segment_start_state, segment_end_state);

//         // If the segment itself is not valid, the whole path is invalid.
//         if (!segment_traj.is_valid) {
//             RCLCPP_WARN(rclcpp::get_logger("fmtx_validator"), "Path invalidated by invalid segment from state %zu to %zu.", i, i+1);
//             return false;
//         }

//         // Calculate the absolute world time when this segment starts.
//         const double segment_global_start_time = t_arrival_predicted - segment_start_state(segment_start_state.size() - 1);

//         // --- FIX: Check the safety of the TRUE trajectory ---
//         if (!obs_checker_->isTrajectorySafe(segment_traj, segment_global_start_time)) {
//             RCLCPP_WARN(rclcpp::get_logger("fmtx_validator"), "Path invalidated by predictive check on segment %zu.", i);
//             return false;
//         }
//     }

//     return true;
// }


bool KinodynamicFMTX::isPathStillValid(const std::vector<Eigen::VectorXd>& path, const Eigen::VectorXd& current_robot_state) const {
    if (path.size() < 2) {
        return true;
    }

    const double t_now = clock_->now().seconds();
    const double robot_time_to_go = current_robot_state(current_robot_state.size() - 1);
    const double t_arrival_predicted = t_now + robot_time_to_go;

    // Find the segment the robot is currently on.
    auto it = std::lower_bound(path.begin(), path.end(), robot_time_to_go,
        [](const Eigen::VectorXd& point, double time_val) {
            return point(point.size() - 1) > time_val;
        });

    size_t start_check_index = 0;
    if (it != path.begin()) {
        start_check_index = std::distance(path.begin(), std::prev(it));
    }

    // Check all future planned segments for safety.
    for (size_t i = start_check_index; i < path.size() - 1; ++i) {
        const Eigen::VectorXd& segment_start_state = path[i];
        const Eigen::VectorXd& segment_end_state = path[i+1];

        // *** ADD THIS CHECK ***
        // If the segment is extremely short, assume it's valid and continue.
        // A threshold of 0.01 seconds (10ms) is a reasonable choice.
        const double segment_duration = segment_start_state(segment_start_state.size() - 1) - segment_end_state(segment_end_state.size() - 1);
        if (segment_duration < 0.01) {
            continue;
        }
        // *********************

        // --- FIX: Generate the TRUE kinodynamic trajectory for this segment ---
        Trajectory segment_traj = statespace_->steer(segment_start_state, segment_end_state);

        // If the segment itself is not valid, the whole path is invalid.
        if (!segment_traj.is_valid) {
            RCLCPP_WARN(rclcpp::get_logger("fmtx_validator"), "Path invalidated by invalid segment from state %zu to %zu.", i, i+1);
            return false;
        }

        // Calculate the absolute world time when this segment starts.
        const double segment_global_start_time = t_arrival_predicted - segment_start_state(segment_start_state.size() - 1);

        // --- FIX: Check the safety of the TRUE trajectory ---
        if (!obs_checker_->isTrajectorySafe(segment_traj, segment_global_start_time)) {
            RCLCPP_WARN(rclcpp::get_logger("fmtx_validator"), "Path invalidated by predictive check on segment %zu.", i);
            return false;
        }
    }

    return true;
}




void KinodynamicFMTX::setStart(const Eigen::VectorXd& start) {
    root_state_index_ = statespace_->getNumStates();
    auto node = std::make_shared<FMTNode>(statespace_->addState(start),tree_.size());
    node->setCost(0);
    node->setTimeToGoal(0);
    // QueueElement2 new_element ={0,0};
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





std::unordered_set<int> KinodynamicFMTX::getDescendants(int node_index) {
    std::unordered_set<int> descendants;
    std::queue<FMTNode*> queue;
    
    // Start with the initial node
    queue.push(tree_[node_index].get());
    
    while (!queue.empty()) {
        FMTNode* current = queue.front();
        queue.pop();
        
        descendants.insert(current->getIndex());
        
        for (FMTNode* child : current->getChildren()) {
            queue.push(child);
        }
    }
    
    return descendants;
}
/*
    The following is for finding if we have any cycles in our graph while we are doing dfs/bfs to find descendant
*/


// std::unordered_set<int> KinodynamicFMTX::getDescendants(int node_index) {
//     std::unordered_set<int> descendants;
//     std::queue<FMTNode*> queue;
//     std::unordered_set<FMTNode*> processing; // Track nodes being processed
    
//     // Debugging variables
//     int cycle_counter = 0;
//     constexpr int MAX_CYCLE_WARNINGS = 5;
//     auto start_time = std::chrono::steady_clock::now();

//     FMTNode* start_node = tree_[node_index].get();
//     queue.push(start_node);
//     processing.insert(start_node);

//     while (!queue.empty()) {
//         // Check for infinite loops
//         if (++cycle_counter > tree_.size() * 2) {
//             auto duration = std::chrono::duration_cast<std::chrono::seconds>(
//                 std::chrono::steady_clock::now() - start_time
//             );
//             std::cerr << "CRITICAL WARNING: Potential infinite loop detected!\n"
//                       << "Current node: " << queue.front()->getIndex() << "\n"
//                       << "Elapsed time: " << duration.count() << "s\n"
//                       << "Descendants found: " << descendants.size() << "\n";
//             break;
//         }

//         FMTNode* current = queue.front();
//         queue.pop();
//         processing.erase(current);

//         // Check if we've already processed this node
//         if (!descendants.insert(current->getIndex()).second) {
//             if (cycle_counter < MAX_CYCLE_WARNINGS) {
//                 std::cerr << "Cycle detected! Already processed node: " 
//                           << current->getIndex() << "\n";
//             }
//             continue;
//         }

//         // Process children with cycle checks
//         const auto& children = current->getChildren();
//         for (FMTNode* child : children) {
//             if (processing.count(child)) {
//                 std::cerr << "Parent-child cycle detected!\n"
//                           << "Parent: " << current->getIndex() << "\n"
//                           << "Child: " << child->getIndex() << "\n";
//                 continue;
//             }

//             if (descendants.count(child->getIndex())) {
//                 std::cerr << "Cross-branch cycle detected!\n"
//                           << "Current branch: " << current->getIndex() << "\n"
//                           << "Existing descendant: " << child->getIndex() << "\n";
//                 continue;
//             }

//             queue.push(child);
//             processing.insert(child);
//         }

//         cycle_counter = 0; // Reset counter if we made progress
//     }

//     // Final check for partial cycles
//     if (!queue.empty()) {
//         std::cerr << "WARNING: Terminated early with " << queue.size()
//                   << " nodes remaining in queue\n";
//     }

//     return descendants;
// }



std::vector<Eigen::VectorXd> KinodynamicFMTX::getSmoothedPathPositions(int num_intermediates, int smoothing_passes) const {
    // Check for invalid inputs
    if (num_intermediates < 1) {
        throw std::invalid_argument("num_intermediates must be at least 1");
    }
    if (smoothing_passes < 0) {
        throw std::invalid_argument("smoothing_passes must be non-negative");
    }

    // Get the original path
    auto original_path = getPathPositions();
    if (original_path.empty()) {
        return original_path; // Return empty path if no points
    }


    // Interpolate the path
    auto interpolated_path = interpolatePath(original_path, num_intermediates);
    if (interpolated_path.empty()) {
        return interpolated_path; // Return empty path if interpolation fails
    }

    // Smooth the path
    auto smoothed_path = smoothPath(interpolated_path, smoothing_passes);
    return smoothed_path;
}


std::vector<Eigen::VectorXd> KinodynamicFMTX::interpolatePath(const std::vector<Eigen::VectorXd>& path, int num_intermediates) const {
    std::vector<Eigen::VectorXd> new_path;

    // Check for invalid inputs
    if (path.empty()) {
        return new_path;
    }
    if (num_intermediates < 1) {
        return path;
    }

    // Add the first point
    new_path.push_back(path[0]);

    // Interpolate between points
    for (size_t i = 1; i < path.size(); ++i) {
        const Eigen::VectorXd& prev = path[i-1];
        const Eigen::VectorXd& curr = path[i];

        // Check for valid points
        if (prev.size() != curr.size()) {
            throw std::runtime_error("Path points have inconsistent dimensions");
        }

        // Add interpolated points
        for (int j = 1; j <= num_intermediates; ++j) {
            double t = static_cast<double>(j) / (num_intermediates + 1);
            Eigen::VectorXd interpolated = prev + t * (curr - prev);
            new_path.push_back(interpolated);
        }

        // Add the current point
        new_path.push_back(curr);
    }

    return new_path;
}


std::vector<Eigen::VectorXd> KinodynamicFMTX::smoothPath(const std::vector<Eigen::VectorXd>& path, int window_size) const {
    // Check for invalid inputs
    if (path.size() <= 2 || window_size < 1) {
        return path; // Return original path if no smoothing is needed
    }

    std::vector<Eigen::VectorXd> smoothed_path = path;
    int half_window = window_size / 2;

    // Smooth each point
    for (size_t i = 0; i < path.size(); ++i) {
        int start = std::max(0, static_cast<int>(i) - half_window);
        int end = std::min(static_cast<int>(path.size() - 1), static_cast<int>(i) + half_window);
        int count = end - start + 1;

        // Check for valid points
        if (count <= 0) {
            throw std::runtime_error("Invalid smoothing window");
        }

        // Compute the average of the window
        Eigen::VectorXd sum = Eigen::VectorXd::Zero(path[0].size());
        for (int j = start; j <= end; ++j) {
            sum += path[j];
        }
        smoothed_path[i] = sum / count;
    }

    return smoothed_path;
}

// void KinodynamicFMTX::visualizeTree() {
//     if (partial_plot==true) {
//         std::vector<Eigen::VectorXd> nodes;
//         std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
//         double goal_node_cost = tree_.at(robot_state_index_)->getCost();
        
//         // Create a set to store valid nodes based on cost
//         std::unordered_set<int> valid_node_indices;

//         // Collect valid nodes
//         for (size_t i = 0; i < tree_.size(); ++i) {
//             if (tree_[i]->getCost() <= goal_node_cost) {
//                 nodes.push_back(tree_[i]->getStateValue());
//                 valid_node_indices.insert(i);
//             }
//         }

//         // Generate edges only for valid nodes
//         for (int index : valid_node_indices) {
//             int parent_index = tree_[index]->getParent()->getIndex();
//             if (parent_index != -1) {
//                 edges.emplace_back(tree_.at(parent_index)->getStateValue(), tree_.at(index)->getStateValue());
//             }
//         }
//         // Visualize nodes and edges
//         // visualization_->visualizeNodes(nodes);
//         visualization_->visualizeEdges(edges);
//     }
//     else {
//         std::vector<Eigen::VectorXd> tree_nodes;
//         std::vector<Eigen::VectorXd> vopen_positions;
//         std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;

    
//         for (const auto& tree_node : tree_) {
//             // Collect all tree nodes
//             tree_nodes.push_back(tree_node->getStateValue());

//             // Check if node is in vopen (in_queue_)
//             if (tree_node->in_queue_) {
//                 vopen_positions.push_back(tree_node->getStateValue());
//             }

//             // Collect edges
//             auto parent = tree_node->getParent();
//             if (parent) {
//                 edges.emplace_back(parent->getStateValue(), tree_node->getStateValue());
//             }
//         }
    
//         // // Visualize tree components
//         // visualization_->visualizeNodes(tree_nodes, "map", 
//         //                             std::vector<float>{1.0f, 0.0f, 0.0f},  // Red for tree
//         //                             "tree_nodes");
        
//         // // Visualize vopen nodes with different color/namespace
//         // visualization_->visualizeNodes(vopen_positions);

//         // Visualize edges
//         visualization_->visualizeEdges(edges, "map");
//     }


// }

// void KinodynamicFMTX::visualizeTree() {
//     // Using a more appropriate name since the edges are straight lines.
//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
//     // std::cout<<robot_node_->getCost()<<"\n";
//     // OPTIMIZATION: Reserve memory for one edge per node (except the root).
//     // This is more accurate and prevents reallocations.
//     if (!tree_.empty()) {
//         edges.reserve(tree_.size());
//     }
//     std::vector<Eigen::VectorXd> tree_nodes;

//     // Iterate through all nodes in the tree.
//     for (const auto& node_ptr : tree_) {
//         FMTNode* child_node = node_ptr.get();
//         FMTNode* parent_node = child_node->getParent();

//         tree_nodes.push_back(node_ptr->getStateValue());

//         // If a node has a parent, it forms a valid edge in the tree.
//         if (parent_node) {
//             // Add a single, straight-line edge from the parent's state to the child's state.
//             // No need to check for intermediate points.
//             edges.emplace_back(parent_node->getStateValue(), child_node->getStateValue());
//         }
//     }

//     visualization_->visualizeNodes(tree_nodes, "map", 
//                             std::vector<float>{0.0f, 1.0f, 0.0f},  // Red for tree
//                             "tree_nodes");
    
//     // Visualize the collected straight-line edges.
//     visualization_->visualizeEdges(edges, "map");
// }


// Edge + Nodes
void KinodynamicFMTX::visualizeTree() {
    // Using a more appropriate name since the edges are straight lines.
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    
    if (!tree_.empty()) {
        edges.reserve(tree_.size());
    }
    std::vector<Eigen::VectorXd> tree_nodes;

    // --- NEW VARIABLE TO COUNT CONNECTED NODES ---
    int connected_nodes_count = 0;

    // Iterate through all nodes in the tree.
    for (const auto& node_ptr : tree_) {
        FMTNode* child_node = node_ptr.get();
        FMTNode* parent_node = child_node->getParent();

        // --- COUNTING LOGIC ---
        // A node is considered "connected" if its cost is not INFINITY.
        // Dangling nodes that were never reached from the root will have infinite cost.
        if (child_node->getCost() != std::numeric_limits<double>::infinity()) {
            connected_nodes_count++;
        }

        // --- Original visualization logic ---
        tree_nodes.push_back(node_ptr->getStateValue());

        // If a node has a parent, it forms a valid edge in the tree.
        if (parent_node) {
            // Add a single, straight-line edge from the parent's state to the child's state.
            edges.emplace_back(parent_node->getStateValue(), child_node->getStateValue());
        }
    }

    // --- PRINT THE COUNTS ---
    std::cout << "[FMTX INFO] Total nodes available: " << tree_.size()
              << " | Nodes connected to graph: " << connected_nodes_count << std::endl;


    // Visualize all nodes that were sampled/loaded
    // visualization_->visualizeNodes(tree_nodes, "map", 
    //                         std::vector<float>{0.0f, 1.0f, 0.0f},  // Green color
    //                         "tree_nodes");
    
    // Visualize the edges forming the connected tree
    visualization_->visualizeEdges(edges, "map");
}

// // Edge + Nodes + Time Of Arrival
// void KinodynamicFMTX::visualizeTree() {
//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
//     if (!tree_.empty()) {
//         edges.reserve(tree_.size());
//     }
    
//     std::vector<Eigen::VectorXd> tree_nodes;

//     // --- NEW: Data structures for time-of-arrival text ---
//     std::vector<Eigen::Vector3d> text_positions;
//     std::vector<std::string> time_texts;
//     text_positions.reserve(tree_.size());
//     time_texts.reserve(tree_.size());

//     // --- NEW: Calculate the predicted global arrival time at the goal ---
//     const double t_now = (clock_) ? clock_->now().seconds() : 0.0;
//     const double best_known_time_to_goal = (robot_current_time_to_goal_ != std::numeric_limits<double>::infinity())
//                                            ? robot_current_time_to_goal_
//                                            : 0.0;
//     const double t_arrival_predicted = t_now + best_known_time_to_goal;
    
//     int connected_nodes_count = 0;

//     for (const auto& node_ptr : tree_) {
//         FMTNode* child_node = node_ptr.get();
//         FMTNode* parent_node = child_node->getParent();

//         if (child_node->getCost() != std::numeric_limits<double>::infinity()) {
//             connected_nodes_count++;

//             // --- NEW: Calculate and store the global arrival time text ---
//             const double node_time_to_goal = child_node->getTimeToGoal();
//             if (node_time_to_goal != std::numeric_limits<double>::infinity()) {
//                 const double node_global_arrival_time = t_arrival_predicted - node_time_to_goal;

//                 const Eigen::VectorXd& state = child_node->getStateValue();
//                 text_positions.emplace_back(state.x(), state.y(), (state.size() > 2) ? state.z() : 0.0);
                
//                 std::stringstream ss;
//                 ss << std::fixed << std::setprecision(2) << node_global_arrival_time;
//                 time_texts.push_back(ss.str());
//             }
//         }

//         tree_nodes.push_back(node_ptr->getStateValue());


//         if (parent_node) {
//             edges.emplace_back(parent_node->getStateValue(), child_node->getStateValue());
//         }
//     }

//     std::cout << "[FMTX INFO] Total nodes available: " << tree_.size()
//               << " | Nodes connected to graph: " << connected_nodes_count << std::endl;

//     // Visualize all nodes that were sampled/loaded
//     // visualization_->visualizeNodes(tree_nodes, "map", 
//     //                         std::vector<float>{0.0f, 1.0f, 0.0f},  // Green color
//     //                         "tree_nodes");


//     // --- Original visualization calls ---
//     visualization_->visualizeEdges(edges, "map");

//     // --- NEW: Call the text visualization function ---
//     if (visualization_ && !text_positions.empty()) {
//         visualization_->visualizeText(text_positions, time_texts, "map", "arrival_times");
//     }
// }



// // // Edge + Nodes + (Time Of Arrival, Time To Goal)
// void KinodynamicFMTX::visualizeTree() {
//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
//     if (!tree_.empty()) {
//         edges.reserve(tree_.size());
//     }
    
//     // Data structures for time-of-arrival text
//     std::vector<Eigen::Vector3d> text_positions;
//     std::vector<std::string> time_texts;
//     text_positions.reserve(tree_.size());
//     time_texts.reserve(tree_.size());

//     // Calculate the predicted global arrival time at the goal
//     const double t_now = (clock_) ? clock_->now().seconds() : 0.0;
//     const double best_known_time_to_goal = (robot_current_time_to_goal_ != std::numeric_limits<double>::infinity())
//                                            ? robot_current_time_to_goal_
//                                            : 0.0;
//     const double t_arrival_predicted = t_now + best_known_time_to_goal;
    
//     int connected_nodes_count = 0;

//     for (const auto& node_ptr : tree_) {
//         FMTNode* child_node = node_ptr.get();
//         FMTNode* parent_node = child_node->getParent();

//         if (child_node->getCost() != std::numeric_limits<double>::infinity()) {
//             connected_nodes_count++;

//             // Calculate and store the text for visualization
//             const double node_time_to_goal = child_node->getTimeToGoal();
//             if (node_time_to_goal != std::numeric_limits<double>::infinity()) {
//                 const double node_global_arrival_time = t_arrival_predicted - node_time_to_goal;

//                 const Eigen::VectorXd& state = child_node->getStateValue();
//                 text_positions.emplace_back(state.x(), state.y(), (state.size() > 2) ? state.z() : 0.0);
                
//                 // --- MODIFIED: Format the string as "(arrival, ttg)" ---
//                 std::stringstream ss;
//                 ss << "(" << std::fixed << std::setprecision(2) << node_global_arrival_time
//                    << ", " << std::fixed << std::setprecision(2) << node_time_to_goal << ")";
//                 time_texts.push_back(ss.str());
//             }
//         }

//         if (parent_node) {
//             edges.emplace_back(parent_node->getStateValue(), child_node->getStateValue());
//         }
//     }

//     std::cout << "[FMTX INFO] Total nodes available: " << tree_.size()
//               << " | Nodes connected to graph: " << connected_nodes_count << std::endl;

//     // Original visualization calls
//     visualization_->visualizeEdges(edges, "map");

//     // Call the text visualization function
//     if (visualization_ && !text_positions.empty()) {
//         visualization_->visualizeText(text_positions, time_texts, "map", "arrival_times");
//     }
// }



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
        edges.emplace_back(start_point, end_point);
    }

    // Use your existing visualization class to draw the edges.
    // We'll use a distinct namespace and color (e.g., green and thick) to see it clearly.
    if (visualization_) {
        // The last argument is a namespace to keep it separate from the main tree visualization.
        visualization_->visualizeEdges(edges, "map", "0.0,1.0,0.0", "executable_path");
    }
}


void KinodynamicFMTX::visualizeSmoothedPath(const std::vector<Eigen::VectorXd>& shortest_path_) {
    // Extract nodes and edges from the smoothed path
    std::vector<Eigen::VectorXd> nodes;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;

    // // Add nodes to the list
    // for (const auto& point : shortest_path_) {
    //     nodes.push_back(point); // Each point in the smoothed path is a node
    // }

    // Add edges to the list
    for (size_t i = 1; i < shortest_path_.size(); ++i) {
        edges.emplace_back(shortest_path_[i - 1], shortest_path_[i]); // Create edges between consecutive points
    }

    // Use the visualization class to visualize nodes and edges
    if (visualization_) {
        // visualization_->visualizeNodes(nodes); // Visualize the nodes
        visualization_->visualizeEdges(edges, "map", "0.0,1.0,0.0"); // Visualize the edges in green
    } 
}


void KinodynamicFMTX::visualizeHeapAndUnvisited() {
    std::vector<Eigen::VectorXd> vopen_positions;
    bool found_conflict = false;

    const auto& heap_elements = v_open_heap_.getHeap();
    
    for (const auto& element : heap_elements) {
        // Access priority with element.first and node with element.second
        if (element.first == INFINITY) {
            std::cerr << "Warning: Node " << element.second->getIndex() 
                      << " is in v_open_heap_ but has INF cost!" << std::endl;
            found_conflict = true;
        }

        Eigen::VectorXd vec(2);
        vec << element.second->getStateValue()(0), element.second->getStateValue()(1);
        vopen_positions.push_back(vec);
    }

    if (found_conflict) {
        std::cerr << "There were nodes in v_open_heap_ with INF cost!" << std::endl;
    }

    // visualization_->visualizeNodes(vopen_positions);
    visualization_->visualizeNodes(vopen_positions, "map", std::vector<float>{0.0f,1.0f,0.0f}, "vopen");

}
//////////////////////////////////////////////
void KinodynamicFMTX::dumpTreeToCSV(const std::string& filename) const {
    std::ofstream fout(filename);
    if (!fout.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing\n";
        return;
    }
    // 1) figure out dimension of states
    if (tree_.empty()) {
        std::cerr << "Tree is empty. Nothing to dump.\n";
        return;
    }
    size_t dim = tree_[0]->getStateValue().size();
    // 2) write CSV header
    fout << "node_id";
    for (size_t d = 0; d < dim; ++d) {
        fout << ",x" << d;
    }
    fout << ",parent_id\n";

    // 3) for each node in tree_, write: node_id, coords..., parent_id
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




/////////////////////////////////////////corrected for the is prune true case to use the isTrajectorySafe:

// void KinodynamicFMTX::handleAddedObstacleSamples(const std::vector<int>& added_indices) {
//     std::unordered_set<int> nodes_to_make_orphan_and_process_neighbors;

//     for (int idx : added_indices) {
//         FMTNode* node = tree_[idx].get();
//         bool node_itself_is_now_in_obstacle = false;

//         if (ignore_sample) {
//             samples_in_obstacles_.insert(idx);
//             node_itself_is_now_in_obstacle = true;
//         } else if (prune) {
//             if (!obs_checker_->isObstacleFree(node->getStateValue())) {
//                 node_itself_is_now_in_obstacle = true;
//             }
//         }

//         if (!ignore_sample && prune && node_itself_is_now_in_obstacle) {
//             // Node 'idx' itself is in an obstacle. Invalidate all its edges.
//             near(idx);
//             for (auto& [neighbor, edge_info] : node->neighbors()) {
//                 edge_info.distance = INFINITY;
//                 near(neighbor->getIndex());
//                 if (neighbor->neighbors().count(node)) {
//                     neighbor->neighbors().at(node).distance = INFINITY;
//                 }
//                 if (node->getParent() == neighbor) {
//                     nodes_to_make_orphan_and_process_neighbors.insert(node->getIndex());
//                 }
//                 if (neighbor->getParent() == node) {
//                     nodes_to_make_orphan_and_process_neighbors.insert(neighbor->getIndex());
//                 }
//             }
//             nodes_to_make_orphan_and_process_neighbors.insert(idx);

//         } else if (!ignore_sample && prune && !node_itself_is_now_in_obstacle) {
//             // Node is fine, but check its edges individually using the new method.
//             near(idx);
//             for (auto& [neighbor, edge_info] : node->neighbors()) {
//                 if (edge_info.distance == INFINITY) continue;

//                 // --- START OF MODIFIED LOGIC ---
//                 // 1. Ensure the trajectory for this edge is computed and cached.
//                 //    We check the trajectory from the potential child (node) to the potential parent (neighbor).
//                 if (!edge_info.is_trajectory_computed) {
//                     edge_info.cached_trajectory = statespace_->steer(node->getStateValue(), neighbor->getStateValue());
//                     edge_info.is_trajectory_computed = true;
//                 }
                
//                 // 2. Perform the predictive safety check using the cached trajectory.
//                 bool is_safe = true;
//                 if (edge_info.cached_trajectory.is_valid) {
//                     // Since we are reacting to a change now, we check the trajectory's safety
//                     // as if it were to start at the current time. This is a necessary heuristic.
//                     const double global_start_time_heuristic = clock_->now().seconds();
//                     is_safe = obs_checker_->isTrajectorySafe(edge_info.cached_trajectory, global_start_time_heuristic);
//                 } else {
//                     is_safe = false; // An invalid trajectory is never safe.
//                 }

//                 if (!is_safe) {
//                     // --- END OF MODIFIED LOGIC ---
//                     // Edge is blocked
//                     edge_info.distance = INFINITY;
//                     near(neighbor->getIndex());
//                     if (neighbor->neighbors().count(node)) {
//                         neighbor->neighbors().at(node).distance = INFINITY;
//                     }

//                     // Handle parent relationships leading to orphans
//                     if (node->getParent() == neighbor) {
//                         nodes_to_make_orphan_and_process_neighbors.insert(node->getIndex());
//                     }
//                     if (neighbor->getParent() == node) {
//                         nodes_to_make_orphan_and_process_neighbors.insert(neighbor->getIndex());
//                     }
//                 }
//             }
//         } else {
//             // Original logic for ignore_sample or non-pruning modes.
//             nodes_to_make_orphan_and_process_neighbors.insert(idx);
//         }
//     }

//     // --- (The rest of the function for processing orphans remains the same) ---
//     std::unordered_set<int> final_orphan_nodes;
//     for (int orphan_idx : nodes_to_make_orphan_and_process_neighbors) {
//         final_orphan_nodes.insert(orphan_idx);
//         auto descendants = getDescendants(orphan_idx);
//         final_orphan_nodes.insert(descendants.begin(), descendants.end());
//     }

//     for (int node_index : final_orphan_nodes) {
//         auto node = tree_.at(node_index).get();
//         if (node->in_queue_) {
//             v_open_heap_.remove(node);
//         }
//         if (node->getIndex() != 0) {
//             node->setCost(INFINITY); 
//             node->setTimeToGoal(std::numeric_limits<double>::infinity());
//         }
//         node->setParent(nullptr, INFINITY);
//         edge_length_[node_index] = -std::numeric_limits<double>::infinity();
//     }
  
//     for (int node_index : final_orphan_nodes) {
//         auto node = tree_.at(node_index).get();
//         near(node_index);
//         for (const auto& [neighbor_ptr, edge_data] : node->neighbors()){
//             if (neighbor_ptr->in_queue_ || neighbor_ptr->getCost() == INFINITY) continue;
//             double h_value = use_heuristic ? heuristic(neighbor_ptr->getIndex()) : 0.0;
//             v_open_heap_.add(neighbor_ptr, neighbor_ptr->getCost() + h_value);
//         }
//     }
// }

// void KinodynamicFMTX::handleRemovedObstacleSamples(const std::vector<int>& removed) {
//     for (const auto& node_index : removed) {
//         auto node = tree_[node_index].get();
//         if (node->in_queue_ && node->getCost() == INFINITY) {
//             v_open_heap_.remove(node);
//         }

//         if (!ignore_sample && prune) {
//             near(node_index);
//             for (auto& [neighbor, edge_info] : node->neighbors()) {
//                 if (edge_info.distance != INFINITY) continue;

//                 // --- START OF MODIFIED LOGIC ---
//                 // 1. Ensure the trajectory is computed for this previously blocked edge.
//                 if (!edge_info.is_trajectory_computed) {
//                     edge_info.cached_trajectory = statespace_->steer(node->getStateValue(), neighbor->getStateValue());
//                     edge_info.is_trajectory_computed = true;
//                 }

//                 // 2. Check if the trajectory is now safe.
//                 bool is_now_safe = false;
//                 if (edge_info.cached_trajectory.is_valid) {
//                     const double global_start_time_heuristic = clock_->now().seconds();
//                     is_now_safe = obs_checker_->isTrajectorySafe(edge_info.cached_trajectory, global_start_time_heuristic);
//                 }

//                 if (is_now_safe) {
//                 // --- END OF MODIFIED LOGIC ---
//                     edge_info.distance = edge_info.distance_original;
//                     near(neighbor->getIndex());
//                     if (neighbor->neighbors().count(node)) {
//                         neighbor->neighbors().at(node).distance = edge_info.distance_original;
//                     }
//                 }
//             }
//         }
//     }

//     // --- (The rest of the function for queueing neighbors remains the same) ---
//     for (int node_index : removed) {
//         auto node = tree_.at(node_index).get();
//         near(node_index);
//         for (const auto& [neighbor, dist] : node->neighbors()) {
//             const int n_idx = neighbor->getIndex();
//             if (neighbor->in_queue_ || neighbor->getCost() == INFINITY) continue;
//             double h_value = use_heuristic ? heuristic(n_idx) : 0.0;
//             v_open_heap_.add(neighbor , neighbor->getCost() + h_value);
//         }
//     }
// }
