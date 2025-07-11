#include "motion_planning/planners/kinodynamic/kinodynamic_rrtx.hpp"

KinodynamicRRTX::KinodynamicRRTX(std::shared_ptr<StateSpace> statespace, 
    std::shared_ptr<ProblemDefinition> problem_def,
    std::shared_ptr<ObstacleChecker> obs_checker): statespace_(statespace), problem_(problem_def), obs_checker_(obs_checker){
        std::cout<<"KinodynamicRRTX constructor \n";
}


void KinodynamicRRTX::setStart(const Eigen::VectorXd& start) {
    robot_state_index_ = statespace_->getNumStates();
    auto node = std::make_shared<RRTxNode>(statespace_->addState(start) ,  tree_.size());
    tree_.push_back(node);
    node->setTimeToGoal(0);

    std::cout << "RRTX: Start node created on Index: " << robot_state_index_ << "\n";
}
void KinodynamicRRTX::setGoal(const Eigen::VectorXd& goal) {
    root_state_index_ = statespace_->getNumStates();
    auto node = std::make_shared<RRTxNode>(statespace_->addState(goal) ,  tree_.size());
    vbot_index_ = 1;
    vbot_node_ = node.get();
    node->setTimeToGoal(std::numeric_limits<double>::infinity());

    
    tree_.push_back(node); // Fixed parenthesis
    std::cout << "KinodynamicRRTX: Goal node created on Index: " << root_state_index_ << "\n";
}

void KinodynamicRRTX::setClock(rclcpp::Clock::SharedPtr clock) {
    clock_ = clock;
}

std::vector<int> KinodynamicRRTX::getPathIndex() const {
    std::vector<int> path;
    int idx = vbot_index_;
    while (idx != -1) {
        path.push_back(idx);
        RRTxNode* current_node = tree_.at(idx).get();
        RRTxNode* parent = current_node->getParent();
        if (parent == nullptr) {
            break;
        }
        idx = parent->getIndex();
    }
    return path;
}


// This function should be part of the KinodynamicRRTX class
std::vector<Eigen::VectorXd> KinodynamicRRTX::getPathPositions() const {
    // 1. Check if the planner has a valid anchor point for the robot.
    if (!vbot_node_ || vbot_node_->getCost() == INFINITY) {
        RCLCPP_ERROR(rclcpp::get_logger("RRTX_Path_Assembly"),
                     "Robot has no valid anchor node in the tree. Cannot build path.");
        return {}; // Return empty path
    }

    // 2. Generate the "bridge" trajectory from the robot's continuous state
    //    to the anchor node on the fly.
    Trajectory bridge_traj = statespace_->steer(robot_continuous_state_, vbot_node_->getStateValue());

    if (!bridge_traj.is_valid) {
        RCLCPP_ERROR(rclcpp::get_logger("RRTX_Path_Assembly"), // Note: Typo fixed from FMTX to RRTX
                     "Failed to steer from robot's continuous state to the anchor node.");
        return {};
    }

    // 3. Start the final path with this bridge trajectory.
    std::vector<Eigen::VectorXd> final_executable_path = bridge_traj.path_points;

    // 4. Traverse the rest of the tree from the anchor node using parent pointers.
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


void KinodynamicRRTX::setRobotIndex(const Eigen::VectorXd& robot_position) {
    robot_position_ = robot_position;

    const double MAX_SEARCH_RADIUS = 5.0; // Meters
    std::vector<size_t> nearest_indices = kdtree_->radiusSearch(robot_position, MAX_SEARCH_RADIUS);

    size_t best_index = std::numeric_limits<size_t>::max(); 
    double min_total_cost = std::numeric_limits<double>::max();
    RRTxNode* best_node = nullptr; 

    for (size_t index : nearest_indices) {
        auto node = tree_.at(index).get();
        if (node->getCost() == std::numeric_limits<double>::infinity()) continue;

        Eigen::VectorXd node_position = node->getStateValue();  // Fixed typo: getStateValue -> getStateValue
        double distance_to_node = (node_position - robot_position).norm();
        double total_cost = distance_to_node + node->getCost();

        if (total_cost < min_total_cost) {
            min_total_cost = total_cost;
            best_index = index;
            best_node = node;
        }
    }

    if (best_index != std::numeric_limits<size_t>::max()) {
        vbot_index_ = best_node->getIndex();
        vbot_node_ = best_node; // Directly assign the raw pointer
        return;
    }

    bool keep_prev_state_ = false;
    if (vbot_node_ && keep_prev_state_ == true) {
        std::cout << "No valid node found in neighborhood. Keeping previous vbot_node_.\n";
        return;
    }
    if (vbot_node_) {
        std::cout << "No valid node found in neighborhood. Setting to nearest node.\n";
        std::vector<size_t> nearest_indices = kdtree_->knnSearch(robot_position, 1);
        int nearest = nearest_indices.empty() ? -1 : static_cast<int>(nearest_indices[0]);  
        vbot_node_ = tree_.at(nearest).get();
        vbot_index_ = vbot_node_->getIndex();
        return;
    }

    vbot_index_ = -1;
    vbot_node_ = nullptr; 
    std::cout << "No valid node found and no previous vbot_node_. Setting vbot_node_ to nullptr.\n";
}

void KinodynamicRRTX::clearPlannerState() {
    // Clear all critical variables
    // start_.reset();
    // goal_.reset();
    // path_.clear();

    // for (auto& node : tree_) {
    //     node.reset();  // Explicitly reset each shared_ptr
    // }


    // Clear the inconsistency queue
    inconsistency_queue_.clear();

    for (auto& node : tree_) {
        node->disconnectFromGraph();  // Safe to call even if nodes are shared
        node.reset();
    }

    tree_.clear();  // Clear the vector

    // Reset the StateSpace
    statespace_->reset();

    // // Clear neighbor management structures
    // N0_in_.clear();
    // N0_out_.clear();
    // Nr_in_.clear();
    // Nr_out_.clear();


    // // Clear distance dictionary
    // distance_.clear();





    // if(kdtree_)
    //     kdtree_->clearData();
    kdtree_.reset();

    samples_in_obstacles_.clear();
    // obstacle_samples_.clear();



    // Clear edge length map and reset max length
    edge_length_.clear();
    max_length_edge_ind = -1;
    max_length = -std::numeric_limits<double>::infinity();

    // Reset indices
    vbot_index_ = -1;
    vgoal_index_ = -1;
    root_state_index_ = -1;
    robot_state_index_ = -1;

    Vc_T_.clear();

    // Clear the sample counter
    sample_counter = 0;



}


void KinodynamicRRTX::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    auto start = std::chrono::high_resolution_clock::now();
    clearPlannerState();

    sample_counter = 0;


    visualization_ = visualization;



    num_of_samples_ = params.getParam<int>("num_of_samples");
    partial_update = params.getParam<bool>("partial_update");
    ignore_sample = params.getParam<bool>("ignore_sample");
    static_obs_presence = params.getParam<bool>("static_obs_presence");

    lower_bounds_ = problem_->getLowerBound();
    upper_bounds_ = problem_->getUpperBound();
    use_kdtree = params.getParam<bool>("use_kdtree");
    std::string kdtree_type = params.getParam<std::string>("kdtree_type");


    if (use_kdtree == true && kdtree_type == "NanoFlann"){
        Eigen::VectorXd weights(2);
        // weights << 1.0, 1.0, 1.0; // Weights for x, y, time
        weights << 1.0, 1.0; // Weights for x, y,
        kdtree_ = std::make_shared<WeightedNanoFlann>(2, weights);
    } else if (use_kdtree == true && kdtree_type == "LieKDTree"){
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

    // put the start and goal node in kdtree
    if (use_kdtree == true) {
        // kdtree_->addPoints(statespace_->getSamplesCopy());
        // kdtree_->buildTree();

        // 1. Get the full 3D (or 4D) samples from the state space.
        Eigen::MatrixXd all_samples = statespace_->getSamplesCopy();

        // 2. Define how many spatial dimensions you have.
        //    This makes the code robust for future changes (e.g., to 3D space).
        //    Assuming (x, y, time), the spatial dimension is 2.
        int spatial_dimension = 2; // For (x, y)
        // For a future Dubins (x, y, theta, time) planner, this would still be 2.

        // 3. Use .leftCols() to create a new matrix with only the spatial data.
        //    .eval() is used to ensure we pass a concrete matrix, not a temporary expression.
        Eigen::MatrixXd spatial_samples_only = all_samples.leftCols(spatial_dimension).eval();
        
        // 4. Pass the 2D spatial matrix to the KD-tree.
        kdtree_->addPoints(spatial_samples_only);
        
        // 5. Build the tree all at once after we fill the data.
        kdtree_->buildTree();
    }

    /////////////////////////SETTING UP DS//////////////


    tree_.at(0)->setCost(0);
    tree_.at(0)->setLMC(0);

    // MAYBE it would be better to just set the max edge length to delta (max extension!) instead of calculating it
    edge_length_[0] = -std::numeric_limits<double>::infinity();
    edge_length_[1] = -std::numeric_limits<double>::infinity();

    v_indices_.insert(0);
    // v_indices_.insert(1); 
    ///////////////////Neighborhood Radius////////////////////////////////
    dimension_ = statespace_->getDimension();
    int d = dimension_;
    // double mu = std::pow(problem_->getUpperBound()[0] - problem_->getLowerBound()[0] , 2);
    Eigen::VectorXd range = upper_bounds_ - lower_bounds_;
    double mu = range.prod(); // .prod() computes the product of all coefficients
    std::cout<<"mu "<<mu<<"\n";
    double zetaD = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
    // gamma_ = 2 * std::pow(1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); //FMT star gamma
    gamma_ = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); //RRT star gamma



    // Since i want to put a cap on the number of samples and i want RRTX to be as close as to FMTX im gonna set step size (delta) to this:
    factor = params.getParam<double>("factor");
    std::cout<<"factor: "<<factor<<"\n";
    delta = factor * gamma_ * std::pow(std::log(num_of_samples_) / num_of_samples_, 1.0 / d);
    // delta = 15.0;
    std::cout << "Computed value of delta: " << delta << std::endl;



    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "---\n";

    sample_counter = 1;
}


// Well for now I use this for the first tree creation
void KinodynamicRRTX::plan() {

    auto start = std::chrono::high_resolution_clock::now();
    // if (cap_samples_==true && sample_counter < num_of_samples_) { // TODO: later when you add the robot you can put the condtion of the while loop here and we use the while true outside because we want it to always work to update the gazebo obstale positions
    while ( cap_samples_==true && sample_counter < num_of_samples_) { // TODO: later when you add the robot you can put the condtion of the while loop here and we use the while true outside because we want it to always work to update the gazebo obstale positions
        neighborhood_radius_ = shrinkingBallRadius();
        // neighborhood_radius_ = 10.0;
        // delta = neighborhood_radius_;

        Eigen::VectorXd sample = Eigen::VectorXd::Random(dimension_);
        sample = (lower_bounds_.array() + (upper_bounds_ - lower_bounds_).array() * ((sample.array() + 1.0) / 2.0)).matrix();

        sample_counter++;
        std::vector<size_t> nearest_indices = kdtree_->knnSearch(sample.head(2), 1);
        RRTxNode* nearest_node = tree_[nearest_indices[0]].get();
        Eigen::VectorXd nearest_state = nearest_node->getStateValue();
        
        // Steer towards sample
        // Calculate direction and distance using only the first two elements
        Eigen::Vector2d direction_2d = sample.head(2) - nearest_state.head(2);
        double distance_2d = direction_2d.norm();

        // Check if the 2D distance exceeds the step size
        if (distance_2d > delta) {
            // Update only the first two elements of the original 'sample' vector
            sample.head(2) = nearest_state.head(2) + (direction_2d / distance_2d) * delta;
        }

        // Attempt to extend tree
        bool node_added = false;
        if (obs_checker_->isObstacleFree(sample)) {
            node_added = extend(sample);
        }
            
        if (node_added) {
            RRTxNode* new_node = tree_.back().get();
            
            // Add to active node set
            v_indices_.insert(tree_.size()-1);
            
            // Update node costs and neighbors
            rewireNeighbors(new_node);
            reduceInconsistency();
            new_node->setCost(new_node->getLMC());
        }
        
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Planning time: " << duration.count() << " ms" << std::endl;



    // ===== Add this verification loop at the end =====
    bool has_inconsistency = false;
    int edge_counter = 0;

    // Loop through all nodes in the tree
    for (const auto& node_ptr : tree_) {
        // Check outgoing edges
        for (const auto& [neighbor, edge_info] : node_ptr->outgoingEdges()) {
            edge_counter++;
            
            // Check if distance != distance_original (unexpected)
            if (edge_info.distance != edge_info.distance_original) {
                std::cerr << "ERROR: Outgoing edge mismatch at node " << node_ptr->getIndex()
                          << " -> " << neighbor->getIndex() 
                          << " | dist=" << edge_info.distance 
                          << ", dist_original=" << edge_info.distance_original << "\n";
                has_inconsistency = true;
            }

            // Check if distance_original is zero (invalid)
            if (edge_info.distance_original == 0.0) {
                std::cerr << "WARNING: Zero distance_original at node " << node_ptr->getIndex()
                          << " -> " << neighbor->getIndex() 
                          << " | State1: " << node_ptr->getStateValue().transpose()
                          << " | State2: " << neighbor->getStateValue().transpose() << "\n";
            }
        }

        // Check incoming edges
        for (const auto& [neighbor, edge_info] : node_ptr->incomingEdges()) {
            edge_counter++;
            
            if (edge_info.distance != edge_info.distance_original) {
                std::cerr << "ERROR: Incoming edge mismatch at node " << node_ptr->getIndex()
                          << " <- " << neighbor->getIndex() 
                          << " | dist=" << edge_info.distance 
                          << ", dist_original=" << edge_info.distance_original << "\n";
                has_inconsistency = true;
            }

            if (edge_info.distance_original == 0.0) {
                std::cerr << "WARNING: Zero distance_original at node " << node_ptr->getIndex()
                          << " <- " << neighbor->getIndex() 
                          << " | State1: " << neighbor->getStateValue().transpose()
                          << " | State2: " << node_ptr->getStateValue().transpose() << "\n";
            }
        }
    }

    std::cout << "Edge verification complete. Checked " << edge_counter << " edges.\n";
    if (!has_inconsistency) {
        std::cout << "All edges have consistent distance/distance_original.\n";
    }







}


bool KinodynamicRRTX::extend(Eigen::VectorXd v) {
    auto new_node = std::make_shared<RRTxNode>(statespace_->addState(v), sample_counter);
    auto neighbors = kdtree_->radiusSearch(new_node->getStateValue().head(2), neighborhood_radius_ + 0.01);
    
    auto trajs_from_v_to_u = findParent(new_node, neighbors);

    if (!new_node->getParent()) {
        sample_counter--;
        return false;
    }

    tree_.push_back(new_node);
    kdtree_->addPoint(new_node->getStateValue().head(2));
    kdtree_->buildTree(); 

    // Algorithm 2 lines 7-13 implementation
    const double t_now = clock_->now().seconds();
    const double t_arrival_predicted = t_now + robot_current_time_to_goal_;
    for (auto const& [neighbor, v_to_u_traj] : trajs_from_v_to_u) {
        // Persistent outgoing from new node (N⁰+)
        // Establish the Edge (new_node -> neighbor) using the pre-computed trajectory.
        if (v_to_u_traj.is_valid) {
            new_node->addNeighbor(neighbor, true, false, v_to_u_traj);
        }
        
        double time_to_goal_at_u = neighbor->getTimeToGoal();
        const double global_edge_start_time_at_u = t_arrival_predicted - time_to_goal_at_u;
        // Temporary outgoing from neighbors (Nr+)
        // Establish the Edge (neighbor -> new_node) by computing it now.
        /*
            One thing that bothers me here is collision checking with the current obstalces would me some nodes to not find their neighbors 
            and later when the obstalce moves then we do not update this.
            I think RRTx relies on continuous sampling but for now i use a pre-plan obstalce free phase in my test to gather all the neighbors correctly
        */
        Trajectory u_to_v_traj = statespace_->steer(neighbor->getStateValue(), new_node->getStateValue());
        if (u_to_v_traj.is_valid && obs_checker_->isTrajectorySafe(u_to_v_traj, global_edge_start_time_at_u )) {
            neighbor->addNeighbor(new_node.get(), false, true, u_to_v_traj);
        }
    }
    
    return true;
}







std::unordered_map<RRTxNode*, Trajectory> KinodynamicRRTX::findParent(std::shared_ptr<RRTxNode> v, const std::vector<size_t>& candidates) {
    double min_lmc = INFINITY; // v is a new node so it has not LMC yet (you can also use v->getLMC() instead of INFINITY)
    RRTxNode* best_parent = nullptr;
    double best_dist = 0.0;
    Trajectory best_traj;
    std::unordered_map<RRTxNode*, Trajectory> all_trajectories_from_v_to_u;
    const double t_now = clock_->now().seconds();
    const double t_arrival_predicted = t_now + robot_current_time_to_goal_;


    // candidates are "u"
    for (size_t idx : candidates) {
        auto& candidate = tree_[idx];
        if (candidate == v) continue;

        Trajectory traj_from_v_to_u = statespace_->steer(v->getStateValue(), candidate->getStateValue());

        all_trajectories_from_v_to_u.insert({candidate.get(), traj_from_v_to_u});

        if (!traj_from_v_to_u.is_valid) {
            continue;
        }
        /*
          The obstalce check we do here right now is the v->u  (new node to neighbors) and can also be used for the v->u trajcetories in extend function
          obstalce check (maybe later use a map or something) but u->v should be done in extend.
        */
        double time_to_goal_at_v = candidate->getTimeToGoal() + traj_from_v_to_u.time_duration;
        const double global_edge_start_time_at_v = t_arrival_predicted - time_to_goal_at_v;
        if (traj_from_v_to_u.cost <= neighborhood_radius_+0.01 && obs_checker_->isTrajectorySafe(traj_from_v_to_u, global_edge_start_time_at_v)) {
            const double candidate_lmc = candidate->getLMC() + traj_from_v_to_u.cost;

            if (candidate_lmc < min_lmc && candidate_lmc < v->getLMC()) {
                min_lmc = candidate_lmc;
                best_parent = candidate.get();
                best_traj = traj_from_v_to_u; 
                best_dist = traj_from_v_to_u.geometric_distance;
            }
        }
    }

    if (best_parent) {
        v->setParent(best_parent, best_traj);
        v->setTimeToGoal(best_parent->getTimeToGoal() + best_traj.time_duration);
        v->setLMC(min_lmc);
        edge_length_[v->getIndex()] = best_dist;
    }

    return all_trajectories_from_v_to_u;
}






void KinodynamicRRTX::rewireNeighbors(RRTxNode* v) {
    const double inconsistency = v->getCost() - v->getLMC();
    if (inconsistency <= epsilon_) return;

    cullNeighbors(v);

    for (auto& [u, edge] : v->incomingEdges()) {
        // if (u == v->getParent() || !isValidEdge(u, v, edge)) continue;

        // if (v->getIndex() == 538 && u->getIndex() == 260 || 
        //     u->getIndex() == 260 && v->getIndex() == 538  
        //     )
        //     std::cout<<edge.distance <<"\n";
        if (u == v->getParent() ) continue;


        const double candidate_lmc = v->getLMC() + edge.distance;
        if (u->getLMC() > candidate_lmc) {
            // The trajectory for the u -> v edge is what's stored in 'edge_from_u_on_v'
            // const Trajectory& trajectory_to_parent = v->outgoing_edges_.at(u).cached_trajectory;

            u->setLMC(candidate_lmc);
            // makeParentOf(u, v, edge.distance);
            u->setParent(v,edge.cached_trajectory);
            edge_length_[u->getIndex()] = edge.distance;
            if (u->getCost() - candidate_lmc > epsilon_) {
                verifyQueue(u);
            }
        }
    }
}

void KinodynamicRRTX::reduceInconsistency() {
    while (!inconsistency_queue_.empty() 
            && (!partial_update ||
               (inconsistency_queue_.top().first < vbot_node_->getCost() ||  // .first instead of .min_key
                vbot_node_->getLMC() != vbot_node_->getCost() ||
                vbot_node_->getCost() == INFINITY ||
                vbot_node_->in_queue_ == true))
        ) 
    {
        auto top_element = inconsistency_queue_.top();
        inconsistency_queue_.pop();
        
        RRTxNode* node = top_element.second;  // .second instead of .node
        // node->in_queue_ = false;

        int node_idx = node->getIndex();
        if (node_idx == -1 || Vc_T_.count(node_idx)) continue;

        if (node->getCost() - node->getLMC() > epsilon_) {
            updateLMC(node);
            rewireNeighbors(node);
        }

        node->setCost(node->getLMC());
    }
}



double KinodynamicRRTX::shrinkingBallRadius() const {
    auto rad = factor * gamma_ * pow(log(tree_.size()) / tree_.size(), 1.0/dimension_);
    return std::min(rad, delta);
    // return 15.0;

}

std::unordered_set<int> KinodynamicRRTX::findSamplesNearObstacles(
    const ObstacleVector& obstacles,
    double max_length
) {
    std::unordered_set<int> conflicting_samples;
    const double PREDICTION_HORIZON_SECONDS = 3.0; // How far into the future to predict
    
    // Controls how many steps we check between the start and end of the horizon.
    // 0 = just start and end points. 1 = start, middle, and end points.
    const int num_intermediate_steps = 3; 

    for (const auto& obstacle : obstacles) {
        // --- The existing logic for calculating search radius is good. ---
        double obstacle_radius;
        if (obstacle.type == Obstacle::CIRCLE) {
            obstacle_radius = obstacle.dimensions.radius + obstacle.inflation;
        } else { // BOX
            double half_diagonal = std::sqrt(
                std::pow(obstacle.dimensions.width/2, 2) +
                std::pow(obstacle.dimensions.height/2, 2)
            );
            obstacle_radius = half_diagonal + obstacle.inflation;
        }
        // This search radius is a heuristic to find potentially colliding EDGES.
        // It should be the radius of our "swept volume" check.
        double edge_heuristic_radius = std::sqrt(
            std::pow(obstacle_radius, 2) +
            std::pow(max_length / 2.0, 2)
        );
        // double edge_heuristic_radius = obstacle_radius + max_length;

        // --- LOGIC FOR DYNAMIC OBSTACLES ---
        if (obstacle.is_dynamic && obstacle.velocity.norm() > 1e-6) {
            
            // Perform searches at multiple points along the predicted path
            for (int i = 0; i <= num_intermediate_steps + 1; ++i) {
                // Calculate the time for the current step
                double t = (static_cast<double>(i) / (num_intermediate_steps + 1)) * PREDICTION_HORIZON_SECONDS;
                
                // Predict the obstacle's position at time t
                Eigen::VectorXd predicted_pos = obstacle.position + obstacle.velocity * t;

                // Perform the radius search at this intermediate point
                auto indices = kdtree_->radiusSearch(predicted_pos, edge_heuristic_radius);
                conflicting_samples.insert(indices.begin(), indices.end());
            }

        } else {
            // --- STATIC OBSTACLE LOGIC (Unchanged) ---
            Eigen::VectorXd obs_state(2);
            obs_state << obstacle.position(0), obstacle.position(1);
            auto sample_indices = kdtree_->radiusSearch(obs_state, edge_heuristic_radius);
            conflicting_samples.insert(sample_indices.begin(), sample_indices.end());
        }
    }
    return conflicting_samples;
}


void KinodynamicRRTX::updateLMC(RRTxNode* v) {
    cullNeighbors(v);
    double min_lmc = v->getLMC();
    RRTxNode* best_parent = nullptr;
    double best_edge_distance = INFINITY;  // Track the distance of the best edge
    Trajectory best_traj = v->getParentTrajectory();

    

    // Iterate over outgoing edges (v → u)
    for (const auto& [u, edge] : v->outgoingEdges()) {
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
            best_traj = edge.cached_trajectory;
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


        v->setParent(best_parent, best_traj);  // Use the captured distance
        v->setLMC(min_lmc);
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

void KinodynamicRRTX::cullNeighbors(RRTxNode* v) {
    if (cap_samples_ == true && sample_counter >= num_of_samples_-1)
    // if (cap_samples_ == true && update_obstacle == true)
        return; // to not waste time when we put a cap on the number of samples!


    auto& outgoing = v->outgoingEdges();
    auto it = outgoing.begin();
    while (it != outgoing.end()) {
        auto [neighbor, edge] = *it;
        if (!edge.is_initial && 
            edge.cached_trajectory.cost > neighborhood_radius_+0.01 &&// (v->getStateValue() - neighbor->getStateValue()).norm() > neighborhood_radius_ &&
            neighbor != v->getParent() ) 
        {
            auto& incoming = neighbor->incomingEdges();
            if (auto incoming_it = incoming.find(v); incoming_it != incoming.end()) {
                // Only remove temporary incoming edges (Nr⁻)
                if (!incoming_it->second.is_initial) {
                    incoming.erase(incoming_it);
                }
            }
            it = outgoing.erase(it);
        } else {
            ++it;
        }
    }
}

// void KinodynamicRRTX::cullNeighbors(RRTxNode* v) {
//     if (cap_samples_ == true && sample_counter >= num_of_samples_ - 1) {
//         // This early exit for a fixed number of samples is a reasonable optimization.
//         return;
//     }

//     auto& outgoing = v->outgoingEdges();
//     auto it = outgoing.begin();
//     while (it != outgoing.end()) {
//         auto [neighbor, edge] = *it;

//         // The edge must be temporary ("running") and not part of the shortest-path tree.
//         // This logic is correct according to the RRTx algorithm. [cite: 72, 372]
//         if (!edge.is_initial && neighbor != v->getParent()) {
            
//             // Before accessing the cached trajectory, ensure it has been computed.
//             if (edge.is_trajectory_computed) {

//                 // === THE FIX ===
//                 // Compare the GEOMETRIC distance to the GEOMETRIC radius.
//                 if (edge.cached_trajectory.geometric_distance > neighborhood_radius_ + 0.01) {
                    
//                     // The rest of your removal logic is correct.
//                     auto& incoming = neighbor->incomingEdges();
//                     if (auto incoming_it = incoming.find(v); incoming_it != incoming.end()) {
//                         // Only remove the corresponding temporary incoming edge on the neighbor.
//                         if (!incoming_it->second.is_initial) {
//                             incoming.erase(incoming_it);
//                         }
//                     }
//                     it = outgoing.erase(it); // Erase the edge from this node's list.
//                 } else {
//                     ++it;
//                 }
//             } else {
//                  ++it;
//             }
//         } else {
//             ++it;
//         }
//     }
// }



void KinodynamicRRTX::verifyQueue(RRTxNode* node) {
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







void KinodynamicRRTX::verifyOrphan(RRTxNode* node) {
    if(node->in_queue_==true){
        inconsistency_queue_.remove(node);
        // node->in_queue_=false;
    }

    // int idx = findNodeIndex(node);
    int idx = node->getIndex();
    if (idx != -1) {
        Vc_T_.insert(idx);
    }
}

void KinodynamicRRTX::propagateDescendants() {
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

    // Step 2: Invalidate costs for neighbors of affected nodes
    for (int idx : Vc_T_) {
        if (idx < 0 || idx >= tree_.size()) continue;
        auto node = tree_[idx].get();

        // Process outgoing neighbors (N⁺(v) \ Vc_T)
        for (const auto& [neighbor, edge] : node->outgoingEdges()) {
            // Skip invalid edges
            // if (!isValidEdge(node, neighbor, edge)) continue; 

            int neighbor_idx = neighbor->getIndex();
            if (neighbor_idx == -1 || Vc_T_.count(neighbor_idx)) continue;

            edge_length_[neighbor_idx] = -INFINITY;
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
                    edge_length_[parent_idx] = -INFINITY;
                    parent->setCost(INFINITY);
                    verifyQueue(parent);
                }
            }
        }
    }

    // Step 3: Reset orphaned nodes using new parent API
    for (int idx : Vc_T_) {
        if (idx < 0 || idx >= tree_.size()) continue;
        auto node = tree_[idx].get();

        node->setCost(INFINITY);
        node->setLMC(INFINITY);
        node->setTimeToGoal(std::numeric_limits<double>::infinity());
        node->setParent(nullptr, Trajectory{});
    }

    Vc_T_.clear();
}





bool KinodynamicRRTX::isValidEdge(RRTxNode* from, RRTxNode* to, const EdgeInfo& edge) const {
    return edge.distance != INFINITY && 
           obs_checker_->isObstacleFree(from->getStateValue(), to->getStateValue());
}

// void KinodynamicRRTX::visualizeTree() {
//     std::vector<Eigen::VectorXd> nodes;
//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
//     const double goal_cost = vbot_node_ ? vbot_node_->getCost() : INFINITY;
    
//     // Collect valid nodes and their connections
//     std::unordered_set<RRTxNode*> valid_nodes;
//     for (const auto& node : tree_) {
//         if (node->getCost() <= goal_cost) {
//             nodes.push_back(node->getStateValue());
//             valid_nodes.insert(node.get());
//         }
//     }

//     // Generate edges for valid nodes
//     for (const auto& node : valid_nodes) {
//         if (node->getParent()) {
//             edges.emplace_back(node->getParent()->getStateValue(),
//                              node->getStateValue());
//         }
//     }

//     visualization_->visualizeEdges(edges);
// }

void KinodynamicRRTX::visualizeTree() {
    // Using a more appropriate name since the edges are straight lines.
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    // std::cout<<vbot_node_->getCost()<<"\n";
    // OPTIMIZATION: Reserve memory for one edge per node (except the root).
    // This is more accurate and prevents reallocations.
    if (!tree_.empty()) {
        edges.reserve(tree_.size());
    }
    std::vector<Eigen::VectorXd> tree_nodes;

    // Iterate through all nodes in the tree.
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
    
    std::cout<<"Tree Size: "<< tree_nodes.size()<<"\n";

    // visualization_->visualizeNodes(tree_nodes, "map", 
    //                         std::vector<float>{0.0f, 1.0f, 0.0f},  // Red for tree
    //                         "tree_nodes");
    
    // Visualize the collected straight-line edges.
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



void KinodynamicRRTX::visualizeSmoothedPath(const std::vector<Eigen::VectorXd>& shortest_path_) {
    // Extract nodes and edges from the smoothed path
    std::vector<Eigen::VectorXd> nodes;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;

    // Add nodes to the list
    for (const auto& point : shortest_path_) {
        nodes.push_back(point); // Each point in the smoothed path is a node
    }

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


std::vector<Eigen::VectorXd> KinodynamicRRTX::getSmoothedPathPositions(int num_intermediates, int smoothing_passes) const {
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


std::vector<Eigen::VectorXd> KinodynamicRRTX::interpolatePath(const std::vector<Eigen::VectorXd>& path, int num_intermediates) const {
    std::vector<Eigen::VectorXd> new_path;

    // Check for invalid inputs
    if (path.empty()) {
        return new_path; // Return empty path if input is empty
    }
    if (num_intermediates < 1) {
        return path; // Return original path if no interpolation is needed
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


std::vector<Eigen::VectorXd> KinodynamicRRTX::smoothPath(const std::vector<Eigen::VectorXd>& path, int window_size) const {
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


void KinodynamicRRTX::updateObstacleSamples(const ObstacleVector& obstacles) {
    update_obstacle = true;

    // // Common initialization
    // if (edge_length_[max_length_edge_ind] != max_length) {
    //     auto max_it = std::max_element(edge_length_.begin(), edge_length_.end(),
    //         [](const auto& a, const auto& b) { return a.second < b.second; });
    //     max_length = max_it->second;
    //     max_length_edge_ind = max_it->first;
    // }
    max_length = delta;
    std::cout<<"Max Length: "<<max_length <<"\n";
    auto current = findSamplesNearObstacles(obstacles, max_length);
    // if (current == samples_in_obstacles_ && current.size()!=tree_.size()) return false; // Early exit if nothing has changed


    // Common code for finding added/removed samples
    std::vector<int> added, removed;
    for (int sample : current) {
        if (!samples_in_obstacles_.count(sample)) added.push_back(sample);
    }
    for (int sample : samples_in_obstacles_) {
        if (!current.count(sample)) removed.push_back(sample);
    }

    std::vector<int> cur, prev;
    for(int c: current)
        cur.push_back(c);

    for(int p: samples_in_obstacles_)
        prev.push_back(p);

    // // // Visualization
    // std::vector<Eigen::VectorXd> positions4;
    // std::string color_str = "0.0,0.0,1.0"; // Blue color
    // for (int r : removed){
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(r)->getStateValue();
    //     positions4.push_back(vec);
    // }
    // visualization_->visualizeNodes(positions4,"map",color_str);

/////////////



    // std::vector<std::shared_ptr<RRTxNode>> nodes_to_visualize;
    // nodes_to_visualize.reserve(current.size());
    // for (int sample_index : current) {
    //     if (sample_index >= 0 && sample_index < tree_.size()) {
    //         nodes_to_visualize.push_back(tree_[sample_index]);
    //     }
    // }

    // // 2. THE FIX: Create a new vector to hold just the positions
    // std::vector<Eigen::VectorXd> positions_to_visualize;
    // positions_to_visualize.reserve(nodes_to_visualize.size());

    // // 3. Loop through the nodes and extract their state/position
    // for (const auto& node : nodes_to_visualize) {
    //     positions_to_visualize.push_back(node->getStateValue());
    // }

    // // 4. Call visualizeNodes with the correct types and number of arguments
    // if (visualization_) {
    //     // This calls the version that accepts a position vector, frame_id, color, and namespace.
    //     // Example color: Red (r=1.0, g=0.0, b=0.0, a=1.0)
    //     std::vector<float> color = {1.0f, 0.0f, 0.0f, 1.0f};
    //     visualization_->visualizeNodes(positions_to_visualize, "map", color, "current_obstacle_neighbors");
    // }



////////////
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



    if (ignore_sample) {
        // Version 1: Track samples on obstacles without explicit checks

        // I update here because in removeObstalce i need to avoid samples that are still on obstalces
        samples_in_obstacles_ = current;
        
        if (!added.empty()|| force_repair) {
            addNewObstacle(added);
            propagateDescendants();
            verifyQueue(tree_[vbot_index_].get());
        }
        if (!removed.empty() || force_repair) {
            removeObstacle(removed);
        }
    } else {
        // Version 2: Use explicit obstacle checks
        samples_in_obstacles_ = current; // doesn't matter to update here or after the functions because we alread filled prev with samples_in_obstacles
        /*
            we use removed and added for the condtions but we use prev and cur as the input
            prev is the whole samples previously on the obstalces --> we use prev because we need prev in remove obstacle
            cur is all the current samples on the obstalces
            as opposed to added removed which we use in the version 1 ---> the version one is much faster and we can use it because we ignore sample in obstalces in remove obstalce function
            so in version one we only send what is added wrt to the previous one! --> the obstalce mayb only moved a little bit and only a fraction of samples has been added or removed wrt to previous iteration
            this simplificatin only works if you ignore samples in obstalce in remove obstalce function of version 1 or else we might connect some nodes and think we are okay but the next cycle added/removed wouldn't 
            cover the obstacly part.
        */

        if (!removed.empty() || force_repair) 
            removeObstacle(prev);
        if (!added.empty() || force_repair) {
            auto start = std::chrono::high_resolution_clock::now();
            addNewObstacle(cur);
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            std::cout << "Time taken for the addNew and propagate : " << duration.count() << " milliseconds\n";
            propagateDescendants();
            verifyQueue(tree_[vbot_index_].get());
        }
    }
    reduceInconsistency();
}

// void KinodynamicRRTX::addNewObstacle(const std::vector<int>& added_indices) {
//     for (int idx : added_indices) {
//         RRTxNode* node = tree_[idx].get();
        
//         if (ignore_sample) {
//             samples_in_obstacles_.insert(idx);
//         }

//         for (auto& [u, edge] : node->outgoingEdges()) {
//             // Common edge invalidation logic
//             const bool should_invalidate = ignore_sample ? true : 
//                 (edge.distance != INFINITY && 
//                 !obs_checker_->isObstacleFree(node->getStateValue(), u->getStateValue()));

//             if (!should_invalidate) continue;
//             /*
//                 so since we are iterating over outgoing (you can iterate over ingoing also doesnt matter!), and our focus is on "node" thne if node has collision with its neighbor u then
//                 outgoing node's dist from node->u should be inf, also incoming node from u->node should be inf also incoming node from node->u should be inf!
//                 so its like the outgoing of node i.e, node->u (with the focus on node!) and incoming of node (u->node) should be inf and also the incoming of u from node i.e, node->u needs to be inf, but how about outgoing of u i.e., u->node  --> this should be ALSO handled!--> don't confuse the asymetry
//             */
//             // Common invalidation operations
//             edge.distance = INFINITY;
//             u->incomingEdges().at(node).distance = INFINITY;
//             u->outgoingEdges().at(node).distance = INFINITY;
//             node->incomingEdges().at(u).distance = INFINITY;

//             // Common parent relationship handling
//             if (u->getParent() == node) {
//                 u->setParent(nullptr, INFINITY);
//                 verifyOrphan(u);
//             }
//             if (node->getParent() == u) {
//                 node->setParent(nullptr, INFINITY);
//                 verifyOrphan(node);
//             }
//         }

//         // // Additional operations for ignore_sample mode
//         // if (ignore_sample) {
//         //     node->setCost(INFINITY);
//         //     node->setLMC(INFINITY);
//         //     edge_length_[idx] = -INFINITY;
//         //     node->setParent(nullptr, 0.0);
//         //     verifyOrphan(node);
//         // }
//     }
// }
// void KinodynamicRRTX::removeObstacle(const std::vector<int>& removed_indices) {
//     for (int idx : removed_indices) {
//         RRTxNode* node = tree_[idx].get();

//         if (ignore_sample) {
//             samples_in_obstacles_.erase(idx); // Update obstacle set
//         }

//         for (auto& [u, edge] : node->outgoingEdges()) {
//             // Mode-specific condition components
//             const bool is_neighbor_clear = !samples_in_obstacles_.count(u->getIndex());
//             const bool was_invalidated = (edge.distance == INFINITY);
//             const bool is_now_clear = obs_checker_->isObstacleFree(node->getStateValue(), 
//                                                                   u->getStateValue());
            
//             // Unified restoration condition
//             const bool should_restore = ignore_sample 
//                 ? is_neighbor_clear          // Sample tracking mode
//                 : was_invalidated && is_now_clear;  // Explicit check mode

//             if (!should_restore) continue;

//             // Common restoration logic
//             edge.distance = edge.distance_original;
//             u->incomingEdges().at(node).distance = edge.distance_original;
//             u->outgoingEdges().at(node).distance = edge.distance_original;
//             node->incomingEdges().at(u).distance = edge.distance_original;
//         }

//         // Common node updates
//         updateLMC(node);
//         if (node->getCost() != node->getLMC()) {
//             verifyQueue(node);
//         }
//     }
// }
// //////////////////////////////////////////////////////////////////////////////

/*
    mind that i compared FMTx and RRTx in env with only dynamic obstalce so persisting static obstalce is not implemented here because right now
    isObstalceFree considers all obstalces. it can be implemented later so that in addNewObstalce we dont check the old obstalces! but right now assume
    everything moves, hence its new!
    Although in FMTx i implmeneted something in findsamplesnearobstacle for static obs to be considered only once and since fmtx in its default form has delayed obstacle check
    then its fine. even though one might argue in the prune true case (rrtx style proactive obstalce check) in fmtx we need some reconsideration if static obs are present
*/


void KinodynamicRRTX::addNewObstacle(const std::vector<int>& added_indices) {
    // We need a stable time anchor for all checks in this cycle.
    const double t_now = clock_->now().seconds();
    const double t_arrival_predicted = t_now + robot_current_time_to_goal_;
    std::cout<<"added indices : "<<added_indices.size()<<"\n";
    for (int idx : added_indices) {
        RRTxNode* node = tree_[idx].get();
        bool node_itself_is_unusable = false;

        if (ignore_sample) {
            // In ignore_sample mode, if idx is in added_indices, we treat the node as unusable
            // and mark it for special handling (e.g., its edges will be invalidated based on this mark).
            samples_in_obstacles_.insert(idx);
            node_itself_is_unusable = true;
        } else {
            // ignore_sample is false: explicitly check if the node's location is now in an obstacle.
            if (!obs_checker_->isObstacleFree(node->getStateValue())) { // Check the node's point itself
                node_itself_is_unusable = true;
            }
        }

        if (node_itself_is_unusable) {
            // Node is inside an obstacle (or treated as such via ignore_sample).
            // All its existing edges become invalid WITHOUT individual collision checks for these edges.
            for (auto& [u, edge] : node->outgoingEdges()) {
                // Invalidate this edge (node -> u)
                edge.distance = INFINITY;
                // And its symmetric counterparts if your graph stores them this way
                if (u->incomingEdges().count(node)) {
                    u->incomingEdges().at(node).distance = INFINITY;
                }
                if (u->outgoingEdges().count(node)) { // For u -> node
                    u->outgoingEdges().at(node).distance = INFINITY;
                }
                if (node->incomingEdges().count(u)) { // For u -> node (from node's perspective)
                    node->incomingEdges().at(u).distance = INFINITY;
                }

                // If this invalidated edge was a parent link, handle orphaning
                if (u->getParent() == node) {
                    u->setParent(nullptr, INFINITY);
                    verifyOrphan(u);
                }
                // Note: if node->getParent() == u, this will be handled below
                // when 'node' itself is orphaned.
            }
            // Also invalidate any incoming edges to 'node' not caught by iterating its outgoingEdges' symmetry
            // (This might be redundant if outgoingEdges and their symmetric pairs cover all connections)
            // For example, if edges are strictly directed and only stored one way initially.
            // However, the current symmetric updates in the loop above likely cover this.

            // Directly mark 'node' as unusable and orphan it.
            node->setCost(INFINITY); // Assuming setCost updates the main cost (g-value)
            node->setLMC(INFINITY);  // Assuming setLMC updates the lookahead-cost (rhs-value)
            
            RRTxNode* old_parent = node->getParent();
            if (old_parent) {
                // If 'node' had a parent, its link to that parent is now broken.
                // The call to node->setParent below handles updating 'node'.
                // We might need to ensure 'old_parent' updates its children list if applicable,
                // though 'verifyOrphan(node)' and subsequent processing should handle graph consistency.
            }
            node->setParent(nullptr, INFINITY); // Sever parent link
            verifyOrphan(node); // Ensure 'node' itself is processed by the orphan logic

        } else {
            // Node itself is in free space, and ignore_sample is false.
            // Check its outgoing edges individually.
            for (auto& [u, edge] : node->outgoingEdges()) {
                // We check the cached trajectory for the edge node -> u
                const Trajectory& traj_node_to_u = edge.cached_trajectory;
                
                // Calculate the global time this edge starts.
                const double global_edge_start_time = t_arrival_predicted - node->getTimeToGoal();

                if (edge.distance != INFINITY &&
                    !obs_checker_->isTrajectorySafe(traj_node_to_u, global_edge_start_time)) {
                    // This specific edge (node -> u) is now blocked.
                    edge.distance = INFINITY;
                    if (u->incomingEdges().count(node)) {
                        u->incomingEdges().at(node).distance = INFINITY;
                    }
                    if (u->outgoingEdges().count(node)) {
                        u->outgoingEdges().at(node).distance = INFINITY;
                    }
                    if (node->incomingEdges().count(u)) {
                        node->incomingEdges().at(u).distance = INFINITY;
                    }

                    // Handle parent relationships
                    if (u->getParent() == node) {
                        u->setParent(nullptr, INFINITY);
                        verifyOrphan(u);
                    }
                    if (node->getParent() == u) {
                        node->setParent(nullptr, INFINITY);
                        verifyOrphan(node);
                    }
                }
            }
        }
    }
}


void KinodynamicRRTX::removeObstacle(const std::vector<int>& removed_indices) {
    const double t_now = clock_->now().seconds();
    const double t_arrival_predicted = t_now + robot_current_time_to_goal_;

    for (int idx : removed_indices) {
        RRTxNode* node = tree_[idx].get();
        bool node_was_in_ignored_obstacle_state = false;

        if (ignore_sample) {
            if (samples_in_obstacles_.count(idx)) {
                samples_in_obstacles_.erase(idx); // Node is no longer "ignored as obstacle"
                node_was_in_ignored_obstacle_state = true;
            }
        }

        // Determine if the node's location itself is now clear to decide if its edges *can* be restored.
        bool node_location_is_currently_free = obs_checker_->isObstacleFree(node->getStateValue());

        if ((ignore_sample && node_was_in_ignored_obstacle_state) || // Was ignored, now un-ignored
            (!ignore_sample && node_location_is_currently_free)) {   // Wasn't ignored, and its location is free
            
            // Only attempt to restore edges if the node itself is considered potentially usable.
            // If !node_location_is_currently_free (and not ignore_sample), it means 'node' is *still*
            // in some *other* obstacle, so its edges should remain unusable/INFINITE.

            if (node_location_is_currently_free) { // Essential check for non-ignore_sample, good for ignore_sample too
                for (auto& [u, edge] : node->outgoingEdges()) {
                    bool should_attempt_restore = false;
                    if (ignore_sample) {
                        // If node was un-ignored, and neighbor u is also not in an ignored state
                        if (node_was_in_ignored_obstacle_state && !samples_in_obstacles_.count(u->getIndex())) {
                            should_attempt_restore = true;
                        }
                    } else { // Not ignore_sample
                        if (edge.distance == INFINITY) { // Only consider edges that were previously blocked
                            should_attempt_restore = true;
                        }
                    }

                    if (should_attempt_restore) {
                        // Crucially, check if the edge (node->u) is *actually* free now from any obstacle
                        // Re-check the original trajectory for safety.
                        const Trajectory& original_traj = edge.cached_trajectory;
                        const double global_edge_start_time = t_arrival_predicted - node->getTimeToGoal();

                        if (obs_checker_->isTrajectorySafe(original_traj, global_edge_start_time)) {
                            edge.distance = edge.distance_original;
                            if (u->incomingEdges().count(node)) u->incomingEdges().at(node).distance = edge.distance_original;
                            if (u->outgoingEdges().count(node)) u->outgoingEdges().at(node).distance = edge.distance_original;
                            if (node->incomingEdges().count(u)) node->incomingEdges().at(u).distance = edge.distance_original;
                        }
                        // If it's not free (e.g., blocked by another, different obstacle), its distance remains INFINITY or its current value.
                    }
                }
            }
            
            // After attempting to restore edges, update LMC and queue if inconsistent.
            // This should happen if the node itself became free and thus usable.
            updateLMC(node);
            if (node->getCost() != node->getLMC()) {
                verifyQueue(node);
            }
        }
        // If the node's location is *still* not free (and not ignore_sample),
        // its LMC should remain Inf or be updated accordingly by updateLMC if all edges are Inf.
        // It might still need verifyQueue if its previous state was different.
        // However, the existing updateLMC/verifyQueue at the end of the outer loop in the original
        // code seems to cover general cases. The refined logic above tries to be more precise
        // about *when* to update LMC based on the node's own state.
        // For simplicity and safety, the original broader updateLMC/verifyQueue might be fine:
        // updateLMC(node);
        // if (node->getCost() != node->getLMC()) {
        //     verifyQueue(node);
        // }
        // This will work because if node_location_is_currently_free is false, updateLMC should ideally
        // result in node->LMC being INFINITY if all its edges are INFINITY.
    }
}
/////////////////////////////////////////////////////////////////////////////////////////
void KinodynamicRRTX::dumpTreeToCSV(const std::string& filename) const {
    std::ofstream fout(filename);
    if (!fout.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing\n";
        return;
    }
    // 1. Determine the dimension of the states from the first node
    if (tree_.empty()) {
        std::cerr << "Tree is empty. Nothing to dump.\n";
        return;
    }
    size_t dim = tree_[0]->getStateValue().size();

    // 2. Write the CSV header
    fout << "node_id";
    for (size_t d = 0; d < dim; ++d) {
        fout << ",x" << d;
    }
    fout << ",parent_id\n";

    // 3. Iterate through each node and write its data
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
    // 1. Store the robot's continuous state
    robot_continuous_state_ = robot_state;

    // --- STABILIZATION FIX START ---

    // Define a hysteresis factor. A new path must be at least 5% cheaper to be adopted.
    // This prevents switching for negligible gains.
    const double hysteresis_factor = 0.80;
    double cost_of_current_path = std::numeric_limits<double>::infinity();

    // First, calculate the cost of sticking with the current anchor node, if it's valid.
    // This gives us a baseline to beat.
    if (vbot_node_ && vbot_node_->getCost() != INFINITY) {
        Trajectory bridge_to_current_anchor = statespace_->steer(robot_continuous_state_, vbot_node_->getStateValue());
        if (bridge_to_current_anchor.is_valid && obs_checker_->isTrajectorySafe(bridge_to_current_anchor, clock_->now().seconds())) {
            cost_of_current_path = bridge_to_current_anchor.cost + vbot_node_->getCost();
        }
    }
    // --- STABILIZATION FIX END ---

    // 2. Search for the best *potential* anchor node in the neighborhood.
    // This part of your logic remains unchanged.
    RRTxNode* best_candidate_node = nullptr;
    Trajectory best_candidate_bridge;
    double best_candidate_cost = std::numeric_limits<double>::infinity();
    double current_search_radius = neighborhood_radius_;
    const int max_attempts = 4;
    const double radius_multiplier = 1.2;

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        auto nearby_indices = kdtree_->radiusSearch(robot_continuous_state_.head<2>(), current_search_radius);
        
        // Note: The min_total_cost is reset each attempt to find the best in the new, larger radius.
        double min_cost_in_radius = std::numeric_limits<double>::infinity();

        for (auto idx : nearby_indices) {
            RRTxNode* candidate = tree_[idx].get();
            if (candidate->getCost() == INFINITY) continue;

            Trajectory bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());
            if (!bridge.is_valid || !obs_checker_->isTrajectorySafe(bridge, clock_->now().seconds())) continue;

            double cost = bridge.cost + candidate->getCost();
            if (cost < min_cost_in_radius) {
                min_cost_in_radius = cost;
                best_candidate_node = candidate;
                best_candidate_bridge = bridge;
                best_candidate_cost = cost;
            }
        }

        if (best_candidate_node) break; // Exit if a connection was found
        current_search_radius *= radius_multiplier;
    }

    // --- STABILIZATION FIX START ---

    // 3. Make a stable decision.
    // Only switch to the new candidate if it's significantly better than our current path.
    if (best_candidate_node && best_candidate_cost < cost_of_current_path * hysteresis_factor) {
        // The new node is significantly better. It's worth switching.
        vbot_node_ = best_candidate_node;
        robot_current_time_to_goal_ = best_candidate_bridge.time_duration + best_candidate_node->getTimeToGoal();
    } else if (vbot_node_) {
        // The new candidate is not significantly better, or none was found.
        // Stick with the old anchor node to maintain stability.
        // We still need to recalculate the time-to-go in case the tree costs updated.
        Trajectory bridge_to_kept_anchor = statespace_->steer(robot_continuous_state_, vbot_node_->getStateValue());
        if (bridge_to_kept_anchor.is_valid) {
             robot_current_time_to_goal_ = bridge_to_kept_anchor.time_duration + vbot_node_->getTimeToGoal();
        }
    } else {
        // This case handles when there was no previous anchor OR no valid new anchor.
        // If we found a candidate but didn't switch, we still need to set it for the first time.
        vbot_node_ = best_candidate_node; // This will be nullptr if none found.
        if (vbot_node_) {
             robot_current_time_to_goal_ = best_candidate_bridge.time_duration + best_candidate_node->getTimeToGoal();
        } else {
             robot_current_time_to_goal_ = std::numeric_limits<double>::infinity();
        }
    }
    // --- STABILIZATION FIX END ---
}




bool KinodynamicRRTX::isPathStillValid(const std::vector<Eigen::VectorXd>& path, const Eigen::VectorXd& current_robot_state) const {
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




bool KinodynamicRRTX::arePathsSimilar(const std::vector<Eigen::VectorXd>& path_a, const std::vector<Eigen::VectorXd>& path_b, double tolerance) const {
    // If paths have different numbers of waypoints, they are not similar.
    if (path_a.size() != path_b.size()) {
        return false;
    }

    // Check each waypoint pair for proximity.
    for (size_t i = 0; i < path_a.size(); ++i) {
        // If the distance between corresponding points is greater than the tolerance,
        // the paths are different.
        if ((path_a[i] - path_b[i]).norm() > tolerance) {
            return false;
        }
    }

    // If all waypoints are within the tolerance, the paths are considered similar.
    return true;
}
