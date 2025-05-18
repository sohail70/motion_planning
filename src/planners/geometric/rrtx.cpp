#include "motion_planning/planners/geometric/rrtx.hpp"

RRTX::RRTX(std::shared_ptr<StateSpace> statespace, 
    std::shared_ptr<ProblemDefinition> problem_def,
    std::shared_ptr<ObstacleChecker> obs_checker): statespace_(statespace), problem_(problem_def), obs_checker_(obs_checker){
        std::cout<<"RRTX constructor \n";
}


void RRTX::setStart(const Eigen::VectorXd& start) {
    robot_state_index_ = statespace_->getNumStates();
    tree_.push_back(std::make_shared<RRTxNode>(statespace_->addState(start) ,  tree_.size()));
    std::cout << "RRTX: Start node created on Index: " << robot_state_index_ << "\n";
}
void RRTX::setGoal(const Eigen::VectorXd& goal) {
    root_state_index_ = statespace_->getNumStates();
    auto node = std::make_shared<RRTxNode>(statespace_->addState(goal) ,  tree_.size());
    vbot_index_ = 1;
    vbot_node_ = node.get();

    
    tree_.push_back(node); // Fixed parenthesis
    std::cout << "RRTX: Goal node created on Index: " << root_state_index_ << "\n";
}



std::vector<int> RRTX::getPathIndex() const {
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
std::vector<Eigen::VectorXd> RRTX::getPathPositions() const {
    std::vector<Eigen::VectorXd> path_positions;
    
    if (vbot_node_ == nullptr) {
        return path_positions;
    }

    // Get the dimension from the first node to check consistency
    const size_t expected_dim = vbot_node_->getStateValue().size();

    // Add robot position if it matches dimension
    if (robot_position_.size() == expected_dim) {
        path_positions.push_back(robot_position_);
    }

    RRTxNode* current_node = vbot_node_;
    while (current_node != nullptr) {
        Eigen::VectorXd state = current_node->getStateValue();
        if (state.size() != expected_dim) {
            throw std::runtime_error("Inconsistent state dimensions in path");
        }
        path_positions.push_back(state);
        current_node = current_node->getParent();
    }

    return path_positions;
}

void RRTX::setRobotIndex(const Eigen::VectorXd& robot_position) {
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

void RRTX::clearPlannerState() {
    // Clear all critical variables
    // start_.reset();
    // goal_.reset();
    // path_.clear();

    // for (auto& node : tree_) {
    //     node.reset();  // Explicitly reset each shared_ptr
    // }

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

    // Clear the inconsistency queue
    inconsistency_queue_.clear();



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


void RRTX::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    auto start = std::chrono::high_resolution_clock::now();
    clearPlannerState();

    sample_counter = 0;


    visualization_ = visualization;



    num_of_samples_ = params.getParam<int>("num_of_samples");
    partial_update = params.getParam<bool>("partial_update");
    ignore_sample = params.getParam<bool>("ignore_sample");

    lower_bound_ = problem_->getLowerBound();
    upper_bound_ = problem_->getUpperBound();
    use_kdtree = params.getParam<bool>("use_kdtree");
    std::string kdtree_type = params.getParam<std::string>("kdtree_type");
    if (use_kdtree == true && kdtree_type == "NanoFlann")
        kdtree_ = std::make_shared<NanoFlann>(statespace_->getDimension());
    else
        throw std::runtime_error("Unknown KD-Tree type");

    std::cout << "RRTX setup complete: num_of_samples=" << num_of_samples_
                << ", bounds=[" << lower_bound_ << ", " << upper_bound_ << "]\n";


    std::cout << "--- \n";
    std::cout << "Taking care of the samples: \n \n";
    setStart(problem_->getStart());
    std::cout << "--- \n";
    setGoal(problem_->getGoal()); //robots current position

    // put the start and goal node in kdtree
    if (use_kdtree == true) {
        kdtree_->addPoints(statespace_->getSamplesCopy());
        kdtree_->buildTree();
    }

    /////////////////////////SETTING UP DS//////////////


    tree_.at(0)->setCost(0);
    tree_.at(0)->setLMC(0);

    edge_length_[0] = -std::numeric_limits<double>::infinity();
    edge_length_[1] = -std::numeric_limits<double>::infinity();

    v_indices_.insert(0);
    // v_indices_.insert(1); 
    ///////////////////Neighborhood Radius////////////////////////////////
    dimension_ = statespace_->getDimension();
    int d = dimension_;
    double mu = std::pow(problem_->getUpperBound() - problem_->getLowerBound() , 2);
    double zetaD = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
    gamma_ = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d);
    // gamma_ = 2 * std::pow(1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); //FMT star gamma



    // Since i want to put a cap on the number of samples and i want RRTX to be as close as to FMTX im gonna set step size (delta) to this:
    factor = params.getParam<double>("factor");
    std::cout<<"factor: "<<factor<<"\n";
    delta = factor * gamma_ * std::pow(std::log(num_of_samples_) / num_of_samples_, 1.0 / d);
    // delta = 5.0;
    std::cout << "Computed value of delta: " << delta << std::endl;



    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "---\n";

    sample_counter = 1;
}


// Well for now I use this for the first tree creation
void RRTX::plan() {

    auto start = std::chrono::high_resolution_clock::now();
    // if (cap_samples_==true && sample_counter < num_of_samples_) { // TODO: later when you add the robot you can put the condtion of the while loop here and we use the while true outside because we want it to always work to update the gazebo obstale positions
    while ( cap_samples_==true && sample_counter < num_of_samples_) { // TODO: later when you add the robot you can put the condtion of the while loop here and we use the while true outside because we want it to always work to update the gazebo obstale positions
        neighborhood_radius_ = shrinkingBallRadius();
        // neighborhood_radius_ = 10.0;
        // delta = neighborhood_radius_;

        Eigen::VectorXd sample = Eigen::VectorXd::Random(dimension_);
        sample = lower_bound_ + (upper_bound_ - lower_bound_) * (sample.array() + 1) / 2;

        sample_counter++;
        std::vector<size_t> nearest_indices = kdtree_->knnSearch(sample, 1);
        RRTxNode* nearest_node = tree_[nearest_indices[0]].get();
        Eigen::VectorXd nearest_state = nearest_node->getStateValue();
        
        // Steer towards sample
        Eigen::VectorXd direction = sample - nearest_state;
        double distance = direction.norm();
        if (distance > delta) {
            sample = nearest_state + (direction/distance) * (delta);
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


bool RRTX::extend(Eigen::VectorXd v) {
    auto new_node = std::make_shared<RRTxNode>(statespace_->addState(v), sample_counter);
    auto neighbors = kdtree_->radiusSearch(new_node->getStateValue(), neighborhood_radius_ + 0.01);
    
    findParent(new_node, neighbors);

    if (!new_node->getParent()) {
        sample_counter--;
        return false;
    }

    tree_.push_back(new_node);
    kdtree_->addPoint(new_node->getStateValue());
    kdtree_->buildTree(); 
    // Algorithm 2 lines 7-13 implementation
    for (size_t idx : neighbors) {
        RRTxNode* neighbor = tree_[idx].get();
        if (neighbor == new_node.get()) continue;

        // const bool v_to_u_free = obs_checker_->isObstacleFree(new_node->getStateValue(), neighbor->getStateValue()); // This we do in find parent and we don't need to do it again but right now i didn't store them in findParent
        const bool u_to_v_free = obs_checker_->isObstacleFree(neighbor->getStateValue(), new_node->getStateValue());
        const bool v_to_u_free = u_to_v_free;  // Easy way out for now since im not doing tracjetories right now
        const double dist = (new_node->getStateValue() - neighbor->getStateValue()).norm();

        // Persistent outgoing from new node (N⁰+)
        if (v_to_u_free) {
            new_node->addNeighbor(neighbor, true, false, dist);  // N⁰+(v) ← u
        }
        
        // Temporary outgoing from neighbors (Nr+)
        if (u_to_v_free) {
            neighbor->addNeighbor(new_node.get(), false, true, dist);  // Nr+(u) ← v
        }
    }
    
    return true;
}







void RRTX::findParent(std::shared_ptr<RRTxNode> v, const std::vector<size_t>& candidates) {
    double min_lmc = INFINITY;
    RRTxNode* best_parent = nullptr;
    double best_dist = 0.0;

    for (size_t idx : candidates) {
        auto& candidate = tree_[idx];
        if (candidate == v) continue;
        const double dist = (v->getStateValue() - candidate->getStateValue()).norm();
        /*
          The obstalce check we do here right now is the v->u  (new node to neighbors) and can also be used for the v->u trajcetories in extend function
          obstalce check (maybe later use a map or something) but u->v should be done in extend.
        */
        if (dist <= neighborhood_radius_+0.01 && obs_checker_->isObstacleFree(v->getStateValue(), candidate->getStateValue())) {
            const double candidate_lmc = candidate->getLMC() + dist;
            
            if (candidate_lmc < min_lmc && candidate_lmc < v->getLMC()) {
                min_lmc = candidate_lmc;
                best_parent = candidate.get();
                best_dist = dist;
            }
        }
    }

    if (best_parent) {
        v->setParent(best_parent, best_dist);
        v->setLMC(min_lmc);
        edge_length_[v->getIndex()] = best_dist;
    }
}






void RRTX::rewireNeighbors(RRTxNode* v) {
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
            u->setLMC(candidate_lmc);
            // makeParentOf(u, v, edge.distance);
            u->setParent(v,edge.distance);
            edge_length_[u->getIndex()] = edge.distance;
            if (u->getCost() - candidate_lmc > epsilon_) {
                verifyQueue(u);
            }
        }
    }
}

void RRTX::reduceInconsistency() {
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



double RRTX::shrinkingBallRadius() const {
    auto rad = factor * gamma_ * pow(log(tree_.size()) / tree_.size(), 1.0/dimension_);
    return std::min(rad, delta);

}


std::unordered_set<int> RRTX::findSamplesNearObstacles(
    const std::vector<Obstacle>& obstacles, double max_length) {
    std::unordered_set<int> conflicting_samples;
        
    for (const auto& obstacle : obstacles) {
        double obstacle_radius;
        if (obstacle.type == Obstacle::CIRCLE) {
            // For circles: radius + inflation
            obstacle_radius = obstacle.dimensions.circle.radius + obstacle.inflation;
        } else { // BOX
            // For boxes: half diagonal + inflation
            double half_diagonal = std::sqrt(
                std::pow(obstacle.dimensions.box.width/2, 2) + 
                std::pow(obstacle.dimensions.box.height/2, 2)
            );
            obstacle_radius = half_diagonal + obstacle.inflation;
        }

        // Calculate search radius using safety margin formula
        double search_radius = std::sqrt(
            std::pow(obstacle_radius, 2) + 
            std::pow(max_length / 2.0, 2)
        );

        auto sample_indices = kdtree_->radiusSearch(obstacle.position, search_radius);
        conflicting_samples.insert(sample_indices.begin(), sample_indices.end());
    }
    
    return conflicting_samples;
}


void RRTX::updateLMC(RRTxNode* v) {
    cullNeighbors(v);
    double min_lmc = v->getLMC();
    RRTxNode* best_parent = nullptr;
    double best_edge_distance = INFINITY;  // Track the distance of the best edge

    // Iterate over incoming edges (u → v)
    for (const auto& [u, edge] : v->incomingEdges()) {
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


        v->setParent(best_parent, best_edge_distance);  // Use the captured distance
        v->setLMC(min_lmc);
    } 
}


void RRTX::cullNeighbors(RRTxNode* v) {
    if (cap_samples_ == true && sample_counter >= num_of_samples_-1)
    // if (cap_samples_ == true && update_obstacle == true)
        return; // to not waste time when we put a cap on the number of samples!


    auto& outgoing = v->outgoingEdges();
    auto it = outgoing.begin();
    while (it != outgoing.end()) {
        auto [neighbor, edge] = *it;
        if (!edge.is_initial && 
            edge.distance > neighborhood_radius_+0.01 &&// (v->getStateValue() - neighbor->getStateValue()).norm() > neighborhood_radius_ &&
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


void RRTX::verifyQueue(RRTxNode* node) {
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







void RRTX::verifyOrphan(RRTxNode* node) {
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

void RRTX::propagateDescendants() {
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



        node->setParent(nullptr, 0.0);
    }

    Vc_T_.clear();
}





bool RRTX::isValidEdge(RRTxNode* from, RRTxNode* to, const EdgeInfo& edge) const {
    return edge.distance != INFINITY && 
           obs_checker_->isObstacleFree(from->getStateValue(), to->getStateValue());
}

void RRTX::visualizeTree() {
    std::vector<Eigen::VectorXd> nodes;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    const double goal_cost = vbot_node_ ? vbot_node_->getCost() : INFINITY;
    
    // Collect valid nodes and their connections
    std::unordered_set<RRTxNode*> valid_nodes;
    for (const auto& node : tree_) {
        if (node->getCost() <= goal_cost) {
            nodes.push_back(node->getStateValue());
            valid_nodes.insert(node.get());
        }
    }

    // Generate edges for valid nodes
    for (const auto& node : valid_nodes) {
        if (node->getParent()) {
            edges.emplace_back(node->getParent()->getStateValue(),
                             node->getStateValue());
        }
    }

    visualization_->visualizeEdges(edges);
}

void RRTX::visualizePath(const std::vector<RRTxNode*>& path) {
    std::vector<Eigen::VectorXd> nodes;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;

    // Reserve memory for efficiency
    nodes.reserve(path.size());
    edges.reserve(path.size());

    for (const auto& node : path) {
        nodes.push_back(node->getStateValue());
        if (node->getParent()) {
            edges.emplace_back(node->getParent()->getStateValue(),
                             node->getStateValue());
        }
    }

    visualization_->visualizeEdges(edges, "map", "0.0,1.0,0.0");
}


void RRTX::visualizeSmoothedPath(const std::vector<Eigen::VectorXd>& shortest_path_) {
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


std::vector<Eigen::VectorXd> RRTX::getSmoothedPathPositions(int num_intermediates, int smoothing_passes) const {
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


std::vector<Eigen::VectorXd> RRTX::interpolatePath(const std::vector<Eigen::VectorXd>& path, int num_intermediates) const {
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


std::vector<Eigen::VectorXd> RRTX::smoothPath(const std::vector<Eigen::VectorXd>& path, int window_size) const {
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


void RRTX::updateObstacleSamples(const std::vector<Obstacle>& obstacles) {
    update_obstacle = true;

    // Common initialization
    if (edge_length_[max_length_edge_ind] != max_length) {
        auto max_it = std::max_element(edge_length_.begin(), edge_length_.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });
        max_length = max_it->second;
        max_length_edge_ind = max_it->first;
    }

    auto current = findSamplesNearObstacles(obstacles, max_length);
    // if (current == samples_in_obstacles_) return; // Early exit if nothing has changed

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




    if (ignore_sample) {
        // Version 1: Track samples on obstacles without explicit checks

        // I update here because in removeObstalce i need to avoid samples that are still on obstalces
        samples_in_obstacles_ = current;
        
        if (!added.empty()) {
            addNewObstacle(added);
            propagateDescendants();
            verifyQueue(tree_[vbot_index_].get());
        }
        if (!removed.empty()) {
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

        if (!removed.empty()) removeObstacle(prev);
        if (!added.empty()) {
            addNewObstacle(cur);
            propagateDescendants();
            verifyQueue(tree_[vbot_index_].get());
        }
    }

    reduceInconsistency();
}

void RRTX::addNewObstacle(const std::vector<int>& added_indices) {
    for (int idx : added_indices) {
        RRTxNode* node = tree_[idx].get();
        
        if (ignore_sample) {
            samples_in_obstacles_.insert(idx);
        }

        for (auto& [u, edge] : node->outgoingEdges()) {
            // Common edge invalidation logic
            const bool should_invalidate = ignore_sample ? true : 
                (edge.distance != INFINITY && 
                !obs_checker_->isObstacleFree(node->getStateValue(), u->getStateValue()));

            if (!should_invalidate) continue;
            /*
                so since we are iterating over outgoing (you can iterate over ingoing also doesnt matter!), and our focus is on "node" thne if node has collision with its neighbor u then
                outgoing node's dist from node->u should be inf, also incoming node from u->node should be inf also incoming node from node->u should be inf!
                so its like the outgoing of node i.e, node->u (with the focus on node!) and incoming of node (u->node) should be inf and also the incoming of u from node i.e, node->u needs to be inf, but how about outgoing of u i.e., u->node  --> this should be ALSO handled!--> don't confuse the asymetry
            */
            // Common invalidation operations
            edge.distance = INFINITY;
            u->incomingEdges().at(node).distance = INFINITY;
            u->outgoingEdges().at(node).distance = INFINITY;
            node->incomingEdges().at(u).distance = INFINITY;

            // Common parent relationship handling
            if (u->getParent() == node) {
                u->setParent(nullptr, INFINITY);
                verifyOrphan(u);
            }
            if (node->getParent() == u) {
                node->setParent(nullptr, INFINITY);
                verifyOrphan(node);
            }
        }

        // // Additional operations for ignore_sample mode
        // if (ignore_sample) {
        //     node->setCost(INFINITY);
        //     node->setLMC(INFINITY);
        //     edge_length_[idx] = -INFINITY;
        //     node->setParent(nullptr, 0.0);
        //     verifyOrphan(node);
        // }
    }
}
void RRTX::removeObstacle(const std::vector<int>& removed_indices) {
    for (int idx : removed_indices) {
        RRTxNode* node = tree_[idx].get();

        if (ignore_sample) {
            samples_in_obstacles_.erase(idx); // Update obstacle set
        }

        for (auto& [u, edge] : node->outgoingEdges()) {
            // Mode-specific condition components
            const bool is_neighbor_clear = !samples_in_obstacles_.count(u->getIndex());
            const bool was_invalidated = (edge.distance == INFINITY);
            const bool is_now_clear = obs_checker_->isObstacleFree(node->getStateValue(), 
                                                                  u->getStateValue());
            
            // Unified restoration condition
            const bool should_restore = ignore_sample 
                ? is_neighbor_clear          // Sample tracking mode
                : was_invalidated && is_now_clear;  // Explicit check mode

            if (!should_restore) continue;

            // Common restoration logic
            edge.distance = edge.distance_original;
            u->incomingEdges().at(node).distance = edge.distance_original;
            u->outgoingEdges().at(node).distance = edge.distance_original;
            node->incomingEdges().at(u).distance = edge.distance_original;
        }

        // Common node updates
        updateLMC(node);
        if (node->getCost() != node->getLMC()) {
            verifyQueue(node);
        }
    }
}