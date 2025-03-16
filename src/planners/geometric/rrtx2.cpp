#include "motion_planning/planners/geometric/rrtx2.hpp"

RRTX::RRTX(std::unique_ptr<StateSpace> statespace, 
    std::shared_ptr<ProblemDefinition> problem_def,
    std::shared_ptr<ObstacleChecker> obs_checker): statespace_(std::move(statespace)), problem_(problem_def), obs_checker_(obs_checker) ,inconsistency_queue_(50){
        std::cout<<"RRTX constructor \n";
}


void RRTX::setStart(const Eigen::VectorXd& start) {
    robot_state_index_ = statespace_->getNumStates();
    tree_.push_back(std::make_shared<RRTxNode>(statespace_->addState(start) ,  tree_.size()));
    std::cout << "RRTX: Start node created on Index: " << robot_state_index_ << "\n";
}
void RRTX::setGoal(const Eigen::VectorXd& goal) {
    root_state_index_ = statespace_->getNumStates();
    tree_.push_back(std::make_shared<RRTxNode>(statespace_->addState(goal) ,  tree_.size())); // Fixed parenthesis
    std::cout << "RRTX: Goal node created on Index: " << root_state_index_ << "\n";
}



std::vector<int> RRTX::getPathIndex() const {

    int idx = vbot_index_;
    std::vector<int> path;
    while (idx != -1) {
        path.push_back(idx);
        idx = tree_.at(idx)->getParentIndex();
    }
    return path;
}

std::vector<Eigen::VectorXd> RRTX::getPathPositions() const {
    int idx = vbot_index_;
    std::vector<Eigen::VectorXd> path_positions;

    if (vbot_index_ != -1){
        path_positions.push_back(robot_position_);
    }

    while (idx != -1) {
        path_positions.push_back(tree_.at(idx)->getStateVlaue());
        idx = tree_.at(idx)->getParentIndex();
    }
    return path_positions;
}


void RRTX::setRobotIndex(const Eigen::VectorXd& robot_position) {
    const double MAX_SEARCH_RADIUS = 5.0;  // Meters
    auto candidates = kdtree_->radiusSearch(robot_position, MAX_SEARCH_RADIUS);

    std::shared_ptr<RRTxNode> best_node = nullptr;
    double min_total_cost = INFINITY;

    for (int idx : candidates) {
        std::shared_ptr<RRTxNode> node = tree_[idx]; // Use the existing shared_ptr
        if (node->getCost() == INFINITY) continue;

        const double distance_to_node = (robot_position - node->getStateVlaue()).norm();
        const double total_cost = distance_to_node + node->getCost();

        if (total_cost < min_total_cost) {
            min_total_cost = total_cost;
            best_node = node; // Assign the shared_ptr directly
        }
    }

    vbot_node_ = best_node ? best_node : vbot_node_;
}

void RRTX::clearPlannerState() {
    // Clear all critical variables
    // start_.reset();
    // goal_.reset();
    // path_.clear();

    // for (auto& node : tree_) {
    //     node.reset();  // Explicitly reset each shared_ptr
    // }
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
    vbot_index_ = 1;
    vbot_node_ = std::make_shared<RRTxNode>(statespace_->addState(problem_->getGoal()) , vbot_index_);

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
    gamma_ = 2 * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d);


    // Since i want to put a cap on the number of samples and i want RRTX to be as close as to FMTX im gonna set step size (delta) to this:
    double factor = 2.0;
    // delta = factor * gamma_ * std::pow(std::log(num_of_samples_) / num_of_samples_, 1.0 / d);
    delta = 10.0;
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
        delta = neighborhood_radius_;
        Eigen::VectorXd sample = Eigen::VectorXd::Random(dimension_);
        sample = lower_bound_ + (upper_bound_ - lower_bound_) * (sample.array() + 1) / 2;

        sample_counter++;
        std::vector<size_t> nearest_indices = kdtree_->knnSearch(sample, 1);
        RRTxNode* nearest_node = tree_[nearest_indices[0]].get();
        Eigen::VectorXd nearest_state = nearest_node->getStateVlaue();
        
        // Steer towards sample
        Eigen::VectorXd direction = sample - nearest_state;
        double distance = direction.norm();
        if (distance > delta) {
            sample = nearest_state + (direction/distance) * (delta);
        }

        // Attempt to extend tree
        // if (obs_checker_->isObstacleFree(sample)) {
            bool node_added = extend(sample);
        // }
            
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
}


bool RRTX::extend(Eigen::VectorXd v) {
    auto new_node = std::make_shared<RRTxNode>(statespace_->addState(v), sample_counter);
    auto neighbors = kdtree_->radiusSearch(new_node->getStateVlaue(), neighborhood_radius_ + 0.01);
    
    findParent(new_node, neighbors);

    if (!new_node->getParent()) {
        sample_counter--;
        return false;
    }

    tree_.push_back(new_node);
    kdtree_->addPoint(new_node->getStateVlaue());
    kdtree_->buildTree(); 
    // Algorithm 2 lines 7-13 implementation
    for (size_t idx : neighbors) {
        RRTxNode* neighbor = tree_[idx].get();
        if (neighbor == new_node.get()) continue;

        const bool v_to_u_free = obs_checker_->isObstacleFree(new_node->getStateVlaue(), neighbor->getStateVlaue());
        // const bool u_to_v_free = obs_checker_->isObstacleFree(neighbor->getStateVlaue(), new_node->getStateVlaue());
        const bool u_to_v_free = v_to_u_free;
        const double dist = (new_node->getStateVlaue() - neighbor->getStateVlaue()).norm();

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
\






void RRTX::findParent(std::shared_ptr<RRTxNode> v, const std::vector<size_t>& candidates) {
    double min_lmc = INFINITY;
    RRTxNode* best_parent = nullptr;
    double best_dist = 0.0;

    for (size_t idx : candidates) {
        auto& candidate = tree_[idx];
        if (candidate == v) continue;

        const double dist = (v->getStateVlaue() - candidate->getStateVlaue()).norm();
        if (dist <= neighborhood_radius_+0.01 && obs_checker_->isObstacleFree(v->getStateVlaue(), candidate->getStateVlaue())) {
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
        if (u == v->getParent() || !isValidEdge(u, v, edge)) continue;
        // if (u == v->getParent() ) continue;


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
        //     && 
        //    (inconsistency_queue_.top().min_key < vbot_node_->getCost() ||
        //     vbot_node_->getLMC() != vbot_node_->getCost() ||
        //     vbot_node_->getCost() == INFINITY ||
        //     inconsistency_queue_.contains(vbot_node_.get())) 
        ) 
    {
        auto top_element = inconsistency_queue_.top();
        inconsistency_queue_.pop();

        RRTxNode* node = top_element.node;


        // int node_idx = findNodeIndex(node);
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
    double factor = 2.0;
    auto rad = factor * gamma_ * pow(log(tree_.size()) / tree_.size(), 1.0/dimension_);
    return std::min(rad, delta);;

}


std::unordered_set<int> RRTX::findSamplesNearObstacles(
    const std::vector<Obstacle>& obstacles, double max_length) {
    std::unordered_set<int> conflicting_samples;
    
    for (const auto& obstacle : obstacles) {
        // Query samples within obstacle radius (5 units)
        // auto sample_indices = kdtree_->radiusSearch(obstacle.position, 1.0 * obstacle.radius);
        auto sample_indices = kdtree_->radiusSearch(obstacle.position, std::sqrt(std::pow(obstacle.radius, 2) + std::pow(max_length / 2.0, 2)));

        conflicting_samples.insert(sample_indices.begin(), sample_indices.end());
    }
    
    return conflicting_samples;
}




// To handle changes in the environment
void RRTX::updateObstacleSamples(const std::vector<Obstacle>& obstacles) {

    if (edge_length_[max_length_edge_ind] != max_length) // This condition also triggeres the first calculation os It's okay
    {
        auto max_it = std::max_element(edge_length_.begin() , edge_length_.end() ,[](const std::pair<int, double>& a , const std::pair<int, double>& b){
            return a.second < b.second;
        });
        max_length = max_it->second;
        max_length_edge_ind = max_it->first;
        // std::cout<<max_it->first << "  " << max_it->second <<" \n"; 

    }
    // // Visualizing the maximum length node
    // if (max_length_edge_ind !=-1){
    //     std::string color_str = "0.0,0.0,1.0"; // Blue color
    //     std::vector<Eigen::VectorXd> positions4;
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(max_length_edge_ind)->getStateVlaue();
    //     positions4.push_back(vec);
    //     visualization_->visualizeNodes(positions4,"map",color_str);

    // }


    // Similar obstacle sampling to FMTX but with RRTX propagation
    // TODO: Later i need to implement the max length of and edge to find the scaling paramter accurately
    auto current = findSamplesNearObstacles(obstacles, max_length); // TODO: i don't think its correct to scale this but its necessary to (it needs to be integrated with max length) --> its not correct in a sense that the scaled onces shoudlnt go into the samples in obstalces i guess because i avoid them in the main while loop --> weirdly it works but i'll take a look later!



    std::vector<int> added;
    for (int sample : current) {
        if (samples_in_obstacles_.find(sample) == samples_in_obstacles_.end()) {
            added.push_back(sample); 
        }
    }

    std::vector<int> removed;
    for (int sample : samples_in_obstacles_) {
        if (current.find(sample) == current.end()) {
            removed.push_back(sample); 
        }
    }


    // // Visualizing the maximum length node
    // std::vector<Eigen::VectorXd> positions4;
    // std::string color_str = "0.0,0.0,1.0"; // Blue color
    // for (int r : removed){
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(r)->getStateVlaue();
    //     positions4.push_back(vec);

    // }
    // visualization_->visualizeNodes(positions4,"map",color_str);


    // I update here because in removeObstalce i need to avoid samples that are still on obstalces
    // Update the set of samples in obstacles
    samples_in_obstacles_ = std::move(current);


    // Handle added and removed samples
    if (!added.empty()) {
        addNewObstacle(added);
        propagateDescendants();
        verifyQueue(tree_[vbot_index_].get()); // Convert index to node pointer

    }
    if (!removed.empty()) {
        removeObstacle(removed);
    }

    reduceInconsistency();
}

void RRTX::updateLMC(RRTxNode* v) {
    cullNeighbors(v);
    double min_lmc = v->getLMC();
    RRTxNode* best_parent = nullptr;
    double best_edge_distance = INFINITY;  // Track the distance of the best edge

    // Iterate over incoming edges (u → v)
    for (const auto& [u, edge] : v->incomingEdges()) {
        if (Vc_T_.count(u->getIndex()) || edge.distance == INFINITY) continue;

        const double candidate_lmc = u->getLMC() + edge.distance;
        if (candidate_lmc < min_lmc) {
            min_lmc = candidate_lmc;
            best_parent = u;
            best_edge_distance = edge.distance;  // Capture the distance here
        }
    }

    if (best_parent) {
        v->setParent(best_parent, best_edge_distance);  // Use the captured distance
        v->setLMC(min_lmc);
    } 
}

// void RRTX::updateLMC(RRTxNode* v) {

//     cullNeighbors(v);
//     double min_lmc = INFINITY;
//     RRTxNode* best_parent = nullptr;

//     for (auto& [u, edge] : v->outgoingEdges()) {
//         if (Vc_T_.count(u->getIndex()) || u == v->getParent()) continue;
//         if (edge.distance == INFINITY) continue;

//         const double candidate_lmc = u->getLMC() + edge.distance;
//         if (candidate_lmc < min_lmc) {
//             min_lmc = candidate_lmc;
//             best_parent = u;
//         }
//     }

//     if (best_parent) {
//         makeParentOf(v, best_parent, best_parent->outgoingEdges()[v].distance);
//         v->setLMC(min_lmc);
//     } else {
//         v->setLMC(INFINITY);
//     }
// }



// Mind that nothing is_initial (original neighbors) must not get deleted

// void RRTX::cullNeighbors(RRTxNode* v) {
//     auto& outgoing = v->outgoingEdges();
//     auto it = outgoing.begin();
    
//     while (it != outgoing.end()) {
//         auto [neighbor, edge] = *it;
        
//         // Fix 1: Typo in getStateValue()
//         const double actual_dist = (v->getStateVlaue() - neighbor->getStateVlaue()).norm();
        
//         // Fix 2: Check edge validity before distance comparison
//         const bool is_valid_edge = isValidEdge(v, neighbor, edge);
        
//         if (!edge.is_initial && is_valid_edge && 
//             actual_dist > neighborhood_radius_ && 
//             neighbor != v->getParent()) 
//         {
//             // Remove bidirectional temporary edges
//             auto& incoming = neighbor->incomingEdges();
//             if (auto incoming_it = incoming.find(v); incoming_it != incoming.end()) {
//                 // Only remove temporary incoming edges (Nr⁻)
//                 if (!incoming_it->second.is_initial) {
//                     incoming.erase(incoming_it);
//                 }
//             }
//             it = outgoing.erase(it);
//         } else {
//             ++it;
//         }
//     }
// }

void RRTX::cullNeighbors(RRTxNode* v) {
    if (cap_samples_ == true && sample_counter > num_of_samples_)
        return; // to not waste time when we put a cap on the number of samples!
    auto& outgoing = v->outgoingEdges();
    auto it = outgoing.begin();
    while (it != outgoing.end()) {
        auto [neighbor, edge] = *it;
        if (!edge.is_initial && 
            (v->getStateVlaue() - neighbor->getStateVlaue()).norm() > neighborhood_radius_ &&
            neighbor != v->getParent() ) 
        {
            // Remove temporary edge bidirectionally
            // neighbor->incomingEdges().erase(v);

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


void RRTX::makeParentOf(RRTxNode* child, RRTxNode* new_parent, double edge_dist ) {
    if (RRTxNode* old_parent = child->getParent()) {
        // Use successorsMutable() for write access
        auto& succ = old_parent->successorsMutable();
        succ.erase(std::remove(succ.begin(), succ.end(), child), succ.end());
    }
    
    child->setParent(new_parent, edge_dist);
    
    if (new_parent) {
        // Use successorsMutable() for write access
        new_parent->successorsMutable().push_back(child);
    }
}


void RRTX::verifyQueue(RRTxNode* node) {
    const double min_key = std::min(node->getLMC(), node->getCost());
    const double g_value = node->getCost();
    
    inconsistency_queue_.upsert({
        min_key,
        g_value,
        node
    });
}



void RRTX::removeObstacle(const std::vector<int>& removed_indices) {
    for (int idx : removed_indices) {
        RRTxNode* node = tree_[idx].get();
        samples_in_obstacles_.erase(idx); // Remove from obstacle set

        // First loop: Process outgoing edges (node → neighbor)
        for (auto& [neighbor, edge_info] : node->outgoingEdges()) {
            const int neighbor_idx = neighbor->getIndex();
            if (samples_in_obstacles_.count(neighbor_idx)) continue;

            // Recalculate distance and check obstacle-free path
            const Eigen::VectorXd node_state = node->getStateVlaue();
            const Eigen::VectorXd neighbor_state = neighbor->getStateVlaue();
            const double dist = (node_state - neighbor_state).norm();
            // const bool is_free = obs_checker_->isObstacleFree(node_state, neighbor_state);
            const bool is_free = true;

            // Update node's outgoing edge
            edge_info.distance = is_free ? dist : INFINITY;

            // Update neighbor's corresponding incoming edge (preserve is_initial)
            auto neighbor_in_edge = neighbor->incomingEdges().find(node);
            if (neighbor_in_edge != neighbor->incomingEdges().end()) {
                neighbor_in_edge->second.distance = is_free ? dist : INFINITY;
            }
        }

        // Second loop: Process incoming edges (neighbor → node)
        for (auto& [neighbor, edge_info] : node->incomingEdges()) {
            const int neighbor_idx = neighbor->getIndex();
            if (samples_in_obstacles_.count(neighbor_idx)) continue;

            // Recalculate distance and check obstacle-free path
            const Eigen::VectorXd node_state = node->getStateVlaue();
            const Eigen::VectorXd neighbor_state = neighbor->getStateVlaue();
            const double dist = (neighbor_state - node_state).norm();
            // const bool is_free = obs_checker_->isObstacleFree(neighbor_state, node_state);
            const bool is_free = true;

            // Update node's incoming edge
            edge_info.distance = is_free ? dist : INFINITY;

            // Update neighbor's corresponding outgoing edge (preserve is_initial)
            auto neighbor_out_edge = neighbor->outgoingEdges().find(node);
            if (neighbor_out_edge != neighbor->outgoingEdges().end()) {
                neighbor_out_edge->second.distance = is_free ? dist : INFINITY;
            }
        }

        // Update LMC and queue
        updateLMC(node);
        if (node->getCost() != node->getLMC()) {
            verifyQueue(node);
        }
    }
}



void RRTX::addNewObstacle(const std::vector<int>& added_indices) {
    for (int idx : added_indices) {
        RRTxNode* node = tree_[idx].get();
        samples_in_obstacles_.insert(idx);

        // Invalidate all edges connected to this node
        for (auto& [u, edge] : node->outgoingEdges()) {

            edge.distance = INFINITY;
            u->incomingEdges()[node].distance = INFINITY;
            u->outgoingEdges()[node].distance = INFINITY;
            if (u->getParent() == node) verifyOrphan(u);
        }
        for (auto& [u, edge] : node->incomingEdges()) {

            edge.distance = INFINITY;
            u->outgoingEdges()[node].distance = INFINITY;
            u->incomingEdges()[node].distance = INFINITY;
            if (node->getParent() == u) verifyOrphan(node);
        }

        // Mark node as invalid and propagate
        node->setCost(INFINITY);
        node->setLMC(INFINITY);
        edge_length_[idx] = -INFINITY;
        node->setParent(nullptr, 0.0);  // Clear parent
        verifyOrphan(node);  // Add to Vc_T_
    }
}

void RRTX::verifyOrphan(RRTxNode* node) {
    inconsistency_queue_.remove(node);
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
        for (RRTxNode* child : current->successors()) {
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
           obs_checker_->isObstacleFree(from->getStateVlaue(), to->getStateVlaue());
}

void RRTX::visualizeTree() {
    std::vector<Eigen::VectorXd> nodes;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    const double goal_cost = vbot_node_ ? vbot_node_->getCost() : INFINITY;
    
    // Collect valid nodes and their connections
    std::unordered_set<RRTxNode*> valid_nodes;
    for (const auto& node : tree_) {
        if (node->getCost() <= goal_cost) {
            nodes.push_back(node->getStateVlaue());
            valid_nodes.insert(node.get());
        }
    }

    // Generate edges for valid nodes
    for (const auto& node : valid_nodes) {
        if (node->getParent()) {
            edges.emplace_back(node->getParent()->getStateVlaue(),
                             node->getStateVlaue());
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
        nodes.push_back(node->getStateVlaue());
        if (node->getParent()) {
            edges.emplace_back(node->getParent()->getStateVlaue(),
                             node->getStateVlaue());
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