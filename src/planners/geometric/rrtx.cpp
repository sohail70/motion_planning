#include "motion_planning/planners/geometric/rrtx.hpp"

RRTX::RRTX(std::unique_ptr<StateSpace> statespace, 
    std::unique_ptr<ProblemDefinition> problem_def,
    std::shared_ptr<ObstacleChecker> obs_checker): statespace_(std::move(statespace)), problem_(std::move(problem_def)), obs_checker_(obs_checker) ,inconsistency_queue_(50){
        std::cout<<"RRTX constructor \n";
}


void RRTX::setStart(const Eigen::VectorXd& start) {
    robot_state_index_ = statespace_->getNumStates();
    tree_.push_back(std::make_shared<TreeNode>(statespace_->addState(start)));
    std::cout << "RRTX: Start node created on Index: " << robot_state_index_ << "\n";
}
void RRTX::setGoal(const Eigen::VectorXd& goal) {
    root_state_index_ = statespace_->getNumStates();
    tree_.push_back(std::make_shared<TreeNode>(statespace_->addState(goal)));
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

void RRTX::setRobotIndex(const Eigen::VectorXd& robot_position) {
    std::vector<size_t> nearest_indices = kdtree_->knnSearch(robot_position, 1);
    int nearest = nearest_indices.empty() ? -1 : static_cast<int>(nearest_indices[0]);  

    vbot_index_ = nearest;
}



void RRTX::setup(const PlannerParams& params, std::shared_ptr<Visualization> visualization) {
    auto start = std::chrono::high_resolution_clock::now();
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

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "---\n";


}


// Well for now I use this for the first tree creation
void RRTX::plan() {

    auto start = std::chrono::high_resolution_clock::now();
    // if (cap_samples_==true && sample_counter < num_of_samples_) { // TODO: later when you add the robot you can put the condtion of the while loop here and we use the while true outside because we want it to always work to update the gazebo obstale positions
    while ( cap_samples_==true && sample_counter < num_of_samples_) { // TODO: later when you add the robot you can put the condtion of the while loop here and we use the while true outside because we want it to always work to update the gazebo obstale positions
        neighborhood_radius_ = shrinkingBallRadius();
        Eigen::VectorXd v = Eigen::VectorXd::Random(dimension_);
        v = lower_bound_ + (upper_bound_ - lower_bound_) * (v.array() + 1) / 2; // TODO: need to add a raw unfirom sampling without creating state! in the euclidean state space class!

        sample_counter++;
        std::vector<size_t> nearest_indices = kdtree_->knnSearch(v, 1);
        int nearest = nearest_indices.empty() ? -1 : static_cast<int>(nearest_indices[0]); 
        Eigen::VectorXd nearest_state = tree_.at(nearest)->getStateVlaue();
        Eigen::VectorXd direction = v - nearest_state;
        double d = direction.norm();

        if (d > delta) {
            v = nearest_state + (direction / d) * delta; // Saturate the step
        }

        // since i don't want to ignore the v i don't implement the obstalce chekc on the point because later that point could be useful and since i want to put a cap on number of samples then this seems more useful
        // if v is not in obstalce then do this:
        if (obs_checker_->isObstacleFree(v)) // TODO OR NOT TODO --> mostly depends on your assumption if all obstalces are dynamic or not! because in the end the question is if those samples that are in the obstalce right now is gonna be useful or not in the later stages!. you can also keep the simulation running so that dynamic obstalces move and random samples fill those void areas if you don't want to comment this line.
            extend(v);
        int current_index = tree_.size() -1;
        if (v_indices_.find(current_index) != v_indices_.end()) {
            rewireNeighbors(current_index);
            reduceInconsistency();
            tree_.at(current_index)->setCost(tree_.at(current_index)->getLMC()); //According to my conversation with micheal on 2017! --> though he said we should set it to set it to  min lmc of neighbors and put it in findParent! --> my version looks simpler
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by while loop: " << duration.count() << " milliseconds\n";
}


void RRTX::extend(Eigen::VectorXd v) {
    /*
        Add the new node to the tree --> i do this a little bit earlier than rrtx since i need to use lmc in find parent!
        or else i have to create a separate lmc set for all the samples that im creating. pros and cons! because this one also feels a bit redundant if we have static obstalces and its like we are creating a treenode that will be useless forever!
    */
    tree_.push_back(std::make_shared<TreeNode>(statespace_->addState(v)));
    int current_index = tree_.size() - 1;
    kdtree_->addPoint(v);
    kdtree_->buildTree();

    // Find nearby candidates within the neighborhood radius
    auto neighbors = kdtree_->radiusSearch(v, neighborhood_radius_);

    // Find the best parent for the new node
    findParent(v, neighbors);

    // If no parent is found, remove the node and return
    if (tree_.at(current_index)->getParentIndex() == -1) {
        return;
    }

    // Add the new node to the set of TREE nodes. This is tau in rrtx (not my treenode!)
    v_indices_.insert(current_index);

    // Update neighbor relationships
    for (int u_index : neighbors) {
        if (u_index==current_index) {
            continue;
        }
        const Eigen::VectorXd& u_state = tree_.at(u_index)->getStateVlaue();
        // TODO: For now i don't distinguish between bidirectional paths between v,u and u,v. Later when we have to deal with dynamics and trajectory we'll deal with this.
        bool is_path_free = obs_checker_->isObstacleFree(v, u_state); 

        if (is_path_free) {
            // Update original and running neighbors
            N0_out_[current_index].insert(u_index);  // N0_out for v
            Nr_in_[u_index].insert(current_index);    // Nr_in for u

            Nr_out_[u_index].insert(current_index);   // Nr_out for u
            N0_in_[current_index].insert(u_index);    // N0_in for v

            // Store distances --> rrtx didn't mention that in pseudo code explicitly but it uses this (I guess) and weirdly you shouldn't use this map in cullNeighbors (because it could be inf and goes to the if condition)
            double distance = (v - u_state).norm();
            distance_[current_index][u_index] = distance;
            distance_[u_index][current_index] = distance;
        }
    }
}



void RRTX::findParent(Eigen::VectorXd v, const std::vector<size_t>& candidate_indices) {
    const Eigen::VectorXd& v_state = v;
    double min_cost = std::numeric_limits<double>::infinity();
    int best_parent = -1;
    int current_index_ = tree_.size()-1;
    // Iterate through all candidate nodes
    double best_parent_distance;
    for (int u_index : candidate_indices) {
        const Eigen::VectorXd& u_state = tree_.at(u_index)->getStateVlaue();

        // Compute trajectory and distance between v and u
        double distance = (v_state - u_state).norm();
        bool is_path_free = obs_checker_->isObstacleFree(v_state, u_state); //TODO: later

        // Check conditions for valid parent
        if (distance <= neighborhood_radius_ && 
            tree_.at(current_index_)->getLMC() > distance + tree_.at(u_index)->getLMC() &&
            is_path_free) {
            
            // Update best parent if this candidate is better
            double total_cost = distance +tree_.at(u_index)->getLMC();
            if (total_cost < min_cost) {
                min_cost = total_cost;
                best_parent = u_index;
                best_parent_distance = distance;
            }
        }
    }

    // If a valid parent is found, update the node's parent and lmc
    if (best_parent != -1) {
        tree_.at(current_index_)->setParentIndex(best_parent);
        tree_.at(current_index_)->setLMC(min_cost);
        tree_.at(best_parent)->setChildrenIndex(current_index_); // i don't know why rrtx pseudo code updates the children in the extend
        // Should i update the gvalue (costtoroot) here or wait so that in the rewire neighbor and reduce inconsistency to do it!? I put it in the plan function after reduceInconsistency
        edge_length_[current_index_] = best_parent_distance;
    }

}



void RRTX::rewireNeighbors(int v_index) {
    // Check if the node is inconsistent
    if (tree_[v_index]->getCost() - tree_[v_index]->getLMC() > epsilon_) {
        // Cull neighbors outside the current radius
        cullNeighbors(v_index);

        // Get incoming neighbors (N0_in ∪ Nr_in)
        std::unordered_set<int> incoming_neighbors = N0_in_[v_index];
        incoming_neighbors.insert(Nr_in_[v_index].begin(), Nr_in_[v_index].end());

        // Iterate over incoming neighbors, excluding the parent
        for (int u_index : incoming_neighbors) {
            if (u_index == tree_[v_index]->getParentIndex()) {
                continue;  // Skip the parent
            }

            // Get the trajectory cost from the distance map
            double trajectory_cost = std::numeric_limits<double>::infinity();
            if (distance_.count(v_index) && distance_[v_index].count(u_index)) {
                trajectory_cost = distance_[v_index][u_index];
            }

            // Check if the path through v is better
            if (tree_[u_index]->getLMC() > trajectory_cost + tree_[v_index]->getLMC()) {
                tree_[u_index]->setLMC(trajectory_cost + tree_[v_index]->getLMC());
                makeParentOf(u_index, v_index);
                edge_length_[u_index] = trajectory_cost;

                // If u is inconsistent, add it to the queue
                if (tree_[u_index]->getCost() - tree_[u_index]->getLMC() > epsilon_) {
                    verifyQueue(u_index);
                }
            }
        }
    }
}

void RRTX::reduceInconsistency() {
    while (!inconsistency_queue_.empty() 
            && (inconsistency_queue_.top().min_key < tree_[vbot_index_]->getCost() ||
                tree_[vbot_index_]->getLMC() != tree_[vbot_index_]->getCost() ||
                tree_[vbot_index_]->getCost() == std::numeric_limits<double>::infinity() ||
                inconsistency_queue_.contains(vbot_index_))
    ){
        QueueElement top_element = inconsistency_queue_.top();
        inconsistency_queue_.pop();

        int v_index = top_element.index;
        // // Skip the node if it is in the orphan set (Vc_T_) --> why remove from the Q in verify orphan when i can ignore it here?
        // if (Vc_T_.find(v_index) != Vc_T_.end()) {
        //     continue;
        // }

        // Process the node if it is inconsistent
        if (tree_[v_index]->getCost() - tree_[v_index]->getLMC() > epsilon_) {
            updateLMC(v_index);
            rewireNeighbors(v_index);
        }

        // Set the cost of the node to its LMC
        tree_[v_index]->setCost(tree_[v_index]->getLMC());
    }

}





double RRTX::shrinkingBallRadius() const {
    double factor = 2.0;
    auto rad = factor * gamma_ * pow(log(tree_.size()) / tree_.size(), 1.0/dimension_);
    return std::min(rad, delta);;

}


std::unordered_set<int> RRTX::findSamplesNearObstacles(
    const std::vector<Obstacle>& obstacles, 
    double max_length
) {
    std::unordered_set<int> conflicting_samples;
    
    for (const auto& obstacle : obstacles) {
        // Query samples within obstacle radius (5 units)
        // auto sample_indices = kdtree_->radiusSearch(obstacle.position, scale_factor * obstacle.radius);
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

    // I update here because in removeObstalce i need to avoid samples that are still on obstalces
    // Update the set of samples in obstacles
    samples_in_obstacles_ = std::move(current);


    // Handle added and removed samples
    if (!added.empty()) {
        addNewObstacle(added);
        propagateDescendants();
        verifyQueue(vbot_index_);
    }
    if (!removed.empty()) {
        removeObstacle(removed);
    }

    reduceInconsistency();
}


void RRTX::updateLMC(int v_index) {
    cullNeighbors(v_index);

    // Iterate over outgoing neighbors (N0_out ∪ Nr_out)
    std::unordered_set<int> outgoing_neighbors = N0_out_[v_index];
    outgoing_neighbors.insert(Nr_out_[v_index].begin(), Nr_out_[v_index].end());

    for (int vc_node : Vc_T_) {
        outgoing_neighbors.erase(vc_node);
    }

    for (int u_index : outgoing_neighbors) {
        if (tree_[u_index]->getParentIndex() == v_index) {
            continue;  // Skip the parent
        }

        double new_lmc = distance_[v_index][u_index] + tree_[u_index]->getLMC();

        if (tree_[v_index]->getLMC() > new_lmc) {
            makeParentOf(v_index, u_index);
            tree_[v_index]->setLMC(new_lmc);
        }
    }
}



void RRTX::cullNeighbors(int v_index) {
    // Loop over outgoing neighbors (Nr_out)
    if (cap_samples_ == true && sample_counter > num_of_samples_)
        return; // to not waste time when we put a cap on the number of samples!
    for (auto it = Nr_out_[v_index].begin(); it != Nr_out_[v_index].end();) {
        int u_index = *it;

        // Do not use the distance map because it might have inf value and culls uneccessary neighbors! or if you want ot use the map just add another condtion to avoid neighbors that have cost of inf
        // double distance = distance_[v_index][u_index];

        double distance = (tree_.at(v_index)->getStateVlaue() - tree_.at(u_index)->getStateVlaue()).norm(); // for cullNeighbor we don't use the distance map because that map sometimes has inf in it!
        // Check if the distance is greater than the neighborhood radius
        // and u is not the parent of v

        if ( distance > neighborhood_radius_ &&
            tree_[v_index]->getParentIndex() != u_index) {
            // Remove u from Nr_out of v
            it = Nr_out_[v_index].erase(it); 

            // Remove v from Nr_in of u
            auto& nr_in_u = Nr_in_[u_index];
            auto it_u = nr_in_u.find(v_index);
            if (it_u != nr_in_u.end()) {
                nr_in_u.erase(it_u);
            }
        } else {
            ++it;
        }
    }
}

void RRTX::makeParentOf(int child_index, int parent_index) {
    // Remove child from its current parent's children
    int current_parent_index = tree_[child_index]->getParentIndex();
    if (current_parent_index != -1) {
        auto& children = tree_[current_parent_index]->getChildrenIndices();
        auto it = std::find(children.begin(), children.end(), child_index);
        if (it != children.end()) {
            children.erase(it);
        }
    }

    // Update the parent of the child
    tree_[child_index]->setParentIndex(parent_index);

    // Add child to the new parent's children
    if (parent_index != -1) {
        tree_[parent_index]->getChildrenIndices().push_back(child_index);
    }
}

void RRTX::verifyQueue(int v_index) {
    double min_key = std::min(tree_[v_index]->getCost(), tree_[v_index]->getLMC());
    double g_value = tree_[v_index]->getCost();
    QueueElement new_element = {min_key, g_value, v_index};

    if (inconsistency_queue_.contains(v_index)) {
        inconsistency_queue_.update(v_index, new_element);
    } else {
        inconsistency_queue_.add(new_element);
    }
}


void RRTX::removeObstacle(const std::vector<int>& removed_samples) {
    for (int sample_index : removed_samples) {
        // samples_in_obstacles_.erase(sample_index);
        std::unordered_set<int> all_neighbors;
        // Add N0_in neighbors
        all_neighbors.insert(N0_in_[sample_index].begin(), N0_in_[sample_index].end());
        // Add N0_out neighbors
        all_neighbors.insert(N0_out_[sample_index].begin(), N0_out_[sample_index].end());
        // Add Nr_in neighbors
        all_neighbors.insert(Nr_in_[sample_index].begin(), Nr_in_[sample_index].end());
        // Add Nr_out neighbors
        all_neighbors.insert(Nr_out_[sample_index].begin(), Nr_out_[sample_index].end());

        // Recalculate distances to neighbors
        for (int neighbor_index : all_neighbors) {
            if (samples_in_obstacles_.find(neighbor_index)!=samples_in_obstacles_.end()) {
                continue;
            }
            double new_distance = (tree_[sample_index]->getStateVlaue() - tree_[neighbor_index]->getStateVlaue()).norm();
            distance_[sample_index][neighbor_index] = new_distance;
            distance_[neighbor_index][sample_index] = new_distance;
        }

        // Update the LMC for this sample
        updateLMC(sample_index);

        // If the sample is inconsistent, add it to the queue
        if (tree_[sample_index]->getLMC() != tree_[sample_index]->getCost()) {
            verifyQueue(sample_index);
        }
    }
}


void RRTX::addNewObstacle(const std::vector<int>& added_samples) {
    // Add the new obstacle samples to the obstacle set
    for (int sample_index : added_samples) {
        samples_in_obstacles_.insert(sample_index);
    }

    // Invalidate all edges connected to the obstacle samples
    for (int sample_index : added_samples) {
        // Get all neighbors of the sample (both incoming and outgoing)
        std::unordered_set<int> all_neighbors;
        all_neighbors.insert(N0_in_[sample_index].begin(), N0_in_[sample_index].end());
        all_neighbors.insert(N0_out_[sample_index].begin(), N0_out_[sample_index].end());
        all_neighbors.insert(Nr_in_[sample_index].begin(), Nr_in_[sample_index].end());
        all_neighbors.insert(Nr_out_[sample_index].begin(), Nr_out_[sample_index].end());

        // Invalidate edges connected to this sample
        for (int neighbor_index : all_neighbors) {
            // Invalidate the edge cost bidirectionally
            distance_[sample_index][neighbor_index] = std::numeric_limits<double>::infinity();
            distance_[neighbor_index][sample_index] = std::numeric_limits<double>::infinity();

            // If the neighbor is the parent of this sample, mark it as orphaned
            if (tree_[neighbor_index]->getParentIndex() == sample_index) {
                verifyOrphan(neighbor_index);
            }

            // If this sample is the parent of the neighbor, mark the neighbor as orphaned
            if (tree_[sample_index]->getParentIndex() == neighbor_index) {
                verifyOrphan(sample_index);
            }
        }

        // If this sample is the parent of any node, mark those nodes as orphaned
        for (int child_index : tree_[sample_index]->getChildrenIndices()) {
            verifyOrphan(child_index);
        }

        // Invalidate the sample's cost and LMC
        tree_[sample_index]->setCost(std::numeric_limits<double>::infinity());
        tree_[sample_index]->setLMC(std::numeric_limits<double>::infinity());
        edge_length_[sample_index] = -std::numeric_limits<double>::infinity();
        
        // Remove the sample from the tree structure
        if (tree_[sample_index]->getParentIndex() != -1) {
            auto& parent_children = tree_[tree_[sample_index]->getParentIndex()]->getChildrenIndices();
            auto it = std::find(parent_children.begin(), parent_children.end(), sample_index);
            if (it != parent_children.end()) {
                parent_children.erase(it);
            }
        }
        tree_[sample_index]->setParentIndex(-1);
    }
}


void RRTX::verifyOrphan(int v_index) {
    if (inconsistency_queue_.contains(v_index)) {
        inconsistency_queue_.remove(v_index);
    }
    Vc_T_.insert(v_index);  // Add v to Vc_T
}

void RRTX::propagateDescendants() {
    // Propagate orphan status to descendants
    std::queue<int> to_process;

    // Initialize the queue with nodes in Vc_T_
    for (int v_index : Vc_T_) {
        to_process.push(v_index);
    }

    // Process all descendants of nodes in Vc_T_
    while (!to_process.empty()) {
        int v_index = to_process.front();
        to_process.pop();

        // Add all children of v_index to Vc_T_ and the queue
        for (int child_index : tree_[v_index]->getChildrenIndices()) {
            if (Vc_T_.find(child_index) == Vc_T_.end()) {
                Vc_T_.insert(child_index);
                to_process.push(child_index);
            }
        }
    }

    // Step 2: Update costs and verify queue for affected nodes
    for (int v_index : Vc_T_) {
        // Merge N0_out and Nr_out into a single set (N+(v))
        std::unordered_set<int> outgoing_neighbors = N0_out_[v_index];
        outgoing_neighbors.insert(Nr_out_[v_index].begin(), Nr_out_[v_index].end());

        // Include the parent of v (p+_T(v)) in the set of neighbors
        int parent_index = tree_[v_index]->getParentIndex();
        if (parent_index != -1) {
            outgoing_neighbors.insert(parent_index);
        }

        // Process all neighbors in (N+(v) ∪ {p+_T(v)}) \ Vc_T
        for (int u_index : outgoing_neighbors) {
            if (Vc_T_.find(u_index) == Vc_T_.end()) {
                // Set g(u) to infinity
                tree_[u_index]->setCost(std::numeric_limits<double>::infinity());
                edge_length_[u_index] = -std::numeric_limits<double>::infinity();
                // Verify the queue for u
                verifyQueue(u_index);
            }
        }
    }

    // Step 3: Reset orphaned nodes
    for (int v_index : Vc_T_) {
        // Set g(v) and lmc(v) to infinity
        tree_[v_index]->setCost(std::numeric_limits<double>::infinity());
        tree_[v_index]->setLMC(std::numeric_limits<double>::infinity());
        edge_length_[v_index] = -std::numeric_limits<double>::infinity();

        // Remove v_index from its parent's children list
        int parent_index = tree_[v_index]->getParentIndex();
        if (parent_index != -1) {
            auto& children = tree_[parent_index]->getChildrenIndices();
            auto it = std::find(children.begin(), children.end(), v_index);
            if (it != children.end()) {
                children.erase(it);
            }
        }

        // Reset parent of v_index
        tree_[v_index]->setParentIndex(-1);
    }

    // Clear Vc_T_ after processing
    Vc_T_.clear();
}


void RRTX::visualizeTree() {
    std::vector<Eigen::VectorXd> nodes;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    double goal_node_cost = tree_.at(vbot_index_)->getCost();
    
    // Create a set to store valid nodes based on cost
    std::unordered_set<int> valid_node_indices;

    // Collect valid nodes
    for (size_t i = 0; i < tree_.size(); ++i) {
        if (tree_[i]->getCost() <= goal_node_cost) {
            nodes.push_back(tree_[i]->getStateVlaue());
            valid_node_indices.insert(i);
        }
    }

    // Generate edges only for valid nodes
    for (int index : valid_node_indices) {
        int parent_index = tree_[index]->getParentIndex();
        if (parent_index != -1) {
            edges.emplace_back(tree_.at(parent_index)->getStateVlaue(), tree_.at(index)->getStateVlaue());
        }
    }

    // Visualize nodes and edges
    // visualization_->visualizeNodes(nodes);
    visualization_->visualizeEdges(edges);
}



void RRTX::visualizePath(std::vector<int> path_indices) {
    std::vector<Eigen::VectorXd> nodes;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;

    // Add nodes to the list
    for (const auto& index : path_indices) {
        nodes.push_back(tree_.at(index)->getStateVlaue());
    }

    // Add edges to the list
    for (const auto& index : path_indices) {
        int parent_index = tree_.at(index)->getParentIndex();
        if (parent_index != -1) {
            edges.emplace_back(tree_.at(parent_index)->getStateVlaue(), tree_.at(index)->getStateVlaue());
        }
    }

    // Use the visualization class to visualize nodes and edges
    // visualization_->visualizeNodes(nodes);
    visualization_->visualizeEdges(edges,"map","0.0,1.0,0.0");
}