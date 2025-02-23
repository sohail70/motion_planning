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
    // int idx = robot_state_index_;
    // std::vector<int> path;
    // while (idx != -1) {
    //     path.push_back(idx);
    //     idx = tree_.at(idx)->getParentIndex();
    // }
    // return path;
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
    // for (int i = 0 ; i < num_of_samples_; i++) {  // BUT THIS DOESNT CREATE A TREE NODE FOR START AND GOAL !!!
    //     tree_.push_back(std::make_unique<TreeNode>(statespace_->sampleUniform(lower_bound_ , upper_bound_)));
    // }
    std::cout << "--- \n";
    setGoal(problem_->getGoal()); //robots current position

    // std::cout << "KDTree: \n\n";
    if (use_kdtree == true) {
        // Put all the points at once because fmtx doesnt need incremental addition
        // std::cout<<statespace_->getSamplesCopy()<<"\n";
        kdtree_->addPoints(statespace_->getSamplesCopy());
        // Build the tree all at once after we fill the data_ in the KDTree
        kdtree_->buildTree();
        // kdtree_->radiusSearch(tree_.at(0)->getStateVlaue(), 10);
        // std::cout << "---- \n";
        // kdtree_->knnSearch(tree_.at(0)->getStateVlaue(), 10);
    }

    /////////////////////////SETTING UP DS//////////////
    vbot_index_ = 1;
    tree_.at(0)->setCost(0);
    tree_.at(0)->setLMC(0);
    v_indices_.insert(0);
    // v_indices_.insert(1); // TODO: whihc one to add to the v_indices which is the big V in the rrtx paper?
    ///////////////////Neighborhood Radius////////////////////////////////
    dimension_ = statespace_->getDimension();
    int d = dimension_;
    double mu = std::pow(problem_->getUpperBound() - problem_->getLowerBound() , 2);
    double zetaD = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
    gamma_ = 2 * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d);

    ////////////////// Test Neighborhood////////////////////////
    // near(0); // Create
    // std::cout << "-------- \n";
    // auto indices = near(0);  // Get from the cache
    // for  (auto index : indices) {
    //     std::cout << index.index <<",";
    // }


    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    // std::cout << "---\n";


}



void RRTX::plan() {

    auto start = std::chrono::high_resolution_clock::now();

    // while (nodes_[vbot_index_].parent != vgoal_index_ and sample_counter < num_of_samples_) {
    
    // if (obs_checker_->obstacles_changed()) {
    //     handleObstacleChanges();
    // }
    
    // Sample new node
    // if (cap_samples_==true && sample_counter < num_of_samples_) { // TODO: later when you add the robot you can put the condtion of the while loop here and we use the while true outside because we want it to always work to update the gazebo obstale positions
    while ( sample_counter < num_of_samples_) { // TODO: later when you add the robot you can put the condtion of the while loop here and we use the while true outside because we want it to always work to update the gazebo obstale positions
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
        // if v is not in obstalce then do this: // TODO: implement later because obs checker need a function to check if a point in on obstalce!
        // Extend and rewiree
        if (obs_checker_->isObstacleFree(v))
            extend(v);

        int current_index = tree_.size() -1;
        if (v_indices_.find(current_index) != v_indices_.end()) {
            rewireNeighbors(current_index);
            reduceInconsistency();
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by while loop: " << duration.count() << " milliseconds\n";
}


void RRTX::extend(Eigen::VectorXd v) {
    // Add the new node to the tree
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
        // tree_.pop_back();
        // kdtree_->removePoint(v);
        return;
    }

    // Add the new node to the set of valid nodes
    v_indices_.insert(current_index);

    // Update neighbor relationships
    for (int u_index : neighbors) {
        if (u_index==current_index) {
            continue;
        }
        const Eigen::VectorXd& u_state = tree_.at(u_index)->getStateVlaue();
        // TODO: for now i don't distinguish between bidirectional paths between v,u and u,v
        bool is_path_free = obs_checker_->isObstacleFree(v, u_state); 

        if (is_path_free) {
            // Update original and running neighbors
            N0_out_[current_index].insert(u_index);  // N0_out for v
            Nr_in_[u_index].insert(current_index);    // Nr_in for u

            Nr_out_[u_index].insert(current_index);   // Nr_out for u
            N0_in_[current_index].insert(u_index);    // N0_in for v

            // Store distances
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
    for (int u_index : candidate_indices) {
        const Eigen::VectorXd& u_state = tree_.at(u_index)->getStateVlaue();

        // Compute trajectory and distance between v and u
        double distance = (v_state - u_state).norm();
        bool is_path_free = obs_checker_->isObstacleFree(v_state, u_state); //TODO: later
        // bool is_path_free = true;

        // Check conditions for valid parent
        if (distance <= neighborhood_radius_ && 
            tree_.at(current_index_)->getLMC() > distance + tree_.at(u_index)->getLMC() &&
            is_path_free) {
            
            // Update best parent if this candidate is better
            double total_cost = distance +tree_.at(u_index)->getLMC();
            if (total_cost < min_cost) {
                min_cost = total_cost;
                best_parent = u_index;
            }
        }
    }

    // If a valid parent is found, update the node's parent and lmc
    if (best_parent != -1) {
        tree_.at(current_index_)->setParentIndex(best_parent);
        tree_.at(current_index_)->setLMC(min_cost);
        tree_.at(best_parent)->setChildrenIndex(current_index_); // i don't know why rrtx pseudo code updates the children in the extend
        // TODO: should i update the gvalue (costtoroot) here or wait so that in the rewire neighbor and reduce inconsistency to do it!?
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

            // Get the trajectory cost from the distance dictionary
            double trajectory_cost = std::numeric_limits<double>::infinity();
            if (distance_.count(v_index) && distance_[v_index].count(u_index)) {
                trajectory_cost = distance_[v_index][u_index];
            }

            // Check if the path through v is better
            if (tree_[u_index]->getLMC() > trajectory_cost + tree_[v_index]->getLMC()) {
                // Update the look-ahead cost (lmc) of u
                tree_[u_index]->setLMC(trajectory_cost + tree_[v_index]->getLMC());

                // Make v the new parent of u
                makeParentOf(u_index, v_index);

                // If u is inconsistent, add it to the queue
                if (tree_[u_index]->getCost() - tree_[u_index]->getLMC() > epsilon_) {
                    verifyQueue(u_index);
                }
            }
        }
    }
}

// TODO: put the v_bot in Q in the or sections!
void RRTX::reduceInconsistency() {
    while (!inconsistency_queue_.empty() 
            // &&
        //    (inconsistency_queue_.top().min_key < tree_[vbot_index_]->getCost() ||
            // tree_[vbot_index_]->getLMC() != tree_[vbot_index_]->getCost() ||
            // tree_[vbot_index_]->getCost() == std::numeric_limits<double>::infinity())
    ){
    // while (!inconsistency_queue_.empty() ) {
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
    // const double gamma = 2.0 * pow((1 + 1.0/d) * (volume / zetaD), 1.0/d);
    // neighborhood_radius_ = factor * gamma * std::pow(std::log(statespace_->getNumStates()) / statespace_->getNumStates(), 1.0 / d);
    auto rad = factor * gamma_ * pow(log(tree_.size()) / tree_.size(), 1.0/dimension_);
    return std::min(rad, delta);;

}


// TODO: This should only be on dynamic obstacles! --> But how do we know maybe some static obstalce become dynamic! --> not motion planning concern maybe some method to classify static and dynamic obstalces!
std::unordered_set<int> RRTX::findSamplesNearObstacles(
    const std::vector<Eigen::Vector2d>& obstacles, 
    double obstacle_radius
) {
    std::unordered_set<int> conflicting_samples;
    
    for (const auto& obstacle : obstacles) {
        // Query samples within obstacle radius (5 units)
        auto sample_indices = kdtree_->radiusSearch(obstacle, obstacle_radius);
        conflicting_samples.insert(sample_indices.begin(), sample_indices.end());
    }
    
    return conflicting_samples;
}




void RRTX::updateObstacleSamples(const std::vector<Eigen::Vector2d>& obstacles) {
    // Similar obstacle sampling to FMTX but with RRTX propagation
    auto current = findSamplesNearObstacles(obstacles, 2.2*5.0); // TODO: i don't think its correct to scale this but its necessary to (it needs to be integrated with max length) --> its not correct in a sense that the scaled onces shoudlnt go into the samples in obstalces i guess because i avoid them in the main while loop --> weirdly it works but i'll take a look later!


    // std::vector<Eigen::VectorXd> positions;
    // for (const auto& y: current) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(y)->getStateVlaue();
    //     positions.push_back(vec);
    // }
    // std::string color_str = "1.0,0.0,0.0"; // Blue color
    // visualization_->visualizeNodes(positions,"map",color_str);



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

    samples_in_obstacles_ = std::move(current);



    // std::vector<Eigen::VectorXd> nodes;
    // for (const auto& sample_ : samples_in_obstacles_) {
    //     nodes.push_back(tree_[sample_]->getStateVlaue());
    // }
    // visualization_->visualizeNodes(nodes);



    // Handle added and removed samples
    if (!added.empty()) {
        addNewObstacle(added);
        propagateDescendants();
        // verifyQueue();
    }
    if (!removed.empty()) {
        removeObstacle(removed);
    }

    reduceInconsistency();
    // Update the set of samples in obstacles
}

void RRTX::handleObstacleChanges() {
    // Implementation similar to FMTX's updateObstacleSamples but with:
    // - propagateDescendants()
    // - verifyQueue() calls
    // - Obstacle edge management
}

// Updated RRTX implementation with fixed TreeNode usage

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

        // Get the distance between v and u
        double distance = distance_[v_index][u_index];
        // Check if the distance is greater than the neighborhood radius
        // and u is not the parent of v

        // or if you don't wwant to check infinity you can directly find the distance without using the distnace map
        if (distance != std::numeric_limits<double>::infinity() &&
            distance > neighborhood_radius_ &&
            tree_[v_index]->getParentIndex() != u_index) {
            // Remove u from Nr_out of v
            it = Nr_out_[v_index].erase(it); //TODO: this line creates sub-optimal solutions! right after removeObstalce happens!

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
// This is wrong! it should also be able to update it instead of just add it! we should change the structure!
// void RRTX::verifyQueue(int u_index) {
//     double min_key = std::min(tree_[u_index]->getCost(), tree_[u_index]->getLMC());
//     inconsistency_queue_.emplace(min_key, u_index);
// }

void RRTX::verifyQueue(int v_index) {
    double min_key = std::min(tree_[v_index]->getCost(), tree_[v_index]->getLMC());
    double g_value = tree_[v_index]->getCost();
    QueueElement new_element = {min_key, g_value, v_index};

    if (inconsistency_queue_.contains(v_index)) {
        inconsistency_queue_.update(v_index, new_element);
        // std::cout << "UPDATE QUEUE \n";
    } else {
        inconsistency_queue_.add(new_element);
        // std::cout << "ADD QUEUE \n";
    }
}


void RRTX::removeObstacle(const std::vector<int>& removed_samples) {
    for (int sample_index : removed_samples) {
        // Mark the sample as no longer being in an obstacle
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
    // Step 1: Add the new obstacle samples to the obstacle set
    for (int sample_index : added_samples) {
        samples_in_obstacles_.insert(sample_index);
    }

    // Step 2: Invalidate all edges connected to the obstacle samples
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

// TODO: how to delete that from the Q !!!???? i skipped it in reduceinconsisteny!
// void RRTX::verifyOrphan(int v_index) {
//     Vc_T_.insert(v_index);
// }

void RRTX::verifyOrphan(int v_index) {
    if (inconsistency_queue_.contains(v_index)) {
        inconsistency_queue_.remove(v_index);
        // std::cout << "DELETE FROM QUEUE \n";
    }
    Vc_T_.insert(v_index);  // Add v to Vc_T
}

void RRTX::propagateDescendants() {
    // Step 1: Propagate orphan status to descendants
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

    // Add nodes to the list
    for (const auto& tree_node : tree_) {
        nodes.push_back(tree_node->getStateVlaue());
    }

    // Add edges to the list
    for (const auto& tree_node : tree_) {
        int parent_index = tree_node->getParentIndex();
        if (parent_index != -1) {
            edges.emplace_back(tree_.at(parent_index)->getStateVlaue(), tree_node->getStateVlaue());
        }
    }

    // Use the visualization class to visualize nodes and edges
    // visualization_->visualizeNodes(nodes);
    visualization_->visualizeEdges(edges);
}
