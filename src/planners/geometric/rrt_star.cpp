#include "motion_planning/planners/geometric/rrtx.hpp"

RRTX::RRTX(std::unique_ptr<StateSpace> statespace, 
    std::unique_ptr<ProblemDefinition> problem_def,
    std::shared_ptr<ObstacleChecker> obs_checker): statespace_(std::move(statespace)), problem_(std::move(problem_def)), obs_checker_(obs_checker){
        std::cout<<"RRTX constructor \n";
}


void RRTX::setStart(const Eigen::VectorXd& start) {
    robot_state_index_ = statespace_->getNumStates();
    tree_.push_back(std::make_shared<TreeNode>(statespace_->addState(start)));
    std::cout << "FMTX: Start node created on Index: " << robot_state_index_ << "\n";
}
void RRTX::setGoal(const Eigen::VectorXd& goal) {
    root_state_index_ = statespace_->getNumStates();
    tree_.push_back(std::make_shared<TreeNode>(statespace_->addState(goal)));
    std::cout << "FMTX: Goal node created on Index: " << root_state_index_ << "\n";
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

    std::cout << "FMTX setup complete: num_of_samples=" << num_of_samples_
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
    tree_.at(0)->setCost(0);
    tree_.at(0)->setLMC(0);
    v_indices_.insert(0);
    v_indices_.insert(1); // TODO: whihc one to add to the v_indices which is the big V in the rrtx paper?
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
    size_t sample_counter = 0;
    double delta = 10.0; // Step size limit
    static size_t sample_counter = 0;
    // while (nodes_[vbot_index_].parent != vgoal_index_ and sample_counter < num_of_samples_) {
    while (sample_counter < num_of_samples_) {
        neighborhood_radius_ = shrinkingBallRadius();
        
        // if (obs_checker_->obstacles_changed()) {
        //     handleObstacleChanges();
        // }
        
        // Sample new node
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

        // if v is not in obstalce then do this: // TODO: implement later because obs checker need a function to check if a point in on obstalce!
        // Extend and rewiree
        extend(v);

        int current_index = tree_.size() -1;
        if (v_indices_.find(current_index) != v_indices_.end()) {
            rewireNeighbors(current_index);
            // reduce_inconsistency();
        }
       
    }

    visualizeTree();
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
        // bool is_path_free = obs_checker_->isPathFree(v, u_state); // TODO implement later because you need to add another function or an overload to see if a point is in obstalces or not!
        bool is_path_free = true;

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
        // bool is_path_free = obs_checker_->isObstacleFree(v_state, u_state); //TODO: later
        bool is_path_free = true;

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

void RRTX::reduceInconsistency() {
    while (!inconsistency_queue_.empty() &&
           (inconsistency_queue_.top().first < tree_[vbot_index_]->getCost() ||
            tree_[vbot_index_]->getLMC() != tree_[vbot_index_]->getCost() ||
            tree_[vbot_index_]->getCost() == std::numeric_limits<double>::infinity())) {
        auto [_, v_index] = inconsistency_queue_.top();
        inconsistency_queue_.pop();

        if (tree_[v_index]->getCost() - tree_[v_index]->getLMC() > epsilon_) {
            updateLMC(v_index);
            rewireNeighbors(v_index);
            tree_[v_index]->setCost(tree_[v_index]->getLMC());
        }
    }
}


double RRTX::shrinkingBallRadius() const {
    double factor = 2.0;
    // const double gamma = 2.0 * pow((1 + 1.0/d) * (volume / zetaD), 1.0/d);
    // neighborhood_radius_ = factor * gamma * std::pow(std::log(statespace_->getNumStates()) / statespace_->getNumStates(), 1.0 / d);
    auto rad = factor * gamma_ * pow(log(tree_.size()) / tree_.size(), 1.0/dimension_);
    return rad;

}

void RRTX::updateObstacles(const std::vector<Eigen::Vector2d>& new_obstacles) {
    // Similar obstacle sampling to FMTX but with RRTX propagation
    current_obstacles_ = new_obstacles;
    handleObstacleChanges();
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

void RRTX::propagateDescendants(int v_index) {
    // std::queue<int> queue;
    // queue.push(v_index);
    
    // while (!queue.empty()) {
    //     int current = queue.front();
    //     queue.pop();
        
    //     auto& current_node = tree_[current];
        
    //     // Reset costs for affected nodes
    //     current_node->setCost(std::numeric_limits<double>::infinity());
    //     current_node->setLMC(std::numeric_limits<double>::infinity());
        
    //     // Add to verification queue
    //     verifyQueue(current);
        
    //     // Propagate to children
    //     for (int child : current_node->getChildrenIndices()) {
    //         queue.push(child);
    //     }
        
    //     // Remove from parent's children list if necessary
    //     int parent = current_node->getParentIndex();
    //     if (parent != -1) {
    //         auto& parent_children = tree_[parent]->getChildrenIndices();
    //         parent_children.erase(
    //             std::remove(parent_children.begin(), parent_children.end(), current),
    //             parent_children.end()
    //         );
    //     }
        
    //     // Clear parent relationship
    //     current_node->setParentIndex(-1);
    // }
}


void RRTX::cullNeighbors(int v_index) {
    // Cull incoming neighbors (Nr_in)
    auto& nr_in = Nr_in_[v_index];
    for (auto it = nr_in.begin(); it != nr_in.end();) {
        if (distance_[*it][v_index] > neighborhood_radius_ &&
            tree_[*it]->getParentIndex() != v_index) {
            it = nr_in.erase(it);
        } else {
            ++it;
        }
    }

    // Cull outgoing neighbors (Nr_out)
    auto& nr_out = Nr_out_[v_index];
    for (auto it = nr_out.begin(); it != nr_out.end();) {
        if (distance_[v_index][*it] > neighborhood_radius_ &&
            tree_[v_index]->getParentIndex() != *it) {
            it = nr_out.erase(it);
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

void RRTX::verifyQueue(int u_index) {
    double priority = std::min(tree_[u_index]->getCost(), tree_[u_index]->getLMC());
    inconsistency_queue_.emplace(priority, u_index);
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
    visualization_->visualizeEdges(edges,"map","1.0,0.0,0.0");
}