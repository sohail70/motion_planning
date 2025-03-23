// Copyright 2025 Soheil E.nia
#include "motion_planning/planners/geometric/fmtx2.hpp"
std::string getRandomColor() {
    // Seed the random number generator
    std::srand(std::time(nullptr));

    // Generate random values for RGB between 0.0 and 1.0
    float r = static_cast<float>(std::rand()) / RAND_MAX;
    float g = static_cast<float>(std::rand()) / RAND_MAX;
    float b = static_cast<float>(std::rand()) / RAND_MAX;

    // Convert to string
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << r << "," << g << "," << b;
    return ss.str();
}
FMTX::FMTX(std::unique_ptr<StateSpace> statespace ,std::shared_ptr<ProblemDefinition> problem_def, std::shared_ptr<ObstacleChecker> obs_checker) :  statespace_(std::move(statespace)), problem_(problem_def), obs_checker_(obs_checker),v_open_heap_(50) {
    std::cout<< "FMTX Constructor \n";

}

void FMTX::clearPlannerState() {
    // Clear all critical variables
    start_.reset();
    goal_.reset();
    path_.clear();

    // for (auto& node : tree_) {
    //     node.reset();  // Explicitly reset each shared_ptr
    // }
    tree_.clear();  // Clear the vector

    // Reset the StateSpace
    statespace_->reset();

    // if(kdtree_)
    //     kdtree_->clearData();
    kdtree_.reset();

    v_open_heap_.clear();

    // samples_in_obstacles_.clear();
    // samples_in_obstacles_2_.clear();
    // inflated_samples_.clear();

    // edge_length_.clear();
    max_length_edge_ind = -1;
    max_length = -std::numeric_limits<double>::infinity();

    root_state_index_ = -1;
    robot_state_index_ = -1;

}


void FMTX::setup(const Params& params, std::shared_ptr<Visualization> visualization) {

    auto start = std::chrono::high_resolution_clock::now();
    clearPlannerState();

    visualization_ = visualization;


    num_of_samples_ = params.getParam<int>("num_of_samples");
    partial_update = params.getParam<bool>("partial_update");
    use_heuristic= params.getParam<bool>("use_heuristic");
    partial_plot = params.getParam<bool>("partial_plot");
    obs_cache = params.getParam<bool>("obs_cache");

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
    for (int i = 0 ; i < num_of_samples_; i++) {  // BUT THIS DOESNT CREATE A TREE NODE FOR START AND GOAL !!!
        auto node = std::make_unique<FMTXNode>(statespace_->sampleUniform(lower_bound_ , upper_bound_),tree_.size());
        node->in_unvisited_ = true;
        tree_.push_back(std::move(node));
    }
    std::cout << "--- \n";
    setGoal(problem_->getGoal());


    std::cout << "KDTree: \n\n";
    if (use_kdtree == true) {
        // Put all the points at once because fmtx doesnt need incremental addition
        kdtree_->addPoints(statespace_->getSamplesCopy());
        // Build the tree all at once after we fill the data_ in the KDTree
        kdtree_->buildTree();

    }


    ///////////////////Neighborhood Radius////////////////////////////////
    int d = statespace_->getDimension();
    double mu = std::pow(problem_->getUpperBound() - problem_->getLowerBound() , 2);
    double zetaD = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
    double gamma = 2 * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d);
    double factor = 2.0;
    neighborhood_radius_ = factor * gamma * std::pow(std::log(statespace_->getNumStates()) / statespace_->getNumStates(), 1.0 / d);
    // neighborhood_radius_ = 5.0;
    std::cout << "Computed value of rn: " << neighborhood_radius_ << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "---\n";





}

double FMTX::heuristic(int current_index) {
    Eigen::VectorXd current_position = tree_.at(current_index)->getStateVlaue();
    Eigen::VectorXd goal_position = tree_.at(robot_state_index_)->getStateVlaue();
    return (goal_position-current_position).norm();
}

bool FMTX::isValidYnear(int index, 
                        const std::unordered_set<int>& v_open_set, 
                        const std::vector<std::unordered_set<int>>& invalid_connections, 
                        int xIndex, 
                        bool use_heuristic) {
    bool inOpenSet = (v_open_set.find(index) != v_open_set.end());
    if (!use_heuristic) {
        return inOpenSet; // Early exit if heuristic is not used
    }
    
    // Check if the pair (xIndex, index) exists in invalid_connections
    bool notInInvalidNeighbors = (invalid_connections.at(xIndex).find(index) == invalid_connections.at(xIndex).end());
    // bool notInInvalidNeighbors = true;
    
    return inOpenSet && notInInvalidNeighbors;
}


void FMTX::plan() {
    // std::cout<<"SIZE: "<<v_open_heap_.size()<<"\n";

    // std::vector<Eigen::VectorXd> positions2;
    // for (const auto& y: v_open_set_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(y)->getStateVlaue();
    //     positions2.push_back(vec);
    // }
    // // std::string color_str = "0.0,0.0,1.0"; // Blue color
    // visualization_->visualizeNodes(positions2,"map");

    // std::vector<Eigen::VectorXd> positions3;
    // for (const auto& y: v_unvisited_set_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(y)->getStateVlaue();
    //     positions3.push_back(vec);
    // }
    // std::string color_str = "0.0,0.0,1.0"; // Blue color
    // visualization_->visualizeNodes(positions3,"map",color_str);

    // std::unordered_map<std::pair<int, int>, bool, pair_hash> obstacle_check_cache;
    int uncached = 0;
    int cached = 0;
    int checks = 0;






    // while (!v_open_heap_.empty()) {
    // while (!v_open_heap_.empty() && (partial_update ? v_open_heap_.top().index != robot_state_index_ : true)) {
    // while (!v_open_heap_.empty() && v_unvisited_set_.find(robot_state_index_) != v_unvisited_set_.end() ) {
    while (
            !v_open_heap_.empty() &&
           (partial_update ? (v_open_heap_.top().min_key < robot_node_->getCost() ||
            robot_node_->getCost() == INFINITY ||
            robot_node_->in_unvisited_==true || // do we need this ? cost inf and unvisted means the same thing
            robot_node_->in_queue_==true)  : true  )
          ) {
        auto top_element = v_open_heap_.top();
        double cost = top_element.min_key;
        int zIndex = top_element.index;
        FMTXNode* z = tree_[top_element.index].get();
        // if (partial_update == true && (zIndex==robot_state_index_ || z->getCost() > robot_node_->getCost())){
        //     /*
        //         preserve the v open heap nodes! no need to clear it because we might use them on the next pass!
        //     */
        //     break;
        // }
        v_open_heap_.pop();

        near(zIndex);
        // for (const auto& [xIndex, cost_to_neighbor]: zNeighborsInfo) {
        for (const auto& [x, cost_to_neighbor] : z->neighbors()) {
            int xIndex = x->getIndex();
            /*
                This trick actually enables us to only track changes in the update obstalce otherwise we have to use the 
                current nodes instead of added in the handle add obstalce or the other way is to check all the edges around 
                added obstalce for obstalce

                I also added this feature to rrtx so we have a fair comparison between these two!
            */
            if (samples_in_obstacles_.find(xIndex) != samples_in_obstacles_.end())
                continue;

            
            if (x->in_unvisited_==true || x->getCost() > (z->getCost() + cost_to_neighbor.distance ) ){

                near(xIndex);
                double min_cost = std::numeric_limits<double>::infinity();
                FMTXNode* best_neighbor_node = nullptr;
                double best_edge_length = 0.0;

                // Single pass through neighbors to both filter and find minimum
                for (const auto& [neighbor, dist] : x->neighbors()) {
                    // if(use_heuristic==true && x->blocked_best_neighbors.count(neighbor->getIndex()) > 0)
                        // continue;
                    if (neighbor->in_queue_) {
                        const double total_cost = neighbor->getCost() + dist.distance;
                        if (total_cost < min_cost) {
                            min_cost = total_cost;
                            best_neighbor_node = neighbor;
                            best_edge_length = dist.distance;
                        }
                    }
                }

                if (!best_neighbor_node) {
                    continue;
                }

                int best_neighbor_index = best_neighbor_node->getIndex();


                bool obstacle_free;
                // // Create a key for the cache
                // if (obs_cache == true) {
                //     // Create a key for the cache
                //     auto edge_key = (best_neighbor_index < xIndex) ? std::make_pair(best_neighbor_index, xIndex) : std::make_pair(xIndex, best_neighbor_index);

                //     // Check if the obstacle check result is already in the cache
                //     if (obstacle_check_cache.find(edge_key) != obstacle_check_cache.end()) {
                //         obstacle_free = obstacle_check_cache[edge_key];
                //         cached++;
                //     } else {
                //         // Perform the obstacle check and store the result in the cache
                //         obstacle_free = obs_checker_->isObstacleFree(x->getStateVlaue() , best_neighbor_node->getStateVlaue());
                //         obstacle_check_cache[edge_key] = obstacle_free;
                //         uncached++;
                //     }
                // }
                // else { //SOMETIMES BEST_NEIGHBOR_INDEX is -1 which means all the Ynear nodes has inf cost --> inf cost means its either samples_in_obstalces or vUnvisted or it was made to inf in the handleAddObstalce! --> THESE nodes shouldn't be in vOpen --> sometimes a node lingers in vOpen because of early exit so you have to erase it in handleAddObstalce or you have to check some ifs in Ynear node push_back!
                //     obstacle_free = obs_checker_->isObstacleFree(x->getStateVlaue() , best_neighbor_node->getStateVlaue());
                // }

                if (in_dynamic == false){
                    obstacle_free = obs_checker_->isObstacleFree(x->getStateVlaue() , best_neighbor_node->getStateVlaue());
                }
                else{
                    obstacle_free = true;
                    checks++;
                }


                if (obstacle_free) {
                    double newCost = min_cost;
                    if (newCost < x->getCost()) {
                        // if (use_heuristic==true) x->blocked_best_neighbors.clear(); // Well if x is connected then i don't care about neighbors that can't be connected
                        x->setCost(newCost);
                        double h_value = use_heuristic ? heuristic(xIndex) : 0.0;
                        double priorityCost = newCost + h_value;
                        QueueElement2 new_element = {priorityCost, xIndex};
                        if (x->in_queue_ == true){
                            v_open_heap_.update(xIndex, priorityCost);
                        } else{
                            v_open_heap_.add(new_element);
                            x->in_queue_ = true;
                        }
                        x->in_unvisited_ = false;

                        /*
                            //IMPORTANT CONCEPT --> sometimes x's parent is the same as best_neighbor_node so why did we end up here? because the parent's cost has changed because of my new condtion that i put there!!!!
                            setParent: 
                                so its either the same parent with new cost! --> due to the secoond if condtion --> so no need to change the children
                                or a new parent with new cost! --> we need to set the parent and children
                                or the same parent with the same cost!  ---> we can early return in the setParent
                        */

                        x->setParent(best_neighbor_node,best_edge_length); 
                        edge_length_[xIndex] = best_edge_length;

                    }
                }
                // else{
                //     /*
                //         Tracking the following set doesn put much performance issues as i tested with 20K nodes and the uupdate time was almost identical
                //         This set enables us to use A* heuristic if we want
                //     */
                //     if (use_heuristic==true) {
                //         x->blocked_best_neighbors.insert(best_neighbor_index);
                //     }
                // }
            }
        }

        z->in_queue_=false;
        z->in_unvisited_= false; // close the node if its not already --> i dont think i need this but let it be to be sure!
    }

    // std::cout<<"checks: "<< checks <<"\n";
}



std::vector<size_t> FMTX::getPathIndex() const {
    int idx = robot_state_index_;
    std::vector<size_t> path_index;
    while (idx != -1) {
        path_index.push_back(idx);
        idx = tree_.at(idx)->getParentIndex();
    }
    return path_index;
}

std::vector<Eigen::VectorXd> FMTX::getPathPositions() const {
    std::vector<Eigen::VectorXd> path_positions;

    // Add the robot's current position if robot_node_ is valid
    if (robot_node_ != nullptr) {
        path_positions.push_back(robot_position_);
    }

    // Start at the robot's node
    FMTXNode* current_node = robot_node_;

    // Traverse the tree from the robot's node to the root
    while (current_node != nullptr) {
        path_positions.push_back(current_node->getStateVlaue());

        // Move to the parent node
        current_node = current_node->getParent();
    }

    return path_positions;
}

void FMTX::setRobotIndex(const Eigen::VectorXd& robot_position) {
    robot_position_ = robot_position;

    const double MAX_SEARCH_RADIUS = 5.0; // Meters
    std::vector<size_t> nearest_indices = kdtree_->radiusSearch(robot_position, MAX_SEARCH_RADIUS);

    size_t best_index = std::numeric_limits<size_t>::max(); 
    double min_total_cost = std::numeric_limits<double>::max();
    FMTXNode* best_node = nullptr; 

    for (size_t index : nearest_indices) {
        auto node = tree_.at(index).get();
        if (node->getCost() == std::numeric_limits<double>::infinity()) continue;

        Eigen::VectorXd node_position = node->getStateVlaue(); // Fixed typo: getStateVlaue -> getStateValue
        double dx = node_position[0] - robot_position[0];
        double dy = node_position[1] - robot_position[1];
        double distance_to_node = std::hypot(dx, dy);

        double total_cost = distance_to_node + node->getCost();

        if (total_cost < min_total_cost) {
            min_total_cost = total_cost;
            best_index = index;
            best_node = node;
        }
    }

    if (best_index != std::numeric_limits<size_t>::max()) {
        robot_state_index_ = best_node->getIndex();
        robot_node_ = best_node; // Directly assign the raw pointer
        return;
    }

    bool keep_prev_state_ = false;
    if (robot_node_ && keep_prev_state_ == true) {
        std::cout << "No valid node found in neighborhood. Keeping previous robot_node_.\n";
        return;
    }
    if (robot_node_) {
        std::cout << "No valid node found in neighborhood. Setting to nearest unvisited node.\n";
        // so it must be on the vunvisted zones --> lets get the nearest vunvisted and then rely on plan function to reach there!
        std::vector<size_t> nearest_indices = kdtree_->knnSearch(robot_position, 1);
        int nearest = nearest_indices.empty() ? -1 : static_cast<int>(nearest_indices[0]);  
        robot_node_ = tree_.at(nearest).get();
        robot_state_index_ = robot_node_->getIndex();
        return;
    }

    robot_state_index_ = -1;
    robot_node_ = nullptr; 
    std::cout << "No valid node found and no previous robot_node_. Setting robot_node_ to nullptr.\n";
}

void FMTX::setStart(const Eigen::VectorXd& start) {
    root_state_index_ = statespace_->getNumStates();
    auto node = std::make_unique<FMTXNode>(statespace_->addState(start),tree_.size());
    node->setCost(0);
    node->in_queue_ = true;
    QueueElement2 new_element ={0,0};
    v_open_heap_.add(new_element);

    tree_.push_back(std::move(node));
    std::cout << "FMTX: Start node created on Index: " << robot_state_index_ << "\n";
}
void FMTX::setGoal(const Eigen::VectorXd& goal) {
    robot_state_index_ = statespace_->getNumStates();
    auto node = std::make_unique<FMTXNode>(statespace_->addState(goal),tree_.size());
    node->in_unvisited_ = true;
    robot_node_ = node.get(); // Management of the node variable above will be done by the unique_ptr i'll send to tree_ below so robot_node_ is just using it!
    tree_.push_back(std::move(node));
    std::cout << "FMTX: Goal node created on Index: " << root_state_index_ << "\n";
}

void FMTX::near(int node_index) {
    auto node = tree_[node_index].get();
    if (!node->neighbors().empty()) return;

    auto indices = kdtree_->radiusSearch(node->getStateVlaue(), neighborhood_radius_); //!!!!! WHEN KD TREE PROVIDES DISTANCE WHY DO YOU CALC DISTS AGAIN IN BELOW! --> ALSO DO THIS FOR RRTX
    for(int idx : indices) {
        if(idx == node->getIndex()) continue;
        FMTXNode* neighbor = tree_[idx].get();
        auto dist = (node->getStateVlaue() - neighbor->getStateVlaue()).norm();
        node->neighbors()[neighbor] = FMTxEdgeInfo{dist,dist};
    }
}

void FMTX::visualizeTree() {
    if (partial_plot==true) {
        std::vector<Eigen::VectorXd> nodes;
        std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
        double goal_node_cost = tree_.at(robot_state_index_)->getCost();
        
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
    else {
        std::vector<Eigen::VectorXd> nodes;
        std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    
        // Add nodes to the list
        for (const auto& tree_node : tree_) {
            nodes.push_back(tree_node->getStateVlaue());
        }
    
        // Add edges to the list
        for (const auto& tree_node : tree_) {
            auto parent = tree_node->getParent();
            if (parent) {
                edges.emplace_back(parent->getStateVlaue(), tree_node->getStateVlaue());
            }
        }
    
        // Use the visualization class to visualize nodes and edges
        // visualization_->visualizeNodes(nodes);
        visualization_->visualizeEdges(edges);
    }


}

void FMTX::visualizePath(std::vector<size_t> path_indices) {
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


void FMTX::visualizeSmoothedPath(const std::vector<Eigen::VectorXd>& shortest_path_) {
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

// TODO: This should only be on dynamic obstacles! --> But how do we know maybe some static obstalce become dynamic! --> not motion planning concern maybe some method to classify static and dynamic obstalces!
std::unordered_set<int> FMTX::findSamplesNearObstacles(
    const std::vector<Obstacle>& obstacles, 
    double max_length
) {
    std::unordered_set<int> conflicting_samples;
    for (const auto& obstacle : obstacles) {
        // auto sample_indices = kdtree_->radiusSearch(obstacle.position, 1.5 * obstacle.radius);
        auto sample_indices = kdtree_->radiusSearch(obstacle.position, std::sqrt(std::pow(obstacle.radius + obstacle.inflation , 2) + std::pow(max_length / 2.0, 2)));
        conflicting_samples.insert(sample_indices.begin(), sample_indices.end());
    }
    
    return conflicting_samples;
}

std::pair<std::unordered_set<int>, std::unordered_set<int>> FMTX::findSamplesNearObstaclesDual(
    const std::vector<Obstacle>& obstacles, 
    double scale_factor
) {
    std::unordered_set<int> conflicting_samples_inflated;
    std::unordered_set<int> conflicting_samples;

    for (const auto& obstacle : obstacles) {
        // Call the radiusSearchDual function
        auto [sample_indices1, sample_indices2] = kdtree_->radiusSearchDual(obstacle.position, scale_factor*obstacle.radius, obstacle.radius);

        // Insert the results into the unordered_sets
        conflicting_samples_inflated.insert(sample_indices1.begin(), sample_indices1.end());
        conflicting_samples.insert(sample_indices2.begin(), sample_indices2.end());
    }

    return {conflicting_samples_inflated, conflicting_samples};
}
void FMTX::updateObstacleSamples(const std::vector<Obstacle>& obstacles) {
    in_dynamic = true;

    // Calculating the max length when the max_length edge is updated or the obstalce is on the previous max_length edge!


    /*
        Now that im thinking about this the max_length's upper bound is neighborhood_radius_ in fmtx! this is not a rrt star based algorithm!
        I guess we don't need to track the max_edge! and we can easily use rn for this but for now i'll leave this as is!
    
    */
    max_length = neighborhood_radius_; // At first Static plan we don't have max_length --> either do this or do a static plan

    // if (edge_length_[max_length_edge_ind] != max_length) // This condition also triggeres the first calculation os It's okay
    // {
    //     auto max_it = std::max_element(edge_length_.begin() , edge_length_.end() ,[](const std::pair<int, double>& a , const std::pair<int, double>& b){
    //         return a.second < b.second;
    //     });
    //     max_length = max_it->second;
    //     max_length_edge_ind = max_it->first;
    //     // std::cout<<max_it->first << "  " << max_it->second <<" \n"; 

    // }

    // Find current samples in obstacles
    auto current = findSamplesNearObstacles(obstacles, max_length); // TODO: i don't think its correct to scale this but its necessary to (it needs to be integrated with max length) --> its not correct in a sense that the scaled onces shoudlnt go into the samples in obstalces i guess because i avoid them in the main while loop --> weirdly it works but i'll take a look later!
    // auto [current,current2] = findSamplesNearObstaclesDual(obstacles, 2.2); 

    // When you put this here it mean no update on the tree is gonna happens unless some obstalce change happens in the environment so don't just move the robot anywhere you like by grabbing it and expect the tree to react!
    if (current==samples_in_obstacles_) // TODO: I think this should be doen in gazeboObstalceChecker level not here! the obstacleChecker needs to be able to report if obstalces has changed.
        return; //because nothing has changed! 
    

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

    /*
        handling the problem of reconnecion to the inflation zone is not easy! you can't just keep track of the difference between added and current
        and you have to use current everytime in the handleAddedObstalceSamples because if you don't the added will only go forward (with a distance to the obstalce ofcourse)
        and the obstalce might move and end up on top of a edge but since the added is just a difference between previous and current iteration it doesnt cover the nodes on that edge
        so the timeline is this --> added --> then plan takes care of the nodes in the inflation zone --> obstalce move and end up on some long edge --> the added is far away and doesnt invalidate those long edge because it invalidated it in previous iteraion and moved on!
    */



    // Handle changes first. whic one should be first? doesnt matter! well TBH removed obstalce would put some node in vunvisted and also 
    if (!added.empty()) {
        handleAddedObstacleSamples(added);  // Only call if added has elements
    }
    if (!removed.empty()) {
        handleRemovedObstacleSamples(removed);  // Only call if removed has elements
    }


    
    // Update the tracked set
    // samples_in_obstacles_ = std::move(current2); // for reconnection in inflation zone acitvate the for loop with obstalce check above and uncomment this line als use the dual radius search
    samples_in_obstacles_ = std::move(current);







    // Whats the point of putting these in vUnvisted when they are on obstalce! BUT SHOULD I DO IT BEFORE THE PLAN OR AFTER THE PLAN?? WELL the samples_in_obstalces_ is used in the main while loop anyway!
    for (int idx : samples_in_obstacles_) {
        tree_[idx]->in_unvisited_ = false;
    }



    // std::vector<Eigen::VectorXd> positions;
    // for (const auto& y: v_unvisited_set_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(y)->getStateVlaue();
    //     positions.push_back(vec);
    // }
    // std::string color_str = "0.0,0.0,1.0"; // Blue color
    // visualization_->visualizeNodes(positions,"map",color_str);

    // std::vector<Eigen::VectorXd> positions2;
    // for (const auto& y: v_open_set_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(y)->getStateVlaue();
    //     positions2.push_back(vec);
    // }
    // // std::string color_str = "0.0,0.0,1.0"; // Blue color
    // visualization_->visualizeNodes(positions2,"map");
    

    // std::vector<Eigen::VectorXd> positions3;
    // for (const auto& y: samples_in_obstacles_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(y)->getStateVlaue();
    //     positions3.push_back(vec);
    // }
    // visualization_->visualizeNodes(positions3,"map");

}


std::unordered_set<int> FMTX::getDescendants(int node_index) {
    std::unordered_set<int> descendants;
    std::queue<FMTXNode*> queue;
    
    // Start with the initial node
    queue.push(tree_[node_index].get());
    
    while (!queue.empty()) {
        FMTXNode* current = queue.front();
        queue.pop();
        
        // Store the index in the result set
        descendants.insert(current->getIndex());
        
        // Process children through pointers
        for (FMTXNode* child : current->getChildren()) {
            queue.push(child);
        }
    }
    
    return descendants;
}
// std::unordered_set<int> FMTX::getDescendants(int node_index) {
//     std::unordered_set<int> descendants;
//     std::queue<FMTXNode*> queue;
//     std::unordered_set<FMTXNode*> processing; // Track nodes being processed
    
//     // Debugging variables
//     int cycle_counter = 0;
//     constexpr int MAX_CYCLE_WARNINGS = 5;
//     auto start_time = std::chrono::steady_clock::now();

//     FMTXNode* start_node = tree_[node_index].get();
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

//         FMTXNode* current = queue.front();
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
//         for (FMTXNode* child : children) {
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

void FMTX::handleAddedObstacleSamples(const std::vector<int>& added) {
    std::unordered_set<int> orphan_nodes;
    // Step 1: Identify orphan nodes (nodes now in obstacles)
    for (int idx : added) {
        // Mark the node and its descendants as orphans
        orphan_nodes.insert(idx);
        auto descendants = getDescendants(idx);
        orphan_nodes.insert(descendants.begin(), descendants.end());

    }

    /*
        one might ask why do you put orphan nodes into v_unvisited_set when you have a mechanism in the main loop to find these automatically?! 
        The reason is these help the finding of the v open nodes later in the update obstalce sample function
        If we only rely on that mechansim we can't find connections to other branches because we are blind to see other branches! like on the other side of the tree
        Imagine the one side of the plier and some nodes get better cost if they get connected to the other tip of the plier but since we didn't put the other side nodes into v open we never know!

        (side note: Also imagine if the the two tips of the the plier is far apart so you can't rely on the neighborhood raidus of one side to get to the other!)

        So that condtion in the main loop is just for one direction expansion and is good for the nodes that gor removed from the obstalce--> Although its a reasonable question here also to ask ourselves why its not the case
        for the remove obstlace to now know their v open at first!
        the difference between addObstalce and removeObstalce is adding and obstalce most certainly adds cost to orphan nodes
        but removing an obstlace most certainly reduces cost of the neighbor nodes! and reducing happens in the current branch and direction of the expansion that happens in dijkstra like (like fmtx) algorithm 
        so we don't need to worry about the other side of plier (per say!) because they are gonna connect to us! not us connecting to them (and by "us" i mean the current direction of the expansion)
    */
    // v_unvisited_set_.insert(orphan_nodes.begin() , orphan_nodes.end()); 
    for (auto node_index : orphan_nodes) { // we should do it here --> don't be greedy because if you put it down below , in the nested for loop you might put some neighbor in the vopen that would later in the first loop became vunvisited! --> bu you can find a way to put this in the above loop!
        tree_.at(node_index)->in_unvisited_ = true;
    }

    for (auto node_index : orphan_nodes) {
        auto node = tree_.at(node_index).get();
        near(node_index);
        for (const auto& [neighbor,dist] : node->neighbors()) {
            int index = neighbor->getIndex();
            /*
              IMPORTNAT NOTE: My assumption is we do not need to update the queue here because we only need to ADD to queue. 
              UPDATE IS WHEN A COST of node has changed and that happens only in the main plan function. here we only make the cost to inf and we removed. you may ask
              how can you be sure we don't have any vopen heap nodes left? in partial update false the vopen heap gets fully cleaned because of the while loop but in partial update true
              we early exit that loop so some vopen heap nodes are left! in this scenraio now imagine an obstalce is being added and at the worst case scenario it is being added to the region where there are already vopen heap nodes!
              the result of adding obstalce means two things! ---> some vclosed (conceptually i mean because i don't use vclose in my algorithm!) nodes become vunvisted or some vopen nodes become vunvisted! and the previously vunvisited due to partial update
              stays in vunvisted! mind that the cost of these per say messeup nodes will become infinity or stay infinity
              for expansion we need CORRECT vopen heap nodes! (by correct i mean the correct cost that resembles the changes of the environment) and one rule you need to follow for safety is to not put any vunvisted node into vopen or if some vunvisted node is in vopen you need to remove it
              so since the cost of nodes doesnt change to any numbers but inf! so we only need to remove them. the following addition to heap is also for covering the vunvisted node for the next batch of update in plan function
              in the next for loop i do the heap removal

              but in the plan function since i update the nodes cost because of the second condtion in the main if! i have to update the queue's priority --> and its only happening frequntly in obstalce removal(even though it might sometimes happens in the regular loop due to the main problem that fmt has in low sample number which is negligible when samples counters go to inf theoretically!)

            */
            if (neighbor->in_queue_== false && 
                neighbor->in_unvisited_ == false &&
                samples_in_obstacles_.count(index) == 0 ) {
                    
                double h_value = use_heuristic ? heuristic(index) : 0.0;
                double priorityCost = neighbor->getCost() + h_value;
                QueueElement2 new_element = {priorityCost, index};
                v_open_heap_.add(new_element);
                neighbor->in_queue_ = true;

            }
        }
    }




    // Step 2: Update the tree structure ---> DOESNT MATTER IF THIS LOOP WOULD GO HIGHER THAN THE ABOVE FOR LOOP BECAUSE THE VUNVISTED UPDATE LOOP IS GONNA HELP THE ABOVE LOOP
    for (int orphan : orphan_nodes) {
        auto node = tree_[orphan].get();
        if (node->in_queue_==true){
            v_open_heap_.remove(orphan);
            node->in_queue_ = false;
        }

        node->setCost(INFINITY);
        node->setParent(nullptr,INFINITY);
        node->getChildrenMutable().clear();
        edge_length_[orphan] = -std::numeric_limits<double>::infinity(); // Good trick to ignore this in max_element call
    }



}



void FMTX::handleRemovedObstacleSamples(const std::vector<int>& removed) {
    // v_unvisited_set_.insert(removed.begin() , removed.end());
    for (const auto& node_index: removed) {
        auto node = tree_[node_index].get();
        node->in_unvisited_ = true;
        // I didn't use this before and had no problem (and the reason is we are putting lower priority vopen heap nodes in the handleAddObstalce and by the time we reach to this vopen node we already have a cost for that node)but i need to follow the rule of no vunvisted nodes should be in vopen because vunvisted WILL turn to vopen eventually
        if (node->in_queue_==true){ 
            v_open_heap_.remove(node_index);
            node->in_queue_ = false;
        }
    }


    // Directly process neighbors of revalidated nodes
    for (int node_index : removed) {
        auto node = tree_.at(node_index).get();
        near(node_index);
        for (const auto& [neighbor,dist] : node->neighbors()) {
            const int n_idx = neighbor->getIndex();
            if (!samples_in_obstacles_.count(n_idx) && 
                neighbor->in_unvisited_== false &&
                neighbor->in_queue_== false) {
                
                double h_value = use_heuristic ? heuristic(n_idx) : 0.0;
                double priorityCost = neighbor->getCost() + h_value;
                QueueElement2 new_element = {priorityCost, n_idx};
                v_open_heap_.add(new_element);
                neighbor->in_queue_=true;
            }
        }
    }




}


std::vector<Eigen::VectorXd> FMTX::getSmoothedPathPositions(int num_intermediates, int smoothing_passes) const {
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


std::vector<Eigen::VectorXd> FMTX::interpolatePath(const std::vector<Eigen::VectorXd>& path, int num_intermediates) const {
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


std::vector<Eigen::VectorXd> FMTX::smoothPath(const std::vector<Eigen::VectorXd>& path, int window_size) const {
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