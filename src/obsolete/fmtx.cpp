// Copyright 2025 Soheil E.nia
#include "motion_planning/planners/geometric/fmtx.hpp"
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

    v_open_set_.clear();
    v_unvisited_set_.clear();
    v_open_heap_.clear();
    boundary_.clear();

    neighbors_dict_.clear();
    // samples_in_obstacles_.clear();
    // samples_in_obstacles_2_.clear();
    inflated_samples_.clear();

    // edge_length_.clear();
    max_length_edge_ind = -1;
    max_length = -std::numeric_limits<double>::infinity();

    root_state_index_ = -1;
    robot_state_index_ = -1;

    invalid_best_neighbors.clear();
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
    first_method = params.getParam<bool>("first_method");

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
        tree_.push_back(std::make_unique<TreeNode>(statespace_->sampleUniform(lower_bound_ , upper_bound_)));
    }
    std::cout << "--- \n";
    setGoal(problem_->getGoal());

    std::cout << "KDTree: \n\n";
    if (use_kdtree == true) {
        // Put all the points at once because fmtx doesnt need incremental addition
        kdtree_->addPoints(statespace_->getSamplesCopy());
        // Build the tree all at once after we fill the data_ in the KDTree
        kdtree_->buildTree();
        // kdtree_->radiusSearch(tree_.at(0)->getStateValue(), 10);
        // std::cout << "---- \n";
        // kdtree_->knnSearch(tree_.at(0)->getStateValue(), 10);
    }

    /////////////////////////SETTING UP DS//////////////
    tree_.at(0)->setCost(0);
    v_open_set_.insert(0);
    // std::cout<<statespace_->getNumStates()<<"\n";
    v_unvisited_set_.reserve(statespace_->getNumStates()-1); // minus the root;
    for (int i = 1; i < statespace_->getNumStates() ; i++)
        v_unvisited_set_.insert(i);

    // for (auto i : v_unvisited_set_)
    //     std::cout<<i<<",";

    // v_open_heap_.push({0, 0});
    
    QueueElement2 new_element ={0,0};
    v_open_heap_.add(new_element);
    ///////////////////Neighborhood Radius////////////////////////////////
    int d = statespace_->getDimension();
    double mu = std::pow(problem_->getUpperBound() - problem_->getLowerBound() , 2);
    double zetaD = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
    double gamma = 2 * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d);
    double factor = 2.0;
    neighborhood_radius_ = factor * gamma * std::pow(std::log(statespace_->getNumStates()) / statespace_->getNumStates(), 1.0 / d);
    std::cout << "Computed value of rn: " << neighborhood_radius_ << std::endl;

    ////////////////// Test Neighborhood////////////////////////
    // near(0); // Create
    // std::cout << "-------- \n";
    // auto indices = near(0);  // Get from the cache
    // for  (auto index : indices) {
    //     std::cout << index.index <<",";
    // }


    // invalid_best_neighbors.resize(num_of_samples_+2, std::vector<bool>(num_of_samples_+2, false));
    // invalid_best_neighbors.resize(num_of_samples_ + 2, std::vector<int>(num_of_samples_ + 2, -1));

    invalid_best_neighbors.resize(num_of_samples_ + 2);




    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    // std::cout << "---\n";





}

double FMTX::heuristic(int current_index) {
    Eigen::VectorXd current_position = tree_.at(current_index)->getStateValue();
    Eigen::VectorXd goal_position = tree_.at(robot_state_index_)->getStateValue();
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

    // std::vector<Eigen::VectorXd> positions2;
    // for (const auto& y: v_open_set_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(y)->getStateValue();
    //     positions2.push_back(vec);
    // }
    // // std::string color_str = "0.0,0.0,1.0"; // Blue color
    // visualization_->visualizeNodes(positions2,"map");

    // std::vector<Eigen::VectorXd> positions3;
    // for (const auto& y: v_unvisited_set_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(y)->getStateValue();
    //     positions3.push_back(vec);
    // }
    // std::string color_str = "0.0,0.0,1.0"; // Blue color
    // visualization_->visualizeNodes(positions3,"map",color_str);

    // std::cout<< "Plan FMTX \n";
    auto start = std::chrono::high_resolution_clock::now();
    std::unordered_map<std::pair<int, int>, bool, pair_hash> obstacle_check_cache;
    int uncached = 0;
    int cached = 0;
    // std::unordered_map<int, std::unordered_set<int>> invalid_best_neighbors;
    // std::unordered_set<std::pair<int, int>, pair_hash> invalid_best_neighbors;

    if (use_heuristic==true) {
        // for (auto& row : invalid_best_neighbors) {
        //     std::fill(row.begin(), row.end(), false);
        // }
        // invalid_best_neighbors.resize(num_of_samples_+2, std::vector<bool>(num_of_samples_+2, false));
        for (auto& invalid_set : invalid_best_neighbors) {
            invalid_set.clear();
        }
        invalid_best_neighbors.resize(num_of_samples_+2);
    }


    // current_timestamp++;


    while (!v_open_heap_.empty()) {
    // while (!v_open_heap_.empty() && (partial_update ? v_open_heap_.top().index != robot_state_index_ : true)) {
    // while (!v_open_heap_.empty() && v_unvisited_set_.find(robot_state_index_) != v_unvisited_set_.end() ) {

        if (v_open_heap_.size() != v_open_set_.size()) {
            std::cout<<v_open_heap_.size() <<" "<<v_open_set_.size()<<"\n";
        }

        // auto [cost, zIndex] = v_open_heap_.top();
        auto top_element = v_open_heap_.top();
        double cost = top_element.min_key;
        int zIndex = top_element.index;
        if (partial_update == true && (zIndex==robot_state_index_ || tree_.at(zIndex)->getCost() > tree_.at(robot_state_index_)->getCost())){
            // v_open_heap_.clear(); //TODO: shoud i or not?
            // v_open_set_.clear();
            break;
        }
        v_open_heap_.pop();

        // if (v_open_set_.find(zIndex)==v_open_set_.end()) //TODO: should i?
        //     continue;



        // std::vector<Eigen::VectorXd> positions2;
        // Eigen::VectorXd vec(2);
        // vec << tree_.at(zIndex)->getStateValue();
        // positions2.push_back(vec);
        // visualization_->visualizeNodes(positions2,"map");

        // // Add a small delay for  clarity
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));



        // if (v_open_set_.find(zIndex) == v_open_set_.end() || 
        //     cost != tree_.at(zIndex)->getCost()) {
        //     continue;
        // }

        // EARLY EXIT???
        // std::cout<<zIndex<<"   " << robot_state_index_<<"\n";
        


        // if (samples_in_obstacles_.find(zIndex) != samples_in_obstacles_.end())
        // {
        //     v_open_set_.erase(zIndex);
        //     std::cout<<"OHOHOHOHOH \n";
        //     continue;
        // }

        std::vector<Eigen::VectorXd> positions;

        auto zNeighborsInfo = near(zIndex);
        bool what;
        for (const auto& [xIndex, cost_to_neighbor]: zNeighborsInfo) {

            /*
                This trick actually enables us to only track changes in the update obstalce otherwise we have to use the 
                current nodes instead of added in the handle add obstalce or the other way is to check all the edges around 
                added obstalce for obstalce

                I also added this feature to rrtx so we have a fair comparison between these two!
            */
            if (samples_in_obstacles_.find(xIndex) != samples_in_obstacles_.end()) 
                continue;

            // std::vector<Eigen::VectorXd> positions2;
            // Eigen::VectorXd vec(2);
            // vec << tree_.at(xIndex)->getStateValue();
            // positions2.push_back(vec);
            // visualization_->visualizeNodes(positions2,"map");

            // // Add a small delay for  clarity
            // std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            // if (v_unvisited_set_.find(xIndex) != v_unvisited_set_.end() || (tree_.at(xIndex)->getCost() -(tree_.at(zIndex)->getCost() + cost_to_neighbor ) > 1e-9)) {
            if (v_unvisited_set_.find(xIndex) != v_unvisited_set_.end() || tree_.at(xIndex)->getCost() > (tree_.at(zIndex)->getCost() + cost_to_neighbor ) ){
                
                // because of the second condtion in the above if sometimes the xIndex is not present in the v unvisited
                // if (v_open_set_.count(xIndex)!=0 && tree_.at(xIndex)->getCost()==std::numeric_limits<double>::infinity() && (tree_.at(xIndex)->getCost() -(tree_.at(zIndex)->getCost() + cost_to_neighbor ) > 1e-9)){
                //     std::cout<<"SOMETIMESBAD \n";
                // }





                if (v_unvisited_set_.find(xIndex) != v_unvisited_set_.end()) {
                    what = true;
                }
                else {
                    what=false;
                    // v_unvisited_set_.insert(xIndex);
                }

                // if (v_open_set_.count(xIndex)!=0) {
                //     v_open_set_.erase(xIndex);
                // }

                // if (use_heuristic && invalid_best_neighbors.find(xIndex) == invalid_best_neighbors.end()) {
                //     invalid_best_neighbors[xIndex] = std::unordered_set<int>(); 
                // }



                auto xNeighborInfo = near(xIndex);
                std::vector<NeighborInfo> Ynear;
                // for (const auto& info : xNeighborInfo) {
                //     // if (v_open_set_.find(info.index) != v_open_set_.end()) {
                //     if (isValidYnear(info.index , v_open_set_ , invalid_best_neighbors , xIndex , use_heuristic)) {
                //     // if (samples_in_obstacles_.count(info.index)==0 && v_unvisited_set_.count(info.index)==0 && v_open_set_.find(info.index) != v_open_set_.end()) {
                //         Ynear.push_back(info);
                //     }
                // }

                for (const auto& info : xNeighborInfo) {
                    if (isValidYnear(info.index, v_open_set_, invalid_best_neighbors, xIndex, use_heuristic)) {
                        Ynear.push_back(info);
                    }
                }



                // v_unvisited_set_.erase(xIndex);  //NOT GOOD TO DO IT HERE FOR THE REAL UNVISTED NODES! IT WAS GOOD FOR PROMISING BUT NOW YOU DECIED TO NOT EVEN PUT THEM IN VUNIVSTED AND JUST RECHECK THEM WITHOUT EVEN PUTTING THEM IN VUNVISTED! AND IF THEY CHANGE WE JUST PUT THEM IN VOPEN FOR THE CASCADE!
                if (Ynear.empty()) {
                    continue;
                }

                double min_cost = std::numeric_limits<double>::infinity();
                int best_neighbor_index = -1;
                double best_edge_length = std::numeric_limits<double>::infinity();
                for (const auto& y : Ynear) {
                    // if (tree_.at(y.index)->getCost()==std::numeric_limits<double>::infinity()) {
                    //     Eigen::VectorXd vec(2);
                    //     vec << tree_.at(y.index)->getStateValue();
                    //     positions.push_back(vec);
                    // }
                    double total_cost = tree_.at(y.index)->getCost() + y.distance;
                    if (total_cost < min_cost) {
                        min_cost = total_cost;
                        best_neighbor_index = y.index;
                        best_edge_length = y.distance;
                    }
                }

                // std::string color_str = "1.0,1.0,0.0"; // Blue color
                // visualization_->visualizeNodes(positions,"map",color_str);

                // positions.clear();
                // Eigen::VectorXd vec(2);
                // vec << tree_.at(zIndex)->getStateValue();
                // positions.push_back(vec);
                // visualization_->visualizeNodes(positions,"map");



                // if(best_neighbor_index==-1) {
                //     continue;
                // }

                bool obstacle_free;
                // Create a key for the cache
                if (obs_cache == true) {
                    // Create a key for the cache
                    auto edge_key = (best_neighbor_index < xIndex) ? std::make_pair(best_neighbor_index, xIndex) : std::make_pair(xIndex, best_neighbor_index);

                    // Check if the obstacle check result is already in the cache
                    if (obstacle_check_cache.find(edge_key) != obstacle_check_cache.end()) {
                        obstacle_free = obstacle_check_cache[edge_key];
                        cached++;
                    } else {
                        // Perform the obstacle check and store the result in the cache
                        obstacle_free = obs_checker_->isObstacleFree(tree_.at(xIndex)->getStateValue(), tree_.at(best_neighbor_index)->getStateValue());
                        obstacle_check_cache[edge_key] = obstacle_free;
                        uncached++;
                    }
                }
                else { //SOMETIMES BEST_NEIGHBOR_INDEX is -1 which means all the Ynear nodes has inf cost --> inf cost means its either samples_in_obstalces or vUnvisted or it was made to inf in the handleAddObstalce! --> THESE nodes shouldn't be in vOpen --> sometimes a node lingers in vOpen because of early exit so you have to erase it in handleAddObstalce or you have to check some ifs in Ynear node push_back!
                    obstacle_free = obs_checker_->isObstacleFree(tree_.at(xIndex)->getStateValue() , tree_.at(best_neighbor_index)->getStateValue());
                }

                if (obstacle_free) {
                    double newCost = min_cost;
                    if (newCost < tree_.at(xIndex)->getCost()) {
                    // if (newCost < tree_.at(xIndex)->getCost() + 1e-9) {
                        // If the node has a parent, remove it from the parent's children list
                        int parentIndex = tree_.at(xIndex)->getParentIndex();
                        if (parentIndex != -1) {
                            auto& parentChildren = tree_.at(parentIndex)->getChildrenIndices();
                            parentChildren.erase(
                                std::remove(parentChildren.begin(), parentChildren.end(), xIndex),
                                parentChildren.end()
                            );
                        }

                        // Update the node's cost
                        tree_.at(xIndex)->setCost(newCost);
                        // if (v_open_set_.count(xIndex)==0){
                            // v_open_heap_.push({newCost, xIndex});

                        ////////////////////////////////////////////////
                        double h_value = use_heuristic ? heuristic(xIndex) : 0.0;
                        double priorityCost = newCost + h_value;

                        QueueElement2 new_element = {priorityCost, xIndex};
                        if (v_open_heap_.contains(xIndex)){
                            v_open_heap_.update(xIndex, priorityCost);
                        } else{
                            v_open_heap_.add(new_element);
                        }



                        //////////////////////////////////////////////////
                        // QueueElement2 new_element = {newCost, xIndex};
                        // if (v_open_heap_.contains(xIndex)){
                        //     v_open_heap_.update(xIndex, newCost);

                        // } else{
                        //     v_open_heap_.add(new_element);
                        // }
                        ////////////////////////////////////////////////


                            v_open_set_.insert(xIndex);
                        
                        // }
                        // else{
                        //     std::cout<<"IT WAS THERE \n";
                        // }

                        // Remove the node from v_unvisited_set_
                        v_unvisited_set_.erase(xIndex); // I do this earlier!
                        // Update the node's parent and add it to the new parent's children list
                        tree_.at(xIndex)->setParentIndex(best_neighbor_index);
                        tree_.at(best_neighbor_index)->setChildrenIndex(xIndex);
                        
                        edge_length_[xIndex] = best_edge_length;

                    }
                }else{
                    if (use_heuristic==true) {
                        // invalid_best_neighbors[xIndex].insert(best_neighbor_index);
                        // invalid_best_neighbors.insert({xIndex, best_neighbor_index});
                        // invalid_best_neighbors[xIndex][best_neighbor_index] = true;
                        // invalid_best_neighbors[xIndex][best_neighbor_index] = current_timestamp;
                        // invalid_best_neighbors[xIndex][best_neighbor_index] = current_timestamp;
                        invalid_best_neighbors.at(xIndex).insert(best_neighbor_index);



                    }
                }
            }

        }


        v_open_set_.erase(zIndex);
        if (v_unvisited_set_.find(zIndex) != v_unvisited_set_.end()) {
            v_unvisited_set_.erase(zIndex);
        }




        
    }


    // std::string color_str = "0.0,1.0,1.0"; // Blue color
    // visualization_->visualizeNodes(positions,"map",color_str);

    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // std::vector<Eigen::VectorXd> positions2;
    // for (const auto& y: v_unvisited_set_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(y)->getStateValue();
    //     positions2.push_back(vec);
    // }
    // std::string color_str2 = "1.0,1.0,0.0"; // Blue color
    // visualization_->visualizeNodes(positions2,"map",color_str2);


    // if (cached !=0) {
    //     std::cout << "Cached: " << cached << "\n";
    // }

    // if (duration.count()>0){
    //     if (v_unvisited_set_.empty())
    //     {
    //         std::cout << "Time taken by while loop: " << duration.count() << " milliseconds"<<" vUnvisted is EMPTY \n";
    //     }
    //     else {
    //         std::cout << "Time taken by while loop: " << duration.count() << " milliseconds"<<" vUnvisted is  NOT  \n";
    //     }
    // }




    // // Write tree nodes to text file after planning is complete
    // std::ofstream output_file("tree_nodes.txt");

    // if (output_file.is_open()) {
    //     for (size_t i = 0; i < tree_.size(); ++i) {
    //         auto node = tree_[i];
    //         int parentIndex = node->getParentIndex();
    //         Eigen::VectorXd state = node->getStateValue();  // Assuming this gives you the position (2D)
    //         double x = state[0], y = state[1];

    //         // Write the node index, parent index, and position to the file
    //         output_file << i << ", " << parentIndex << ", " << x << ", " << y << std::endl;
    //     }

    //     output_file.close();
    //     std::cout << "Tree data written to tree_nodes.txt" << std::endl;
    // } else {
    //     std::cerr << "Failed to open file for writing!" << std::endl;
    // }
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
    int idx = robot_state_index_;
    std::vector<Eigen::VectorXd> path_positions;

    if (robot_state_index_ != -1){
        path_positions.push_back(robot_position_);
    }

    while (idx != -1) {
        path_positions.push_back(tree_.at(idx)->getStateValue());
        idx = tree_.at(idx)->getParentIndex();
    }
    return path_positions;
}

// void FMTX::setRobotIndex(const Eigen::VectorXd& robot_position) {
//     std::vector<size_t> nearest_indices = kdtree_->knnSearch(robot_position, 1);
//     int nearest = nearest_indices.empty() ? -1 : static_cast<int>(nearest_indices[0]);  

//     robot_state_index_ = nearest;
// }

// void FMTX::setRobotIndex(const Eigen::VectorXd& robot_position) {
//     // constexpr int MAX_NEIGHBORS_TO_CHECK = 10; // Adjust based on node density
//     // std::vector<size_t> nearest_indices = kdtree_->knnSearch(robot_position, MAX_NEIGHBORS_TO_CHECK);

//     double MAX_SEARCH_RADIUS = 5.0; // Meters
//     std::vector<size_t> nearest_indices = kdtree_->radiusSearch(robot_position, MAX_SEARCH_RADIUS);

    
//     // Iterate through neighbors to find the first valid (finite-cost) node
//     for (size_t index : nearest_indices) {
//         if (tree_[index]->getCost() < std::numeric_limits<double>::infinity()) {
//             robot_state_index_ = static_cast<int>(index);
//             return;
//         }
//     }
    
//     // Fallback: Use original nearest node (optional, log a warning)
//     // RCLCPP_WARN(this->get_logger(), "No valid node found within %d neighbors", MAX_NEIGHBORS_TO_CHECK);
//     robot_state_index_ = nearest_indices.empty() ? -1 : static_cast<int>(nearest_indices[0]);
// }

void FMTX::setRobotIndex(const Eigen::VectorXd& robot_position) {
    
    robot_position_ = robot_position;

    const double MAX_SEARCH_RADIUS = 5.0; // Meters
    std::vector<size_t> nearest_indices = kdtree_->radiusSearch(robot_position, MAX_SEARCH_RADIUS);
    // const int MAX_SEARCH_NODES = 30;
    // std::vector<size_t> nearest_indices = kdtree_->knnSearch(robot_position, MAX_SEARCH_NODES);
    

    size_t best_index = -1;
    double min_total_cost = std::numeric_limits<double>::max();

    // Calculate total cost for each candidate node: distance(robot→node) + cost(node→goal)
    for (size_t index : nearest_indices) {
        // Skip invalid nodes
        if (tree_[index]->getCost() == std::numeric_limits<double>::infinity()) continue;

        // Distance from robot to node
        Eigen::VectorXd node_position = tree_[index]->getStateValue();
        double dx = node_position[0] - robot_position[0];
        double dy = node_position[1] - robot_position[1];
        double distance_to_node = std::hypot(dx, dy);

        // Total cost = distance(robot→node) + cost(node→goal)
        double total_cost = distance_to_node + tree_[index]->getCost();

        if (total_cost < min_total_cost) {
            min_total_cost = total_cost;
            best_index = index;
        }
    }

    if (best_index != -1) {
        robot_state_index_ = static_cast<int>(best_index);
        return;
    }

    // // Fallback 1: Find any valid node (ignore total cost)
    // for (size_t index : nearest_indices) {
    //     if (tree_[index]->getCost() < std::numeric_limits<double>::infinity()) {
    //         robot_state_index_ = static_cast<int>(index);
    //         return;
    //     }
    // }

    // // Fallback 2: Nearest node (even if invalid)
    // robot_state_index_ = nearest_indices.empty() ? -1 : static_cast<int>(nearest_indices[0]);
}



void FMTX::setStart(const Eigen::VectorXd& start) {
    root_state_index_ = statespace_->getNumStates();
    tree_.push_back(std::make_shared<TreeNode>(statespace_->addState(start)));
    std::cout << "FMTX: Start node created on Index: " << robot_state_index_ << "\n";
}
void FMTX::setGoal(const Eigen::VectorXd& goal) {
    robot_state_index_ = statespace_->getNumStates();
    tree_.push_back(std::make_shared<TreeNode>(statespace_->addState(goal)));
    std::cout << "FMTX: Goal node created on Index: " << root_state_index_ << "\n";
}



std::vector<NeighborInfo> FMTX::near(int node_index) {
    if (neighbors_dict_.find(node_index) != neighbors_dict_.end()) {
        // std::cout<<"Got it from the cache \n";
        return neighbors_dict_[node_index];
    }

    auto indices = kdtree_->radiusSearch(tree_.at(node_index)->getStateValue(), neighborhood_radius_);
    // auto indices = kdtree_->knnSearch(tree_.at(node_index)->getStateValue(), 54);

    if (indices.empty()) {
        neighbors_dict_[node_index] = {};
        return {};
    }

    std::vector<NeighborInfo> neighbors_info;
    auto node_value = tree_.at(node_index)->getStateValue();
    for (int index : indices) {
        if(index == node_index) {
            continue;
        }
        double distance = (node_value - tree_.at(index)->getStateValue()).norm();
        neighbors_info.push_back({index, distance});

    }
    neighbors_dict_[node_index] = neighbors_info;
    return neighbors_info;
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
                nodes.push_back(tree_[i]->getStateValue());
                valid_node_indices.insert(i);
            }
        }

        // Generate edges only for valid nodes
        for (int index : valid_node_indices) {
            int parent_index = tree_[index]->getParentIndex();
            if (parent_index != -1) {
                edges.emplace_back(tree_.at(parent_index)->getStateValue(), tree_.at(index)->getStateValue());
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
            nodes.push_back(tree_node->getStateValue());
        }
    
        // Add edges to the list
        for (const auto& tree_node : tree_) {
            int parent_index = tree_node->getParentIndex();
            if (parent_index != -1) {
                edges.emplace_back(tree_.at(parent_index)->getStateValue(), tree_node->getStateValue());
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
        nodes.push_back(tree_.at(index)->getStateValue());
    }

    // Add edges to the list
    for (const auto& index : path_indices) {
        int parent_index = tree_.at(index)->getParentIndex();
        if (parent_index != -1) {
            edges.emplace_back(tree_.at(parent_index)->getStateValue(), tree_.at(index)->getStateValue());
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

// TODO: This should only be on dynamic obstacles! --> But how do we know maybe some static obstalce become dynamic! --> not motion planning concern maybe some method to classify static and dynamic obstalces!
std::unordered_set<int> FMTX::findSamplesNearObstacles(
    const ObstacleVector& obstacles, 
    double max_length
) {
    std::unordered_set<int> conflicting_samples;
    for (const auto& obstacle : obstacles) {
        // auto sample_indices = kdtree_->radiusSearch(obstacle.position, 1.5 * obstacle.radius);
        auto sample_indices = kdtree_->radiusSearch(obstacle.position, std::sqrt(std::pow(obstacle.radius, 2) + std::pow(max_length / 2.0, 2)));
        conflicting_samples.insert(sample_indices.begin(), sample_indices.end());
    }
    
    return conflicting_samples;
}

std::pair<std::unordered_set<int>, std::unordered_set<int>> FMTX::findSamplesNearObstaclesDual(
    const ObstacleVector& obstacles, 
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
void FMTX::updateObstacleSamples(const ObstacleVector& obstacles) {

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


    // // Visualizing the maximum length node
    // if (max_length_edge_ind !=-1){
    //     std::string color_str = "0.0,0.0,1.0"; // Blue color
    //     std::vector<Eigen::VectorXd> positions4;
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(max_length_edge_ind)->getStateValue();
    //     positions4.push_back(vec);
    //     visualization_->visualizeNodes(positions4,"map",color_str);

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


    // for (int sample : current) {
    //     // Loop through each child of the current sample
    //     for (int child : tree_[sample]->getChildrenIndices()) {
    //         // Check if the connection from 'sample' to 'child' is obstacle-free
    //         bool obstacle_free = obs_checker_->isObstacleFree(
    //             tree_.at(sample)->getStateValue(), 
    //             tree_.at(child)->getStateValue()
    //         );
    //         if (obstacle_free) {
    //             added.push_back(child);
    //         }
    //     }
    // }


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



    // Handle changes first. whic one should be first?
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
    for (auto it = samples_in_obstacles_.begin(); it != samples_in_obstacles_.end(); ++it) {
        v_unvisited_set_.erase(*it);  // Only erase each element if it exists in v_unvisited_set_
    }

    // std::cout<<boundary_.size() << " "<<v_unvisited_set_.size() <<"\n";


    // std::vector<Eigen::VectorXd> positions;
    // TODO: maybe an easier way instead of solving looping the over the v_unvisted_set thorugh tracking is to loop over the v_unvisted that are their heuristic is less than the current robots costToRoot! --> or if that doesnt work we can use the tracking that i used in python!
    if(first_method==false){

        for (int node : v_unvisited_set_) {
            auto neighbors = near(node);
            for (const auto& neighbor : neighbors) {
                if (v_open_set_.count(neighbor.index) == 0 &&
                    v_unvisited_set_.count(neighbor.index) == 0 &&
                    samples_in_obstacles_.count(neighbor.index) == 0 ) {
                    // if (tree_[neighbor.index]->getCost() == std::numeric_limits<double>::infinity()) { //TODO: Think about this --> i guess since you clear the vunvisted you gotta use cost inf to avoid putting thme in vOpen instead of vunsietd check in the above if condition --> think about this more! --> because later when you want to add early exit you might not even clear the vunvisted so this might be usesless later! --> maybe think about what should be in vOpen! --> the nodes that cleary have a cost other than inf!
                    //     Eigen::VectorXd vec(2);
                    //     vec << tree_.at(neighbor.index)->getStateValue();
                    //     positions.push_back(vec);
                    //     continue; //TODO: the reason why some vunvisted remains that got not connected and also they are not promising but just pure vunvisted (have cost of inf) --> it means on the last pahse they got put in the vunvisted in the handle add obstalce! but later in the plan function they didn't get connected --> but you may ask why they didn't get connected?
                    // } //TODO: continuation of the above comment --> the reason it happens is this --> imagine a scenraio that you have removed nodes that gets into v unvisted but all the vOpen are not on samples on obstacles! so that v unvisted doest get the chance to get connected to any thing else!
                    
                    // v_open_heap_.push({tree_[neighbor.index]->getCost(), neighbor.index});

                    // QueueElement2 new_element ={tree_[neighbor.index]->getCost(), neighbor.index};
                    // v_open_heap_.add(new_element);


                    double h_value = use_heuristic ? heuristic(neighbor.index) : 0.0;
                    double priorityCost = tree_[neighbor.index]->getCost() + h_value;
                    if (priorityCost < tree_[robot_state_index_]->getCost()){ // either this condition or put it in the early exit condtion so that we wouldn't put nodes behind the robot in the expansion because they don't matter! but when they do matter? when the robot get an inf cost and then this condtion becoms moot!
                        QueueElement2 new_element = {priorityCost, neighbor.index};
                        v_open_heap_.add(new_element);


                        /*
                            TODO: Think about this specially for the partial update case the above add keeps adding stuff i guess (or not!) --> you should check the std cout comparing vopen set and vopen heap.
                                Maybe because you clear v open set and heap in the early exit(partial update) we don't need to update because everything is cleared!
                        */
                        // if (v_open_heap_.contains(neighbor.index)){
                        //     v_open_heap_.update(neighbor.index, priorityCost);
                        // } else{
                        //     v_open_heap_.add(new_element);
                        // }

                        v_open_set_.insert(neighbor.index);

                    }


                }
            }
        }
    }


    // Do we need this? 
    // if (first_method==true){
    //     for (auto it = v_open_set_.begin(); it != v_open_set_.end(); ) {
    //         auto node = *it;
    //         if (tree_.at(node)->getCost()==std::numeric_limits<double>::infinity()) {
    //             std::cout<<"NO WAY \n";
    //         }
    //         // if (v_unvisited_set_.count(node) != 0 || tree_.at(node)->getCost() == std::numeric_limits<double>::infinity()) {
    //         if (v_unvisited_set_.count(node) != 0  || samples_in_obstacles_.count(node)) {
    //             std::cout<<"WWWHHYY \n";
    //             it = v_open_set_.erase(it); // Safely erase and move to the next element
    //             if (v_open_heap_.contains(node)) {
    //                 v_open_heap_.remove(node);
    //             }
    //         } else {
    //             ++it; // Move to the next element
    //         }
    //     }
    // }







    // std::string color_str = "1.0,1.0,0.0"; // Blue color
    // visualization_->visualizeNodes(positions,"map",color_str);


    // std::vector<Eigen::VectorXd> positions;
    // for (const auto& y: v_unvisited_set_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(y)->getStateValue();
    //     positions.push_back(vec);
    // }
    // std::string color_str = "0.0,0.0,1.0"; // Blue color
    // visualization_->visualizeNodes(positions,"map",color_str);

    // std::vector<Eigen::VectorXd> positions2;
    // for (const auto& y: v_open_set_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(y)->getStateValue();
    //     positions2.push_back(vec);
    // }
    // // std::string color_str = "0.0,0.0,1.0"; // Blue color
    // visualization_->visualizeNodes(positions2,"map");
    

    // std::vector<Eigen::VectorXd> positions3;
    // for (const auto& y: samples_in_obstacles_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(y)->getStateValue();
    //     positions3.push_back(vec);
    // }
    // visualization_->visualizeNodes(positions3,"map");

    // // Whats the point of putting these in vUnvisted when they are on obstalce! BUT SHOULD I DO IT BEFORE THE PLAN OR AFTER THE PLAN?? WELL the samples_in_obstalces_ is used in the main while loop anyway!
    // for (auto it = samples_in_obstacles_.begin(); it != samples_in_obstacles_.end(); ++it) {
    //     v_unvisited_set_.erase(*it);  // Only erase each element if it exists in v_unvisited_set_
    // }
   
    // plan(); //lets put it outside!



}



std::unordered_set<int> FMTX::getDescendants(int node_index) {
    std::unordered_set<int> descendants;
    std::queue<int> queue;
    queue.push(node_index);
    while (!queue.empty()) {
        int current = queue.front();
        queue.pop();
        descendants.insert(current);

        for (int child : tree_[current]->getChildrenIndices()) {
            queue.push(child);
        }
    }

    return descendants;
}



void FMTX::handleAddedObstacleSamples(const std::vector<int>& added) {
    std::unordered_set<int> orphan_nodes;
    std::unordered_set<int> initial_messedup;
    // Step 1: Identify orphan nodes (nodes now in obstacles)
    for (int idx : added) {
        initial_messedup.insert(idx);
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
    v_unvisited_set_.insert(orphan_nodes.begin() , orphan_nodes.end()); 

    if (first_method == true){
        for (auto node : orphan_nodes) {
            auto neighbors = near(node);
            for (const auto& neighbor : neighbors) {
                if (v_open_set_.count(neighbor.index) == 0 &&
                    v_unvisited_set_.count(neighbor.index) == 0 &&
                    samples_in_obstacles_.count(neighbor.index) == 0 ) {
                        
                    double h_value = use_heuristic ? heuristic(neighbor.index) : 0.0;
                    double priorityCost = tree_[neighbor.index]->getCost() + h_value;
                    QueueElement2 new_element = {priorityCost, neighbor.index};
                    // v_open_heap_.add(new_element);
                    if (v_open_heap_.contains(neighbor.index)){
                        v_open_heap_.update(neighbor.index, priorityCost);
                    } else{
                        v_open_heap_.add(new_element);
                    }
                    v_open_set_.insert(neighbor.index);

                }
            }
        }
    }




    // Step 2: Update the tree structure
    for (int orphan : orphan_nodes) {
        v_open_set_.erase(orphan);
        if (v_open_heap_.contains(orphan))
            v_open_heap_.remove(orphan);


        int parent_idx = tree_[orphan]->getParentIndex();
        if (parent_idx != -1) {
            // Remove the orphan from its parent's children list
            auto& parent_children = tree_.at(parent_idx)->getChildrenIndices();
            parent_children.erase(
                std::remove(parent_children.begin(), parent_children.end(), orphan),
                parent_children.end()
            );
        }
        // Clear orphan's children list
        tree_[orphan]->getChildrenIndices().clear();
        tree_[orphan]->setCost(std::numeric_limits<double>::infinity());
        tree_[orphan]->setParentIndex(-1);
        edge_length_[orphan] = -std::numeric_limits<double>::infinity(); // Good trick to ignore this in max_element call
    }

}



void FMTX::handleRemovedObstacleSamples(const std::vector<int>& removed) {
    v_unvisited_set_.insert(removed.begin() , removed.end());


    if(first_method==true){
        // Directly process neighbors of revalidated nodes
        for (int node : removed) {
            for (const auto& neighbor : near(node)) {
                const int n_idx = neighbor.index;
                if (!samples_in_obstacles_.count(n_idx) && 
                    !v_unvisited_set_.count(n_idx) &&
                    !v_open_set_.count(n_idx)) {
                    
                    double h_value = use_heuristic ? heuristic(neighbor.index) : 0.0;
                    double priorityCost = tree_[neighbor.index]->getCost() + h_value;
                    QueueElement2 new_element = {priorityCost, neighbor.index};
                    // v_open_heap_.add(new_element);
                    if (v_open_heap_.contains(neighbor.index)){
                        v_open_heap_.update(neighbor.index, priorityCost);
                    } else{
                        v_open_heap_.add(new_element);
                    }
                    v_open_set_.insert(neighbor.index);
                }
            }
        }
    }




}




// std::vector<Eigen::VectorXd> FMTX::getSmoothedPathPositions(int num_intermediates = 3, int smoothing_window = 3) const {
//     auto original_path = getPathPositions();
//     std::reverse(original_path.begin(), original_path.end());

//     auto interpolated_path = interpolatePath(original_path, num_intermediates);
//     auto smoothed_path = smoothPath(interpolated_path, smoothing_window);
//     // smoothed_path_ = smoothed_path;
//     return smoothed_path;
// }



// std::vector<Eigen::VectorXd> FMTX::interpolatePath(const std::vector<Eigen::VectorXd>& path, int num_intermediates) const {
//     std::vector<Eigen::VectorXd> new_path;
//     if (path.empty()) return new_path;

//     new_path.push_back(path[0]);
//     for (size_t i = 1; i < path.size(); ++i) {
//         const Eigen::VectorXd& prev = path[i-1];
//         const Eigen::VectorXd& curr = path[i];

//         for (int j = 1; j <= num_intermediates; ++j) {
//             double t = static_cast<double>(j) / (num_intermediates + 1);
//             Eigen::VectorXd interpolated = prev + t * (curr - prev);
//             new_path.push_back(interpolated);
//         }
//         new_path.push_back(curr);
//     }
//     return new_path;
// }

// std::vector<Eigen::VectorXd> FMTX::smoothPath(const std::vector<Eigen::VectorXd>& path, int window_size) const {
//     if (path.size() <= 2 || window_size < 1) return path;

//     std::vector<Eigen::VectorXd> smoothed_path = path;
//     int half_window = window_size / 2;

//     for (size_t i = 0; i < path.size(); ++i) {
//         int start = std::max(0, static_cast<int>(i) - half_window);
//         int end = std::min(static_cast<int>(path.size() - 1), static_cast<int>(i) + half_window);
//         int count = end - start + 1;

//         Eigen::VectorXd sum = Eigen::VectorXd::Zero(path[0].size());
//         for (int j = start; j <= end; ++j) {
//             sum += path[j];
//         }
//         smoothed_path[i] = sum / count;
//     }

//     return smoothed_path;
// }



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