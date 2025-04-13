// Copyright 2025 Soheil E.nia
#include "motion_planning/planners/geometric/fmta.hpp"

FMTA::FMTA(std::shared_ptr<StateSpace> statespace ,std::shared_ptr<ProblemDefinition> problem_def, std::shared_ptr<ObstacleChecker> obs_checker) :  statespace_(statespace), problem_(problem_def), obs_checker_(obs_checker) {
    std::cout<< "FMTA Constructor \n";

}

void FMTA::clearPlannerState() {

    for (auto& node : tree_) {
        node->disconnectFromGraph();
        node.reset();  
    }
    tree_.clear();
    statespace_->reset();
    kdtree_.reset();
    v_open_heap_.clear();
    root_state_index_ = -1;
    robot_state_index_ = -1;

}


void FMTA::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    auto start = std::chrono::high_resolution_clock::now();
    clearPlannerState();
    visualization_ = visualization;
    num_of_samples_ = params.getParam<int>("num_of_samples");
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

    std::cout << "num_of_samples=" << num_of_samples_
                << ", bounds=[" << lower_bound_ << ", " << upper_bound_ << "]\n";


    std::cout << "Taking care of the samples: \n \n";
    setStart(problem_->getStart());
    for (int i = 0 ; i < num_of_samples_; i++) {  // BUT THIS DOESNT CREATE A TREE NODE FOR START AND GOAL !!!
        auto node = std::make_shared<FMTNode>(statespace_->sampleUniform(lower_bound_ , upper_bound_),tree_.size());
        node->in_unvisited_ = true;
        tree_.push_back(node);
    }
    setGoal(problem_->getGoal());


    std::cout << "KDTree: \n\n";
    if (use_kdtree == true) {
        // Put all the points at once because FMTA doesnt need incremental addition
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

void FMTA::plan() {

    // visualizeHeapAndUnvisited();

    /*
        Does caching obstacle for each run helps in fmta? it does in fmt but for fmta i dont think so! 
    */
    std::unordered_map<std::pair<int, int>, bool, pair_hash> obstacle_check_cache;
    int uncached = 0;
    int cached = 0;
    int checks = 0;

    while (!v_open_heap_.empty() &&
           ((v_open_heap_.top().first < robot_node_->getCost() ||
            robot_node_->in_unvisited_==true ||
            robot_node_->in_queue_==true))){

        // visualizeHeapAndUnvisited();
        auto top_element = v_open_heap_.top();
        double cost = top_element.first;
        FMTNode* z = top_element.second;
        int zIndex = z->getIndex();

        near(zIndex);
        for (const auto& [x, cost_to_neighbor] : z->neighbors()) {
            int xIndex = x->getIndex(); // As I refactor the code I don't need to use xIndex anymore but I still need som refactoring.


            /*
                Important note: if you are skeptical on why the tree it self is not optimal i gotta say the path from robot to root is indeed optimal
                                and the reason you see some not optimal routes is that you early exit because of the conditions after the "&"
                                i.e.,:
                                ((v_open_heap_.top().first < robot_node_->getCost() ||
                                    robot_node_->in_unvisited_==true ||
                                    robot_node_->in_queue_==true)))

                also if you let go of the above condtion and use my repair condtion below instead of "if (x->in_unvisited_==true  ){"
                if (x->getCost() > (z->getCost() + cost_to_neighbor.distance ) ){ 

                you'll see it'll get optimal but it will take much more time to create the whole graph as opposed to the original fmt itself because
                in fmta your queue is biased toward the robot (Also obstalce cache helps less here! you can check the uncache and cache value!)

                so an important insight i got here is if the queue is BFS the "if (x->getCost() > (z->getCost() + cost_to_neighbor.distance ) ){" condtion is implicitly
                like "if (x->in_unvisited_==true  ){" in static planning but in A* like (DFS like!) my condtion gets to repair the wrong first pass and does redundant stuff 
                because the first pass did some wrong connections!
            */

            if (x->in_unvisited_==true  ){
            // if (x->getCost() > (z->getCost() + cost_to_neighbor.distance ) ){

                checks++;
                near(xIndex);
                double min_cost = x->getCost();
                FMTNode* best_neighbor_node = nullptr;
                double best_edge_length = 0.0;

                for (const auto& [neighbor, dist] : x->neighbors()) {
                    if(x->blocked_best_neighbors.count(neighbor->getIndex()) > 0)
                        continue;
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
                if (obs_cache == true) {
                    // Create a key for the cache
                    auto edge_key = (best_neighbor_index < xIndex) ? std::make_pair(best_neighbor_index, xIndex) : std::make_pair(xIndex, best_neighbor_index);

                    // Check if the obstacle check result is already in the cache
                    if (obstacle_check_cache.find(edge_key) != obstacle_check_cache.end()) {
                        obstacle_free = obstacle_check_cache[edge_key];
                        cached++;
                    } else {
                        // Perform the obstacle check and store the result in the cache
                        obstacle_free = obs_checker_->isObstacleFree(x->getStateValue() , best_neighbor_node->getStateValue());
                        obstacle_check_cache[edge_key] = obstacle_free;
                        uncached++;
                    }
                }
                else { //SOMETIMES BEST_NEIGHBOR_INDEX is -1 which means all the Ynear nodes has inf cost --> inf cost means its either samples_in_obstalces or vUnvisted or it was made to inf in the handleAddObstalce! --> THESE nodes shouldn't be in vOpen --> sometimes a node lingers in vOpen because of early exit so you have to erase it in handleAddObstalce or you have to check some ifs in Ynear node push_back!
                    obstacle_free = obs_checker_->isObstacleFree(x->getStateValue() , best_neighbor_node->getStateValue());
                }

                // obstacle_free = obs_checker_->isObstacleFree(x->getStateValue() , best_neighbor_node->getStateValue());
             

                if (obstacle_free) {
                    double newCost = min_cost;
                    if (newCost < x->getCost()) {
                        x->blocked_best_neighbors.clear(); // Well if x is connected then i don't care about neighbors that can't be connected so what a better place to clearing them than here. this is for when you use heuristic
                        x->setCost(newCost);
                        double h_value =  heuristic(xIndex);
                        double priorityCost = newCost + h_value;
                        v_open_heap_.add(x,priorityCost);
                        x->setParent(best_neighbor_node,best_edge_length); 
                        x->in_unvisited_=false;

                    }
                }
                else{
                    x->blocked_best_neighbors.insert(best_neighbor_index);
                }
            }
        }

        v_open_heap_.pop();
        z->in_unvisited_ =false;
    }

    // std::cout<<"Obs checks: "<< checks <<"\n";
    std::cout<<"cached: "<< cached <<"\n";
    std::cout<<"uncached: "<< uncached <<"\n";
    std::cout<<"cost: "<<robot_node_->getCost()<<"\n";

}


void FMTA::near(int node_index) {
    auto node = tree_[node_index].get();
    if (!node->neighbors().empty()) return;

    auto indices = kdtree_->radiusSearch(node->getStateValue(), neighborhood_radius_); 
    for(int idx : indices) {
        if(idx == node->getIndex()) continue;
        FMTNode* neighbor = tree_[idx].get();
        auto dist = (node->getStateValue() - neighbor->getStateValue()).norm();
        node->neighbors()[neighbor] = EdgeInfo{dist,dist};
    }
}


double FMTA::heuristic(int current_index) {
    Eigen::VectorXd current_position = tree_.at(current_index)->getStateValue();
    Eigen::VectorXd goal_position = tree_.at(robot_state_index_)->getStateValue();
    return (goal_position-current_position).norm();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<size_t> FMTA::getPathIndex() const {
    int idx = robot_state_index_;
    std::vector<size_t> path_index;
    while (idx != -1) {
        path_index.push_back(idx);
        idx = tree_.at(idx)->getParent()->getIndex();
    }
    return path_index;
}

std::vector<Eigen::VectorXd> FMTA::getPathPositions() const {
    std::vector<Eigen::VectorXd> path_positions;

    if (robot_node_ != nullptr) {
        path_positions.push_back(robot_position_);
    }

    FMTNode* current_node = robot_node_;

    // Traverse the tree from the robot's node to the root
    while (current_node != nullptr) {
        path_positions.push_back(current_node->getStateValue());
        current_node = current_node->getParent();
    }

    return path_positions;
}

void FMTA::setRobotIndex(const Eigen::VectorXd& robot_position) {
    robot_position_ = robot_position;

    const double MAX_SEARCH_RADIUS = 5.0; // Meters
    std::vector<size_t> nearest_indices = kdtree_->radiusSearch(robot_position, MAX_SEARCH_RADIUS);

    size_t best_index = std::numeric_limits<size_t>::max(); 
    double min_total_cost = std::numeric_limits<double>::max();
    FMTNode* best_node = nullptr; 

    for (size_t index : nearest_indices) {
        auto node = tree_.at(index).get();
        if (node->getCost() == std::numeric_limits<double>::infinity()) continue;

        Eigen::VectorXd node_position = node->getStateValue();
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
        robot_node_ = best_node;
        return;
    }

    bool keep_prev_state_ = false;
    if (robot_node_ && keep_prev_state_ == true) {
        std::cout << "No valid node found in neighborhood. Keeping previous robot_node_.\n";
        return;
    }
    if (robot_node_) {
        std::cout << "No valid node found in neighborhood. Setting to nearest unvisited(cost=inf) node.\n";
        // so it must be on the vunvisted zones --> lets get the nearest vunvisted and then rely on plan function to reach there if it can!
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

void FMTA::setStart(const Eigen::VectorXd& start) {
    root_state_index_ = statespace_->getNumStates();
    auto node = std::make_shared<FMTNode>(statespace_->addState(start),tree_.size());
    node->setCost(0);
    // node->in_queue_ = true;
    // QueueElement2 new_element ={0,0};
    v_open_heap_.add(node.get(), 0);

    tree_.push_back(node);
    std::cout << "FMTA: Start node created on Index: " << robot_state_index_ << "\n";
}
void FMTA::setGoal(const Eigen::VectorXd& goal) {
    robot_state_index_ = statespace_->getNumStates();
    auto node = std::make_shared<FMTNode>(statespace_->addState(goal),tree_.size());
    node->in_unvisited_ = true;

    robot_node_ = node.get(); // Management of the node variable above will be done by the unique_ptr i'll send to tree_ below so robot_node_ is just using it!
    tree_.push_back(node);
    std::cout << "FMTA: Goal node created on Index: " << root_state_index_ << "\n";
}


std::vector<Eigen::VectorXd> FMTA::getSmoothedPathPositions(int num_intermediates, int smoothing_passes) const {
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


std::vector<Eigen::VectorXd> FMTA::interpolatePath(const std::vector<Eigen::VectorXd>& path, int num_intermediates) const {
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


std::vector<Eigen::VectorXd> FMTA::smoothPath(const std::vector<Eigen::VectorXd>& path, int window_size) const {
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

void FMTA::visualizeTree() {
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
            int parent_index = tree_[index]->getParent()->getIndex();
            if (parent_index != -1) {
                edges.emplace_back(tree_.at(parent_index)->getStateValue(), tree_.at(index)->getStateValue());
            }
        }
        // Visualize nodes and edges
        // visualization_->visualizeNodes(nodes);
        visualization_->visualizeEdges(edges);
    }
    else {
        std::vector<Eigen::VectorXd> tree_nodes;
        std::vector<Eigen::VectorXd> vopen_positions;
        std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;

    
        for (const auto& tree_node : tree_) {
            // Collect all tree nodes
            tree_nodes.push_back(tree_node->getStateValue());

            // Check if node is in vopen (in_queue_)
            if (tree_node->in_queue_) {
                vopen_positions.push_back(tree_node->getStateValue());
            }

            // Collect edges
            auto parent = tree_node->getParent();
            if (parent) {
                edges.emplace_back(parent->getStateValue(), tree_node->getStateValue());
            }
        }
    
        // // Visualize tree components
        // visualization_->visualizeNodes(tree_nodes, "map", 
        //                             std::vector<float>{1.0f, 0.0f, 0.0f},  // Red for tree
        //                             "tree_nodes");
        
        // // Visualize vopen nodes with different color/namespace
        // visualization_->visualizeNodes(vopen_positions);

        // Visualize edges
        visualization_->visualizeEdges(edges, "map");
    }


}

void FMTA::visualizePath(std::vector<size_t> path_indices) {
    std::vector<Eigen::VectorXd> nodes;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;

    // Add nodes to the list
    for (const auto& index : path_indices) {
        nodes.push_back(tree_.at(index)->getStateValue());
    }

    // Add edges to the list
    for (const auto& index : path_indices) {
        int parent_index = tree_.at(index)->getParent()->getIndex();
        if (parent_index != -1) {
            edges.emplace_back(tree_.at(parent_index)->getStateValue(), tree_.at(index)->getStateValue());
        }
    }

    // Use the visualization class to visualize nodes and edges
    // visualization_->visualizeNodes(nodes);
    visualization_->visualizeEdges(edges,"map","0.0,1.0,0.0");
}


void FMTA::visualizeSmoothedPath(const std::vector<Eigen::VectorXd>& shortest_path_) {
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


void FMTA::visualizeHeapAndUnvisited() {
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
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    visualization_->visualizeNodes(vopen_positions);
}



