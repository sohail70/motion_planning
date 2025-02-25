// Copyright 2025 Soheil E.nia
#include "motion_planning/planners/geometric/fmtx.hpp"

FMTX::FMTX(std::unique_ptr<StateSpace> statespace ,std::unique_ptr<ProblemDefinition> problem_def, std::shared_ptr<ObstacleChecker> obs_checker) :  statespace_(std::move(statespace)), problem_(std::move(problem_def)), obs_checker_(obs_checker) {
    std::cout<< "FMTX Constructor \n";

}

void FMTX::setup(const PlannerParams& params, std::shared_ptr<Visualization> visualization) {

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
        // kdtree_->radiusSearch(tree_.at(0)->getStateVlaue(), 10);
        // std::cout << "---- \n";
        // kdtree_->knnSearch(tree_.at(0)->getStateVlaue(), 10);
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

    v_open_heap_.push({0, 0});
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


    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    // std::cout << "---\n";


}

void FMTX::plan() {
    // std::cout<< "Plan FMTX \n";
    auto start = std::chrono::high_resolution_clock::now();
    std::unordered_map<std::pair<int, int>, bool, pair_hash> obstacle_check_cache;
    std::vector<Eigen::VectorXd> positions;
    int uncached = 0;
    int cached = 0;
    while (! v_open_heap_.empty()) {
        auto [cost, zIndex] = v_open_heap_.top();
        v_open_heap_.pop();
        // if (v_open_set_.find(zIndex)==v_open_set_.end()) //TODO: should i?
        //     continue;
        if (zIndex==1270){

            // std::cout<<"1270 is zIndex \n";
        }



        auto zNeighborsInfo = near(zIndex);
        for (const auto& [xIndex, cost_to_neighbor]: zNeighborsInfo) {

            if (samples_in_obstacles_.find(xIndex) != samples_in_obstacles_.end())
                continue;

            if (v_unvisited_set_.find(xIndex) != v_unvisited_set_.end()) {

                // std::cout<<"Proces \n";
            } else if (tree_.at(xIndex)->getCost() > tree_.at(zIndex)->getCost() + cost_to_neighbor) {

                if (xIndex==1270) {
                    // std::cout<<"GOT PROMISING  with zIndex: "<<zIndex <<"\n";
                    // fflush(stdout);
                }
                Eigen::VectorXd vec(2);
                vec << tree_.at(xIndex)->getStateVlaue();
                positions.push_back(vec);


                v_unvisited_set_.insert(xIndex);
                // if (v_open_set_.find(xIndex)!=v_open_set_.end()) //TODO: should i?
                //     v_open_set_.erase(xIndex);

                auto parent_of_xIndex = tree_.at(xIndex)->getParentIndex();
                // TODO: Think about the second condition for the below if! and also read the comments below! --> you can indeed set parent and delete chldren but for the early exit you might find problem
                // and rememerb the reason for the following if! -->you might invalid some node but later lose the best vOpen that it already has! --? because vOPen for dynamic obstalce is made by hand by me! it doesnt cover everything and it around the current changes in the environemtn --> but you are actually in this code tracking all the obstalces!!!! so its not incremental so you might even doesnt need the following if! --> in python removing and adding was consecutive but here i simultanelusy check both of them! --> does it even matter for this problem --> im just babbling!
                // if (v_open_set_.find(parent_of_xIndex) == v_open_set_.end() && tree_[parent_of_xIndex]->getCost()!=std::numeric_limits<double>::infinity()) {
                // // if (v_open_set_.find(parent_of_xIndex) == v_open_set_.end() && v_unvisited_set_.find(parent_of_xIndex) == v_unvisited_set_.end()) { //the second condition means the parent is in vUnvisted then later it will be in the vOpen so no need to rush it ! (you can not use it any as  vopen any way because the cost is inf! but their child parent relationship is valid because of the early exit problem i had in python)
                //     v_open_heap_.push({tree_.at(parent_of_xIndex)->getCost() , parent_of_xIndex});
                //     v_open_set_.insert(parent_of_xIndex);

                //     if (tree_[parent_of_xIndex]->getCost() == std::numeric_limits<double>::infinity()) {
                //         std::cout<<parent_of_xIndex<< "\n";
                //         std::cout<<"why 2\n";
                //     }
                //     // sometimes i make a node's cost to inf in here but i don't delete its children and parent index so later their children might again be in this loop and when i want to put their parent in the vOpen then i'd put inf node to vOpen but vOpen nodes are for expaning and it measn they are visisted once and ready to expand other but cost of inf means unvisted!
                //     // I remember not delting chld and parent was only because of early exit! --> maybe there is a better solution
                //     // But is this really a problem? wouldn't they eventually update! --> i put the parent inf check in the if but not much though about it --> weirdly this doesnt happen when i make the circle bigger for findnearestnodesaroundobstalce
                    


                // }
                // if(v_unvisited_set_.find(1270) != v_unvisited_set_.end() && xIndex==1270)
                //     std::cout<<"1270 is in vUnivsted \n";

                // if (parent_of_xIndex != -1) {
                //     std::vector<int>& childs = tree_.at(parent_of_xIndex)->getChildrenIndices();
                //     childs.erase(std::remove(childs.begin(), childs.end(), xIndex),childs.end());
                // }
                // tree_.at(xIndex)->setParentIndex(-1);

                // tree_.at(xIndex)->setCost(std::numeric_limits<double>::infinity()); //TODO: WTF ? why by commenting this it works now! but it gets slower!
            } else {
                continue;
            }

            auto xNeighborInfo = near(xIndex);
            std::vector<NeighborInfo> Ynear;
            for (const auto& info : xNeighborInfo) {
                if (v_open_set_.find(info.index) != v_open_set_.end()) {
                    Ynear.push_back(info);
                }
            }

            if (Ynear.empty()) {
                continue;
            }

            double min_cost = std::numeric_limits<double>::infinity();
            int best_neighbor_index = -1;
            for (const auto& y : Ynear) {
                double total_cost = tree_.at(y.index)->getCost() + y.distance;
                if (total_cost < min_cost) {
                    min_cost = total_cost;
                    best_neighbor_index = y.index;
                }
            }

            // // I change the position of handleAdd and handleRemove in update and i don't need the below check! --> look further into this!
            // if (best_neighbor_index==-1) {
            //     std::cout<<"WHY \n";
            //     continue;
            // }
            //obstcle check! later!

            // bool obstalce_check = true;
            // if (best_neighbor_index==-1) {
            //     std::vector<Eigen::VectorXd> positions;
            //     for (const auto& y: Ynear) {
            //         Eigen::VectorXd vec(2);
            //         vec << tree_.at(y.index)->getStateVlaue();
            //         positions.push_back(vec);
            //         visualization_->visualizeNodes(positions,"map");
            //         std::cout<<"not good"<<y.index << "\n";
            //     }
            // }
            
            // std::vector<Eigen::VectorXd> positions;
            // Eigen::VectorXd vec(2);
            // vec << tree_.at(best_neighbor_index)->getStateVlaue();
            // positions.push_back(vec);
            // visualization_->visualizeNodes(positions,"map");
            /////////////////////////////////////////
             bool obstacle_free;
            // Create a key for the cache
            if (obs_cache == true) {
                // Create a key for the cache
                auto edge_key = (best_neighbor_index < xIndex) ? std::make_pair(best_neighbor_index, xIndex) : std::make_pair(xIndex, best_neighbor_index);

                // Check if the obstacle check result is already in the cache
                bool obstacle_free;
                if (obstacle_check_cache.find(edge_key) != obstacle_check_cache.end()) {
                    obstacle_free = obstacle_check_cache[edge_key];
                    cached++;
                } else {
                    // Perform the obstacle check and store the result in the cache
                    obstacle_free = obs_checker_->isObstacleFree(tree_.at(xIndex)->getStateVlaue(), tree_.at(best_neighbor_index)->getStateVlaue());
                    obstacle_check_cache[edge_key] = obstacle_free;
                    uncached++;
                }
            }
            else {
                obstacle_free = obs_checker_->isObstacleFree(tree_.at(xIndex)->getStateVlaue() , tree_.at(best_neighbor_index)->getStateVlaue());
            }
            ///////////////////////////////////////




            if (obstacle_free) {
                double newCost = min_cost; // newCost is the cost from the best neighbor
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

                    // Add the node to v_open_heap_ and v_open_set_ if it's not already there
                    if (v_open_set_.find(xIndex) == v_open_set_.end()) {
                        v_open_heap_.push({newCost, xIndex});
                        v_open_set_.insert(xIndex);
                    }

                    // Remove the node from v_unvisited_set_
                    if (xIndex==1270) {
                        // std::cout<<"1270 erased from vUnvisted (connected) to: " << best_neighbor_index <<"\n";
                    }
                    v_unvisited_set_.erase(xIndex);

                    // Update the node's parent and add it to the new parent's children list
                    tree_.at(xIndex)->setParentIndex(best_neighbor_index);
                    tree_.at(best_neighbor_index)->setChildrenIndex(xIndex);

                    // Eigen::VectorXd parent_state = tree_.at(best_neighbor_index)->getStateVlaue();
                    // Eigen::VectorXd child_state = tree_.at(xIndex)->getStateVlaue();
                    // double edge_length = (parent_state - child_state).norm();

                    // // Update max_edge_ if this is the largest edge so far
                    // if (edge_length > max_edge_.length) {
                    //     max_edge_.length = edge_length;
                    //     max_edge_.from = best_neighbor_index;
                    //     max_edge_.to = xIndex;
                    // }



                }
            }

            //Early termination --> later


        }


        v_open_set_.erase(zIndex);
        if (v_unvisited_set_.find(zIndex) != v_unvisited_set_.end()) {
            v_unvisited_set_.erase(zIndex);
            if (zIndex==1270){
                // std::cout<<"1270 got deleted from vUNvisted \n";

            }
        }

    }

    // std::cout<<"------ \n";
    // std::cout<<"uncached: "<<uncached <<"\n";
    if (cached!=0)
        std::cout<<"cached: "<<cached <<"\n";

    // std::cout<<"------ \n";


    std::string color_str = "0.0,1.0,1.0"; // Blue color
    // visualization_->visualizeNodes(positions,"map",color_str);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout << "Time taken by while loop: " << duration.count() << " milliseconds\n";
    // std::cout << "The End \n";


    // // Write tree nodes to text file after planning is complete
    // std::ofstream output_file("tree_nodes.txt");

    // if (output_file.is_open()) {
    //     for (size_t i = 0; i < tree_.size(); ++i) {
    //         auto node = tree_[i];
    //         int parentIndex = node->getParentIndex();
    //         Eigen::VectorXd state = node->getStateVlaue();  // Assuming this gives you the position (2D)
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

std::vector<int> FMTX::getPathIndex() const {
    int idx = robot_state_index_;
    std::vector<int> path;
    while (idx != -1) {
        path.push_back(idx);
        idx = tree_.at(idx)->getParentIndex();
    }
    return path;
}

void FMTX::setRobotIndex(const Eigen::VectorXd& robot_position) {
    std::vector<size_t> nearest_indices = kdtree_->knnSearch(robot_position, 1);
    int nearest = nearest_indices.empty() ? -1 : static_cast<int>(nearest_indices[0]);  

    robot_state_index_ = nearest;
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

    auto indices = kdtree_->radiusSearch(tree_.at(node_index)->getStateVlaue(), neighborhood_radius_);
    // auto indices = kdtree_->knnSearch(tree_.at(node_index)->getStateVlaue(), 54);

    if (indices.empty()) {
        neighbors_dict_[node_index] = {};
        return {};
    }

    std::vector<NeighborInfo> neighbors_info;
    auto node_value = tree_.at(node_index)->getStateVlaue();
    for (int index : indices) {
        if(index == node_index) {
            continue;
        }
        double distance = (node_value - tree_.at(index)->getStateVlaue()).norm();
        neighbors_info.push_back({index, distance});

    }
    neighbors_dict_[node_index] = neighbors_info;
    return neighbors_info;
}


void FMTX::visualizeTree() {
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

void FMTX::visualizePath(std::vector<int> path_indices) {
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


// TODO: This should only be on dynamic obstacles! --> But how do we know maybe some static obstalce become dynamic! --> not motion planning concern maybe some method to classify static and dynamic obstalces!
std::unordered_set<int> FMTX::findSamplesNearObstacles(
    const std::vector<Eigen::Vector2d>& obstacles, 
    double obstacle_radius
) {
    std::unordered_set<int> conflicting_samples;
    double robot_range = 20.0;    
    Eigen::Vector2d robot_position;
    robot_position << tree_.at(robot_state_index_)->getStateVlaue();
    for (const auto& obstacle : obstacles) {
        if (use_range==true) {
            if ((obstacle - robot_position).norm() <= robot_range) {
                // Query samples within obstacle radius (5 units)
                auto sample_indices = kdtree_->radiusSearch(obstacle, obstacle_radius);
                conflicting_samples.insert(sample_indices.begin(), sample_indices.end());
            }
        }
        else {
                // Query samples within obstacle radius (5 units)
                auto sample_indices = kdtree_->radiusSearch(obstacle, obstacle_radius);
                conflicting_samples.insert(sample_indices.begin(), sample_indices.end());
        }
    }
    
    return conflicting_samples;
}

void FMTX::updateObstacleSamples(const std::vector<Eigen::Vector2d>& obstacles) {
    // Find current samples in obstacles
    auto current = findSamplesNearObstacles(obstacles, 2.2*5.0); // TODO: i don't think its correct to scale this but its necessary to (it needs to be integrated with max length) --> its not correct in a sense that the scaled onces shoudlnt go into the samples in obstalces i guess because i avoid them in the main while loop --> weirdly it works but i'll take a look later!
    
    // Compute added/removed samples
    // std::vector<int> added, removed;
    // std::set_difference(
    //     current.begin(), current.end(),
    //     samples_in_obstacles_.begin(), samples_in_obstacles_.end(),
    //     std::back_inserter(added)
    // );
    // std::set_difference(
    //     samples_in_obstacles_.begin(), samples_in_obstacles_.end(),
    //     current.begin(), current.end(),
    //     std::back_inserter(removed)
    // );

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


    // std::vector<Eigen::VectorXd> positions;
    // for (int node : v_unvisited_set_) {
    //     if (tree_[node]->getCost() == std::numeric_limits<double>::infinity()) {
    //         std::cout << "Node " << node << " has infinite cost!\n";
    //         // Eigen::VectorXd vec(2);
    //         // vec << tree_.at(y)->getStateVlaue();
    //         // positions.push_back(vec);
    //         // visualization_->visualizeNodes(positions,"map");
    //     }
    // }

    v_unvisited_set_.clear();  //TODO: should i clear?
    // Handle changes first. whic one should be first?
    if (!added.empty()) {
        handleAddedObstacleSamples(added);  // Only call if added has elements
    }
    if (!removed.empty()) {
        handleRemovedObstacleSamples(removed);  // Only call if removed has elements
    }


    
    // Update the tracked set
    samples_in_obstacles_ = std::move(current);

    // for (auto it = samples_in_obstacles_.begin(); it != samples_in_obstacles_.end(); ++it) {
    //     v_unvisited_set_.erase(*it);  // Only erase each element if it exists in v_unvisited_set_
    // }
    
    // int a = 1270;
    // if (samples_in_obstacles_.find(a) != samples_in_obstacles_.end())
    //     std::cout<<"Makes sense1!"<<"\n";
    // if (v_unvisited_set_.find(a) != v_unvisited_set_.end())
    //     std::cout<<"Makes sense2!"<<"\n";
    ////////////////////////////////////////////////////////////////////////////////
    // std::unordered_set<int> difference;
    // for (int num : v_unvisited_set_) {
    //     if (samples_in_obstacles_.find(num) == samples_in_obstacles_.end()) {
    //         difference.insert(num);
    //     }
    // }
    // std::vector<Eigen::VectorXd> positions0;
    // for (const auto& y: difference) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(y)->getStateVlaue();
    //     positions0.push_back(vec);
    // }
    // visualization_->visualizeNodes(positions0,"map");

    // std::vector<Eigen::VectorXd> positions;

    // TODO: maybe an easier way instead of solving looping the over the v_unvisted_set thorugh tracking is to loop over the v_unvisted that are their heuristic is less than the current robots costToRoot! --> or if that doesnt work we can use the tracking that i used in python!
    for (int node : v_unvisited_set_) {
        auto neighbors = near(node);
        for (const auto& neighbor : neighbors) {
            if (v_open_set_.count(neighbor.index) == 0 &&
                v_unvisited_set_.count(neighbor.index) == 0 &&
                samples_in_obstacles_.count(neighbor.index) == 0 ) {
                if (tree_[neighbor.index]->getCost() == std::numeric_limits<double>::infinity()) { //TODO: Think about this --> i guess since you clear the vunvisted you gotta use cost inf to avoid putting thme in vOpen instead of vunsietd check in the above if condition --> think about this more! --> because later when you want to add early exit you might not even clear the vunvisted so this might be usesless later! --> maybe think about what should be in vOpen! --> the nodes that cleary have a cost other than inf!
                    continue; //TODO: the reason why some vunvisted remains that got not connected and also they are not promising but just pure vunvisted (have cost of inf) --> it means on the last pahse they got put in the vunvisted in the handle add obstalce! but later in the plan function they didn't get connected --> but you may ask why they didn't get connected?
                } //TODO: continuation of the above comment --> the reason it happens is this --> imagine a scenraio that you have removed nodes that gets into v unvisted but all the vOpen are not on samples on obstacles! so that v unvisted doest get the chance to get connected to any thing else!
                
                v_open_set_.insert(neighbor.index);
                v_open_heap_.push({tree_[neighbor.index]->getCost(), neighbor.index});
                
                if (tree_[neighbor.index]->getCost() == std::numeric_limits<double>::infinity()) {
                    std::cout<<neighbor.index<< "\n";
                    std::cout<<"why \n";
                    // Eigen::VectorXd vec(2);
                    // vec << tree_.at(neighbor.index)->getStateVlaue();
                    // positions.push_back(vec);
                }
            }
        }
    }

    // visualization_->visualizeNodes(positions,"map");

    // std::vector<Eigen::VectorXd> positions;
    // for (const auto& y: v_unvisited_set_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(y)->getStateVlaue();
    //     positions.push_back(vec);
    // }
    // std::string color_str = "1.0,1.0,0.0"; // Blue color
    // visualization_->visualizeNodes(positions,"map",color_str);

    // std::vector<Eigen::VectorXd> positions2;
    // for (const auto& y: v_open_set_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(y)->getStateVlaue();
    //     positions2.push_back(vec);
    // }
    // visualization_->visualizeNodes(positions2,"map");
    plan(); //lets put it outside!

    // std::vector<Eigen::VectorXd> samples_in_obstalce_position;
    // for (const auto& sample : samples_in_obstacles_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(sample)->getStateVlaue();
    //     samples_in_obstalce_position.push_back(vec);
    // }

    // Visualize obstacles in RViz
    // visualization_->visualizeNodes(samples_in_obstalce_position, "map");


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

// std::unordered_set<int> FMTX::getDescendants(int node_index) {
//     std::unordered_set<int> descendants;
//     std::queue<int> queue;
//     queue.push(node_index);

//     // Debugging: Track the number of iterations
//     int iteration_count = 0;
//     const int max_iterations = 3000; // Set a reasonable threshold

//     while (!queue.empty()) {
//         // Check if the loop is running for too long
//         if (iteration_count > max_iterations) {
//             std::cerr << "WARNING: Potential cyclic bug detected in getDescendants!\n";
//             std::cerr << "Current node: " << queue.front() << "\n";
//             std::cerr << "Descendants collected so far: " << descendants.size() << "\n";
//             std::cerr << "Breaking out of the loop to avoid infinite execution.\n";
//             // break; // Exit the loop to prevent infinite execution
//         }

//         int current = queue.front();
//         queue.pop();
//         descendants.insert(current);

//         for (int child : tree_[current]->getChildrenIndices()) {
//             queue.push(child);
//         }

//         iteration_count++; // Increment the iteration counter
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

        // Invalidate the node


        // Remove the node from v_open_set and add it to v_unvisited_set
        // v_open_set_.erase(idx);
    }

    v_unvisited_set_.insert(orphan_nodes.begin() , orphan_nodes.end());
    // Step 2: Update the tree structure
    for (int orphan : orphan_nodes) {
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
    }

    // // Step 3: Reconnect orphaned nodes if possible
    // for (int orphan : orphan_nodes) {
    //     auto neighbors = near(orphan);
    //     double min_cost = std::numeric_limits<double>::infinity();
    //     int best_parent = -1;

    //     for (const auto& neighbor : neighbors) {
    //         if (obs_checker_->isObstacleFree(
    //             tree_[neighbor.index]->getStateVlaue(), 
    //             tree_[orphan]->getStateVlaue()
    //         )) {
    //             double cost = tree_[neighbor.index]->getCost() + neighbor.distance;
    //             if (cost < min_cost) {
    //                 min_cost = cost;
    //                 best_parent = neighbor.index;
    //             }
    //         }
    //     }
    // }

    // // Step 3: Add valid nodes to v_open_set_ and v_open_heap_
    // for (int node : v_unvisited_set_) {
    //     auto neighbors = near(node);
    //     for (const auto& neighbor : neighbors) {
    //         if (v_open_set_.count(neighbor.index) == 0 &&
    //             v_unvisited_set_.count(neighbor.index) == 0 &&
    //             orphan_nodes.count(neighbor.index) == 0 &&
    //             samples_in_obstacles_.count(neighbor.index) == 0) {
    //             v_open_set_.insert(neighbor.index);
    //             v_open_heap_.push({tree_[neighbor.index]->getCost(), neighbor.index});
    //             if (tree_[neighbor.index]->getCost() == std::numeric_limits<double>::infinity())
    //             {
    //                 std::cout<<neighbor.index<< "\n";
    //                 std::cout<<"why \n";
    //             }
    //         }
    //     }
    // }


    // // Step 4: Visualization before planning

    // // Visualize v_open_set_ (Green Nodes)
    // std::vector<Eigen::VectorXd> open_set_positions;
    // for (const auto& node : v_open_set_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(node)->getStateVlaue();
    //     open_set_positions.push_back(vec);
    // }
    // visualization_->visualizeNodes(open_set_positions, "map");

    // // Visualize v_unvisited_set_ (Orange Nodes)
    // std::vector<Eigen::VectorXd> unvisited_positions;
    // for (const auto& node : v_unvisited_set_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(node)->getStateVlaue();
    //     unvisited_positions.push_back(vec);
    // }
    // visualization_->visualizeNodes(unvisited_positions, "map");




    // plan again!;
    // plan();
}



// TODO: THis function shoudln't try to reconnect the nodes but it works weirdly!
void FMTX::handleRemovedObstacleSamples(const std::vector<int>& removed) {

    // TODO: is erase necessary???
//     for (auto it = samples_in_obstacles_.begin(); it != samples_in_obstacles_.end(); ++it) {
//     v_unvisited_set_.erase(*it);  // Only erase each element if it exists in v_unvisited_set_
// }
    v_unvisited_set_.insert(removed.begin() , removed.end());


    // // Step 1: Revalidate nodes that are no longer in obstacles
    // for (int idx : removed) {
    //     // Check if the node is still valid
    //     if (!obs_checker_->isObstacleFree(
    //         tree_[idx]->getStateVlaue(), 
    //         tree_[idx]->getStateVlaue() // Self-check
    //     )) {
    //         continue;
    //     }

    //     // Reconnect the node to the tree
    //     auto neighbors = near(idx);
    //     double min_cost = std::numeric_limits<double>::infinity();
    //     int best_parent = -1;

    //     for (const auto& neighbor : neighbors) {
    //         if (obs_checker_->isObstacleFree(
    //             tree_[neighbor.index]->getStateVlaue(), 
    //             tree_[idx]->getStateVlaue()
    //         )) {
    //             double cost = tree_[neighbor.index]->getCost() + neighbor.distance;
    //             if (cost < min_cost) {
    //                 min_cost = cost;
    //                 best_parent = neighbor.index;
    //             }
    //         }
    //     }

    //     if (best_parent != -1) {
    //         tree_[idx]->setParentIndex(best_parent);
    //         tree_[idx]->setCost(min_cost);
    //         v_open_set_.insert(idx);
    //         v_open_heap_.push({min_cost, idx});
    //     }

    //     // Remove the node from v_unvisited_set
    //     v_unvisited_set_.erase(idx);
    // }

    // // Step 2: Update v_open_set and v_unvisited_set
    // for (int idx : removed) {
    //     if (v_unvisited_set_.find(idx) != v_unvisited_set_.end()) {
    //         v_unvisited_set_.erase(idx);
    //     }
    // }


    // for (int node : v_unvisited_set_) {
    //     auto neighbors = near(node);
    //     for (const auto& neighbor : neighbors) {
    //         if (v_open_set_.count(neighbor.index) == 0 &&
    //             v_unvisited_set_.count(neighbor.index) == 0 &&
    //             samples_in_obstacles_.count(neighbor.index) == 0 ) {
    //             v_open_set_.insert(neighbor.index);
    //             v_open_heap_.push({tree_[neighbor.index]->getCost(), neighbor.index});
                
    //             if (tree_[neighbor.index]->getCost() == std::numeric_limits<double>::infinity()) {
    //                 std::cout<<neighbor.index<< "\n";
    //                 std::cout<<"why \n";
    //             }
    //         }
    //     }
    // }

    // // std::make_heap(v_open_heap_.begin(), v_open_heap_.end(), std::greater<>{}); // Equivalent to heapq.heapify(vOpen) in Python

    // for (int node : v_unvisited_set_) {
    //     auto neighbors_info = near(v, v[node], rn, Obs, tree); // Assuming near() returns a vector of neighbor information
    //     for (const auto& neighbor : neighbors_info) {
    //         int neighbor_index = neighbor.index;
    //         if (v_open_set_.count(neighbor_index) == 0 &&
    //             v_unvisited_set_.count(neighbor_index) == 0 &&
    //             samples_in_obstacle.count(neighbor_index) == 0) {
    //             // Add to the open set and heap
    //             double cost = tree_[neighbor_index]->getCost(); // Equivalent to costToInit[neighbor_index] in Python
    //             v_open_heap_.push_back({cost, neighbor_index}); // Equivalent to vOpen.append(...) in Python
    //             v_open_set_.insert(neighbor_index); // Equivalent to vOpen_set.add(...) in Python
    //         }
    //     }
    // }

    // // Heapify v_open_heap_ to maintain the heap property



}