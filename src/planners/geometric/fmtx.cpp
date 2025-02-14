// Copyright 2025 Soheil E.nia
#include "motion_planning/planners/geometric/fmtx.hpp"

FMTX::FMTX(std::unique_ptr<StateSpace> statespace ,std::unique_ptr<ProblemDefinition> problem_def) :  statespace_(std::move(statespace)), problem_(std::move(problem_def)) {
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
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "---\n";


}

void FMTX::plan() {
    std::cout<< "Plan FMTX \n";
    auto start = std::chrono::high_resolution_clock::now();

    while (! v_open_heap_.empty()) {
        auto [cost, zIndex] = v_open_heap_.top();
        v_open_heap_.pop();

        auto zNeighborsInfo = near(zIndex);
        for (const auto& [xIndex, cost_to_neighbor]: zNeighborsInfo) {

            //if xIndex on samples_in_obstalces_ then you should continue but i haven't implemented that yet
            if (v_unvisited_set_.find(xIndex) != v_unvisited_set_.end()) {
                // std::cout<<"Proces \n";
            } else if (tree_.at(xIndex)->getCost() > tree_.at(zIndex)->getCost() + cost_to_neighbor) {
                v_unvisited_set_.insert(xIndex);

                if (v_open_set_.find(tree_.at(xIndex)->getParentIndex()) == v_open_set_.end()) {
                    auto parent_of_xIndex = tree_.at(xIndex)->getParentIndex();
                    v_open_heap_.push({tree_.at(parent_of_xIndex)->getCost() , parent_of_xIndex});
                    v_open_set_.insert(parent_of_xIndex);
                }
                tree_.at(xIndex)->setCost(std::numeric_limits<double>::infinity());
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

            //obstcle check! later!
            bool obstalce_check = true;
            if (obstalce_check) {
                double newCost = min_cost;
                if (newCost < tree_.at(xIndex)->getCost()) {
                    // if (parent if not -1 then delete the children! because its rewiring) --> later

                    tree_.at(xIndex)->setCost(newCost);
                    if (v_open_set_.find(xIndex) == v_open_set_.end()) {
                        v_open_heap_.push({newCost , xIndex});
                        v_open_set_.insert(xIndex);
                    }
                    v_unvisited_set_.erase(xIndex);
                    tree_.at(xIndex)->setParentIndex(best_neighbor_index);
                    tree_.at(xIndex)->setChildrenIndex(xIndex);
                }
            }

            //Early termination --> later


        }

        v_open_set_.erase(zIndex);
        if (v_unvisited_set_.find(zIndex) != v_unvisited_set_.end()) {
            v_unvisited_set_.erase(zIndex);
        }

    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by while loop: " << duration.count() << " milliseconds\n";
    std::cout << "The End \n";


    // Write tree nodes to text file after planning is complete
    std::ofstream output_file("tree_nodes.txt");

    if (output_file.is_open()) {
        for (size_t i = 0; i < tree_.size(); ++i) {
            auto node = tree_[i];
            int parentIndex = node->getParentIndex();
            Eigen::VectorXd state = node->getStateVlaue();  // Assuming this gives you the position (2D)
            double x = state[0], y = state[1];

            // Write the node index, parent index, and position to the file
            output_file << i << ", " << parentIndex << ", " << x << ", " << y << std::endl;
        }

        output_file.close();
        std::cout << "Tree data written to tree_nodes.txt" << std::endl;
    } else {
        std::cerr << "Failed to open file for writing!" << std::endl;
    }
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

void FMTX::setStart(const Eigen::VectorXd& start) {
    robot_state_index_ = statespace_->getNumStates();
    tree_.push_back(std::make_shared<TreeNode>(statespace_->addState(start)));
    std::cout << "FMTX: Start node created on Index: " << robot_state_index_ << "\n";
}
void FMTX::setGoal(const Eigen::VectorXd& goal) {
    root_state_index_ = statespace_->getNumStates();
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
    visualization_->visualizeNodes(nodes);
    visualization_->visualizeEdges(edges);
}
