// Copyright 2025 Soheil E.nia
#include "motion_planning/planners/geometric/informed_any_fmta.hpp"

InformedANYFMTA::InformedANYFMTA(std::shared_ptr<StateSpace> statespace ,std::shared_ptr<ProblemDefinition> problem_def, std::shared_ptr<ObstacleChecker> obs_checker) :  statespace_(statespace), problem_(problem_def), obs_checker_(obs_checker) {
    std::cout<< "ANY FMT Constructor \n";

}

void InformedANYFMTA::clearPlannerState() {

    for (auto& node : tree_) {
        node->disconnectFromGraph();
        node.reset();  
    }
    tree_.clear();
    statespace_->reset();
    kdtree_tree_.reset();
    kdtree_samples_.reset();
    v_open_heap_.clear();
    samples_.clear();
    open_nodes.clear();
    root_state_index_ = -1;
    robot_state_index_ = -1;
    obstacle_check_cache.clear(); // This is needed for any time algs of the fmt variants!
    collision_check_ = 0;

}


void InformedANYFMTA::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    auto start = std::chrono::high_resolution_clock::now();
    clearPlannerState();
    visualization_ = visualization;
    num_of_samples_ = params.getParam<int>("num_of_samples");
    num_batch_ = params.getParam<int>("num_batch");
    partial_plot = params.getParam<bool>("partial_plot");
    obs_cache = params.getParam<bool>("obs_cache");
    lower_bound_ = problem_->getLowerBound();
    upper_bound_ = problem_->getUpperBound();
    use_kdtree = params.getParam<bool>("use_kdtree");
    std::string kdtree_type = params.getParam<std::string>("kdtree_type");
    if (use_kdtree == true && kdtree_type == "NanoFlann"){
        kdtree_samples_ = std::make_shared<NanoFlann>(statespace_->getDimension());
        kdtree_tree_ = std::make_shared<NanoFlann>(statespace_->getDimension());
    }
    else{
        throw std::runtime_error("Unknown KD-Tree type");
    }

    std::cout << "num_of_samples=" << num_of_samples_
                << ", bounds=[" << lower_bound_ << ", " << upper_bound_ << "]\n";


    std::cout << "Taking care of the samples: \n \n";
    setGoal(problem_->getGoal());
    setStart(problem_->getStart());
    // for (int i = 0 ; i < num_of_samples_; i++) {  // BUT THIS DOESNT CREATE A TREE NODE FOR START AND GOAL !!!
    //     auto node = std::make_shared<IFMTNode>(statespace_->sampleUniform(lower_bound_ , upper_bound_),tree_.size());
    //     tree_.push_back(std::move(node));
    // }

    // Precompute heuristics for all existing nodes
    // for (auto& n : tree_) {
    //     double h = (n->getStateValue() - robot_node_->getStateValue()).norm();
    //     n->cacheHeuristic(h);
    // }



    // std::cout << "KDTree: \n\n";
    // if (use_kdtree == true) {
    //     // Put all the points at once because FMT doesnt need incremental addition
    //     kdtree_->addPoints(statespace_->getSamplesCopy());
    //     // Build the tree all at once after we fill the data_ in the KDTree
    //     kdtree_->buildTree();
    // }

    ///////////////////Neighborhood Radius////////////////////////////////
    d = statespace_->getDimension();
    mu = std::pow(problem_->getUpperBound() - problem_->getLowerBound() , 2);
    zetaD = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
    gamma = 2 * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d);
    factor = 1.0;
    /*
        to understand this imagine you have two nodes (start and goal), then what should be the neighbor hood radius so that these two see each other! the formula makes sense!
    */

    int n  = tree_.size() + samples_.size();
    neighborhood_radius_ = factor * gamma * std::pow(std::log(n) / n, 1.0 / d);
    max_edge_length_ = 15;
    neighborhood_radius_ = std::min(neighborhood_radius_,max_edge_length_);
    // neighborhood_radius_ = 5.0;
    std::cout << "Computed value of rn: " << neighborhood_radius_ << std::endl;
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "---\n";





}

// void InformedANYFMTA::plan() {

//     // visualizeHeapAndUnvisited();

//     int uncached = 0;
//     int cached = 0;
//     int checks = 0;
//     /*
//         YOU NEED TO PRUNE THE SAMPLES_ ALSO IN THE PRUNE *********************
    
//     */
//     if(v_open_heap_.empty()){
//         prune();
//         pruneSamples();

//         if(robot_node_->getCost()==INFINITY)
//             addBatchOfSamplesUninformed(num_batch_);
//         else
//             addBatchOfSamples(num_batch_);
//     } 

//     // auto start = std::chrono::high_resolution_clock::now();
//     // std::cout<<v_open_heap_.getHeap().size()<<"\n";
//     std::cout<<tree_.size()<<"\n";

//     while (!v_open_heap_.empty() ){
//     // while (!v_open_heap_.empty()){

//         // addBatchOfSamplesUninformed(num_batch_);

//         auto top_element = v_open_heap_.top();
//         double priority = top_element.first;
//         std::shared_ptr<IFMTNode> z = top_element.second;
//         int zIndex = z->getIndex();
//         // std::vector<Eigen::VectorXd> nodes;
//         // nodes.push_back(z->getStateValue());
//         // visualization_->visualizeNodes(nodes);

//         // std::vector<std::shared_ptr<IFMTNode>> tree_samples_ = tree_;
//         // tree_samples_.insert(tree_samples_.end(), samples_.begin(), samples_.end());

//         /*
//             sorting makes it so that at first the best heursitc edges gets processed and those might get to the goal sooner and update the ci_(robotnode)
//             and then after that many edges could be avoided because of the if condtion and it saves collision checks!
//             since im not using sorting and process edges randomly then the first few of them might get processes and collision checked uselessly! 

//         */
//         if (priority > robot_node_->getCost()){ // DOES THIS CAUSE ISSUE FOR REWIRING????????*****************
//             v_open_heap_.clear();
//             break;
//         }

//         std::vector<std::pair<std::shared_ptr<IFMTNode>,EdgeInfo >> near_z;
//         std::vector<std::pair<std::shared_ptr<IFMTNode>,EdgeInfo >> near_z2;
//         near2sample(z,near_z);
//         near2tree(z,near_z2);


//         std::set<std::shared_ptr<IFMTNode>> added_nodes;
//         // Result vector for merged elements
//         std::vector<std::pair<std::shared_ptr<IFMTNode>, EdgeInfo>> merged;

//         // Function to add pairs without duplication
//         auto add_if_unique = [&](const std::pair<std::shared_ptr<IFMTNode>, EdgeInfo>& pair) {
//             if (added_nodes.insert(pair.first).second) { // Insert and check if the node was already added
//                 merged.push_back(pair);
//             }
//         };

//         // Merge the vectors
//         for (const auto& pair : near_z) {
//             add_if_unique(pair);
//         }
//         for (const auto& pair : near_z2) {
//             add_if_unique(pair);
//         }


//         for (const auto& [x, cost_to_neighbor] : merged){
//             int xIndex = x->getIndex(); // As I refactor the code I don't need to use xIndex anymore but I still need som refactoring.
//             if (z->getCost() + cost_to_neighbor.distance + x->getHeuristic() < robot_node_->getCost()){
//                 if (x->getCost() > (z->getCost() + cost_to_neighbor.distance ) ){



//                     double min_cost = x->getCost();
//                     std::shared_ptr<IFMTNode> best_neighbor_node = nullptr;
//                     double best_edge_length = 0.0;

//                     std::vector<std::pair<std::shared_ptr<IFMTNode>,EdgeInfo >> near_x;
//                     near2tree(x,near_x);
//                     for (const auto& [neighbor, dist] : near_x ) {
//                         // if(x->blocked_best_neighbors.count(neighbor->getIndex()) > 0)
//                         //     continue;
//                         if (neighbor->in_queue_) {
//                             const double total_cost = neighbor->getCost() + dist.distance;
//                             if (total_cost < min_cost) {
//                                 min_cost = total_cost;
//                                 best_neighbor_node = neighbor;
//                                 best_edge_length = dist.distance;
//                             }
//                         }
//                     }


//                     if (!best_neighbor_node) {
//                         continue;
//                     }

//                     /*
//                         Now we check the real connection
//                     */
//                     if (best_neighbor_node->getCost() + cost_to_neighbor.distance + x->getHeuristic() > robot_node_->getCost()){
//                         continue;
//                     }


//                     int best_neighbor_index = best_neighbor_node->getIndex();
//                     bool obstacle_free;
//                     obstacle_free = obs_checker_->isObstacleFree(x->getStateValue() , best_neighbor_node->getStateValue());


//                     if (obstacle_free) {
//                         double newCost = min_cost;
//                         if (newCost < x->getCost()) {
//                             // x->blocked_best_neighbors.clear(); // Well if x is connected then i don't care about neighbors that can't be connected so what a better place to clearing them than here. this is for when you use heuristic
//                             x->setCost(newCost);

//                             // double h_value =  heuristic(xIndex);
//                             double h_value = x->getHeuristic();
//                             // double h_value = 0;

//                             double priorityCost = newCost + h_value;

//                             checks++;
//                             // v_open_heap_.add(x,priorityCost);
                                    // x->in_queue_ = true;
//                             if (x!=robot_node_){
//                                 if (x->in_queue_ == true){
//                                     v_open_heap_.update(x,priorityCost);
//                                 } else{
//                                     v_open_heap_.add(x,priorityCost);
                                        // x->in_queue_ = true;
//                                 }
//                             }

//                             x->setParent(best_neighbor_node,best_edge_length); 
                            
//                             tree_.push_back(x);
//                             if (x->in_samples_) {
//                                 // Get index BEFORE any modifications
//                                 const size_t idx = x->samples_index_;
                                
//                                 // Update KD-tree first
//                                 kdtree_samples_->removeByIndex(idx);
                                
//                                 kdtree_samples_->buildTree();
//                                 // Then update samples vector
//                                 if (idx != samples_.size() - 1) {
//                                     std::swap(samples_.at(idx), samples_.back());
//                                     samples_.at(idx)->samples_index_ = idx;
//                                 }
//                                 samples_.pop_back();
                                
//                                 // Update remaining indices
//                                 for (size_t i = idx; i < samples_.size(); ++i) {
//                                     samples_.at(i)->samples_index_ = i;
//                                 }
                                
//                                 // Update node state
//                                 x->in_samples_ = false;
//                                 x->samples_index_ = -1;

//                                 //////////////////////////////
//                                 //     // Convert samples to Eigen matrix
//                                 // Eigen::MatrixXd samples_matrix(samples_.size(), 2);
//                                 // for (size_t i = 0; i < samples_.size(); ++i) {
//                                 //     samples_matrix.row(i) = samples_.at(i)->getStateValue();
//                                 // }
//                                 // std::cout<<"samples_" <<samples_matrix<<"\n";
//                                 // // Compare with KD-tree's data
//                                 // if (!kdtree_samples_->validateAgainstSamples(samples_)) {
//                                 //     std::cerr << "KD-tree validation failed!\n";
//                                 //     throw std::runtime_error("KD-tree data mismatch");
//                                 // }
//                             }
                          
//                             kdtree_tree_->addPoint(x->getStateValue());
//                             kdtree_tree_->buildTree();
//                         }
//                     }

//                 }
//             } 
//         }


        

//         v_open_heap_.pop();
//         // open_nodes.erase( std::remove_if( open_nodes.begin(), open_nodes.end(), [z](const std::shared_ptr<IFMTNode>& node) {
//         //             return node.get() == z;  // Compare raw pointers
//         //         }
//         //     ),
//         //     open_nodes.end()
//         // );

//         // visualizeTree();
//     }




//     // auto end = std::chrono::high_resolution_clock::now();
//     // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//     // std::cout << "time taken for  : " << duration.count() << " milliseconds\n";    

//     std::cout<<"checks: "<< checks <<"\n";
//     std::cout<<"cached: "<< cached <<"\n";
//     std::cout<<"uncached: "<< uncached <<"\n";
//     std::cout<<"cost: "<<robot_node_->getCost()<<"\n";

//     std::ofstream cost_file("robot_costs_2_ifmta.txt", std::ios::app);  // Open file in append mode
//     if (cost_file.is_open()) {
//         double cost = robot_node_->getCost();
//         // double cost = checks;
//         cost_file << cost << "\n";  // Write the cost to the file
//         std::cout << "Cost saved: " << cost << std::endl;
//     } else {
//         std::cerr << "Unable to open file for writing costs." << std::endl;
//     }

//     cost_file.close();  // Close the file after writing

// }



void InformedANYFMTA::plan() {
    if(std::abs(robot_node_->getCost() - tree_[root_state_index_]->getHeuristic()) <0.01){
        return;
    }
    // processed_edges_.clear();
    // duplicate_checks_ = 0;
    // total_checks_ = 0;


    // visualizeHeapAndUnvisited();

    int uncached = 0;
    int cached = 0;
    int checks = 0;
    /*
        YOU NEED TO PRUNE THE SAMPLES_ ALSO IN THE PRUNE *********************
    
    */
    if(v_open_heap_.empty()){
        prune();
        pruneSamples();
        for (auto& sample : samples_) {
            sample->is_new_ = false;
        }
        if(robot_node_->getCost()==INFINITY)
            addBatchOfSamplesUninformed(num_batch_);
        else
            addBatchOfSamples(num_batch_);

        
        const double UNEXPAND_RESET_PROBABILITY = 0.3; // 30% chance to clear unexpand_

        for (auto& node : tree_) {
            // if (node->unexpand_ && (rand() / (double)RAND_MAX) < UNEXPAND_RESET_PROBABILITY) {
            //     node->unexpand_ = false;
            // }
            // double priority = node->getCost() + node->getHeuristic();

            // double priority = node->getHeuristic();

            // if(node->unexpand_==true){
            //     std::cout<<"why \n";
            // }

            // node->is_new_ = false;
            double priority;
            if(robot_node_->getCost() == INFINITY)
                priority =  node->getHeuristic();
            else
                priority = node->getCost() + node->getHeuristic();

            v_open_heap_.add(node, priority);
            node->in_queue_ = true;
        }


    } 

    // auto start = std::chrono::high_resolution_clock::now();
    // std::cout<<v_open_heap_.getHeap().size()<<"\n";
    std::cout<<"tree: "<<tree_.size()<<"\n";
    std::cout<<"samples: "<<samples_.size()<<"\n";
    std::cout<<"cost: "<<robot_node_->getCost()<<"\n";
    std::cout<<"r: "<<neighborhood_radius_<<"\n";
    std::cout<<"Collision check: "<<collision_check_<<"\n";
    if(tree_.size()==155 && samples_.size()==80){
        std::cout<<"start \n";
    }

    int cc = 0;
    while (!v_open_heap_.empty() ){
        cc++;
    // while (!v_open_heap_.empty()){

        // addBatchOfSamplesUninformed(num_batch_);

        auto top_element = v_open_heap_.top();
        double priority = top_element.first;
        std::shared_ptr<IFMTNode> z = top_element.second;
        int zIndex = z->getIndex();
        v_open_heap_.pop();
    new_opens_.clear();

        // std::vector<Eigen::VectorXd> nodes;
        // nodes.push_back(z->getStateValue());
        // visualization_->visualizeNodes(nodes);

        // std::vector<std::shared_ptr<IFMTNode>> tree_samples_ = tree_;
        // tree_samples_.insert(tree_samples_.end(), samples_.begin(), samples_.end());

        /*
            sorting makes it so that at first the best heursitc edges gets processed and those might get to the goal sooner and update the ci_(robotnode)
            and then after that many edges could be avoided because of the if condtion and it saves collision checks!
            since im not using sorting and process edges randomly then the first few of them might get processes and collision checked uselessly! 

        */
        if (priority>robot_node_->getCost() || z->getCost() + z->getHeuristic() > robot_node_->getCost()){ // DOES THIS CAUSE ISSUE FOR REWIRING????????*****************
            std::cout<<"SALAMMMMMMMMMMM:"<<v_open_heap_.getHeap().size() << "\n";
            // for (const auto& entry : v_open_heap_.getHeap()) {
            //     // double priority = entry.first;
            //     // if (priority < robot_node_->getCost()) {
            //     //     std::cout<<"NO WAAAAAAAAYYYYY \n" ;// At least one node is still relevant
            //     // }
            //     entry.second->unexpand_ = false;
            // }


            v_open_heap_.clear();




            break;
        }

        std::vector<std::pair<std::shared_ptr<IFMTNode>, EdgeInfo>> near_z_samples;
        near2sample(z, near_z_samples);  // Get neighbors from samples


        int counter = 0;
        // First loop: Process samples (potential new connections)
        // bool should_delete_ = false;
        for (const auto& [x, cost_to_neighbor] : near_z_samples) {
            
            if (!z->unexpand_ && !x->is_new_) {
                // x->is_new_ = false;
                counter++;
                continue;
            }
            // Skip edges that can't improve the solution
            if (z->getCost() + cost_to_neighbor.distance + x->getHeuristic() > robot_node_->getCost() ) {
            // if (z->getGHat() + cost_to_neighbor.distance + x->getHeuristic() > robot_node_->getCost() && z->unexpand_==false) {
                // should_delete_ = true;
                // int a = 0;
                // for (auto& node : tree_) {
                //     a++;
                //     if(node->unexpand_==true){
                //         std::cout<<"why \n";
                //     }
                // }
                // std::cout<<"clean \n";
                // v_open_heap_.clear();        
                // return;

                // continue;
                break; // Exit loop early (sorted edges)
            }
            processNode(x, z, cost_to_neighbor, true,new_opens_);
            // x->is_new_ = false; // not correct to do it here because is_new_ should be true for the whole duration of this new batch when ever its being called --> so i make them false later when i loop thorugh the samples_. also not all of these new samples gets to be in near2samples output so they remain unconnected but later we have to loop thorugh the samples and make them false i guess
        }
        // if(counter>0)
        //     std::cout<<"couter: "<<counter<<"\n";


        
        // if(z->unexpand_==true && robot_node_->getCost()!=INFINITY){ //&& z->is_new_==false){
        if(z->unexpand_==true){
            std::vector<std::pair<std::shared_ptr<IFMTNode>, EdgeInfo>> near_z_tree;
            near2tree(z, near_z_tree);  // Get neighbors from tree

            
            for (const auto& [x, cost_to_neighbor] : near_z_tree) {
                if (z->getCost() + cost_to_neighbor.distance + x->getHeuristic() > robot_node_->getCost()) {
                // if (z->getGHat() + cost_to_neighbor.distance + x->getHeuristic() > robot_node_->getCost()) {
                //     break; // Exit loop early (sorted edges)
                    // continue;
                    // x->unexpand_ = false;
                    break;
                }
                // if(x->unexpand_==false)
                    processNode(x, z, cost_to_neighbor, false,new_opens_);
                // break;
            }
            z->unexpand_ =false;
        }



        // if(should_delete_){
        //     v_open_heap_.clear();        
        //     return; 
        // }

        z->in_queue_ = false;
        // // Update priority queue (common logic)
        // for (auto open_: new_opens_){
        //     // double priorityCost = open_->getCost() + open_->getHeuristic();
        //     double priorityCost = open_->getCost();  // MAKING IT TO HAVE HIGHER PRIORITY BECAUSE I NEED REWIREING!!! AND v_open_heap.clear() messes thing up
        //     // if (open_ != robot_node_) {
        //         if (open_->in_queue_) {
        //             v_open_heap_.update(open_, priorityCost);
        //         } else {
        //             v_open_heap_.add(open_, priorityCost);
        //             open_->in_queue_ = true;
        //         }
        //     // }

        // }


        // open_nodes.erase( std::remove_if( open_nodes.begin(), open_nodes.end(), [z](const std::shared_ptr<IFMTNode>& node) {
        //             return node.get() == z;  // Compare raw pointers
        //         }
        //     ),
        //     open_nodes.end()
        // );

        // visualizeTree();
    }
    std::cout<<"ccccccccccc : "<<cc<<"\n";


    // for (auto& node : tree_) {
    //     if(node->unexpand_==true){
    //         std::cout<<"why \n";
    //     }
    // }

    // Add debug summary at end
    std::cout << "\n--- Collision Check Report ---\n"
              << "Total checks: " << total_checks_ << "\n"
              << "Duplicate edges: " << duplicate_checks_ << " ("
              << (duplicate_checks_ * 100.0 / total_checks_) << "%)\n"
              << "Unique edges checked: " << processed_edges_.size() << "\n"
              << "Tree size: " << tree_.size() << "\n"
              << "Sample size: " << samples_.size() << "\n";

    
}




// Helper function to handle common logic
void InformedANYFMTA::processNode(
    std::shared_ptr<IFMTNode> x,
    std::shared_ptr<IFMTNode> z,
    const EdgeInfo& cost_to_neighbor,
    bool is_sample_neighbor,
    std::vector<std::shared_ptr<IFMTNode>>& new_opens_
) {
    if (z->getCost() + cost_to_neighbor.distance + x->getHeuristic() >= robot_node_->getCost()){
        // std::cout<<"now check the next samples priority see if they have more cost!";
        // std::cout<<"priority: "<<z->getStateValue()<<" " <<z->getCost() + cost_to_neighbor.distance + x->getHeuristic()<<"\n";
        return;
    }
    if (x->getCost() <= z->getCost() + cost_to_neighbor.distance) return;

    // Find best neighbor (common logic)
    double min_cost = x->getCost();
    std::shared_ptr<IFMTNode> best_neighbor_node = nullptr;
    double best_edge_length = 0.0;

    std::vector<std::pair<std::shared_ptr<IFMTNode>, EdgeInfo>> near_x;
    near2tree(x, near_x);  // Always check tree for best neighbors



    for (const auto& [neighbor, dist] : near_x) {
        // if(x->blocked_best_neighbors.count(neighbor) > 0) // Didn't lower the collision checks attempts!
        //     continue;

        if (neighbor->in_queue_ && (neighbor->getCost() + dist.distance < min_cost)) {
            min_cost = neighbor->getCost() + dist.distance;
            best_neighbor_node = neighbor;
            best_edge_length = dist.distance;
        }
    }

    if (!best_neighbor_node) return;

    if (best_neighbor_node->getCost() + cost_to_neighbor.distance + x->getHeuristic() > robot_node_->getCost()){
        return;
    }

    // Generate unique edge ID (sorted to avoid directional duplicates)
    auto make_edge_id = [](auto a, auto b) {
        return std::make_pair(std::min(a->getUniqueId(), b->getUniqueId()),
                         std::max(a->getUniqueId(), b->getUniqueId()));
    };

    // Track edge processing BEFORE collision check
    auto edge_id = make_edge_id(x, best_neighbor_node);
    
    // Debug: Check if edge was already processed
    if (processed_edges_.count(edge_id)) {
        std::cout << "DUPLICATE EDGE: " 
                  << x->getStateValue().transpose() << " <-> "
                  << best_neighbor_node->getStateValue().transpose()
                  << " | Total duplicates: " << ++duplicate_checks_ <<"| "<<x->getCost() <<" vs " <<min_cost <<"\n";
    } else {
        processed_edges_.insert(edge_id);
    }

    collision_check_++;
    total_checks_++;



    // Collision check (common logic)
    if (!obs_checker_->isObstacleFree(x->getStateValue(), best_neighbor_node->getStateValue())){
        // x->blocked_best_neighbors.insert(best_neighbor_node);
        return;
    }

    // Update costs and structures
    if (min_cost < x->getCost()) {
        double old_cost = x->getCost();
        // x->setCost(min_cost); // Don't use this because updateCostAndPropagate won't get trigered with this!!
        x->setParent(best_neighbor_node, best_edge_length);
        x->updateCostAndPropagate();

        // Handle sample-specific operations
        if (is_sample_neighbor && x->in_samples_) {
            // Remove from samples and update KD-tree
            const size_t idx = x->samples_index_;
            kdtree_samples_->removeByIndex(idx);
            
            if (idx != samples_.size() - 1) {
                std::swap(samples_.at(idx), samples_.back());
                samples_.at(idx)->samples_index_ = idx;
            }
            samples_.pop_back();
            
            // Update remaining indices
            for (size_t i = idx; i < samples_.size(); ++i) {
                samples_.at(i)->samples_index_ = i;
            }
            
            x->unexpand_ = true; // THERE IS DIFFERENT IF YOU DO IT HERE OR DOING IT BELOW THE CONDTIONS --> from full rewring vs one step rewiring (relying on samples to make optimal tree)
            x->in_samples_ = false;
            x->samples_index_ = -1;
            x->is_new_ = false; // since we delete x from samples_ we do not have access to it through the samples_ any more so looping through the samples_ later to make it false doesnt make this specific node false! even though it doesnt matter because this node is out of the picture i guess!
            kdtree_samples_->buildTree();
        }

        // Add to tree KD-tree if not already there
        if (is_sample_neighbor) {
            tree_.push_back(x);
            kdtree_tree_->addPoint(x->getStateValue());
            kdtree_tree_->buildTree();
        }
        new_opens_.push_back(x);
        // x->unexpand_ = true;
        if(x==robot_node_){
            std::cout<<"FOUND SOLUTION \n";
            v_open_heap_.clear();
        }


        // Update priority queue (common logic)
        // double priorityCost = x->getCost() + x->getHeuristic();
        double priorityCost;
        if(robot_node_->getCost() == INFINITY)
            priorityCost =  x->getHeuristic();
        else
            priorityCost = x->getCost() + x->getHeuristic();
            
        // double priorityCost = x->getCost(); // with this you can put the unexpanded nodes on higher levels of the queue (not necessarily higher than some of the expanded nodes but some will be up) --> but do i need to do this?? why not rely on the g+h ??
        // if (x != robot_node_) {
            if (x->in_queue_) {
                v_open_heap_.update(x, priorityCost);
            } else {
                v_open_heap_.add(x, priorityCost);
                    x->in_queue_ = true;
            }
        // }


        // if(x->unexpand_==true){
        //     std::vector<std::pair<std::shared_ptr<IFMTNode>, EdgeInfo>> near_x_tree;
        //     near2tree(x, near_x_tree);  // Get neighbors from tree

            
        //     for (const auto& [xx, cost_to_neighbor] : near_x_tree) {
        //         if (x->getCost() + cost_to_neighbor.distance + xx->getHeuristic() > robot_node_->getCost()) {
        //         // if (z->getGHat() + cost_to_neighbor.distance + x->getHeuristic() > robot_node_->getCost()) {
        //         //     break; // Exit loop early (sorted edges)
        //             // continue;
        //             break;
        //         }
        //         processNode(xx, x, cost_to_neighbor, false,new_opens_);
        //         // break;
        //     }
        //     x->unexpand_ =false;
        // }



        // v_open_heap_.removeIf([](const auto& node) {
        //     return node->getCost() + node->getHeuristic() >= robot_node_->getCost();
        // });


    }



}





void InformedANYFMTA::near2sample( const std::shared_ptr<IFMTNode>& node, std::vector<std::pair<std::shared_ptr<IFMTNode>,EdgeInfo >>& near_nodes) {
    near_nodes.clear();  // Reset output vector
    
    // Perform radius search using kdtree_tree_
    std::vector<size_t> neighbor_indices = 
        kdtree_samples_->radiusSearch(node->getStateValue(), neighborhood_radius_);
    
    
    // Fill results in-place
    near_nodes.reserve(neighbor_indices.size());
    for (size_t idx : neighbor_indices) {
        auto neighbor = samples_.at(idx);
        double dist = (neighbor->getStateValue() - node->getStateValue()).norm();

        if (neighbor != node) {  // Exclude the query node itself
            near_nodes.push_back(std::make_pair(neighbor,EdgeInfo{dist,dist}));
        }
    }

    //maybe also do a sorting!!!!******************************** based on just gvalue(node ) --> because i only need the ordering and current vertex is enough to preserve ordering IM SURE OF IT EVEN THOUGH I NEED TO PROVE IT!
    // Sort the collected nodes by (node_cost + dist + neighbor_heuristic)
    std::sort(
        near_nodes.begin(),
        near_nodes.end(),
        [node_cost = node->getCost()](const auto& a, const auto& b) {
            // Calculate sorting keys
            const double key_a = node_cost + a.second.distance + a.first->getHeuristic();
            const double key_b = node_cost + b.second.distance + b.first->getHeuristic();
            return key_a < key_b; // Ascending order (lowest first)
        }
    );

    
    // std::sort(
    // near_nodes.begin(),
    // near_nodes.end(),
    // [node_cost = node->getCost(), current_best = robot_node_->getCost()](const auto& a, const auto& b) {
    //     const double f_a = node_cost + a.second.distance + a.first->getHeuristic();
    //     const double f_b = node_cost + b.second.distance + b.first->getHeuristic();
    //     // Sort by absolute difference from current_best (ascending)
    //     return std::abs(f_a - current_best) < std::abs(f_b - current_best);
    // }
    // );


}

void InformedANYFMTA::near2tree( const std::shared_ptr<IFMTNode>& node, std::vector<std::pair<std::shared_ptr<IFMTNode>,EdgeInfo >>& near_nodes) {
    near_nodes.clear();  // Reset output vector
    
    // Perform radius search using kdtree_tree_
    std::vector<size_t> neighbor_indices = 
        kdtree_tree_->radiusSearch(node->getStateValue(), neighborhood_radius_);
    
    // Fill results in-place
    near_nodes.reserve(neighbor_indices.size());
    for (size_t idx : neighbor_indices) {
        auto neighbor = tree_[idx];
        double dist = (neighbor->getStateValue() - node->getStateValue()).norm();
        if (neighbor != node) {  // Exclude the query node itself
            near_nodes.push_back(std::make_pair(neighbor,EdgeInfo{dist,dist}));
        }
    }
    // std::sort(near_nodes.begin(), near_nodes.end(), [](const auto& a, const auto& b) {
    //     // New samples first, then old ones
    //     if (a.first->is_new_ != b.first->is_new_) return a.first->is_new_;
    //     return a.second.distance < b.second.distance;
    // });

    // should we prioritize new samples first???
    std::sort(
        near_nodes.begin(),
        near_nodes.end(),
        [node_cost = node->getCost()](const auto& a, const auto& b) {
            // Calculate sorting keys
            const double key_a = node_cost + a.second.distance + a.first->getHeuristic();
            const double key_b = node_cost + b.second.distance + b.first->getHeuristic();
            return key_a < key_b; // Ascending order (lowest first)
        }
    );

    // std::sort(near_nodes.begin(), near_nodes.end(),
    //     [node](const auto& a, const auto& b) {
    //         // Key = potential cost reduction if rewired
    //         double key_a = a.first->getCost() - (node->getCost() + a.second.distance);
    //         double key_b = b.first->getCost() - (node->getCost() + b.second.distance);
    //         return key_a > key_b; // Descending order (most improvement first)
    //     }
    // );



}



double InformedANYFMTA::heuristic(int current_index) {
    Eigen::VectorXd current_position = tree_.at(current_index)->getStateValue();
    Eigen::VectorXd goal_position = tree_.at(robot_state_index_)->getStateValue();
    return (goal_position-current_position).norm();
}

void InformedANYFMTA::prune() {
    // Sort tree nodes by cost (descending)
    std::vector<std::shared_ptr<IFMTNode>> sorted_nodes = tree_;
    std::sort(sorted_nodes.begin(), sorted_nodes.end(),
        [](const auto& a, const auto& b) { return a->getCost() > b->getCost(); });

    // Prune tree nodes and handle dependencies
    std::vector<std::shared_ptr<IFMTNode>> to_remove;
    for (auto& node : sorted_nodes) {
        double f_hat = node->getHeuristic() + (node->getStateValue() - tree_.at(root_state_index_)->getStateValue()).norm();
        if (f_hat > robot_node_->getCost() || node->getCost() + node->getHeuristic() > robot_node_->getCost()) {
            to_remove.push_back(node);
            
            // // Remove from parent's children (using lock() for weak_ptr)
            // if (auto parent = node->getParent()) {
            //     auto& siblings = parent->getChildrenMutable();
            //     siblings.erase(
            //         std::remove_if(siblings.begin(), siblings.end(),
            //             [&node](const std::weak_ptr<IFMTNode>& weak_child) {
            //                 auto child = weak_child.lock();
            //                 return child && child == node;
            //             }),
            //         siblings.end());
            // }

            node->unexpand_ = false;

            // Add to reuse if meets criteria
            if (f_hat < robot_node_->getCost()) {
                node->in_samples_ = true;
                // node->unexpand_ = true; // Not sure about this! --> samples will end up in unexpanded if they are good no need to rush it!

                node->samples_index_ = samples_.size();
                samples_.push_back(node);
                kdtree_samples_->addPoint(node->getStateValue());
                kdtree_samples_->buildTree();
            }


        }
    }

    // Remove from main containers
    auto remove_from = [&](auto& container) {
        container.erase(std::remove_if(container.begin(), container.end(),
            [&](const auto& n) { 
                return std::find(to_remove.begin(), to_remove.end(), n) != to_remove.end(); 
            }),
            container.end());
    };
    
    remove_from(tree_);
    /*
        make this better later!
    */
    kdtree_tree_->clear();
    for (const auto& node : tree_) {
        kdtree_tree_->addPoint(node->getStateValue());
    }
    kdtree_tree_->buildTree();
 

    
}

// void InformedANYFMTA::prune() {
//     // Create vector of nodes with their current indices
//     std::vector<std::pair<std::shared_ptr<IFMTNode>, size_t>> indexed_nodes;
//     for (size_t i = 0; i < tree_.size(); ++i) {
//         indexed_nodes.emplace_back(tree_[i], i);
//         tree_[i]->setIndex(i); // Update node's index
//     }

//     // Sort nodes by cost descending
//     std::sort(indexed_nodes.begin(), indexed_nodes.end(),
//         [](const auto& a, const auto& b) { 
//             return a.first->getCost() > b.first->getCost(); 
//         });

//     std::vector<size_t> to_remove_indices;
//     for (const auto& pair : indexed_nodes) {
//         auto& node = pair.first;
//         size_t original_index = pair.second;

//         double f_hat = node->getHeuristic() + 
//                       (node->getStateValue() - tree_[root_state_index_]->getStateValue()).norm();
//         if (f_hat > robot_node_->getCost() || 
//             node->getCost() + node->getHeuristic() > robot_node_->getCost()) 
//         {
//             to_remove_indices.push_back(original_index);

//             // Remove from parent's children
//             if (auto parent = node->getParent()) {
//                 auto& siblings = parent->getChildrenMutable();
//                 siblings.erase(
//                     std::remove_if(siblings.begin(), siblings.end(),
//                         [&node](const std::weak_ptr<IFMTNode>& weak_child) {
//                             auto child = weak_child.lock();
//                             return child && child == node;
//                         }),
//                     siblings.end());
//             }
//         }
//     }

//     // Sort indices in descending order for safe removal
//     std::sort(to_remove_indices.rbegin(), to_remove_indices.rend());

//     // Remove from KD-tree using original indices
//     for (auto idx : to_remove_indices) {
//         kdtree_tree_->removeByIndex(idx);
//     }

//     // Remove from tree_ vector with swap-and-pop
//     for (auto idx : to_remove_indices) {
//         if (idx >= tree_.size()) continue;
        
//         // Swap with last element
//         if (idx != tree_.size() - 1) {
//             std::swap(tree_[idx], tree_.back());
//             // Update swapped node's index
//             tree_[idx]->setIndex(idx);
//         }
//         tree_.pop_back();
//     }

//     // Update indices for remaining nodes
//     for (size_t i = 0; i < tree_.size(); ++i) {
//         tree_[i]->setIndex(i);
//     }

//     // Rebuild KD-tree once
//     kdtree_tree_->buildTree();
// }


void InformedANYFMTA::pruneSamples() {
    const double current_best_cost = robot_node_->getCost();
    std::vector<size_t> to_remove_indices;
    std::cout << "Before prune: " << samples_.size() << "\n";

    // Step 1: Identify samples to remove (in reverse order for safe deletion)
    for (size_t i = samples_.size(); i-- > 0; ) {
        auto& sample = samples_.at(i);
        double f_value = (sample->getStateValue() - tree_[root_state_index_]->getStateValue()).norm() + 
                         (sample->getStateValue() - robot_node_->getStateValue()).norm();
        
        if (f_value >= current_best_cost) {
            to_remove_indices.push_back(i);
        }
    }

    // Sort indices in descending order to handle removal safely
    std::sort(to_remove_indices.rbegin(), to_remove_indices.rend());

    // Step 2: Batch remove from samples_ and KD-tree
    for (auto idx : to_remove_indices) {
        // Get the sample to remove
        auto& sample = samples_.at(idx);
        
        // Mark as not in samples
        sample->in_samples_ = false;
        sample->samples_index_ = -1;

        // Remove from KD-tree using the current index
        kdtree_samples_->removeByIndex(idx);

        // Remove from samples_ vector using swap-and-pop
        if (idx != samples_.size() - 1) {
            std::swap(samples_.at(idx), samples_.back());
            // Update the swapped node's index
            samples_.at(idx)->samples_index_ = idx;
        }
        samples_.pop_back();
    }

    // Rebuild KD-tree after all removals
    kdtree_samples_->buildTree();

    // Verify consistency
    if (samples_.size() != kdtree_samples_->size()) {
        std::cout << "Size mismatch after pruning: " 
                  << samples_.size() << " vs " << kdtree_samples_->size() << "\n";
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void InformedANYFMTA::addBatchOfSamples(int num_samples) {
    if (num_samples == 0)
        return;
    std::vector<int> added_nodes;


    // Get nodes and positions
    std::shared_ptr<IFMTNode> goal_node = tree_.at(root_state_index_);
    std::shared_ptr<IFMTNode> start_node = robot_node_;
    Eigen::VectorXd start_pos = start_node->getStateValue();
    Eigen::VectorXd goal_pos = goal_node->getStateValue();

    // Calculate ellipsoid parameters
    // Eigen::VectorXd d_vec = goal_pos - start_pos;  // Direction from start to goal
    Eigen::VectorXd d_vec = start_pos - goal_pos;
    double c_min = d_vec.norm();
    double c_best = start_node->getCost();
    Eigen::VectorXd center = (start_pos + goal_pos) / 2.0;  // Midpoint

    // Compute rotation matrix (align major axis with start->goal direction)
    Eigen::MatrixXd R = computeRotationMatrix(d_vec);
    double a = c_best / 2.0;
    double b = std::sqrt(std::max(c_best*c_best - c_min*c_min, 0.0)) / 2.0;    // Add samples

    const double solution_cost = robot_node_->getCost();
    const bool has_solution = solution_cost < INFINITY;
    // std::vector<Eigen::VectorXd> nodes;

    for (int i = 0; i < num_samples; ++i) {
        // Generate sample in ellipsoid
        Eigen::VectorXd sample = sampleInEllipsoid(center, R, a, b);

        // // ======== Stage 1: Geometric Feasibility Check ======== ----> I think ellipsoid already does this!
        // if (has_solution) {
        //     // Quick straight-line lower bounds
        //     const double g_lower = (sample - start_pos).norm();
        //     const double h_lower = (sample - goal_pos).norm();
            
        //     if (g_lower + h_lower > solution_cost ) {
        //         std::cout<<"wwwwwwwwwwww \n";
        //         continue; // Skip geometrically hopeless samples
        //     }
        // }

        double h = (sample - robot_node_->getStateValue() ).norm();
        if (h + (sample - tree_.at(root_state_index_)->getStateValue()).norm()  > robot_node_->getCost())
            continue;

        if (!obs_checker_->isObstacleFree(sample)) 
            continue;

        // nodes.push_back(sample);





        auto node = std::make_shared<IFMTNode>(statespace_->addState(sample), tree_.size());

        // Cache heuristic for new node

        node->cacheHeuristic(h);
        node->cacheGHat((sample - tree_.at(root_state_index_)->getStateValue()).norm());
        size_t node_index = tree_.size();
        // tree_.push_back(std::move(node));
        node->in_samples_ = true;
        node->is_new_ = true;
        node->samples_index_ = samples_.size();
        samples_.push_back(node);
        // if (use_kdtree) {
        //     kdtree_->addPoint(sample);
        // }
        kdtree_samples_->addPoint(sample);

        added_nodes.push_back(node_index);
    }

    if (added_nodes.empty()) return;
    kdtree_samples_->buildTree();

    // visualization_->visualizeNodes(nodes);

    // Rebuild KD-tree and update radius
    // if (use_kdtree) {
    //     kdtree_->buildTree();
    // }

    double n = tree_.size() + samples_.size();
    neighborhood_radius_ = factor * gamma * std::pow(std::log(n) / n, 1.0 / d);
    neighborhood_radius_ = std::min(neighborhood_radius_,max_edge_length_);



    // // Process neighbors for newly added nodes
    // for (const auto& node : samples_) {
    //     // Update neighbors in the heap
    //     std::vector<std::pair<std::shared_ptr<IFMTNode>,EdgeInfo >> near_node;
    //     near2tree(node,near_node);
    //     for (const auto& [neighbor, dist] : near_node ) {
    //         const int n_idx = neighbor->getIndex();
    //         if (neighbor->in_queue_ || neighbor->getCost() == INFINITY) continue;
    //         double h_value = neighbor->getHeuristic();
    //         double priorityCost = neighbor->getCost() + h_value;

    //         // v_open_heap_.add(neighbor.get(), priorityCost);
                    // neighbor->in_queue_ = true;
    //         // open_nodes.push_back(neighbor);

    //         if (neighbor->in_queue_ == true){
    //             v_open_heap_.update(neighbor,priorityCost);
    //         } else{
    //             v_open_heap_.add(neighbor,priorityCost);
                    // neighbor->in_queue_ = true;
    //         }

    //     }
    // }


    // visualizeHeapAndUnvisited();
}

Eigen::MatrixXd InformedANYFMTA::computeRotationMatrix(const Eigen::VectorXd& dir) {
    Eigen::VectorXd u = dir.normalized();
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dir.size(), dir.size());
    A.col(0) = u;
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(A);
    Eigen::MatrixXd Q = qr.householderQ();
    if (Q.col(0).dot(u) < 0) Q.col(0) *= -1;
    return Q;
}


/*
    In informed sampling, the ellipsoid should be centered at the midpoint between start and goal, with the major axis aligned from start to goal.
*/
Eigen::VectorXd InformedANYFMTA::sampleInEllipsoid(const Eigen::VectorXd& center,
                                                const Eigen::MatrixXd& R,
                                                double a, double b) {
    // Generate in unit ball
    Eigen::VectorXd y = sampleUnitBall(center.size());
    
    // Scale to ellipsoid
    Eigen::VectorXd y_scaled(y.size());
    y_scaled[0] = a * y[0];
    for(int i = 1; i < y.size(); ++i) 
        y_scaled[i] = b * y[i];
    
    // Rotate and translate to world frame
    return R * y_scaled + center;
}

Eigen::VectorXd InformedANYFMTA::sampleUnitBall(int dim) {
    // static std::random_device rd;
    // static std::mt19937 gen(rd());

    static std::mt19937 gen(12345);
    static std::normal_distribution<double> normal(0, 1);
    static std::uniform_real_distribution<double> uniform(0, 1);

    Eigen::VectorXd v(dim);
    double norm = 0;
    do {
        for (int i = 0; i < dim; ++i) {
            v[i] = normal(gen);
        }
        norm = v.norm();
    } while (norm == 0);
    
    v.normalize();
    double r = pow(uniform(gen), 1.0 / dim);
    return r * v;
}

void InformedANYFMTA::updateNeighbors(int node_index) {
    auto node = tree_.at(node_index);
    
    // Clear existing neighbors if needed (optional)
    // node->neighbors().clear();

    auto indices = kdtree_tree_->radiusSearch(node->getStateValue(), neighborhood_radius_);
    
    for(int idx : indices) {
        if(idx == node->getIndex()) continue;
        
        std::shared_ptr<IFMTNode> neighbor = tree_.at(idx);
        const double dist = (node->getStateValue() - neighbor->getStateValue()).norm();

        // Add neighbor to current node's list if not already present
        if(!node->neighbors().contains(neighbor)) {
            node->neighbors().emplace(neighbor, EdgeInfo{dist, dist});
        }

        // Add reverse connection to neighbor's list if not present
        if(!neighbor->neighbors().contains(node)) {
            neighbor->neighbors().emplace(node, EdgeInfo{dist, dist});
        }
    }
}



void InformedANYFMTA::addBatchOfSamplesUninformed(int num_samples) {
    if (num_samples==0)
        return;
    std::vector<int> added_nodes;
    const size_t start_index = tree_.size();

    // Add samples one by one and update KD-tree incrementally
    for (int i = 0; i < num_samples; ++i) {
        // Create new node
        Eigen::VectorXd sample = Eigen::VectorXd::Random(d);
        sample = lower_bound_ + (upper_bound_ - lower_bound_) * (sample.array() + 1) / 2;

        double h = (sample - robot_node_->getStateValue()).norm();
        if (h + (sample - tree_.at(root_state_index_)->getStateValue()).norm()  > robot_node_->getCost())
            continue;
        if (!obs_checker_->isObstacleFree(sample)) 
            continue;

        auto node = std::make_shared<IFMTNode>(statespace_->addState(sample), tree_.size());
        size_t node_index = tree_.size(); // Get index BEFORE pushing
        // Store the new sample
        Eigen::VectorXd new_sample = node->getStateValue();
        


        node->cacheHeuristic(h);
        node->cacheGHat((sample - tree_.at(root_state_index_)->getStateValue()).norm());
        
        // Add to tree and KD-tree
        // tree_.push_back(std::move(node));
        node->in_samples_ = true;
        node->is_new_ = true;
        node->samples_index_ = samples_.size();
        samples_.push_back(node);

        added_nodes.push_back(node_index);
        kdtree_samples_->addPoint(sample);

    }
    if(added_nodes.empty())
        return;

    kdtree_samples_->buildTree();
    // Update neighborhood radius
    double n = tree_.size() + samples_.size();
    neighborhood_radius_ = factor * gamma * std::pow(std::log(n) / n, 1.0 / d);
    neighborhood_radius_ = std::min(neighborhood_radius_,max_edge_length_);

    
    // // Process neighbors for newly added nodes
    // for (const auto& node : samples_) {
    //     // Update neighbors in the heap
    //     std::vector<std::pair<std::shared_ptr<IFMTNode>,EdgeInfo >> near_node;
    //     near2tree(node, near_node);
    //     for (const auto& [neighbor, dist] : near_node) {
    //         const int n_idx = neighbor->getIndex();
    //         if (neighbor->in_queue_ || neighbor->getCost() == INFINITY) continue;
    //         double h_value = neighbor->getHeuristic();
    //         double priorityCost = neighbor->getCost() + h_value;

    //         // v_open_heap_.add(neighbor.get(), priorityCost);
                    // neighbor->in_queue_ = true;
    //         // open_nodes.push_back(neighbor);

    //         if (neighbor->in_queue_ == true){
    //             v_open_heap_.update(neighbor,priorityCost);
    //         } else{
    //             v_open_heap_.add(neighbor,priorityCost);
                    // neighbor->in_queue_ = true;
    //         }



    //     }
    // }

}


// std::vector<size_t> InformedANYFMTA::getPathIndex() const {
//     int idx = robot_state_index_;
//     std::vector<size_t> path_index;

//     while (idx != -1) {
//         path_index.push_back(idx);

//         std::shared_ptr<IFMTNode> parent = tree_.at(idx)->getParent();
//         if (!parent) break;

//         idx = parent->getIndex();
//     }

//     return path_index;
// }

std::vector<Eigen::VectorXd> InformedANYFMTA::getPathPositions() const {
    std::vector<Eigen::VectorXd> path_positions;

    if (robot_node_ != nullptr) {
        path_positions.push_back(robot_node_->getStateValue());
    }

    std::shared_ptr<IFMTNode> current_node = robot_node_;

    // Traverse the tree from the robot's node to the root
    while (current_node != nullptr) {
        path_positions.push_back(current_node->getStateValue());
        current_node = current_node->getParent();
    }

    return path_positions;
}

void InformedANYFMTA::setRobotIndex(const Eigen::VectorXd& robot_position) {
    robot_position_ = robot_position;

    const double MAX_SEARCH_RADIUS = 5.0; // Meters
    std::vector<size_t> nearest_indices = kdtree_tree_->radiusSearch(robot_position, MAX_SEARCH_RADIUS);

    size_t best_index = std::numeric_limits<size_t>::max(); 
    double min_total_cost = std::numeric_limits<double>::max();
    std::shared_ptr<IFMTNode> best_node;

    for (size_t index : nearest_indices) {
        auto node = tree_.at(index);
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
        std::vector<size_t> nearest_indices = kdtree_tree_->knnSearch(robot_position, 1);
        int nearest = nearest_indices.empty() ? -1 : static_cast<int>(nearest_indices[0]);  
        robot_node_ = tree_.at(nearest);
        robot_state_index_ = robot_node_->getIndex();
        return;
    }

    robot_state_index_ = -1;
    robot_node_ = nullptr; 
    std::cout << "No valid node found and no previous robot_node_. Setting robot_node_ to nullptr.\n";
}

void InformedANYFMTA::setStart(const Eigen::VectorXd& start) {
    auto node = std::make_shared<IFMTNode>(statespace_->addState(start),tree_.size());
    root_state_index_ = node->getIndex();
    node->setCost(0);
    double h = (node->getStateValue() - robot_node_->getStateValue()).norm();
    node->cacheHeuristic(h);
    node->cacheGHat(0);
    node->unexpand_=true;

    // node->in_queue_ = true;
    v_open_heap_.add(node, node->getCost() + node->getHeuristic());
    node->in_queue_  = true;
    open_nodes.push_back(node);
    tree_.push_back(node);
    kdtree_tree_->addPoint(node->getStateValue());
    kdtree_tree_->buildTree();

    std::cout << "InformedANYFMTA: Start node created on Index: " << robot_state_index_ << "\n";
}
void InformedANYFMTA::setGoal(const Eigen::VectorXd& goal) {
    robot_state_index_ = statespace_->getNumStates();
    auto node = std::make_shared<IFMTNode>(statespace_->addState(goal),-1);
    robot_node_ = node; // Management of the node variable above will be done by the unique_ptr i'll send to tree_ below so robot_node_ is just using it!
    // tree_.push_back(std::move(node));
    samples_.push_back(node);
    node->in_samples_ = true;
    node->is_new_ = true;
    node->samples_index_ = 0;
    kdtree_samples_->addPoint(node->getStateValue());
    kdtree_samples_->buildTree();
    std::cout << "InformedANYFMTA: Goal node created on Index: " << root_state_index_ << "\n";
}


std::vector<Eigen::VectorXd> InformedANYFMTA::getSmoothedPathPositions(int num_intermediates, int smoothing_passes) const {
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


std::vector<Eigen::VectorXd> InformedANYFMTA::interpolatePath(const std::vector<Eigen::VectorXd>& path, int num_intermediates) const {
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


std::vector<Eigen::VectorXd> InformedANYFMTA::smoothPath(const std::vector<Eigen::VectorXd>& path, int window_size) const {
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

// void InformedANYFMTA::visualizeTree() {
//     if (partial_plot==true) {
//         std::vector<Eigen::VectorXd> nodes;
//         std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
//         double goal_node_cost = tree_.at(robot_state_index_)->getCost();
        
//         // Create a set to store valid nodes based on cost
//         std::unordered_set<int> valid_node_indices;

//         // Collect valid nodes
//         for (size_t i = 0; i < tree_.size(); ++i) {
//             if (tree_[i]->getCost() <= goal_node_cost) {
//                 nodes.push_back(tree_[i]->getStateValue());
//                 valid_node_indices.insert(i);
//             }
//         }

//         // Generate edges only for valid nodes
//         for (int index : valid_node_indices) {
//             int parent_index = tree_[index]->getParent()->getIndex();
//             if (parent_index != -1) {
//                 edges.emplace_back(tree_.at(parent_index)->getStateValue(), tree_.at(index)->getStateValue());
//             }
//         }
//         // Visualize nodes and edges
//         // visualization_->visualizeNodes(nodes);
//         visualization_->visualizeEdges(edges);
//     }
//     else {
//         std::vector<Eigen::VectorXd> tree_nodes;
//         std::vector<Eigen::VectorXd> vopen_positions;
//         std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;

    
//         for (const auto& tree_node : tree_) {
//             // Collect all tree nodes
//             tree_nodes.push_back(tree_node->getStateValue());

//             // Check if node is in vopen (in_queue_)
//             if (tree_node->in_queue_) {
//                 vopen_positions.push_back(tree_node->getStateValue());
//             }

//             // Collect edges
//             auto parent = tree_node->getParent();
//             if (parent) {
//                 edges.emplace_back(parent->getStateValue(), tree_node->getStateValue());
//             }
//         }
    
//         // // Visualize tree components
//         // visualization_->visualizeNodes(tree_nodes, "map", 
//         //                             std::vector<float>{1.0f, 0.0f, 0.0f},  // Red for tree
//         //                             "tree_nodes");
        
//         // // Visualize vopen nodes with different color/namespace
//         // visualization_->visualizeNodes(vopen_positions);

//         // Visualize edges
//         visualization_->visualizeEdges(edges, "map");
//     }


// }

// void InformedANYFMTA::visualizePath(std::vector<size_t> path_indices) {
//     std::vector<Eigen::VectorXd> nodes;
//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;

//     for (const auto& index : path_indices) {
//         nodes.push_back(tree_.at(index)->getStateValue());
//     }

//     for (const auto& index : path_indices) {
//         std::shared_ptr<IFMTNode> parent = tree_.at(index)->getParent();
//         if (parent) {
//             edges.emplace_back(parent->getStateValue(), tree_.at(index)->getStateValue());
//         }
//     }

//     // visualization_->visualizeNodes(nodes);
//     visualization_->visualizeEdges(edges, "map", "0.0,1.0,0.0");
// }


void InformedANYFMTA::visualizeSmoothedPath(const std::vector<Eigen::VectorXd>& shortest_path_) {
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


void InformedANYFMTA::visualizeHeapAndUnvisited() {
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

    visualization_->visualizeNodes(vopen_positions);
}


std::vector<std::shared_ptr<IFMTNode>> InformedANYFMTA::getPathNodes() const {
    std::vector<std::shared_ptr<IFMTNode>> path_nodes;
    auto current = robot_node_;
    
    while (current) {
        path_nodes.push_back(current);
        current = current->getParent();
    }
    return path_nodes;
}



void InformedANYFMTA::visualizePath(const std::vector<std::shared_ptr<IFMTNode>>& path_nodes) {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;

    for (const auto& node : path_nodes) {
        if (auto parent = node->getParent()) {
            edges.emplace_back(parent->getStateValue(), node->getStateValue());
        }
    }
    visualization_->visualizeEdges(edges, "map", "0.0,1.0,0.0");
}

void InformedANYFMTA::visualizeTree() {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    std::vector<Eigen::VectorXd> nodes_tree_;
    std::vector<Eigen::VectorXd> nodes_sample_;
    for (const auto& node : tree_) {
        if (auto parent = node->getParent()) {
            edges.emplace_back(parent->getStateValue(), node->getStateValue());
        }
        nodes_tree_.push_back(node->getStateValue());
    }

    for (const auto& node : samples_) {
        nodes_sample_.push_back(node->getStateValue());
    }
    // visualization_->visualizeNodes(nodes_tree_,"map",std::vector<float>{0.0,1.0,0.0} , "nodes_tree");
    visualization_->visualizeNodes(nodes_sample_,"map",std::vector<float>{0.0,0.0,1.0} , "samples_tree");
    visualization_->visualizeEdges(edges, "map");
}



