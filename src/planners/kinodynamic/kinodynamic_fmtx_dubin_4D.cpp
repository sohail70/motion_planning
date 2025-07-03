// Copyright 2025 Soheil E.nia
#include "motion_planning/planners/kinodynamic/kinodynamic_fmtx.hpp"

KinodynamicFMTX::KinodynamicFMTX(std::shared_ptr<StateSpace> statespace ,std::shared_ptr<ProblemDefinition> problem_def, std::shared_ptr<ObstacleChecker> obs_checker) :  statespace_(statespace), problem_(problem_def), obs_checker_(obs_checker) {
    std::cout<< "KinodynamicFMTX Constructor \n";

}

void KinodynamicFMTX::clearPlannerState() {

    v_open_heap_.clear(); //This function also makes the in_queue false so remember to clean it before deleting the node by node.reset()
    // // Step 1: Nullify all raw pointers --> or else if you only use tree_.clear() you have dangling pointers for parent_ and children_ that do not exist now!
    for (auto& node : tree_) {
        node->disconnectFromGraph();
        node.reset();  
    }
    tree_.clear();
    statespace_->reset();
    kdtree_.reset();
    // samples_in_obstacles_.clear();
    edge_length_.clear();
    max_length_edge_ind = -1;
    max_length = -std::numeric_limits<double>::infinity();
    root_state_index_ = -1;
    robot_state_index_ = -1;

}


void KinodynamicFMTX::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    auto start = std::chrono::high_resolution_clock::now();
    clearPlannerState();
    visualization_ = visualization;
    num_of_samples_ = params.getParam<int>("num_of_samples");
    partial_update = params.getParam<bool>("partial_update");
    use_heuristic= params.getParam<bool>("use_heuristic");
    partial_plot = params.getParam<bool>("partial_plot");
    static_obs_presence = params.getParam<bool>("static_obs_presence");
    ignore_sample = params.getParam<bool>("ignore_sample");
    prune = params.getParam<bool>("prune");
    obs_cache = params.getParam<bool>("obs_cache");
    lower_bounds_ = problem_->getLowerBound();
    upper_bounds_ = problem_->getUpperBound();
    use_kdtree = params.getParam<bool>("use_kdtree");
    std::string kdtree_type = params.getParam<std::string>("kdtree_type");



    if (use_kdtree == true && kdtree_type == "NanoFlann"){
        // Assuming DubinsTimeStateSpace which is 4D with weights
        Eigen::VectorXd weights(4);
        // weights << 1.0, 1.0, 0.4, 0.8; // Weights for x, y, theta, time
        weights << 1.0, 1.0, 1.0, 1.0; // Weights for x, y, theta, time
        kdtree_ = std::make_shared<WeightedNanoFlann>(statespace_->getDimension(), weights);
    } else if (use_kdtree == true && kdtree_type == "LieKDTree"){
        kdtree_ = std::make_unique<LieSplittingKDTree>(statespace_->getDimension(), statespace_);
    } else {
        throw std::runtime_error("FMTX requires a KD-Tree.");
    }
    std::cout << "num_of_samples=" << num_of_samples_
                << ", bounds=[" << lower_bounds_ << ", " << upper_bounds_ << "]\n";


    std::cout << "Taking care of the samples: \n \n";
    setStart(problem_->getStart());
    for (int i = 0 ; i < num_of_samples_; i++) {  // BUT THIS DOESNT CREATE A TREE NODE FOR START AND GOAL !!!
        auto node = std::make_shared<FMTNode>(statespace_->sampleUniform(lower_bounds_ , upper_bounds_),tree_.size());
        node->in_unvisited_ = true;
        tree_.push_back(node);
    }
    setGoal(problem_->getGoal());

    std::cout<<statespace_->getSamplesCopy()<<"\n";
    std::cout << "KDTree: \n\n";
    if (use_kdtree == true) {
        // Put all the points at once because fmtx doesnt need incremental addition
        kdtree_->addPoints(statespace_->getSamplesCopy());
        // Build the tree all at once after we fill the data_ in the KDTree
        kdtree_->buildTree();
    }

    ///////////////////Neighborhood Radius////////////////////////////////
    int d = statespace_->getDimension();
    // mu = std::pow(problem_->getUpperBound()[0] - problem_->getLowerBound()[0] , 2);
    Eigen::VectorXd range = upper_bounds_ - lower_bounds_;
    double mu = range.prod(); // .prod() computes the product of all coefficients
    std::cout<<"mu "<<mu<<"\n";
    zetaD = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
    // gamma = 2 * std::pow(1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); //Real FMT star gamma which is smaller than rrt star which makes the neighborhood size less than rrt star hence so much faster performance
    gamma = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d);


    
    factor = params.getParam<double>("factor");
    std::cout<<"factor: "<<factor<<"\n";
    neighborhood_radius_ = factor * gamma * std::pow(std::log(statespace_->getNumStates()) / statespace_->getNumStates(), 1.0 / d);
    // neighborhood_radius_ = 5.0;
    std::cout << "Computed value of rn: " << neighborhood_radius_ << std::endl;
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "---\n";





}


void KinodynamicFMTX::plan() {
    std::unordered_map<FMTNode*, bool> costUpdated;
    int checks = 0;
    while (!v_open_heap_.empty() &&
           (partial_update ? (v_open_heap_.top().first < robot_node_->getCost() ||
                               robot_node_->getCost() == INFINITY || robot_node_->in_queue_ == true) : true)) {

        // visualizeHeapAndUnvisited();
        // Get the node with the lowest cost from the priority queue.
        // FMTNode* z = v_open_heap_.top(); // .pop() also sets z->in_queue_ = false;

        auto top_element = v_open_heap_.top();
        double cost = top_element.first;  // Changed .min_key to .first
        FMTNode* z = top_element.second;  // Changed .index to .second
        int zIndex = z->getIndex();

        // Find neighbors for z if they haven't been found yet.
        near(z->getIndex());
        
        // --- STAGE 1: IDENTIFY POTENTIALLY SUBOPTIMAL NEIGHBORS ---
        // Iterate through all neighbors 'x' of the expanding node 'z'.
        for (auto& [x, edge_info_from_z] : z->neighbors()) {
            // if (zIndex==2 && x->getIndex()==3){
            //     std::cout<<"here \n";
            //     std::cout<<"z: "<<z->getStateValue()<<"\n";
            //     std::cout<<"x: "<<x->getStateValue()<<"\n";
            // } 
            // The edge we care about is from child 'x' to parent 'z' in our backward search.
            // The authoritative trajectory is stored in the child's (x's) map for that edge.
            near(x->getIndex()); // Ensure x's neighbor map is initialized.
            auto& edge_info_from_x = x->neighbors().at(z);

            // Compute the kinodynamic path only once and cache it.
            if (!edge_info_from_x.is_trajectory_computed) {
                edge_info_from_x.cached_trajectory = statespace_->steer(x->getStateValue(), z->getStateValue());
                edge_info_from_x.is_trajectory_computed = true;
            }
            const Trajectory& traj_xz = edge_info_from_x.cached_trajectory;
            if (!traj_xz.is_valid) {
                // std::cout<<"INVALID \n";
                continue;
            }

            // --- THE TRIGGER CONDITION ---
            // Calculate the potential cost for 'x' if it were to connect through 'z'.
            double cost_via_z = z->getCost() + traj_xz.cost;
            // if (!traj_xz.is_valid){
            //     cost_via_z = z->getCost() + edge_info_from_z.distance;
            // } 
            // This condition is the core of FMTX. It serves two purposes:
            // 1. If x has not been connected yet (cost is INF), this is always true, triggering its initial connection.
            // 2. If x is already connected, this condition acts as a "witness" that a better path *might* exist.
            //    It proves x's current cost is suboptimal and justifies the more expensive search that follows.
            if (x->getCost() > cost_via_z) {
                if (costUpdated[x]) {
                    // std::cout<<"Node " << x->getIndex() 
                    //     << " is about to be updated a second time! "
                    //     "previous cost = " << x->getCost() << "\n";
                    
                    checks++;

                } 
                // --- STAGE 2: SEARCH FOR THE TRUE BEST PARENT ---
                // 'x' is suboptimal. We now search for its true best parent among ALL its neighbors
                // that are currently in the open set.
                double min_cost_for_x = std::numeric_limits<double>::infinity();
                FMTNode* best_parent_for_x = nullptr;
                Trajectory best_traj_for_x;
                // std::cout<<x->getIndex()<<"\n";
                // if(x->getIndex()==3)
                //     std::cout<<"\n";
                // std::cout<<"----\n";
                for (auto& [y, edge_info_xy] : x->neighbors()) {
                    // std::cout<<y->getIndex()<<"\n";
                    // std::cout<<"----\n";
                    if (y->in_queue_) { // We only consider parents that are in V_open.
                        // Steer from child 'x' to potential parent 'y'. Reuse cached trajectory if possible.
                        if (!edge_info_xy.is_trajectory_computed) {
                            edge_info_xy.cached_trajectory = statespace_->steer(x->getStateValue(), y->getStateValue());
                            edge_info_xy.is_trajectory_computed = true;
                        }

                        if (edge_info_xy.cached_trajectory.is_valid) {
                            double cost_via_y = y->getCost() + edge_info_xy.cached_trajectory.cost;
                            if (cost_via_y < min_cost_for_x) {
                                min_cost_for_x = cost_via_y;
                                best_parent_for_x = y;
                                best_traj_for_x = edge_info_xy.cached_trajectory;
                            }
                        }
                    }
                }
                if (costUpdated[x]) {
                    // std::cout<<"Node " << x->getIndex() 
                    //     << "  updated a second time! "
                    //     "new cost = " << min_cost_for_x << "\n";
                }

                // --- STAGE 3: UPDATE (if a better parent was found) ---
                if (best_parent_for_x != nullptr) {
                    
                    // Perform the expensive collision check ONLY for the best candidate path.
                    bool obstacle_free = obs_checker_->isObstacleFree(best_traj_for_x.path_points);

                    if (obstacle_free) {
                        costUpdated[x] = true;   // mark “done once”
                        // // The connection is valid and locally optimal. Update the tree and priority queue.
                        // std::cout<<"-------\n";
                        // std::cout<<"index: "<<x->getIndex()<<"\n";
                        // std::cout<<"state: "<<x->getStateValue()<<"\n";
                        // std::cout<<"cost: "<<min_cost_for_x<<"\n";
                        // std::cout<<"-------\n";
                        x->setCost(min_cost_for_x);
                        x->setParent(best_parent_for_x, best_traj_for_x.cost);

                        double h_value = use_heuristic ? heuristic(x->getIndex()) : 0.0;
                        double priorityCost = x->getCost() + h_value;

                        if (x->in_queue_) {
                            v_open_heap_.update(x, priorityCost);
                        } else {
                            v_open_heap_.add(x, priorityCost); // add() also sets in_queue_ = true
                        }
                    }
                }
            } // End of STAGE 2/3 trigger
            // visualizeTree();
            // std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } // End of STAGE 1 loop
        v_open_heap_.pop();
        visualizeTree();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

    } // End of while loop
    std::cout<<"REVISITS: "<<checks<<"\n";
}


/*
    Near function at first uses kd tree and after that it caches node so it doesnt need to spend time on cache
    WHEN KD TREE PROVIDES DISTANCE WHY DO YOU CALC DISTS AGAIN IN BELOW! --> ALSO DO THIS FOR RRTX  ---->
    Im not gonna use distance of kd tree for now because i don't know about other kd tree implementations
*/
// void KinodynamicFMTX::near(int node_index) {
//     auto node = tree_[node_index].get();
//     if (!node->neighbors().empty()) return;

//     auto indices = kdtree_->radiusSearch(node->getStateValue(), neighborhood_radius_); 
//     for(int idx : indices) {
//         if(idx == node->getIndex()) continue;
//         FMTNode* neighbor = tree_[idx].get();
//         auto dist = (node->getStateValue() - neighbor->getStateValue()).norm();
//         node->neighbors()[neighbor] = EdgeInfo{dist,dist}; // The first is for distance and the second is for distance_original so that we have a cache incase we want to reset the obsoleted edges to correct distance (used in prune==true case)
//     }
// }


// Finds potential neighbors but does not calculate final cost.
void KinodynamicFMTX::near(int node_index) {
    auto node = tree_[node_index].get();
    if (node->neighbors_cached_) return;

    auto indices = kdtree_->radiusSearch(node->getStateValue(), neighborhood_radius_);
    for (int idx : indices) {
        if (idx == node_index) continue;
        FMTNode* neighbor = tree_[idx].get();
        // Initialize EdgeInfo without a computed trajectory.
        // The geometric distance is just a placeholder.
        double geometric_dist = (node->getStateValue().head<2>() - neighbor->getStateValue().head<2>()).norm();
        
        // Symmetrically add neighbor relationship
        node->neighbors()[neighbor] = EdgeInfo{geometric_dist, geometric_dist, false, Trajectory(), false};
        neighbor->neighbors()[node] = EdgeInfo{geometric_dist, geometric_dist, false, Trajectory(), false};

    }
    node->neighbors_cached_ = true;
}


/*
    When an obstalce appears on some node we rely on the position of the obstalce and its radius to (D-ball containing that obstalce)
    to find the nodes in the tree that are inside of that obstalce and we use the following formula to handle worst case scenraio that i explained in the plan function i guess
    std::sqrt(std::pow(obstacle.radius + obstacle.inflation , 2) + std::pow(max_length / 2.0, 2))
    this is minimum amount needed to cover all the potential colliding edges and it lower than rrtx paper and better
*/

std::unordered_set<int> KinodynamicFMTX::findSamplesNearObstacles(
    const ObstacleVector& obstacles, 
    double max_length
) {
    std::unordered_set<int> conflicting_samples;
    for (const auto& obstacle : obstacles) {
        /*
            when static_obs_presence is true. then use_range and all the static features also needs to be true
            mind that obstalce check still sees all the static obstalces that are out of range because i design the gazeboObstalceChecker this way (presistent_static_obstalce==true)
            but here we do not need to keep checking the static obstalces and put the surrounding neighbors in vopen!
            i could've done it in the gazebo obstalce checker but since i separated the obstalce_positions from their snapshots since the obstalce need to be fixed during the process of plan function then its a bit conflicting to handle this in that class
            these are all just handling corner cases in a simulation and separting dynamic and static obstalce and simulating a sensor range etc but not core to the algorthim it self.
        */ 
        if (static_obs_presence==true && !obstacle.is_dynamic) {
            // static: only if we haven't handled it before
            auto it = std::find(seen_statics_.begin(), seen_statics_.end(), obstacle);
            if (it != seen_statics_.end()) {
                // already processed this static obstacle → skip
                continue;
            }
            // first time we see this static, remember it
            seen_statics_.push_back(obstacle);
        }



        double obstacle_radius;
        if (obstacle.type == Obstacle::CIRCLE) {
            obstacle_radius = obstacle.dimensions.radius + obstacle.inflation;
        } else { // BOX
            // Calculate half diagonal of the box
            double half_diagonal = std::sqrt(
                std::pow(obstacle.dimensions.width/2, 2) + 
                std::pow(obstacle.dimensions.height/2, 2)
            );
            obstacle_radius = half_diagonal + obstacle.inflation;
        }
        
        double search_radius = std::sqrt(
            std::pow(obstacle_radius, 2) + 
            std::pow(max_length / 2.0, 2)
        );
        
        auto sample_indices = kdtree_->radiusSearch(obstacle.position, search_radius);
        conflicting_samples.insert(sample_indices.begin(), sample_indices.end());
    }
    return conflicting_samples;
}


/*
    This is using two circles with same center but different radius to find the nodes exactly on obstalce and nodes that are on surrounding using the above formula
    its still using the old api scale factor but since i don't wanna use it for now i kept the old api but you can use the above formula instead of scale factor
    the reason to use this is to make some part of the code faster because if you have distinction on what nodes are exactlyon obstalce and what are the nodes that are also on obstalce and surrouning then
    you can use that to your advantage
*/
std::pair<std::unordered_set<int>, std::unordered_set<int>> KinodynamicFMTX::findSamplesNearObstaclesDual(
    const ObstacleVector& obstacles, 
    double max_length
) {
    std::unordered_set<int> conflicting_samples_inflated;
    std::unordered_set<int> conflicting_samples;

    for (const auto& obstacle : obstacles) {
        double obstacle_radius, base_radius;
        if (obstacle.type == Obstacle::CIRCLE) {
            base_radius = obstacle.dimensions.radius;
            obstacle_radius = base_radius + obstacle.inflation;
        } else { // BOX
            // Calculate half diagonal of the box
            double half_diagonal = std::sqrt(
                std::pow(obstacle.dimensions.width/2, 2) + 
                std::pow(obstacle.dimensions.height/2, 2)
            );
            base_radius = half_diagonal;
            obstacle_radius = half_diagonal + obstacle.inflation;
        }

        double outer_radius = std::sqrt(
            std::pow(obstacle_radius, 2) + 
            std::pow(max_length / 2.0, 2)
        );
        
        double inner_radius = base_radius + obstacle.inflation;

        auto [sample_indices1, sample_indices2] = kdtree_->radiusSearchDual(
            obstacle.position, 
            outer_radius, 
            inner_radius
        );

        conflicting_samples_inflated.insert(sample_indices1.begin(), sample_indices1.end());
        conflicting_samples.insert(sample_indices2.begin(), sample_indices2.end());
    }

    return {conflicting_samples_inflated, conflicting_samples};
}




/*
    The procedure is like this depending on if you use ignore_sample to ignore all the samples around obstalce (based on the minimum amounf formula)
    if you ignore sample everything is easy and you don't need obstalce check in handle add and handle remove obstalce
    we can alse use added/removed wrt to the previous 
    imagine two circles that have an intersection the left one is previous position of the upstacle and the right is the new position
    also try to visulize them bigger because of the findSampleNewObstalce that has overestimate formula (which i emphasize agian is the minimum needed for the worst case scenraio)
    now the right crescent would be the "added" nodes and the left crescent would be the "removed" nodes and every thing in between is not necessary to be in update because they are 
    on samples_in_obstalces and we ignore them in the plan function

    the approach of ignoring samples is good but as the number of samples lowers the overestimate formula gets bigger because max_edge gets bigger so we end up ignoring lots of sample (so we are sacrificing optimality)
    even though its optimal for samples that we provide to the plan but since we ignore some samples so the plan() function doesnt see them
    but all in all we can say its much faster than to explicitly check and as the number of nodes increases that problem becomes negligible to non existent i would say although i need to provide some math maybe

    if i want to give some comparison between rrtx and fmtx in ignore sample case --> it would be a fight between incremental rewiring and batch repair! because if we use ignore_sample==true there would be absolutely no obstalce check
    and you can see as we lower the rate of the outer main while loop the batch update outperforms and as we increase the rate we see  they are both on par algorithms.
    to give more explanation lower rate (like 1hz) means the obstalce move alot and suddenly lots of nodes gets orphaned and fmtx shines

    but if we follow rrtx approach to explicitly check edges because why do we ignore useful samples! then we have to use complete current sample and complete previous samples 
    and we explicitly check obstalce for each node and its neighbor to find which of the overestimated node are indeed have an edge on obstalce and which don't

*/

void KinodynamicFMTX::updateObstacleSamples(const ObstacleVector& obstacles) {
    in_dynamic = true;

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

    auto current = findSamplesNearObstacles(obstacles, max_length);
    // auto [current, direct] = findSamplesNearObstaclesDual(obstacles, max_length);
    
    // if (current == samples_in_obstacles_) return; // Early exit if nothing has changed

    std::vector<int> added, removed;
    std::vector<int> cur, prev;

    if (ignore_sample) {
        for (int sample : current) {
            if (!samples_in_obstacles_.count(sample)) added.push_back(sample);
        }
        for (int sample : samples_in_obstacles_) {
            if (!current.count(sample)) removed.push_back(sample);
        }
    } else {
        
        for (int c : current) cur.push_back(c);
        for (int p : samples_in_obstacles_) prev.push_back(p);
    }

    if (ignore_sample) {
        if (!added.empty()) handleAddedObstacleSamples(added);
        if (!removed.empty()) handleRemovedObstacleSamples(removed);
        samples_in_obstacles_ = std::move(current);


    } else {
        if (!cur.empty()) handleAddedObstacleSamples(cur);
        if (!prev.empty()) handleRemovedObstacleSamples(prev);
        samples_in_obstacles_ = current;

    }

    // visualizeHeapAndUnvisited();


    // std::vector<Eigen::VectorXd> positions2;
    // for (const auto& y: v_open_set_) {
    //     Eigen::VectorXd vec(2);
    //     vec << tree_.at(y)->getStateValue();
    //     positions2.push_back(vec);
    // }
    // // std::string color_str = "0.0,0.0,1.0"; // Blue color
    // visualization_->visualizeNodes(positions2,"map");
    // v_open_set_.clear();
    



    /* THIS NOTE MAYBE OUTDATED!
        handling the problem of reconnecion to the inflation zone is not easy! you can't just keep track of the difference between added and current
        and you have to use current everytime in the handleAddedObstalceSamples because if you don't the added will only go forward (with a distance to the obstalce ofcourse)
        and the obstalce might move and end up on top of a edge but since the added is just a difference between previous and current iteration it doesnt cover the nodes on that edge
        so the timeline is this --> added --> then plan takes care of the nodes in the inflation zone --> obstalce move and end up on some long edge --> the added is far away and doesnt invalidate those long edge because it invalidated it in previous iteraion and moved on!
    */

}

// void FMTX::handleAddedObstacleSamples(const std::vector<int>& added) {
//     std::unordered_set<int> orphan_nodes;

//     /*
//         the 2 near function i put here because of rviz new goal feature i've added to the system and since the tree_ is cleared the neighbors need to be set again 
//     */
//     for (int idx : added) {
//         if (!ignore_sample && prune) {
//             auto node = tree_[idx].get();
//             near(idx);
//             for (auto& [neighbor, edge_info] : node->neighbors()) {
//                 if (edge_info.distance == INFINITY) continue;
//                 /*
//                     Note: i think we don't need to obstacle check for all the node->neighbor relatioship because i think we can utilize dualFindSample and instantly make nodes on samples_in_obstalce distances to INFINITY
//                           but for now since rrtx didn't do optimization in this part i keep the code this way
//                 */
//                 bool is_free_ = obs_checker_->isObstacleFree(node->getStateValue(), neighbor->getStateValue());
//                 if (is_free_) continue;

//                 edge_info.distance = INFINITY;
//                 near(neighbor->getIndex());
//                 neighbor->neighbors().at(node).distance = INFINITY;

//                 if (node->getParent() == neighbor){
//                     orphan_nodes.insert(node->getIndex()); // In the current tree_ we check if an edge connection the node and its parent is on obstalce or not, if it is, then we send it and its descendant to orphan list
//                     auto descendants = getDescendants(node->getIndex());
//                     orphan_nodes.insert(descendants.begin(), descendants.end());
//                 }
//                 if (neighbor->getParent() == node) { //We check bidirectionaly because we used "neighbor->neighbors().at(node).distance = INFINITY;" in above line
//                     orphan_nodes.insert(neighbor->getIndex());
//                     auto descendants = getDescendants(neighbor->getIndex());
//                     orphan_nodes.insert(descendants.begin(), descendants.end());
//                 }
//             }
//         }
//         else{
//             orphan_nodes.insert(idx);
//             auto descendants = getDescendants(idx);
//             orphan_nodes.insert(descendants.begin(), descendants.end());
//         }

//     }
//     /*
//         TO DO: Later i might create a hybrid approach to decide between prune true or false
//         Order of obstalce check for rrtx style is O(h ln(n)) --> h being the "added" variable and n being num of samples 
//         Order of obstalce check for fmt style is O(K) --> K being the number of orphan nodes
//     */
//     // std::cout << "Added samples: " << added.size()
//     //           << ", added.size() * ln(n): " << (added.size() * std::log(num_of_samples_))
//     //           << "\n";
//     // std::cout << "Orphan nodes count: " << orphan_nodes.size() << "\n";



//     /*
//         one might ask why do you put orphan nodes into v_unvisited_set when you have a mechanism in the main loop to find these automatically?! 
//         The reason is these help the finding of the v open nodes later in the update obstalce sample function
//         If we only rely on that mechansim we can't find connections to other branches because we are blind to see other branches! like on the other side of the tree
//         Imagine the one side of the plier and some nodes get better cost if they get connected to the other tip of the plier but since we didn't put the other side nodes into v open we never know!

//         (side note: Also imagine if the the two tips of the the plier is far apart so you can't rely on the neighborhood raidus of one side to get to the other!)

//         So that condtion in the main loop is just for one direction expansion and is good for the nodes that gor removed from the obstalce--> Although its a reasonable question here also to ask ourselves why its not the case
//         for the remove obstlace to now know their v open at first!
//         the difference between addObstalce and removeObstalce is adding and obstalce most certainly adds cost to orphan nodes
//         but removing an obstlace most certainly reduces cost of the neighbor nodes! and reducing happens in the current branch and direction of the expansion that happens in dijkstra like (like fmtx) algorithm 
//         so we don't need to worry about the other side of plier (per say!) because they are gonna connect to us! not us connecting to them (and by "us" i mean the current direction of the expansion)
//     */


//     // DOESNT MATTER IF THIS LOOP WOULD GO HIGHER OR LOWER THAN THE BELOW FOR LOOP BECAUSE THE VUNVISTED UPDATE LOOP IS GONNA HELP THE BELOW LOOP
//     for (auto node_index : orphan_nodes) {
//         // tree_.at(node_index)->in_unvisited_ = true;
//         auto node = tree_.at(node_index).get();
        
//         if (node->in_queue_) {
//             v_open_heap_.remove(node);
//             // node->in_queue_ = false;
//         }
//         if (!node->getIndex() == 0) // Root of the tree must keep its zero cost!
//             node->setCost(INFINITY); 
//         node->setParent(nullptr, INFINITY);
//         // node->getChildrenMutable().clear(); // We don't need to do this even though at this current iteration this node has children but they will be removed as we iterate by the setParent function
//         edge_length_[node_index] = -std::numeric_limits<double>::infinity();
//     }
  
//     /*
//         IMPORTNAT NOTE: My assumption is we do not need to update the queue here because we only need to ADD to queue. 
//         UPDATE IS WHEN A COST of node has changed and that happens only in the main plan function. here we only make the cost to inf and we removed. you may ask
//         how can you be sure we don't have any vopen heap nodes left? in partial update false the vopen heap gets fully cleaned because of the while loop but in partial update true
//         we early exit that loop so some vopen heap nodes are left! in this scenraio now imagine an obstalce is being added and at the worst case scenario it is being added to the region where there are already vopen heap nodes!
//         the result of adding obstalce means two things! ---> some vclosed (conceptually i mean because i don't use vclose in my algorithm!) nodes become vunvisted or some vopen nodes become vunvisted! and the previously vunvisited due to partial update
//         stays in vunvisted! mind that the cost of these per say messeup nodes will become infinity or stay infinity
//         for expansion we need CORRECT vopen heap nodes! (by correct i mean the correct cost that resembles the changes of the environment) and one rule you need to follow for safety is to not put any vunvisted node into vopen or if some vunvisted node is in vopen you need to remove it
//         so since the cost of nodes doesnt change to any numbers but inf! so we only need to remove them. the following addition to heap is also for covering the vunvisted node for the next batch of update in plan function
//         in the next for loop i do the heap removal

//         but in the plan function since i update the nodes cost because of the second condtion in the main if! i have to update the queue's priority --> and its only happening frequntly in obstalce removal(even though it might sometimes happens in the regular loop due to the main problem that fmt has in low sample number which is negligible when samples counters go to inf theoretically!)

//     */


//     /*
//         Time Complexity Comparison: Inserting k New Elements into a Heap of Size M
//         Method 1: Individual Insertions (add one by one)
//         Time Complexity:
//         O(klogM)
//         Each insertion takes O(logM) time (due to heapifyUp).

//         For k insertions: O(klogM).

//         Method 2: Bulk Insertion (bulkAdd)
//         Time Complexity: O(M+k)
//         Collecting elements: O(k).

//         Heap construction (Floyd’s method): 
//         O(M+k), where M is the existing heap size.

//         Which is Faster?
//         If k is small compared to M (k ≪ M)

//         O(klogM) vs.  O(M+k)≈O(M).
//         Since M might dominate klogM, Method 1 (individual insertions) could be faster.

//         If k is large (k ≈ M or k > M)
//         O(klogM) vs.  O(M+k)≈O(k).
//         Since k log ⁡ M ≫ k
//         klogM≫k, Method 2 (bulkAdd) wins.
//     */


//     // // Method 1

//     for (auto node_index : orphan_nodes) {
//         auto node = tree_.at(node_index).get();
//         near(node_index);
//         for (const auto& [neighbor, dist] : node->neighbors()){
//             int index = neighbor->getIndex();
//             if (neighbor->in_queue_ || neighbor->getCost()==INFINITY ) continue;
//             double h_value = use_heuristic ? heuristic(index) : 0.0;
//             v_open_heap_.add(neighbor , neighbor->getCost() + h_value);
//             // neighbor->in_queue_ = true;
//         }
//     }
// //////////////////////////////
//     // // // Method 2
//     // // 1) Gather & mark
//     // std::vector<std::pair<double, FMTNode*>> to_enqueue;
//     // // to_enqueue.reserve(orphan_nodes.size() * average_degree); // optional hint

//     // for (auto node_index : orphan_nodes) {
//     //     FMTNode* node = tree_.at(node_index).get();
//     //     near(node_index);  // ensure node->neighbors() is populated

//     //     for (const auto& [neighbor, dist] : node->neighbors()) {
//     //         // skip if already enqueued or not yet reachable
//     //         if (neighbor->in_queue_ || neighbor->getCost() == INFINITY) 
//     //             continue;

//     //         // mark so we don’t enqueue duplicates
//     //         neighbor->in_queue_ = true;

//     //         // compute priority (cost-to-come + optional heuristic)
//     //         double h_value = use_heuristic 
//     //                         ? heuristic(neighbor->getIndex()) 
//     //                         : 0.0;
//     //         double priority = neighbor->getCost() + h_value;

//     //         to_enqueue.emplace_back(priority, neighbor);
//     //     }
//     // }

//     // // 2) Bulk‐add into your custom priority queue in O(K)
//     // v_open_heap_.bulkAdd(to_enqueue);

//     // // Note: after bulkAdd, heap_index_ is set and in_queue_ remains true.
//     // // When you later pop/remove a node, your remove() method will reset in_queue_ = false.


// /////////////////////////////


  
// }

void KinodynamicFMTX::handleAddedObstacleSamples(const std::vector<int>& added_indices) { // Renamed 'added' to 'added_indices' for clarity
    std::unordered_set<int> nodes_to_make_orphan_and_process_neighbors; // Nodes whose state changed to trigger neighbor queueing

    for (int idx : added_indices) {
        FMTNode* node = tree_[idx].get(); // Assuming tree_ stores FMTNode pointers or shared_ptr
        bool node_itself_is_now_in_obstacle = false;

        if (ignore_sample) { // Corresponds to RRTX ignore_sample = true
            // If ignore_sample is true, 'added_indices' are nodes now considered part of an obstacle.
            // No explicit point collision check needed here for 'node' itself if this is the mode's definition.
            samples_in_obstacles_.insert(idx); // Mark this node as being "in an obstacle"
            node_itself_is_now_in_obstacle = true;
        } else if (prune) { // Corresponds to RRTX ignore_sample = false, with proactive checks
            // Explicitly check if the node's point itself is in an obstacle
            if (!obs_checker_->isObstacleFree(node->getStateValue())) { // Assuming this checks the point
                node_itself_is_now_in_obstacle = true;
            }
        }
        // If not ignore_sample and not prune, the original else block logic applies (make all 'added_indices' nodes orphans)

        if (!ignore_sample && prune && node_itself_is_now_in_obstacle) {
            // OPTIMIZATION: Node 'idx' itself is in an obstacle (and prune=true, ignore_sample=false).
            // Invalidate all its existing edges WITHOUT individual collision checks for each edge.
            near(idx); // Ensure neighbors are loaded if needed by your 'near' implementation
            for (auto& [neighbor, edge_info] : node->neighbors()) { // Iterate existing graph neighbors
                edge_info.distance = INFINITY;
                // Also update the symmetric part in the neighbor
                near(neighbor->getIndex()); // Ensure neighbor's neighbors are loaded
                if (neighbor->neighbors().count(node)) { // Check if symmetric entry exists
                    neighbor->neighbors().at(node).distance = INFINITY;
                }

                // If this edge was a parent link, the child becomes an orphan candidate
                if (node->getParent() == neighbor) { // 'node' loses 'neighbor' as parent
                    // 'node' is already determined to be an orphan due to being in an obstacle.
                    // This specific parent link is now broken.
                }
                if (neighbor->getParent() == node) { // 'neighbor' loses 'node' as parent
                    nodes_to_make_orphan_and_process_neighbors.insert(neighbor->getIndex());
                    // No need to get descendants here yet, will do it once for all primary orphans
                }
            }
            // 'node' itself is the primary orphan here
            nodes_to_make_orphan_and_process_neighbors.insert(idx);

        } else if (!ignore_sample && prune && !node_itself_is_now_in_obstacle) {
            // PRUNE MODE, BUT NODE ITSELF IS FINE: Check its edges individually.
            near(idx);
            for (auto& [neighbor, edge_info] : node->neighbors()) {
                if (edge_info.distance == INFINITY) continue;

                // Perform collision check for this specific edge (node -> neighbor)
                if (!obs_checker_->isObstacleFree(node->getStateValue(), neighbor->getStateValue())) {
                    // Edge is blocked
                    edge_info.distance = INFINITY;
                    near(neighbor->getIndex());
                    if (neighbor->neighbors().count(node)) {
                        neighbor->neighbors().at(node).distance = INFINITY;
                    }

                    // Handle parent relationships leading to orphans
                    if (node->getParent() == neighbor) {
                        nodes_to_make_orphan_and_process_neighbors.insert(node->getIndex());
                    }
                    if (neighbor->getParent() == node) {
                        nodes_to_make_orphan_and_process_neighbors.insert(neighbor->getIndex());
                    }
                }
            }
        } else { // Original logic: ignore_sample is true (node_itself_is_now_in_obstacle is true),
                 // OR prune is false. In these cases, all nodes in 'added_indices' are directly considered orphans.
            nodes_to_make_orphan_and_process_neighbors.insert(idx);
        }
    }

    // Now, gather all descendants for the initial set of orphaned nodes
    std::unordered_set<int> final_orphan_nodes;
    for (int orphan_idx : nodes_to_make_orphan_and_process_neighbors) {
        final_orphan_nodes.insert(orphan_idx);
        auto descendants = getDescendants(orphan_idx); // Assuming getDescendants works correctly
        final_orphan_nodes.insert(descendants.begin(), descendants.end());
    }

    // Process all final orphan nodes
    for (int node_index : final_orphan_nodes) {
        auto node = tree_.at(node_index).get();
        
        if (node->in_queue_) {
            v_open_heap_.remove(node); // Assuming remove also sets node->in_queue_ = false
        }
        if (node->getIndex() != 0) { // Assuming 0 is a special root/goal node index
            node->setCost(INFINITY); 
        }
        node->setParent(nullptr, INFINITY); // Sever parent link and set parent_edge_cost_
        // Children links will be broken when their parent (this node) is no longer their parent,
        // or when they themselves are processed as orphans and get a new parent (or nullptr).
        // You might need an explicit RRTxNode::removeChild if setParent doesn't notify the old parent.
        edge_length_[node_index] = -std::numeric_limits<double>::infinity(); // Or some other marker for invalid edge to parent
    }
  
    // Add valid neighbors of the final (now cost-infinity) orphans to the open heap
    // This is your "QueueNeighbors" equivalent from the RRTX discussion
    for (int node_index : final_orphan_nodes) {
        auto node = tree_.at(node_index).get();
        near(node_index); // Ensure neighbors are loaded
        for (const auto& [neighbor_ptr, edge_data] : node->neighbors()){ // Assuming edge_data not used here, just neighbor_ptr
            // 'neighbor_ptr' is of type FMTNode* (or RRTxNode*)
            if (neighbor_ptr->in_queue_ || neighbor_ptr->getCost() == INFINITY ) continue; 
            // If neighbor_ptr->getCost() == INFINITY, it means it's also an orphan, so skip.
            // We only want to queue neighbors that are still validly connected to the goal.

            double h_value = use_heuristic ? heuristic(neighbor_ptr->getIndex()) : 0.0;
            v_open_heap_.add(neighbor_ptr, neighbor_ptr->getCost() + h_value);
            // neighbor_ptr->in_queue_ = true; // Assuming v_open_heap_.add() handles this
        }
    }
}



void KinodynamicFMTX::handleRemovedObstacleSamples(const std::vector<int>& removed) {
    for (const auto& node_index : removed) {
        auto node = tree_[node_index].get();
        /*
            if you use the following line "node->in_unvisited_ = true;  " since some of the surrounding nodes already have a parent and the parent cost
            hasn't changed then we go to plan() function trying to repair some redundant node and the worst thing is all of them goes till the 
            core if condtion for the cost update the cost doesnt get updated because newCost is exaclty the same as the previous cost! so they will end up
            staying in the v unvisted and currupting the upcoming heap addition in the handleadd/remove function!
        
        */

        if (node->in_queue_ && node->getCost()==INFINITY) {
        /*
            it doesnt come here so maybe we don't need this --> im thinking there shouldn't be a node in remove that is in heap
            my reason is the removed nodes are the current nodes of the previous iteration and we didn't put any of the current
            nodes(messedup node actually which is a subset of current) in heap and havent updated those nodes yet!(because we haven't reached
            plan function yet)
            we didn't put them in heap because they had "inf" cost! but im gonna leave this here for later pondering!
        */
            std::cout<<"is it here?? \n";
            v_open_heap_.remove(node);
            // node->in_queue_ = false;
        }

        if (!ignore_sample && prune) {
            near(node_index);
            for (auto& [neighbor, edge_info] : node->neighbors()) {
                if (edge_info.distance != INFINITY) continue;
                if (obs_checker_->isObstacleFree(node->getStateValue(), neighbor->getStateValue())) {
                    edge_info.distance = edge_info.distance_original;
                    near(neighbor->getIndex());
                    neighbor->neighbors().at(node).distance = edge_info.distance_original;
                }
            }
        }
    }

    for (int node_index : removed) {
        auto node = tree_.at(node_index).get();
        near(node_index);
        for (const auto& [neighbor, dist] : node->neighbors()) {
            const int n_idx = neighbor->getIndex();
            if (neighbor->in_queue_ || neighbor->getCost()==INFINITY) continue;
            double h_value = use_heuristic ? heuristic(n_idx) : 0.0;
            v_open_heap_.add(neighbor , neighbor->getCost() + h_value);
            // neighbor->in_queue_ = true;
        }
    }

    // ////////////////////////////////////////////////////
    // // 1) Gather & mark neighbors of removed nodes
    // std::vector<std::pair<double, FMTNode*>> to_enqueue;
    // // to_enqueue.reserve(removed.size() * average_degree);  // optional hint

    // for (int node_index : removed) {
    //     FMTNode* node = tree_.at(node_index).get();
    //     near(node_index);  // ensure neighbors are cached

    //     for (const auto& [neighbor, dist] : node->neighbors()) {
    //         if (neighbor->in_queue_ || neighbor->getCost() == INFINITY) 
    //             continue;

    //         neighbor->in_queue_ = true;  // mark to avoid duplicates

    //         double h_value = use_heuristic 
    //                         ? heuristic(neighbor->getIndex()) 
    //                         : 0.0;
    //         double priority = neighbor->getCost() + h_value;

    //         to_enqueue.emplace_back(priority, neighbor);
    //     }
    // }

    // // 2) Bulk‑add into the open set
    // v_open_heap_.bulkAdd(to_enqueue);

    // // After bulkAdd, in_queue_ remains true; later pops will reset it.
    // ////////////////////////////////////////////////////////////


}





//////////////////////////////////////////////////////////////////////////////////////////////////////

double KinodynamicFMTX::heuristic(int current_index) {
    Eigen::VectorXd current_position = tree_.at(current_index)->getStateValue();
    Eigen::VectorXd goal_position = tree_.at(robot_state_index_)->getStateValue();
    return (goal_position-current_position).norm();
}



std::vector<size_t> KinodynamicFMTX::getPathIndex() const {
    int idx = robot_state_index_;
    std::vector<size_t> path_index;
    while (idx != -1) {
        path_index.push_back(idx);
        idx = tree_.at(idx)->getParent()->getIndex();
    }
    return path_index;
}

std::vector<Eigen::VectorXd> KinodynamicFMTX::getPathPositions() const {
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

// void KinodynamicFMTX::setRobotIndex(const Eigen::VectorXd& robot_position) {
//     robot_position_ = robot_position;

//     const double MAX_SEARCH_RADIUS = 5.0; // Meters
//     std::vector<size_t> nearest_indices = kdtree_->radiusSearch(robot_position, MAX_SEARCH_RADIUS);

//     size_t best_index = std::numeric_limits<size_t>::max(); 
//     double min_total_cost = std::numeric_limits<double>::max();
//     FMTNode* best_node = nullptr; 

//     for (size_t index : nearest_indices) {
//         auto node = tree_.at(index).get();
//         if (node->getCost() == std::numeric_limits<double>::infinity()) continue;

//         Eigen::VectorXd node_position = node->getStateValue();
//         double dx = node_position[0] - robot_position[0];
//         double dy = node_position[1] - robot_position[1];
//         double distance_to_node = std::hypot(dx, dy);

//         double total_cost = distance_to_node + node->getCost();

//         if (total_cost < min_total_cost) {
//             min_total_cost = total_cost;
//             best_index = index;
//             best_node = node;
//         }
//     }

//     if (best_index != std::numeric_limits<size_t>::max()) {
//         robot_state_index_ = best_node->getIndex();
//         robot_node_ = best_node;
//         return;
//     }

//     bool keep_prev_state_ = false;
//     if (robot_node_ && keep_prev_state_ == true) {
//         std::cout << "No valid node found in neighborhood. Keeping previous robot_node_.\n";
//         return;
//     }
//     if (robot_node_) {
//         std::cout << "No valid node found in neighborhood. Setting to nearest unvisited(cost=inf) node.\n";
//         // so it must be on the vunvisted zones --> lets get the nearest vunvisted and then rely on plan function to reach there if it can!
//         std::vector<size_t> nearest_indices = kdtree_->knnSearch(robot_position, 1);
//         int nearest = nearest_indices.empty() ? -1 : static_cast<int>(nearest_indices[0]);  
//         robot_node_ = tree_.at(nearest).get();
//         robot_state_index_ = robot_node_->getIndex();
//         return;
//     }

//     robot_state_index_ = -1;
//     robot_node_ = nullptr; 
//     std::cout << "No valid node found and no previous robot_node_. Setting robot_node_ to nullptr.\n";
// }

// This function finds the best node in the existing tree to serve as the
// parent for the robot's current real-time state. It kinodynamically "attaches"
// the robot to the search graph.
void KinodynamicFMTX::setRobotIndex(const Eigen::VectorXd& robot_state_4d) {
    // Store the robot's current full 4D state
    robot_position_ = robot_state_4d;

    // Define a spatial radius to search for candidate nodes in the tree.
    const double MAX_SEARCH_RADIUS = 5.0; // Meters

    // Use the k-d tree to find a list of potential nodes near the robot.
    // This is an efficient way to avoid checking every node in the tree.
    std::vector<size_t> nearest_indices = kdtree_->radiusSearch(robot_state_4d, MAX_SEARCH_RADIUS);

    size_t best_index = -1;
    double min_total_cost = std::numeric_limits<double>::infinity();
    FMTNode* best_node = nullptr;

    // Iterate through the nearby candidate nodes to find the best connection point.
    for (size_t index : nearest_indices) {
        auto node = tree_.at(index).get();
        
        // Skip nodes that are not part of a valid path to the goal (e.g., orphaned nodes).
        if (node->getCost() == std::numeric_limits<double>::infinity()) {
            continue;
        }

        // --- KINODYNAMIC COST CALCULATION ---
        // Calculate the true kinodynamic path and cost from the robot's current state
        // to the candidate node in the tree.
        Trajectory traj_to_node = statespace_->steer(robot_state_4d, node->getStateValue());
        
        // If a valid trajectory exists...
        if (traj_to_node.is_valid) {
            
            // The total cost to the ultimate goal is the cost (time) to get from the robot to this node,
            // PLUS the node's existing cost-to-goal (cost-to-root in our backward search).
            double total_cost = traj_to_node.cost + node->getCost();

            // If this path is better than the best one we've found so far...
            if (total_cost < min_total_cost) {
                
                // ...do a final check to ensure the connecting trajectory is collision-free.
                if (obs_checker_->isObstacleFree(traj_to_node.path_points)) {
                    // This is our new best connection.
                    min_total_cost = total_cost;
                    best_index = index;
                    best_node = node;
                }
            }
        }
    }

    // After checking all candidates, if we found a valid best node, update the planner's state.
    if (best_node != nullptr) {
        robot_state_index_ = best_node->getIndex();
        robot_node_ = best_node;
        
        // The robot's effective "cost-to-goal" is this newly calculated minimum total cost.
        robot_node_->setCost(min_total_cost); 
        return;
    }

    // --- FALLBACK LOGIC ---
    // This section is executed only if no valid, collision-free connection was found in the search radius.
    std::cout << "WARN: No valid, reachable node found in the neighborhood. Using fallback." << std::endl;
    
    // As a last resort, find the single nearest node in the entire tree, regardless of cost or collisions.
    // This prevents the planner from completely losing track of the robot.
    std::vector<size_t> knn_indices = kdtree_->knnSearch(robot_state_4d, 1);
    if (!knn_indices.empty()) {
        int nearest = knn_indices[0];
        robot_node_ = tree_.at(nearest).get();
        robot_state_index_ = robot_node_->getIndex();
    } else {
        // If the tree is empty or k-d tree fails, there's nothing we can do.
        robot_state_index_ = -1;
        robot_node_ = nullptr;
    }
}




void KinodynamicFMTX::setStart(const Eigen::VectorXd& start) {
    root_state_index_ = statespace_->getNumStates();
    auto node = std::make_shared<FMTNode>(statespace_->addState(start),tree_.size());
    node->setCost(0);
    // QueueElement2 new_element ={0,0};
    v_open_heap_.add(node.get(),0);
    // node->in_queue_ = true;

    tree_.push_back(node);
    std::cout << "KinodynamicFMTX: Start node created on Index: " << robot_state_index_ << "\n";
}
void KinodynamicFMTX::setGoal(const Eigen::VectorXd& goal) {
    robot_state_index_ = statespace_->getNumStates();
    auto node = std::make_shared<FMTNode>(statespace_->addState(goal),tree_.size());
    node->in_unvisited_ = true;

    robot_node_ = node.get(); // Management of the node variable above will be done by the unique_ptr i'll send to tree_ below so robot_node_ is just using it!
    tree_.push_back(node);
    std::cout << "KinodynamicFMTX: Goal node created on Index: " << root_state_index_ << "\n";
}





std::unordered_set<int> KinodynamicFMTX::getDescendants(int node_index) {
    std::unordered_set<int> descendants;
    std::queue<FMTNode*> queue;
    
    // Start with the initial node
    queue.push(tree_[node_index].get());
    
    while (!queue.empty()) {
        FMTNode* current = queue.front();
        queue.pop();
        
        descendants.insert(current->getIndex());
        
        for (FMTNode* child : current->getChildren()) {
            queue.push(child);
        }
    }
    
    return descendants;
}
/*
    The following is for finding if we have any cycles in our graph while we are doing dfs/bfs to find descendant
*/


// std::unordered_set<int> KinodynamicFMTX::getDescendants(int node_index) {
//     std::unordered_set<int> descendants;
//     std::queue<FMTNode*> queue;
//     std::unordered_set<FMTNode*> processing; // Track nodes being processed
    
//     // Debugging variables
//     int cycle_counter = 0;
//     constexpr int MAX_CYCLE_WARNINGS = 5;
//     auto start_time = std::chrono::steady_clock::now();

//     FMTNode* start_node = tree_[node_index].get();
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

//         FMTNode* current = queue.front();
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
//         for (FMTNode* child : children) {
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



std::vector<Eigen::VectorXd> KinodynamicFMTX::getSmoothedPathPositions(int num_intermediates, int smoothing_passes) const {
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


std::vector<Eigen::VectorXd> KinodynamicFMTX::interpolatePath(const std::vector<Eigen::VectorXd>& path, int num_intermediates) const {
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


std::vector<Eigen::VectorXd> KinodynamicFMTX::smoothPath(const std::vector<Eigen::VectorXd>& path, int window_size) const {
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

// void KinodynamicFMTX::visualizeTree() {
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

void KinodynamicFMTX::visualizeTree() {
    // This function will now draw the true, curved Dubins paths.
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> curved_edges;

    for (const auto& node_ptr : tree_) {
        // Get the current node and its parent
        FMTNode* child_node = node_ptr.get();
        FMTNode* parent_node = child_node->getParent();

        // If there is a parent, there is an edge to visualize
        if (parent_node) {
            
            // --- Access the Cached Trajectory ---
            // CORRECTED: The connection from child to parent is stored in the CHILD's neighbor map.
            auto it = child_node->neighbors().find(parent_node);

            // Check if the neighbor relationship and the trajectory exist
            if (it != child_node->neighbors().end() && it->second.is_trajectory_computed) {
                const auto& trajectory = it->second.cached_trajectory;
                
                // If the trajectory is valid and has points, draw its segments
                if (trajectory.is_valid && trajectory.path_points.size() > 1) {
                    
                    // Add a line segment for each pair of consecutive waypoints in the path
                    for (size_t i = 0; i < trajectory.path_points.size() - 1; ++i) {
                        curved_edges.emplace_back(trajectory.path_points[i], trajectory.path_points[i+1]);
                    }
                }
            } else {
                // Fallback for safety: if trajectory not found, draw a straight line
                // This can help debug issues with caching.
                curved_edges.emplace_back(parent_node->getStateValue(), child_node->getStateValue());
            }
        }
    }

    // Visualize the rich set of curved edges
    visualization_->visualizeEdges(curved_edges, "map");
}

void KinodynamicFMTX::visualizePath(std::vector<size_t> path_indices) {
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


void KinodynamicFMTX::visualizeSmoothedPath(const std::vector<Eigen::VectorXd>& shortest_path_) {
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


void KinodynamicFMTX::visualizeHeapAndUnvisited() {
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

    // visualization_->visualizeNodes(vopen_positions);
    visualization_->visualizeNodes(vopen_positions, "map", std::vector<float>{0.0f,1.0f,0.0f}, "vopen");

}
//////////////////////////////////////////////
void KinodynamicFMTX::dumpTreeToCSV(const std::string& filename) const {
    std::ofstream fout(filename);
    if (!fout.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing\n";
        return;
    }
    // 1) figure out dimension of states
    if (tree_.empty()) {
        std::cerr << "Tree is empty. Nothing to dump.\n";
        return;
    }
    size_t dim = tree_[0]->getStateValue().size();
    // 2) write CSV header
    fout << "node_id";
    for (size_t d = 0; d < dim; ++d) {
        fout << ",x" << d;
    }
    fout << ",parent_id\n";

    // 3) for each node in tree_, write: node_id, coords..., parent_id
    for (const auto& node_ptr : tree_) {
        int nid = node_ptr->getIndex(); 
        auto coords = node_ptr->getStateValue();
        FMTNode* parent = node_ptr->getParent();
        int pid = (parent ? parent->getIndex() : -1);

        fout << nid;
        for (size_t d = 0; d < dim; ++d) {
            fout << "," << std::setprecision(10) << coords[d];
        }
        fout << "," << pid << "\n";
    }
    fout.close();
    std::cout << "Tree dumped to " << filename << "\n";
}



