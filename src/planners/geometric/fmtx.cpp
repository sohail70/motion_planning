// Copyright 2025 Soheil E.nia
#include "motion_planning/planners/geometric/fmtx.hpp"

FMTX::FMTX(std::shared_ptr<StateSpace> statespace ,std::shared_ptr<ProblemDefinition> problem_def, std::shared_ptr<ObstacleChecker> obs_checker) :  statespace_(statespace), problem_(problem_def), obs_checker_(obs_checker) {
    std::cout<< "FMTX Constructor \n";

}

void FMTX::clearPlannerState() {

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


void FMTX::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
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
        // Put all the points at once because fmtx doesnt need incremental addition
        kdtree_->addPoints(statespace_->getSamplesCopy());
        // Build the tree all at once after we fill the data_ in the KDTree
        kdtree_->buildTree();
    }

    ///////////////////Neighborhood Radius////////////////////////////////
    int d = statespace_->getDimension();
    mu = std::pow(problem_->getUpperBound() - problem_->getLowerBound() , 2);
    zetaD = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
    gamma = 2 * std::pow(1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d); //Real FMT star gamma which is smaller than rrt star which makes the neighborhood size less than rrt star hence so much faster performance
    // gamma = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d);

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

void FMTX::plan() {

    // visualizeHeapAndUnvisited();

    std::unordered_map<std::pair<int, int>, bool, pair_hash> obstacle_check_cache;
    int uncached = 0;
    int cached = 0;
    // int checks = 0;
    // std::unordered_map<FMTNode*, bool> costUpdated;
    /*
        We go until the heap is empty and the top expansion node has the g_value thats less than the robot's current node g_value because why would we waste time exploring past that!
        also we consider if the g_value of the robot is infinity we need to keep poping heap nodes in the hope it reaches the robot
        or if the robot node is in v_unvistited node which has the same meaning as the cost inf so it maybe redundant
        also if the robot is in heap (queue) we contine poping ---> can't remeber the reason i did this!

        partial_update flag is a convenient bool to just deactivate those conditions!

        I remember I used to put those conditions in an if with a break and i did clean the heap in that if condition but here i preserve the heap because 
        it might be useful later in case we need an expansion
    
    */

    // std::this_thread::sleep_for(std::chrono::milliseconds(100));

    while (!v_open_heap_.empty() &&
           (partial_update ? (v_open_heap_.top().first < robot_node_->getCost() ||
            robot_node_->getCost() == INFINITY || //I think the above condtion also covers this because if the cost is inf the heap pops till the end because the above condition would be always true
            robot_node_->in_queue_==true)  : true  )){





        auto top_element = v_open_heap_.top();
        double cost = top_element.first;  // Changed .min_key to .first
        FMTNode* z = top_element.second;  // Changed .index to .second
        int zIndex = z->getIndex();
        // std::cout<<"current z "<< zIndex<<"\n";
        // std::cout<<"current z pos "<< z->getStateValue()<<"\n";

        // std::vector<Eigen::VectorXd> nodes;
        // nodes.push_back(z->getStateValue());
        // visualization_->visualizeNodes(nodes, "map", 
        //         std::vector<float>{1.0f, 1.0f, 0.0f},
        //         "z");
        // if(in_dynamic) std::this_thread::sleep_for(std::chrono::milliseconds(100));
        /*
            We find z node's (which at first is the root node and later it could be anything) neighbor and use kd tree to find and cache them 
            and after that we are safe to use z->neighbors() 
        */
        near(zIndex);
        for (const auto& [x, cost_to_neighbor] : z->neighbors()) {
            int xIndex = x->getIndex(); // As I refactor the code I don't need to use xIndex anymore but I still need som refactoring.

            /*
                About the following if:
                This trick actually enables us to only track changes in the update obstalce otherwise we have to use the 
                current nodes instead of added in the handle add obstalce or the other way is to check all the edges around 
                added obstalce for obstalce

                I also added this feature to rrtx so we have a fair comparison between these two if we use ignore_sample==true 

                to give some information when we use ignore_sample we are actually ignoring to update the nodes around obstalce. 
                the nodes that we ignore is the minimum amount of nodes that is essential based on the worst case formula:

                sqrt(radius of obstalce ^2 + (max_length/2)^2 ) 

                why max_length/2 ?? because the worst case happens when an edge is tangent to the circle obstalce and half of edge is on the upper semicricle and half is down
                imagine the left is max_edge and the right is a circle

                    |◯

                Now using Pythagorean Theorem you can find the amount of length you need to provide for the kd tree radius search to include at least one of those nodes
                Other if that line is slightly up the lower node will be catched by the kd tree radius search


            */
            if (ignore_sample==true && samples_in_obstacles_.find(xIndex) != samples_in_obstacles_.end()){
                continue;
            }

            /*
                the first condition below is from fmt 
                and the second is what i added to enable rewiring
                the second if is mainly gonna be used when we remove obstalce
                other wise when we addObstacle we specifically find the nodes that we should put in vUnvisited because before
                adding obstacle we have a tree and we can find the descendant using depth first search to orphan all the affected nodes
                but when we remove obstacle we can only put the direct nodes that are now off the obstacle into vunvisited and put their surrounding into vopen
                and rely on the second condition below to correct the nodes as we go downstream
                There is an important concept that you need to understand that there is a fundamental difference between adding obstacle and removing obstacle
                I explained this in the handleAddobstalce function but one thing you need to know is when adding obstacle the leaf nodes might get connected to other side branches 
                (the plier example) but in remove obstalce its not happening because remove obstalce surely reduces cost for some nodes so the update down stream surely connect to the branch
                thats being updated through the immediate nodes that are now free of obstacle. but in add obstalce the cost for some nodes is gonna go up so they might decide to attach themselves 
                to other branches to have a lower cost

                10K nodes static env using dynamic_world.sdf without hitting play and commenting the update function and the static plan before the loop: and making the simulation is pause false
                    checks: 8989
                    Time taken for the update : 786 milliseconds
               
                and with using the second condtion:
                    checks: 8989
                    Time taken for the update : 798 milliseconds

                so it took the same amount! so no burden in the static case. I also tested 1000 and 500 nodes and they both took the same amount
                I thought maybe it would add a little burden of rewiring the already updated nodes in the static case but it didn't maybe because 
                Well i guess the reason is that condtion also relies on z node which we are losing as we go. I thought maybe this condtion would
                help the lack of ability of fmt to completely find optimal solution in low sample case but it doesnt which is good at least i don't
                have to worry about theretical stuff and that condtion serves its purpose in the remove obstacle case.

                Also on another note mind that we are not assigning the nodes that satisfy the second condtion to anything or obsolete them beforhand so they are either getting updated or keep their last parent/cost/children
                this might create confusion but if you think about it its okay! to give an exmaple imgaine second if condtion is satisfied for some node that has a parent and children
                and then it gets to be updated in the cost update procedure after obstalce check. then one should setParent either to a new parent or its old parent (the old parent also happens but obviously the cost of that parent has changed beforehand)
                setParent takes care of connecting the x node to the newParent and remove that x as the child of old parent but how about x's children?
                x node obviously might have had some children that we are not removing! but we don't need to because as the update goes the setParent takes care of those mentioned children one by one
                because they are downstream nodes and they get updated any way on the next z node pop and update procedure, UNTIL you reach the robot and then you might see some inconsistency (visually) that happens after that
                which is also okay because if you move the robot using MOUSE in gazebo you can see they also CAN get updated but we don't need to update that because we are using early exit!


                I TESTED AGAIN AND ONLY THE OBSTALCE CHECKS AND ONLY USING UNVISITED CONDTION ONCE AND COST FUNCTION ONCE SO THE ABOVE COMMENTS ARE OUTDATED AND THE TEST IT SELF MIGHT EVEN BE WRONG BUT THE COMMENTS ARE OKAY TO READ
                10k nodes:

                checks: 152973 obstacle checks --> using only unvisited condition
                    Time taken for the update : 786 milliseconds
               
                and with using the second condtion:
                    checks: 152973 obstacle checks --> using only the cost condition
                    Time taken for the update : 798 milliseconds

            */ 
            // if(x->getIndex()==15 || x->getIndex()==908 || x->getIndex()==753)
            //     std::cout<<x->getIndex() <<" "<<x->getStateValue() <<"\n";
            // if (x->in_unvisited_==true  ){
            if (x->getCost() > (z->getCost() + cost_to_neighbor.distance ) ){ // THE REASON I DITCHED THE x->in_unvisited_==true CONDTION IS USING COST IS IN PAR WITH MAKING THE EDGE DISTANCE TO INF AND I DON'T HAVE TO TAKE CARE OF UNVISTED ON TOP OF IT WHICH IS TIME CONSUMING AND ALSO NOT CLEAN AND REDUNDANT IN MY CASE!
                // if(x->getIndex()==15 || x->getIndex()==908 || x->getIndex()==753)
                //     std::cout<<"here \n";
                // std::vector<Eigen::VectorXd> nodes;
                // if (costUpdated[x]) {
                //     std::cout<<"Node " << xIndex 
                //         << " is about to be updated a second time! "
                //         "previous cost = " << x->getCost() << "\n";

                //     // nodes.push_back(x->getStateValue());
                //     // nodes.push_back(x->getParent()->getStateValue());

                // }


                // std::cout<<x->getCost()<<"\n";
                // checks++;
                near(xIndex);
                double min_cost = std::numeric_limits<double>::infinity(); //the above condtion doesnt let it come after x got its optimal cost after being inf/sub-optimal so no need for the next line but lets leave it for safety until i can have solid math proof
                // double min_cost = x->getCost(); // Because x might have a cost it self and even if it doesnt its INFINITY by default in the FMTNode CLass
                FMTNode* best_neighbor_node = nullptr;
                double best_edge_length = 0.0;
                // EdgeInfo best_edge_info;
                /* 
                    out of those nodes that are near to x, which are also in vopen heap, which has the lowest g_value (cost to come in the paper)
                    this is also the place where you can ignore the blocked best neighbors if you wanna use heuristic otherwise its not necessary
                
                */
                
                for (const auto& [neighbor, dist] : x->neighbors()) {
                    // if(use_heuristic==true && x->blocked_best_neighbors.count(neighbor->getIndex()) > 0)
                        // continue;

                    if (neighbor->in_queue_) {
                        const double total_cost = neighbor->getCost() + dist.distance;
                        if (total_cost < min_cost) {
                            min_cost = total_cost;
                            best_neighbor_node = neighbor;
                            best_edge_length = dist.distance;
                            // best_edge_info = dist;
                        }
                    }
                }
                // if(best_neighbor_node!=z)
                //     std::cout<<"best is not z \n";
                // else
                //     std::cout<<"best is z \n";

                // if (!best_neighbor_node || best_edge_info.distance ==INFINITY) {
                //     continue;
                // }
                if (!best_neighbor_node ) {
                    continue;
                }
                // if (costUpdated[x]) {
                //     std::cout<<"Node " << xIndex 
                //         << "  updated a second time! "
                //         "new cost = " << min_cost << "\n";
                    
                //     // nodes.push_back(best_neighbor_node->getStateValue());
                //     // std::cout<<"x: "<< x->getIndex()<<"\n";
                //     // std::cout<<"old parent(x): "<< x->getParent()->getIndex()<<"\n";
                //     // std::cout<<"newParent: "<< best_neighbor_node->getIndex()<<"\n";
                //     // std::cout<<"x: "<< x->getStateValue()<<"\n";
                //     // std::cout<<"old parent(x): "<< x->getParent()->getStateValue()<<"\n";
                //     // std::cout<<"newParent: "<< best_neighbor_node->getStateValue()<<"\n";
                //     // // visualization_->visualizeNodes(nodes);
                //     // visualization_->visualizeNodes(nodes, "map", 
                //     //         std::vector<float>{1.0f, 0.0f, 1.0f},
                //     //         "mess");
                // }

                int best_neighbor_index = best_neighbor_node->getIndex();

                // checks++;

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
                //         obstacle_free = obs_checker_->isObstacleFree(x->getStateValue() , best_neighbor_node->getStateValue());
                //         obstacle_check_cache[edge_key] = obstacle_free;
                //         uncached++;
                //     }
                // }
                // else { //SOMETIMES BEST_NEIGHBOR_INDEX is -1 which means all the Ynear nodes has inf cost --> inf cost means its either samples_in_obstalces or vUnvisted or it was made to inf in the handleAddObstalce! --> THESE nodes shouldn't be in vOpen --> sometimes a node lingers in vOpen because of early exit so you have to erase it in handleAddObstalce or you have to check some ifs in Ynear node push_back!
                //     obstacle_free = obs_checker_->isObstacleFree(x->getStateValue() , best_neighbor_node->getStateValue());
                // }

         
                bool perform_check = false;
                if (in_dynamic == false) { // Initial static planning phase
                    perform_check = true;
                } else { // Dynamic update phase
                    /*
                        Prune false means we are not using the rrtx like explicit obstacle check in the handle add and remove obstacle  
                        and delay that to plan() function which i don't think its good because obstalce check is only necessary in obstalce
                        surrounding and we can rely on distance inf and cost propagation or else we have to obstalce check the whole orphaned region which is aligned with 
                        fmt philosiphy and frankly has a better time compleixity!
                        for example for one obstalce:
                            RRTX --> O(nodes with edge on obstlace * log(n)) --> log(n) is for average number of neighbors in sampling based planning! ---> worst would be O(n log(n))
                            FMTX ---> O(orphan nodes) ---> worse would be O(n) obstacle checks
                    */

                    if (prune == false) { // If not pruning, we must check dynamically
                        perform_check = true;
                    } else {
                        // If pruning is enabled during dynamic updates,
                        // assume edges made invalid by obstacles were already handled (e.g., cost set to INF).
                        // So, any edge considered here that doesn't have an INF cost is presumed free.
                        obstacle_free = true; // Relies on prior pruning
                    }
                }

                if (perform_check) {
                    if (obs_cache == true) {
                        // Ensure indices are valid before creating a key
                        int current_x_index = x->getIndex(); // Assuming x is always valid
                        int best_neighbor_node_index = best_neighbor_node->getIndex();

                        // Standardize key creation
                        auto edge_key = (best_neighbor_node_index < current_x_index) ? 
                                        std::make_pair(best_neighbor_node_index, current_x_index) : 
                                        std::make_pair(current_x_index, best_neighbor_node_index);

                        auto cache_it = obstacle_check_cache.find(edge_key);
                        if (cache_it != obstacle_check_cache.end()) {
                            obstacle_free = cache_it->second;
                            cached++;
                        } else {
                            obstacle_free = obs_checker_->isObstacleFree(x->getStateValue(), best_neighbor_node->getStateValue());
                            obstacle_check_cache[edge_key] = obstacle_free;
                            uncached++;
                        }
                    } else { // No obstacle caching
                        obstacle_free = obs_checker_->isObstacleFree(x->getStateValue(), best_neighbor_node->getStateValue());
                    }
                }


                /*
                    Mind that because of the second condtion that I've added we might re-update (re-wire) some already updated nodes and
                    it happens solely in the remove obstacle case. I thought maybe it would also happen in regular static case but the test diidn't show anything

                    There is also another case where the same parent with the same cost happens and the reason is putting removed obstalce nodes in v unvisted!
                    you either have to use newCost-0.00001 < x->getCost() or not put nodes in v unvisted in remove obstacle

                    there are so many combinations like i can decide to not set the in_unvisted to true even! like not setting anything but i have
                    to be carefull to not put the orphan nodes in the heap somehow by using the orphan index it self instead of v unvisted ! but does that cause other issues? this is delicate!
                    well off the top of my head not using in_unvisted as a condtion in the for loop of heap addition is not good because that loop need to have access to all the unvisted nodes globally!
                    to not put any of them in the heap!

                    better option maybe is to put that in the unvisted more carefully instead of putting all the removed in the unvisited!

                */

                if (obstacle_free) {
                    double newCost = min_cost;
                    // if (newCost < x->getCost()) { // Seems like we don't need this if condtion!(if you think about it its redundant) because newCost is indeed less than x->getCost()
                        // if (use_heuristic==true) x->blocked_best_neighbors.clear(); // Well if x is connected then i don't care about neighbors that can't be connected so what a better place to clearing them than here. this is for when you use heuristic
                        x->setCost(newCost);
                        // costUpdated[x] = true;   // mark “done once”
                        double h_value = use_heuristic ? heuristic(xIndex) : 0.0;
                        double priorityCost = newCost + h_value;
                        // QueueElement2 new_element = {priorityCost, xIndex};
                        if (x->in_queue_ == true){
                            v_open_heap_.update(x,priorityCost);
                        } else{
                            v_open_heap_.add(x,priorityCost);
                            // x->in_queue_ = true;
                        }

                        /*
                            //IMPORTANT CONCEPT --> sometimes x's parent is the same as best_neighbor_node so why did we end up here? because the parent's cost has changed because of my new condtion that i put there!!!! (it obviouly happens in remove obstalce case)
                            setParent: 
                                so its either the same parent with new cost! --> due to the secoond if condtion --> so no need to change the children
                                or a new parent with new cost! --> we need to set the parent and children
                                or the same parent with the same cost!  ---> we can early return in the setParent
                            
                                The if condition im talking about is "|| x->getCost() > (z->getCost() + cost_to_neighbor.distance" 
                        */

                        x->setParent(best_neighbor_node,best_edge_length); 
                        // x->getChildrenMutable().clear(); // We don't need to do this even though at this current iteration this node has children but they will be removed as we iterate by the setParent function
                        edge_length_[xIndex] = best_edge_length;
                        x->in_unvisited_=false; // I use this only when i want to test if my x_cost condtion yeilds the same number of checks as the x_unvisted check --> you can uncomment the line above my cost condtion and also the cout at the end of the function 

                    // }
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

        v_open_heap_.pop();
        // z->in_queue_=false;
        z->in_unvisited_ =false;
        // visualizeTree();

    }

    /*
        Don't be tempted to clear the vopen heap after the loop because of the early exit! those frontiers in vopen are important in case the robot needs a detour from that speicifc side where those vopens are!
    */

    // std::cout<<" checks: "<< checks <<"\n";
    // std::cout<<" uncached: "<< uncached<<"\n";
    // std::cout<<" cached: "<< cached <<"\n";
}




/*
    Near function at first uses kd tree and after that it caches node so it doesnt need to spend time on cache
    WHEN KD TREE PROVIDES DISTANCE WHY DO YOU CALC DISTS AGAIN IN BELOW! --> ALSO DO THIS FOR RRTX  ---->
    Im not gonna use distance of kd tree for now because i don't know about other kd tree implementations
*/
void FMTX::near(int node_index) {
    auto node = tree_[node_index].get();
    if (!node->neighbors().empty()) return;

    auto indices = kdtree_->radiusSearch(node->getStateValue(), neighborhood_radius_); 
    for(int idx : indices) {
        if(idx == node->getIndex()) continue;
        FMTNode* neighbor = tree_[idx].get();
        auto dist = (node->getStateValue() - neighbor->getStateValue()).norm();
        node->neighbors()[neighbor] = EdgeInfo{dist,dist}; // The first is for distance and the second is for distance_original so that we have a cache incase we want to reset the obsoleted edges to correct distance (used in prune==true case)
    }
}


/*
    When an obstalce appears on some node we rely on the position of the obstalce and its radius to (D-ball containing that obstalce)
    to find the nodes in the tree that are inside of that obstalce and we use the following formula to handle worst case scenraio that i explained in the plan function i guess
    std::sqrt(std::pow(obstacle.radius + obstacle.inflation , 2) + std::pow(max_length / 2.0, 2))
    this is minimum amount needed to cover all the potential colliding edges and it lower than rrtx paper and better
*/

std::unordered_set<int> FMTX::findSamplesNearObstacles(
    const std::vector<Obstacle>& obstacles, 
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
            obstacle_radius = obstacle.dimensions.circle.radius + obstacle.inflation;
        } else { // BOX
            // Calculate half diagonal of the box
            double half_diagonal = std::sqrt(
                std::pow(obstacle.dimensions.box.width/2, 2) + 
                std::pow(obstacle.dimensions.box.height/2, 2)
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
std::pair<std::unordered_set<int>, std::unordered_set<int>> FMTX::findSamplesNearObstaclesDual(
    const std::vector<Obstacle>& obstacles, 
    double max_length
) {
    std::unordered_set<int> conflicting_samples_inflated;
    std::unordered_set<int> conflicting_samples;

    for (const auto& obstacle : obstacles) {
        double obstacle_radius, base_radius;
        if (obstacle.type == Obstacle::CIRCLE) {
            base_radius = obstacle.dimensions.circle.radius;
            obstacle_radius = base_radius + obstacle.inflation;
        } else { // BOX
            // Calculate half diagonal of the box
            double half_diagonal = std::sqrt(
                std::pow(obstacle.dimensions.box.width/2, 2) + 
                std::pow(obstacle.dimensions.box.height/2, 2)
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

void FMTX::updateObstacleSamples(const std::vector<Obstacle>& obstacles) {
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

void FMTX::handleAddedObstacleSamples(const std::vector<int>& added_indices) { // Renamed 'added' to 'added_indices' for clarity
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



void FMTX::handleRemovedObstacleSamples(const std::vector<int>& removed) {
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

double FMTX::heuristic(int current_index) {
    Eigen::VectorXd current_position = tree_.at(current_index)->getStateValue();
    Eigen::VectorXd goal_position = tree_.at(robot_state_index_)->getStateValue();
    return (goal_position-current_position).norm();
}



std::vector<size_t> FMTX::getPathIndex() const {
    int idx = robot_state_index_;
    std::vector<size_t> path_index;
    while (idx != -1) {
        path_index.push_back(idx);
        idx = tree_.at(idx)->getParent()->getIndex();
    }
    return path_index;
}

std::vector<Eigen::VectorXd> FMTX::getPathPositions() const {
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

void FMTX::setRobotIndex(const Eigen::VectorXd& robot_position) {
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

void FMTX::setStart(const Eigen::VectorXd& start) {
    root_state_index_ = statespace_->getNumStates();
    auto node = std::make_shared<FMTNode>(statespace_->addState(start),tree_.size());
    node->setCost(0);
    // QueueElement2 new_element ={0,0};
    v_open_heap_.add(node.get(),0);
    // node->in_queue_ = true;

    tree_.push_back(node);
    std::cout << "FMTX: Start node created on Index: " << robot_state_index_ << "\n";
}
void FMTX::setGoal(const Eigen::VectorXd& goal) {
    robot_state_index_ = statespace_->getNumStates();
    auto node = std::make_shared<FMTNode>(statespace_->addState(goal),tree_.size());
    node->in_unvisited_ = true;

    robot_node_ = node.get(); // Management of the node variable above will be done by the unique_ptr i'll send to tree_ below so robot_node_ is just using it!
    tree_.push_back(node);
    std::cout << "FMTX: Goal node created on Index: " << root_state_index_ << "\n";
}





// std::unordered_set<int> FMTX::getDescendants(int node_index) {
//     std::unordered_set<int> descendants;
//     std::queue<FMTNode*> queue;
    
//     // Start with the initial node
//     queue.push(tree_[node_index].get());
    
//     while (!queue.empty()) {
//         FMTNode* current = queue.front();
//         queue.pop();
        
//         descendants.insert(current->getIndex());
        
//         for (FMTNode* child : current->getChildren()) {
//             queue.push(child);
//         }
//     }
    
//     return descendants;
// }
/*
    The following is for finding if we have any cycles in our graph while we are doing dfs/bfs to find descendant
*/


std::unordered_set<int> FMTX::getDescendants(int node_index) {
    std::unordered_set<int> descendants;
    std::queue<FMTNode*> queue;
    std::unordered_set<FMTNode*> processing; // Track nodes being processed
    
    // Debugging variables
    int cycle_counter = 0;
    constexpr int MAX_CYCLE_WARNINGS = 5;
    auto start_time = std::chrono::steady_clock::now();

    FMTNode* start_node = tree_[node_index].get();
    queue.push(start_node);
    processing.insert(start_node);

    while (!queue.empty()) {
        // Check for infinite loops
        if (++cycle_counter > tree_.size() * 2) {
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start_time
            );
            std::cerr << "CRITICAL WARNING: Potential infinite loop detected!\n"
                      << "Current node: " << queue.front()->getIndex() << "\n"
                      << "Elapsed time: " << duration.count() << "s\n"
                      << "Descendants found: " << descendants.size() << "\n";
            break;
        }

        FMTNode* current = queue.front();
        queue.pop();
        processing.erase(current);

        // Check if we've already processed this node
        if (!descendants.insert(current->getIndex()).second) {
            if (cycle_counter < MAX_CYCLE_WARNINGS) {
                std::cerr << "Cycle detected! Already processed node: " 
                          << current->getIndex() << "\n";
            }
            continue;
        }

        // Process children with cycle checks
        const auto& children = current->getChildren();
        for (FMTNode* child : children) {
            if (processing.count(child)) {
                std::cerr << "Parent-child cycle detected!\n"
                          << "Parent: " << current->getIndex() << "\n"
                          << "Child: " << child->getIndex() << "\n";
                continue;
            }

            if (descendants.count(child->getIndex())) {
                std::cerr << "Cross-branch cycle detected!\n"
                          << "Current branch: " << current->getIndex() << "\n"
                          << "Existing descendant: " << child->getIndex() << "\n";
                continue;
            }

            queue.push(child);
            processing.insert(child);
        }

        cycle_counter = 0; // Reset counter if we made progress
    }

    // Final check for partial cycles
    if (!queue.empty()) {
        std::cerr << "WARNING: Terminated early with " << queue.size()
                  << " nodes remaining in queue\n";
    }

    return descendants;
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

void FMTX::visualizePath(std::vector<size_t> path_indices) {
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


void FMTX::visualizeHeapAndUnvisited() {
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



