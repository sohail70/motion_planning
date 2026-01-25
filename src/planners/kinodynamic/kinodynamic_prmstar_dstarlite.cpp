#include "motion_planning/planners/kinodynamic/kinodynamic_prmstar_dstarlite.hpp"


// Constructor
KinodynamicPRMStarDStarLite::KinodynamicPRMStarDStarLite(
    std::shared_ptr<StateSpace> statespace, 
    std::shared_ptr<ProblemDefinition> pdef,
    std::shared_ptr<ObstacleChecker> obs_checker)
    : statespace_(statespace), problem_def_(pdef), obs_checker_(obs_checker), km_(0.0) {
    std::cout << "KinodynamicPRMStarDStarLite Constructor \n";
}

void KinodynamicPRMStarDStarLite::setStart(const Eigen::VectorXd& start) {
    problem_def_->setStart(start);
}

void KinodynamicPRMStarDStarLite::setGoal(const Eigen::VectorXd& goal) {
    problem_def_->setGoal(goal);
}
void KinodynamicPRMStarDStarLite::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    auto start_time = std::chrono::high_resolution_clock::now();
    visualization_ = visualization;
    
    // 1. Load Parameters
    num_samples_ = params.getParam<int>("num_of_samples");
    use_kdtree_ = params.getParam<bool>("use_kdtree");
    kd_dim_ = params.getParam<int>("kd_dim", 2);
    std::string kdtree_type = params.getParam<std::string>("kdtree_type");
    use_knn_ = params.getParam<bool>("use_knn", false);
    factor_ = params.getParam<double>("factor", 1.0);
    neighbor_precache_ = params.getParam<bool>("precache_neighbors", false);
    
    lower_bounds_ = problem_def_->getLowerBound();
    upper_bounds_ = problem_def_->getUpperBound();

    // 2. Initialize KD-Tree
    if (use_kdtree_ && kdtree_type == "NanoFlann") {
        Eigen::VectorXd weights(kd_dim_);
        switch (kd_dim_) {
            case 2: weights << 1.0, 1.0; break;
            case 3: weights << 1.0, 1.0, 1.0; break;
            case 4: weights << 1.0, 1.0, 1.0, 1.0; break;
            case 5: weights << 1.0, 1.0, 1.0, 1.0, 1.0; break;
            default: throw std::runtime_error("Unsupported k-d tree dimension");
        }
        kdtree_ = std::make_shared<WeightedNanoFlann>(kd_dim_, weights);
    } else if (use_kdtree_ && kdtree_type == "LieKDTree") {
        kdtree_ = std::make_unique<LieSplittingKDTree>(statespace_->getDimension(), statespace_);
    } else {
        throw std::runtime_error("PRM* D* Lite requires a KD-Tree.");
    }

    // 3. Build Graph (Sample + Identify Start/Goal)
    // We do this BEFORE building the KD-Tree so the KD-Tree includes Start/Goal.
    const Eigen::VectorXd& start_state_val = problem_def_->getGoal(); // I accidentally used the getGoal and setGoal in reverse in my main loop so dont get confused this is correct!
    const Eigen::VectorXd& goal_state_val = problem_def_->getStart();


    // Add Goal Node --> root
    auto goal_state_ptr = statespace_->addState(goal_state_val);
    auto goal_node = std::make_unique<DStarLiteNode>(goal_state_ptr, 0);
    goal_node_ = goal_node.get(); // Set pointer immediately
    nodes_.push_back(std::move(goal_node));

    // Add Start Node --> robot 
    auto start_state_ptr = statespace_->addState(start_state_val);
    auto start_node = std::make_unique<DStarLiteNode>(start_state_ptr, 1);
    start_node_ = start_node.get(); // Set pointer immediately
    nodes_.push_back(std::move(start_node));
    // Add Random Samples
    for (int i = 0; i < num_samples_; ++i) {
        auto state_ptr = statespace_->sampleUniform(lower_bounds_, upper_bounds_);
        auto node = std::make_unique<DStarLiteNode>(state_ptr, nodes_.size());
        nodes_.push_back(std::move(node));
    }

    std::cout << "PRM Built with " << nodes_.size() << " nodes (including Start/Goal).\n";

    // 4. Build KD-Tree
    if (use_kdtree_) {
        Eigen::MatrixXd all_samples = statespace_->getSamplesCopy();
        Eigen::MatrixXd spatial_samples_only = all_samples.leftCols(kd_dim_).eval();
        kdtree_->addPoints(spatial_samples_only);
        kdtree_->buildTree();
    }

    // 5. Calculate Radius
    int d = statespace_->getDimension();
    if (use_knn_) {
        double k0_fmt_star_practical = std::pow(2.0, d) * (M_E / d);
        k_neighbors_ = static_cast<int>(std::ceil(factor_ * k0_fmt_star_practical * std::log(statespace_->getNumStates())));
    } else {
        Eigen::VectorXd range = upper_bounds_ - lower_bounds_;
        mu_ = range.prod();
        zetaD_ = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
        gamma_ = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu_ / zetaD_, 1.0 / d);
        connection_radius_ = factor_ * gamma_ * std::pow(std::log(statespace_->getNumStates()) / statespace_->getNumStates(), 1.0 / d);
    }

    // 6. Neighbor Pre-caching
    if (neighbor_precache_) {
        std::cout << "Forcing neighbor caching for all " << nodes_.size() << " nodes..." << std::endl;
        auto cache_start = std::chrono::high_resolution_clock::now();
        for (size_t i = 0; i < nodes_.size(); ++i) {
            near(i);
        }
        auto cache_end = std::chrono::high_resolution_clock::now();
        auto cache_duration = std::chrono::duration_cast<std::chrono::milliseconds>(cache_end - cache_start);
        std::cout << "Neighbor caching complete. Time taken: " << cache_duration.count() << " ms." << std::endl;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";


    std::cout << "\n=== D* Lite Connectivity Check ===" << std::endl;
    
    // Check Start Node
    std::cout << "Start Node ID: " << start_node_->getIndex() << std::endl;
    std::cout << "Start Backward Neighbors (Incoming): " << start_node_->backward_neighbors_.size() << std::endl;
    std::cout << "Start Forward Neighbors (Outgoing): " << start_node_->forward_neighbors_.size() << std::endl;
    
    // Check Goal Node
    std::cout << "Goal Node ID: " << goal_node_->getIndex() << std::endl;
    std::cout << "Goal Backward Neighbors (Incoming): " << goal_node_->backward_neighbors_.size() << std::endl;
    std::cout << "Goal Forward Neighbors (Outgoing): " << goal_node_->forward_neighbors_.size() << std::endl;
    
    std::cout << "====================================\n" << std::endl;


}

// --- NEW: Neighbor Caching Logic ---
void KinodynamicPRMStarDStarLite::near(int node_index) {
    auto node = nodes_[node_index].get();
    
    if (node->neighbors_cached_) return;

    // 1. Get candidate neighbors
    std::vector<size_t> candidate_indices;
    if (use_knn_) {
        if (k_neighbors_ > 0) {
            candidate_indices = kdtree_->knnSearch(node->getStateValue().head(kd_dim_), k_neighbors_);
        }
    } else {
        if (connection_radius_ > 0) {
            candidate_indices = kdtree_->radiusSearch(node->getStateValue().head(kd_dim_), connection_radius_);
        }
    }

    // 2. Populate Neighbors (Proactive Strategy)
    // We assume statespace_->prefersLazyNear() is FALSE for your setup, 
    // meaning we compute trajectories now.
    
    for (int idx : candidate_indices) {
        if (idx == node_index) continue;
        DStarLiteNode* neighbor = nodes_[idx].get();

        // --- Test FORWARD connection (Node -> Neighbor) ---
        Trajectory traj_forward = statespace_->steer(node->getStateValue(), neighbor->getStateValue());
        
        // We only care if the trajectory is valid (steering constraints).
        // We do NOT check obstacle collisions here.
        if (traj_forward.is_valid) {
            EdgeInfo info_forward;
            info_forward.distance = traj_forward.cost; // Or traj_forward.cost depending on your metric
            info_forward.distance_original = info_forward.distance;
            info_forward.cached_trajectory = traj_forward;
            info_forward.is_trajectory_computed = true;
            // No invalidating_obstacles insertion here!

            // Store: Node -> Neighbor
            node->forward_neighbors_[neighbor] = info_forward;
            // Store Reverse: Neighbor -> Node (This is the backward edge for Neighbor)
            neighbor->backward_neighbors_[node] = info_forward;
        }

        // --- Test BACKWARD connection (Neighbor -> Node) ---
        // CRITICAL: This is a different trajectory!
        Trajectory traj_backward = statespace_->steer(neighbor->getStateValue(), node->getStateValue());
        
        if (traj_backward.is_valid) {
            EdgeInfo info_backward;
            info_backward.distance = traj_backward.cost;
            info_backward.distance_original = info_backward.distance;
            info_backward.cached_trajectory = traj_backward;
            info_backward.is_trajectory_computed = true;

            // Store: Neighbor -> Node
            neighbor->forward_neighbors_[node] = info_backward;
            // Store Reverse: Node -> Neighbor (This is the backward edge for Node)
            node->backward_neighbors_[neighbor] = info_backward;
        }
    }

    node->neighbors_cached_ = true;
}


void KinodynamicPRMStarDStarLite::plan() {
    if (!start_node_ || !goal_node_) return;
    initialize(start_node_, goal_node_);
    computeShortestPath();
}

double KinodynamicPRMStarDStarLite::heuristic(DStarLiteNode* a, DStarLiteNode* b) {
    return (a->getStateValue() - b->getStateValue()).norm();
}


void KinodynamicPRMStarDStarLite::updateNodePriority(DStarLiteNode* u) {
    // Calculate k1 and k2 directly
    double min_val = std::min(u->g, u->rhs);
    
    double h_val = 0.0;

    // SAFETY CHECK: Only use heuristic if start_node_ exists
    if (start_node_) {
        h_val = heuristic(u, start_node_);
    } 
    // If start_node_ is null (e.g. during initial setup before robot is placed),
    // we assume h=0. This is safe because the queue will sort primarily by g/rhs initially.
    
    // D* Lite Key Calculation
    u->k1 = min_val + h_val + km_;
    u->k2 = min_val;
    
    // Update the standard priority key for the queue
    u->priority_key_ = u->k1; 
}

void KinodynamicPRMStarDStarLite::initialize(DStarLiteNode* start, DStarLiteNode* goal) {
    km_ = 0.0;
    
    // No need to loop through nodes to set g/rhs to infinity because 
    // the DStarLiteNode constructor does that, and we assume the graph 
    // hasn't changed its structure (nodes added/removed) since setup.
    // We only need to ensure the queue is clean.
    
    open_queue_.clear();
    
    // However, if you are re-planning from scratch (e.g. after a reset), 
    // you MUST reset g/rhs. If this is called only once at startup, 
    // the constructor handles it. 
    // If you call this to reset the search, uncomment the loop:
    /*
    for (auto& node : nodes_) {
        node->g = std::numeric_limits<double>::infinity();
        node->rhs = std::numeric_limits<double>::infinity();
        node->heap_index_ = -1;
        node->in_queue_ = false;
    }
    */

    start_node_ = start;
    goal_node_ = goal;
    
    goal->rhs = 0.0;
    
    updateNodePriority(goal);
    open_queue_.add(goal, goal->priority_key_);
}
// void KinodynamicPRMStarDStarLite::updateVertex(DStarLiteNode* u) {
//     // 1. Calculate RHS and Find Best Parent
//     if (u != goal_node_) {
//         double min_rhs = std::numeric_limits<double>::infinity();
//         DStarLiteNode* best_pred = nullptr;
//         Trajectory best_traj;

//         // Look at predecessors (incoming edges) to find min(g(s') + c(s', u))
//         for (auto& [pred, edge_info] : u->backward_neighbors_) {
//             // Skip invalid edges
//             if (edge_info.invalidating_obstacles.empty()) {
//                 double cost = pred->g + edge_info.distance;
                
//                 // If this path is cheaper than what we've seen, update the best parent
//                 if (cost < min_rhs) {
//                     min_rhs = cost;
//                     best_pred = pred;
//                     best_traj = edge_info.cached_trajectory;
//                 }
//             }
//         }
        
//         // Update the D* Lite rhs value
//         u->rhs = min_rhs;
        
//         // --- HERE IS WHERE WE SET THE PARENT ---
//         u->best_parent_ = best_pred;
//         u->best_parent_trajectory_ = best_traj;
//     } else {
//         // The goal node has no parent (it is the end of the search)
//         u->best_parent_ = nullptr;
//     }

//     // 2. Update Priority Queue (Standard D* Lite Logic)
//     if (u->in_queue_) {
//         if (u->rhs != u->g) {
//             updateNodePriority(u);
//             open_queue_.update(u, u->priority_key_);
//         } else {
//             open_queue_.remove(u);
//         }
//     } else {
//         if (u->rhs != u->g) {
//             updateNodePriority(u);
//             open_queue_.add(u, u->priority_key_);
//         }
//     }
// }
// void KinodynamicPRMStarDStarLite::updateVertex(DStarLiteNode* u) {
//     // // std::cout << "[UpdateVertex] Node " << u->getIndex() << std::endl; // Optional logging
//     // bool verbose = true;
//     // // bool verbose = (u == start_node_); 


//     // if (verbose) {
//     //     std::cout << "\n[UpdateVertex] Processing Node " << u->getIndex() << " (Start Node)" << std::endl;
//     //     std::cout << "  Current g: " << (u->g == std::numeric_limits<double>::infinity() ? "INF" : std::to_string(u->g)) << std::endl;
//     // }

//     if (u != goal_node_) {
//         double min_rhs = std::numeric_limits<double>::infinity();
//         DStarLiteNode* best_parent = nullptr; // This will point to the node closer to Goal
//         Trajectory best_traj;

//         // // --- LOGGING START ---
//         // if (verbose) {
//         //     std::cout << "  Checking Forward Neighbors (Successors): " << u->forward_neighbors_.size() << std::endl;
//         //     std::cout << "  Checking Backward Neighbors (Predecessors): " << u->backward_neighbors_.size() << std::endl;
//         // }
//         // // --- LOGGING END ---


//         // CRITICAL FIX: Iterate over forward_neighbors_
//         // In your physics graph, forward_neighbors_ point towards the Goal.
//         // D* Lite needs to look at "Predecessors" (nodes that can reach u).
//         // Since edges go Start->Goal, the nodes that can reach u are in u->forward_neighbors_.
//         for (auto& [succ, edge_info] : u->forward_neighbors_) {
//             if (!edge_info.invalidating_obstacles.empty()) continue;
//             if (succ->g == std::numeric_limits<double>::infinity()) continue;

//             double cost = edge_info.distance + succ->g;
            
//             if (cost < min_rhs) {
//                 min_rhs = cost;
//                 best_parent = succ; // The successor is the next step to Goal
//                 best_traj = edge_info.cached_trajectory;
//             }
//         }

//         u->rhs = min_rhs;
//         u->best_parent_ = best_parent; // Points towards Goal
//         u->best_parent_trajectory_ = best_traj;
//     } else {
//         u->best_parent_ = nullptr;
//         u->rhs = 0.0;
//         if (u->g != 0.0) u->g = 0.0;
//     }

//     // Queue Logic (Unchanged)
//     if (u->in_queue_) {
//         if (u->rhs != u->g) {
//             updateNodePriority(u);
//             open_queue_.update(u, u->priority_key_);
//         } else {
//             open_queue_.remove(u);
//         }
//     } else {
//         if (u->rhs != u->g) {
//             updateNodePriority(u);
//             open_queue_.add(u, u->priority_key_);
//         }
//     }
// }

void KinodynamicPRMStarDStarLite::updateVertex(DStarLiteNode* u) {
    if (u != goal_node_) {
        double min_rhs = std::numeric_limits<double>::infinity();
        DStarLiteNode* best_parent = nullptr;
        Trajectory best_traj;

        // --- PROPAGATE ALONG TIME-DAG ---
        // Forward neighbors: nodes that u can reach (time decreases)
        for (auto& [succ, edge_info] : u->forward_neighbors_) {
            if (!edge_info.invalidating_obstacles.empty()) continue;
            if (succ->g == std::numeric_limits<double>::infinity()) continue;

            double cost = edge_info.distance + succ->g;
            if (cost < min_rhs) {
                min_rhs = cost;
                best_parent = succ;
                best_traj = edge_info.cached_trajectory;
            }
        }

        u->rhs = min_rhs;
        u->best_parent_ = best_parent;
        u->best_parent_trajectory_ = best_traj;
    } else {
        u->rhs = 0.0;
        u->g = 0.0;
        u->best_parent_ = nullptr;
    }

    // --- UPDATE PRIORITY QUEUE ---
    if (u->rhs != u->g) {
        updateNodePriority(u);
        if (u->in_queue_) open_queue_.update(u, u->priority_key_);
        else open_queue_.add(u, u->priority_key_);
    } else if (u->in_queue_) {
        open_queue_.remove(u);
    }
}





void KinodynamicPRMStarDStarLite::printNodeDetails(const std::string& prefix, DStarLiteNode* node) {
    if (!node) {
        std::cout << prefix << " (NULL)" << std::endl;
        return;
    }
    Eigen::VectorXd val = node->getStateValue();
    // Assuming 2D or 3D, printing first two components for visual verification
    std::cout << prefix 
              << " [ID: " << node->getIndex() 
              << "] Pos: (" << val(0) << ", " << val(1) << ")" 
              << " g: " << (node->g == std::numeric_limits<double>::infinity() ? -1.0 : node->g)
              << " rhs: " << (node->rhs == std::numeric_limits<double>::infinity() ? -1.0 : node->rhs)
              << std::endl;
}

// void KinodynamicPRMStarDStarLite::computeShortestPath() {
//     // std::cout << "[computeShortestPath] Starting..." << std::endl;
    
//     while (!open_queue_.empty()) {
//         auto top_pair = open_queue_.top();
//         DStarLiteNode* u = top_pair.second;
        
//         double old_k1 = u->k1;
//         double old_k2 = u->k2;
//         updateNodePriority(u);

//         bool keys_equal = (std::abs(old_k1 - u->k1) < 1e-9) && (std::abs(old_k2 - u->k2) < 1e-9);
//         bool old_less = false;
//         if (!keys_equal) {
//             if (std::abs(old_k1 - u->k1) > 1e-9) old_less = (old_k1 < u->k1);
//             else old_less = (old_k2 < u->k2);
//         }

//         if (old_less) {
//             open_queue_.update(u, u->priority_key_);
//         }
//         else if (u->g > u->rhs) {
//             u->g = u->rhs;
//             open_queue_.pop();

//             // CRITICAL FIX: Update backward_neighbors_
//             // If u's cost changed, we must update nodes that have an edge TO u.
//             // In your graph, these are stored in backward_neighbors_.
//             for (auto& [pred, edge_info] : u->backward_neighbors_) {
//                  if (edge_info.invalidating_obstacles.empty()) {
//                     updateVertex(pred);
//                 }
//             }
//         }
//         else if (u->g < u->rhs) {
//             u->g = std::numeric_limits<double>::infinity();
//             updateVertex(u);
            
//             // CRITICAL FIX: Update backward_neighbors_
//             for (auto& [pred, edge_info] : u->backward_neighbors_) {
//                  if (edge_info.invalidating_obstacles.empty()) {
//                     updateVertex(pred);
//                 }
//             }
//         }
//         else {
//             open_queue_.pop();
//         }
//     }
// }


void KinodynamicPRMStarDStarLite::computeShortestPath() {
    if (!start_node_ || !goal_node_) return;

    while (!open_queue_.empty()) {
        DStarLiteNode* u = open_queue_.top().second;

        double old_k1 = u->k1;
        double old_k2 = u->k2;
        updateNodePriority(u);

        // --- Priority queue reordering if keys increased ---
        bool keys_equal = (std::abs(old_k1 - u->k1) < 1e-9) && (std::abs(old_k2 - u->k2) < 1e-9);
        bool old_less = false;
        if (!keys_equal) {
            if (std::abs(old_k1 - u->k1) > 1e-9) old_less = (old_k1 < u->k1);
            else old_less = (old_k2 < u->k2);
        }

        if (old_less) {
            open_queue_.update(u, u->priority_key_);
            continue; // wait for next pop
        }

        // --- Termination check: start node consistent + robot bridge cost ---
        if (start_node_->rhs == start_node_->g &&
            u->k1 >= start_node_->g + bridge_cost_) {
            break;
        }

        open_queue_.pop();

        if (u->g > u->rhs) {
            // Cost decreased → propagate to predecessors
            u->g = u->rhs;

            for (auto& [pred, edge_info] : u->backward_neighbors_) {
                if (edge_info.invalidating_obstacles.empty()) {
                    updateVertex(pred);
                }
            }
        } else if (u->g < u->rhs) {
            // Cost increased → propagate to predecessors
            u->g = std::numeric_limits<double>::infinity();
            updateVertex(u);

            for (auto& [pred, edge_info] : u->backward_neighbors_) {
                if (edge_info.invalidating_obstacles.empty()) {
                    updateVertex(pred);
                }
            }
        }
        // else g == rhs → nothing to do
    }
}


DStarLiteNode* KinodynamicPRMStarDStarLite::findNearestNode(const Eigen::VectorXd& state) {
    if (nodes_.empty()) return nullptr;
    double min_dist = std::numeric_limits<double>::infinity();
    DStarLiteNode* nearest = nullptr;
    for (const auto& node : nodes_) {
        double dist = (node->getStateValue() - state).norm();
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node.get();
        }
    }
    return nearest;
}


void KinodynamicPRMStarDStarLite::connectNeighbors(DStarLiteNode* u) {
    // If we pre-cached, we don't need to do anything here.
    if (neighbor_precache_) return;

    // Fallback to dynamic connection if not cached
    std::vector<DStarLiteNode*> neighbors = getNeighbors(u);
    for (auto* v : neighbors) {
        if (u == v) continue;
        if (u->forward_neighbors_.find(v) != u->forward_neighbors_.end()) continue;

        Trajectory traj = statespace_->steer(u->getStateValue(), v->getStateValue());
        bool is_safe = obs_checker_->isTrajectorySafe(traj, 0.0); 
        
        EdgeInfo info;
        info.distance = traj.cost; 
        info.distance_original = info.distance;
        info.cached_trajectory = traj;
        info.is_trajectory_computed = true;
        
        if (!is_safe) {
            info.invalidating_obstacles.insert("static_collision"); 
        }

        u->forward_neighbors_[v] = info;
        v->backward_neighbors_[u] = info;
    }
}


std::vector<DStarLiteNode*> KinodynamicPRMStarDStarLite::getNeighbors(DStarLiteNode* u) {
    std::vector<DStarLiteNode*> result;
    if (!use_kdtree_) {
        for (const auto& node : nodes_) {
            if (node.get() == u) continue;
            double dist = (node->getStateValue() - u->getStateValue()).norm();
            if (dist <= connection_radius_) {
                result.push_back(node.get());
            }
        }
    } else {
        // KDTree logic
    }
    return result;
}

bool KinodynamicPRMStarDStarLite::updateObstacleSamples(const ObstacleVector& turned_obstacles) {
    bool graph_changed = false;

    for (const auto& ob : turned_obstacles) {
        // Check if this obstacle is new or just turning
        // Assuming 'previous_obstacles_' tracks the state
        bool is_new = (previous_obstacles_.find(ob.name) == previous_obstacles_.end());
        
        if (is_new) {
            addNewObstacle(ob);
            graph_changed = true;
        } else {
            // It's a "turnaround" or update event.
            // Strategy: Treat the old path as a "remove" and the new path as an "add".
            // This ensures we don't miss edges that become free when the obstacle moves away.
            
            const Obstacle& old_ob = previous_obstacles_[ob.name];
            
            // 1. Remove old influence
            removeObstacle(old_ob); // This restores edges that are now safe
            
            // 2. Add new influence
            addNewObstacle(ob);     // This invalidates edges hit by the new path
            
            graph_changed = true;
        }
        
        // Update history
        previous_obstacles_[ob.name] = ob;
    }

    // 3. Trigger D* Lite Repair if graph changed
    if (graph_changed) {
        // Update km (key modifier) for D* Lite
        // km += heuristic(last_start, current_start)
        // In your implementation, start_node_ is updated externally via setRobotState
        // You need to track the 'last_start_node_' to calculate this correctly.
        // For simplicity, if you update start_node_ before calling this, you might need to adjust km.
        // Standard D* Lite: km = km + h(s_last, s_start).
        
        computeShortestPath();
    }

    return graph_changed;
}

void KinodynamicPRMStarDStarLite::addNewObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    // 1. Calculate Search Radius
    // We need to cover the "tube" of the obstacle. 
    // Radius = Obstacle Size + Inflation + Robot Radius + Gap Coverage
    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    
    // Gap coverage ensures we don't miss edges between prediction samples
    double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
    double search_radius = obs_r + ob.inflation + gap_coverage_inflation;

    // 2. Find Nodes near the obstacle's path
    std::unordered_set<int> unique_node_indices;
    
    for (const auto& point_3d : ob.predicted_path) {
        Eigen::VectorXd query(kd_dim_);
        // Construct query based on your KDTree dimension (e.g., x, y, time)
        if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
        else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
        
        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) {
            unique_node_indices.insert(static_cast<int>(idx));
        }
    }

    // 3. Check Edges and Invalidate if necessary
    for (int idx : unique_node_indices) {
        DStarLiteNode* u = nodes_[idx].get();
        
        // Check outgoing edges (u -> v)
        for (auto& [v, edge_info] : u->forward_neighbors_) {
            // If already invalid or not computed, skip
            if (!edge_info.is_trajectory_computed || edge_info.distance == std::numeric_limits<double>::infinity()) continue;

            // Check collision specifically against THIS new obstacle
            // We use the cached trajectory from the PRM construction
            double edge_start_time = u->getStateValue()(u->getStateValue().size() - 1); // Assuming last dim is time
            
            if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(edge_info.cached_trajectory, edge_start_time, ob)) {
                // INVALIDATE EDGE
                edge_info.distance = std::numeric_limits<double>::infinity();
                edge_info.invalidating_obstacles.insert(ob.name);
                
                // D* Lite requires updating the vertex if its incoming edge costs change.
                // Since u -> v is an incoming edge to v, we must update v.
                updateVertex(v);
            }
        }
    }
}

void KinodynamicPRMStarDStarLite::removeObstacle(const Obstacle& ob) {
    if (ob.predicted_path.empty()) return;

    // 1. Setup Search Radius (Matches your RRTX logic)
    double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : 
                   std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
    
    // Gap coverage inflation: R * (sqrt(2) - 1)
    double gap_coverage_inflation = obs_r * (std::sqrt(2.0) - 1.0); 
    double search_radius = obs_r + ob.inflation + gap_coverage_inflation; // + delta if you have it

    // 2. Gather Unique Nodes via KD-Tree
    std::unordered_set<int> unique_node_indices;
    
    for (const auto& point_3d : ob.predicted_path) {
        Eigen::VectorXd query(kd_dim_);
        // Handle dimensions based on your specific StateSpace setup
        if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z(); 
        else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
        
        std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
        for (size_t idx : indices) {
            unique_node_indices.insert(static_cast<int>(idx));
        }
    }

    // 3. Process Nodes (Repair & Restoration)
    for (int idx : unique_node_indices) {
        DStarLiteNode* u = nodes_[idx].get();
        
        // Iterate over OUTGOING edges (u -> v)
        // In D* Lite, if edge (u->v) cost changes, we must update 'v'
        for (auto& [v, edge_info] : u->forward_neighbors_) {
            
            // LOGIC STEP 1: Was this edge actually blocked?
            if (edge_info.distance == std::numeric_limits<double>::infinity()) {
                
                // LOGIC STEP 2: Was it blocked by THIS specific obstacle?
                // We check if 'ob.name' exists in the set of invalidators.
                auto it = edge_info.invalidating_obstacles.find(ob.name);
                
                if (it != edge_info.invalidating_obstacles.end()) {
                    // It WAS blocked by this obstacle. Remove the blame.
                    edge_info.invalidating_obstacles.erase(it);
                    
                    // LOGIC STEP 3: Is it still blocked by ANY OTHER obstacle?
                    // Instead of re-running collision checks against all obstacles (expensive),
                    // we simply check if the set is empty.
                    if (edge_info.invalidating_obstacles.empty()) {
                        
                        // It is finally free! Restore original cost.
                        edge_info.distance = edge_info.distance_original;
                        
                        // LOGIC STEP 4: Trigger Graph Repair
                        // In RRTX you call updateLMC(u). 
                        // In D* Lite, since the cost of edge u->v changed, we update v.
                        updateVertex(v);
                    }
                }
            }
        }
    }
}
void KinodynamicPRMStarDStarLite::setRobotState(const Eigen::VectorXd& robot_state) {
    robot_continuous_state_ = robot_state;

    // Extract time for collision checking
    double robot_time = robot_continuous_state_(robot_continuous_state_.size() - 1);

    // --- 1. HYSTERESIS LOGIC ---
    const double hysteresis_factor = 0.98;
    double cost_of_current_anchor = std::numeric_limits<double>::infinity();
    
    if (start_node_ && start_node_->g != std::numeric_limits<double>::infinity()) {
        Trajectory bridge = statespace_->steer(robot_continuous_state_, start_node_->getStateValue());
        if (bridge.is_valid && obs_checker_->isTrajectorySafe(bridge, robot_time)) {
            cost_of_current_anchor = bridge.cost + start_node_->g;
        }
    }

    DStarLiteNode* best_candidate_node = nullptr;
    double best_candidate_cost = std::numeric_limits<double>::infinity();
    
    // --- 2. RADIUS EXPANSION LOGIC ---
    double current_search_radius = connection_radius_;
    const int max_attempts = 5;
    const double radius_multiplier = 2.0;
    
    // std::cout << "\n========== [setRobotState] ==========" << std::endl;
    // std::cout << "Robot Pos: (" << robot_state(0) << ", " << robot_state(1) << ")" << std::endl;
    // std::cout << "Initial Radius: " << current_search_radius << std::endl;

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        std::vector<size_t> candidate_indices;
        
        // Search within current radius
        if (use_kdtree_) {
            candidate_indices = kdtree_->radiusSearch(robot_state.head(kd_dim_), current_search_radius);
        }

        // std::cout << "Attempt " << attempt << " (Radius: " << current_search_radius 
        //           << "): Found " << candidate_indices.size() << " candidates." << std::endl;

        // Evaluate Candidates
        for (size_t idx : candidate_indices) {
            DStarLiteNode* candidate = nodes_[idx].get();

            // 1. CRITICAL: Candidate must be reachable from Goal (Finite G)
            if (candidate->g == std::numeric_limits<double>::infinity()) {
                continue;
            }

            // 2. Check Steering & Collision
            Trajectory bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());
            
            if (!bridge.is_valid) continue;
            if (!obs_checker_->isTrajectorySafe(bridge, robot_time)) continue;

            // 3. Calculate Total Cost: Bridge Cost + Node Cost
            double total_cost = bridge.cost + candidate->g;

            if (total_cost < best_candidate_cost) {
                best_candidate_cost = total_cost;
                best_candidate_node = candidate;
                bridge_cost_ = bridge.cost;
            }
        }

        // If we found a valid candidate, stop expanding
        if (best_candidate_node) {
            // std::cout << "  -> Found valid candidate in this radius." << std::endl;
            break;
        }

        // Expand radius for next attempt
        current_search_radius *= radius_multiplier;
    }

    // --- 3. ASSIGNMENT ---
    if (best_candidate_node && best_candidate_cost < cost_of_current_anchor * hysteresis_factor) {
        std::cout << ">>> SWITCHING Anchor to Node " << best_candidate_node->getIndex() 
                  << " (Cost: " << best_candidate_cost << ") <<<" << std::endl;
        
        // Update km (Key Modifier) for D* Lite if the start node changed
        if (start_node_ && start_node_ != best_candidate_node) {
            km_ += heuristic(start_node_, best_candidate_node);
        }
        
        start_node_ = best_candidate_node;
    } else if (start_node_ && cost_of_current_anchor != std::numeric_limits<double>::infinity()) {
        // std::cout << ">>> KEEPING Current Anchor Node " << start_node_->getIndex() 
        //           << " (Cost: " << cost_of_current_anchor << ") <<<" << std::endl;
    } else {
        std::cout << ">>> FAILURE: No valid anchor found even after expanding radius! <<<" << std::endl;
        start_node_ = nullptr;
        bridge_cost_= std::numeric_limits<double>::infinity();
    }
    
    // std::cout << "======================================\n" << std::endl;

    // // 4. Trigger Repair if necessary
    // if (open_queue_.empty() && start_node_ && start_node_->g != start_node_->rhs) {
    //      std::cout << "Triggering Repair..." << std::endl;
    //      computeShortestPath();
    // }
}

// void KinodynamicPRMStarDStarLite::visualizeGraph() {
//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> one_way_edges;

//     edges.reserve(nodes_.size() * (use_knn_ ? k_neighbors_ : 20));
//     one_way_edges.reserve(nodes_.size() * 2); // Reserve some space for one-way edges

//     for (const auto& node_ptr : nodes_) {
//         DStarLiteNode* u = node_ptr.get();

//         for (const auto& [neighbor, edge_info] : u->forward_neighbors_) {
//             if (edge_info.is_trajectory_computed) {
//                 const Eigen::Vector2d p1 = u->getStateValue().head<2>();
//                 const Eigen::Vector2d p2 = neighbor->getStateValue().head<2>();

//                 // Check if the reverse connection exists
//                 bool has_reverse = false;
//                 auto it = neighbor->forward_neighbors_.find(u);
//                 if (it != neighbor->forward_neighbors_.end()) {
//                     has_reverse = it->second.is_trajectory_computed;
//                 }

//                 if (has_reverse) {
//                     // Bidirectional connection -> Draw Gray
//                     edges.emplace_back(p1, p2);
//                 } else {
//                     // One-way connection -> Draw Red (Warning)
//                     one_way_edges.emplace_back(p1, p2);
//                 }
//             }
//         }
//     }

//     // 1. Visualize Normal Edges (Gray)
//     visualization_->visualizeEdges(edges, "map", "0.7,0.7,0.7", "prm_graph");

//     // 2. Visualize One-Way Edges (Red) - Optional, helps debugging
//     if (!one_way_edges.empty()) {
//         std::cout << "[Visualize] Found " << one_way_edges.size() << " one-way edges." << std::endl;
//         visualization_->visualizeEdges(one_way_edges, "map", "1.0,0.0,0.0", "prm_one_way");
//     }
// }


void KinodynamicPRMStarDStarLite::visualizeGraph() {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> one_way_edges;
    
    edges.reserve(nodes_.size() * (use_knn_ ? k_neighbors_ : 20));
    one_way_edges.reserve(nodes_.size() * 2);

    for (const auto& node_ptr : nodes_) {
        DStarLiteNode* u = node_ptr.get();

        // 1. Filter Nodes
        // Draw if reachable (Finite G) OR if it is the Start Node (always draw start for debugging)
        bool is_reachable = (u->g != std::numeric_limits<double>::infinity());
        bool is_start = (u == start_node_);

        if (!is_reachable && !is_start) {
            continue; // Skip disconnected islands (unless it's the start)
        }

        for (const auto& [neighbor, edge_info] : u->forward_neighbors_) {
            
            // 2. Filter Neighbors
            // Draw if neighbor is reachable OR if neighbor is Start
            bool neighbor_reachable = (neighbor->g != std::numeric_limits<double>::infinity());
            bool neighbor_is_start = (neighbor == start_node_);

            if (!neighbor_reachable && !neighbor_is_start) {
                continue;
            }

            if (edge_info.is_trajectory_computed) {
                const Eigen::Vector2d p1 = u->getStateValue().head<2>();
                const Eigen::Vector2d p2 = neighbor->getStateValue().head<2>();

                // Check if the reverse connection exists
                bool has_reverse = false;
                auto it = neighbor->forward_neighbors_.find(u);
                if (it != neighbor->forward_neighbors_.end()) {
                    has_reverse = it->second.is_trajectory_computed;
                }

                if (has_reverse) {
                    // Bidirectional connection -> Draw Gray
                    edges.emplace_back(p1, p2);
                } else {
                    // One-way connection -> Draw Red
                    one_way_edges.emplace_back(p1, p2);
                }
            }
        }
    }

    // 1. Visualize Normal Edges (Gray)
    visualization_->visualizeEdges(edges, "map", "0.7,0.7,0.7", "prm_graph");

    // 2. Visualize One-Way Edges (Red)
    if (!one_way_edges.empty()) {
        visualization_->visualizeEdges(one_way_edges, "map", "1.0,0.0,0.0", "prm_one_way");
    }
}


std::vector<Eigen::VectorXd> KinodynamicPRMStarDStarLite::getPathPositions() {
    std::vector<Eigen::VectorXd> path;
    if (!start_node_ || !goal_node_) return path;

    // std::cout << "\n--- [Extract Path] ---" << std::endl;
    // std::cout << "Anchor Node: " << start_node_->getIndex() << std::endl;

    // 1. Add Bridge (Robot -> Anchor)
    Trajectory bridge = statespace_->steer(robot_continuous_state_, start_node_->getStateValue());
    if (bridge.is_valid && !bridge.path_points.empty()) {
        path.insert(path.end(), bridge.path_points.begin(), bridge.path_points.end());
        // std::cout << "Added bridge to anchor." << std::endl;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("DStarLite"), "Cannot steer from robot to anchor node.");
        return {};
    }

    // 2. Traverse Graph (Anchor -> Goal)
    DStarLiteNode* current_node = start_node_;
    int steps = 0;
    const int max_steps = nodes_.size();

    while (current_node != goal_node_) {
        if (steps++ > max_steps) {
            // RCLCPP_ERROR(rclcpp::get_logger("DStarLite"), "Path extraction exceeded max steps.");
            break;
        }
        
        DStarLiteNode* next_node = current_node->best_parent_;
        if (!next_node) {
            // RCLCPP_ERROR(rclcpp::get_logger("DStarLite"),
            //              "Path broken at Node %d. Next parent is NULL.",
            //              current_node->getIndex());
            break;
        }

        const Trajectory& traj = current_node->best_parent_trajectory_;
        if (traj.is_valid && traj.path_points.size() > 1) {
            path.insert(path.end(), traj.path_points.begin() + 1, traj.path_points.end());
        } else {
            path.push_back(next_node->getStateValue());
        }
        current_node = next_node;
    }
    
    // std::cout << "Path extracted with " << path.size() << " points." << std::endl;
    // std::cout << "----------------------\n" << std::endl;

    return path;
}


void KinodynamicPRMStarDStarLite::visualizePath(const std::vector<Eigen::VectorXd>& path_waypoints) {
    // A path needs at least two points to have an edge.
    if (path_waypoints.size() < 2) {
        return;
    }

    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    // Iterate through the waypoints to create line segments.
    // The loop goes to size() - 1 to prevent going out of bounds.
    for (size_t i = 0; i < path_waypoints.size() - 1; ++i) {
        // Create an edge from the current point to the next point.
        const Eigen::VectorXd& start_point = path_waypoints[i];
        const Eigen::VectorXd& end_point = path_waypoints[i+1];
        edges.emplace_back(start_point.head(2), end_point.head(2));
    }

    // Use your existing visualization class to draw the edges.
    // We'll use a distinct namespace and color (e.g., green and thick) to see it clearly.
    if (visualization_) {
        // The last argument is a namespace to keep it separate from the main tree visualization.
        visualization_->visualizeEdges(edges, "map", "0.0,1.0,0.0", "executable_path");
    }
}
