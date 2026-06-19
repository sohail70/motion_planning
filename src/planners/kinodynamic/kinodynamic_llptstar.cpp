#include "motion_planning/planners/kinodynamic/kinodynamic_llptstar.hpp"

#define DEBUG_WITH_DIJKSTRA_ 0
#define USE_INVALIDATING_SET_STRATEGY 0
#define USE_THREAT_SET_STRATEGY 0
#define USE_GRID_SAMPLING 0
#define USE_RECOVERY 0 // Emergency Fallback
// Constructor
KinodynamicLLPTStar::KinodynamicLLPTStar(
    std::shared_ptr<StateSpace> statespace, 
    std::shared_ptr<ProblemDefinition> pdef,
    std::shared_ptr<ObstacleChecker> obs_checker)
    : statespace_(statespace), problem_def_(pdef), obs_checker_(obs_checker), km_(0.0) {
    std::cout << "KinodynamicLLPTStar Constructor \n";
}

void KinodynamicLLPTStar::setCurrentRobotTime(double robot_time_) {
    T_robot = robot_time_;
}

void KinodynamicLLPTStar::near(int node_index) {
    auto node = nodes_[node_index].get();
    if (node->neighbors_cached_) return;

    std::vector<size_t> candidate_indices;
    candidate_indices = kdtree_->radiusSearch(node->getStateValue().head(kd_dim_), connection_radius_ + std::numeric_limits<double>::epsilon());

    for (int idx : candidate_indices) {
        if (idx == node_index) continue;
        DStarLiteNode* neighbor = nodes_[idx].get();

        // Test FORWARD connection
        Trajectory traj_forward = statespace_->steer(node->getStateValue(), neighbor->getStateValue());
        std::shared_ptr<Trajectory> shared_traj_forward;
        
        if (traj_forward.is_valid && traj_forward.cost <= connection_radius_ + std::numeric_limits<double>::epsilon()) {
            shared_traj_forward = std::make_shared<Trajectory>(std::move(traj_forward));
            
            EdgeInfo info_forward;
            info_forward.distance = shared_traj_forward->cost;
            info_forward.distance_original = shared_traj_forward->cost;
            info_forward.cached_trajectory = shared_traj_forward;
            info_forward.is_trajectory_computed = true;
            // info_forward.is_evaluated = false; // LLPT Lazy Flag
            info_forward.last_eval_epoch = 0;
            
            node->forward_neighbors_[neighbor] = info_forward;
            neighbor->backward_neighbors_[node] = info_forward;
        }

        // Test BACKWARD connection
        if (is_geometric_mode_) {
            if (shared_traj_forward) { // Only assign if forward was valid
                EdgeInfo info_backward;
                info_backward.distance = shared_traj_forward->cost;
                info_backward.distance_original = shared_traj_forward->cost;
                info_backward.cached_trajectory = shared_traj_forward;
                info_backward.is_trajectory_computed = true;
                // info_backward.is_evaluated = false; // LLPT Lazy Flag
                info_backward.last_eval_epoch = 0;
                
                neighbor->forward_neighbors_[node] = info_backward;
                node->backward_neighbors_[neighbor] = info_backward;
            }
        } else {
            Trajectory traj_backward = statespace_->steer(neighbor->getStateValue(), node->getStateValue());
            if (traj_backward.is_valid && traj_backward.cost <= connection_radius_ + std::numeric_limits<double>::epsilon()) {
                auto shared_traj_backward = std::make_shared<Trajectory>(std::move(traj_backward));
                
                EdgeInfo info_backward;
                info_backward.distance = shared_traj_backward->cost;
                info_backward.distance_original = shared_traj_backward->cost;
                info_backward.cached_trajectory = shared_traj_backward;
                info_backward.is_trajectory_computed = true;
                // info_backward.is_evaluated = false; // LLPT Lazy Flag
                info_backward.last_eval_epoch = 0;
                
                neighbor->forward_neighbors_[node] = info_backward;
                node->backward_neighbors_[neighbor] = info_backward;
            }
        }
    }

    node->neighbors_cached_ = true;
}


void KinodynamicLLPTStar::setStart(const Eigen::VectorXd& start) {
    problem_def_->setStart(start);
}

void KinodynamicLLPTStar::setGoal(const Eigen::VectorXd& goal) {
    problem_def_->setGoal(goal);
}

// void KinodynamicLLPTStar::injectTimePillarNodes(const Eigen::VectorXd& goal_state_val, int num_pillar_nodes) {
//     double max_time = upper_bounds_(statespace_->getDimension() - 1); 

//     for (int i = 1; i <= num_pillar_nodes; ++i) {
//         Eigen::VectorXd pillar_state = goal_state_val;

//         // Set velocities to ZERO so the robot can safely stop at the goal.
//         // Safety check to ensure we only apply this to states with velocity dimensions
//         if (statespace_->getDimension() == 5) {
//             pillar_state(2) = 0.0;
//             pillar_state(3) = 0.0;
//         }

//         // Distribute evenly across time
//         double t_val = (max_time / num_pillar_nodes) * i; 
//         pillar_state(statespace_->getDimension() - 1) = t_val;

//         auto state_ptr = statespace_->addState(pillar_state);
//         auto node = std::make_unique<DStarLiteNode>(state_ptr, nodes_.size());
//         node->setTimeToGoal(t_val);
//         node->rhs = 0;
//         node->g = 0;
//         time_pillar_indices_.insert(node->getIndex());
//         nodes_.push_back(std::move(node));
//     }
// }


void KinodynamicLLPTStar::injectTimePillarNodes(const Eigen::VectorXd& goal_state_val, int num_pillar_nodes) {
    if (num_pillar_nodes <= 0) return;

    auto start_state_val_ = problem_def_->getGoal();
    // 1. Calculate minimum required time (using only X and Y coordinates)
    double dist = (goal_state_val.head(2) - start_state_val_.head(2)).norm();
    double min_arrival_time = dist / statespace_->getMaxVelocity();

    // 2. Get the maximum time budget from the problem bounds
    int time_dim_index = statespace_->getDimension() - 1;
    double max_time = upper_bounds_(time_dim_index); 

    // If max_time is less than min_arrival_time, the goal is physically unreachable.
    if (max_time <= min_arrival_time) {
        std::cerr << "Time budget is too strict. Goal is physically unreachable at max velocity.\n";
        return; 
    }

    // 3. Define the reachable time window
    double time_window = max_time - min_arrival_time;
    std::cout<<"TIME WINDOW!"<<time_window<<"\n";
    // 4. Inject the Time Pillars
    for (int i = 1; i <= num_pillar_nodes; ++i) {
        // Copy the goal state. 
        // This naturally copies vx=0, vy=0 for Thruster if they are set in the configuration.
        Eigen::VectorXd pillar_state = goal_state_val;

        // // Distribute time evenly across the REACHABLE window
        // double t_val = min_arrival_time + (time_window / num_pillar_nodes) * i; 

        // 1. Calculate how long the journey takes (distributed between min_arrival and max_time)
        double travel_time = min_arrival_time + (time_window / num_pillar_nodes) * i; 
        
        // 2. Subtract travel_time from max_time because time decreases from the robot to the goal
        double t_val = max_time - travel_time; 


        pillar_state(time_dim_index) = t_val;

        auto state_ptr = statespace_->addState(pillar_state);
        auto node = std::make_unique<DStarLiteNode>(state_ptr, nodes_.size());
        
        node->setTimeToGoal(t_val);
        
        // Initialize D* Lite specific variables
        node->rhs = 0.0;
        node->g = 0.0;
        
        time_pillar_indices_.insert(node->getIndex());
        nodes_.push_back(std::move(node));
    }
}


void KinodynamicLLPTStar::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    std::cout << "------------------------------------------------------------\n";
    auto start_time = std::chrono::high_resolution_clock::now();
    visualization_ = visualization;
    
    num_samples_ = params.getParam<int>("num_of_samples");
    kd_dim_ = params.getParam<int>("kd_dim", 2);
    partial_update_ = params.getParam<bool>("partial_update");
    std::string kdtree_type = params.getParam<std::string>("kdtree_type");
    use_knn_ = params.getParam<bool>("use_knn", false);
    factor_ = params.getParam<double>("factor", 1.0);
    delta = params.getParam<double>("delta");
    is_geometric_mode_ = params.getParam<bool>("is_geometric_mode", false);
    num_pillar_nodes_ = params.getParam<int>("num_pillar_nodes", 50);
    if (is_geometric_mode_) num_pillar_nodes_ = 0;
    use_heuristic = params.getParam<bool>("heuristic", true);

    lower_bounds_ = problem_def_->getLowerBound();
    upper_bounds_ = problem_def_->getUpperBound();

    if (kdtree_type == "NanoFlann") {
        Eigen::VectorXd weights(kd_dim_);
        switch (kd_dim_) {
            case 2: weights << 1.0, 1.0; break;
            case 3: weights << 1.0, 1.0, 1.0; break;
            case 4: weights << 1.0, 1.0, 1.0, 1.0; break;
            case 5: weights << 1.0, 1.0, 1.0, 1.0, 1.0; break;
            default: throw std::runtime_error("Unsupported k-d tree dimension");
        }
        kdtree_ = std::make_shared<DynamicWeightedNanoFlann>(kd_dim_, weights);
    } else if (kdtree_type == "LieKDTree") {
        kdtree_ = std::make_unique<LieSplittingKDTree>(statespace_->getDimension(), statespace_);
    } else {
        throw std::runtime_error("PRM* D* Lite requires a KD-Tree.");
    }


    const Eigen::VectorXd& start_state_val = problem_def_->getGoal(); 
    const Eigen::VectorXd& goal_state_val = problem_def_->getStart();

    auto goal_state_ptr = statespace_->addState(goal_state_val);
    auto goal_node = std::make_unique<DStarLiteNode>(goal_state_ptr, 0);
    goal_node_ = goal_node.get(); 
    goal_node->setTimeToGoal(0);
    nodes_.push_back(std::move(goal_node));

    // auto start_state_ptr = statespace_->addState(start_state_val);
    // auto start_node = std::make_unique<DStarLiteNode>(start_state_ptr, 1);
    // start_node_ = start_node.get(); 
    // start_node->setTimeToGoal(std::numeric_limits<double>::infinity());
    // nodes_.push_back(std::move(start_node));

    time_pillar_indices_.insert(goal_node_->getIndex());
    if (!is_geometric_mode_) injectTimePillarNodes(goal_state_val, num_pillar_nodes_);

    // Initialize
    km_ = 0.0;
    open_queue_.clear(); // It removes the pillars so dont use it!
    goal_node_->rhs = 0.0;
    goal_node_->g = 0.0;



    Eigen::MatrixXd all_samples = statespace_->getSamplesCopy();
    Eigen::MatrixXd spatial_samples_only = all_samples.leftCols(kd_dim_).eval();
    // kdtree_->addPoints(spatial_samples_only);
    kdtree_->addPoints(all_samples);
    kdtree_->buildTree();

    int d = statespace_->getDimension(); // Since i use the kd dim as dimensions (always for now!) this is the same as d = kd_dim
    Eigen::VectorXd range = upper_bounds_ - lower_bounds_;
    // Same mu as FMTX/RRTX: product of per-dimension ranges (workspace volume).
    mu_ = 1.0;
    for (int i = 0; i < d; ++i) mu_ *= range(i);
    zetaD_ = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
    /*
        Mind that PRM* doenst have the 1/d exponent (Karaman paper) for the "2" 
        But in FMT* paper they mention the prm* rn can be lower (Remark 4.5). 
        So I use RRT*'s rn which is in between FMT* and PRM*. On top of this we use 
        a factor anyway so no worries!
    */
    gamma_ = std::pow(2, 1.0 / d) * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu_ / zetaD_, 1.0 / d); 
    double initial_calc_radius = factor_ * gamma_ * std::pow(std::log(std::max(2,static_cast<int>(nodes_.size()) - num_pillar_nodes_)) / std::max(2,static_cast<int>(nodes_.size()) - num_pillar_nodes_), 1.0 / d);
    connection_radius_ = std::min(delta, initial_calc_radius);
    std::cout << "Computed value of rn: " << connection_radius_ << std::endl;
    std::cout<<"factor: "<<factor_<<"\n";


    // std::cout << "Forcing neighbor caching for all " << nodes_.size() << " nodes..." << std::endl;
    // for (size_t i = 0; i < nodes_.size(); ++i) {
    //     near(i);
    // }


    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "------------------------------------------------------------\n";
}

Eigen::VectorXd KinodynamicLLPTStar::saturate(const Eigen::VectorXd& newPoint, const Eigen::VectorXd& closestPoint, double delta) {
    int dimension = newPoint.size();
    Eigen::VectorXd saturatedPoint = newPoint;

    switch (dimension) {
        case 2: { // State: (x, y)
            double dist = (newPoint - closestPoint).norm();
            if (dist > delta) {
                saturatedPoint = closestPoint + (newPoint - closestPoint) * (delta / dist);
            }
            break;
        }

        case 3: { // State: (x, y, time)
            double dist = (newPoint - closestPoint).norm();
            if (dist > delta) {
                saturatedPoint = closestPoint + (newPoint - closestPoint) * (delta / dist);
            }
            break;
        }

        case 4: { // State: (x, y, theta, time)
            constexpr int X = 0, Y = 1, THETA = 2, TIME = 3;

            // Distance is calculated on Euclidean components (x, y, time)
            Eigen::Vector3d p1(newPoint(X), newPoint(Y), newPoint(TIME));
            Eigen::Vector3d p2(closestPoint(X), closestPoint(Y), closestPoint(TIME));
            double dist = (p1 - p2).norm();

            if (dist > delta) {
                double scale = delta / dist;
                // Saturate Euclidean parts
                saturatedPoint(X) = closestPoint(X) + (newPoint(X) - closestPoint(X)) * scale;
                saturatedPoint(Y) = closestPoint(Y) + (newPoint(Y) - closestPoint(Y)) * scale;
                saturatedPoint(TIME) = closestPoint(TIME) + (newPoint(TIME) - closestPoint(TIME)) * scale;

                // Saturate angle along the shortest path
                double thetaDiff = normalizeAngle(newPoint(THETA) - closestPoint(THETA));
                saturatedPoint(THETA) = closestPoint(THETA) + thetaDiff * scale;
            }
            // Always normalize the final angle
            saturatedPoint(THETA) = normalizeAngle(saturatedPoint(THETA));
            break;
        }

        case 5: { // State: (x, y, vx, vy, time)
            // For this state space, all components are Euclidean.
            // We calculate distance and saturate across all 5 dimensions.
            double dist = (newPoint - closestPoint).norm();
            if (dist > delta) {
                saturatedPoint = closestPoint + (newPoint - closestPoint) * (delta / dist);
            }
            break;
        }

        default:
            // Handle unsupported dimensions
            throw std::runtime_error("Unsupported state dimension for saturate function: " + std::to_string(dimension));
    }

    return saturatedPoint;
}



bool KinodynamicLLPTStar::extendSearchGraph() {


    // 1. Sample
    // Eigen::VectorXd sample = Eigen::VectorXd::Random(kd_dim_);
    // sample = (lower_bounds_.head(kd_dim_).array() + 
    //          (upper_bounds_.head(kd_dim_) - lower_bounds_.head(kd_dim_)).array() * 
    //          ((sample.array() + 1.0) / 2.0)).matrix();
    Eigen::VectorXd sample = statespace_->sampleUnregistered(lower_bounds_, upper_bounds_);

    int dimension_ = sample.size();

    // if (!is_geometric_mode_ && dimension_ >= 3) {
    //     const double R = T_robot * statespace_->getMaxVelocity();
    //     // Degenerate footprint near goal: adding nodes in a collapsing start-disk
    //     // buys nothing and costs everything. Skip the batch.
    //     if (R < 2*connection_radius_) return true;   // tune threshold to taste
    // }

    // if (!is_geometric_mode_ && dimension_ >= 3) {
    //     Eigen::Vector2d root_position_ = problem_def_->getStart().head(2);
    //     const int t_idx = dimension_ - 1;
    //     auto dist = (root_position_ - sample.head(2)).norm();
    //     const double t_reach = dist / statespace_->getMaxVelocity(); // minTimeToReachNode (time-to-goal model, no S.start offset)
    //     const double t_samp  = sample[t_idx];

    //     const bool too_early = (t_samp < t_reach);
    //     const bool behind    = (t_samp > T_robot);
    //     if ((too_early || behind) && T_robot > t_reach) {
    //         // Remap the ALREADY-DRAWN time into [t_reach, t_robot]
    //         // u is uniform in [0,1)
    //         const double u = (t_samp - lower_bounds_[t_idx]) /
    //                         (upper_bounds_[t_idx] - lower_bounds_[t_idx]);
    //         auto before = sample[t_idx];
    //         sample[t_idx] = t_reach + u * (T_robot - t_reach);
    //         // std::cout<<"BEFORE: "<<before<<", AFTER: "<<sample[t_idx]<<"\n";
    //     }
    // }

    if (!is_geometric_mode_ && dimension_ >= 3) {
        const int t_idx = dimension_ - 1;
        // One-sided goal-reachability cone (shared StateSpace helper).
        if (!statespace_->remapTimeToGoalCone(sample, Eigen::Vector2d(problem_def_->getStart().head(2)),
                                              T_robot, lower_bounds_[t_idx], upper_bounds_[t_idx])) {
            return false; // do NOT increment successfully_added
        }
    }



    
    // 2. Nearest
    std::vector<size_t> nearest_indices = kdtree_->knnSearch(sample, 1);
    if (nearest_indices.empty()) return false;
    DStarLiteNode* v_nearest = nodes_[nearest_indices[0]].get();

    // 3. Steer to saturate
    Eigen::VectorXd new_state_val = saturate(sample, v_nearest->getStateValue(), delta);

    // 4. Eager node collision check
    if (!obs_checker_->isObstacleFree(new_state_val)) {
        return false;
    }

    // ---- Phase 1: check only forward edges, bail if none ----
    // double radius = factor_ * gamma_ * std::pow(std::log(nodes_.size() + 1) / (nodes_.size() + 1), 1.0 / kd_dim_);
    // radius = std::min(delta, radius);

    // Prevent log(1)=0 by enforcing a minimum effective graph size of 2 for the math
    int safe_N = std::max(2, static_cast<int>(nodes_.size()) - num_pillar_nodes_);
    double radius = factor_ * gamma_ * std::pow(std::log(safe_N) / safe_N, 1.0 / kd_dim_);
    radius = std::min(delta, radius);

    std::vector<size_t> neighbor_indices = kdtree_->radiusSearch(new_state_val.head(kd_dim_), radius);

    struct ForwardCandidate {
        size_t neighbor_idx;
        Trajectory traj;
    };
    std::vector<ForwardCandidate> forward_candidates;

    for (size_t idx : neighbor_indices) {
        DStarLiteNode* u = nodes_[idx].get();
        Trajectory traj_fwd = statespace_->steer(new_state_val, u->getStateValue());
        if (traj_fwd.is_valid && traj_fwd.cost <= radius + 1e-9) {
            forward_candidates.push_back({idx, std::move(traj_fwd)});
        }
    }

    // Per LLPT* Alg 6 (line 5): the node is added unconditionally once it is collision-free.
    // We do NOT discard a sample for lacking a forward (outgoing) edge — it starts dormant
    // (rhs=inf) and the lifelong search connects it once densification provides an outgoing
    // edge (text after Alg 6: "lmc/g initialized as inf; inserted into Q only if it can yield
    // a solution via a neighbor").

    // ---- Register the node (RRG densification) ----
    auto state_ptr = statespace_->addState(new_state_val);
    auto new_node_ptr = std::make_unique<DStarLiteNode>(state_ptr, nodes_.size());
    nodes_.push_back(std::move(new_node_ptr));
    DStarLiteNode* v_new = nodes_.back().get();

    v_new->g = std::numeric_limits<double>::infinity();
    v_new->rhs = std::numeric_limits<double>::infinity();
    if (!is_geometric_mode_) {
        v_new->setTimeToGoal(new_state_val.tail<1>()[0]);
    } else {
        v_new->setTimeToGoal(0.0);
    }

    // KD‑tree
    Eigen::MatrixXd new_point(1, kd_dim_);
    new_point.row(0) = v_new->getStateValue().head(kd_dim_);
    kdtree_->addPoints(new_point);
    connection_radius_ = radius; // update for later use

    // ---- Phase 2: insert forward edges and call UpdateNode(vnew, u) ----
    for (auto& cand : forward_candidates) {
        DStarLiteNode* u = nodes_[cand.neighbor_idx].get();
        auto shared_fwd = std::make_shared<Trajectory>(std::move(cand.traj));

        EdgeInfo info_fwd;
        info_fwd.distance = shared_fwd->cost;
        info_fwd.distance_original = shared_fwd->cost;
        info_fwd.cached_trajectory = shared_fwd;
        info_fwd.is_trajectory_computed = true;
        // info_fwd.is_evaluated = false;
        info_fwd.last_eval_epoch = 0;

        v_new->forward_neighbors_[u] = info_fwd;
        u->backward_neighbors_[v_new] = info_fwd;

        // The ONLY UpdateNode call (as per paper)
        updateNode(v_new, u);
    }

    // ---- Phase 3: backward edges (no UpdateNode on neighbours) ----
    if (is_geometric_mode_) {
        // Backward edges are identical copies of forward edges
        for (auto& cand : forward_candidates) {
            DStarLiteNode* u = nodes_[cand.neighbor_idx].get();
            EdgeInfo info_bwd = v_new->forward_neighbors_.at(u); // copies shared_ptr
            u->forward_neighbors_[v_new] = info_bwd;
            v_new->backward_neighbors_[u] = info_bwd;
            // No updateNode(u, v_new) call
        }
    } else {
        // Kinodynamic: need to steer backward explicitly for each neighbour
        for (size_t idx : neighbor_indices) {
            DStarLiteNode* u = nodes_[idx].get();
            Trajectory traj_bwd = statespace_->steer(u->getStateValue(), new_state_val);
            if (traj_bwd.is_valid && traj_bwd.cost <= radius + 1e-9) {
                auto shared_bwd = std::make_shared<Trajectory>(std::move(traj_bwd));
                EdgeInfo info_bwd;
                info_bwd.distance = shared_bwd->cost;
                info_bwd.distance_original = shared_bwd->cost;
                info_bwd.cached_trajectory = shared_bwd;
                info_bwd.is_trajectory_computed = true;
                // info_bwd.is_evaluated = false;
                info_bwd.last_eval_epoch = 0;

                u->forward_neighbors_[v_new] = info_bwd;
                v_new->backward_neighbors_[u] = info_bwd;
                // Again, no updateNode(u, v_new)
            }
        }
    }

    return true; // SUCCESS!
}

// void KinodynamicLLPTStar::updateNode(DStarLiteNode* v, DStarLiteNode* u) {
//     auto it = v->forward_neighbors_.find(u);
//     if (it == v->forward_neighbors_.end()) return;

//     // Get lazy weight
//     double edge_cost = getLazyWeight(it->second);
//     if (edge_cost == std::numeric_limits<double>::infinity() ||
//         u->rhs == std::numeric_limits<double>::infinity() ||
//         std::isinf(it->second.distance))   // ADDED: also reject if stored distance is inf
//         return;

//     double candidate_lmc = edge_cost + u->rhs;
//     if (v->rhs > candidate_lmc + 1e-9) {
//         v->setBestParentLLPT(u, it->second.cached_trajectory);
//         v->rhs = candidate_lmc;
//         updateVertex(v);
//     }
// }

void KinodynamicLLPTStar::updateNode(DStarLiteNode* v, DStarLiteNode* u) {
    auto it = v->forward_neighbors_.find(u);
    if (it == v->forward_neighbors_.end()) return;

    const double edge_cost = getLazyWeight(it->second);
    if (std::isinf(edge_cost) || std::isinf(u->rhs)) {
        return;
    }

    const double candidate_rhs = edge_cost + u->rhs;
    if (v->rhs > candidate_rhs + 1e-9) {
        v->setBestParentLLPT(u, it->second.cached_trajectory);
        v->rhs = candidate_rhs;
        updateVertex(v);
    }
}

void KinodynamicLLPTStar::plan() {
    if (!goal_node_) return;

    int successfully_added = 0;
    // 2. LLPT Graph Densification (Algorithm 6)
    // for (int i = 0; i < num_samples_; ++i) {

    // Raw-attempt cap: bounds the loop when the footprint is degenerate
    // (near goal, T_robot <= t_reach rejects almost every draw).
    // Tune the multiplier; large enough that it never bites in normal regions.
    const int max_attempts = num_samples_ * 20;
    int attempts = 0;

    while (successfully_added < num_samples_  && attempts < max_attempts){
        ++attempts;   // every iteration consumes an attempt, including the ones that `continue`

        // Only increment our successfully_added counter if the node was actually kept
        if (extendSearchGraph()) {
            successfully_added++;
        }
    }

    /*
        usage of global_eval_epoch counter here inadvertently gave the planner a "patience" mechanism.
        Because the cache kept clearing, setRobotState refused to acknowledge the infinity trap. 
    */
    // // O(1) global reset
    // global_eval_epoch_++;

    // // 3. If extendSearchGraph added new shortcuts to the queue, resolve them!
    if (start_node_) {
        resolvePathLazy();
    }


    // std::cout<<"COL: "<<collision_checked_<<"\n";



    // // ========== DEBUG: Full path safety from anchor to goal ==========
    // if (start_node_ && start_node_->rhs != std::numeric_limits<double>::infinity()) {

    //     // 1. Check the bridge from robot to anchor
    //     bool bridge_safe_debug = true;
    //     if (current_bridge_trajectory_.is_valid && !current_bridge_trajectory_.path_points.empty()) {
    //         for (const auto& ob : obs_checker_->getObstacles()) {
    //             if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(current_bridge_trajectory_, ob)) {
    //                 bridge_safe_debug = false;
    //                 std::cout << "[DEBUG] Bridge from robot to anchor is UNSAFE against " << ob.name << std::endl;
    //                 break;
    //             }
    //         }
    //         if (bridge_safe_debug)
    //             std::cout << "[DEBUG] Bridge from robot to anchor is SAFE" << std::endl;
    //     } else {
    //         std::cout << "[DEBUG] No valid bridge trajectory (robot already at anchor?)" << std::endl;
    //     }

    //     // 2. Walk the tree from anchor to goal
    //     DStarLiteNode* node = start_node_;
    //     int edge_count = 0;
    //     while (node && node != goal_node_) {
    //         DStarLiteNode* parent = node->getParent();
    //         if (!parent) {
    //             std::cout << "[DEBUG] Path broken at node " << node->getIndex()
    //                     << " (no parent)" << std::endl;
    //             break;
    //         }

    //         // Retrieve the edge trajectory (forward_neighbors_ from node to parent)
    //         auto it = node->forward_neighbors_.find(parent);
    //         if (it == node->forward_neighbors_.end()) {
    //             std::cout << "[DEBUG] Edge from " << node->getIndex()
    //                     << " to parent " << parent->getIndex()
    //                     << " not found in forward_neighbors_" << std::endl;
    //             break;
    //         }

    //         const EdgeInfo& edge = it->second;
    //         if (!edge.cached_trajectory || edge.cached_trajectory->path_points.empty()) {
    //             std::cout << "[DEBUG] Edge from " << node->getIndex()
    //                     << " to " << parent->getIndex()
    //                     << " has no cached trajectory" << std::endl;
    //             break;
    //         }

    //         bool edge_safe = true;
    //         for (const auto& ob : obs_checker_->getObstacles()) {
    //             if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*edge.cached_trajectory, ob)) {
    //                 edge_safe = false;
    //                 std::cout << "[DEBUG] Edge " << node->getIndex()
    //                         << " -> " << parent->getIndex()
    //                         << " UNSAFE against " << ob.name << std::endl;
    //                 break;
    //             }
    //         }
    //         if (edge_safe)
    //             std::cout << "[DEBUG] Edge " << node->getIndex()
    //                     << " -> " << parent->getIndex()
    //                     << " SAFE (cost " << edge.distance << ")" << std::endl;

    //         node = parent;
    //         edge_count++;
    //     }

    //     if (node == goal_node_)
    //         std::cout << "[DEBUG] Reached goal node. Total tree edges checked: " << edge_count << std::endl;
    // }
    // else {
    //     std::cout << "[DEBUG] No valid anchor (start_node_ null or rhs inf)" << std::endl;
    // }





}


double KinodynamicLLPTStar::heuristic(DStarLiteNode* a, DStarLiteNode* b) {
    if (use_heuristic){
        if (!a || !b) return 0.0;

        const Eigen::VectorXd& val_a = a->getStateValue();
        const Eigen::VectorXd& val_b = b->getStateValue();
        
        // Spatial distance (Common to all state spaces)
        double dx = val_a(0) - val_b(0);
        double dy = val_a(1) - val_b(1);
        double spatial_dist_sq = dx * dx + dy * dy;

        double h_val = 0.0;

        // State Space Switching based on KD-Tree Dimension
        if (kd_dim_ == 2) {
            h_val = std::sqrt(spatial_dist_sq);
        } 
        else if (kd_dim_ == 3 || kd_dim_ == 4) {
            // R2T and Dubins: dt is DROPPED to preserve Triangle Inequality 
            // with the zero-cost Time Pillar edges!
            h_val = std::sqrt(spatial_dist_sq);
        } 
        else if (kd_dim_ == 5) {
            // Thruster Space: dt is DROPPED!
            double dvx = val_a(2) - val_b(2);
            double dvy = val_a(3) - val_b(3);
            double dv_dist_sq = dvx * dvx + dvy * dvy;
            
            double c_x = 1.0;
            double c_v = 1.0;
            h_val = std::sqrt((c_x * c_x * spatial_dist_sq) + (c_v * c_v * dv_dist_sq));
        }
        
        // Ensure the heuristic never drops below 0
        return std::max(0.0, h_val);
    }
    else
        return 0.0;
}

DStarLiteKey KinodynamicLLPTStar::calculateKey(DStarLiteNode* u) {
    double min_val = std::min(u->g, u->rhs);
    double h_val = (start_node_) ? heuristic(u, start_node_) : 0.0;
    return { min_val + h_val + km_, min_val };
}


void KinodynamicLLPTStar::initialize(DStarLiteNode* start, DStarLiteNode* goal) {
    km_ = 0.0;
    open_queue_.clear(); // It removes the pillars so dont use it!
    start_node_ = start;
    goal_node_ = goal;
    goal->rhs = 0.0;

    open_queue_.add(goal, calculateKey(goal));
    for (int idx : time_pillar_indices_) {
        DStarLiteNode* node = nodes_[idx].get();
        node->rhs = 0.0;
        open_queue_.add(node, calculateKey(node));
    }

}



void KinodynamicLLPTStar::updateVertex(DStarLiteNode* u) {
    bool is_consistent = (u->g == u->rhs);


    if (!is_consistent && u->in_queue_) {
        // Update priority
        open_queue_.update(u, calculateKey(u));
    } 
    else if (!is_consistent && !u->in_queue_) {
        // Insert into queue
        open_queue_.add(u, calculateKey(u));
    } 
    else if (is_consistent && u->in_queue_) {
        // Remove from queue
        open_queue_.remove(u);
    }
}



// void KinodynamicLLPTStar::computeShortestPath() {
//     if (!start_node_ || !goal_node_) return;

//     while (!open_queue_.empty()) {
//         DStarLiteKey target_key = calculateKey(start_node_);
//         bool start_is_consistent = (std::abs(start_node_->g - start_node_->rhs) <= 1e-9);
//         bool start_is_inf = std::isinf(start_node_->rhs);

//         if (partial_update_) {
//             if (start_is_consistent && !start_is_inf && !start_node_->in_queue_ && !(open_queue_.topKey() < target_key)) {
//                 break;
//             }
//         }

//         DStarLiteNode* v = open_queue_.top();
//         DStarLiteKey k_old = open_queue_.topKey();
//         if (!v) break;

//         DStarLiteKey k_new = calculateKey(v);

//         if (k_old < k_new) {
//             open_queue_.update(v, k_new);
//         } else {
//             open_queue_.remove(v);

//             if (v->rhs > v->g + 1e-9 || std::isinf(v->rhs)) {
//                 for (auto& [u, edge_info] : v->forward_neighbors_) {
//                     bool in_VT = (u->getParent() != nullptr || u == goal_node_);
//                     // USE GET LAZY WEIGHT HERE
//                     // if (in_VT && !std::isinf(getLazyWeight(edge_info))) {
//                     if (in_VT && !std::isinf(getLazyWeight(edge_info)) && !std::isinf(edge_info.distance)){

//                         double new_cost = getLazyWeight(edge_info) + u->rhs;
//                         if (v->rhs > new_cost + 1e-9) {
//                             v->setBestParentLLPT(u, edge_info.cached_trajectory);
//                             v->rhs = new_cost;
//                         }
//                     }
//                 }
//             }

//             if (!std::isinf(v->rhs)) {
//                 for (auto& [u, edge_info] : v->backward_neighbors_) {
//                     if (u == v->getParent()) continue; 
//                     // USE GET LAZY WEIGHT HERE
//                     // if (std::isinf(getLazyWeight(edge_info))) continue;
//                     if (std::isinf(getLazyWeight(edge_info)) || std::isinf(edge_info.distance)) continue;


//                     double new_cost = getLazyWeight(edge_info) + v->rhs;
//                     if (u->rhs > new_cost + 1e-9) {
//                         u->setBestParentLLPT(v, edge_info.cached_trajectory);
//                         u->rhs = new_cost;
//                         updateVertex(u);
//                     }
//                 }
//             }
//             v->g = v->rhs;
//         }
//     }
// }


void KinodynamicLLPTStar::computeShortestPath() {
    if (!start_node_ || !goal_node_) return;

    while (!open_queue_.empty()) {
        DStarLiteKey target_key = calculateKey(start_node_);
        bool start_is_consistent = (std::abs(start_node_->g - start_node_->rhs) <= 1e-9);
        bool start_is_inf = std::isinf(start_node_->rhs);

        if (partial_update_) {
            if (start_is_consistent && !start_is_inf &&
                !start_node_->in_queue_ &&
                !(open_queue_.topKey() < target_key)) {
                break;
            }
        }

        DStarLiteNode* v = open_queue_.top();
        DStarLiteKey k_old = open_queue_.topKey();
        if (!v) break;

        DStarLiteKey k_new = calculateKey(v);

        if (k_old < k_new) {
            open_queue_.update(v, k_new);
            continue;
        }

        open_queue_.remove(v);

        if (v->rhs > v->g + 1e-9 || std::isinf(v->rhs)) {
            for (auto& [u, edge_info] : v->forward_neighbors_) {
                bool in_VT = (u->getParent() != nullptr || u == goal_node_);
                if (!in_VT) continue;

                const double w_bar = getLazyWeight(edge_info);
                if (std::isinf(w_bar) || std::isinf(u->rhs)) continue;

                const double new_cost = w_bar + u->rhs;
                if (v->rhs > new_cost + 1e-9) {
                    v->setBestParentLLPT(u, edge_info.cached_trajectory);
                    v->rhs = new_cost;
                }
            }
        }

        if (!std::isinf(v->rhs)) {
            for (auto& [u, edge_info] : v->backward_neighbors_) {
                if (u == v->getParent()) continue;

                const double w_bar = getLazyWeight(edge_info);
                if (std::isinf(w_bar)) continue;

                const double new_cost = w_bar + v->rhs;
                if (u->rhs > new_cost + 1e-9) {
                    u->setBestParentLLPT(v, edge_info.cached_trajectory);
                    u->rhs = new_cost;
                    updateVertex(u);
                }
            }
        }

        v->g = v->rhs;
    }
}

void KinodynamicLLPTStar::updateObstacles(const ObstacleVector& turned_obstacles) {
    if (turned_obstacles.empty()) return;

    global_eval_epoch_++;

    double T_robot = 0.0;
    if (!is_geometric_mode_ && robot_continuous_state_.size() > 0) {
        T_robot = robot_continuous_state_(robot_continuous_state_.size() - 1);
    }

    for (const auto& incoming_ob : turned_obstacles) {
        Obstacle& stored_ob = previous_obstacles_[incoming_ob.name];

        // FOR REMOVED STATIC OBS
        // PURGE: visible obstacle the robot reached. Free edges, erase, never re-add. --> Remove static obstalce!
        if (incoming_ob.is_removed) {
            previous_obstacles_.erase(incoming_ob.name);
            continue;
        }
        //////////////////

        stored_ob = incoming_ob; 
        // stored_ob.predicted_path = obs_checker_->generatePrediction(stored_ob, T_robot);
    }


    // // Heal the specific broken edges
    // Heal edges that were previously found collided
   // Heal edges that were previously marked as collided
    for (auto& [u, v] : collided_edges_) {
        auto it_fwd = u->forward_neighbors_.find(v);
        if (it_fwd != u->forward_neighbors_.end()) {
            it_fwd->second.distance = it_fwd->second.distance_original;
            it_fwd->second.last_eval_epoch = 0;
        }
        // Backward edge (if exists)
        auto it_rev = v->backward_neighbors_.find(u);
        if (it_rev != v->backward_neighbors_.end()) {
            it_rev->second.distance = it_rev->second.distance_original;
            it_rev->second.last_eval_epoch = 0;
        }
        // Update node to potentially rewire
        updateNode(u, v);
        if (is_geometric_mode_) updateNode(v, u);
    }



    collided_edges_.clear(); 




}



void KinodynamicLLPTStar::propagateCostToLeave(DStarLiteNode* v) {
    if (!v) return;

    std::queue<DStarLiteNode*> to_orphan;
    to_orphan.push(v);
    
    while (!to_orphan.empty()) {
        DStarLiteNode* current = to_orphan.front();
        to_orphan.pop();
        
        // 1. Disconnect
        current->setBestParentLLPT(nullptr, nullptr); 
        
        // 2. Set infinite cost
        current->rhs = std::numeric_limits<double>::infinity();
        updateVertex(current);
        
        // 3. Queue dependents
        for (auto& [pred, edge_info] : current->backward_neighbors_) {
            if (pred->getParent() == current) {
                to_orphan.push(pred);
            }
        }
    }
}

// // RECURSIVE! --> TOO HARD ON STACK!
// void KinodynamicLLPTStar::propagateCostToLeave(DStarLiteNode* v) {
//     // 1. Disconnect from broken parent
//     v->setBestParentLLPT(nullptr, nullptr); 
    
//     // 2. Set LMC (rhs) to infinity and push to queue so it finds a new path later
//     v->rhs = std::numeric_limits<double>::infinity();
//     updateVertex(v);
    
//     // 3. Instantly cascade to all descendants that rely on 'v'
//     for (auto& [pred, edge_info] : v->backward_neighbors_) {
//         if (pred->getParent() == v) {
//             propagateCostToLeave(pred);
//         }
//     }
// }




void KinodynamicLLPTStar::resolvePathLazy() {
    if (!start_node_) {
        LLPT_WARN("[LLPT] start_node is null, cannot resolve path.");
        return;
    }


    bool path_is_collision_free = false;
    int max_inner_iterations = 500; 
    int iterations = 0;

    // // --- ADD THIS LOG ---
    // DStarLiteNode* previous_best_parent = start_node_->getParent();
    // // --------------------


    while (!path_is_collision_free && iterations++ < max_inner_iterations) {
    // while (!path_is_collision_free ) {
        
        computeShortestPath(); 

        // // --- ADD THIS LOG ---
        // if (start_node_->getParent() != previous_best_parent) {
        //     LLPT_WARN(">>> PATH SHORTCUT FOUND! Start node parent changed from " 
        //               << previous_best_parent << " to " << start_node_->getParent());
        //     previous_best_parent = start_node_->getParent(); // Update tracking
        // }
        // // --------------------

        if (start_node_->rhs == std::numeric_limits<double>::infinity()) {
            LLPT_WARN("[LLPT] No solution exists in the current graph!");
            break; 
        }

        std::vector<DStarLiteNode*> collided_nodes = evaluateEdge(eval_batch_size_);

        if (collided_nodes.empty() && isPathFullyEvaluated()) {
            path_is_collision_free = true;
        }
        else {
            // ALGORITHM 3: You MUST call this! Do not try to recalculate min_rhs locally.
            // This manually cascades the 'infinity' wave down the tree to avoid Zombies.
            for (auto* v : collided_nodes) {
                propagateCostToLeave(v); 
            }
        }
    }

}

std::vector<DStarLiteNode*> KinodynamicLLPTStar::evaluateEdge(int max_evals) {
    std::vector<DStarLiteNode*> collided_nodes;
    DStarLiteNode* current = start_node_;
    int evals = 0;
    
    while (current != goal_node_ && current != nullptr) {
        DStarLiteNode* parent = current->getParent(); 
        if (!parent) break;
        
        auto it = current->forward_neighbors_.find(parent);
        if (it == current->forward_neighbors_.end()) break;   
        auto& edge = it->second;
        
        // If it hit a static wall in the past, DO NOT evaluate it again!
        if (edge.permanently_blocked) {
            collided_nodes.push_back(current);
            break;
        }

        if (edge.last_eval_epoch != global_eval_epoch_) {

            // // --- ADD THIS LOG ---
            // LLPT_INFO(">>> EVALUATING EDGE! Edge Epoch: " << edge.last_eval_epoch 
            //           << " | Global Epoch: " << global_eval_epoch_ 
            //           << " | Current Node: " << current->getIndex()
            //           << " | Parent: " << parent->getIndex());
            // // --------------------


            if (evals >= max_evals) {
                break;
            }

            bool safe = true;
            bool hit_static_wall = false; // Track what we hit

            for (const auto& ob : obs_checker_->getObstacles()) {
                last_replan_metrics_.obstacle_checks++;
                collision_checked_++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*edge.cached_trajectory, ob)) {
                    safe = false;
                    if (!ob.is_dynamic) {
                        hit_static_wall = true; // It's a static wall!
                    }
                    break;
                }
            }
            
            edge.last_eval_epoch = global_eval_epoch_; 

            // Synchronize the backward edge copy
            auto bwd_it = parent->backward_neighbors_.find(current);
            if (bwd_it != parent->backward_neighbors_.end()) {
                bwd_it->second.last_eval_epoch = global_eval_epoch_;
            }
            // ==========================================

            
            if (!safe) {
                if (hit_static_wall) {
                    // PERMANENT BLOCK: Set both to infinity forever
                    edge.distance = std::numeric_limits<double>::infinity();
                    edge.distance_original = std::numeric_limits<double>::infinity();
                    edge.permanently_blocked = true;
                    
                    if (parent->backward_neighbors_.count(current)) {
                        parent->backward_neighbors_[current].distance = std::numeric_limits<double>::infinity();
                        parent->backward_neighbors_[current].distance_original = std::numeric_limits<double>::infinity();
                        parent->backward_neighbors_[current].permanently_blocked = true;
                    }
                } else {
                    // DYNAMIC BLOCK: Only set current distance to inf, keep original for healing
                    edge.distance = std::numeric_limits<double>::infinity();
                    if (parent->backward_neighbors_.count(current)) {
                        parent->backward_neighbors_[current].distance = std::numeric_limits<double>::infinity();
                    }
                    collided_edges_.push_back({current, parent}); 
                }

                collided_nodes.push_back(current);
                break; 
            }
            evals++;
        }
        current = parent;
    }
    return collided_nodes;
}

// std::vector<DStarLiteNode*> KinodynamicLLPTStar::evaluateEdge(int max_evals) {
//     std::vector<DStarLiteNode*> collided_nodes;
//     DStarLiteNode* current = start_node_;
//     int evals = 0;
    
//     while (current != goal_node_ && current != nullptr) {
//         DStarLiteNode* parent = current->getParent(); 
//         if (!parent) break;
        


//         auto it = current->forward_neighbors_.find(parent);
//         if (it == current->forward_neighbors_.end()) break;   // safety – should never happen
//         auto& edge = it->second;
//         // auto& edge = current->forward_neighbors_[parent];
        
//         // Epoch Check! Only evaluate if not checked this frame
//         if (edge.last_eval_epoch != global_eval_epoch_) {
            
//             // If we hit our batch limit, STOP evaluating, but return a dummy node 
//             // so resolvePathLazy knows we didn't finish checking the whole path!
//             if (evals >= max_evals) {
//                 // collided_nodes.push_back(current); 
//                 break;
//             }

//             last_replan_metrics_.obstacle_checks++;
//             bool safe = true;
//             for (const auto& ob : obs_checker_->getObstacles()) {
//                 if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*edge.cached_trajectory, ob)) {
//                     safe = false;
//                     break;
//                 }
//             }
            
//             edge.last_eval_epoch = global_eval_epoch_; // Mark checked!
            
//             if (!safe) {
//                 edge.distance = std::numeric_limits<double>::infinity();
//                 if (parent->backward_neighbors_.count(current)) {
//                     parent->backward_neighbors_[current].distance = std::numeric_limits<double>::infinity();
//                 }
//                 collided_nodes.push_back(current);
                
//                 // TRACK ONLY THE BROKEN ONES
//                 collided_edges_.push_back({current, parent}); 
//                 break; // Path is broken, stop checking further!
//             }
//             evals++;
//         }
//         current = parent;
//     }
//     return collided_nodes;
// }


// void KinodynamicLLPTStar::setRobotState(const Eigen::VectorXd& robot_state) {
//     robot_continuous_state_ = robot_state;

//     double robot_time_to_go = 0.0;
//     if (!is_geometric_mode_ && robot_continuous_state_.size() > 0) {
//         robot_time_to_go = robot_continuous_state_(robot_continuous_state_.size() - 1);
//     }

//     Eigen::VectorXd query_point = Eigen::VectorXd::Zero(kd_dim_);
//     if (robot_continuous_state_.size() >= 2) {
//         query_point(0) = robot_continuous_state_(0);
//         query_point(1) = robot_continuous_state_(1);
//     }
//     if (kd_dim_ == 3) {
//         query_point(2) = robot_time_to_go;
//     } else if (kd_dim_ == 4) {
//         query_point(2) = robot_continuous_state_(2); 
//         query_point(3) = robot_time_to_go;
//     } else if (kd_dim_ == 5) {
//         query_point = robot_continuous_state_; 
//     }

//     const double hysteresis_factor = 0.98;
//     double cost_of_current_anchor = std::numeric_limits<double>::infinity();
    
//     Trajectory bridge;
//     bool safe = true;
//     if (start_node_ && start_node_->rhs != std::numeric_limits<double>::infinity()) {
//         bridge = statespace_->steer(robot_continuous_state_, start_node_->getStateValue());
//         if (bridge.is_valid) {
//             const auto& obstacles = obs_checker_->getObstacles();
//             for (const auto& ob : obstacles) {
//                 last_replan_metrics_.obstacle_checks++;
//                 if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(bridge, ob)) {
//                     safe = false;
//                     break;
//                 }
//             }

//             if (safe) {
//                 cost_of_current_anchor = bridge.cost + start_node_->rhs;
//             }
//         }
//     }

//     // Tracks the TRUE best connected node (verified by resolvePathLazy)
//     DStarLiteNode* best_connected_node = nullptr;
//     Trajectory best_connected_bridge;
//     double best_connected_cost = std::numeric_limits<double>::infinity();
    
//     // Tracks the safest physical node to reach, even if it is currently unexplored (g = infinity)
//     DStarLiteNode* best_fallback_node = nullptr;
//     Trajectory best_fallback_bridge;
//     double best_fallback_cost = std::numeric_limits<double>::infinity();
    
//     double current_search_radius = connection_radius_;
//     const int max_attempts = 5;
//     const double radius_multiplier = 2.0;
//     std::unordered_set<size_t> tested_indices;
    
//     // Structure to hold candidate nodes for sorting
//     struct Candidate {
//         DStarLiteNode* node;
//         Trajectory bridge;
//         double optimistic_cost;
//     };

//     double total_repair_time = 0.0; // Accumulate time spent in resolvePathLazy

//     for (int attempt = 1; attempt <= max_attempts; ++attempt) {
//         std::vector<size_t> candidate_indices = kdtree_->radiusSearch(query_point, current_search_radius);
//         std::vector<Candidate> current_radius_candidates;

//         for (size_t idx : candidate_indices) {
//             if (!tested_indices.insert(idx).second) continue;
//             DStarLiteNode* candidate = nodes_[idx].get();

//             // Check Steering & Collision FIRST
//             Trajectory temp_bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());
//             if (!temp_bridge.is_valid) continue;

//             bool bridge_safe = true;
//             for (const auto& ob : obs_checker_->getObstacles()) {
//                 last_replan_metrics_.obstacle_checks++;
//                 if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(temp_bridge, ob)) {
//                     bridge_safe = false;
//                     break; 
//                 }
//             }
//             if (!bridge_safe) continue;

// #if USE_RECOVERY
//             if (temp_bridge.cost < best_fallback_cost) {
//                 best_fallback_cost = temp_bridge.cost;
//                 best_fallback_node = candidate;
//                 best_fallback_bridge = temp_bridge;
//             }
// #endif

//             // Only attach to nodes that have an optimistic path to goal
//             if (candidate->rhs != std::numeric_limits<double>::infinity()) {
//                 double optimistic_total = temp_bridge.cost + candidate->rhs;
//                 current_radius_candidates.push_back({candidate, temp_bridge, optimistic_total});
//             }
//         }

//         // Sort candidates by their optimistic heuristic cost
//         std::sort(current_radius_candidates.begin(), current_radius_candidates.end(),
//                   [](const Candidate& a, const Candidate& b) {
//                       return a.optimistic_cost < b.optimistic_cost;
//                   });

//         // Test candidates one by one using lazy evaluation
//         // Keep track of the last anchor we updated km_ against
//         // 1. Store the TRUE start node before the testing loop begins
//         DStarLiteNode* original_start_node = start_node_;
//         double original_km = km_; // Store the original global km_
        
//         // Test candidates one by one using lazy evaluation
//         for (const auto& cand : current_radius_candidates) {
            
//             // 2. Compute the exact km_ for THIS specific candidate from the original anchor
//             double candidate_km_shift = 0.0;
//             if (original_start_node && original_start_node != cand.node) {
//                 candidate_km_shift = heuristic(original_start_node, cand.node);
//             }
            
//             // Set the global km_ strictly for this candidate
//             km_ = original_km + candidate_km_shift;
            
//             start_node_ = cand.node;

//             // 3. Evaluate the path
//             auto t1 = std::chrono::steady_clock::now();
//             resolvePathLazy(); 
//             auto t2 = std::chrono::steady_clock::now();
//             total_repair_time += std::chrono::duration<double, std::milli>(t2 - t1).count();

//             // 4. Check if the anchor survived
//             if (start_node_->rhs != std::numeric_limits<double>::infinity()) {
//                 // SUCCESS! 
//                 best_connected_node = start_node_;
//                 best_connected_bridge = cand.bridge;
//                 best_connected_cost = cand.bridge.cost + start_node_->rhs; 
                
//                 // We break here. The global km_ and the priority queue are now 
//                 // perfectly aligned with this successful candidate.
//                 break; 
//             } else {
//                 // FAILURE!
//                 // We do NOT rollback the graph (rhs, g, parent pointers). 
//                 // The physical obstacles discovered during resolvePathLazy() are real.
//                 // We simply loop to the next candidate, which will recalculate its own km_ 
//                 // from the original_start_node.
//             }
//         }
//         if (!best_connected_node) {
//             km_ = original_km;   // no anchor change → keep original km_
//             start_node_ = original_start_node;   // <-- add this

//         }

//         // Only break early if we found a CONNECTED and VERIFIED node
//         if (best_connected_node) break;
//         current_search_radius *= radius_multiplier;
//     }

//     last_anchor_repair_ms_ = total_repair_time;
//     // if (total_repair_time > 0.0) {
//     //     RCLCPP_INFO(rclcpp::get_logger("LLPT"), "Anchor repair search (lazy loops) took: %.6f ms", total_repair_time);
//     // }

//     // ---------- Diagnostic variables ----------
//     bool cached_bridge_safe = false;                
//     std::string first_unsafe_obstacle_name_;        

//     auto trap_reason = [&]() -> std::string {
//         std::ostringstream oss;
//         oss << "TRAP DIAGNOSTIC: ";
//         if (best_connected_node) {
//             oss << "candidate exists (cost " << best_connected_cost
//                 << ") but not better than current (" << cost_of_current_anchor
//                 << ") or hysteresis (factor " << hysteresis_factor << "). ";
//         } else {
//             oss << "no candidate with safe bridge + finite rhs found within "
//                 << (current_search_radius / radius_multiplier) << " m radius. "
//                 << "Searched " << tested_indices.size() << " nodes. ";
//         }
//         if (start_node_) {
//             oss << "Current anchor rhs=" << start_node_->rhs << ", ";
//         } else {
//             oss << "No current anchor. ";
//         }
//         oss << "Fresh steer safe=" << safe << ", bridge valid=" << bridge.is_valid;
//         if (!bridge.is_valid && start_node_) {
//             oss << " (steer failed from " << robot_continuous_state_.head<2>().transpose()
//                 << " to " << start_node_->getStateValue().head<2>().transpose() << ")";
//         }
//         if (current_bridge_trajectory_.is_valid) {
//             oss << ", cached bridge exists, re-verified="
//                 << (cached_bridge_safe ? "safe" : "unsafe");
//         } else {
//             oss << ", no cached bridge";
//         }
//         return oss.str();
//     };

//     // DECISION LOGIC
//     // 1) Better connected anchor
//     if (best_connected_node &&
//         best_connected_cost < cost_of_current_anchor * hysteresis_factor) {
            
//         start_node_ = best_connected_node;
//         bridge_cost_ = best_connected_bridge.cost;
//         last_replan_metrics_.path_cost = best_connected_cost;
//         current_bridge_trajectory_ = best_connected_bridge;
        
//         // NOTE: We no longer call resolvePathLazy() here, because we already called 
//         // it inside the candidate loop to verify this exact node!
//     } 
//     // 2) Keep current anchor with fresh bridge
//     else if (safe && start_node_ &&
//              cost_of_current_anchor != std::numeric_limits<double>::infinity() &&
//              bridge.is_valid) {
//         bridge_cost_ = bridge.cost;
//         last_replan_metrics_.path_cost = cost_of_current_anchor;
//         current_bridge_trajectory_ = bridge;
//     } 
//     // 3) Recovery: go to nearest safe tree node
// #if USE_RECOVERY
//     else if (best_fallback_node) {
//         bridge_cost_ = best_fallback_cost;
//         last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
//         current_bridge_trajectory_ = best_fallback_bridge;
//         LLPT_WARN("[Set Robot State] USE_RECOVERY: Falling back to nearest safe tree node.");
//     }
// #endif
//     // 4) Reuse cached bridge (with time re-stamping and re-verification)
//     else if (start_node_ &&
//              start_node_->rhs != std::numeric_limits<double>::infinity() &&
//              current_bridge_trajectory_.is_valid &&
//              !current_bridge_trajectory_.path_points.empty()) {

//         const auto& old_pts = current_bridge_trajectory_.path_points;
//         std::vector<Eigen::VectorXd> new_pts = old_pts;  

//         double dt_old = (old_pts.front()(2) - old_pts.back()(2)) / (old_pts.size() - 1);
//         double new_start = robot_time_to_go;
//         for (size_t i = 0; i < new_pts.size(); ++i) {
//             new_pts[i](2) = new_start - i * dt_old;
//         }

//         Trajectory re_stamped_bridge = current_bridge_trajectory_;
//         re_stamped_bridge.path_points = new_pts;

//         cached_bridge_safe = true;
//         first_unsafe_obstacle_name_.clear();

//         for (const auto& ob : obs_checker_->getObstacles()) {
//             last_replan_metrics_.obstacle_checks++;
//             if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(re_stamped_bridge, ob)) {
//                 cached_bridge_safe = false;
//                 first_unsafe_obstacle_name_ = ob.name;
//                 break;
//             }
//         }

//         if (!cached_bridge_safe) {
//             LLPT_WARN("[Set Robot State] " << trap_reason());
//             start_node_ = nullptr;
//             bridge_cost_ = std::numeric_limits<double>::infinity();
//             current_bridge_trajectory_ = Trajectory();
//             last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
//             LLPT_WARN("[Set Robot State] Cached bridge became unsafe. TRULY TRAPPED.");
//         } else {
//             bridge_cost_ = re_stamped_bridge.cost;
//             cost_of_current_anchor = re_stamped_bridge.cost + start_node_->rhs;
//             last_replan_metrics_.path_cost = cost_of_current_anchor;
//             current_bridge_trajectory_ = re_stamped_bridge;   
//             LLPT_WARN("[Set Robot State] Fresh steer failed near anchor/root. Reusing cached bridge (verified safe).");
//         }
//     }
//     // 5) Truly trapped
//     else {
//         start_node_ = nullptr;
//         bridge_cost_ = std::numeric_limits<double>::infinity();
//         current_bridge_trajectory_ = Trajectory();
//         last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
//         LLPT_WARN("[Set Robot State] LOST SAFE ANCHOR. TRULY TRAPPED.");
//     }



//     // // ========== DEBUG: Full path safety from anchor to goal ==========
//     // if (start_node_ && start_node_->rhs != std::numeric_limits<double>::infinity()) {

//     //     // 1. Check the bridge from robot to anchor
//     //     bool bridge_safe_debug = true;
//     //     if (current_bridge_trajectory_.is_valid && !current_bridge_trajectory_.path_points.empty()) {
//     //         for (const auto& ob : obs_checker_->getObstacles()) {
//     //             if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(current_bridge_trajectory_, ob)) {
//     //                 bridge_safe_debug = false;
//     //                 std::cout << "[DEBUG] Bridge from robot to anchor is UNSAFE against " << ob.name << std::endl;
//     //                 break;
//     //             }
//     //         }
//     //         if (bridge_safe_debug)
//     //             std::cout << "[DEBUG] Bridge from robot to anchor is SAFE" << std::endl;
//     //     } else {
//     //         std::cout << "[DEBUG] No valid bridge trajectory (robot already at anchor?)" << std::endl;
//     //     }

//     //     // 2. Walk the tree from anchor to goal
//     //     DStarLiteNode* node = start_node_;
//     //     int edge_count = 0;
//     //     while (node && node != goal_node_) {
//     //         DStarLiteNode* parent = node->getParent();
//     //         if (!parent) {
//     //             std::cout << "[DEBUG] Path broken at node " << node->getIndex()
//     //                     << " (no parent)" << std::endl;
//     //             break;
//     //         }

//     //         // Retrieve the edge trajectory (forward_neighbors_ from node to parent)
//     //         auto it = node->forward_neighbors_.find(parent);
//     //         if (it == node->forward_neighbors_.end()) {
//     //             std::cout << "[DEBUG] Edge from " << node->getIndex()
//     //                     << " to parent " << parent->getIndex()
//     //                     << " not found in forward_neighbors_" << std::endl;
//     //             break;
//     //         }

//     //         const EdgeInfo& edge = it->second;
//     //         if (!edge.cached_trajectory || edge.cached_trajectory->path_points.empty()) {
//     //             std::cout << "[DEBUG] Edge from " << node->getIndex()
//     //                     << " to " << parent->getIndex()
//     //                     << " has no cached trajectory" << std::endl;
//     //             break;
//     //         }

//     //         bool edge_safe = true;
//     //         for (const auto& ob : obs_checker_->getObstacles()) {
//     //             if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(*edge.cached_trajectory, ob)) {
//     //                 edge_safe = false;
//     //                 std::cout << "[DEBUG] Edge " << node->getIndex()
//     //                         << " -> " << parent->getIndex()
//     //                         << " UNSAFE against " << ob.name << std::endl;
//     //                 break;
//     //             }
//     //         }
//     //         if (edge_safe)
//     //             std::cout << "[DEBUG] Edge " << node->getIndex()
//     //                     << " -> " << parent->getIndex()
//     //                     << " SAFE (cost " << edge.distance << ")" << std::endl;

//     //         node = parent;
//     //         edge_count++;
//     //     }

//     //     if (node == goal_node_)
//     //         std::cout << "[DEBUG] Reached goal node. Total tree edges checked: " << edge_count << std::endl;
//     // }
//     // else {
//     //     std::cout << "[DEBUG] No valid anchor (start_node_ null or rhs inf)" << std::endl;
//     // }

// }



// void KinodynamicLLPTStar::setRobotState(const Eigen::VectorXd& robot_state) {
//     robot_continuous_state_ = robot_state;

//     double robot_time_to_go = 0.0;
//     if (!is_geometric_mode_ && robot_continuous_state_.size() > 0) {
//         robot_time_to_go = robot_continuous_state_(robot_continuous_state_.size() - 1);
//     }

//     Eigen::VectorXd query_point = Eigen::VectorXd::Zero(kd_dim_);
//     if (robot_continuous_state_.size() >= 2) {
//         query_point(0) = robot_continuous_state_(0);
//         query_point(1) = robot_continuous_state_(1);
//     }
//     if (kd_dim_ == 3) {
//         query_point(2) = robot_time_to_go;
//     } else if (kd_dim_ == 4) {
//         query_point(2) = robot_continuous_state_(2); 
//         query_point(3) = robot_time_to_go;
//     } else if (kd_dim_ == 5) {
//         query_point = robot_continuous_state_; 
//     }

//     const double hysteresis_factor = 0.98;
//     double cost_of_current_anchor = std::numeric_limits<double>::infinity();
    
//     Trajectory bridge;
//     bool safe = true;
//     if (start_node_ && start_node_->rhs != std::numeric_limits<double>::infinity()) {
//         bridge = statespace_->steer(robot_continuous_state_, start_node_->getStateValue());
//         if (bridge.is_valid) {
//             const auto& obstacles = obs_checker_->getObstacles();
//             for (const auto& ob : obstacles) {
//                 last_replan_metrics_.obstacle_checks++;
//                 if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(bridge, ob)) {
//                     safe = false;
//                     break;
//                 }
//             }

//             if (safe) {
//                 cost_of_current_anchor = bridge.cost + start_node_->rhs;
//                 // return;
//             }
//         }
//     }

//     // Tracks the best node that ALREADY has a valid path to the goal
//     DStarLiteNode* best_connected_node = nullptr;
//     Trajectory best_connected_bridge;
//     double best_connected_cost = std::numeric_limits<double>::infinity();
    
//     // Tracks the safest physical node to reach, even if it is currently unexplored (g = infinity)
//     DStarLiteNode* best_fallback_node = nullptr;
//     Trajectory best_fallback_bridge;
//     double best_fallback_cost = std::numeric_limits<double>::infinity();
    
//     double current_search_radius = connection_radius_;
//     const int max_attempts = 5;
//     const double radius_multiplier = 2.0;
//     std::unordered_set<size_t> tested_indices;
//     for (int attempt = 1; attempt <= max_attempts; ++attempt) {
//         std::vector<size_t> candidate_indices = kdtree_->radiusSearch(query_point, current_search_radius);

//         for (size_t idx : candidate_indices) {
//             if (!tested_indices.insert(idx).second) {
//                 continue;
//             }
//             DStarLiteNode* candidate = nodes_[idx].get();

//             // Check Steering & Collision FIRST (Do not skip nodes just because g == inf!)
//             Trajectory temp_bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());
//             if (!temp_bridge.is_valid) continue;

//             bool safe = true;
//             const auto& obstacles = obs_checker_->getObstacles();
//             for (const auto& ob : obstacles) {
//                 last_replan_metrics_.obstacle_checks++;
//                 if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(temp_bridge, ob)) {
//                     safe = false;
//                     break; // early exit on collision
//                 }
//             }

//             if (!safe) continue;


// #if USE_RECOVERY
//             // Fallback Tracking: Track the easiest physical node to reach
//             if (temp_bridge.cost < best_fallback_cost) {
//                 best_fallback_cost = temp_bridge.cost;
//                 best_fallback_node = candidate;
//                 best_fallback_bridge = temp_bridge;
//             }
// #endif



//             // Only attach to nodes that are fully consistent and NOT in the queue!
//             if (candidate->rhs != std::numeric_limits<double>::infinity() 
//                 // &&  candidate->g == candidate->rhs && !candidate->in_queue_
//                 ) 
//                 {
                
//                 double total_cost = temp_bridge.cost + candidate->rhs;
//                 if (total_cost < best_connected_cost) {
//                     best_connected_cost = total_cost;
//                     best_connected_node = candidate;
//                     best_connected_bridge = temp_bridge;
//                 }
//             }



//         }

//         // Only break early if we found a CONNECTED node. 
//         // If we only found a fallback, keep expanding radius to see if a connected one is slightly further away!
//         if (best_connected_node) break;
//         current_search_radius *= radius_multiplier;
//     }



//     // ---------- Diagnostic variables and lambda ----------
//     bool cached_bridge_safe = false;                // will be set inside the cached‑bridge block
//     std::string first_unsafe_obstacle_name_;        // ditto
//     bool anchor_selected_but_repair_failed = false;



//     auto trap_reason = [&]() -> std::string {
//         std::ostringstream oss;
//         oss << "TRAP DIAGNOSTIC: ";
//         if (best_connected_node) {
//             if (anchor_selected_but_repair_failed) {
//                 oss << "candidate selected (cost " << best_connected_cost
//                     << ") but resolvePathLazy left rhs=inf. ";
//             } else {
//                 oss << "candidate exists (cost " << best_connected_cost
//                     << ") but not better than current (" << cost_of_current_anchor
//                     << ") or hysteresis (factor " << hysteresis_factor << "). ";
//             }
//         }

//         else {
//             oss << "no candidate with safe bridge + finite rhs found within "
//                 << (current_search_radius / radius_multiplier) << " m radius. "
//                 << "Searched " << tested_indices.size() << " nodes. ";
//         }
//         if (start_node_) {
//             oss << "Current anchor rhs=" << start_node_->rhs << ", ";
//         } else {
//             oss << "No current anchor. ";
//         }
//         oss << "Fresh steer safe=" << safe << ", bridge valid=" << bridge.is_valid;
//         if (!bridge.is_valid && start_node_) {
//             oss << " (steer failed from " << robot_continuous_state_.head<2>().transpose()
//                 << " to " << start_node_->getStateValue().head<2>().transpose() << ")";
//         }
//         if (current_bridge_trajectory_.is_valid) {
//             oss << ", cached bridge exists, re‑verified="
//                 << (cached_bridge_safe ? "safe" : "unsafe");
//             if (!cached_bridge_safe) {
//                 oss << " (collision with " << first_unsafe_obstacle_name_ << ")";
//             }
//         } else {
//             oss << ", no cached bridge";
//         }
//         oss << ", recovery=" << (USE_RECOVERY ? "enabled" : "disabled");
//         return oss.str();
//     };
//     // ---------- End lambda ----------


//     // DECISION LOGIC
//     // 1) Better connected anchor
//     if (best_connected_node &&
//         best_connected_cost < cost_of_current_anchor * hysteresis_factor) {
//         if (start_node_ && start_node_ != best_connected_node) {
//             km_ += heuristic(start_node_, best_connected_node);
//         }
            
//         start_node_ = best_connected_node;
//         bridge_cost_ = best_connected_bridge.cost;
//         last_replan_metrics_.path_cost = best_connected_cost;
//         current_bridge_trajectory_ = best_connected_bridge;

     
//         auto t1 = std::chrono::steady_clock::now();
//         resolvePathLazy();
//         auto t2 = std::chrono::steady_clock::now();
//         last_anchor_repair_ms_ = std::chrono::duration<double, std::milli>(t2 - t1).count();
//         double ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
//         RCLCPP_INFO(rclcpp::get_logger("LLPT"), "Anchor repair search took: %.6f ms", ms);
//         // // If resolvePathLazy broke the anchor, discard it and trap
//         // if (start_node_->rhs == std::numeric_limits<double>::infinity()) {
//         //     anchor_selected_but_repair_failed = true;
//         //     LLPT_WARN("[Set Robot State] " << trap_reason());
//         //     start_node_ = nullptr;
//         //     LLPT_WARN("[Set Robot State] LOST SAFE ANCHOR. TRULY TRAPPED.");
//         // }
//         if (start_node_->rhs == std::numeric_limits<double>::infinity()) {
//             anchor_selected_but_repair_failed = true;
//             LLPT_WARN("[Set Robot State] " << trap_reason());
//             start_node_ = nullptr;   // no valid anchor this cycle, but we'll try again later
//         }
            
//     } 
//     // 2) Keep current anchor with fresh bridge
//     else if (safe && start_node_ &&
//              cost_of_current_anchor != std::numeric_limits<double>::infinity() &&
//              bridge.is_valid) {
//         bridge_cost_ = bridge.cost;
//         last_replan_metrics_.path_cost = cost_of_current_anchor;
//         current_bridge_trajectory_ = bridge;
//     } 
//     // 3) Recovery: go to nearest safe tree node (HIGHEST PRIORITY)
// #if USE_RECOVERY
//     else if (best_fallback_node) {
//         // start_node_ = best_fallback_node;
//         bridge_cost_ = best_fallback_cost;
//         last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
//         current_bridge_trajectory_ = best_fallback_bridge;
//         LLPT_WARN("[Set Robot State] USE_RECOVERY: Falling back to nearest safe tree node.");
//     }
// #endif
//     // // 4) Blind cached reuse (NO obstacle check - survival mode)
//     // else if (start_node_ &&
//     //          current_bridge_trajectory_.is_valid &&
//     //          !current_bridge_trajectory_.path_points.empty()) {
//     //     bridge_cost_ = current_bridge_trajectory_.cost;
//     //     cost_of_current_anchor = current_bridge_trajectory_.cost + start_node_->rhs;
//     //     last_replan_metrics_.path_cost = cost_of_current_anchor;
//     //     LLPT_WARN("[Set Robot State] Fresh steer failed near anchor/root. Reusing cached bridge.");
//     // } 

//     // 4) Reuse cached bridge (with time re‑stamping and re‑verification)
//     else if (start_node_ &&
//              start_node_->rhs != std::numeric_limits<double>::infinity() &&
//              current_bridge_trajectory_.is_valid &&
//              !current_bridge_trajectory_.path_points.empty()) {

//         // ----- Re‑stamp cached bridge to current time -----
//         const auto& old_pts = current_bridge_trajectory_.path_points;
//         std::vector<Eigen::VectorXd> new_pts = old_pts;  // copy spatial points

//         double dt_old = (old_pts.front()(2) - old_pts.back()(2)) / (old_pts.size() - 1);
//         double new_start = robot_time_to_go;
//         for (size_t i = 0; i < new_pts.size(); ++i) {
//             new_pts[i](2) = new_start - i * dt_old;
//         }

//         Trajectory re_stamped_bridge = current_bridge_trajectory_;
//         re_stamped_bridge.path_points = new_pts;

//         // ----- Re‑verify against current obstacles -----
//         cached_bridge_safe = true;
//         first_unsafe_obstacle_name_.clear();

//         const auto& obstacles = obs_checker_->getObstacles();
//         for (const auto& ob : obstacles) {
//             last_replan_metrics_.obstacle_checks++;
//             if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(
//                     re_stamped_bridge, ob)) {
//                 cached_bridge_safe = false;
//                 first_unsafe_obstacle_name_ = ob.name;
//                 break;
//             }
//         }

//         if (!cached_bridge_safe) {
//             // The old bridge is now unsafe – truly trapped.
//             LLPT_WARN("[Set Robot State] " << trap_reason());
//             start_node_ = nullptr;
//             bridge_cost_ = std::numeric_limits<double>::infinity();
//             current_bridge_trajectory_ = Trajectory();
//             last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
//             LLPT_WARN("[Set Robot State] Cached bridge became unsafe. TRULY TRAPPED.");
//         } else {
//             // The cached bridge is still safe – reuse it.
//             bridge_cost_ = re_stamped_bridge.cost;
//             cost_of_current_anchor = re_stamped_bridge.cost + start_node_->rhs;
//             last_replan_metrics_.path_cost = cost_of_current_anchor;
//             current_bridge_trajectory_ = re_stamped_bridge;   // update to re‑stamped version
//             LLPT_WARN("[Set Robot State] Fresh steer failed near anchor/root. Reusing cached bridge (verified safe).");
//         }
//     }


//     // 5) Truly trapped
//     else {
//         start_node_ = nullptr;
//         bridge_cost_ = std::numeric_limits<double>::infinity();
//         current_bridge_trajectory_ = Trajectory();
//         last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
//         LLPT_WARN("[Set Robot State] LOST SAFE ANCHOR. TRULY TRAPPED.");
//     }




// // --- DIAGNOSTIC: check anchor's ancestor consistency ---
// if (start_node_ && start_node_ != goal_node_ &&
//     time_pillar_indices_.find(start_node_->getIndex()) == time_pillar_indices_.end()) {
//     DStarLiteNode* check = start_node_->getParent();
//     int depth = 0;
//     while (check && check != goal_node_ && depth++ < 10) {
//         if (time_pillar_indices_.find(check->getIndex()) != time_pillar_indices_.end()) break;
//         if (std::isinf(check->rhs) || check->g != check->rhs) {
//             LLPT_WARN("[AnchorCheck] Anchor " << start_node_->getIndex()
//                            << " (g=" << start_node_->g << ", rhs=" << start_node_->rhs
//                            << ") has inconsistent ancestor "
//                            << check->getIndex() << " (g=" << check->g
//                            << ", rhs=" << check->rhs << ")");
//             break;
//         }
//         check = check->getParent();
//     }
// }

// }

// void KinodynamicLLPTStar::updateObstacles(const ObstacleVector& turned_obstacles) {
//     if (turned_obstacles.empty()) return;

//     double T_robot = 0.0;
//     if (!is_geometric_mode_ && robot_continuous_state_.size() > 0) {
//         T_robot = robot_continuous_state_(robot_continuous_state_.size() - 1);
//     }

//     for (const auto& incoming_ob : turned_obstacles) {
//         Obstacle& stored_ob = previous_obstacles_[incoming_ob.name];
        
//         // Remove OLD Tube
//         if (!stored_ob.predicted_path.empty()) {
//             removeObstacle(stored_ob); // NOW ONLY MARKS DIRTY & RESTORES OPTIMISTIC COST
//         }

//         // Update Logic
//         stored_ob = incoming_ob; 
//         stored_ob.predicted_path = obs_checker_->generatePrediction(stored_ob, T_robot);

//         // Add NEW Tube
//         addNewObstacle(stored_ob); // NOW ONLY MARKS EDGES is_evaluated = false
//     }

//     resolvePathLazy();

//     #if DEBUG_WITH_DIJKSTRA_
//         computeShortestPathDijkstraMode(); 
//         debugCompareDijkstraVsDStarLite();
//     #endif
// }

// void KinodynamicLLPTStar::addNewObstacle(const Obstacle& ob) {
//     if (ob.predicted_path.empty()) return;

//     double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
//     double search_radius = is_geometric_mode_ ? (obs_r + ob.inflation + connection_radius_) : (obs_r + ob.inflation + connection_radius_ + obs_r * (std::sqrt(2.0) - 1.0));

//     std::unordered_set<int> unique_node_indices;
//     for (const auto& point_3d : ob.predicted_path) {
//         Eigen::VectorXd query(kd_dim_);
//         if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
//         else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
//         else if (kd_dim_ == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
//         else if (kd_dim_ == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
//         std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
//         for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
//     }

//     for (int idx : unique_node_indices) {
//         DStarLiteNode* u = nodes_[idx].get();
        
//         for (auto& [neighbor, edge] : u->forward_neighbors_) {
//             // LLPT LAZY LOGIC: We do NOT evaluate the collision here!
//             // We just mark the edge as "dirty" because the environment changed.
//             // If the edge was previously collision-free, it stays collision-free for now.
//             edge.is_evaluated = false;
            
//             if (is_geometric_mode_ && neighbor->forward_neighbors_.count(u)) {
//                 neighbor->forward_neighbors_.at(u).is_evaluated = false;
//             }
//         }
//     }
// }


// void KinodynamicLLPTStar::removeObstacle(const Obstacle& ob) {
//     if (ob.predicted_path.empty()) return;

//     double obs_r = (ob.type == Obstacle::CIRCLE) ? ob.dimensions.radius : std::hypot(ob.dimensions.width/2.0, ob.dimensions.height/2.0);
//     double search_radius = is_geometric_mode_ ? (obs_r + ob.inflation + connection_radius_) : (obs_r + ob.inflation + connection_radius_ + obs_r * (std::sqrt(2.0) - 1.0));

//     std::unordered_set<int> unique_node_indices;
//     for (const auto& point_3d : ob.predicted_path) {
//         Eigen::VectorXd query(kd_dim_);
//         if (kd_dim_ == 3) query << point_3d.x(), point_3d.y(), point_3d.z();
//         else if (kd_dim_ == 2) query << point_3d.x(), point_3d.y();
//         else if (kd_dim_ == 4) query << point_3d.x(), point_3d.y(), M_PI, point_3d.z();
//         else if (kd_dim_ == 5) query << point_3d.x(), point_3d.y(), 0.0, 0.0, point_3d.z();
        
//         std::vector<size_t> indices = kdtree_->radiusSearch(query, search_radius);
//         for (size_t idx : indices) unique_node_indices.insert(static_cast<int>(idx));
//     }

//     // LLPT LAZY LOGIC: We DO NOT fetch all_obstacles and we DO NOT check collisions.
//     for (int idx : unique_node_indices) {
//         DStarLiteNode* u = nodes_[idx].get();
//         bool u_needs_update = false;
        
//         for (auto& [v, edge] : u->forward_neighbors_) {
//             // We only care if the edge is currently BLOCKED
//             if (edge.distance == std::numeric_limits<double>::infinity()) {
                
//                 double c_new = edge.distance_original;
                
//                 // Optimistically restore the edge
//                 edge.distance = c_new;
//                 edge.is_evaluated = false; // It must be rigorously checked if selected
//                 u_needs_update = true;

//                 if (v->backward_neighbors_.count(u)) {
//                     v->backward_neighbors_.at(u).distance = c_new;
//                 }

//                 // ---- standard overconsistent update ----
//                 if (v->g != std::numeric_limits<double>::infinity() && c_new != std::numeric_limits<double>::infinity()) {
//                     double candidate = c_new + v->g;
//                     if (candidate + 1e-9 < u->rhs) {
//                         u->rhs = candidate;
//                         u->setBestParentLLPT(v, edge.cached_trajectory);
//                     }
//                 }

//                 // handle geometric reverse edge restoration
//                 if (is_geometric_mode_ && v->forward_neighbors_.count(u)) {
//                     auto& rev_edge = v->forward_neighbors_.at(u);
//                     double rev_new = rev_edge.distance_original;
                    
//                     rev_edge.distance = rev_new;
//                     rev_edge.is_evaluated = false;
                    
//                     if (u->backward_neighbors_.count(v)) {
//                         u->backward_neighbors_.at(v).distance = rev_new;
//                     }

//                     if (u->g != std::numeric_limits<double>::infinity()) {
//                         double candidate_rev = rev_new + u->g;
//                         if (candidate_rev + 1e-9 < v->rhs) {
//                             v->rhs = candidate_rev;
//                             v->setBestParentLLPT(u, rev_edge.cached_trajectory);
//                         }
//                     }
//                     updateVertex(v);
//                 }
//             }
//         }
        
//         if (u_needs_update) {
//             updateVertex(u);
//         }
//     }
// }



void KinodynamicLLPTStar::visualizeTree() {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> dslite_tree_edges;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> dijkstra_tree_edges;
    std::vector<Eigen::VectorXd> dslite_nodes_pos;
    std::vector<Eigen::VectorXd> dijkstra_nodes_pos;

    // Reserve space – at most one edge per node
    dslite_tree_edges.reserve(nodes_.size());
    dslite_nodes_pos.reserve(nodes_.size());
    if (!dijkstra_tree_parents_.empty()) {
        dijkstra_tree_edges.reserve(dijkstra_tree_parents_.size());
        dijkstra_nodes_pos.reserve(dijkstra_tree_parents_.size());
    }

    // Build Dijkstra visited set once (if needed)
    std::unordered_set<DStarLiteNode*> dijkstra_visited;
    if (!dijkstra_tree_parents_.empty()) {
        for (const auto &kv : dijkstra_tree_parents_)
            dijkstra_visited.insert(kv.first);
    }

    for (const auto& node_ptr : nodes_) {
        DStarLiteNode* u = node_ptr.get();
        const Eigen::VectorXd pos = node_ptr->getStateValue();

        // D* Lite tree: node is connected to goal iff g is finite
        if (u->g != std::numeric_limits<double>::infinity()) {
            dslite_nodes_pos.push_back(pos);

            DStarLiteNode* parent = u->best_parent_;
            if (parent) {
                Eigen::Vector2d p1 = pos.head<2>();
                Eigen::Vector2d p2 = parent->getStateValue().head<2>();
                dslite_tree_edges.emplace_back(p1, p2);
            }
        }

        // Optional Dijkstra overlay (only if you still need it)
        auto it = dijkstra_tree_parents_.find(u);
        if (it != dijkstra_tree_parents_.end()) {
            dijkstra_nodes_pos.push_back(pos);
            DStarLiteNode* parent = it->second;
            if (parent) {
                Eigen::Vector2d p1 = pos.head<2>();
                Eigen::Vector2d p2 = parent->getStateValue().head<2>();
                dijkstra_tree_edges.emplace_back(p1, p2);
            }
        }
    }

    // --- Now send the collected data to the visualizer ---

    // Anchor point
    if (start_node_) {
        std::vector<Eigen::VectorXd> anchor_pt = { start_node_->getStateValue().head<2>() };
        visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");
    }

    // D* Lite tree (dark gray)
    if (!dslite_tree_edges.empty()) {
        visualization_->visualizeEdges(dslite_tree_edges, "map",
            std::array<float,3>{0.5f, 0.5f, 0.5f},   // dark gray
            1.0f,                                    // alpha
            0.15f,                                   // thin line
            "dslite_tree",
            2,
            false,
            0.2);
    }

    // Dijkstra tree (thin red) – comment out if not needed
    if (!dijkstra_tree_edges.empty()) {
        visualization_->visualizeEdges(dijkstra_tree_edges, "map",
            std::array<float,3>{1.0f, 0.0f, 0.0f},
            1.0f,
            0.05f,
            "dijkstra_tree",
            3,
            false,
            0.2);
    }

    // // Optionally draw all nodes (if desired)
    // if (!dslite_nodes_pos.empty())
    //     visualization_->visualizeNodes(dslite_nodes_pos, "map", {0.0f, 1.0f, 0.0f}, "dslite_nodes");
}


std::vector<Eigen::VectorXd> KinodynamicLLPTStar::getPathPositions() const{
    // FIXED: Only check for null pointer, NOT rhs == infinity!
    // Recovery nodes have rhs == infinity but are still valid anchors!
    if (!start_node_ || start_node_->rhs == std::numeric_limits<double>::infinity()) {
        LLPT_ERROR("[LLPT_Path_Assembly] Robot has no valid anchor node. Cannot build path");
        return {};
    }

    // Safety check on the cached bridge computed in setRobotState
    if (!current_bridge_trajectory_.is_valid || current_bridge_trajectory_.path_points.empty()) {
        LLPT_ERROR("LLPT_Path_Assembly: Cached bridge trajectory is invalid. Cannot build path");
        return {};
    }



    // Start the final path with the CACHED bridge trajectory! (Zero computation time)
    std::vector<Eigen::VectorXd> path = current_bridge_trajectory_.path_points;

    // Traverse Graph (Anchor -> Goal)
    DStarLiteNode* current_node = start_node_;
    int steps = 0;
    const int max_steps = nodes_.size();

    while (current_node != goal_node_) {
        if (steps++ > max_steps) {
            LLPT_WARN("[LLPT_Path_Assembly] Cycle detected. Aborting.");
            break;
        }
        
        DStarLiteNode* next_node = current_node->best_parent_;

        if (!next_node) {
            break; // Reached goal or dead end (expected for recovery nodes)
        }

        auto traj = current_node->best_parent_trajectory_;
        if (traj && traj->is_valid && traj->path_points.size() > 1) {
            // Skip the first point to avoid duplicate overlapping points at the nodes
            path.insert(path.end(), traj->path_points.begin() + 1, traj->path_points.end());
        } else {
            path.push_back(next_node->getStateValue());
        }
        current_node = next_node;
    }

    return path;
}


// std::vector<Eigen::VectorXd> KinodynamicLLPTStar::getPathPositions() const{
//     if (!start_node_ || start_node_->rhs == std::numeric_limits<double>::infinity()) {
//         LLPT_ERROR("[LLPT_Path_Assembly] Robot has no valid anchor node. Cannot build path");
//         return {};
//     }

//     if (!current_bridge_trajectory_.is_valid || current_bridge_trajectory_.path_points.empty()) {
//         LLPT_ERROR("LLPT_Path_Assembly: Cached bridge trajectory is invalid. Cannot build path");
//         return {};
//     }

//     std::vector<Eigen::VectorXd> path = current_bridge_trajectory_.path_points;

//     DStarLiteNode* current_node = start_node_;
//     int steps = 0;
//     const int max_steps = nodes_.size();

//     // ====================================================================
//     // PATH ASSEMBLY & COST ACCUMULATION LOG
//     // ====================================================================
//     std::cout << "\n================ [PATH ASSEMBLY LOG] ================\n";
//     std::cout << "[START] Anchor Node " << current_node->getIndex() 
//               << " | Pos: " << current_node->getStateValue().transpose() 
//               << " | RHS (Optimistic): " << current_node->rhs << "\n";

//     double total_true_path_cost = current_bridge_trajectory_.cost; // Start with the bridge cost
//     bool contains_lazy_edge = false;

//     while (current_node != goal_node_) {
//         if (steps++ > max_steps) {
//             LLPT_WARN("[LLPT_Path_Assembly] Cycle detected. Aborting.");
//             break;
//         }
        
//         DStarLiteNode* next_node = current_node->best_parent_;

//         if (!next_node) {
//             std::cout << "  -> [BROKEN] Node " << current_node->getIndex() << " has NO best_parent_! Stopping.\n";
//             break; 
//         }

//         // --- FETCH EDGE DATA ---
//         double edge_eval_dist = std::numeric_limits<double>::infinity();
//         double lazy_dist = std::numeric_limits<double>::infinity();
//         auto it = current_node->forward_neighbors_.find(next_node);
        
//         if (it != current_node->forward_neighbors_.end()) {
//             edge_eval_dist = it->second.distance;
//             lazy_dist = getLazyWeight(it->second);
//         }

//         std::cout << "  -> [STEP " << steps << "] Node " << current_node->getIndex() 
//                   << " -> Node " << next_node->getIndex() << "\n"
//                   << "      | LazyWeight: " << lazy_dist 
//                   << " | TrueDist: " << edge_eval_dist 
//                   << " | Pos: " << next_node->getStateValue().transpose() << "\n";
        
//         // Accumulate the cost
//         if (!std::isinf(edge_eval_dist)) {
//             total_true_path_cost += edge_eval_dist;
//         } else if (!std::isinf(lazy_dist)) {
//             // Edge is lazy (not evaluated yet). We can't add INF to the total cost, 
//             // so we add the optimistic guess, but flag the path as "Unverified"
//             total_true_path_cost += lazy_dist;
//             contains_lazy_edge = true;
//             std::cout << "      *** WARNING: FOLLOWING UNEVALUATED LAZY EDGE! ***\n";
//         }

//         // Add to path vector
//         auto traj = current_node->best_parent_trajectory_;
//         if (traj && traj->is_valid && traj->path_points.size() > 1) {
//             path.insert(path.end(), traj->path_points.begin() + 1, traj->path_points.end());
//         } else {
//             path.push_back(next_node->getStateValue());
//         }
        
//         current_node = next_node;
//     }
    
//     std::cout << "-----------------------------------------------------\n";
//     std::cout << "[SUMMARY] Reached " << (current_node == goal_node_ ? "GOAL" : "DEAD END") << "\n";
//     std::cout << "  -> Total Evaluated Physical Cost: " << total_true_path_cost;
//     if (contains_lazy_edge) {
//         std::cout << " (WARNING: Contains unevaluated lazy segments. True cost may be INF!)";
//     }
//     std::cout << "\n=====================================================\n\n";

//     return path;
// }


void KinodynamicLLPTStar::visualizeTreeGradient() {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
    std::vector<double> edge_costs;

    if (!nodes_.empty()) {
        edges.reserve(nodes_.size());
        edge_costs.reserve(nodes_.size());
    }
    
    // Find max cost in the D* Lite tree to normalize the gradient
    double max_tree_cost = 0.001; 
    for (const auto& node_ptr : nodes_) {
        DStarLiteNode* u = node_ptr.get();
        // DStarLiteNode directly accesses the 'g' value
        if (u->g > max_tree_cost && !std::isinf(u->g)) {
            max_tree_cost = u->g;
        }
    }

    for (const auto& node_ptr : nodes_) {
        DStarLiteNode* u = node_ptr.get();
        DStarLiteNode* parent_node = u->best_parent_; // The successor node pointing towards the goal

        if (parent_node && !std::isinf(u->g)) {
            // Draw edge from current node to its best parent
            edges.emplace_back(u->getStateValue(), parent_node->getStateValue());
            
            // Normalize the cost between 0.0 and 1.0
            edge_costs.push_back(u->g / max_tree_cost); 
        }
    }
    
    if (start_node_) {
        std::vector<Eigen::VectorXd> anchor_pt = { start_node_->getStateValue().head<2>() };
        visualization_->visualizeNodes(anchor_pt, "map", {0.0f, 1.0f, 1.0f}, "debug_anchor_point");
    }
    visualization_->visualizeTreeGradient(edges, edge_costs, "map");
}

void KinodynamicLLPTStar::visualizePathGradient(const std::vector<Eigen::VectorXd>& path_waypoints) {
    // D* Lite uses start_node_ to track the robot's physical location
    if (path_waypoints.size() < 2 || !visualization_ || start_node_ == nullptr) {
        return;
    }
    
    double robot_g = start_node_->g;
    if (std::isinf(robot_g)) return;

    double current_robot_cost = robot_g + bridge_cost_;
    if (global_max_cost_ < current_robot_cost) {
        global_max_cost_ = current_robot_cost;
    }
    double max_c = (global_max_cost_ > 0.0) ? global_max_cost_ : 1.0;
    std::vector<double> path_costs(path_waypoints.size(), 0.0);
    size_t N = path_waypoints.size();
    
    for (size_t i = 0; i < N; ++i) {
        double fraction = (double)(N - 1 - i) / (N - 1);
        path_costs[i] = fraction * current_robot_cost;
    }

    visualization_->visualizePathGradient(path_waypoints, path_costs, "map", max_c);
}

void KinodynamicLLPTStar::visualizePath(const std::vector<Eigen::VectorXd>& path_waypoints) {
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

    if (visualization_) {
        visualization_->visualizeEdges(edges, "map", "0.0,1.0,0.0", "executable_path");
    }
}


bool KinodynamicLLPTStar::isCurrentBridgeSafe(const ObstacleVector& obstacles) const {
    // If we don't have a valid bridge, it's definitely not safe.
    if (!current_bridge_trajectory_.is_valid || current_bridge_trajectory_.path_points.empty()) {
        return false; 
    }

    // Check the current bridge against all newly turned obstacles
    for (const auto& ob : obstacles) {
        if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(current_bridge_trajectory_, ob)) {
            return false; // The dynamic obstacle crashed into our trajectory!
        }
    }
    if (start_node_ != nullptr) {
        if (start_node_->rhs == std::numeric_limits<double>::infinity()) {
            // LLPT_WARN("[Bridge Check] GRAPH DESYNC! The immediate bridge is safe, but the anchor node (LMC=INF) has been cut off by plan()!");
            return false; // The bridge leads to a dead end!
        }
    } else {
        // LLPT_WARN("[Bridge Check] GRAPH DESYNC! robot_node_ is NULL!");
        return false;
    }

    return true;
}

bool KinodynamicLLPTStar::hasReachedAnchor(const Eigen::VectorXd& current_sim_state) const {
    // If we don't have an anchor, trigger a new search
    if (!start_node_) return true; 
    if (start_node_->rhs==INFINITY) return true; 

    // --- NEW: THE GOAL BYPASS ---
    // If the anchor we are driving toward is the Goal Node (or a Time Pillar root),
    // we NEVER consider it "reached" for the purpose of graph re-anchoring.
    // We just want to coast into the physical goal radius and let the main loop terminate.
    if (start_node_->rhs <= 1e-9 || start_node_ == goal_node_) {
        // std::cout<<"ITS GOAL! \n";
        return false; 
    }
    // ----------------------------


    // Calculate the time remaining on the current edge.
    // T_robot is the time left in the simulation budget (the z-axis in your kd-tree).
    double current_T_robot = current_sim_state(current_sim_state.size() - 1);
    
    // The anchor node's T_robot
    double anchor_T_robot = start_node_->getStateValue()(current_sim_state.size() - 1);

    // We consider the anchor "reached" if we are within a small temporal threshold 
    // of the node's timestamp (e.g., 0.000001 seconds away from the node).
    if (current_T_robot <= anchor_T_robot + 1e-6) {
        return true;
    }

    return false;
}

// void KinodynamicLLPTStar::setRobotState(const Eigen::VectorXd& robot_state) {
//     robot_continuous_state_ = robot_state;

//     double robot_time_to_go = 0.0;
//     if (!is_geometric_mode_ && robot_continuous_state_.size() > 0) {
//         robot_time_to_go = robot_continuous_state_(robot_continuous_state_.size() - 1);
//     }

//     Eigen::VectorXd query_point = Eigen::VectorXd::Zero(kd_dim_);
//     if (robot_continuous_state_.size() >= 2) {
//         query_point(0) = robot_continuous_state_(0);
//         query_point(1) = robot_continuous_state_(1);
//     }
//     if (kd_dim_ == 3) {
//         query_point(2) = robot_time_to_go;
//     } else if (kd_dim_ == 4) {
//         query_point(2) = robot_continuous_state_(2); 
//         query_point(3) = robot_time_to_go;
//     } else if (kd_dim_ == 5) {
//         query_point = robot_continuous_state_; 
//     }

//     // Tracks the TRUE best connected node (verified by resolvePathLazy)
//     DStarLiteNode* best_connected_node = nullptr;
//     Trajectory best_connected_bridge;
//     double best_connected_cost = std::numeric_limits<double>::infinity();
    
//     // Tracks the safest physical node to reach, even if it is currently unexplored (g = infinity)
//     DStarLiteNode* best_fallback_node = nullptr;
//     Trajectory best_fallback_bridge;
//     double best_fallback_cost = std::numeric_limits<double>::infinity();
    
//     double current_search_radius = connection_radius_;
//     const int max_attempts = 5;
//     const double radius_multiplier = 2.0;
//     std::unordered_set<size_t> tested_indices;
    
//     // Structure to hold candidate nodes for sorting
//     struct Candidate {
//         DStarLiteNode* node;
//         Trajectory bridge;
//         double optimistic_cost;
//     };

//     double total_repair_time = 0.0; // Accumulate time spent in resolvePathLazy

//     // 1. Store the TRUE start node and km before any testing begins
//     DStarLiteNode* original_start_node = start_node_;
//     double original_km = km_; 

//     // 1. PERFORM RADIUS SEARCH FOR A NEW ANCHOR
//     for (int attempt = 1; attempt <= max_attempts; ++attempt) {
//         std::vector<size_t> candidate_indices = kdtree_->radiusSearch(query_point, current_search_radius);
//         std::vector<Candidate> current_radius_candidates;

//         for (size_t idx : candidate_indices) {
//             if (!tested_indices.insert(idx).second) continue;
//             DStarLiteNode* candidate = nodes_[idx].get();

//             // Check Steering & Collision FIRST
//             Trajectory temp_bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());
//             if (!temp_bridge.is_valid) continue;

//             bool bridge_safe = true;
//             for (const auto& ob : obs_checker_->getObstacles()) {
//                 last_replan_metrics_.obstacle_checks++;
//                 if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(temp_bridge, ob)) {
//                     bridge_safe = false;
//                     break; 
//                 }
//             }
//             if (!bridge_safe) continue;

// #if USE_RECOVERY
//             if (temp_bridge.cost < best_fallback_cost) {
//                 best_fallback_cost = temp_bridge.cost;
//                 best_fallback_node = candidate;
//                 best_fallback_bridge = temp_bridge;
//             }
// #endif

//             // Only attach to nodes that have an optimistic path to goal
//             if (candidate->rhs != std::numeric_limits<double>::infinity()) {
//                 double optimistic_total = temp_bridge.cost + candidate->rhs;
//                 current_radius_candidates.push_back({candidate, temp_bridge, optimistic_total});
//             }
//         }

//         // Sort candidates by their optimistic heuristic cost
//         std::sort(current_radius_candidates.begin(), current_radius_candidates.end(),
//                   [](const Candidate& a, const Candidate& b) {
//                       return a.optimistic_cost < b.optimistic_cost;
//                   });

//         // std::cout<<"Attemp: "<<attempt<<" , "<<"canidadate size for this radius: "<<current_radius_candidates.size()<<"\n";
//         // Test candidates one by one using lazy evaluation
//         for (const auto& cand : current_radius_candidates) {
            
//             // 2. Compute the exact km_ for THIS specific candidate from the original anchor
//             double candidate_km_shift = 0.0;
//             if (original_start_node && original_start_node != cand.node) {
//                 candidate_km_shift = heuristic(original_start_node, cand.node);
//             }
            
//             // Set the global km_ strictly for this candidate
//             km_ = original_km + candidate_km_shift;
//             start_node_ = cand.node;

//             // 3. Evaluate the path
//             auto t1 = std::chrono::steady_clock::now();
//             resolvePathLazy(); 
//             auto t2 = std::chrono::steady_clock::now();
//             total_repair_time += std::chrono::duration<double, std::milli>(t2 - t1).count();

//             // 4. Check if the anchor survived
//             if (start_node_->rhs != std::numeric_limits<double>::infinity()) {
//                 // SUCCESS! 
//                 best_connected_node = start_node_;
//                 best_connected_bridge = cand.bridge;
//                 best_connected_cost = cand.bridge.cost + start_node_->rhs; 
//                 break; 
//             }
//             else{
//                 std::cout<<"FAILED ATTEMPT \n";
//             }
//         }
        
//         if (!best_connected_node) {
//             km_ = original_km;   // no anchor change → keep original km_
//             start_node_ = original_start_node; 
//         }

//         // Only break early if we found a CONNECTED and VERIFIED node
//         if (best_connected_node) break;
//         current_search_radius *= radius_multiplier;
//     }

//     last_anchor_repair_ms_ = total_repair_time;

//     // 2. SIMPLE ASSIGNMENT LOGIC (We only get here if we MUST switch anchors)
//     if (best_connected_node) {
//         start_node_ = best_connected_node;
//         bridge_cost_ = best_connected_bridge.cost;
//         last_replan_metrics_.path_cost = best_connected_cost;
//         current_bridge_trajectory_ = best_connected_bridge;
//     } 
// #if USE_RECOVERY
//     else if (best_fallback_node) {
//         bridge_cost_ = best_fallback_cost;
//         last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
//         current_bridge_trajectory_ = best_fallback_bridge;
//         LLPT_WARN("[Set Robot State] USE_RECOVERY: Falling back to nearest safe tree node.");
//     }
// #endif
//     else {
//         start_node_ = nullptr;
//         bridge_cost_ = std::numeric_limits<double>::infinity();
//         current_bridge_trajectory_ = Trajectory();
//         last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
//         LLPT_WARN("[Set Robot State] LOST SAFE ANCHOR. TRULY TRAPPED.");
//     }
// }


void KinodynamicLLPTStar::setRobotState(const Eigen::VectorXd& robot_state) {
    robot_continuous_state_ = robot_state;

    double robot_time_to_go = 0.0;
    if (!is_geometric_mode_ && robot_continuous_state_.size() > 0) {
        robot_time_to_go = robot_continuous_state_(robot_continuous_state_.size() - 1);
    }

    Eigen::VectorXd query_point = Eigen::VectorXd::Zero(kd_dim_);
    if (robot_continuous_state_.size() >= 2) {
        query_point(0) = robot_continuous_state_(0);
        query_point(1) = robot_continuous_state_(1);
    }
    if (kd_dim_ == 3) {
        query_point(2) = robot_time_to_go;
    } else if (kd_dim_ == 4) {
        query_point(2) = robot_continuous_state_(2); 
        query_point(3) = robot_time_to_go;
    } else if (kd_dim_ == 5) {
        query_point = robot_continuous_state_; 
    }

    // Tracks the TRUE best connected node (verified by resolvePathLazy)
    DStarLiteNode* best_connected_node = nullptr;
    Trajectory best_connected_bridge;
    double best_connected_cost = std::numeric_limits<double>::infinity();
    
    // Tracks the safest physical node to reach, even if it is currently unexplored (g = infinity)
    DStarLiteNode* best_fallback_node = nullptr;
    Trajectory best_fallback_bridge;
    double best_fallback_cost = std::numeric_limits<double>::infinity();
    
    double current_search_radius = connection_radius_;
    const int max_attempts = 5;
    const double radius_multiplier = 2.0;
    std::unordered_set<size_t> tested_indices;
    
    // Structure to hold candidate nodes for sorting
    struct Candidate {
        DStarLiteNode* node;
        Trajectory bridge;
        double optimistic_cost;
    };

    double total_repair_time = 0.0; // Accumulate time spent in resolvePathLazy

    // 1. Store the TRUE start node before testing, and track the shifting anchor
    DStarLiteNode* original_start_node = start_node_;
    DStarLiteNode* current_anchor = start_node_; 
    // Notice: We NO LONGER store original_km! km_ will only ever strictly increase.

    // 1. PERFORM RADIUS SEARCH FOR A NEW ANCHOR
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        std::vector<size_t> candidate_indices = kdtree_->radiusSearch(query_point, current_search_radius);
        std::vector<Candidate> current_radius_candidates;

        for (size_t idx : candidate_indices) {
            if (!tested_indices.insert(idx).second) continue;
            DStarLiteNode* candidate = nodes_[idx].get();

            // Check Steering & Collision FIRST
            Trajectory temp_bridge = statespace_->steer(robot_continuous_state_, candidate->getStateValue());
            if (!temp_bridge.is_valid) continue;

            bool bridge_safe = true;
            for (const auto& ob : obs_checker_->getObstacles()) {
                last_replan_metrics_.obstacle_checks++;
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(temp_bridge, ob)) {
                    bridge_safe = false;
                    break; 
                }
            }
            if (!bridge_safe) continue;

#if USE_RECOVERY
            if (temp_bridge.cost < best_fallback_cost) {
                best_fallback_cost = temp_bridge.cost;
                best_fallback_node = candidate;
                best_fallback_bridge = temp_bridge;
            }
#endif

            // Only attach to nodes that have an optimistic path to goal
            if (candidate->rhs != std::numeric_limits<double>::infinity()) {
                double optimistic_total = temp_bridge.cost + candidate->rhs;
                current_radius_candidates.push_back({candidate, temp_bridge, optimistic_total});
            }
        }

        // Sort candidates by their optimistic heuristic cost
        std::sort(current_radius_candidates.begin(), current_radius_candidates.end(),
                  [](const Candidate& a, const Candidate& b) {
                      return a.optimistic_cost < b.optimistic_cost;
                  });

        // Test candidates one by one using lazy evaluation
        for (const auto& cand : current_radius_candidates) {
            
            // 2. INCREMENTAL KM SHIFT
            // Calculate shift from the LAST tested candidate (or original start) to the NEW candidate
            double candidate_km_shift = 0.0;
            if (current_anchor && current_anchor != cand.node) {
                candidate_km_shift = heuristic(current_anchor, cand.node);
            }
            
            // Accumulate km_ (Because h >= 0, km_ strictly increases, preserving D* Lite invariant)
            km_ += candidate_km_shift;
            start_node_ = cand.node;
            current_anchor = cand.node; // Update tracking for next shift

            // 3. Evaluate the path
            auto t1 = std::chrono::steady_clock::now();
            resolvePathLazy(); 
            auto t2 = std::chrono::steady_clock::now();
            total_repair_time += std::chrono::duration<double, std::milli>(t2 - t1).count();

            // 4. Check if the anchor survived
            if (start_node_->rhs != std::numeric_limits<double>::infinity()) {
                // SUCCESS! 
                best_connected_node = start_node_;
                best_connected_bridge = cand.bridge;
                best_connected_cost = cand.bridge.cost + start_node_->rhs; 
                break; 
            }
            else{
                std::cout<<"FAILED ATTEMPT \n";
            }
        }
        
        // If this radius attempt failed to find a connected node, we must revert the start node.
        // However, to keep km_ monotonic, we mathematically "move" the search anchor back to original!
        if (!best_connected_node) {
            double revert_km_shift = 0.0;
            if (current_anchor && current_anchor != original_start_node) {
                revert_km_shift = heuristic(current_anchor, original_start_node);
            }
            km_ += revert_km_shift; // Keep increasing km_!
            start_node_ = original_start_node; 
            current_anchor = original_start_node; // Ready for the next radius expansion
        }

        // Only break early if we found a CONNECTED and VERIFIED node
        if (best_connected_node) break;
        current_search_radius *= radius_multiplier;
    }

    last_anchor_repair_ms_ = total_repair_time;

    // 2. SIMPLE ASSIGNMENT LOGIC (We only get here if we MUST switch anchors)
    if (best_connected_node) {
        start_node_ = best_connected_node;
        bridge_cost_ = best_connected_bridge.cost;
        last_replan_metrics_.path_cost = best_connected_cost;
        current_bridge_trajectory_ = best_connected_bridge;
    } 
#if USE_RECOVERY
    else if (best_fallback_node) {
        bridge_cost_ = best_fallback_cost;
        last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
        current_bridge_trajectory_ = best_fallback_bridge;
        LLPT_WARN("[Set Robot State] USE_RECOVERY: Falling back to nearest safe tree node.");
    }
#endif
    else {
        start_node_ = nullptr;
        bridge_cost_ = std::numeric_limits<double>::infinity();
        current_bridge_trajectory_ = Trajectory();
        last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
        LLPT_WARN("[Set Robot State] LOST SAFE ANCHOR. TRULY TRAPPED.");
    }
}


// SEMI-EAGER APPROACH --> NOT FAITHFUL TO THE PAPER --> Better not use it for benchmark!
// inflates the success rate (even though its a good architecture change on top of LLPT but 
// its not purely lazy! because we shouldnt touch collision check (i.e, calling resolvepathlazy) unless we have to!)
/*
    Dont think this is all good too! because here also lies a trade off for safety and speed! the more hashosrtcut call means the more
    time wasted on resolvepathlazy! but I think for deployment usage of the strcit lazyness of LLPT we must use this in practice 
    but usng this kinda semi-eager (lookahead like) approach by paying the price on repair time which the paper avoided!
    but overall for benchmarking we dont use the following to get the pure layziness tradeoff!

*/
bool KinodynamicLLPTStar::hasShortcut(const Eigen::VectorXd& robot_state, double threshold) {
    if (!start_node_ || start_node_->rhs == std::numeric_limits<double>::infinity()) return false;
    double current_cost = bridge_cost_ + start_node_->rhs;
    if (current_cost <= 0.001) return false;


    double robot_time_to_go = 0.0;
    if (!is_geometric_mode_ && robot_state.size() > 0) {
        robot_time_to_go = robot_state(robot_state.size() - 1);
    }

    Eigen::VectorXd query_point = Eigen::VectorXd::Zero(kd_dim_);
    if (robot_state.size() >= 2) {
        query_point(0) = robot_state(0);
        query_point(1) = robot_state(1);
    }
    if (kd_dim_ == 3) {
        query_point(2) = robot_time_to_go;
    } else if (kd_dim_ == 4) {
        query_point(2) = robot_state(2);
        query_point(3) = robot_time_to_go;
    } else if (kd_dim_ == 5) {
        query_point = robot_state;
    }

    double current_search_radius = 2*connection_radius_;   // ALIGN with setRobotState (was *2.0)
    const int max_attempts = 5;
    const double radius_multiplier = 2.0;
    std::unordered_set<size_t> tested_indices;

    struct Candidate { DStarLiteNode* node; Trajectory bridge; double optimistic_cost; };

    DStarLiteNode* original_start = start_node_;
    double original_km = km_;
    bool found = false;

    for (int attempt = 1; attempt <= max_attempts && !found; ++attempt) {
        auto idxs = kdtree_->radiusSearch(query_point, current_search_radius);
        std::vector<Candidate> cands;
        for (size_t idx : idxs) {
            if (!tested_indices.insert(idx).second) continue;
            DStarLiteNode* c = nodes_[idx].get();
            if (c->rhs == std::numeric_limits<double>::infinity()) continue;

            Trajectory b = statespace_->steer(robot_state, c->getStateValue());
            if (!b.is_valid) continue;

            // optimistic prefilter is SOUND: rhs underestimates, so true_cost >= optimistic_cost.
            // If optimistic improvement < threshold, true improvement can't reach it either.
            double opt = b.cost + c->rhs;
            if ((current_cost - opt) / current_cost < threshold) continue;

            bool safe = true;
            for (const auto& ob : obs_checker_->getObstacles()) {
                if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(b, ob)) { safe = false; break; }
            }
            if (!safe) continue;

            cands.push_back({c, b, opt});
        }
        std::sort(cands.begin(), cands.end(),
                  [](const Candidate& a, const Candidate& b){ return a.optimistic_cost < b.optimistic_cost; });

        for (const auto& cand : cands) {
            double km_shift = (original_start != cand.node) ? heuristic(original_start, cand.node) : 0.0;
            km_ = original_km + km_shift;
            start_node_ = cand.node;
            resolvePathLazy();

            if (start_node_->rhs != std::numeric_limits<double>::infinity()) {
                double true_cost = cand.bridge.cost + start_node_->rhs;
                if ((current_cost - true_cost) / current_cost >= threshold) { found = true; break; }
            }
        }
        current_search_radius *= radius_multiplier;
    }

    start_node_ = original_start;   // probe only — roll back anchor + km_
    km_ = original_km;
    return found;
}





std::vector<Eigen::VectorXd> KinodynamicLLPTStar::getLivePathPositions(const Eigen::VectorXd& current_state) const
{
    // Check if the planner has a valid anchor point for the robot
    if (!start_node_ || start_node_->rhs == std::numeric_limits<double>::infinity()) {
        return {}; 
    }

    // 1. Create a temporary, real-time bridge from the robot to the anchor
    Trajectory live_bridge = statespace_->steer(current_state, start_node_->getStateValue());
    
    if (!live_bridge.is_valid || live_bridge.path_points.empty()) {
        // If the live steer fails (e.g. numerical issue very close to the node),
        // fallback to the cached path
        return getPathPositions(); 
    }

    // 2. Start the final path with the live bridge trajectory
    std::vector<Eigen::VectorXd> final_executable_path = live_bridge.path_points;

    // 3. Traverse the rest of the tree from the anchor node using parent pointers
    DStarLiteNode* current = start_node_;
    DStarLiteNode* parent = current->getParent();

    while (parent) {
        auto cached_traj = current->getParentTrajectory();
        if (cached_traj && cached_traj->is_valid && cached_traj->path_points.size() > 1) {
            // Append all points from the segment except the first one to avoid duplicates
            final_executable_path.insert(final_executable_path.end(),
                                         cached_traj->path_points.begin() + 1,
                                         cached_traj->path_points.end());
        } else {
            // If a valid cached trajectory doesn't exist, stop traversing
            break;
        }
        current = parent;
        parent = current->getParent();
    }

    return final_executable_path;
}








/*
    Important! basically, setRobotstate function for LLPT is the bottle neck! so if getting oppurtunistic cost imporvement is a double edges sword!
    so I'm thinking to completely ignore the hasShortcut function call in the main loop when benchmarking or else the results gonna be bad for LLPT!
    it's a correct feature for eager planners. the one that they could use very efficiently but for llpt is like all the delayed lazy work is gonna catch up!
    so im only gonna call the orignal setrobotstate (the above function not below) when necessary (edge_destryoed or reached the current anchor)

*/

// // IF YOU MUST USE hasShortcut its better to use the following pair! with hasShortcut being only readonly and setrobotstate also considers the original anchor. 
// // hasShortctus has a lot of false positives and it makes the setrobotstate to be called alot and setrobotstate loses its orignal anchor if you dont consider the orignal one!

// void KinodynamicLLPTStar::setRobotState(const Eigen::VectorXd& robot_state) {
//     robot_continuous_state_ = robot_state;

//     double robot_time_to_go = 0.0;
//     if (!is_geometric_mode_ && robot_continuous_state_.size() > 0) {
//         robot_time_to_go = robot_continuous_state_(robot_continuous_state_.size() - 1);
//     }

//     Eigen::VectorXd query_point = Eigen::VectorXd::Zero(kd_dim_);
//     if (robot_continuous_state_.size() >= 2) {
//         query_point(0) = robot_continuous_state_(0);
//         query_point(1) = robot_continuous_state_(1);
//     }
//     if (kd_dim_ == 3) {
//         query_point(2) = robot_time_to_go;
//     } else if (kd_dim_ == 4) {
//         query_point(2) = robot_continuous_state_(2);
//         query_point(3) = robot_time_to_go;
//     } else if (kd_dim_ == 5) {
//         query_point = robot_continuous_state_;
//     }

//     // Original routing state. All probing mutations get rolled back to this.
//     DStarLiteNode* original_start_node = start_node_;
//     double original_km = km_;

//     // ---- Seed the INCUMBENT as a baseline, measured from the CURRENT position ----
//     bool       incumbent_valid = false;
//     double     incumbent_cost  = std::numeric_limits<double>::infinity();
//     if (original_start_node &&
//         original_start_node->rhs != std::numeric_limits<double>::infinity()) {
//         Trajectory inc = statespace_->steer(robot_continuous_state_,
//                                             original_start_node->getStateValue());
//         if (inc.is_valid) {
//             bool safe = true;
//             for (const auto& ob : obs_checker_->getObstacles()) {
//                 last_replan_metrics_.obstacle_checks++;
//                 if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(inc, ob)) {
//                     safe = false; break;
//                 }
//             }
//             if (safe) {
//                 incumbent_valid = true;
//                 incumbent_cost  = inc.cost + original_start_node->rhs;
//             }
//         }
//     }

//     // A candidate must beat this to be worth a switch (inf if no valid incumbent).
//     double switch_margin_ = 0.1;
//     const double switch_threshold = incumbent_cost * (1.0 - switch_margin_);

//     // Best CANDIDATE that already beats switch_threshold (not the incumbent).
//     DStarLiteNode* best_candidate_node = nullptr;
//     Trajectory     best_candidate_bridge;
//     double         best_candidate_cost = switch_threshold;

//     // Safest physical node, even if unexplored (recovery).
//     DStarLiteNode* best_fallback_node = nullptr;
//     Trajectory     best_fallback_bridge;
//     double         best_fallback_cost = std::numeric_limits<double>::infinity();

//     double current_search_radius = connection_radius_;
//     const int max_attempts = 5;
//     const double radius_multiplier = 2.0;
//     std::unordered_set<size_t> tested_indices;

//     struct Candidate {
//         DStarLiteNode* node;
//         Trajectory bridge;
//         double optimistic_cost;
//     };

//     double total_repair_time = 0.0;

//     for (int attempt = 1; attempt <= max_attempts; ++attempt) {
//         std::vector<size_t> candidate_indices =
//             kdtree_->radiusSearch(query_point, current_search_radius);
//         std::vector<Candidate> radius_candidates;

//         for (size_t idx : candidate_indices) {
//             if (!tested_indices.insert(idx).second) continue;
//             DStarLiteNode* candidate = nodes_[idx].get();

//             Trajectory temp_bridge =
//                 statespace_->steer(robot_continuous_state_, candidate->getStateValue());
//             if (!temp_bridge.is_valid) continue;

//             bool bridge_safe = true;
//             for (const auto& ob : obs_checker_->getObstacles()) {
//                 last_replan_metrics_.obstacle_checks++;
//                 if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(temp_bridge, ob)) {
//                     bridge_safe = false; break;
//                 }
//             }
//             if (!bridge_safe) continue;

// #if USE_RECOVERY
//             if (temp_bridge.cost < best_fallback_cost) {
//                 best_fallback_cost   = temp_bridge.cost;
//                 best_fallback_node   = candidate;
//                 best_fallback_bridge = temp_bridge;
//             }
// #endif
//             if (candidate->rhs != std::numeric_limits<double>::infinity()) {
//                 radius_candidates.push_back(
//                     {candidate, temp_bridge, temp_bridge.cost + candidate->rhs});
//             }
//         }

//         std::sort(radius_candidates.begin(), radius_candidates.end(),
//                   [](const Candidate& a, const Candidate& b) {
//                       return a.optimistic_cost < b.optimistic_cost;
//                   });

//         // Probe in optimistic order; keep the best VERIFIED candidate.
//         for (const auto& cand : radius_candidates) {
//             if (cand.node == original_start_node) continue;           // incumbent handled above
//             if (cand.optimistic_cost >= best_candidate_cost) break;   // LB can't beat -> stop

//             double km_shift = 0.0;
//             if (original_start_node && original_start_node != cand.node) {
//                 km_shift = heuristic(original_start_node, cand.node);
//             }
//             km_         = original_km + km_shift;
//             start_node_ = cand.node;

//             auto t1 = std::chrono::steady_clock::now();
//             resolvePathLazy();
//             auto t2 = std::chrono::steady_clock::now();
//             total_repair_time += std::chrono::duration<double, std::milli>(t2 - t1).count();

//             if (start_node_->rhs != std::numeric_limits<double>::infinity()) {
//                 double verified = cand.bridge.cost + start_node_->rhs;
//                 if (verified < best_candidate_cost) {
//                     best_candidate_cost   = verified;
//                     best_candidate_node   = cand.node;
//                     best_candidate_bridge = cand.bridge;
//                 }
//             }
//         }

//         // ALWAYS roll back routing. Blocked-edge discoveries persist in the graph/E_eval;
//         // only start_node_/km_ are restored so probing leaves no side effect on the anchor.
//         start_node_ = original_start_node;
//         km_         = original_km;

//         // We have a usable target (incumbent or a margin-beating candidate). Stop widening.
//         if (best_candidate_node || incumbent_valid) break;
//         current_search_radius *= radius_multiplier;
//     }

//     last_anchor_repair_ms_ = total_repair_time;

//     // ---- COMMIT ----
//     if (best_candidate_node) {
//         // A candidate beat the incumbent by switch_margin_ -> SWITCH.
//         start_node_                    = best_candidate_node;
//         bridge_cost_                   = best_candidate_bridge.cost;
//         current_bridge_trajectory_     = best_candidate_bridge;
//         last_replan_metrics_.path_cost = best_candidate_cost;
//     }
//     else if (incumbent_valid) {
//         // Incumbent survives and nothing beat the margin -> KEEP IT.
//         // Leave the committed bridge untouched to avoid controller discontinuity.
//         start_node_                    = original_start_node;
//         km_                            = original_km;
//         last_replan_metrics_.path_cost = incumbent_cost;
//     }
// #if USE_RECOVERY
//     else if (best_fallback_node) {
//         start_node_                    = original_start_node;  // routing unchanged
//         km_                            = original_km;
//         bridge_cost_                   = best_fallback_cost;
//         current_bridge_trajectory_     = best_fallback_bridge;
//         last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
//         LLPT_WARN("[Set Robot State] USE_RECOVERY: Falling back to nearest safe tree node.");
//     }
// #endif
//     else {
//         start_node_                    = nullptr;
//         bridge_cost_                   = std::numeric_limits<double>::infinity();
//         current_bridge_trajectory_     = Trajectory();
//         last_replan_metrics_.path_cost = std::numeric_limits<double>::infinity();
//         LLPT_WARN("[Set Robot State] LOST SAFE ANCHOR. TRULY TRAPPED.");
//     }
// }



// bool KinodynamicLLPTStar::hasShortcut(const Eigen::VectorXd& robot_state, double threshold) {
//     if (!start_node_ || start_node_->rhs == std::numeric_limits<double>::infinity()) return false;
//     double current_cost = bridge_cost_ + start_node_->rhs;
//     if (current_cost <= 0.001) return false;

//     double robot_time_to_go = 0.0;
//     if (!is_geometric_mode_ && robot_state.size() > 0) {
//         robot_time_to_go = robot_state(robot_state.size() - 1);
//     }

//     Eigen::VectorXd query_point = Eigen::VectorXd::Zero(kd_dim_);
//     if (robot_state.size() >= 2) { query_point(0) = robot_state(0); query_point(1) = robot_state(1); }
//     if (kd_dim_ == 3)      { query_point(2) = robot_time_to_go; }
//     else if (kd_dim_ == 4) { query_point(2) = robot_state(2); query_point(3) = robot_time_to_go; }
//     else if (kd_dim_ == 5) { query_point = robot_state; }

//     double current_search_radius = 2 * connection_radius_;   // aligned with setRobotState
//     const int max_attempts = 5;
//     const double radius_multiplier = 2.0;
//     std::unordered_set<size_t> tested_indices;

//     for (int attempt = 1; attempt <= max_attempts; ++attempt) {
//         auto idxs = kdtree_->radiusSearch(query_point, current_search_radius);
//         for (size_t idx : idxs) {
//             if (!tested_indices.insert(idx).second) continue;
//             DStarLiteNode* c = nodes_[idx].get();
//             if (c->rhs == std::numeric_limits<double>::infinity()) continue;

//             Trajectory b = statespace_->steer(robot_state, c->getStateValue());
//             if (!b.is_valid) continue;

//             // optimistic prefilter is SOUND: rhs underestimates, so true_cost >= opt.
//             // if optimistic improvement < threshold, true improvement can't reach it either.
//             double opt = b.cost + c->rhs;
//             if ((current_cost - opt) / current_cost < threshold) continue;

//             // single bridge-edge check — the ONLY collision work in the trigger,
//             // symmetric with FMTX's one-bridge peek.
//             bool safe = true;
//             for (const auto& ob : obs_checker_->getObstacles()) {
//                 if (!obs_checker_->isTrajectorySafeAgainstSingleObstacle(b, ob)) { safe = false; break; }
//             }
//             if (!safe) continue;

//             // optimistic trigger satisfied + bridge clear -> worth interrupting.
//             // truth is deferred to setRobotState (resolve + probe), where it's counted.
//             return true;
//         }
//         current_search_radius *= radius_multiplier;
//     }
//     return false;
// }