#include "motion_planning/planners/geometric/bit_star.hpp"

BITStar::BITStar(std::unique_ptr<StateSpace> statespace,
               std::shared_ptr<ProblemDefinition> problem_def,
               std::shared_ptr<ObstacleChecker> obs_checker)
    : statespace_(std::move(statespace)),
      problem_(problem_def),
      obs_checker_(obs_checker),
      ci_(INFINITY),
      start_node_(nullptr),
      goal_node_(nullptr),
      batch_size_(100),
      samples_added_(0) {}

void BITStar::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    visualization_ = visualization;
    radius_ = params.getParam<double>("bitstar_radius");
    kdtree_ = std::make_shared<NanoFlann>(statespace_->getDimension());
    initialize();
}

void BITStar::initialize() {
    nodes_.clear();
    unconnected_.clear();
    solution_nodes_.clear();
    ci_ = INFINITY;

    setStart(problem_->getStart());
    setGoal(problem_->getGoal());
    
    sampleBatch(true);
}

void BITStar::setStart(const Eigen::VectorXd& start) {
    auto state = statespace_->addState(start);
    nodes_.push_back(std::make_unique<BITNode>(std::move(state), nodes_.size()));
    start_node_ = nodes_.back().get();
    start_node_->g = 0.0;
    start_node_->h_hat = heuristic(start_node_->getStateValue());
    start_node_->in_unconnected = false;
    vertex_queue_.emplace(start_node_->g + start_node_->h_hat, start_node_);
}

void BITStar::setGoal(const Eigen::VectorXd& goal) {
    auto state = statespace_->addState(goal);
    nodes_.push_back(std::make_unique<BITNode>(std::move(state), nodes_.size()));
    goal_node_ = nodes_.back().get();
    goal_node_->h_hat = 0.0;
    unconnected_.push_back(goal_node_);
}

double BITStar::heuristic(const Eigen::VectorXd& state) const {
    return (problem_->getGoal() - state).norm();
}

void BITStar::plan() {
    while (!problem_->hasSolution()) {
        processBatch();
        
        if (vertex_queue_.empty() && edge_queue_.empty()) {
            prune();
            sampleBatch();
        }
        
        updateSolution();
        
        if (visualization_) {
            visualizeTree();
        }
    }
}


void BITStar::processBatch() {
    while (!vertex_queue_.empty() || !edge_queue_.empty()) {
        // Expand vertices until edge queue has better candidates
        while (!vertex_queue_.empty() && 
              (edge_queue_.empty() || vertex_queue_.top().first <= std::get<0>(edge_queue_.top()))) {
            auto [f_val, vertex] = vertex_queue_.top();
            vertex_queue_.pop();
            expandVertex(vertex);
        }

        if (!edge_queue_.empty()) {
            auto [f_edge, order, c_hat, parent, child] = edge_queue_.top();
            edge_queue_.pop();

            // Condition 1: Estimated solution cost
            if (f_edge >= ci_) continue;
            
            // Condition 2: Estimated cost improvement
            if (parent->g + c_hat >= child->g) continue;

            // Collision check
            if (!obs_checker_->isObstacleFree(parent->getStateValue(), child->getStateValue()))
                continue;

            double actual_cost = (parent->getStateValue() - child->getStateValue()).norm();
            
            // Condition 3: Actual solution cost
            if (parent->g + actual_cost + child->h_hat >= ci_) continue;
            
            // Condition 4: Actual cost improvement
            if (parent->g + actual_cost < child->g) {
                addEdge(parent, child, actual_cost);
                if (child == goal_node_)
                    ci_ = parent->g + actual_cost;
            }
        }
    }
}

void BITStar::expandVertex(BITNode* vertex) {
    std::vector<BITNode*> near_nodes;
    
    if (vertex->expanded) {
        // Only consider new unconnected nodes
        near_nodes = findNear(vertex, unconnected_);
    } else {
        // // Consider all nearby nodes
        // near_nodes = findNear(vertex, nodes_);

        std::vector<BITNode*> temp_nodes;
        temp_nodes.reserve(nodes_.size());
        for (const auto& node : nodes_) {
            temp_nodes.push_back(node.get());
        }
        near_nodes = findNear(vertex, temp_nodes);


    }

    for (BITNode* neighbor : near_nodes) {
        double c_hat = (vertex->getStateValue() - neighbor->getStateValue()).norm();
        double edge_cost_est = vertex->g + c_hat + neighbor->h_hat;
        
        if (edge_cost_est < ci_) {
            edge_queue_.emplace(edge_cost_est, qe_order_++, c_hat, vertex, neighbor);
        }
    }
    
    vertex->expanded = true;
}

void BITStar::addEdge(BITNode* parent, BITNode* child, double cost) {
    child->g = parent->g + cost;
    child->setParent(parent, cost);
    
    // Move from unconnected to tree
    if (child->in_unconnected) {
        unconnected_.erase(std::remove(unconnected_.begin(), unconnected_.end(), child), 
                          unconnected_.end());
        child->in_unconnected = false;
    }
    
    vertex_queue_.emplace(child->g + child->h_hat, child);
}

void BITStar::sampleBatch(bool initial_batch) {
    std::vector<Eigen::VectorXd> samples;
    double min = problem_->getLowerBound();
    double max = problem_->getUpperBound();
    
    if (ci_ < INFINITY) {
        // PHS sampling
        Eigen::VectorXd start = problem_->getStart();
        Eigen::VectorXd goal = problem_->getGoal();
        double c_min = (goal - start).norm();
        
        Eigen::VectorXd center = (start + goal) / 2.0;
        Eigen::VectorXd a1 = (goal - start).normalized();
        
        // Create rotation matrix (2D example)
        Eigen::Matrix2d cwe;
        double theta = std::atan2(a1[1], a1[0]);
        cwe << std::cos(theta), -std::sin(theta),
               std::sin(theta), std::cos(theta);
        
        double r1 = ci_ / 2.0;
        double r2 = std::sqrt(ci_*ci_ - c_min*c_min) / 2.0;
        
        for (int i = 0; i < batch_size_; ++i) {
            // Sample unit ball
            Eigen::Vector2d x_ball = Eigen::Vector2d::Random().normalized();
            x_ball *= pow(static_cast<double>(rand())/RAND_MAX, 1.0/2.0); // 2D
            
            // Transform to PHS
            Eigen::Vector2d sample = cwe * (Eigen::Vector2d(r1, r2).asDiagonal() * x_ball) + center;
            
            if (obs_checker_->isObstacleFree(sample))
                samples.push_back(sample);
        }
    } else {
        // Uniform sampling
        statespace_->sampleUniform(min, max, batch_size_);
        Eigen::MatrixXd samples_matrix = statespace_->getSamples();
        samples.clear();
        samples.reserve(samples_matrix.rows());
        for (int i = 0; i < samples_matrix.rows(); ++i) {
            samples.push_back(samples_matrix.row(i));
        }
    }

    // Add new nodes
    for (auto& sample : samples) {
        auto state = statespace_->addState(sample);
        nodes_.push_back(std::make_unique<BITNode>(std::move(state), nodes_.size()));
        BITNode* node = nodes_.back().get();
        node->h_hat = heuristic(node->getStateValue());
        unconnected_.push_back(node);
        kdtree_->addPoint(node->getStateValue()); // Changed from insert()
    }
    samples_added_ += batch_size_;
}

std::vector<BITStar::BITNode*> BITStar::findNear(BITNode* node, const std::vector<BITNode*>& nodes) {
    std::vector<BITNode*> near_nodes;
    std::vector<size_t> neighbor_indices = kdtree_->radiusSearch(node->getStateValue(), radius_);
    for (size_t index : neighbor_indices) {
        BITNode* neighbor = nodes_.at(index).get();
        // Check if neighbor is in the 'nodes' list and meets other conditions
        if (std::find(nodes.begin(), nodes.end(), neighbor) != nodes.end() &&
            neighbor != node &&
            (neighbor->getStateValue() - node->getStateValue()).norm() < radius_) {
            near_nodes.push_back(neighbor);
        }
    }
    return near_nodes;
}

void BITStar::prune() {
    // Prune unconnected nodes
    auto it = std::remove_if(unconnected_.begin(), unconnected_.end(),
        [this](BITNode* n) { 
            return (n->g + n->h_hat) >= ci_;
        });
    unconnected_.erase(it, unconnected_.end());

    // Prune tree nodes and collect reusable
    x_reuse_.clear();
    for (auto& node : nodes_) {
        if (node->g + node->h_hat >= ci_ && node.get() != start_node_) {
            if (node->g + node->h_hat < ci_) { // Reusable
                x_reuse_.push_back(node.get());
            }
            node->disconnectFromGraph();
        }
    }
}

void BITStar::updateSolution() {
    if (goal_node_->g < ci_) {
        ci_ = goal_node_->g;
        solution_nodes_.clear();
        for (BITNode* node = goal_node_; node != nullptr; node = static_cast<BITNode*>(node->getParent())) {
            solution_nodes_.push_back(node);
        }
        std::reverse(solution_nodes_.begin(), solution_nodes_.end());
        problem_->setSolution(ci_);
    }
}

void BITStar::visualizeTree() {
    std::vector<Eigen::VectorXd> tree_nodes;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;

    for (const auto& node : nodes_) {
        tree_nodes.push_back(node->getStateValue());
        if (node->getParent()) {
            edges.emplace_back(node->getParent()->getStateValue(), node->getStateValue());
        }
    }

    visualization_->visualizeNodes(tree_nodes, "map", {1.0, 0.0, 0.0}, "bit_nodes");
    visualization_->visualizeEdges(edges, "map");
    
    // // Visualize solution path
    // if (!solution_nodes_.empty()) {
    //     std::vector<Eigen::VectorXd> path;
    //     for (auto* node : solution_nodes_) {
    //         path.push_back(node->getStateValue());
    //     }
    //     visualization_->visualizePath(path, "map");
    // }
}