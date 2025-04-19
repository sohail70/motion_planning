// Copyright 2025 Soheil E.nia
#include "motion_planning/planners/geometric/bit_star.hpp"

BITStar::BITStar(std::shared_ptr<StateSpace> statespace,
                               std::shared_ptr<ProblemDefinition> problem_def,
                               std::shared_ptr<ObstacleChecker> obs_checker)
    : statespace_(statespace),
      problem_(problem_def),
      obs_checker_(obs_checker) {
    std::cout << "ANY FMT Constructor\n";
}

void BITStar::clearPlannerState() {
    for (auto& node : tree_) {
        node->disconnectFromGraph();
    }
    tree_.clear();
    statespace_->reset();
    kdtree_tree_.reset();
    kdtree_samples_.reset();
    vertex_queue_.clear();
    samples_.clear();
    vsol_.clear();
    unconnected_.clear();
    unexpanded_.clear();
    unprocessed_nodes_.clear();
    unconnected_nodes_container_.clear();
    root_state_index_ = -1;
    robot_state_index_ = -1;
    obstacle_check_cache.clear();
    collision_check_ = 0;
}

void BITStar::setup(const Params& params, std::shared_ptr<Visualization> visualization) {
    auto start = std::chrono::high_resolution_clock::now();
    clearPlannerState();
    visualization_ = visualization;
    
    // Parameter initialization
    num_of_samples_ = params.getParam<int>("num_of_samples");
    num_batch_ = params.getParam<int>("num_batch");
    partial_plot = params.getParam<bool>("partial_plot");
    obs_cache = params.getParam<bool>("obs_cache");
    lower_bound_ = problem_->getLowerBound();
    upper_bound_ = problem_->getUpperBound();
    use_kdtree = params.getParam<bool>("use_kdtree");
    
    std::string kdtree_type = params.getParam<std::string>("kdtree_type");
    if (use_kdtree && kdtree_type == "NanoFlann") {
        kdtree_samples_ = std::make_shared<NanoFlann>(statespace_->getDimension());
        kdtree_tree_ = std::make_shared<NanoFlann>(statespace_->getDimension());
    } else {
        throw std::runtime_error("Unknown KD-Tree type");
    }

    std::cout << "num_of_samples=" << num_of_samples_
              << ", bounds=[" << lower_bound_ << ", " << upper_bound_ << "]\n";

    // Initialize start and goal
    /*
        I set robot node in the setGoal and heuristic for start node is being calculated in setStart and needs the robot_node_ so i do setGoal first!
    */
    setGoal(problem_->getGoal()); 
    setStart(problem_->getStart());

    // // Build KD-Tree if needed
    // if (use_kdtree) {
    //     kdtree_tree_->addPoints(statespace_->getSamplesCopy());
    //     kdtree_tree_->buildTree();
    // }

    // Initialize algorithm state

    // unconnected_.push_back(robot_node_);
    // unexpanded_.push_back(tree_.at(0));

    ci_ = std::numeric_limits<double>::infinity();

    // Calculate neighborhood radius
    d = statespace_->getDimension();
    mu = std::pow(problem_->getUpperBound() - problem_->getLowerBound(), 2);
    zetaD = std::pow(M_PI, d / 2.0) / std::tgamma((d / 2.0) + 1);
    gamma = 2 * std::pow(1 + 1.0 / d, 1.0 / d) * std::pow(mu / zetaD, 1.0 / d);
    factor = 1.0;
    neighborhood_radius_ = factor * gamma * 
        std::pow(std::log(statespace_->getNumStates()) / statespace_->getNumStates(), 1.0 / d);



    max_edge_length_ = 15;
    neighborhood_radius_ = std::min(neighborhood_radius_,max_edge_length_);

    
    std::cout << "Computed value of rn: " << neighborhood_radius_ << std::endl;
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by setup: " << duration.count() << " milliseconds\n";
    std::cout << "---\n";
}


void BITStar::plan() {
    if(std::abs(robot_node_->getCost() - tree_[root_state_index_]->getHeuristic()) <0.01){
        return;
    }


    if (vertex_queue_.empty() && edge_queue_.empty()) {
        std::cout<<"GGGGGGGGGGGGGGGGGGGGGGGGGGGG \n";
        for (auto s : samples_){
            s->is_new_ = false;
        }
        prune(); 
        pruneSamples();


        // samples_.clear(); // Clearing samples would violate uniform sampling assumption! // also messes up with your kdtree_samples_ indices consistency with samples_ vector!
        if (robot_node_->getCost() == INFINITY) {
            addBatchOfSamplesUninformed(num_batch_);
        } else {
            addBatchOfSamples(num_batch_);
        }


        // Add all tree nodes to the vertex queue
        for (auto& node : tree_) {
            double priority = node->getCost() + node->getHeuristic();
            // vertex_queue_.add(node, priority);
            // vertex_queue_.emplace(priority, node);


            vertex_queue_.add(node, priority);
            node->in_queue_ = true;
        }
    }
    std::cout<<"tree: "<<tree_.size()<<"\n";
    std::cout<<"samples: "<<samples_.size()<<"\n";
    std::cout<<"cost: "<<robot_node_->getCost()<<"\n";
    std::cout<<"r: "<<neighborhood_radius_<<"\n";
    std::cout<<"collision check: "<<collision_check_<<"\n";
    // I think the paper meant a do-while because at time where edge_queue is empty we need to at least do it once!
    while (!vertex_queue_.empty() && (edge_queue_.empty() || vertex_queue_.top().first < edge_queue_.top().estimated_cost)){
        auto [current_cost, vmin] = vertex_queue_.top();
        vertex_queue_.pop();
        vmin->in_queue_ = false;

       

        std::vector<std::shared_ptr<BITNode>> x_near;

        // Always get nearby samples (no special case for new vs old vertices)
        near2sample(vmin, x_near);

        // Process sample neighbors
        for (auto& neighbor : x_near) {
            // // Skip processing if vmin is not unexpand AND neighbor is not new --> because we checked that edge before and its redundant to check again
            if (!vmin->unexpand_ && !neighbor->is_new_) {
                // neighbor->is_new_ = false;
                continue;
            }

            double edge_cost = (vmin->getStateValue() - neighbor->getStateValue()).norm();
            double a_hat = vmin->getGHat()+ edge_cost + neighbor->getHeuristic();
            
            if (a_hat < ci_) {
                double cost = vmin->getCost() + edge_cost + neighbor->getHeuristic();
                edge_queue_.emplace(a_hat, vmin, neighbor);
                // edge_queue_.emplace(cost, vmin, neighbor);
            }

            // neighbor->is_new_ = false; // for the whole process of this while search i think neighbor can be is_new_ true and only when we get out of this while loop we can set it to is_new_=false because according to the paper Xnew is for the whole duration of processing this new batch!
        }

        // Additional processing for new vertices
        if (vmin->unexpand_) {
            std::vector<std::shared_ptr<BITNode>> v_near;
            near2tree(vmin, v_near);
            
            for (auto& v : v_near) {
                if (v == vmin) continue;  // Skip self
                if(v->getParent() == vmin) continue; // Skip this edge
                
                double edge_cost = (vmin->getStateValue() - v->getStateValue()).norm();
                double a_hat = vmin->getGHat() + edge_cost + v->getHeuristic();
                double path_cost = vmin->getCost() + edge_cost;
                if (a_hat < ci_ && path_cost < v->getCost()) {
                    edge_queue_.emplace(a_hat, vmin, v);
                    // double cost = vmin->getCost() + edge_cost + v->getHeuristic();
                    // edge_queue_.emplace(cost, vmin, v);
                }
            }
            
            // Mark vertex as processed
            vmin->unexpand_ = false;

        }




    }



    if (!edge_queue_.empty()) {
        EdgeCandidate best_edge = edge_queue_.top();
        edge_queue_.pop();

        auto vmin = best_edge.from;
        auto xmin = best_edge.to;
        double c_hat = (vmin->getStateValue() - xmin->getStateValue()).norm();

        if (vmin->getCost() + c_hat + xmin->getHeuristic() < ci_ ){
            if( vmin->getCost() + c_hat < xmin->getCost()){
                
                collision_check_++;
                double edge_cost = obs_checker_->isObstacleFree(vmin->getStateValue(), xmin->getStateValue()) 
                                ? c_hat : std::numeric_limits<double>::infinity();

                if (vmin->getCost() + edge_cost + xmin->getHeuristic() < ci_ &&
                    vmin->getCost() + edge_cost < xmin->getCost()) {
                    // std::cout<<"prev cost : "<<xmin->getCost() << "current : "<<vmin->getCost() + edge_cost<<"\n";
                    xmin->setParent(vmin, edge_cost);
                    xmin->updateCostAndPropagate();
                    // xmin->setIndex(tree_.size()); // Even though i guess when we are gonna prune every thing will be a mess index wise!
                    if (xmin->in_samples_) {
                        // Remove from samples and update KD-tree
                        const size_t idx = xmin->samples_index_;
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
                        
                        if (samples_.size() != kdtree_samples_->size()) {
                            std::cout<< "IIIIIIIIIINNNN 2nd phase WRONG SIZE" << samples_.size() << " vs "<< kdtree_samples_->size()<<"\n";
                        }
                        if(!kdtree_samples_->validateAgainstSamples(samples_)){
                            std::cout<<"why \n";
                        }
                        xmin->in_samples_ = false;
                        xmin->samples_index_ = -1;
                        kdtree_samples_->buildTree();
                        xmin->is_new_ = false; //since we delete xmin from samples_ we do not have access to it any more in looping thourhg the samples_ again!
                    xmin->unexpand_ = true;

                    }
                    else{
                        // std::cout<<"Not  in_samples \n";
                    }


                    tree_.push_back(xmin);
                    kdtree_tree_->addPoint(xmin->getStateValue());
                    kdtree_tree_->buildTree();

                    double priority =  xmin->getCost() + xmin->getHeuristic();
                    if (xmin->in_queue_) {
                        vertex_queue_.update(xmin, priority);
                        // std::cout<<"UPDATE \n";
                    } else {
                        vertex_queue_.add(xmin, priority);
                        xmin->in_queue_ = true;
                    }



                    if ((xmin->getStateValue() - robot_node_->getStateValue()).norm() < 0.5) {
                        vsol_.push_back(xmin);
                    }

                    double min_vsol_cost = std::numeric_limits<double>::infinity();
                    for (const auto& node : vsol_) {
                        min_vsol_cost = std::min(min_vsol_cost, node->getCost());
                    }
                    ci_ = min_vsol_cost;
                }
            }
        }else{
            // std::cout<<"CLEAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAR \n";
            vertex_queue_.clear();
            edge_queue_ = std::priority_queue<EdgeCandidate, std::vector<EdgeCandidate>, 
                                            std::greater<EdgeCandidate>>();
        }
    }
}

// std::vector<std::shared_ptr<BITNode>> BITStar::near2sample( const std::vector<std::shared_ptr<BITNode>>& search_set, const std::shared_ptr<BITNode>& node) {
//     std::vector<std::shared_ptr<BITNode>> near_nodes;
//     near_nodes.reserve(search_set.size() / 4);

//     for (const auto& n : search_set) {
//         if (n == node) continue;
//         double dist = (n->getStateValue() - node->getStateValue()).norm();
//         if (dist <= neighborhood_radius_) {
//             near_nodes.push_back(n);
//         }
//     }
//     return near_nodes;
// }

// std::vector<std::shared_ptr<BITNode>> BITStar::near2tree( const std::vector<std::shared_ptr<BITNode>>& search_set, const std::shared_ptr<BITNode>& node) {
//     std::vector<std::shared_ptr<BITNode>> near_nodes;
//     near_nodes.reserve(search_set.size() / 4);

//     for (const auto& n : search_set) {
//         if (n == node) continue;
//         double dist = (n->getStateValue() - node->getStateValue()).norm();
//         if (dist <= neighborhood_radius_) {
//             near_nodes.push_back(n);
//         }
//     }
//     return near_nodes;
// }


void BITStar::near2sample( const std::shared_ptr<BITNode>& node, std::vector<std::shared_ptr<BITNode>>& near_nodes) {
    near_nodes.clear();  // Reset output vector
    
    // Perform radius search using kdtree_samples_
    std::vector<size_t> neighbor_indices = 
        kdtree_samples_->radiusSearch(node->getStateValue(), neighborhood_radius_);
    
    // Fill results in-place
    near_nodes.reserve(neighbor_indices.size());
    for (size_t idx : neighbor_indices) {
        auto neighbor = samples_.at(idx);
        if (neighbor != node) {  // Exclude the query node itself
            near_nodes.push_back(neighbor);
        }
    }
}

void BITStar::near2tree( const std::shared_ptr<BITNode>& node, std::vector<std::shared_ptr<BITNode>>& near_nodes) {
    near_nodes.clear();  // Reset output vector
    
    // Perform radius search using kdtree_tree_
    std::vector<size_t> neighbor_indices = 
        kdtree_tree_->radiusSearch(node->getStateValue(), neighborhood_radius_);
    
    // Fill results in-place
    near_nodes.reserve(neighbor_indices.size());
    for (size_t idx : neighbor_indices) {
        auto neighbor = tree_.at(idx);
        if (neighbor != node) {  // Exclude the query node itself
            near_nodes.push_back(neighbor);
        }
    }
}

void BITStar::near(int node_index) {
    // auto node = tree_[node_index];
    // auto neighbors = kdtree_->radiusSearch(node->getStateValue(), neighborhood_radius_);
    
    // for (int idx : neighbors) {
    //     if (idx == node->getIndex()) continue;
        
    //     auto neighbor = tree_[idx];
    //     double edge_cost = (node->getStateValue() - neighbor->getStateValue()).norm();
    //     double estimated_total = node->getCost() + edge_cost + neighbor->getHeuristic();
        
    //     if (estimated_total < ci_) {
    //         edge_queue_.emplace(estimated_total, node, neighbor);
    //     }
    // }
}

double BITStar::heuristic(int current_index) {
    auto current_position = tree_.at(current_index)->getStateValue();
    auto goal_position = tree_.at(robot_state_index_)->getStateValue();
    return (goal_position - current_position).norm();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
void BITStar::addBatchOfSamplesUninformed(int num_samples) {
    if (num_samples == 0) return;
    int counter = 0;
    for (int i = 0; i < num_batch_; ++i) {
        Eigen::VectorXd sample = Eigen::VectorXd::Random(d);
        sample = lower_bound_ + (upper_bound_ - lower_bound_) * (sample.array() + 1) / 2;
        if (!obs_checker_->isObstacleFree(sample)) continue;
        counter++;
        auto node = std::make_shared<BITNode>(statespace_->addState(sample), -1);// later use setIndex and tree_.size() because right now its not being added to the tree_ (tau)
        node->cacheHeuristic((sample - robot_node_->getStateValue()).norm());
        node->cacheGHat((sample - tree_.at(root_state_index_)->getStateValue()).norm());
        node->in_samples_ = true;
        node->is_new_ = true;
        node->samples_index_ = samples_.size();
        samples_.push_back(node);
        // std::cout<<"sample: "<<sample<<"\n";
        kdtree_samples_->addPoint(sample);
    }
    if(counter==0) return;

    kdtree_samples_->buildTree();

    // double n = tree_.size() + unconnected_.size(); - counter;  // Total vertices
    double n = tree_.size() + samples_.size();

    neighborhood_radius_ = factor * gamma * std::pow(std::log(n) / n, 1.0 / d);
    neighborhood_radius_ = std::min(neighborhood_radius_,max_edge_length_);

}



void BITStar::addBatchOfSamples(int num_samples) {
    if (num_samples == 0) return;

    auto start_node = robot_node_;
    auto goal_node = tree_[root_state_index_];
    Eigen::VectorXd start_pos = start_node->getStateValue();
    Eigen::VectorXd goal_pos = goal_node->getStateValue();

    Eigen::VectorXd d_vec = start_pos - goal_pos;
    double c_min = d_vec.norm();
    double c_best = ci_;
    std::cout<<"ci "<<ci_;
    Eigen::VectorXd center = (start_pos + goal_pos) / 2.0;
    Eigen::MatrixXd R = computeRotationMatrix(d_vec);
    double a = c_best / 2.0;
    // double b = std::sqrt(std::max(c_best * c_best - c_min * c_min, 0.0)) / 2.0;
    double b = (c_best > c_min) ? std::sqrt(c_best*c_best - c_min*c_min)/2.0 : 0.0;




    int counter = 0;
    for (int i = 0; i < num_samples; ++i) {
        Eigen::VectorXd sample = sampleInEllipsoid(center, R, a, b);
        if (!obs_checker_->isObstacleFree(sample)) continue;
        counter++;
        // nodes.push_back(sample);
        auto node = std::make_shared<BITNode>(statespace_->addState(sample), -1);
        node->cacheHeuristic((sample - robot_node_->getStateValue()).norm());
        node->cacheGHat((sample - tree_.at(root_state_index_)->getStateValue()).norm());
        node->in_samples_ = true;
        node->is_new_ = true;
        node->samples_index_ = samples_.size();
        samples_.push_back(node);
        // visualization_->visualizeNodes(nodes);
        // std::cout<<"sample: "<<sample<<"\n";


        kdtree_samples_->addPoint(sample);
    }
    if(counter==0) return;
    kdtree_samples_->buildTree();

    // double n = tree_.size() + unconnected_.size() - counter;  // Total vertices
    double n = tree_.size() + samples_.size();
    neighborhood_radius_ = factor * gamma * std::pow(std::log(n) / n, 1.0 / d);
    neighborhood_radius_ = std::min(neighborhood_radius_,max_edge_length_);


}

// void BITStar::addBatchOfSamples(int num_samples) {
//     if (num_samples == 0) return;

//     auto start_node = tree_[root_state_index_];  // Should be start node
//     auto goal_node = robot_node_;                // Should be goal node
//     Eigen::VectorXd start_pos = start_node->getStateValue();
//     Eigen::VectorXd goal_pos = goal_node->getStateValue();

//     Eigen::VectorXd d_vec = goal_pos - start_pos;
//     double c_min = d_vec.norm();
//     double c_best = ci_;
    
//     // Debug output
//     std::cout << "Sampling Ellipsoid - c_min: " << c_min << " c_best: " << c_best << std::endl;

//     // // Handle case where c_best <= c_min (should fall back to uninformed sampling) --> this is just for percision errors!
//     // if (c_best <= c_min + 1e-6) {  // Small epsilon for floating point comparison
//     //     addBatchOfSamplesUninformed(num_samples);
//     //     return;
//     // }

//     Eigen::VectorXd center = (start_pos + goal_pos) / 2.0;
//     Eigen::MatrixXd R = computeRotationMatrix(d_vec);
    
//     // Ellipsoid parameters
//     double a = c_best / 2.0;
//     double b = std::sqrt(c_best*c_best - c_min*c_min)/2.0;

//     int counter = 0;
//     for (int i = 0; i < num_samples; ++i) {
//         Eigen::VectorXd sample;
//         bool valid_sample = false;
//         int attempts = 0;
        
//         // Keep sampling until we get a valid point or exceed max attempts
//         // while (!valid_sample && attempts < 100) {
//             sample = sampleInEllipsoid(center, R, a, b);
//             valid_sample=true; 
//             // // Verify the sample actually falls within the ellipsoid
//             // Eigen::VectorXd centered = sample - center;
//             // Eigen::VectorXd rotated = R.transpose() * centered;
//             // double term1 = rotated[0]*rotated[0]/(a*a);
//             // double term2 = (rotated.tail(rotated.size()-1).squaredNorm())/(b*b);

//             // // for percision errors! 
//             // if (term1 + term2 <= 1.0) {
//             //     valid_sample = true;
//             // }
//             // else{
//             //     std::cout<<"Wronggggggggggggggggggggggggggg \n";
//             // }
//             // attempts++;
//         // }

//         if (!valid_sample || !obs_checker_->isObstacleFree(sample)) {
//             continue;
//         }

//         counter++;
//         auto node = std::make_shared<BITNode>(statespace_->addState(sample), -1);
//         node->cacheHeuristic((sample - goal_pos).norm());
//         node->cacheGHat((sample - start_pos).norm());
//         node->in_samples_ = true;
//         node->is_new_ = true;
//         node->samples_index_ = samples_.size();
//         samples_.push_back(node);
//         kdtree_samples_->addPoint(sample);
//     }

//     if (counter > 0) {
//         kdtree_samples_->buildTree();
//         double n = tree_.size() + samples_.size();
//         neighborhood_radius_ = std::min(
//             factor * gamma * std::pow(std::log(n)/n, 1.0/d),
//             max_edge_length_
//         );
//     }


// }




Eigen::MatrixXd BITStar::computeRotationMatrix(const Eigen::VectorXd& dir) {
    Eigen::VectorXd u = dir.normalized();
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dir.size(), dir.size());
    A.col(0) = u;
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(A);
    Eigen::MatrixXd Q = qr.householderQ();
    if (Q.col(0).dot(u) < 0) Q.col(0) *= -1;
    return Q;
}

Eigen::VectorXd BITStar::sampleInEllipsoid(const Eigen::VectorXd& center,
                                                 const Eigen::MatrixXd& R,
                                                 double a, double b) {
    Eigen::VectorXd y = sampleUnitBall(center.size());
    
    Eigen::VectorXd y_scaled(y.size());
    y_scaled[0] = a * y[0];
    for(int i = 1; i < y.size(); ++i) 
        y_scaled[i] = b * y[i];
    
    return R * y_scaled + center;
}


Eigen::VectorXd BITStar::sampleUnitBall(int dim) {
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


void BITStar::updateChildrenCosts(std::shared_ptr<BITNode> node) {
    for (auto& child : node->getChildrenMutable()) {
        double new_cost = node->getCost() + 
                         (child->getStateValue() - node->getStateValue()).norm();
        if (new_cost < child->getCost()) {
            child->setCost(new_cost);
            updateChildrenCosts(child);
        }
    }
}

void BITStar::prune() {
    // Sort tree nodes by cost (descending)
    std::vector<std::shared_ptr<BITNode>> sorted_nodes = tree_;
    std::sort(sorted_nodes.begin(), sorted_nodes.end(),
        [](const auto& a, const auto& b) { return a->getCost() > b->getCost(); });

    // Prune tree nodes and handle dependencies
    std::vector<std::shared_ptr<BITNode>> to_remove;
    for (auto& node : sorted_nodes) {
        // Skip start (first node) and goal (robot_node_)
        // if (node == tree_.front() || node == robot_node_) continue;

        double f_hat = node->getHeuristic() + (node->getStateValue() - tree_.at(root_state_index_)->getStateValue()).norm();
        if (f_hat > ci_ || node->getCost() + node->getHeuristic() > ci_) {
            // Remove from all containers
            to_remove.push_back(node);
            
            node->unexpand_ = false;
            // // Remove from parent's children
            // if (auto parent = node->getParent()) {
            //     auto& siblings = parent->getChildrenMutable();
            //     siblings.erase(std::remove(siblings.begin(), siblings.end(), node), siblings.end());
            // }

            // Add to reuse if meets criteria
            if (f_hat < ci_) {
                node->in_samples_ = true;
                node->is_new_= true;
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
            [&](const auto& n) { return std::find(to_remove.begin(), to_remove.end(), n) != to_remove.end(); }),
            container.end());
    };
    
    remove_from(tree_);
    remove_from(vsol_);
    // remove_from(unexpanded_);

    // Rebuild spatial index ---> later use remove instead of clearing all the data!
    kdtree_tree_->clear();
    for (const auto& node : tree_) {
        kdtree_tree_->addPoint(node->getStateValue());
    }
    kdtree_tree_->buildTree();



    // // Ensure goal is in unconnected
    // if (std::find(unconnected_.begin(), unconnected_.end(), robot_node_) == unconnected_.end()) {
    //     unconnected_.push_back(robot_node_);
    // }
}


void BITStar::pruneSamples() {
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



std::vector<std::shared_ptr<BITNode>> BITStar::getPathNodes() const {
    std::vector<std::shared_ptr<BITNode>> path_nodes;
    auto current = robot_node_;
    
    while (current) {
        path_nodes.push_back(current);
        current = current->getParent();
    }
    return path_nodes;
}


std::vector<Eigen::VectorXd> BITStar::getPathPositions() const {
    std::vector<Eigen::VectorXd> path_positions;
    auto current = robot_node_;
    
    while (current) {
        path_positions.push_back(current->getStateValue());
        current = current->getParent();
    }
    return path_positions;
}

void BITStar::setRobotIndex(const Eigen::VectorXd& robot_position) {
    robot_position_ = robot_position;
    const double MAX_SEARCH_RADIUS = 5.0;
    
    auto nearest_indices = kdtree_tree_->radiusSearch(robot_position, MAX_SEARCH_RADIUS);
    std::shared_ptr<BITNode> best_node;
    double min_total_cost = std::numeric_limits<double>::max();

    for (size_t index : nearest_indices) {
        auto node = tree_[index];
        if (node->getCost() == INFINITY) continue;

        double distance = (node->getStateValue() - robot_position).norm();
        double total_cost = distance + node->getCost();
        
        if (total_cost < min_total_cost) {
            min_total_cost = total_cost;
            best_node = node;
        }
    }

    if (best_node) {
        robot_node_ = best_node;
        robot_state_index_ = best_node->getIndex();
        return;
    }

    if (!robot_node_ && !tree_.empty()) {
        auto nearest = kdtree_tree_->knnSearch(robot_position, 1);
        if (!nearest.empty()) {
            robot_node_ = tree_[nearest[0]];
            robot_state_index_ = robot_node_->getIndex();
        }
    }
}

void BITStar::setStart(const Eigen::VectorXd& start) {
    auto node = std::make_shared<BITNode>(statespace_->addState(start), tree_.size());
    root_state_index_ = node->getIndex();
    node->setCost(0);
    double h = (node->getStateValue() - robot_node_->getStateValue()).norm();
    double g_hat = 0;
    node->cacheHeuristic(h);
    node->cacheGHat(g_hat);
    tree_.push_back(node);
    node->unexpand_=true;
    vertex_queue_.add(node,node->getCost()+node->getHeuristic());
    node->in_queue_ = true;
    kdtree_tree_->addPoint(node->getStateValue());
    kdtree_tree_->buildTree();
}

void BITStar::setGoal(const Eigen::VectorXd& goal) {
    robot_state_index_ = statespace_->getNumStates();
    auto node = std::make_shared<BITNode>(statespace_->addState(goal),-1);
    robot_node_ = node;
    samples_.push_back(node);
    node->in_samples_ = true;
    node->is_new_ = true;
    node->samples_index_ = 0;
    kdtree_samples_->addPoint(node->getStateValue());
    kdtree_samples_->buildTree();
}


std::vector<Eigen::VectorXd> BITStar::getSmoothedPathPositions(int num_intermediates, int smoothing_passes) const {
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


std::vector<Eigen::VectorXd> BITStar::interpolatePath(const std::vector<Eigen::VectorXd>& path, int num_intermediates) const {
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


std::vector<Eigen::VectorXd> BITStar::smoothPath(const std::vector<Eigen::VectorXd>& path, int window_size) const {
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

void BITStar::visualizePath(const std::vector<std::shared_ptr<BITNode>>& path_nodes) {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;

    for (const auto& node : path_nodes) {
        if (auto parent = node->getParent()) {
            edges.emplace_back(parent->getStateValue(), node->getStateValue());
        }
    }
    visualization_->visualizeEdges(edges, "map", "0.0,1.0,0.0");
}

void BITStar::visualizeTree() {
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
    visualization_->visualizeNodes(nodes_sample_,"map",std::vector<float>{0.0,0.0,1.0} , "nodes_sample");
    visualization_->visualizeEdges(edges, "map");
}


void BITStar::visualizeSmoothedPath(const std::vector<Eigen::VectorXd>& shortest_path_) {
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;

    // Add edges to the list
    for (size_t i = 1; i < shortest_path_.size(); ++i) {
        edges.emplace_back(shortest_path_[i - 1], shortest_path_[i]);
    }

    if (visualization_) {
        visualization_->visualizeEdges(edges, "map", "0.0,1.0,0.0");
    } 
}

void BITStar::visualizeHeapAndUnvisited() {
    std::vector<Eigen::VectorXd> vopen_positions;
    
    // Create temporary copy of the priority queue
    auto temp_queue = vertex_queue_;
    while (!temp_queue.empty()) {
        auto [priority, node] = temp_queue.top();
        temp_queue.pop();
        
        if (priority == INFINITY) {
            std::cerr << "Warning: Node " << node->getIndex() 
                      << " is in vertex_queue_ but has INF cost!\n";
        }

        vopen_positions.push_back(node->getStateValue());
    }
    
    visualization_->visualizeNodes(vopen_positions);
}

