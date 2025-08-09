// Copyright 2025 Soheil E.nia

#include "motion_planning/utils/lie_kd_tree.hpp"
#include <queue>
#include <stdexcept>
#include <cmath>
#include <iostream>

// The internal structure for each node in the tree
struct LieNode {
    Eigen::VectorXd point;
    size_t index;
    LieNode* left = nullptr;
    LieNode* right = nullptr;
    Eigen::VectorXd split_normal;
    double split_value;
};

// --- Constructor / Destructor ---

LieSplittingKDTree::LieSplittingKDTree(int dimension, std::shared_ptr<StateSpace> space) : dimension_(dimension), space_(space) {
    if (dimension != 4) {
        // This implementation is specialized for the 4D DubinsTimeStateSpace.
        // It could be generalized by passing weights and constants in the constructor.
        throw std::invalid_argument("LieSplittingKDTree is currently implemented for 4 dimensions (x,y,theta,t).");
    }
}

LieSplittingKDTree::~LieSplittingKDTree() {
    clear();
}

void LieSplittingKDTree::clear() {
    delete_recursive(root_);
    root_ = nullptr;
    points_.clear();
}

void LieSplittingKDTree::delete_recursive(LieNode* node) {
    if (node) {
        delete_recursive(node->left);
        delete_recursive(node->right);
        delete node;
    }
}

size_t LieSplittingKDTree::size() const {
    return points_.size();
}

// --- Core Public Methods ---

void LieSplittingKDTree::addPoint(const Eigen::VectorXd& stateValue) {
    if (stateValue.size() != dimension_) {
        throw std::invalid_argument("Point dimension mismatches tree dimension.");
    }
    points_.push_back(stateValue);
    root_ = insert_recursive(root_, stateValue, 0);
}

std::vector<size_t> LieSplittingKDTree::knnSearch(const Eigen::VectorXd& query, int k) const {
    if (k <= 0) return {};

    // A max-heap to keep track of the k-nearest neighbors found so far.
    // Stores pairs of {geometric_distance^2, index}.
    std::priority_queue<std::pair<double, size_t>> pq;

    knn_search_recursive(root_, query, k, pq);

    // Extract indices from the priority queue
    std::vector<size_t> result;
    result.reserve(pq.size());
    while (!pq.empty()) {
        result.push_back(pq.top().second);
        pq.pop();
    }
    std::reverse(result.begin(), result.end()); // Sort from nearest to farthest
    return result;
}


// --- Recursive Helpers and Geometric Logic ---

LieNode* LieSplittingKDTree::insert_recursive(LieNode* current, const Eigen::VectorXd& point, int depth) {
    if (current == nullptr) {
        LieNode* newNode = new LieNode();
        newNode->point = point;
        newNode->index = points_.size() - 1;
        newNode->split_normal = getSplitNormal(depth, point);
        newNode->split_value = newNode->split_normal.dot(point);
        return newNode;
    }

    double side = point.dot(current->split_normal) - current->split_value;

    if (side < 0) {
        current->left = insert_recursive(current->left, point, depth + 1);
    } else {
        current->right = insert_recursive(current->right, point, depth + 1);
    }

    return current;
}

void LieSplittingKDTree::knn_search_recursive(LieNode* current, const Eigen::VectorXd& query, int k,
                                            std::priority_queue<std::pair<double, size_t>>& pq) const {
    if (current == nullptr) {
        return;
    }

    // Add current point to the priority queue
    // We need the geometric path length for the Ball-Box theorem.

    // Trajectory traj_geom = space->steer(query, current->point);
    Trajectory traj_geom = space_->steer(current->point, query); //IMPORTANT: WE NEED THE BACKWARD TRAJECTORY OR ELSE IT WILL BE INVALID TRAJ BECAUSE OF TIME CONSTRAINT (time needs to be forward from the robot to goal and goal is root or we can say "from" time is later than "to" time)
    if (traj_geom.is_valid) {
        double dist_sq = traj_geom.geometric_distance * traj_geom.geometric_distance;
        if (pq.size() < k || dist_sq < pq.top().first) {
            pq.push({dist_sq, current->index});
            if (pq.size() > k) {
                pq.pop();
            }
        }
    }


    // Decide which subtree to explore first
    double side = query.dot(current->split_normal) - current->split_value;
    LieNode* near_child = (side < 0) ? current->left : current->right;
    LieNode* far_child = (side < 0) ? current->right : current->left;

    knn_search_recursive(near_child, query, k, pq);

    // Check if the other subtree needs to be explored
    double radius = (pq.size() == k) ? std::sqrt(pq.top().first) : std::numeric_limits<double>::infinity();
    
    // This is the crucial pruning step from the paper 
    if (ballHyperplaneIntersection(query, radius, current->point, current->split_normal)) {
        knn_search_recursive(far_child, query, k, pq);
    }
}


/**
 * Gets the privileged directions for a Dubins car at a given state.
 * These are the local, configuration-dependent axes of motion.
 */
void LieSplittingKDTree::getPrivilegedAxes(const Eigen::VectorXd& state, std::vector<Eigen::VectorXd>& axes) const {
    axes.clear();
    axes.resize(4, Eigen::VectorXd::Zero(4));
    double theta = state[2];
    double cos_th = std::cos(theta);
    double sin_th = std::sin(theta);

    // f̂(x) - Forward direction (weight 1) 
    axes[0] << cos_th, sin_th, 0, 0;
    // l̂(x) - Lateral direction from Lie bracket [f̂, θ̂] (weight 2) 
    axes[1] << -sin_th, cos_th, 0, 0;
    // θ̂(x) - Rotational direction (weight 1) 
    axes[2] << 0, 0, 1, 0;
    // t̂ - Time direction (weight 1, standard cardinal axis)
    axes[3] << 0, 0, 0, 1;
}

/**
 * Implements the Lie Splitting Strategy.
 * Selects a splitting normal with frequency proportional to its weight. 
 */
Eigen::VectorXd LieSplittingKDTree::getSplitNormal(int depth, const Eigen::VectorXd& point) const {
    double theta = point[2];
    double cos_th = std::cos(theta);
    double sin_th = std::sin(theta);

    // Sequence respects weights {w_f=1, w_l=2, w_θ=1, w_t=1}. Total W=5.
    // Sequence: l, l, f, θ, t
    int mod = depth % 5;
    if (mod == 0 || mod == 1) { // Lateral direction, weight 2
        return Eigen::Vector4d(-sin_th, cos_th, 0, 0);
    } else if (mod == 2) { // Forward direction, weight 1
        return Eigen::Vector4d(cos_th, sin_th, 0, 0);
    } else if (mod == 3) { // Rotational direction, weight 1
        return Eigen::Vector4d(0, 0, 1, 0);
    } else { // (mod == 4) Time direction, weight 1
        return Eigen::Vector4d(0, 0, 0, 1);
    }
}

/**
 * Checks intersection using the "Outer Box Bound" method. 
 * Approximates the sub-Riemannian ball with a tight bounding box and checks
 * if that box intersects the given hyperplane. 
 */
bool LieSplittingKDTree::ballHyperplaneIntersection(const Eigen::VectorXd& query, double radius,
                                                  const Eigen::VectorXd& plane_point, const Eigen::VectorXd& plane_normal) const {
    if (radius == std::numeric_limits<double>::infinity()) {
        return true;
    }
    
    // Get the local privileged directions at the query point
    std::vector<Eigen::VectorXd> axes;
    getPrivilegedAxes(query, axes);

    // Calculate the half-lengths of the outer bounding box sides.
    // The length scales as R^(w_i) according to the Ball-Box Theorem.
    std::vector<Eigen::VectorXd> half_lengths;
    for (size_t i = 0; i < axes.size(); ++i) {
        half_lengths.push_back(axes[i] * C_constants_[i] * std::pow(radius, weights_[i]));
    }
    
    // Check if all vertices of the box lie on one side of the hyperplane.
    // There are 2^4 = 16 vertices for a 4D box.
    int first_sign = 0;
    for (int i = 0; i < 16; ++i) {
        Eigen::VectorXd vertex = query;
        // Construct the vertex by adding/subtracting half-lengths
        if ((i & 1) != 0) vertex += half_lengths[0]; else vertex -= half_lengths[0];
        if ((i & 2) != 0) vertex += half_lengths[1]; else vertex -= half_lengths[1];
        if ((i & 4) != 0) vertex += half_lengths[2]; else vertex -= half_lengths[2];
        if ((i & 8) != 0) vertex += half_lengths[3]; else vertex -= half_lengths[3];

        double side = (vertex - plane_point).dot(plane_normal);
        int current_sign = (side > 1e-9) ? 1 : ((side < -1e-9) ? -1 : 0);

        if (current_sign == 0) continue; // Vertex is on the plane, so intersection exists

        if (first_sign == 0) {
            first_sign = current_sign;
        } else if (current_sign != first_sign) {
            return true; // Vertices are on opposite sides, so intersection exists
        }
    }

    return false; // All vertices are on the same side
}


// --- Radius Search Implementation ---

std::vector<size_t> LieSplittingKDTree::radiusSearch(const Eigen::VectorXd& query, double radius) const {
    std::vector<size_t> results;
    if (radius > 0) {
        radius_search_recursive(root_, query, radius, results);
    }
    return results;
}

void LieSplittingKDTree::radius_search_recursive(LieNode* current, const Eigen::VectorXd& query, double radius, std::vector<size_t>& results) const {
    if (current == nullptr) {
        return;
    }

    // Check if the current node is within the radius
    // NOTE: We use the base DubinsStateSpace to get the pure geometric distance,
    // which corresponds to the radius 'r' of the sub-Riemannian ball B(p,r).

    // Trajectory traj = space_->steer(query, current->point);
    Trajectory traj = space_->steer(current->point, query);
    if (traj.is_valid && traj.geometric_distance <= radius) {
        results.push_back(current->index);
    }

    // Determine which subtree to search first
    double side = query.dot(current->split_normal) - current->split_value;
    LieNode* near_child = (side < 0) ? current->left : current->right;
    LieNode* far_child = (side < 0) ? current->right : current->left;

    // Recurse into the "near" side
    radius_search_recursive(near_child, query, radius, results);

    // Prune the "far" side using the paper's core concept 
    if (ballHyperplaneIntersection(query, radius, current->point, current->split_normal)) {
        radius_search_recursive(far_child, query, radius, results);
    }
}


// --- Dual Radius Search Implementation ---

std::pair<std::vector<size_t>, std::vector<size_t>> LieSplittingKDTree::radiusSearchDual(const Eigen::VectorXd& query, double radius1, double radius2) const {
    std::vector<size_t> results1, results2;
    if (radius1 <= 0) return {};
    if (radius2 > radius1) {
        // To work correctly, the pruning radius (radius1) must be the larger one.
        throw std::invalid_argument("In radiusSearchDual, radius1 must be >= radius2.");
    }
    
    radius_search_dual_recursive(root_, query, radius1, radius2, results1, results2);
    
    return {results1, results2};
}


void LieSplittingKDTree::radius_search_dual_recursive(LieNode* current, const Eigen::VectorXd& query, double r1, double r2, std::vector<size_t>& results1, std::vector<size_t>& results2) const {
    if (current == nullptr) {
        return;
    }

    // Check the current node against both radii

    // Trajectory traj = space_->steer(query, current->point);
    Trajectory traj = space_->steer(current->point, query);
    if (traj.is_valid && traj.geometric_distance <= r1) {
        results1.push_back(current->index);
        // If it's within the larger radius, check against the smaller one
        if (traj.geometric_distance <= r2) {
            results2.push_back(current->index);
        }
    }

    // Branching logic is identical to single radius search
    double side = query.dot(current->split_normal) - current->split_value;
    LieNode* near_child = (side < 0) ? current->left : current->right;
    LieNode* far_child = (side < 0) ? current->right : current->left;
    
    radius_search_dual_recursive(near_child, query, r1, r2, results1, results2);

    // Pruning is done using the LARGER radius (r1)
    if (ballHyperplaneIntersection(query, r1, current->point, current->split_normal)) {
        radius_search_dual_recursive(far_child, query, r1, r2, results1, results2);
    }
}

// USELESS FOR NOW

void LieSplittingKDTree::addPoints(const std::vector<Eigen::VectorXd>& statesValues) {}
void LieSplittingKDTree::addPoints(const Eigen::MatrixXd& states) {
    for(int i = 0 ; i <states.rows() ; i++)
    {
        addPoint(states.row(i));
    }
}
void LieSplittingKDTree::buildTree() {}
void LieSplittingKDTree::printData() const {}
bool LieSplittingKDTree::removePoint(const Eigen::VectorXd& query) {}
bool LieSplittingKDTree::removeByIndex(size_t index) {}
bool LieSplittingKDTree::validateAgainstSamples(const std::vector<std::shared_ptr<IFMTNode>>& samples) const {}
bool LieSplittingKDTree::validateAgainstSamples(const std::vector<std::shared_ptr<BITNode>>& samples) const {}