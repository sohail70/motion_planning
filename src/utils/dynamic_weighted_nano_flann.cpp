// Copyright 2025 Soheil E.nia
#include "motion_planning/utils/dynamic_weighted_nano_flann.hpp"
#include <stdexcept>
#include <set>
#include <map>

namespace {
    double normalizeAngle(double angle) {
        angle = fmod(angle, 2.0 * M_PI);
        if (angle > M_PI) angle -= 2.0 * M_PI;
        else if (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
}



DynamicWeightedNanoFlann::DynamicWeightedNanoFlann(int dimension, 
                                                   const Eigen::VectorXd& weights,
                                                   const std::vector<int>& wrap_dims,
                                                   const std::vector<double>& wrap_periods)
    : dimension_(dimension),
      weights_(weights),
      wrap_dims_(wrap_dims),
      wrap_periods_(wrap_periods),
      scaled_data_(0, dimension),
      unscaled_data_(0, dimension) {

    if (weights.size() != dimension) {
        throw std::invalid_argument("DynamicWeightedNanoFlann: Weights vector size must match dimension.");
    }
    if (wrap_dims.size() != wrap_periods.size()) {
        throw std::invalid_argument("DynamicWeightedNanoFlann: wrap_dims and wrap_periods must have the same size.");
    }

    // Initialize adaptor with the scaled data matrix
    adaptor_ = std::make_unique<EigenMatrixAdaptor>(scaled_data_);
    
    // Initialize the Dynamic Tree
    // Arguments: dimensionality, adaptor reference, max leaf (10)
    dynamicTree_ = std::make_unique<DynamicKDTreeType>(dimension, *adaptor_, 10);
}

void DynamicWeightedNanoFlann::addPoint(const Eigen::VectorXd& stateValue) {
    if (stateValue.size() != dimension_) {
        throw std::invalid_argument("DynamicWeightedNanoFlann::addPoint: Point has the wrong dimension.");
    }

    // 1. Store Unscaled
    size_t new_index = unscaled_data_.rows();
    unscaled_data_.conservativeResize(new_index + 1, Eigen::NoChange);
    unscaled_data_.row(new_index) = stateValue;

    // 2. Calculate and Store Scaled
    Eigen::VectorXd scaled_point = stateValue.cwiseProduct(weights_);
    scaled_data_.conservativeResize(new_index + 1, Eigen::NoChange);
    scaled_data_.row(new_index) = scaled_point;

    // 3. Add to Dynamic Tree (O(log N))
    dynamicTree_->addPoints(new_index, new_index);
}

void DynamicWeightedNanoFlann::addPoints(const std::vector<Eigen::VectorXd>& statesValues) {
    if (statesValues.empty()) return;
    
    size_t old_rows = unscaled_data_.rows();
    size_t num_new = statesValues.size();
    
    // Resize matrices
    unscaled_data_.conservativeResize(old_rows + num_new, Eigen::NoChange);
    scaled_data_.conservativeResize(old_rows + num_new, Eigen::NoChange);
    
    for (size_t i = 0; i < num_new; ++i) {
        if (statesValues[i].size() != dimension_) {
             throw std::invalid_argument("DynamicWeightedNanoFlann::addPoints(vector): A point has the wrong dimension.");
        }
        size_t current_index = old_rows + i;
        
        unscaled_data_.row(current_index) = statesValues[i];
        scaled_data_.row(current_index) = statesValues[i].cwiseProduct(weights_);
        
        dynamicTree_->addPoints(current_index, current_index);
    }
}

void DynamicWeightedNanoFlann::addPoints(const Eigen::MatrixXd& states) {
    if (states.cols() != dimension_) {
        throw std::invalid_argument("DynamicWeightedNanoFlann::addPoints(matrix): Matrix has the wrong number of columns.");
    }
    
    size_t old_rows = unscaled_data_.rows();
    size_t num_new = states.rows();
    
    unscaled_data_.conservativeResize(old_rows + num_new, Eigen::NoChange);
    unscaled_data_.bottomRows(num_new) = states;
    
    scaled_data_.conservativeResize(old_rows + num_new, Eigen::NoChange);
    // Efficiently scale the new block
    scaled_data_.bottomRows(num_new) = states.array().rowwise() * weights_.transpose().array();
    
    for (size_t i = 0; i < num_new; ++i) {
        dynamicTree_->addPoints(old_rows + i, old_rows + i);
    }
}

void DynamicWeightedNanoFlann::buildTree() {
    // No-op for dynamic tree. It is already built/updated during addPoint.
}

void DynamicWeightedNanoFlann::clear() {
    unscaled_data_.resize(0, dimension_);
    scaled_data_.resize(0, dimension_);
    
    // Reset the dynamic tree by creating a new one
    dynamicTree_ = std::make_unique<DynamicKDTreeType>(dimension_, *adaptor_, 10);
}

bool DynamicWeightedNanoFlann::removeByIndex(size_t index) {
    if (index >= static_cast<size_t>(unscaled_data_.rows())) return false;
    
    // 1. Remove from Dynamic Tree
    dynamicTree_->removePoint(index);
    
    // 2. Remove from Data Stores
    // Note: We perform a swap-and-pop to keep the matrix compact.
    // However, this changes the index of the last element!
    // The Dynamic Adaptor's removePoint just marks the point as deleted internally.
    // If we swap-pop in our matrix, the tree's internal index mapping becomes invalid 
    // for the moved point unless we update the tree.
    // Since updating the tree for a swap is complex with nanoflann, 
    // the safest "Dynamic" way is to just mark it removed in the tree and leave the data 
    // (or perform a full rebuild if strictly necessary).
    
    // For this implementation, we will just remove from tree and leave data in matrix 
    // to avoid index invalidation, as rebuilding is O(N).
    // If you need to reclaim memory, you must call buildTree() or a specific rebuild method.
    
    return true;
}

bool DynamicWeightedNanoFlann::removePoint(const Eigen::VectorXd& query) {
    // Find nearest neighbor
    std::vector<size_t> neighbors = knnSearch(query, 1);
    if (neighbors.empty()) return false;
    
    // Check if it's actually the same point (optional, depends on tolerance)
    // For now, just remove the nearest one.
    return removeByIndex(neighbors[0]);
}

Eigen::VectorXd DynamicWeightedNanoFlann::getUnscaledPoint(size_t index) const {
    if (index >= static_cast<size_t>(unscaled_data_.rows())) {
        throw std::out_of_range("getUnscaledPoint: Index out of range.");
    }
    return unscaled_data_.row(index);
}

std::vector<size_t> DynamicWeightedNanoFlann::knnSearch(const Eigen::VectorXd& query, int k) const {
    if (query.size() != dimension_) {
        throw std::invalid_argument("DynamicWeightedNanoFlann::knnSearch: Query has the wrong dimension.");
    }
    
    std::map<double, size_t> k_best_map;
    
    // Lambda to handle the actual query against the dynamic tree
    auto query_and_update = [&](const Eigen::VectorXd& current_query) {
        // FIX: Store scaled query in a variable to access .data()
        Eigen::VectorXd scaled_query = current_query.cwiseProduct(weights_);
        
        std::vector<size_t> candidate_indices(k); // RESIZE to k
        std::vector<double> out_dists_sqr(k);
        nanoflann::KNNResultSet<double> resultSet(k);
        
        resultSet.init(&candidate_indices[0], &out_dists_sqr[0]);
        
        // Perform search on dynamic tree using scaled_query.data()
        dynamicTree_->findNeighbors(resultSet, scaled_query.data(), nanoflann::SearchParameters(10));
        
        // Resize result set to actual found size
        candidate_indices.resize(resultSet.size());
        
        for (size_t i = 0; i < resultSet.size(); ++i) {
            Eigen::VectorXd unscaled_point = getUnscaledPoint(candidate_indices[i]);
            Eigen::VectorXd diff = unscaled_point - query;
            
            // Handle wrapping
            for (size_t j = 0; j < wrap_dims_.size(); ++j) {
                diff(wrap_dims_[j]) = normalizeAngle(diff(wrap_dims_[j]));
            }
            
            double true_dist_sq = diff.cwiseProduct(weights_).squaredNorm();
            
            if (k_best_map.size() < k) {
                k_best_map[true_dist_sq] = candidate_indices[i];
            } else if (true_dist_sq < k_best_map.rbegin()->first) {
                k_best_map.erase(std::prev(k_best_map.end()));
                k_best_map[true_dist_sq] = candidate_indices[i];
            }
        }
    };

    // Execute query for original and ghost points
    query_and_update(query);
    for (size_t i = 0; i < wrap_dims_.size(); ++i) {
        Eigen::VectorXd ghost_query = query;
        ghost_query[wrap_dims_[i]] += wrap_periods_[i];
        query_and_update(ghost_query);
        ghost_query[wrap_dims_[i]] -= 2 * wrap_periods_[i];
        query_and_update(ghost_query);
    }

    std::vector<size_t> final_indices;
    for (const auto& pair : k_best_map) {
        final_indices.push_back(pair.second);
    }
    return final_indices;
}

std::vector<size_t> DynamicWeightedNanoFlann::radiusSearch(const Eigen::VectorXd& query, double radius) const {
    if (query.size() != dimension_) {
        throw std::invalid_argument("DynamicWeightedNanoFlann::radiusSearch: Query has the wrong dimension.");
    }
    
    std::set<size_t> unique_indices;
    
    auto query_and_add = [&](const Eigen::VectorXd& current_query) {
        // FIX: Store scaled query in a variable
        Eigen::VectorXd scaled_query = current_query.cwiseProduct(weights_);
        
        // FIX: Use nanoflann::ResultItem instead of std::pair
        std::vector<nanoflann::ResultItem<size_t, double>> ret_matches;
        nanoflann::RadiusResultSet<double, size_t> resultSet(radius * radius, ret_matches);
        
        dynamicTree_->findNeighbors(resultSet, scaled_query.data(), nanoflann::SearchParameters());
        
        for (const auto& match : ret_matches) {
            Eigen::VectorXd unscaled_point = getUnscaledPoint(match.first);
            Eigen::VectorXd diff = unscaled_point - query;
            
            for (size_t j = 0; j < wrap_dims_.size(); ++j) {
                 diff(wrap_dims_[j]) = normalizeAngle(diff(wrap_dims_[j]));
            }
            
            if (diff.cwiseProduct(weights_).norm() <= radius) {
                unique_indices.insert(match.first);
            }
        }
    };
    
    query_and_add(query);
    for (size_t i = 0; i < wrap_dims_.size(); ++i) {
        Eigen::VectorXd ghost_query = query;
        ghost_query[wrap_dims_[i]] += wrap_periods_[i];
        query_and_add(ghost_query);
        ghost_query[wrap_dims_[i]] -= 2 * wrap_periods_[i];
        query_and_add(ghost_query);
    }
    
    return std::vector<size_t>(unique_indices.begin(), unique_indices.end());
}

std::pair<std::vector<size_t>, std::vector<size_t>> DynamicWeightedNanoFlann::radiusSearchDual(
    const Eigen::VectorXd& query, double radius1, double radius2) const {
    
    // Simple implementation: call radiusSearch twice or implement logic similar to radiusSearch
    // For efficiency, you might want to merge the logic, but calling twice is safer for correctness here.
    std::vector<size_t> r1 = radiusSearch(query, radius1);
    std::vector<size_t> r2;
    
    // Filter r1 for r2
    for (size_t idx : r1) {
        Eigen::VectorXd unscaled_point = getUnscaledPoint(idx);
        Eigen::VectorXd diff = unscaled_point - query;
        for (size_t j = 0; j < wrap_dims_.size(); ++j) {
             diff(wrap_dims_[j]) = normalizeAngle(diff(wrap_dims_[j]));
        }
        if (diff.cwiseProduct(weights_).norm() <= radius2) {
            r2.push_back(idx);
        }
    }
    return {r1, r2};
}

void DynamicWeightedNanoFlann::printData() const {
    std::cout << "Dynamic KD-Tree Unscaled Data:\n";
    std::cout << unscaled_data_ << std::endl;
}

size_t DynamicWeightedNanoFlann::size() const {
    return unscaled_data_.rows();
}