#include "motion_planning/utils/weighted_nano_flann.hpp"
#include <iostream>
#include <set>
#include <map>
#include <stdexcept>

WeightedNanoFlann::WeightedNanoFlann(int dimension, const Eigen::VectorXd& weights,
                                     const std::vector<int>& wrap_dims,
                                     const std::vector<double>& wrap_periods)
    : dimension_(dimension),
      weights_(weights),
      rawTree_(dimension),
      wrap_dims_(wrap_dims),
      wrap_periods_(wrap_periods),
      unscaled_data_(0, dimension) {
    if (weights.size() != dimension) {
        throw std::invalid_argument("WeightedNanoFlann: Weights vector size must match dimension.");
    }
    if (wrap_dims.size() != wrap_periods.size()) {
        throw std::invalid_argument("WeightedNanoFlann: wrap_dims and wrap_periods must have the same size.");
    }
}

// --- Data Management Methods ---

void WeightedNanoFlann::addPoint(const Eigen::VectorXd& stateValue) {
    if (stateValue.size() != dimension_) {
        throw std::invalid_argument("addPoint: Point has the wrong dimension.");
    }
    // Add to our unscaled data store
    unscaled_data_.conservativeResize(unscaled_data_.rows() + 1, Eigen::NoChange);
    unscaled_data_.row(unscaled_data_.rows() - 1) = stateValue;
    // Add the scaled version to the raw internal tree
    rawTree_.addPoint(stateValue.cwiseProduct(weights_));
}

void WeightedNanoFlann::addPoints(const std::vector<Eigen::VectorXd>& statesValues) {
    if (statesValues.empty()) return;
    
    size_t old_rows = unscaled_data_.rows();
    unscaled_data_.conservativeResize(old_rows + statesValues.size(), Eigen::NoChange);
    
    Eigen::MatrixXd scaled_matrix(statesValues.size(), dimension_);
    for (size_t i = 0; i < statesValues.size(); ++i) {
        if (statesValues[i].size() != dimension_) {
             throw std::invalid_argument("addPoints(vector): A point has the wrong dimension.");
        }
        unscaled_data_.row(old_rows + i) = statesValues[i];
        scaled_matrix.row(i) = statesValues[i].cwiseProduct(weights_);
    }
    rawTree_.addPoints(scaled_matrix);
}

void WeightedNanoFlann::addPoints(const Eigen::MatrixXd& states) {
    if (states.cols() != dimension_) {
        throw std::invalid_argument("addPoints(matrix): Matrix has the wrong number of columns.");
    }
    size_t old_rows = unscaled_data_.rows();
    unscaled_data_.conservativeResize(old_rows + states.rows(), Eigen::NoChange);
    unscaled_data_.bottomRows(states.rows()) = states;

    Eigen::MatrixXd scaled_states = states;
    for (int i = 0; i < states.rows(); ++i) {
        scaled_states.row(i) = states.row(i).cwiseProduct(weights_.transpose());
    }
    rawTree_.addPoints(scaled_states);
    // std::cout<<"------------- \n";
    // std::cout<<unscaled_data_<<"\n";
    // std::cout<<"------------- \n";
    // std::cout<<scaled_states<<"\n";
}

// --- Core Search Logic with Ghost Points ---

std::vector<size_t> WeightedNanoFlann::knnSearch(const Eigen::VectorXd& query, int k) const {
    if (query.size() != dimension_) {
        throw std::invalid_argument("knnSearch: Query has the wrong dimension.");
    }

    std::map<double, size_t> k_best_map;

    auto query_and_update = [&](const Eigen::VectorXd& current_query) {
        std::vector<size_t> candidate_indices = rawTree_.knnSearch(current_query.cwiseProduct(weights_), k);
        for (size_t idx : candidate_indices) {
            Eigen::VectorXd unscaled_point = getUnscaledPoint(idx);
            Eigen::VectorXd diff = unscaled_point - query;
            for (size_t i = 0; i < wrap_dims_.size(); ++i) {
                diff(wrap_dims_[i]) = normalizeAngle(diff(wrap_dims_[i]));
            }
            double true_dist_sq = diff.cwiseProduct(weights_).squaredNorm();

            if (k_best_map.size() < k) {
                k_best_map[true_dist_sq] = idx;
            } else if (true_dist_sq < k_best_map.rbegin()->first) {
                k_best_map.erase(std::prev(k_best_map.end()));
                k_best_map[true_dist_sq] = idx;
            }
        }
    };

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

std::vector<size_t> WeightedNanoFlann::radiusSearch(const Eigen::VectorXd& query, double radius) const {
    if (query.size() != dimension_) {
        throw std::invalid_argument("radiusSearch: Query has the wrong dimension.");
    }
    
    std::set<size_t> unique_indices;

    auto query_and_add = [&](const Eigen::VectorXd& current_query) {
        std::vector<size_t> candidate_indices = rawTree_.radiusSearch(current_query.cwiseProduct(weights_), radius);
        for (size_t idx : candidate_indices) {
            Eigen::VectorXd unscaled_point = getUnscaledPoint(idx);
            Eigen::VectorXd diff = unscaled_point - query;
            for (size_t i = 0; i < wrap_dims_.size(); ++i) {
                 diff(wrap_dims_[i]) = normalizeAngle(diff(wrap_dims_[i]));
            }
            if (diff.cwiseProduct(weights_).norm() <= radius) {
                unique_indices.insert(idx);
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

std::pair<std::vector<size_t>, std::vector<size_t>> WeightedNanoFlann::radiusSearchDual(
    const Eigen::VectorXd& query, double r1, double r2) const {
    
    std::vector<size_t> results_r1 = radiusSearch(query, r1);
    std::vector<size_t> results_r2;
    std::set<size_t> set_r1(results_r1.begin(), results_r1.end());

    for (size_t idx : results_r1) {
        Eigen::VectorXd unscaled_point = getUnscaledPoint(idx);
        Eigen::VectorXd diff = unscaled_point - query;
        for (size_t i = 0; i < wrap_dims_.size(); ++i) {
            diff(wrap_dims_[i]) = normalizeAngle(diff(wrap_dims_[i]));
        }
        if (diff.cwiseProduct(weights_).norm() <= r2) {
            results_r2.push_back(idx);
        }
    }
    return {results_r1, results_r2};
}

// --- Helper and Forwarding Methods ---

Eigen::VectorXd WeightedNanoFlann::getUnscaledPoint(size_t index) const {
    if (index >= static_cast<size_t>(unscaled_data_.rows())) {
        throw std::out_of_range("getUnscaledPoint: Index out of range.");
    }
    return unscaled_data_.row(index);
}

bool WeightedNanoFlann::removePoint(const Eigen::VectorXd& query) {
    // This is complex: requires finding the closest point in the unscaled data first.
    // For now, we don't support it to avoid ambiguity.
    throw std::logic_error("removePoint is not supported in WeightedNanoFlann.");
    return false;
}

bool WeightedNanoFlann::removeByIndex(size_t index) {
    if (index >= static_cast<size_t>(unscaled_data_.rows())) return false;
    
    if (index < static_cast<size_t>(unscaled_data_.rows()) - 1) {
        unscaled_data_.block(index, 0, unscaled_data_.rows() - index - 1, dimension_) = 
            unscaled_data_.bottomRows(unscaled_data_.rows() - index - 1);
    }
    unscaled_data_.conservativeResize(unscaled_data_.rows() - 1, Eigen::NoChange);
    
    return rawTree_.removeByIndex(index);
}

void WeightedNanoFlann::buildTree() { rawTree_.buildTree(); }
void WeightedNanoFlann::printData() const { rawTree_.printData(); }
void WeightedNanoFlann::clear() {
    rawTree_.clear();
    unscaled_data_.resize(0, dimension_);
}
size_t WeightedNanoFlann::size() const { return rawTree_.size(); }

// --- Validation Functions ---
bool WeightedNanoFlann::validateAgainstSamples(const std::vector<std::shared_ptr<IFMTNode>>& s) const { return false; }
bool WeightedNanoFlann::validateAgainstSamples(const std::vector<std::shared_ptr<BITNode>>& s) const { return false; }
