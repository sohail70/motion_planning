// Copyright 2025 Soheil E.nia

#include "motion_planning/utils/nano_flann.hpp"


NanoFlann::NanoFlann(int dimension) : dimension_(dimension), data_(0, dimension) {
    kdtree_ = std::make_unique<NFKDTree>(dimension, data_, 10 /* max leaf */);
    kdtree_->index_->buildIndex();
    std::cout << "KDTree initialized with dimension: " << dimension_ << std::endl;

}

void NanoFlann::addPoint(const std::unique_ptr<State>& state) {
    Eigen::VectorXd value = state->getValue();
    if (value.size() != dimension_) {
        throw std::invalid_argument("State dimension does not match KDTree dimension");
    }

    // Resize the data matrix to accommodate the new point
    data_.conservativeResize(data_.rows() + 1, Eigen::NoChange);
    data_.row(data_.rows() - 1) = value;

    // Rebuild the KDTree index
    kdtree_->index_->buildIndex();
    std::cout << "Added point: " << value.transpose() << std::endl;
    std::cout << "Data matrix size: " << data_.rows() << "x" << data_.cols() << std::endl;
}

void NanoFlann::addPoints(const std::vector<std::shared_ptr<State>>& states) {
    for (const auto& state : states) {
        Eigen::VectorXd value = state->getValue();
        if (value.size() != dimension_) {
            throw std::invalid_argument("State dimension does not match KDTree dimension");
        }

        // Resize the data matrix to accommodate the new points
        data_.conservativeResize(data_.rows() + 1, Eigen::NoChange);
        data_.row(data_.rows() - 1) = value;
    }

    // Rebuild the KDTree index
    kdtree_->index_->buildIndex();
    std::cout << "Added " << states.size() << " points." << std::endl;
    std::cout << "Data matrix size: " << data_.rows() << "x" << data_.cols() << std::endl;
}

std::vector<std::shared_ptr<State>> NanoFlann::knnSearch(const std::unique_ptr<State>& query, int k) const {
    Eigen::VectorXd queryValue = query->getValue();
    if (queryValue.size() != dimension_) {
        throw std::invalid_argument("Query state dimension does not match KDTree dimension");
    }

    std::vector<size_t> ret_indexes(k);
    std::vector<double> out_dists_sqr(k);

    nanoflann::KNNResultSet<double> resultSet(k);
    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);

    kdtree_->index_->findNeighbors(resultSet, queryValue.data(), nanoflann::SearchParameters(10));

    std::vector<std::shared_ptr<State>> result;
    for (size_t i = 0; i < resultSet.size(); ++i) {
        Eigen::VectorXd point = data_.row(ret_indexes[i]);
        result.push_back(std::make_shared<EuclideanState>(point));
    }

    std::cout << "knnSearch results for query: " << queryValue.transpose() << std::endl;
    for (size_t i = 0; i < result.size(); ++i) {
        std::cout << "Neighbor " << i + 1 << ": " << result[i]->getValue().transpose()
                  << " (distance squared: " << out_dists_sqr[i] << ")" << std::endl;
    }


    return result;
}

std::vector<std::shared_ptr<State>> NanoFlann::radiusSearch(const std::unique_ptr<State>& query, double radius) const {
    Eigen::VectorXd queryValue = query->getValue();
    if (queryValue.size() != dimension_) {
        throw std::invalid_argument("Query state dimension does not match KDTree dimension");
    }

    // Use the correct type for ret_matches
    std::vector<nanoflann::ResultItem<long int, double>> ret_matches;
    nanoflann::SearchParameters params;
    params.sorted = true;

    // Perform the radius search
    size_t nMatches = kdtree_->index_->radiusSearch(queryValue.data(), radius * radius, ret_matches, params);

    // Convert results to shared_ptr<State>
    std::vector<std::shared_ptr<State>> result;
    for (size_t i = 0; i < nMatches; ++i) {
        Eigen::VectorXd point = data_.row(ret_matches[i].first);
        result.push_back(std::make_shared<EuclideanState>(point));
    }

    // Debug prints
    std::cout << "radiusSearch results for query: " << queryValue.transpose() << " with radius: " << radius << std::endl;
    for (size_t i = 0; i < result.size(); ++i) {
        std::cout << "Match " << i + 1 << ": " << result[i]->getValue().transpose()
                  << " (distance squared: " << ret_matches[i].second << ")" << std::endl;
    }

    return result;
}