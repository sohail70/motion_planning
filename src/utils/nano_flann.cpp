// Copyright 2025 Soheil E.nia

#include "motion_planning/utils/nano_flann.hpp"


NanoFlann::NanoFlann(int dimension) : dimension_(dimension), data_(0, dimension) {
    kdtree_ = std::make_unique<NFKDTree>(dimension, data_, 10 /* max leaf */);
    kdtree_->index_->buildIndex();
    std::cout << "KDTree initialized with dimension: " << dimension_ << std::endl;

}


void NanoFlann::addPoint(const Eigen::VectorXd& stateValue) {
    Eigen::VectorXd value = stateValue;
    if (value.size() != dimension_) {
        throw std::invalid_argument("State dimension does not match KDTree dimension");
    }

    // Resize the data matrix to accommodate the new point
    data_.conservativeResize(data_.rows() + 1, Eigen::NoChange);
    data_.row(data_.rows() - 1) = value;

    // std::cout << "Added point: " << value.transpose() << std::endl;
    // std::cout << "Data matrix size: " << data_.rows() << "x" << data_.cols() << std::endl;
}

void NanoFlann::addPoints(const std::vector<Eigen::VectorXd>& statesValues) {
    // for (const auto& state : states) {
    //     Eigen::VectorXd value = state->getValue();
    //     if (value.size() != dimension_) {
    //         throw std::invalid_argument("State dimension does not match KDTree dimension");
    //     }

    //     // Resize the data matrix to accommodate the new points
    //     data_.conservativeResize(data_.rows() + 1, Eigen::NoChange);
    //     data_.row(data_.rows() - 1) = value;
    // }

    // // Rebuild the KDTree index
    // kdtree_->index_->buildIndex();
    // std::cout << "Added " << states.size() << " points." << std::endl;
    // std::cout << "Data matrix size: " << data_.rows() << "x" << data_.cols() << std::endl;
}

void NanoFlann::addPoints(const Eigen::MatrixXd& data) {
    this->data_ = data;
    std::cout << "Added lots of points \n";
}


void NanoFlann::buildTree() {
    // std::cout << "Building KDTREE \n";
    // Rebuild the KDTree index
    kdtree_->index_->buildIndex();
}


std::vector<size_t> NanoFlann::knnSearch(const Eigen::VectorXd& query, int k) const { //SHOULDN"T WE CHECK IF QUERY IS EVEN IN THE DATA OR NOT!!!! BECAUSE NANO FLANN MIGHT JUST GIVE THE NEIGHBOR OF EVERYTHING!
    Eigen::VectorXd queryValue = query;
    if (queryValue.size() != dimension_) {
        throw std::invalid_argument("Query state dimension does not match KDTree dimension");
    }

    std::vector<size_t> ret_indexes(k);
    std::vector<double> out_dists_sqr(k);

    nanoflann::KNNResultSet<double> resultSet(k);
    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);

    kdtree_->index_->findNeighbors(resultSet, queryValue.data(), nanoflann::SearchParameters(10));

    // // Return only the indices of the found neighbors
    // std::cout << "knnSearch results for query: " << queryValue.transpose() << std::endl;
    // for (size_t i = 0; i < resultSet.size(); ++i) {
    //     std::cout << "Neighbor " << i + 1 << ": Index " << ret_indexes[i]
    //               << " (distance squared: " << out_dists_sqr[i] << ")" << std::endl;
    // }

    return ret_indexes;  // Return only the indices
}





std::vector<size_t> NanoFlann::radiusSearch(const Eigen::VectorXd& query, double radius) const {
    Eigen::VectorXd queryValue = query;
    if (queryValue.size() != dimension_) {
        throw std::invalid_argument("Query state dimension does not match KDTree dimension");
    }

    std::vector<nanoflann::ResultItem<long int, double>> ret_matches;
    nanoflann::SearchParameters params;
    params.sorted = true;

    // Perform the radius search
    size_t nMatches = kdtree_->index_->radiusSearch(queryValue.data(), radius * radius, ret_matches, params);

    // Collect only the indices of the results
    std::vector<size_t> resultIndices;
    for (size_t i = 0; i < nMatches; ++i) {
        resultIndices.push_back(ret_matches[i].first);
    }

    // // Debug prints
    // std::cout << "radiusSearch results for query: " << queryValue.transpose() << " with radius: " << radius << std::endl;
    // for (size_t i = 0; i < resultIndices.size(); ++i) {
    //     std::cout << "Match " << i + 1 << ": Index " << resultIndices[i]
    //               << " (distance squared: " << ret_matches[i].second << ")" << std::endl;
    // }

    return resultIndices;  // Return only the indices
}

// I use this for inflation!
// Radius 1 must be bigger than radius 2 for this to work
std::pair<std::vector<size_t>, std::vector<size_t>> NanoFlann::radiusSearchDual(const Eigen::VectorXd& query, double radius1, double radius2) const {
    Eigen::VectorXd queryValue = query;
    if (queryValue.size() != dimension_) {
        throw std::invalid_argument("Query state dimension does not match KDTree dimension");
    }

    std::vector<nanoflann::ResultItem<long int, double>> ret_matches;
    nanoflann::SearchParameters params;
    params.sorted = true;

    // Perform the radius search for the larger radius
    size_t nMatches = kdtree_->index_->radiusSearch(queryValue.data(), radius1 * radius1, ret_matches, params);

    // Collect indices for both radii
    std::vector<size_t> resultIndices1; // Nodes within radius1
    std::vector<size_t> resultIndices2; // Nodes within radius2

    for (size_t i = 0; i < nMatches; ++i) {
        double distanceSquared = ret_matches[i].second;
        if (distanceSquared <= radius2 * radius2) {
            resultIndices2.push_back(ret_matches[i].first);
            resultIndices1.push_back(ret_matches[i].first);
        }else{
            resultIndices1.push_back(ret_matches[i].first);
        }
    }

    return {resultIndices1, resultIndices2};
}


void NanoFlann::printData() const {
    std::cout << "KD-Tree Data:\n";
    for (int i = 0; i < data_.rows(); ++i) {
        std::cout << "Point " << i << ": " << data_.row(i) << "\n";
    }
}