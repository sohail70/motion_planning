// Copyright 2025 Soheil E.nia

#include "motion_planning/utils/nano_flann.hpp"


NanoFlann::NanoFlann(int dimension) : dimension_(dimension), data_(0, dimension), num_points_(0), capacity_(0){
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


    //     // Resize in chunks to minimize reallocations
    // if (num_points_ >= capacity_) {
    //     size_t new_capacity = (capacity_ == 0) ? 1 : capacity_ * 2;
    //     data_.conservativeResize(new_capacity, dimension_);
    //     // Initialize the newly added rows with infinity
    //     data_.bottomRows(new_capacity - num_points_).setConstant(std::numeric_limits<double>::infinity());


    //     capacity_ = new_capacity;
    // }

    // data_.row(num_points_++) = stateValue;
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

void NanoFlann::clear() {
    data_.resize(0, dimension_); // Clear all stored points
    kdtree_->index_->buildIndex(); // Rebuild the empty index
}



// Add this to the public section of NanoFlann in nano_flann.hpp
bool NanoFlann::removePoint(const Eigen::VectorXd& query) {
    if (data_.rows() == 0) {
        return false;  // No points to remove
    }

    // 1. Find the nearest neighbor (k=1)
    std::vector<size_t> nearestIndices = knnSearch(query, 1);
    if (nearestIndices.empty()) {
        return false;  // No neighbors found (shouldn't happen if tree is built)
    }

    size_t rowToRemove = nearestIndices[0];

    // Optional: Check if the found point matches the query (if exact removal is needed)
    // if ((data_.row(rowToRemove) - query.transpose()).norm() > 1e-6) {
    //     return false;  // Nearest neighbor is not an exact match
    // }

    // 2. Remove the row from data_
    removeRow(data_, rowToRemove);

    // 3. Rebuild the KD-tree
    kdtree_->index_->buildIndex();

    return true;
}

// Helper function to remove a row from an Eigen::MatrixXd
void NanoFlann::removeRow(Eigen::MatrixXd& matrix, size_t rowToRemove) {
    size_t numRows = matrix.rows() - 1;
    size_t numCols = matrix.cols();

    if (rowToRemove < numRows) {
        matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) = 
            matrix.bottomRows(numRows - rowToRemove);
    }
    matrix.conservativeResize(numRows, numCols);
}

bool NanoFlann::removeByIndex(size_t index) {
    if (index >= data_.rows()) return false;
    
    // Mirror swap-and-pop from samples_
    if (index != data_.rows() - 1) {
        data_.row(index) = data_.row(data_.rows() - 1);
    }
    data_.conservativeResize(data_.rows() - 1, Eigen::NoChange);
    return true;
}


// Eigen::VectorXd NanoFlann::getPoint(size_t index) const {
//     return data_.row(index);
// }

// size_t NanoFlann::size() const { return data_.rows(); }





// Compare with external samples vector
bool NanoFlann::validateAgainstSamples(const std::vector<std::shared_ptr<IFMTNode>>& samples) const {
    if (data_.rows() != samples.size()) {
        std::cerr << "Size mismatch: KD-tree has " << data_.rows() 
                    << " points, samples has " << samples.size() << "\n";
        return false;
    }

    for (size_t i = 0; i < samples.size(); ++i) {
        if (!data_.row(i).isApprox(samples[i]->getStateValue().transpose(), 1e-6)) {
            std::cerr << "Mismatch at index " << i << ":\n"
                        << "KD-tree: " << data_.row(i) << "\n"
                        << "Sample:  " << samples[i]->getStateValue().transpose() << "\n";
            return false;
        }
    }
    return true;
}


// Compare with external samples vector
bool NanoFlann::validateAgainstSamples(const std::vector<std::shared_ptr<BITNode>>& samples) const {
    if (data_.rows() != samples.size()) {
        std::cerr << "Size mismatch: KD-tree has " << data_.rows() 
                    << " points, samples has " << samples.size() << "\n";
        return false;
    }

    for (size_t i = 0; i < samples.size(); ++i) {
        if (!data_.row(i).isApprox(samples[i]->getStateValue().transpose(), 1e-6)) {
            std::cerr << "Mismatch at index " << i << ":\n"
                        << "KD-tree: " << data_.row(i) << "\n"
                        << "Sample:  " << samples[i]->getStateValue().transpose() << "\n";
            return false;
        }
    }
    return true;
}