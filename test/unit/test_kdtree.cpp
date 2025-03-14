#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include "motion_planning/state_space/euclidean_statespace.hpp"
#include "motion_planning/utils/nano_flann.hpp"

int main() {
    std::srand(42);  // Fixed seed for reproducibility
    std::cout << "KDTree Test \n";
    int dim = 2;
    std::shared_ptr<StateSpace> statespace = std::make_shared<EuclideanStateSpace>(dim, 4);
    std::shared_ptr<NanoFlann> kd = std::make_shared<NanoFlann>(dim);

    // Manually add the specified points to the KD-tree
    Eigen::MatrixXd points(4, dim); // 4 points, 2 dimensions
    points << 0, 0,               // Point 0
             -4.69771, -1.71217,  // Point 1
             4.63176, -1.88329,   // Point 2
             1.75788, 4.6808;     // Point 3

    // Add points to the KD-tree
    for (int i = 0; i < points.rows(); i++) {
        kd->addPoint(points.row(i));
    }

    // Rebuild the KD-tree after adding points
    kd->buildTree();

    // Print the KD-tree data
    std::cout << "KD-Tree Data:\n";
    kd->printData();

    // Define the query point
    Eigen::VectorXd queryPoint(dim);
    queryPoint << -13.5007, 26.5381;

    // Perform k-nearest neighbor search with k=1
    std::cout << "KNN Search Results for Query Point (" << queryPoint.transpose() << "):\n";
    auto neighbors = kd->knnSearch(queryPoint, 1); // k=1
    for (size_t idx : neighbors) {
        std::cout << "Nearest Neighbor: " << points.row(idx) << "\n";
    }

    return 0;
}