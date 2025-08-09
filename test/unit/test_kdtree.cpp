// Copyright 2025 Soheil E.nia

#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include "motion_planning/state_space/euclidean_statespace.hpp"
#include "motion_planning/utils/nano_flann.hpp"

int main() {
    std::srand(42);  // Fixed seed for reproducibility
    std::cout << "KDTree Test \n";
    int dim = 2;
    unsigned int seed = 42;
    std::shared_ptr<StateSpace> statespace = std::make_shared<EuclideanStateSpace>(dim, 4, seed);
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
    std::cout << "Initial KD-Tree Data:\n";
    kd->printData();

    // Define the point to remove (Point 1)
    Eigen::VectorXd pointToRemove(dim);
    pointToRemove << -4.69771, -1.71217;

    // Remove Point 1
    std::cout << "\nAttempting to remove Point 1 (" << pointToRemove.transpose() << ")...\n";
    bool removed = kd->removePoint(pointToRemove);
    if (removed) {
        std::cout << "Removal successful!\n";
    } else {
        std::cout << "Removal failed (point not found).\n";
    }

    // Print the updated KD-tree data
    std::cout << "\nUpdated KD-Tree Data:\n";
    kd->printData();

    return 0;
}