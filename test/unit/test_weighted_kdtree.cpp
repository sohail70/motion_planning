// // Copyright 2025 Soheil E.nia

// #include <iostream>
// #include <memory>
// #include <vector>
// #include <Eigen/Dense>
// #include <cmath> // For M_PI
// #include <cassert> // For assertions
// #include <algorithm> // For std::find
// #include "motion_planning/utils/weighted_nano_flann.hpp"

// static void printVec(const Eigen::VectorXd& v) {
//     std::cout << "[ ";
//     for (int i = 0; i < v.size(); ++i) {
//         std::cout << v[i] << (i + 1 < v.size() ? ", " : " ");
//     }
//     std::cout << "]";
// }

// static bool contains(const std::vector<size_t>& vec, size_t val) {
//     return std::find(vec.begin(), vec.end(), val) != vec.end();
// }

// int main() {
//     std::cout << "--- WeightedNanoFlann Comprehensive Test ---\n\n";
//     const int dim = 4;  // State is (x, y, theta, time)

//     Eigen::VectorXd weights(dim);
//     weights << 1.0, 1.0, 0.5, 0.2; // Give theta a moderate weight, time a low one
    
//     std::vector<int> wrap_dims = {2}; // The 3rd element (index 2) is theta
//     std::vector<double> wrap_periods = {2.0 * M_PI};
    
//     auto tree = std::make_unique<WeightedNanoFlann>(dim, weights, wrap_dims, wrap_periods);
//     std::cout << "Testing with 1 wrapping dimension (theta at index 2).\n";
//     std::cout << "Weights: "; printVec(weights); std::cout << "\n\n";

//     std::vector<Eigen::VectorXd> pts;
//     pts.push_back((Eigen::VectorXd(dim) << 1, 1, 0.5, 0).finished());     // Point 0: Spatially close, angularly close.
//     pts.push_back((Eigen::VectorXd(dim) << 1, 1, 6.2, 0).finished());     // Point 1: The TRUE nearest neighbor due to wrap-around.
//     pts.push_back((Eigen::VectorXd(dim) << 1, 1, -0.2, 0).finished());    // Point 2: Also very close angularly.
//     pts.push_back((Eigen::VectorXd(dim) << 20, 20, 3.0, 10).finished());  // Point 3: A distant point.
//     pts.push_back((Eigen::VectorXd(dim) << 0.5, 0.5, 0.0, 0).finished()); // Point 4: Spatially very close.
//     pts.push_back((Eigen::VectorXd(dim) << 5, 5, M_PI, 5).finished());   // Point 5: Moderately distant.


//     tree->addPoints(pts);
//     tree->buildTree();
    
//     std::cout << "Inserted " << tree->size() << " points.\n\n";
    
//     Eigen::VectorXd query(dim);
//     query << 1, 1, 0.1, 0;
//     std::cout << "--- kNN Search Test ---\n";
//     std::cout << "Querying for k=3 nearest neighbors to "; printVec(query); std::cout << "\n";
    
//     // Expected order based on weighted wrapped distance:
//     // 1st: Point 1 (theta=6.2 is ~0.18 away from 0.1)
//     // 2nd: Point 2 (theta=-0.2 is 0.3 away from 0.1)
//     // 3rd: Point 0 (theta=0.5 is 0.4 away from 0.1)
//     std::cout << "EXPECTED ORDER: 1, 2, 0\n";
//     std::vector<size_t> nn = tree->knnSearch(query, 3);
    
//     std::cout << "\nRESULT:\n";
//     std::cout << "  knnSearch(k=3) found indices: ";
//     for (auto idx : nn) std::cout << idx << " ";
//     std::cout << "\n";

//     assert(nn.size() == 3 && "kNN search should return 3 results.");
//     assert(nn[0] == 1 && "kNN: Nearest should be index 1 (wrapped).");
//     assert(nn[1] == 2 && "kNN: Second nearest should be index 2.");
//     assert(nn[2] == 0 && "kNN: Third nearest should be index 0.");
//     std::cout << "  SUCCESS: kNN search returned correct neighbors in the correct order.\n\n";

//     double R = 0.25; // A weighted radius. Expected to catch points 0, 1, and 2.
//     std::cout << "--- Radius Search Test ---\n";
//     std::cout << "Querying for neighbors within weighted radius R = " << R << "\n";
//     std::cout << "EXPECTED: Should find indices 0, 1, and 2.\n";
    
//     std::vector<size_t> r_res = tree->radiusSearch(query, R);
//     std::cout << "\nRESULT:\n";
//     std::cout << "  radiusSearch found indices: ";
//     for (auto idx : r_res) std::cout << idx << " ";
//     std::cout << "\n";
    
//     bool found0 = contains(r_res, 0);
//     bool found1 = contains(r_res, 1);
//     bool found2 = contains(r_res, 2);

//     if (found0 && found1 && found2 && r_res.size() == 3) {
//         std::cout << "  SUCCESS: Found all expected direct and wrapped neighbors within the radius." << std::endl;
//     } else {
//         std::cout << "  FAILURE: Did not find the expected set of neighbors." << std::endl;
//     }
//     assert(found0 && found1 && found2 && r_res.size() == 3 && "Radius search wrap-around test failed!");
    
//     std::cout << "\n--- Test Complete ---\n";
//     return 0;
// }


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include "motion_planning/utils/weighted_nano_flann.hpp"

static void print_vector(const Eigen::VectorXd& v) {
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "[ ";
    for (int i = 0; i < v.size(); ++i) {
        std::cout << std::setw(10) << v[i] << (i + 1 < v.size() ? ", " : " ");
    }
    std::cout << "]";
}

// The main function where the test is executed.
int main() {
    std::cout << "--- Weighted KD-Tree Test with Custom Data ---\n\n";

    const int dimension = 4;

    Eigen::VectorXd weights(dimension);
    weights << 1.0, 1.0, 1.0, 1.0; // Weights for x, y, theta, time

    std::vector<int> wrap_dims = {2};
    std::vector<double> wrap_periods = {2.0 * M_PI};

    auto tree = std::make_unique<WeightedNanoFlann>(dimension, weights, wrap_dims, wrap_periods);
    std::cout << "KD-Tree created for " << dimension << " dimensions.\n";
    std::cout << "Weights applied: ";
    print_vector(weights.transpose());
    std::cout << "\nWrapping dimension at index " << wrap_dims[0] << " with period " << wrap_periods[0] << "\n\n";

    std::vector<Eigen::VectorXd> points;
    points.push_back((Eigen::VectorXd(dimension) << 0, 0, 1.5708, 0).finished());
    points.push_back((Eigen::VectorXd(dimension) << -46.653, -17.0036, 1.1978, 8.06984).finished());
    points.push_back((Eigen::VectorXd(dimension) << -29.3735, -24.9872, 0.858023, 13.3635).finished());
    points.push_back((Eigen::VectorXd(dimension) << -19.8344, -47.5076, -0.848276, 12.1846).finished());
    points.push_back((Eigen::VectorXd(dimension) << -18.214, -36.4272, -2.47089, 12.1281).finished());
    points.push_back((Eigen::VectorXd(dimension) << -41.8328, 5.09562, 0.40622, 11.9192).finished());
    points.push_back((Eigen::VectorXd(dimension) << 48.176, -28.115, -0.286539, 9.22385).finished());
    points.push_back((Eigen::VectorXd(dimension) << 7.38417, 5.5231, 1.436, 10.5465).finished());
    points.push_back((Eigen::VectorXd(dimension) << 29.6128, 8.37158, -1.41518, 12.9552).finished());
    points.push_back((Eigen::VectorXd(dimension) << 41.368, 46.5402, -1.5577, 4.43934).finished());
    points.push_back((Eigen::VectorXd(dimension) << -28.4469, 38.8643, 3.03834, 9.20623).finished());
    points.push_back((Eigen::VectorXd(dimension) << 48, 48, -1.5708, 15).finished());

    tree->addPoints(points);
    tree->buildTree();
    std::cout << "Successfully inserted " << tree->size() << " points into the tree.\n\n";

    Eigen::VectorXd query_point(dimension);
    query_point << 0, 0, 1.5708, 0;
    const double radius = 40.0;

    std::cout << "--- Radius Search Query ---\n";
    std::cout << "Query Point: ";
    print_vector(query_point.transpose());
    std::cout << "\nSearch Radius: " << radius << "\n\n";

    std::vector<size_t> found_indices = tree->radiusSearch(query_point, radius);

    std::cout << "--- Search Results ---\n";
    if (found_indices.empty()) {
        std::cout << "No neighbors found within the given radius.\n";
    } else {
        std::cout << "Found " << found_indices.size() << " neighbor(s) within radius " << radius << ":\n";
        for (size_t index : found_indices) {
            Eigen::VectorXd diff = points[index] - query_point;
            diff(wrap_dims[0]) = normalizeAngle(diff(wrap_dims[0])); // Apply angle normalization for distance check.
            double weighted_distance = diff.cwiseProduct(weights).norm();

            std::cout << "  - Index " << std::setw(2) << index << ": ";
            print_vector(points[index].transpose());
            std::cout << " (Weighted Distance: " << weighted_distance << ")\n";
        }
    }

    std::cout << "\n--- Test Complete ---\n";
    return 0;
}