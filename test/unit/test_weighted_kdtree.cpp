#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <cmath> // For M_PI
#include <cassert> // For assertions
#include <algorithm> // For std::find
#include "motion_planning/utils/weighted_nano_flann.hpp"

// Helper function to print vectors cleanly for better test output
static void printVec(const Eigen::VectorXd& v) {
    std::cout << "[ ";
    for (int i = 0; i < v.size(); ++i) {
        std::cout << v[i] << (i + 1 < v.size() ? ", " : " ");
    }
    std::cout << "]";
}

// Helper to check if a vector contains a specific value
static bool contains(const std::vector<size_t>& vec, size_t val) {
    return std::find(vec.begin(), vec.end(), val) != vec.end();
}

int main() {
    std::cout << "--- WeightedNanoFlann Comprehensive Test ---\n\n";
    const int dim = 4;  // State is (x, y, theta, time)

    // 1. Define weights to emphasize spatial distance slightly over angular/temporal
    Eigen::VectorXd weights(dim);
    weights << 1.0, 1.0, 0.5, 0.2; // Give theta a moderate weight, time a low one
    
    // 2. Define the wrapping dimension (theta) and its period (2*PI)
    std::vector<int> wrap_dims = {2}; // The 3rd element (index 2) is theta
    std::vector<double> wrap_periods = {2.0 * M_PI};
    
    // 3. Create the weighted KD-tree with wrapping information
    auto tree = std::make_unique<WeightedNanoFlann>(dim, weights, wrap_dims, wrap_periods);
    std::cout << "Testing with 1 wrapping dimension (theta at index 2).\n";
    std::cout << "Weights: "; printVec(weights); std::cout << "\n\n";

    // 4. Define a more extensive set of test points
    std::vector<Eigen::VectorXd> pts;
    // --- Points for wrap-around testing ---
    pts.push_back((Eigen::VectorXd(dim) << 1, 1, 0.5, 0).finished());     // Point 0: Spatially close, angularly close.
    pts.push_back((Eigen::VectorXd(dim) << 1, 1, 6.2, 0).finished());     // Point 1: The TRUE nearest neighbor due to wrap-around.
    pts.push_back((Eigen::VectorXd(dim) << 1, 1, -0.2, 0).finished());    // Point 2: Also very close angularly.
    // --- Other points ---
    pts.push_back((Eigen::VectorXd(dim) << 20, 20, 3.0, 10).finished());  // Point 3: A distant point.
    pts.push_back((Eigen::VectorXd(dim) << 0.5, 0.5, 0.0, 0).finished()); // Point 4: Spatially very close.
    pts.push_back((Eigen::VectorXd(dim) << 5, 5, M_PI, 5).finished());   // Point 5: Moderately distant.


    tree->addPoints(pts);
    tree->buildTree();
    
    std::cout << "Inserted " << tree->size() << " points.\n\n";
    
    // 5. kNN Search Test with Wrap-Around
    Eigen::VectorXd query(dim);
    query << 1, 1, 0.1, 0;
    std::cout << "--- kNN Search Test ---\n";
    std::cout << "Querying for k=3 nearest neighbors to "; printVec(query); std::cout << "\n";
    
    // Expected order based on weighted wrapped distance:
    // 1st: Point 1 (theta=6.2 is ~0.18 away from 0.1)
    // 2nd: Point 2 (theta=-0.2 is 0.3 away from 0.1)
    // 3rd: Point 0 (theta=0.5 is 0.4 away from 0.1)
    std::cout << "EXPECTED ORDER: 1, 2, 0\n";
    std::vector<size_t> nn = tree->knnSearch(query, 3);
    
    std::cout << "\nRESULT:\n";
    std::cout << "  knnSearch(k=3) found indices: ";
    for (auto idx : nn) std::cout << idx << " ";
    std::cout << "\n";

    assert(nn.size() == 3 && "kNN search should return 3 results.");
    assert(nn[0] == 1 && "kNN: Nearest should be index 1 (wrapped).");
    assert(nn[1] == 2 && "kNN: Second nearest should be index 2.");
    assert(nn[2] == 0 && "kNN: Third nearest should be index 0.");
    std::cout << "  SUCCESS: kNN search returned correct neighbors in the correct order.\n\n";

    // 6. Radius Search Test with Wrap-Around
    double R = 0.25; // A weighted radius. Expected to catch points 0, 1, and 2.
    std::cout << "--- Radius Search Test ---\n";
    std::cout << "Querying for neighbors within weighted radius R = " << R << "\n";
    std::cout << "EXPECTED: Should find indices 0, 1, and 2.\n";
    
    std::vector<size_t> r_res = tree->radiusSearch(query, R);
    std::cout << "\nRESULT:\n";
    std::cout << "  radiusSearch found indices: ";
    for (auto idx : r_res) std::cout << idx << " ";
    std::cout << "\n";
    
    bool found0 = contains(r_res, 0);
    bool found1 = contains(r_res, 1);
    bool found2 = contains(r_res, 2);

    if (found0 && found1 && found2 && r_res.size() == 3) {
        std::cout << "  SUCCESS: Found all expected direct and wrapped neighbors within the radius." << std::endl;
    } else {
        std::cout << "  FAILURE: Did not find the expected set of neighbors." << std::endl;
    }
    assert(found0 && found1 && found2 && r_res.size() == 3 && "Radius search wrap-around test failed!");
    
    std::cout << "\n--- Test Complete ---\n";
    return 0;
}