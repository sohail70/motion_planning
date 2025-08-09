// Copyright 2025 Soheil E.nia

#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include "motion_planning/utils/lie_kd_tree.hpp"
#include "motion_planning/state_space/dubins_time_statespace.hpp"

static void print_vector(const Eigen::VectorXd& v) {
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "[ ";
    for (int i = 0; i < v.size(); ++i) {
        std::cout << std::setw(10) << v[i] << (i + 1 < v.size() ? ", " : " ");
    }
    std::cout << "]";
}

int main() {
    std::cout << "--- Lie KD-Tree Test with Custom Data ---\n\n";

    const int dimension = 4;
    const double min_turning_radius = 5.0; // meters
    const double min_velocity = 2.0;       // m/s
    const double max_velocity = 20.0;      // m/s
    std::shared_ptr<StateSpace> dubins_time_ss = std::make_shared<DubinsTimeStateSpace>(min_turning_radius, min_velocity, max_velocity);
    // Instantiate the WeightedNanoFlann KD-Tree with the defined parameters.
    auto tree = std::make_unique<LieSplittingKDTree>(dimension, dubins_time_ss);
    std::cout << "Lie-KD-Tree created for " << dimension << " dimensions.\n";
    // std::cout << "Weights applied: ";
    // print_vector(weights.transpose());
    // std::cout << "\nWrapping dimension at index " << wrap_dims[0] << " with period " << wrap_periods[0] << "\n\n";
    

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

    for (auto& p : points){
        tree->addPoint(p);
    }
    int k = 3;
    double r = 40.0;
    Eigen::VectorXd query_point = points[0];


    auto result = tree->knnSearch(query_point, k);
    std::cout<<"The "<< k <<" node(s) near to: "<<query_point.transpose()<<" ,  are: \n";
    for(auto& r : result){
        std::cout<<points[r].transpose() << " with index " <<r<<"\n";
    }

    std::cout<<"RADIUS SEARCH: \n";

    auto result2 = tree->radiusSearch(query_point, r);
    for(auto& r : result2){
        std::cout<<points[r].transpose() << " with index " <<r<<"\n";
    }



}