// Copyright 2025 Soheil E.nia

#include "motion_planning/state_space/euclidean_statespace.hpp"

EuclideanStateSpace::EuclideanStateSpace(int dimension,int capacity):StateSpace(dimension,capacity) {
    std::cout << "Euclidean state space constructor \n";
}

std::unique_ptr<State> EuclideanStateSpace::allocState(const Eigen::VectorXd& value) const {
     return std::make_unique<EuclideanState>(value);
}

// void EuclideanStateSpace::sampleUniform() {
//     Eigen::VectorXd values = Eigen::VectorXd::Random(dimension_); // Random sample in [-1, 1]
//     this->addState(std::make_unique<EuclideanState>(values));
    
// }

void EuclideanStateSpace::sampleUniform(double min = 0.0, double max = 1.0) {
    Eigen::VectorXd values = Eigen::VectorXd::Random(dimension_); // Generate values in [-1,1]
    values = min + (max - min) * (values.array() + 1) / 2; // Scale to [min, max]
    this->addState(std::make_unique<EuclideanState>(values));
}

void EuclideanStateSpace::sampleUniform(double min = 0.0, double max = 1.0, int k = 1) {
    this->samples_ = Eigen::MatrixXd::Random(k, dimension_);
    this->samples_ = min * Eigen::MatrixXd::Ones(k, dimension_) + 
                     0.5 * (max - min) * (this->samples_ + Eigen::MatrixXd::Ones(k, dimension_));  //Because Random is between -1,1 so you have to shift it to [0,2] then divide it to [0,1] and the scale it to [min,max]

    for (int i = 0; i < k; ++i) {
        Eigen::VectorXd sample = samples_.row(i);
        addState(std::make_unique<EuclideanState>(sample));
    }
}

double EuclideanStateSpace::distance(const std::unique_ptr<State>& state1 , const std::unique_ptr<State>& state2) const {
    return (state1->getValue() - state2->getValue()).norm(); // Euclidean distance
}

std::unique_ptr<State> EuclideanStateSpace::interpolate(const std::unique_ptr<State>& state1, const std::unique_ptr<State>& state2, double t) const { // t in [0, 1]
    Eigen::VectorXd interpolated = state1->getValue() + t * (state2->getValue() - state1->getValue());
    return std::make_unique<EuclideanState>(interpolated);
}

bool EuclideanStateSpace::isValid(const std::unique_ptr<State>& state) const {
    // Add validation logic here (e.g., check bounds)
    return true;
}

