// Copyright 2025 Soheil E.nia

#include "motion_planning/state_space/euclidean_statespace.hpp"
#include <random> 

EuclideanStateSpace::EuclideanStateSpace(int dimension,int capacity,unsigned int seed):StateSpace(dimension,capacity) {
    std::srand(seed); // TODO: For sampling the same batch every time just for debug and test. --> remove it later.

    std::cout << "Euclidean state space constructor \n";
}

std::shared_ptr<State> EuclideanStateSpace::addState(const Eigen::VectorXd& value) {
    return StateSpace::addState(std::make_shared<EuclideanState>(value));
}


std::shared_ptr<State> EuclideanStateSpace::sampleUniform(double min = 0.0, double max = 1.0) {
    Eigen::VectorXd values = Eigen::VectorXd::Random(dimension_); // Generate values in [-1,1]
    values = min + (max - min) * (values.array() + 1) / 2; // Scale to [min, max]
    return StateSpace::addState(std::make_shared<EuclideanState>(values));
}

void EuclideanStateSpace::sampleUniform(double min = 0.0, double max = 1.0, int k = 1) {
    this->states_ = Eigen::MatrixXd::Random(k, dimension_);
    this->states_ = min * Eigen::MatrixXd::Ones(k, dimension_) + 
                     0.5 * (max - min) * (this->states_ + Eigen::MatrixXd::Ones(k, dimension_));  //Because Random is between -1,1 so you have to shift it to [0,2] then divide it to [0,1] and the scale it to [min,max]

    for (int i = 0; i < k; ++i) {
        Eigen::VectorXd sample = states_.row(i);
        StateSpace::addState(std::make_shared<EuclideanState>(sample));
    }
}

double EuclideanStateSpace::distance(const std::shared_ptr<State>& state1 , const std::shared_ptr<State>& state2) const {
    return (state1->getValue() - state2->getValue()).norm(); // Euclidean distance
}

std::shared_ptr<State> EuclideanStateSpace::interpolate(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2, double t) const { // t in [0, 1]
    Eigen::VectorXd interpolated = state1->getValue() + t * (state2->getValue() - state1->getValue());
    return std::make_shared<EuclideanState>(interpolated);
}

bool EuclideanStateSpace::isValid(const std::shared_ptr<State>& state) const {
    // Add validation logic here (e.g., check bounds)
    return true;
}

