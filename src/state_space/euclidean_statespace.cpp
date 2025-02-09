// Copyright 2025 Soheil E.nia

#include "motion_planning/state_space/euclidean_statespace.hpp"

EuclideanStateSpace::EuclideanStateSpace(int dimension):dimension_(dimension) {
    std::cout << "Euclidean state space constructor \n";
}

std::unique_ptr<State> EuclideanStateSpace::allocState() const {
    return std::make_unique<EuclideanState>(dimension_);
}

std::unique_ptr<State> EuclideanStateSpace::sampleUniform() const {
    
}


double EuclideanStateSpace::distance(const State& state1 , const State& state2) const {

}

std::unique_ptr<State> EuclideanStateSpace::interpolate(const State& state1, const State& state2, double t) const { // t in [0, 1]

}

