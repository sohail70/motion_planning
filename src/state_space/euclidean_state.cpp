// Copyright 2025 Soheil E.nia

#include "motion_planning/state_space/euclidean_state.hpp"

EuclideanState::EuclideanState(int dimension_): value_(std::vector<double>(dimension_ , 0)) {
    std::cout << "EulideanState Constructor \n";
}
EuclideanState::EuclideanState(const std::vector<double>& values) {
    std::cout << "EulideanState Constructor 2 \n";
}

std::unique_ptr<State> EuclideanState::clone() const {

}
bool EuclideanState::equals(const State& other) const {

}
std::string EuclideanState::toString() const {

}



