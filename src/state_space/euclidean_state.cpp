// Copyright 2025 Soheil E.nia

#include "motion_planning/state_space/euclidean_state.hpp"
 
EuclideanState::EuclideanState(int dimension):value_(Eigen::VectorXd::Zero(dimension)) {
    std::cout << "EulideanState Constructor \n";
}
EuclideanState::EuclideanState(const Eigen::VectorXd& value): value_(value) {
    // std::cout << "EulideanState Constructor 2 \n";
}

std::unique_ptr<State> EuclideanState::clone() const {
    return std::make_unique<EuclideanState>(value_);
}
bool EuclideanState::equals(const State& other) const {
    const auto& otherState = dynamic_cast<const EuclideanState&>(other);
    return value_.isApprox(otherState.value_);
}
std::string EuclideanState::toString() const {
    std::ostringstream oss;
    oss << "EuclideanState: " << value_.transpose();
    return oss.str();
}

Eigen::VectorXd EuclideanState::getValue() const {
     return value_;
}



