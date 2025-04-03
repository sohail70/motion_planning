
// #pragma once

// #include "motion_planning/state_space/state.hpp"

// class SE2State : public State {
//  public:
//     SE2State(double x, double y, double theta) : value_(3) {
//         value_ << x, y, theta;
//     }

//     std::unique_ptr<State> clone() const override {
//         return std::make_unique<SE2State>(value_[0], value_[1], value_[2]);
//     }

//     bool equals(const State& other) const override {
//         const auto* otherState = dynamic_cast<const SE2State*>(&other);
//         return otherState && value_.isApprox(otherState->value_);
//     }

//     std::string toString() const override {
//         return "[" + std::to_string(value_[0]) + ", " +
//                      std::to_string(value_[1]) + ", " +
//                      std::to_string(value_[2]) + "]";
//     }

//     Eigen::VectorXd getValue() const override { return value_; }

//  private:
//     Eigen::VectorXd value_;  // (x, y, theta)
// };