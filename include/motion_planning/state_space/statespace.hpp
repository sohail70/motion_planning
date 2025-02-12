// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/state_space/euclidean_state.hpp"


class StateSpace {
 public:
    explicit StateSpace(int dimension, int initial_capacity = 1000)
        : dimension_(dimension), num_states_(0) {
        samples_.resize(initial_capacity, dimension);
    }

    virtual ~StateSpace() = default;
    virtual std::unique_ptr<State> addState(const Eigen::VectorXd& value) = 0;
    virtual std::unique_ptr<State> sampleUniform(double min, double max) = 0;
    virtual void sampleUniform(double min, double max, int k) = 0;
    virtual double distance(const std::unique_ptr<State>& state1, const std::unique_ptr<State>& state2) const = 0;
    virtual std::unique_ptr<State> interpolate(const std::unique_ptr<State>& state1, const std::unique_ptr<State>& state2, double t) const = 0;
    virtual bool isValid(const std::unique_ptr<State>& state) const = 0;







    const Eigen::MatrixXd& getSamples() const { return samples_;} // This one might have zeros in it because of doubling the capacity in the addState function
    Eigen::MatrixXd getSamplesCopy() const { return samples_.topRows(num_states_).eval();}  // eval creates a copy. without eval its just a (reference) view of the first num_states_ columns of the matrix!
    int getNumStates() const { return num_states_; }
    int getDimension() const { return dimension_; }



 protected:

    std::unique_ptr<State> addState(std::unique_ptr<State> state) {
        if (num_states_ >= samples_.rows()) {
            int new_capacity = static_cast<int>(samples_.rows() * 1.5);
            samples_.conservativeResize(new_capacity, Eigen::NoChange);
        }
        samples_.row(num_states_) = state->getValue();
        num_states_++;
        return state;
    }


    int dimension_;
    int num_states_;

    Eigen::MatrixXd samples_;   
};







