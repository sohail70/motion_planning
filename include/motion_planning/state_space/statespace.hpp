// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/state_space/euclidean_state.hpp"


class StateSpace {
 public:
    explicit StateSpace(int dimension, int initial_capacity = 1000)
        : dimension_(dimension), num_states_(0) {
        samples_.resize(dimension, initial_capacity);
        states_.resize(initial_capacity);
    }

    virtual ~StateSpace() = default;
    virtual std::unique_ptr<State> allocState() const = 0;
    virtual void sampleUniform(double min, double max) = 0;
    virtual void sampleUniform(int k) = 0;
    virtual double distance(const std::unique_ptr<State>& state1, const std::unique_ptr<State>& state2) const = 0;
    virtual std::unique_ptr<State> interpolate(const std::unique_ptr<State>& state1, const std::unique_ptr<State>& state2, double t) const = 0;
    virtual bool isValid(const std::unique_ptr<State>& state) const = 0; 

    void addState(std::unique_ptr<State> state) {
        if (num_states_ >= samples_.cols()) {
            samples_.conservativeResize(Eigen::NoChange, samples_.cols() * 2); // Resize dataset if needed
        }
        samples_.col(num_states_) = state->getValue();  // Store the values in the matrix

        // Resize the states container if needed
        if (num_states_ >= states_.size()) {
            states_.resize(states_.size() * 2);
        }
        states_[num_states_] = std::move(state);  // Move the actual state object
        num_states_++;
    }

    std::unique_ptr<State>& getState(int index) {
        if (index < 0 || index >= num_states_) {
            throw std::out_of_range("Index out of range");
        }
        return states_[index];  // Return reference to the unique_ptr<State>
    }



    const Eigen::MatrixXd& getSamples() const { return samples_;}
    Eigen::MatrixXd getSamplesCopy() const { return samples_.leftCols(num_states_).eval();}  // eval creates a copy. without eval its just a (reference) view of the first num_states_ columns of the matrix!
    int getNumStates() const { return num_states_; }
 protected:
    int dimension_;
    int num_states_;
    Eigen::MatrixXd samples_;   
    std::vector<std::unique_ptr<State>> states_;
};







