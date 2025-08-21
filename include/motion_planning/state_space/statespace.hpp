// Copyright 2025 Soheil E.nia
/**
 * TODO: I cant remember because of what problem I used "Keep Alive" in this class, because to the best of my knowledge owner of the state is some other class so remove it later! 
 * TODO: Make the steer to use node's pointer instead of Eigen! for that i need to put the different planner's node in the same interface. With this way I can make the steer in min snap to be integrated in the upper interface
 */
#pragma once

#include "motion_planning/state_space/euclidean_state.hpp"
#include "motion_planning/ds/edge_info.hpp" // For Trajectory struct


class StateSpace {
 public:
    explicit StateSpace(int dimension, int initial_capacity = 1000)
        : dimension_(dimension), num_states_(0) {
        states_.resize(initial_capacity, dimension);
    }

    virtual ~StateSpace() = default;
    virtual std::shared_ptr<State> addState(const Eigen::VectorXd& value) = 0;
    virtual std::shared_ptr<State> sampleUniform(double min, double max) = 0;
    virtual void sampleUniform(double min, double max, int k) = 0;
    virtual std::shared_ptr<State> sampleUniform(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds) = 0;

    virtual double distance(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2) const = 0;
    virtual std::shared_ptr<State> interpolate(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2, double t) const = 0;
    virtual bool isValid(const std::shared_ptr<State>& state) const = 0;


    // This is the fundamental steer function that ALL state spaces MUST implement.
    virtual Trajectory steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const = 0;

    // Add the advanced steer function for the main planning loop.
    virtual Trajectory steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to,
                             const Eigen::VectorXd& to_vel, const Eigen::VectorXd& to_accel) const {
        // By default, a simple state space just ignores the extra derivative info
        // and calls its own basic steer function.
        return steer(from, to);
    }

    // Add the steer function for bridging the robot to the tree.
    virtual Trajectory steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to,
                             const Eigen::VectorXd& from_vel, const Eigen::VectorXd& from_accel,
                             const Eigen::VectorXd& to_vel, const Eigen::VectorXd& to_accel) const {
        // By default, this also falls back to the simpler version above.
        return steer(from, to, to_vel, to_accel);
    }



    virtual bool prefersLazyNear() const { return false; } // Default to proactive (i.e., caching and not lazy)



    const Eigen::MatrixXd& getSamples() const { return states_;} // This one might have zeros in it because of doubling the capacity in the addState function
    Eigen::MatrixXd getSamplesCopy() const { return states_.topRows(num_states_).eval();}  // eval creates a copy. without eval its just a (reference) view of the first num_states_ columns of the matrix!
    int getNumStates() const { return num_states_; }
    int getDimension() const { return dimension_; }

    void reset() {
        num_states_ = 0; // Reset the counter
        // states_.resize(0, dimension_); // Clear the states matrix
        std::cout << "StateSpace reset: num_states_ = " << num_states_ << std::endl;
    }



 protected:
    std::vector<std::shared_ptr<State>> state_objects_;
    std::mutex state_mutex_;


    std::shared_ptr<State> addState(std::shared_ptr<State> state) {
        // std::lock_guard<std::mutex> lock(state_mutex_);

        state_objects_.push_back(state);  // Keep alive

        if (num_states_ >= states_.rows()) {
            int new_capacity = static_cast<int>(states_.rows() * 1.5);
            states_.conservativeResize(new_capacity, Eigen::NoChange);
        }
        states_.row(num_states_) = state->getValue();
        num_states_++;
        return state;
    }



    int dimension_;
    int num_states_;

    Eigen::MatrixXd states_;   
};







