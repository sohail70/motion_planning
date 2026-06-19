// Copyright 2025 Soheil E.nia
/**
 * TODO: I cant remember because of what problem I used "Keep Alive" in this class, because to the best of my knowledge owner of the state is some other class so remove it later! 
 * TODO: Make the steer to use node's pointer instead of Eigen! for that i need to put the different planner's node in the same interface. With this way I can make the steer in min snap to be integrated in the upper interface
 */
#pragma once

#include "motion_planning/state_space/euclidean_state.hpp"
#include "motion_planning/ds/edge_info.hpp" // For Trajectory struct

// Global toggle for the centralized sampler (see sampleUnregistered() below).
#define USE_DETERMINISTIC_SAMPLING 0 // 1 = Halton low-dispersion (Janson-Ichter-Pavone); 0 = i.i.d. uniform

// Dynamics-aware sample shaping (velocity reachability cone / heading bias). 1 = on, 0 = off.
#define USE_DYNAMICS_AWARE_SAMPLING 0

class StateSpace {
 public:
    explicit StateSpace(int dimension, int initial_capacity = 5000)
        : dimension_(dimension), num_states_(0) {
        states_.resize(initial_capacity, dimension);
    }

    virtual ~StateSpace() = default;
    virtual std::shared_ptr<State> addState(const Eigen::VectorXd& value) = 0;
    virtual std::shared_ptr<State> sampleUniform(double min, double max) = 0;
    virtual void sampleUniform(double min, double max, int k) = 0;
    virtual std::shared_ptr<State> sampleUniform(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds) = 0;
    virtual Eigen::VectorXd sampleUniformUnregistered(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds) = 0;


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


    // Generates a local, safe survival maneuver for exactly one time slice (dt)
    virtual Trajectory generateEmergencyManeuver(const Eigen::VectorXd& state, double dt) const = 0;
    // Generates multiple dynamically feasible escape trajectories for 'dt' seconds
    virtual std::vector<Trajectory> getEscapePrimitives(const Eigen::VectorXd& state, double dt) const = 0;
    virtual std::vector<Trajectory> generateRecoveryPrimitives( const Eigen::VectorXd& state, double dt, int num_uniform_dirs = 16, int integration_steps = 12) const {};

    virtual bool prefersLazyNear() const { return false; } // Default to proactive (i.e., caching and not lazy)

    virtual double getMaxVelocity() const { return 0.0; }
    virtual double getMaxAcceleration() const { return 0.0; }

    // Dynamics-aware sample shaping. After remapTimeToGoalCone fixes the time coordinate,
    // make the remaining coordinates (velocity, heading, ...) consistent with the system's
    // reachability so nearby samples are more often mutually steerable. Default: no-op
    // (holonomic R2T needs none). Tree-independent (sample + goal + dynamics only) -> AO-safe.
    virtual void shapeKinodynamicSample(Eigen::VectorXd& /*sample*/, const Eigen::Vector2d& /*root_xy*/) const {}

    // Shared kinodynamic time remap (one-sided goal-reachability cone), used by every
    // planner. The last coordinate is time-to-goal; a sample at (x,y) needs at least
    // dist(xy, root_xy)/v_max of time-to-goal to reach the goal. Remaps the sampled time
    // uniformly into [t_to_goal, budget] (budget = the planner's time horizon / remaining
    // time), and returns false if the cone is empty (budget <= t_to_goal) so the caller
    // can redraw/skip. A fixed-set sampling-domain restriction (NOT informed sampling),
    // so FMT*/RRT* asymptotic optimality is preserved.
    bool remapTimeToGoalCone(Eigen::VectorXd& sample,
                             const Eigen::Vector2d& root_xy,
                             double budget,
                             double t_lower,
                             double t_upper) const {
        const double v = getMaxVelocity();
        if (v <= 0.0) return true;                 // no velocity model -> no shaping
        const int t_idx = dimension_ - 1;
        const double t_to_goal = (Eigen::Vector2d(sample.head(2)) - root_xy).norm() / v;
        if (budget <= t_to_goal) return false;     // empty cone -> caller redraws/skips
        const double span = t_upper - t_lower;
        const double u = (span > 0.0) ? (sample[t_idx] - t_lower) / span : 0.0;
        sample[t_idx] = t_to_goal + u * (budget - t_to_goal);
#if USE_DYNAMICS_AWARE_SAMPLING
        shapeKinodynamicSample(sample, root_xy);   // velocity cone / heading bias (no-op by default)
#endif
        return true;
    }

    // --- Centralized sampling strategy (used by every planner) ---------------
    // Returns an UNREGISTERED sample drawn with the configured strategy: a scrambled
    // Halton low-dispersion sequence when USE_DETERMINISTIC_SAMPLING==1, else i.i.d.
    // uniform. Register it with addState() if it is kept. Centralizing here makes all
    // planners sample identically (and removes per-planner copies).
    Eigen::VectorXd sampleUnregistered(const Eigen::VectorXd& lower, const Eigen::VectorXd& upper) {
#if USE_DETERMINISTIC_SAMPLING
        return sampleHaltonUnregistered(lower, upper);
#else
        return sampleUniformUnregistered(lower, upper);
#endif
    }

    // Scrambled-Halton low-dispersion draw. Stateful: continues ONE sequence as samples
    // accumulate, and a per-run random offset (Cranley-Patterson rotation, seeded lazily
    // from the uniform RNG) preserves low dispersion while giving per-seed variation.
    // Index/offset are cleared in reset(), so re-seed each run.
    Eigen::VectorXd sampleHaltonUnregistered(const Eigen::VectorXd& lower, const Eigen::VectorXd& upper) {
        if (halton_offset_.size() != lower.size()) {
            halton_offset_ = sampleUniformUnregistered(
                Eigen::VectorXd::Zero(lower.size()), Eigen::VectorXd::Ones(lower.size()));
            halton_index_ = 0;
        }
        static const unsigned int primes[] = {2u,3u,5u,7u,11u,13u,17u,19u,23u,29u};
        ++halton_index_;
        const int d = static_cast<int>(lower.size());
        Eigen::VectorXd p(d);
        for (int k = 0; k < d; ++k) {
            double u = haltonRadicalInverse(halton_index_, primes[k < 10 ? k : 9]) + halton_offset_[k];
            u -= std::floor(u);                          // wrap into [0,1)
            p[k] = lower[k] + u * (upper[k] - lower[k]);
        }
        return p;
    }

    const Eigen::MatrixXd& getSamples() const { return states_;} // This one might have zeros in it because of doubling the capacity in the addState function
    Eigen::MatrixXd getSamplesCopy() const { return states_.topRows(num_states_).eval();}  // eval creates a copy. without eval its just a (reference) view of the first num_states_ columns of the matrix!
    int getNumStates() const { return num_states_; }
    int getDimension() const { return dimension_; }

    void reset() {
        num_states_ = 0; // Reset the counter
        halton_index_ = 0;                  // re-seed the deterministic sampler per run
        halton_offset_ = Eigen::VectorXd();
        // states_.resize(0, dimension_); // Clear the states matrix
        std::cout << "StateSpace reset: num_states_ = " << num_states_ << std::endl;
    }



 protected:
    // std::vector<std::shared_ptr<State>> state_objects_;
    // std::mutex state_mutex_;


    std::shared_ptr<State> addState(std::shared_ptr<State> state) {
        // std::lock_guard<std::mutex> lock(state_mutex_);

        // state_objects_.push_back(state);  // Keep alive

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

 private:
    // Deterministic (scrambled-Halton) low-dispersion sampler state.
    static double haltonRadicalInverse(unsigned int i, unsigned int base) {
        double f = 1.0, r = 0.0;
        while (i > 0) { f /= base; r += f * static_cast<double>(i % base); i /= base; }
        return r;
    }
    unsigned int halton_index_ = 0;
    Eigen::VectorXd halton_offset_;
};







