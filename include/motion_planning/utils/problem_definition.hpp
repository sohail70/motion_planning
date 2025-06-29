// // Copyright 2025 Soheil E.nia
// #pragma once
// #include <Eigen/Dense>
// #include <memory>

// class ProblemDefinition {
//  public:
//       ProblemDefinition(int dim) : dimension_(dim), start_(Eigen::VectorXd::Zero(dim)), goal_(Eigen::VectorXd::Zero(dim)) {}

//       void setStart(const Eigen::VectorXd& start) { start_ = start; }
//       void setGoal(const Eigen::VectorXd& goal) { goal_ = goal; }
    
//       const Eigen::VectorXd& getStart() const { return start_; }
//       const Eigen::VectorXd& getGoal() const { return goal_; }

//       void setBounds(double lower, double upper) { lower_bound_ = lower; upper_bound_ = upper; }
//       double getLowerBound() const { return lower_bound_; }
//       double getUpperBound() const { return upper_bound_; }

//       bool hasSolution() const { return solution_cost_ < std::numeric_limits<double>::max(); }
//       void setSolution(double cost) { solution_cost_ = cost; }


//  private:
//     int dimension_;
//     Eigen::VectorXd start_;
//     Eigen::VectorXd goal_;
//     double lower_bound_ = 0;
//     double upper_bound_ = 0;


//    double solution_cost_ = std::numeric_limits<double>::max();

// };

//////////////////////////////////

// Copyright 2025 Soheil E.nia
#pragma once
#include <Eigen/Dense>
#include <memory>
#include <limits> // Required for std::numeric_limits

class ProblemDefinition {
 public:
    /**
     * @brief Construct a new Problem Definition object.
     * @param dim The dimension of the state space.
     */
    ProblemDefinition(int dim) 
        : dimension_(dim),
          start_(Eigen::VectorXd::Zero(dim)),
          goal_(Eigen::VectorXd::Zero(dim)),
          lower_bounds_(Eigen::VectorXd::Zero(dim)),
          upper_bounds_(Eigen::VectorXd::Zero(dim)) {}

    /**
     * @brief Set the start state.
     * @param start The start state vector.
     */
    void setStart(const Eigen::VectorXd& start) { start_ = start; }

    /**
     * @brief Set the goal state.
     * @param goal The goal state vector.
     */
    void setGoal(const Eigen::VectorXd& goal) { goal_ = goal; }
    
    /**
     * @brief Get the start state.
     * @return A const reference to the start state vector.
     */
    const Eigen::VectorXd& getStart() const { return start_; }

    /**
     * @brief Get the goal state.
     * @return A const reference to the goal state vector.
     */
    const Eigen::VectorXd& getGoal() const { return goal_; }

    /**
     * @brief Set the state space bounds for each dimension individually.
     * @param lower A vector of lower bounds for each dimension.
     * @param upper A vector of upper bounds for each dimension.
     */
    void setBounds(const Eigen::VectorXd& lower, const Eigen::VectorXd& upper) {
        lower_bounds_ = lower;
        upper_bounds_ = upper;
    }

    /**
     * @brief Set uniform state space bounds across all dimensions.
     * @param lower The lower bound value to apply to all dimensions.
     * @param upper The upper bound value to apply to all dimensions.
     */
    void setBounds(double lower, double upper) {
        lower_bounds_ = Eigen::VectorXd::Constant(dimension_, lower);
        upper_bounds_ = Eigen::VectorXd::Constant(dimension_, upper);
    }

    /**
     * @brief Get the vector of lower bounds.
     * @return A const reference to the lower bounds vector.
     */
    const Eigen::VectorXd& getLowerBound() const { return lower_bounds_; }

    /**
     * @brief Get the vector of upper bounds.
     * @return A const reference to the upper bounds vector.
     */
    const Eigen::VectorXd& getUpperBound() const { return upper_bounds_; }

    /**
     * @brief Check if a solution has been found.
     * @return True if the solution cost is less than infinity.
     */
    bool hasSolution() const { return solution_cost_ < std::numeric_limits<double>::max(); }

    /**
     * @brief Set the cost of the found solution.
     * @param cost The solution cost.
     */
    void setSolution(double cost) { solution_cost_ = cost; }

 private:
    int dimension_;
    Eigen::VectorXd start_;
    Eigen::VectorXd goal_;

    // Changed to Eigen::VectorXd to support per-dimension bounds
    Eigen::VectorXd lower_bounds_;
    Eigen::VectorXd upper_bounds_;

    double solution_cost_ = std::numeric_limits<double>::max();
};