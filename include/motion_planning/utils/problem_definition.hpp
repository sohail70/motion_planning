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

    ProblemDefinition(int dim) 
        : dimension_(dim),
          start_(Eigen::VectorXd::Zero(dim)),
          goal_(Eigen::VectorXd::Zero(dim)),
          lower_bounds_(Eigen::VectorXd::Zero(dim)),
          upper_bounds_(Eigen::VectorXd::Zero(dim)) {}


    void setStart(const Eigen::VectorXd& start) { start_ = start; }


    void setGoal(const Eigen::VectorXd& goal) { goal_ = goal; }
    

    const Eigen::VectorXd& getStart() const { return start_; }


    const Eigen::VectorXd& getGoal() const { return goal_; }

    void setBounds(const Eigen::VectorXd& lower, const Eigen::VectorXd& upper) {
        lower_bounds_ = lower;
        upper_bounds_ = upper;
    }

    void setBounds(double lower, double upper) {
        lower_bounds_ = Eigen::VectorXd::Constant(dimension_, lower);
        upper_bounds_ = Eigen::VectorXd::Constant(dimension_, upper);
    }

    const Eigen::VectorXd& getLowerBound() const { return lower_bounds_; }

    const Eigen::VectorXd& getUpperBound() const { return upper_bounds_; }


    bool hasSolution() const { return solution_cost_ < std::numeric_limits<double>::max(); }

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