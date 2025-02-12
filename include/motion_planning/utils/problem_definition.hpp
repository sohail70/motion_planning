// Copyright 2025 Soheil E.nia
#pragma once
#include <Eigen/Dense>
#include <memory>

class ProblemDefinition {
 public:
    ProblemDefinition(int dim) : dimension_(dim), start_(Eigen::VectorXd::Zero(dim)), goal_(Eigen::VectorXd::Zero(dim)) {}

    void setStart(const Eigen::VectorXd& start) { start_ = start; }
    void setGoal(const Eigen::VectorXd& goal) { goal_ = goal; }
    
    const Eigen::VectorXd& getStart() const { return start_; }
    const Eigen::VectorXd& getGoal() const { return goal_; }

    void setBounds(double lower, double upper) { lower_bound_ = lower; upper_bound_ = upper; }
    double getLowerBound() const { return lower_bound_; }
    double getUpperBound() const { return upper_bound_; }

 private:
    int dimension_;
    Eigen::VectorXd start_;
    Eigen::VectorXd goal_;
    double lower_bound_ = 0;
    double upper_bound_ = 0;
};