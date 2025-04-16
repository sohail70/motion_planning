// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/state_space/state.hpp"
#include "motion_planning/state_space/euclidean_state.hpp"
#include "motion_planning/pch.hpp"
#include "motion_planning/ds/ifmt_node.hpp"
#include "motion_planning/ds/bit_node.hpp"
class KDTree {
 public:
     virtual ~KDTree() = default;
     virtual void addPoint(const Eigen::VectorXd& stateValue) = 0;
     virtual void addPoints(const std::vector<Eigen::VectorXd>& statesValues) = 0;
     virtual void addPoints(const Eigen::MatrixXd& states) = 0;

     virtual void buildTree() = 0;
     virtual std::vector<size_t> knnSearch(const Eigen::VectorXd& query , int k) const = 0;
     virtual std::vector<size_t> radiusSearch(const Eigen::VectorXd& query , double radius) const = 0;
     virtual void printData() const = 0;
     // To find nodes in the r-neighborhood and a*r-neighborhood with a<1 in one pass!
     virtual std::pair<std::vector<size_t>, std::vector<size_t>> radiusSearchDual(const Eigen::VectorXd& query, double radius1, double radius2) const = 0;
     virtual void clear() = 0; 
     virtual bool removePoint(const Eigen::VectorXd& query) = 0;
     virtual bool removeByIndex(size_t index) = 0;

    //  virtual Eigen::VectorXd getPoint(size_t index) const = 0
     virtual size_t size() const = 0;
    
     virtual bool validateAgainstSamples(const std::vector<std::shared_ptr<IFMTNode>>& samples) const = 0;
     virtual bool validateAgainstSamples(const std::vector<std::shared_ptr<BITNode>>& samples) const = 0;

 private:

};
