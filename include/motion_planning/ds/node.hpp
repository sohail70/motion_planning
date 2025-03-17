// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/state_space/state.hpp"

class Node {
 public:
    virtual ~Node() = default;
    virtual const Eigen::VectorXd& getStateVlaue() const = 0;

    virtual void setCost(double cost) = 0;
    virtual double getCost() const = 0;
    virtual void setLMC(double lmc) = 0;
    virtual double getLMC() const = 0;
    virtual void setParentIndex(int index) = 0;
    virtual int getParentIndex() const = 0;
    virtual void setChildrenIndex(int index) = 0;
    virtual std::vector<int>& getChildrenIndices() = 0; 
};





