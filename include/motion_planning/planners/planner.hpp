// Copyright 2025 SOheil E.nia

#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/state_space/state.hpp"
#include "motion_planning/state_space/statespace.hpp"
#include "motion_planning/utils/problem_definition.hpp"
#include "motion_planning/utils/planner_params.hpp"


enum class PlannerType{
    FMTX,
    RRTX,
    RRTSTAR,
    RRT,
};


class Planner {
 public:
    Planner() = default;
    virtual ~Planner() = default;



    // virtual void setStart(const Eigen::VectorXd& start) {
    //     this->root_state_index_ = statespace_->getNumStates();
    //     statespace_->addState(start);
    // }

    // virtual void setGoal(const Eigen::VectorXd& goal) {
    //     this->robot_state_index_ = statespace_->getNumStates();
    //     statespace_->addState(goal);
    // }

    virtual void setStart(const Eigen::VectorXd& start) = 0;
    virtual void setGoal(const Eigen::VectorXd& goal) = 0;


    
    virtual void setup(const PlannerParams& params) = 0;
    virtual void plan() = 0;
    virtual std::vector<std::shared_ptr<State>> getPath() const = 0;

 protected:


};


