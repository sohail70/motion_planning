// Copyright 2025 SOheil E.nia

#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/state_space/state.hpp"
#include "motion_planning/state_space/statespace.hpp"

enum class PlannerType{
    FMTX,
    RRTX,
    RRTSTAR,
    RRT,
};


class Planner {
 public:
    Planner(std::unique_ptr<StateSpace> statespace) : statespace_(std::move(statespace)) {

    }
    virtual ~Planner() = default;

    void setStart(const Eigen::VectorXd& start) {
        this->root_state_index_ = statespace_->getNumStates();
        statespace_->addState(statespace_->allocState(start));
    }

    void setGoal(const Eigen::VectorXd& goal) {
        this->robot_state_index_ = statespace_->getNumStates();
        statespace_->addState(statespace_->allocState(goal));
    }

    int getGoalIndex() {return robot_state_index_;}
    int getStarIndex() {return root_state_index_;}
    

    virtual void plan() = 0;
    virtual std::vector<std::shared_ptr<State>> getPath() const = 0;

 private:
    std::unique_ptr<StateSpace> statespace_;
    int root_state_index_;
    int robot_state_index_;
};


