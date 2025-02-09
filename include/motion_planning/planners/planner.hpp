// Copyright 2025 SOheil E.nia

#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/state_space/state.hpp"

enum class PlannerType{
    FMTX,
    RRTX,
    RRTSTAR,
    RRT,
};


class Planner {
 public:
    virtual ~Planner() = default;
    virtual void setStart(const State& start) = 0;
    virtual void setGoal(const State& goal) = 0;
    virtual void plan() = 0;
    virtual std::vector<std::shared_ptr<State>> getPath() const = 0;
};


