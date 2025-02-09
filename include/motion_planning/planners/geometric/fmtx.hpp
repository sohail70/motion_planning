// Copyright 2025 Soheil E.Nia
#pragma once

#include<motion_planning/pch.hpp>
#include<motion_planning/planners/planner.hpp>

class FMTX : public Planner {
 public:
    FMTX();
    void setStart(const State& start) override;
    void setGoal(const State& goal) override;
    void plan() override;
    std::vector<std::shared_ptr<State>> getPath() const override;

 private:
    std::shared_ptr<State> start_;
    std::shared_ptr<State> goal_;
    std::vector<std::shared_ptr<State>> path_;
};

