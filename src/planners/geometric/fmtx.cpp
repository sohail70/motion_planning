// Copyright 2025 Soheil E.nia
#include "motion_planning/planners/geometric/fmtx.hpp"

FMTX::FMTX(std::unique_ptr<StateSpace> statespace) : Planner(std::move(statespace)) {
    std::cout<< "FMTX Constructor \n";
}

void FMTX::plan() {
    std::cout<< "Plan FMTX \n";
}


std::vector<std::shared_ptr<State>> FMTX::getPath() const {

}