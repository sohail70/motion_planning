// Copyright 2025 Soheil E.nia

#include "motion_planning/planners/planner_factory.hpp"
#include "motion_planning/planners/geometric/fmtx.hpp"

PlannerFactory::PlannerFactory() {
    std::cout<<"Planner Factory Constructor \n";
}

PlannerFactory& PlannerFactory::getInstance() {
    static PlannerFactory planner_factory_;
    return planner_factory_;
}

void PlannerFactory::registerPlanner(PlannerType type, Creator creator ) {
    planners_.insert(std::make_pair(type,creator));
}

std::unique_ptr<Planner> PlannerFactory::createPlanner(PlannerType type, std::shared_ptr<StateSpace> statespace, std::shared_ptr<ProblemDefinition> problem, std::shared_ptr<ObstacleChecker> obs_checker ) {  
    auto object = planners_.find(type);
    if (object != planners_.end())
    {
        return object->second(statespace , problem , obs_checker);
    }
    throw std::invalid_argument("Unknown planner type: ");
}

