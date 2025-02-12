// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/planners/planner.hpp"
#include "motion_planning/planners/geometric/fmtx.hpp"

class PlannerFactory {
 public:
    using Creator = std::function<std::unique_ptr<Planner>(std::unique_ptr<StateSpace>, std::unique_ptr<ProblemDefinition>)>;

    std::unique_ptr<Planner> createPlanner(PlannerType, std::unique_ptr<StateSpace> statespace , std::unique_ptr<ProblemDefinition> problem);
    void registerPlanner(PlannerType, Creator);
    static PlannerFactory& getInstance();
 private:
    PlannerFactory();
    std::unordered_map<PlannerType, Creator > planners_;
};

template<typename pType>
class AutorRegisterPlanners { 
 public:
   AutorRegisterPlanners(const PlannerType& type){
      PlannerFactory::getInstance().registerPlanner(type,[](std::unique_ptr<StateSpace> statespace, std::unique_ptr<ProblemDefinition> problem){
         return std::make_unique<pType>(std::move(statespace), std::move(problem));
      });
   }
};


static AutorRegisterPlanners<FMTX> autoRegisterFmtx(PlannerType::FMTX);
// static AutorRegisterPlanners<RRTX> autoRegisterFmtx(PlannerType::RRTX);
