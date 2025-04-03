// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/planners/planner.hpp"
#include "motion_planning/planners/geometric/fmt.hpp"
#include "motion_planning/planners/geometric/fmta.hpp"
#include "motion_planning/planners/geometric/fmtx.hpp"
#include "motion_planning/planners/geometric/rrtx.hpp"

class PlannerFactory {
 public:
    using Creator = std::function<std::unique_ptr<Planner>(std::unique_ptr<StateSpace>, std::shared_ptr<ProblemDefinition> , std::shared_ptr<ObstacleChecker> )>;

    std::unique_ptr<Planner> createPlanner(PlannerType, std::unique_ptr<StateSpace> statespace , std::shared_ptr<ProblemDefinition> problem , std::shared_ptr<ObstacleChecker> obs_checker );
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
      PlannerFactory::getInstance().registerPlanner(type,[](std::unique_ptr<StateSpace> statespace, std::shared_ptr<ProblemDefinition> problem , std::shared_ptr<ObstacleChecker> obs_checker ){
         return std::make_unique<pType>(std::move(statespace), problem , obs_checker);
      });
   }
};


static AutorRegisterPlanners<FMT> autoRegisterFMT(PlannerType::FMT);
static AutorRegisterPlanners<FMTA> autoRegisterFMTa(PlannerType::FMTA);
static AutorRegisterPlanners<FMTX> autoRegisterFMTx(PlannerType::FMTX);
static AutorRegisterPlanners<RRTX> autoRegisterRRTx(PlannerType::RRTX);
