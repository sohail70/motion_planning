// Copyright 2025 Soheil E.nia
/**
 * TODO: Create solve() and setPlanner() in Problem Definition Class    
 * TODO: Put variables in fmtx in a struct
 */
#include "motion_planning/state_space/euclidean_statespace.hpp"
#include "motion_planning/planners/planner_factory.hpp"

int main() {
    bool using_factory = true;
    int dim = 2;
    auto problem_def = std::make_unique<ProblemDefinition>(dim);
    problem_def->setStart(Eigen::VectorXd::Zero(dim));
    problem_def->setGoal(Eigen::VectorXd::Ones(dim)*50);
    problem_def->setBounds(-50, 50);

    PlannerParams params;
    params.setParam("num_of_samples", 5000);
    params.setParam("use_kdtree", true);
    params.setParam("kdtree_type", "NanoFlann");

    std::unique_ptr<StateSpace> statespace = std::make_unique<EuclideanStateSpace>(dim, 100);

    std::unique_ptr<Planner> planner;

    if (using_factory)
        planner = PlannerFactory::getInstance().createPlanner(PlannerType::FMTX, std::move(statespace), std::move(problem_def));
    else
        planner = std::make_unique<FMTX>(std::move(statespace), std::move(problem_def));
    planner->setup(std::move(params));

    planner->plan();
}
