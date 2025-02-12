// Copyright 2025 Soheil E.nia

#include "motion_planning/state_space/euclidean_statespace.hpp"
#include "motion_planning/planners/planner_factory.hpp"

// int main() {
//     // Step 1: Define the state space (e.g., 2D Euclidean space)
//     int dimension = 2; // 2D space
//     EuclideanStateSpace stateSpace(dimension,10);

//     // Step 2: Create start and goal states
//     std::vector<double> startValues = {0.0, 0.0}; // Start at (0, 0)
//     std::vector<double> goalValues = {10.0, 10.0};  // Goal at (10, 10)

//     auto startState = std::make_shared<EuclideanState>(startValues);
//     auto goalState = std::make_shared<EuclideanState>(goalValues);

//     // Step 3: Create a planner (e.g., FMTX)
//     PlannerFactory& factory = PlannerFactory::getInstance();
//     auto planner = factory.createPlanner(PlannerType::FMTX);

//     // Step 4: Set the start and goal states for the planner
//     planner->setStart(*startState);
//     planner->setGoal(*goalState);

//     // Step 5: Plan the path
//     planner->plan();

//     // Step 6: Retrieve and print the path
//     auto path = planner->getPath();
//     std::cout << "Planned Path:" << std::endl;
//     for (const auto& state : path) {
//         auto euclideanState = dynamic_cast<const EuclideanState*>(state.get());
//         if (euclideanState) {
//             std::cout << "(" << euclideanState->value_[0] << ", " << euclideanState->value_[1] << ")" << std::endl;
//         }
//     }

//     return 0;
// }

//////////////////////////////////////////////////////////////////
// int main() {
//     bool using_factory = true;
//     int dim = 2;
//     std::unique_ptr<StateSpace> statespace = std::make_unique<EuclideanStateSpace>(dim, 5);
//     statespace->sampleUniform(0, 1, 5);

//     std::unique_ptr<Planner> planner;

//     if (using_factory)
//         planner = PlannerFactory::getInstance().createPlanner(PlannerType::FMTX, std::move(statespace));
//     else
//         planner = std::make_unique<FMTX>(std::move(statespace));

//     Eigen::VectorXd start(dim);
//     start << 0, 0;
//     Eigen::VectorXd goal{dim};
//     start << 1, 1;
 
//     planner->setStart(start);
//     planner->setGoal(goal);

//     std::cout << planner->getStarIndex() << "\n";
//     std::cout << planner->getGoalIndex() << "\n";
// }

/////////////////////////////////////////////////////////////////

int main() {
    bool using_factory = true;
    int dim = 2;
    auto problem_def = std::make_unique<ProblemDefinition>(dim);
    problem_def->setStart(Eigen::VectorXd::Zero(dim));
    problem_def->setGoal(Eigen::VectorXd::Ones(dim)*30);
    problem_def->setBounds(0, 30);

    PlannerParams params;
    params.setParam("num_of_samples", 15);
    params.setParam("lower_bound", 0);
    params.setParam("upper_bound", 30);
    params.setParam("use_kdtree", true);
    params.setParam("kdtree_type", "NanoFlann");

    std::unique_ptr<StateSpace> statespace = std::make_unique<EuclideanStateSpace>(dim, 5);

    std::unique_ptr<Planner> planner;

    if (using_factory)
        planner = PlannerFactory::getInstance().createPlanner(PlannerType::FMTX, std::move(statespace), std::move(problem_def));
    else
        planner = std::make_unique<FMTX>(std::move(statespace), std::move(problem_def));
    planner->setup(std::move(params));





    planner->plan();
}