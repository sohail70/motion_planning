// Copyright 2025 Soheil E.nia

#include "motion_planning/state_space/euclidean_statespace.hpp"
#include "motion_planning/planners/planner_factory.hpp"

int main() {
    // Step 1: Define the state space (e.g., 2D Euclidean space)
    int dimension = 2; // 2D space
    EuclideanStateSpace stateSpace(dimension);

    // Step 2: Create start and goal states
    std::vector<double> startValues = {0.0, 0.0}; // Start at (0, 0)
    std::vector<double> goalValues = {10.0, 10.0};  // Goal at (10, 10)

    auto startState = std::make_shared<EuclideanState>(startValues);
    auto goalState = std::make_shared<EuclideanState>(goalValues);

    // Step 3: Create a planner (e.g., FMTX)
    PlannerFactory& factory = PlannerFactory::getInstance();
    auto planner = factory.createPlanner(PlannerType::FMTX);

    // Step 4: Set the start and goal states for the planner
    planner->setStart(*startState);
    planner->setGoal(*goalState);

    // Step 5: Plan the path
    planner->plan();

    // Step 6: Retrieve and print the path
    auto path = planner->getPath();
    std::cout << "Planned Path:" << std::endl;
    for (const auto& state : path) {
        auto euclideanState = dynamic_cast<const EuclideanState*>(state.get());
        if (euclideanState) {
            std::cout << "(" << euclideanState->value_[0] << ", " << euclideanState->value_[1] << ")" << std::endl;
        }
    }

    return 0;
}


