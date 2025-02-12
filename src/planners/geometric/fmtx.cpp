// Copyright 2025 Soheil E.nia
#include "motion_planning/planners/geometric/fmtx.hpp"

FMTX::FMTX(std::unique_ptr<StateSpace> statespace ,std::unique_ptr<ProblemDefinition> problem_def) :  statespace_(std::move(statespace)), problem_(std::move(problem_def)) {
    std::cout<< "FMTX Constructor \n";

}

void FMTX::setup(const PlannerParams& params) {
    num_of_samples_ = params.getParam<int>("num_of_samples");
    lower_bound_ = params.getParam<double>("lower_bound");
    upper_bound_ = params.getParam<double>("upper_bound");
    std::cout << "FMTX setup complete: num_of_samples=" << num_of_samples_ 
                << ", bounds=[" << lower_bound_ << ", " << upper_bound_ << "]\n";

    setStart(problem_->getStart());
    setGoal(problem_->getGoal());
}

void FMTX::plan() {
    std::cout<< "Plan FMTX \n";
    for (int i = 0 ; i < num_of_samples_ ; i++) { // BUT THIS DOESNT CREATE A TREE NODE FOR START AND GOAL !!!
        
        tree_.push_back(std::make_unique<TreeNode>(statespace_->sampleUniform(lower_bound_ , upper_bound_)));
    }

}

std::vector<std::shared_ptr<State>> FMTX::getPath() const {

}

void FMTX::setStart(const Eigen::VectorXd& start) {
    robot_state_index_ = statespace_->getNumStates();
    tree_.push_back(std::make_shared<TreeNode>(statespace_->addState(start)));
    std::cout << "FMTX: Start node created on Index: " << robot_state_index_ << "\n";
}
void FMTX::setGoal(const Eigen::VectorXd& goal) {
    root_state_index_ = statespace_->getNumStates();
    tree_.push_back(std::make_shared<TreeNode>(statespace_->addState(goal)));
    std::cout << "FMTX: Goal node created on Index: " << root_state_index_ << "\n";
}