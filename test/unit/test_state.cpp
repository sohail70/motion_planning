// Copyright 2025 Soheil E.nia

#include "motion_planning/state_space/euclidean_state.hpp"
#include "motion_planning/state_space/euclidean_statespace.hpp"

int main() {
    // int dim = 2;
    // ///////////////////////////////////////////
    // std::cout << "Incremental Sampling \n";
    // unsigned int seed = 42;
    // std::shared_ptr<EuclideanStateSpace> statespace = std::make_shared<EuclideanStateSpace>(dim,4, seed); // 3 means it will be doubled if the 3 column storage gets full
    // for (int i = 0; i < 5 ; i++) {
    //     statespace->sampleUniform(0, 10);
    // }
    // std::cout << statespace->getSamples()<<"\n";
    // std::cout<<"----------------------------- \n";
    // std::cout << statespace->getSamplesCopy()<<"\n";

    // ///////////////////////////////////////////
    // std::cout << "Batch Sampling \n";
    // unsigned int seed = 42;
    // std::shared_ptr<EuclideanStateSpace> statespace2 = std::make_shared<EuclideanStateSpace>(dim,4,seed); // 3 means it will be doubled if the 3 column storage gets full
    // statespace2->sampleUniform(0, 10, 10);
    // std::cout << statespace2->getSamples() << "\n";
    // // If you need to access each state in the statespace by the index --> can't think of a reason for you to get it because changing it means messing up the samples_ matrix! maybe i should return a const ref! but thats okay because i don't have access to the value_ in the state!
    // for (int i = 0 ; i < 10 ; i++) { 
    //     auto& state = statespace2->getState(i);
    //     std::cout << state->getValue() << "\n";
    //     std::cout << "--" << "\n";
    // }

}
