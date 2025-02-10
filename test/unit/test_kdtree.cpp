// Copyright 2025 Soheil E.nia

#include "motion_planning/utils/nano_flann.hpp"
#include "motion_planning/state_space/euclidean_state.hpp"
#include "motion_planning/state_space/euclidean_statespace.hpp"
int main() {
    std::srand(std::time(0));  // Seed the random number generator
    std::cout << "KDTree Test \n";
    int dim = 2;
    std::shared_ptr<StateSpace> statespace = std::make_shared<EuclideanStateSpace>(dim,4); // 3 means it will be doubled if the 3 column storage gets full
    std::shared_ptr<NanoFlann> kd = std::make_shared<NanoFlann>(dim);

    for (int i = 0; i < 5 ; i++) {
        statespace->sampleUniform(0, 20);
        kd->addPoint(statespace->getState(i));
    }
    std::cout << statespace->getSamples()<<"\n";
    std::cout << "------- \n";
    auto neighbors = kd->knnSearch(statespace->getState(1),2);
    // std::cout << neighbors.at(0)->getValue() << "\n";
    // std::cout << neighbors.at(1)->getValue() << "\n";
    std::cout << "----- \n";
    auto neighbors2 = kd->radiusSearch(statespace->getState(1),10);
    // std::cout << neighbors2.at(0)->getValue() << "\n";

}

