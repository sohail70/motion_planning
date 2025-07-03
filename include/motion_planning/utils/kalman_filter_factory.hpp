#pragma once

#include "motion_planning/utils/kalman_filter.hpp" // Assumes your refactored KF is here
#include <string>

class KalmanFilterFactory {
public:
    /**
     * @brief Creates a KalmanFilter instance based on a model name.
     * @param model_name The name of the model ("cv", "ca", or "singer").
     * @return A configured KalmanFilter object.
     */
    static KalmanFilter createFilter(const std::string& model_name);
};