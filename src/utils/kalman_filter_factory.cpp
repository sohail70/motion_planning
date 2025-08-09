// Copyright 2025 Soheil E.nia

#include "motion_planning/utils/kalman_filter_factory.hpp"

KalmanFilter KalmanFilterFactory::createFilter(const std::string& model_name) {
    if (model_name == "cv") {
        return KalmanFilter(KalmanModelType::CONSTANT_VELOCITY);
    } 
    else if (model_name == "ca") {
        return KalmanFilter(KalmanModelType::CONSTANT_ACCELERATION);
    }
    else if (model_name == "singer") {
        // You can pull these parameters from a config file as well
        double singer_alpha = 0.1;
        double singer_sigma_a = 5.0; // Based on Max Acceleration
        return KalmanFilter(KalmanModelType::SINGER, singer_alpha, singer_sigma_a);
    }
    else {
        // Default to the simplest, most robust model if the name is unknown
        return KalmanFilter(KalmanModelType::CONSTANT_VELOCITY);
    }
}