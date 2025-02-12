// Copyright 2025 Soheil E.nia
#pragma once
#include <unordered_map>
#include <string>
#include <stdexcept>

class PlannerParams {
 public:
    template <typename T>
    T getParam(const std::string& key) const {
        auto it = params_.find(key);
        if (it == params_.end()) {
            throw std::runtime_error("Parameter not found: " + key);
        }
        return convert<T>(it->second);
    }

    template <typename T>
    void setParam(const std::string& key, const T& value) {
        params_[key] = std::to_string(value);
    }

 private:
    std::unordered_map<std::string, std::string> params_;

    // Generic template for convert
    template <typename T>
    T convert(const std::string& value) const;
};

// Specializations of convert moved outside the class: because -> C++ does not allow explicit specialization of member functions inside a class
template <>
inline int PlannerParams::convert<int>(const std::string& value) const {
    return std::stoi(value);
}

template <>
inline double PlannerParams::convert<double>(const std::string& value) const {
    return std::stod(value);
}

template <>
inline std::string PlannerParams::convert<std::string>(const std::string& value) const {
    return value;
}
