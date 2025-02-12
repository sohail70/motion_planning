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

    // Specialization for const char*
    /*
    overloaded versions of setParam for const char* and std::string directly in the class. This simplifies the code and avoids the need for template specializations.
    */
    void setParam(const std::string& key, const char* value) {
        params_[key] = std::string(value);
    }

    // Specialization for std::string
    void setParam(const std::string& key, const std::string& value) {
        params_[key] = value;
    }

 private:
    std::unordered_map<std::string, std::string> params_;

    // Generic template for convert
    template <typename T>
    T convert(const std::string& value) const;
};

// Specializations of convert moved outside the class
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


// Specialization for bool
template <>
inline bool PlannerParams::convert<bool>(const std::string& value) const {
    if (value == "true" || value == "1") {
        return true;
    } else if (value == "false" || value == "0") {
        return false;
    } else {
        throw std::runtime_error("Invalid boolean value: " + value);
    }
}