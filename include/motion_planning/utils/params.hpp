// Copyright 2025 Soheil E.nia

#pragma once

#include <unordered_map>
#include <string>
#include <stdexcept>
#include <type_traits>

class Params {
public:
    template <typename T>
    T getParam(const std::string& key) const {
        auto it = params_.find(key);
        if (it == params_.end()) {
            throw std::runtime_error("Parameter not found: " + key);
        }
        return convert<T>(it->second);
    }

    // Get param with default value!
    template<class T>
    T getParam(const std::string& key, const T& default_value) const {
        if (!hasParam(key)) { // Re-use hasParam for clarity
            return default_value;
        }
        return getParam<T>(key); // Call the single-argument version
    }


    // Check if a parameter exists
    bool hasParam(const std::string& key) const {
        return params_.find(key) != params_.end();
    }



    // Specialization for const char*
    /*
    overloaded versions of setParam for const char* and std::string directly in the class. This simplifies the code and avoids the need for template specializations.
    */
    void setParam(const std::string& key, const char* value) {
        params_[key] = std::string(value);
    }


    template <typename T>
    void setParam(const std::string& key, const T& value) {
        if constexpr (std::is_same_v<T, std::string> || std::is_same_v<T, const char*>) {
            params_[key] = std::string(value); // Handle strings directly
        } else if constexpr (std::is_same_v<T, bool>) {
            params_[key] = value ? "true" : "false"; // Handle booleans
        } else {
            params_[key] = std::to_string(value); // Handle numeric types
        }
    }

private:
    std::unordered_map<std::string, std::string> params_;

    // Generic template for convert
    template <typename T>
    T convert(const std::string& value) const;
};

// Specializations of convert outside the class
template <>
inline int Params::convert<int>(const std::string& value) const {
    return std::stoi(value);
}

template <>
inline double Params::convert<double>(const std::string& value) const {
    return std::stod(value);
}

template <>
inline std::string Params::convert<std::string>(const std::string& value) const {
    return value;
}

template <>
inline bool Params::convert<bool>(const std::string& value) const {
    if (value == "true" || value == "1") {
        return true;
    } else if (value == "false" || value == "0") {
        return false;
    } else {
        throw std::runtime_error("Invalid boolean value: " + value);
    }
}


// Specialization for std::vector<double>
template <>
inline std::vector<double> Params::convert<std::vector<double>>(const std::string& value) const {
    std::vector<double> result;
    std::stringstream ss(value);
    double num;
    while (ss >> num) {
        result.push_back(num);
        if (ss.peek() == ',' || ss.peek() == ' ') ss.ignore(); // Skip commas/spaces
    }
    return result;
}