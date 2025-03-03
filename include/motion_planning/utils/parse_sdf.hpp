// #include "motion_planning/pch.hpp"


#include <unordered_map>
#include <tinyxml2.h>
#include <iostream>

std::unordered_map<std::string, double> parseSdfForObstacleRadii(const std::string& sdf_path) {
    std::unordered_map<std::string, double> obstacle_radii;
    tinyxml2::XMLDocument doc;
    
    // Load the SDF file
    if (doc.LoadFile(sdf_path.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "Failed to load SDF file: " << sdf_path << std::endl;
        return obstacle_radii;
    }

    // Get the root <sdf> element
    const auto* root = doc.RootElement();
    if (!root) {
        std::cerr << "Invalid SDF file: No root <sdf> element." << std::endl;
        return obstacle_radii;
    }

    // Get the <world> element
    const auto* world = root->FirstChildElement("world");
    if (!world) {
        std::cerr << "Invalid SDF file: No <world> element." << std::endl;
        return obstacle_radii;
    }

    // Iterate through all <model> elements in the world
    for (const auto* model = world->FirstChildElement("model"); model; 
         model = model->NextSiblingElement("model")) {
        
        // Get the model name
        const char* name = model->Attribute("name");
        if (!name) continue;

        const std::string model_name(name);

        // Check if it's a static or moving cylinder
        const bool is_static = model_name.find("static_cylinder_") != std::string::npos;
        const bool is_moving = model_name.find("moving_cylinder_") != std::string::npos;
        
        if (!is_static && !is_moving) continue;

        // Get the first <link> element (regardless of name)
        const auto* link = model->FirstChildElement("link");
        if (!link) continue;

        // Get the <visual> element
        const auto* visual = link->FirstChildElement("visual");
        if (!visual) continue;

        // Get the <geometry> element inside <visual>
        const auto* geometry = visual->FirstChildElement("geometry");
        if (!geometry) continue;

        // Get the <cylinder> element inside <geometry>
        const auto* cylinder = geometry->FirstChildElement("cylinder");
        if (!cylinder) continue;

        // Get the <radius> element inside <cylinder>
        const auto* radius_elem = cylinder->FirstChildElement("radius");
        if (!radius_elem) continue;

        // Parse the radius value
        double radius = 0.0;
        if (radius_elem->QueryDoubleText(&radius) != tinyxml2::XML_SUCCESS) {
            std::cerr << "Failed to parse radius for model: " << model_name << std::endl;
            continue;
        }

        // Store the radius in the map
        obstacle_radii[model_name] = radius;
    }
    
    return obstacle_radii;
}