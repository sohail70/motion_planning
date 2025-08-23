// Copyright 2025 Soheil E.nia


// #include "motion_planning/pch.hpp"
#include <unordered_map>
#include <tinyxml2.h>
#include <sstream>

// struct ObstacleInfo {
//     enum Type { CYLINDER, BOX };
//     Type type;
//     double radius;    // For cylinders
//     double width;     // For boxes
//     double height;    // For boxes
// };

std::unordered_map<std::string, ObstacleInfo> parseSdfObstacles(const std::string& sdf_path) {
    std::unordered_map<std::string, ObstacleInfo> obstacles;
    tinyxml2::XMLDocument doc;
    
    // Attempt to load the file; return empty map on failure.
    if (doc.LoadFile(sdf_path.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "Error: Could not load SDF file from path: " << sdf_path << std::endl;
        return obstacles;
    }

    const auto* world = doc.RootElement()->FirstChildElement("world");
    if (!world) {
        std::cerr << "Error: No <world> element found in the SDF file." << std::endl;
        return obstacles;
    }

    // Iterate through all <model> elements in the world.
    for (const auto* model = world->FirstChildElement("model"); model; 
         model = model->NextSiblingElement("model")) {
        
        const char* name = model->Attribute("name");
        if (!name) continue; // Skip models without a name attribute.

        const std::string model_name(name);
        ObstacleInfo info;

        const auto* link = model->FirstChildElement("link");
        if (!link) continue;

        // Prioritize <collision> geometry, but fall back to <visual> if not present.
        const auto* geometry = link->FirstChildElement("collision") 
                             ? link->FirstChildElement("collision")->FirstChildElement("geometry")
                             : link->FirstChildElement("visual")->FirstChildElement("geometry");
        
        if (!geometry) continue;

        // --- PARSING LOGIC ---
        // Check for each geometry type explicitly instead of relying on the model name.
        
        const auto* box_geom = geometry->FirstChildElement("box");
        const auto* cylinder_geom = geometry->FirstChildElement("cylinder");
        const auto* sphere_geom = geometry->FirstChildElement("sphere");

        if (box_geom) {
            info.type = ObstacleInfo::BOX;
            if (const auto* size = box_geom->FirstChildElement("size")) {
                std::stringstream ss(size->GetText());
                double x, y, z;
                
                // Read all three dimensions for 3D correctness,
                // but only store x and y for backward compatibility.
                if (ss >> x >> y >> z) {
                    if (x <= 0 || y <= 0) {
                        std::cerr << "Warning: Invalid box dimensions for '" << name 
                                  << "': " << x << "x" << y << std::endl;
                        continue;
                    }
                    info.width = x;  // Store x-size in 'width'
                    info.height = y; // Store y-size in 'height'
                } else {
                    std::cerr << "Warning: Failed to parse box size for '" << name 
                              << "': " << size->GetText() << std::endl;
                    continue;
                }
            }
        } else if (cylinder_geom) {
            info.type = ObstacleInfo::CYLINDER;
            // Both cylinders and spheres are represented by a radius in 2D.
            cylinder_geom->FirstChildElement("radius")->QueryDoubleText(&info.radius);
        } else if (sphere_geom) {
            info.type = ObstacleInfo::CYLINDER;
            // Treat a sphere as a circle in 2D, so we just need the radius.
            sphere_geom->FirstChildElement("radius")->QueryDoubleText(&info.radius);
        } else {
            // This model is not a recognized obstacle type, so we skip it.
            continue;
        }

        // If we successfully parsed an obstacle, add it to the map.
        obstacles[model_name] = info;
    }
    
    return obstacles;
}