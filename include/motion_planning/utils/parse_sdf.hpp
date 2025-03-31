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
    
    if (doc.LoadFile(sdf_path.c_str()) != tinyxml2::XML_SUCCESS) return obstacles;

    const auto* world = doc.RootElement()->FirstChildElement("world");
    if (!world) return obstacles;

    for (const auto* model = world->FirstChildElement("model"); model; 
         model = model->NextSiblingElement("model")) {
        
        const char* name = model->Attribute("name");
        if (!name) continue;

        ObstacleInfo info;
        const std::string model_name(name);
        bool is_cylinder = model_name.find("cylinder") != std::string::npos;
        bool is_box = model_name.find("box") != std::string::npos;

        if (!is_cylinder && !is_box) continue;

        const auto* link = model->FirstChildElement("link");
        if (!link) continue;

        // Get geometry from collision or visual
        const auto* geometry = link->FirstChildElement("collision") 
                             ? link->FirstChildElement("collision")->FirstChildElement("geometry")
                             : link->FirstChildElement("visual")->FirstChildElement("geometry");
        
        if (!geometry) continue;

        if (is_cylinder) {
            info.type = ObstacleInfo::CYLINDER;
            if (const auto* cylinder = geometry->FirstChildElement("cylinder")) {
                cylinder->FirstChildElement("radius")->QueryDoubleText(&info.radius);
            }
        } 


        else if (is_box) {
            info.type = ObstacleInfo::BOX;
            if (const auto* box = geometry->FirstChildElement("box")) {
                if (const auto* size = box->FirstChildElement("size")) {
                    std::stringstream ss(size->GetText());
                    double x, y, z;
                    
                    // Read space-separated values without explicit delimiters
                    if (ss >> x >> y >> z) {  // Changed this line
                        // Add validation
                        if (x <= 0 || y <= 0) {
                            std::cerr << "Invalid box dimensions for " << name 
                                    << ": " << x << "x" << y << std::endl;
                            continue;
                        }
                        info.width = x;
                        info.height = y;
                    } else {
                        std::cerr << "Failed to parse box size for " << name 
                                << ": " << size->GetText() << std::endl;
                    }
                }
            }
        }

        obstacles[model_name] = info;
    }
    
    return obstacles;
}