// Copyright 2025 Soheil E.nia


#include <unordered_map>
#include <tinyxml2.h>
#include <sstream>
#include <Eigen/Dense>

std::unordered_map<std::string, ObstacleInfo> parseSdfObstacles(const std::string& sdf_path) {
    std::unordered_map<std::string, ObstacleInfo> obstacles;
    tinyxml2::XMLDocument doc;
    
    if (doc.LoadFile(sdf_path.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "Error: Could not load SDF file: " << sdf_path << std::endl;
        return obstacles;
    }

    const auto* world = doc.RootElement()->FirstChildElement("world");
    if (!world) return obstacles;

    for (const auto* model = world->FirstChildElement("model"); model; 
         model = model->NextSiblingElement("model")) {
        
        const char* name = model->Attribute("name");
        if (!name) continue;

        const std::string model_name(name);
        ObstacleInfo info;

        // 1. Extract Initial Pose (Start point for oscillation)
        if (const auto* pose = model->FirstChildElement("pose")) {
            std::stringstream ss(pose->GetText());
            double x, y, z;
            if (ss >> x >> y >> z) {
                info.initial_pose << x, y, z;
            }
        }

        // 2. Parse Geometry (Existing Logic)
        const auto* link = model->FirstChildElement("link");
        if (!link) continue;

        const auto* geometry = link->FirstChildElement("collision") 
                             ? link->FirstChildElement("collision")->FirstChildElement("geometry")
                             : link->FirstChildElement("visual")->FirstChildElement("geometry");
        
        if (!geometry) continue;

        const auto* box_geom = geometry->FirstChildElement("box");
        const auto* cylinder_geom = geometry->FirstChildElement("cylinder");
        const auto* sphere_geom = geometry->FirstChildElement("sphere");

        if (box_geom) {
            info.type = ObstacleInfo::BOX;
            if (const auto* size = box_geom->FirstChildElement("size")) {
                std::stringstream ss(size->GetText());
                double x, y, z;
                if (ss >> x >> y >> z) {
                    info.width = x;
                    info.height = y;
                }
            }
        } else if (cylinder_geom || sphere_geom) {
            info.type = ObstacleInfo::CYLINDER;
            const auto* target = cylinder_geom ? cylinder_geom : sphere_geom;
            target->FirstChildElement("radius")->QueryDoubleText(&info.radius);
        } else {
            continue; 
        }

        // 3. Extract Plugin Data (MoverPluginC)
        for (const auto* plugin = model->FirstChildElement("plugin"); plugin; 
             plugin = plugin->NextSiblingElement("plugin")) {
            
            const char* plugin_name = plugin->Attribute("name");
            if (plugin_name && std::string(plugin_name) == "MoverPluginC") {
                info.is_dynamic = true;
                
                // Parse Speed
                if (auto* s = plugin->FirstChildElement("speed")) 
                    s->QueryDoubleText(&info.speed);
                
                // Parse Amplitude
                if (auto* a = plugin->FirstChildElement("amplitude")) 
                    a->QueryDoubleText(&info.amplitude);
                
                // Parse Direction Vector
                if (auto* d = plugin->FirstChildElement("direction")) {
                    std::stringstream ss(d->GetText());
                    double dx, dy, dz;
                    if (ss >> dx >> dy >> dz) {
                        info.direction << dx, dy, dz;
                    }
                }
            }
        }

        obstacles[model_name] = info;
    }
    
    return obstacles;
}