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

        // 3. Extract deterministic obstacle-motion metadata. Legacy motion
        // variants carry a type attribute and are intentionally ignored here.
        if (const auto* motion = model->FirstChildElement("motion");
            motion && !motion->Attribute("type")) {
            if (const auto* speed = motion->FirstChildElement("speed"))
                speed->QueryDoubleText(&info.speed);

            if (const auto* amplitude = motion->FirstChildElement("amplitude"))
                amplitude->QueryDoubleText(&info.amplitude);

            info.is_dynamic = info.speed > 1e-6 && info.amplitude > 1e-6;

            if (const auto* direction = motion->FirstChildElement("direction")) {
                std::stringstream ss(direction->GetText());
                double dx, dy, dz;
                if (ss >> dx >> dy >> dz)
                    info.direction << dx, dy, dz;
            }
        }

        obstacles[model_name] = info;
    }
    
    return obstacles;
}