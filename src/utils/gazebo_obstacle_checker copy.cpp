
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"






GazeboObstacleChecker::GazeboObstacleChecker(const std::string& robot_model_name,
                        const std::unordered_map<std::string, double>& obstacle_radii,
                        const std::string& world_name)
        : robot_model_name_(robot_model_name),
          obstacle_radii_(obstacle_radii),
          robot_position_(Eigen::Vector2d::Zero()) {

    std::string topic = "/world/" + world_name + "/pose/info";
    if (!gz_node_.Subscribe(topic, &GazeboObstacleChecker::poseInfoCallback, this)) {
        std::cerr << "Failed to subscribe to Gazebo topic: " << topic << std::endl;
    }

}


GazeboObstacleChecker::~GazeboObstacleChecker() = default;

bool GazeboObstacleChecker::isObstacleFree(const Eigen::VectorXd& start, const Eigen::VectorXd& end) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    Eigen::Vector2d start2d = start.head<2>();
    Eigen::Vector2d end2d = end.head<2>();

    for (const auto& obstacle : obstacle_positions_) {
        if (lineIntersectsCircle(start2d, end2d, obstacle.position, obstacle.radius)) {
            return false;
        }
    }
    return true;
}

bool GazeboObstacleChecker::isObstacleFree(const Eigen::VectorXd& point) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    Eigen::Vector2d point2d = point.head<2>();

    for (const auto& obstacle : obstacle_positions_) {
        if (pointIntersectsCircle(point2d, obstacle.position, obstacle.radius)) {
            return false;
        }
    }
    return true;
}

Eigen::Vector2d GazeboObstacleChecker::getRobotPosition() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return robot_position_;
}

Eigen::VectorXd GazeboObstacleChecker::getRobotOrientation() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return robot_orientation_;

}

std::vector<Obstacle> GazeboObstacleChecker::getObstaclePositions() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return obstacle_positions_;
}

// void GazeboObstacleChecker::poseInfoCallback(const gz::msgs::Pose_V& msg) {
//     std::lock_guard<std::mutex> lock(data_mutex_);
//     obstacle_positions_.clear();

//     for (int i = 0; i < msg.pose_size(); ++i) {
//         const auto& pose = msg.pose(i);
//         // std::cout << "    Name: " << pose.name() << std::endl;
//         Eigen::Vector2d position(pose.position().x(), pose.position().y());

//         if (pose.name() == robot_model_name_) {
//             robot_position_ = position;

//             // // Print pose information
//             // std::cout << "  Pose " << i << ":" << std::endl;
//             // std::cout << "    Name: " << pose.name() << std::endl;
//             // std::cout << "    Position: "
//             //         << pose.position().x() << ", "
//             //         << pose.position().y() << ", "
//             //         << pose.position().z() << std::endl;
//             // std::cout << "    Orientation: "
//             //         << pose.orientation().w() << ", "
//             //         << pose.orientation().x() << ", "
//             //         << pose.orientation().y() << ", "
//             //         << pose.orientation().z() << std::endl;


            
//         } else if (pose.name().find("moving_cylinder_") != std::string::npos) {
//             // if the position is in range of the robot then add it to obstalces else don't add it!
//             if (use_range==true){
//                 if ((robot_position_-position).norm() < sensor_range){
//                     obstacle_positions_.push_back(position); 
//                 }
//             } else {
//                 obstacle_positions_.push_back(position);
//             }

//             // // Print pose information
//             // std::cout << "  Pose " << i << ":" << std::endl;
//             // std::cout << "    Name: " << pose.name() << std::endl;
//             // std::cout << "    Position: "
//             //         << pose.position().x() << ", "
//             //         << pose.position().y() << ", "
//             //         << pose.position().z() << std::endl;
//             // std::cout << "    Orientation: "
//             //         << pose.orientation().w() << ", "
//             //         << pose.orientation().x() << ", "
//             //         << pose.orientation().y() << ", "
//             //         << pose.orientation().z() << std::endl;

//         } else if (pose.name().find("static_cylinder_") != std::string::npos) {

//             if (use_range==true){
//                 if ((robot_position_-position).norm() < sensor_range){
//                     obstacle_positions_.push_back(position); 
//                 }
//             } else {
//                 obstacle_positions_.push_back(position);
//             }

//         // Check if the name contains "static_cylinder_" (any number after it)
//         // obstacle_positions_.push_back(position);

//         // // Optional: Print the static cylinder pose information
//         // std::cout << "Static cylinder " << pose.name() << " at position: "
//         //           << pose.position().x() << ", "
//         //           << pose.position().y() << ", "
//         //           << pose.position().z() << std::endl;
//         }


    
//     }
// }


//     // Modify your callback to use the radius map
// void GazeboObstacleChecker::poseInfoCallback(const gz::msgs::Pose_V& msg) {
//         std::lock_guard<std::mutex> lock(data_mutex_);
//         obstacle_positions_.clear();

//         for (int i = 0; i < msg.pose_size(); ++i) {
//             const auto& pose = msg.pose(i);
//             Eigen::Vector2d position(pose.position().x(), pose.position().y());
//             const std::string name = pose.name();

//             if (name == robot_model_name_) {
//                 robot_position_ = position;
//             }
//             else if (name.find("moving_cylinder_") != std::string::npos || 
//                     name.find("static_cylinder_") != std::string::npos) {
                
//                 // Look up the radius from our map
//                 auto it = obstacle_radii_.find(name);
//                 if (it != obstacle_radii_.end()) {
//                     Obstacle obstacle;
//                     obstacle.position = position;
//                     obstacle.radius = it->second;
                    
//                     if (use_range && (robot_position_ - position).norm() < sensor_range) {
//                         obstacle_positions_.push_back(obstacle);
//                     }
//                     else if (!use_range) {
//                         obstacle_positions_.push_back(obstacle);
//                     }
//                 } else { //Default radius!
//                     Obstacle obstacle;
//                     obstacle.position = position;
//                     obstacle.radius = 5;
                    
//                     if (use_range && (robot_position_ - position).norm() < sensor_range) {
//                         obstacle_positions_.push_back(obstacle);
//                     }
//                     else if (!use_range) {
//                         obstacle_positions_.push_back(obstacle);
//                     }
//                 }
//             }
//         }
//     }


// void GazeboObstacleChecker::poseInfoCallback(const gz::msgs::Pose_V& msg) {
//     std::lock_guard<std::mutex> lock(data_mutex_);
//     obstacle_positions_.clear();  // Clear current obstacles

//     // Track dynamic obstacles in this callback
//     std::vector<Obstacle> current_dynamic_obstacles;

//     // Update robot position
//     for (int i = 0; i < msg.pose_size(); ++i) {
//         const auto& pose = msg.pose(i);
//         if (pose.name() == robot_model_name_) {
//             robot_position_ = Eigen::Vector2d(pose.position().x(), pose.position().y());
//             break;
//         }
//     }

//     // Process obstacles
//     for (int i = 0; i < msg.pose_size(); ++i) {
//         const auto& pose = msg.pose(i);
//         const std::string name = pose.name();
//         Eigen::Vector2d position(pose.position().x(), pose.position().y());

//         if (name == robot_model_name_) continue;

//         bool is_static = name.find("static_cylinder_") != std::string::npos;
//         bool is_moving = name.find("moving_cylinder_") != std::string::npos;

//         if (!is_static && !is_moving) continue;

//         // Get or assign radius
//         double radius = 5.0;
//         auto radius_it = obstacle_radii_.find(name);
//         if (radius_it != obstacle_radii_.end()) {
//             radius = radius_it->second;
//         }

//         Obstacle obstacle{position, radius};

//         // Check if within sensor range
//         bool within_range = !use_range || (robot_position_ - position).norm() < sensor_range;

//         if (is_static) {
//             // Check if the static obstacle has been detected before
//             auto it = static_obstacle_positions_.find(name);
//             if (it == static_obstacle_positions_.end()) {
//                 // First time detection: add to static_obstacle_positions_ if within range
//                 if (within_range) {
//                     static_obstacle_positions_[name] = obstacle;
//                 }
//             } else {
//                 // Update position if already detected
//                 it->second.position = position;
//             }
//         } else if (is_moving && within_range) {
//             current_dynamic_obstacles.push_back(obstacle);
//         }
//     }

//     // Add dynamic obstacles
//     obstacle_positions_.insert(obstacle_positions_.end(), current_dynamic_obstacles.begin(), current_dynamic_obstacles.end());

//     // Add all previously detected static obstacles to obstacle_positions_
//     for (const auto& [name, static_obs] : static_obstacle_positions_) {
//         obstacle_positions_.push_back(static_obs);
//     }
// }


void GazeboObstacleChecker::poseInfoCallback(const gz::msgs::Pose_V& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    obstacle_positions_.clear();  // Clear current obstacles

    // Track dynamic obstacles in this callback
    std::vector<Obstacle> current_dynamic_obstacles;

    // Update robot position
    for (int i = 0; i < msg.pose_size(); ++i) {
        const auto& pose = msg.pose(i);
        if (pose.name() == robot_model_name_) {
            robot_position_ = Eigen::Vector2d(pose.position().x(), pose.position().y());
            break;
        }
    }

    // Process obstacles
    for (int i = 0; i < msg.pose_size(); ++i) {
        const auto& pose = msg.pose(i);
        const std::string name = pose.name();
        Eigen::Vector2d position(pose.position().x(), pose.position().y());

        if (name == robot_model_name_) continue;

        bool is_static = name.find("static_cylinder_") != std::string::npos;
        bool is_moving = name.find("moving_cylinder_") != std::string::npos;

        if (!is_static && !is_moving) continue;

        // Get or assign radius
        double radius = 5.0;
        auto radius_it = obstacle_radii_.find(name);
        if (radius_it != obstacle_radii_.end()) {
            radius = radius_it->second;
        }

        Obstacle obstacle{position, radius};

        // Check if within sensor range
        bool within_range = !use_range || (robot_position_ - position).norm() < sensor_range;

        if (is_static) {
            if (persistent_static_obstacles) {
                // Check if the static obstacle has been detected before
                auto it = static_obstacle_positions_.find(name);
                if (it == static_obstacle_positions_.end()) {
                    // First time detection: add to static_obstacle_positions_ if within range
                    if (within_range) {
                        static_obstacle_positions_[name] = obstacle;
                        obstacle_positions_.push_back(obstacle);
                    }
                } else {
                    // Update position if already detected
                    it->second.position = position;
                }
            } else {
                // Treat static obstacles like dynamic obstacles
                if (within_range) {
                    obstacle_positions_.push_back(obstacle);
                }
            }
        } else if (is_moving && within_range) {
            current_dynamic_obstacles.push_back(obstacle);
        }
    }

    // Add dynamic obstacles
    obstacle_positions_.insert(obstacle_positions_.end(), current_dynamic_obstacles.begin(), current_dynamic_obstacles.end());

    // Add all previously detected static obstacles to obstacle_positions_ if persistent_static_obstacles is true
    if (persistent_static_obstacles) {
        for (const auto& [name, static_obs] : static_obstacle_positions_) {
            obstacle_positions_.push_back(static_obs);
        }
    }
}



bool GazeboObstacleChecker::lineIntersectsCircle(const Eigen::Vector2d& start,
                                                 const Eigen::Vector2d& end,
                                                 const Eigen::Vector2d& center,
                                                 double radius) {
    const Eigen::Vector2d d = end - start;
    const Eigen::Vector2d f = start - center;
    
    const double a = d.dot(d);
    const double b = 2 * f.dot(d);
    const double c = f.dot(f) - radius * radius;

    double discriminant = b * b - 4 * a * c;
    
    // If the discriminant is negative, no intersection
    if (discriminant < 0) return false;

    // Compute the square root of the discriminant
    discriminant = std::sqrt(discriminant);

    // Calculate the parametric intersection points
    const double t1 = (-b - discriminant) / (2 * a);
    const double t2 = (-b + discriminant) / (2 * a);

    // Check if either intersection point lies within the bounds of the segment
    return (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1);
}

bool GazeboObstacleChecker::pointIntersectsCircle(const Eigen::Vector2d& point,
                                                  const Eigen::Vector2d& center,
                                                  double radius) {
    // Calculate the squared distance between the point and the center of the circle
    const double squaredDistance = (point - center).squaredNorm();

    // Check if the squared distance is less than or equal to the squared radius
    return squaredDistance <= (radius * radius);
}