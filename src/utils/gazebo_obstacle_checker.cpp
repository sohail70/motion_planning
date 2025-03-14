#include "motion_planning/utils/gazebo_obstacle_checker.hpp"

GazeboObstacleChecker::GazeboObstacleChecker(const Params& params,
                        const std::unordered_map<std::string, double>& obstacle_radii)
        : obstacle_radii_(obstacle_radii),
          robot_position_(Eigen::Vector2d::Zero()),
          robot_orientation_(Eigen::VectorXd(4)) {  // Initialize orientation as a 4D vector for quaternion

    robot_model_name_ = params.getParam<std::string>("robot_model_name");
    world_name_ = params.getParam<std::string>("world_name");
    use_range = params.getParam<bool>("use_range");
    sensor_range = params.getParam<double>("sensor_range");
    persistent_static_obstacles = params.getParam<bool>("persistent_static_obstacles");
    robot_position_ << params.getParam<double>("default_robot_x"), params.getParam<double>("default_robot_y");

    // Subscribe to the robot pose topic
    std::string robot_pose_topic = "/model/" + robot_model_name_ + "/tf";
    if (!gz_node_.Subscribe(robot_pose_topic, &GazeboObstacleChecker::robotPoseCallback, this)) {
            std::cerr << "Failed to subscribe to robot pose topic: " << robot_pose_topic << std::endl;
        } else {
            std::cout << "Successfully subscribed to robot pose topic: " << robot_pose_topic << std::endl;
    }
    

    std::string topic = "/world/" + world_name_ + "/pose/info";
    if (!gz_node_.Subscribe(topic, &GazeboObstacleChecker::poseInfoCallback, this)) {
        std::cerr << "Failed to subscribe to Gazebo topic: " << topic << std::endl;
    }


    path_pub_ = gz_node_.Advertise<gz::msgs::Pose_V>("/path");
    if (!path_pub_) {
        std::cerr << "Failed to advertise /path topic." << std::endl;
    } else {
        std::cout << "Successfully advertised /path topic." << std::endl;
    }



}

GazeboObstacleChecker::~GazeboObstacleChecker() = default;

// Method to publish a path
void GazeboObstacleChecker::publishPath(const std::vector<Eigen::VectorXd>& waypoints) {
    gz::msgs::Pose_V path_msg;

    for (const auto& waypoint : waypoints) {
        gz::msgs::Pose* pose = path_msg.add_pose();
        pose->mutable_position()->set_x(waypoint.x());
        pose->mutable_position()->set_y(waypoint.y());
        pose->mutable_position()->set_z(0.0);  // Assuming 2D path
    }

    // Publish the path
    if (!path_pub_.Publish(path_msg)) {
        std::cerr << "Failed to publish path to /path topic." << std::endl;
    }
}




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

Eigen::VectorXd GazeboObstacleChecker::getRobotEulerAngles() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return quaternionToEuler(robot_orientation_);
}

Eigen::VectorXd GazeboObstacleChecker::quaternionToEuler(const Eigen::VectorXd& quaternion) const {
    // Ensure the quaternion is of size 4 (x, y, z, w)
    if (quaternion.size() != 4) {
        throw std::invalid_argument("Quaternion must be a 4D vector (x, y, z, w).");
    }

    // Extract quaternion components
    double x = quaternion(0);
    double y = quaternion(1);
    double z = quaternion(2);
    double w = quaternion(3);

    // Convert quaternion to Euler angles (roll, pitch, yaw)
    double roll, pitch, yaw;

    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1) {
        pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    } else {
        pitch = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    // Return Euler angles as a 3D vector
    Eigen::VectorXd euler_angles(3);
    euler_angles << roll, pitch, yaw;
    return euler_angles;
}



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

double GazeboObstacleChecker::calculateYawFromQuaternion(const Eigen::VectorXd& quaternion) {
    // Ensure the quaternion is valid (x, y, z, w)
    if (quaternion.size() != 4) {
        throw std::invalid_argument("Quaternion must be a 4D vector (x, y, z, w).");
    }

    // Extract quaternion components
    double x = quaternion[0];
    double y = quaternion[1];
    double z = quaternion[2];
    double w = quaternion[3];

    // Convert quaternion to yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}



void GazeboObstacleChecker::robotPoseCallback(const gz::msgs::Pose_V& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (msg.pose_size() > 0) {
        const auto& pose = msg.pose(0);

        // Update robot position
        // robot_position_ = Eigen::Vector2d(pose.position().x(), pose.position().y());

        // Update robot orientation (quaternion)
        robot_orientation_ = Eigen::VectorXd(4);
        robot_orientation_ << pose.orientation().x(),
                              pose.orientation().y(),
                              pose.orientation().z(),
                              pose.orientation().w();

        // Calculate yaw from quaternion for debug!
        // double yaw = calculateYawFromQuaternion(robot_orientation_);
        // std::cout<< "ROBOT YAW: " << yaw <<"\n";
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




std::vector<Obstacle> GazeboObstacleChecker::getObstacles() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<Obstacle> filtered_obstacles;
    
    for (const auto& obstacle : obstacle_positions_) {
        if (use_range) {
            // Calculate distance from robot to obstacle
            double distance = (robot_position_ - obstacle.position).norm();
            
            // Only include obstacles within sensor range
            if (distance <= sensor_range) {
                filtered_obstacles.push_back(obstacle);
            }
        } else {
            // Include all obstacles if range checking is disabled
            filtered_obstacles.push_back(obstacle);
        }
    }
    
    return filtered_obstacles;
}
