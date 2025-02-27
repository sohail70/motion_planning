
#include "motion_planning/utils/gazebo_obstacle_checker.hpp"


GazeboObstacleChecker::GazeboObstacleChecker(const std::string& robot_model_name,
                                           double obstacle_radius,
                                           const std::string& world_name)
    : robot_model_name_(robot_model_name), 
      obstacle_radius_(obstacle_radius),
      robot_position_(Eigen::Vector2d::Zero()) {
    
    std::string topic = "/world/" + world_name + "/pose/info";
    if (!gz_node_.Subscribe(topic, &GazeboObstacleChecker::poseInfoCallback, this)) {
        std::cerr << "Failed to subscribe to Gazebo topic: " << topic << std::endl;
    }
}

GazeboObstacleChecker::~GazeboObstacleChecker() = default;

bool GazeboObstacleChecker::isObstacleFree(const Eigen::VectorXd& start, 
                                         const Eigen::VectorXd& end) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    Eigen::Vector2d start2d = start.head<2>();
    Eigen::Vector2d end2d = end.head<2>();

    for (const auto& obstacle : obstacle_positions_) {
        if (lineIntersectsCircle(start2d, end2d, obstacle, obstacle_radius_)) {
            return false;
        }
    }
    return true;
}


bool GazeboObstacleChecker::isObstacleFree(const Eigen::VectorXd& point) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Extract the 2D position of the point
    Eigen::Vector2d point2d = point.head<2>();

    // Iterate through all obstacles and check if the point intersects with any of them
    for (const auto& obstacle : obstacle_positions_) {
        if (pointIntersectsCircle(point2d, obstacle, obstacle_radius_)) {
            return false; // The point is inside or on an obstacle
        }
    }
    return true; // The point is obstacle-free
}

Eigen::Vector2d GazeboObstacleChecker::getRobotPosition() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return robot_position_;
}

std::vector<Eigen::Vector2d> GazeboObstacleChecker::getObstaclePositions() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return obstacle_positions_;
}

void GazeboObstacleChecker::poseInfoCallback(const gz::msgs::Pose_V& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    obstacle_positions_.clear();

    for (int i = 0; i < msg.pose_size(); ++i) {
        const auto& pose = msg.pose(i);
        // std::cout << "    Name: " << pose.name() << std::endl;
        Eigen::Vector2d position(pose.position().x(), pose.position().y());

        if (pose.name() == robot_model_name_) {
            robot_position_ = position;

            // // Print pose information
            // std::cout << "  Pose " << i << ":" << std::endl;
            // std::cout << "    Name: " << pose.name() << std::endl;
            // std::cout << "    Position: "
            //         << pose.position().x() << ", "
            //         << pose.position().y() << ", "
            //         << pose.position().z() << std::endl;
            // std::cout << "    Orientation: "
            //         << pose.orientation().w() << ", "
            //         << pose.orientation().x() << ", "
            //         << pose.orientation().y() << ", "
            //         << pose.orientation().z() << std::endl;


            
        } else if (pose.name().find("moving_cylinder_") != std::string::npos) {
            // if the position is in range of the robot then add it to obstalces else don't add it!
            if (use_range==true){
                if ((robot_position_-position).norm() < sensor_range){
                    obstacle_positions_.push_back(position); 
                }
            } else {
                obstacle_positions_.push_back(position);
            }

            // // Print pose information
            // std::cout << "  Pose " << i << ":" << std::endl;
            // std::cout << "    Name: " << pose.name() << std::endl;
            // std::cout << "    Position: "
            //         << pose.position().x() << ", "
            //         << pose.position().y() << ", "
            //         << pose.position().z() << std::endl;
            // std::cout << "    Orientation: "
            //         << pose.orientation().w() << ", "
            //         << pose.orientation().x() << ", "
            //         << pose.orientation().y() << ", "
            //         << pose.orientation().z() << std::endl;

        } else if (pose.name().find("static_cylinder_") != std::string::npos) {

            if (use_range==true){
                if ((robot_position_-position).norm() < sensor_range){
                    obstacle_positions_.push_back(position); 
                }
            } else {
                obstacle_positions_.push_back(position);
            }

        // Check if the name contains "static_cylinder_" (any number after it)
        // obstacle_positions_.push_back(position);

        // // Optional: Print the static cylinder pose information
        // std::cout << "Static cylinder " << pose.name() << " at position: "
        //           << pose.position().x() << ", "
        //           << pose.position().y() << ", "
        //           << pose.position().z() << std::endl;
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