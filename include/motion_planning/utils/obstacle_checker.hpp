// Copyright 2025 Soheil E.nia

#pragma once

#include <memory>
#include "motion_planning/pch.hpp"
#include "motion_planning/ds/edge_info.hpp"


#include "rclcpp/rclcpp.hpp" 

// This struct replaces the union, making the object safe to copy.
struct ObstacleDimensions {
    double radius = 0.0;
    double width = 0.0;
    double height = 0.0;
    double rotation = 0.0;
};

struct Obstacle {
    // THIS IS THE CRITICAL FIX for Eigen memory alignment issues when
    // this struct is used in STL containers like std::vector.
    /*
        When you store Obstacle in an STL container (like ObstacleVector) and then copy it, 
        Eigen’s fixed‑size vectorizable types (Eigen::Vector2d, etc.) need 16‑byte alignment — but by 
        default std::vector will only honor the alignment requirements of the element type itself, 
        which for a plain new’d Obstacle isn’t enough once Eigen can choose vectorized code paths. 
        The result: your element copies into unaligned storage, stomping past the end of the allocation.
    */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int turnaround_step_index = -1;
    int steps_per_cycle = 0; // Add this


    std::string name;
    enum Type { CIRCLE, BOX };
    Type type;
    Eigen::Vector2d position;
    Eigen::Vector2d previous_position;
    Eigen::Vector2d velocity;
    Eigen::Vector2d acceleration;
    rclcpp::Time last_update_time;
    
    // Lets for easier simulation assume 3D obstalce has constant z and move in XY plane (or else i have to change lots of things)
    bool is_3d = false; // If an obstalce lives in 3D space (x,y,z)
    double z = 0.0; // TODO: unfortunately right now i dont have time to integrate 3D to the position, velocity and acceleration so lets just use another varibale

    // The union has been replaced with this struct.
    ObstacleDimensions dimensions;

    double inflation;
    bool is_dynamic = false;

    // Ground Truth Fields for Deterministic Prediction
    bool has_ground_truth = false;
    Eigen::Vector2d motion_axis = Eigen::Vector2d::UnitX(); 
    Eigen::Vector2d initial_origin = Eigen::Vector2d::Zero();
    double speed_scalar = 0.0;
    double motion_limit = 0.0;


    bool is_initialized_in_graph = false; // For the first time obstalce moves we need to inform the graph with this!


    // This is the TIME BUDGET's "guess" currently marked as blocked in the KD-tree
    std::vector<Eigen::Vector3d> predicted_path; 

    // The "Timer": Ground truth time (T) when the next turn occurs
    double nextDirectionChangeTime = -1.0;

    // --- NEW: AABB for the Spacetime Tube ---
    // These define the rectangular bounds of the entire predicted_path
    // Mark these as mutable so generatePrediction can update them
    mutable double min_x = std::numeric_limits<double>::max();
    mutable double max_x = std::numeric_limits<double>::lowest();
    mutable double min_y = std::numeric_limits<double>::max();
    mutable double max_y = std::numeric_limits<double>::lowest();


    // Default constructor
    Obstacle() : type(CIRCLE), position(Eigen::Vector2d::Zero()), previous_position(Eigen::Vector2d::Zero()), inflation(0.0) {}


    // Circle constructor
    Obstacle(Eigen::Vector2d pos, double rad, double infl, bool dynamic = false)
        : type(CIRCLE), position(pos), inflation(infl), is_dynamic(dynamic) {
        dimensions.radius = rad;
    }

    // Box constructor
    Obstacle(Eigen::Vector2d pos, double w, double h, double rot, double infl, bool dynamic = false)
        : type(BOX), position(pos), inflation(infl), is_dynamic(dynamic) {
        dimensions.width = w;
        dimensions.height = h;
        dimensions.rotation = rot;
    }

    // With the union removed, the default compiler-generated copy constructor
    // and copy assignment operator are now safe and correct. We no longer
    // need to write our own, which removes the source of the memory error.
    
    bool operator==(const Obstacle& other) const {
        if (type != other.type || position != other.position || inflation != other.inflation)
            return false;
        
        if (type == CIRCLE) {
            return dimensions.radius == other.dimensions.radius;
        }
        // BOX
        return dimensions.width == other.dimensions.width &&
               dimensions.height == other.dimensions.height &&
               dimensions.rotation == other.dimensions.rotation;
    }
};
using ObstacleVector = std::vector<Obstacle, Eigen::aligned_allocator<Obstacle>>;

struct ObstacleInfo {
    enum Type { CYLINDER, BOX };
    Type type;
    double radius = 0.0;    // For cylinders (initialize!)
    double width = 0.0;     // For boxes (initialize!)
    double height = 0.0;    // For boxes (initialize!)


    // --- Fields for Ground Truth Prediction ---
    bool is_dynamic = false;
    double speed = 0.0;
    double amplitude = 0.0;
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
    Eigen::Vector3d initial_pose = Eigen::Vector3d::Zero(); // To know the center of oscillation


};

// Operator<< definition OUTSIDE the struct
inline std::ostream& operator<<(std::ostream& os, const ObstacleInfo& info) {
    if (info.type == ObstacleInfo::CYLINDER) {
        os << "Cylinder(r=" << info.radius << ")";
    } else {
        os << "Box(w=" << info.width << ", h=" << info.height << ")";
    }
    return os;
}

class ObstacleChecker {
public:
    virtual ~ObstacleChecker() = default;

    // Pure virtual method for obstacle checking
    virtual bool isObstacleFree(const Eigen::VectorXd& start, const Eigen::VectorXd& end) const = 0;
    virtual bool isObstacleFree(const Eigen::VectorXd& point) const = 0;
    // New virtual function to check a whole path
    virtual bool isObstacleFree(const std::vector<Eigen::VectorXd>& path) const = 0;

    virtual bool isObstacleFreeAgainstSingleObstacle(const Eigen::VectorXd& start, const Eigen::VectorXd& end, const Obstacle& obs) const = 0;
    virtual bool isObstacleFreeAgainstSingleObstacle(const Eigen::VectorXd& point, const Obstacle& obs) const = 0;

    virtual bool isTrajectorySafe( const Trajectory& trajectory) const = 0;
    
    virtual bool isTrajectorySafeAgainstSingleObstacle(const Trajectory& trajectory, const Obstacle& obstacle) const = 0;

    // virtual void updateGrid(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid) = 0;
    virtual const ObstacleVector& getObstacles() const = 0;
    virtual int getObstaclesSize() const = 0;
                                       

    virtual std::vector<Eigen::Vector3d> generatePrediction(const Obstacle& ob, double current_time) const {
        // Default behavior: return empty path (no prediction)
        return {}; 
    }
    
    virtual bool isNodeInObstacleTube(const Eigen::VectorXd& node_state, const Obstacle& ob, double max_edge_length) const {
        return 0;
    }

};
