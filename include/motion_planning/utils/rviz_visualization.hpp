// Copyright 2025 Soheil E.nia

#pragma once


#include "motion_planning/utils/visualization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry> // for Eigen::Quaterniond and AngleAxisd
#include <algorithm> // for std::clamp
#include <vector>
#include <string>
#include "motion_planning/utils/obstacle_checker.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
class RVizVisualization : public Visualization {
public:
    RVizVisualization(rclcpp::Node::SharedPtr node, const std::string& marker_topic = "fmtx_markers");

    void visualizeNodes(const std::vector<Eigen::VectorXd>& nodes, const std::string& frame_id = "map") override;
    void visualizeNodes(const std::vector<Eigen::VectorXd>& nodes, const std::string& frame_id, const std::vector<float>& color, const std::string& ns) override;
 
    void visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id = "map") override;
    void visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id, const std::string& color_str) override;
    void visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id, const std::string& color_str,const std::string& ns) override;

    void visualizeEdges( const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id, const std::array<float,3>& color, float alpha, float line_width, const std::string& ns, int marker_id, bool dashed = false, double dash_length = 0.5);
    void visualizeCylinder(const std::vector<Eigen::VectorXd>& obstacles, const std::vector<double>& radii, const std::string& frame_id , const std::vector<float>& color , const std::string& ns);
    void visualizeSpheres( const std::vector<Eigen::VectorXd>& obstacles_positions, const std::vector<double>& radii, const std::string& frame_id, const std::vector<float>& color, const std::string& ns);

    void visualizeRobotArrow( const Eigen::VectorXd& robot_position, const Eigen::VectorXd& robot_orientation, const std::string& frame_id, const std::vector<float>& color,const std::string& ns);
    void visualizeQuadcopter( const Eigen::Vector3d& position, const Eigen::VectorXd& orientation_quat, const std::string& frame_id, const std::vector<float>& color, const std::string& ns);
    void visualizeCube(const std::vector<std::tuple<Eigen::Vector2d, double, double, double>>& box_obstacles, const std::string& frame_id, const std::vector<float>& color, const std::string& ns) ;
    void visualizeCube( const std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, double>>& box_obstacles, const std::string& frame_id, const std::vector<float>& color, const std::string& ns);


    void visualizeTrajectories(const std::vector<std::vector<Eigen::Vector2d>>& trajectories, 
                          const std::string& frame_id,
                          const std::vector<float>& color,
                          const std::string& ns);
    void visualizeTrajectories(const std::vector<std::vector<Eigen::VectorXd>>& trajectories, 
                          const std::string& frame_id,
                          const std::vector<float>& color,
                          const std::string& ns);
    void visualizeSingleEdge(const Eigen::VectorXd& start_point, const Eigen::VectorXd& end_point, int edge_id, const std::string& frame_id) override;


    void visualizeFutureGhosts( const ObstacleVector& obstacles, double prediction_horizon, const std::string& frame_id);

    void visualizeVelocityVectors( const std::vector<Eigen::Vector2d>& positions, const std::vector<Eigen::Vector2d>& velocities, const std::string& frame_id, const std::vector<float>& color, const std::string& ns);
    void visualizeVelocityVectors( const std::vector<Eigen::Vector3d>& positions, const std::vector<Eigen::Vector2d>& velocities, const std::string& frame_id, const std::vector<float>& color, const std::string& ns);


    void visualizeCircle( const Eigen::Vector2d& center, double radius, const std::string& frame_id, const std::vector<float>& color, const std::string& ns);

    void visualizeText( const std::vector<Eigen::Vector3d>& points, const std::vector<std::string>& texts, const std::string& frame_id, const std::string& ns) override;

    void visualizeLineToNearest( const Eigen::Vector3d& robot_pos, const Eigen::Vector3d& nearest_obs_pos, const std::string& frame_id, const std::vector<float>& color, const std::string& ns);

    void visualizeDottedLineToNearest( const Eigen::Vector3d& robot_pos, const Eigen::Vector3d& nearest_obs_pos, int num_points, const std::string& frame_id, const std::vector<float>& color, const std::string& ns);

    void clearMarkers(const std::string& ns = "");

    void visualizePathGradient( const std::vector<Eigen::VectorXd>& path_waypoints, const std::vector<double>& waypoint_costs, const std::string& frame_id, double global_max_cost); 
    void visualizeContinuousMesh( const std::vector<Eigen::VectorXd>& points, const std::vector<double>& costs, double global_max_cost, double neighborhood_radius, const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds, const std::string& frame_id = "map");

    void visualizeAxes( const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds, double step_size = 5.0, const std::string& frame_id = "map");
    void visualizeTimeToGoal(double t_robot, double x = -45.0, double y = 45.0);
    void takeScreenshot(double t_robot, bool is_final = false);
    
    void visualizeTreeGradient( const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::vector<double>& costs, const std::string& frame_id);
    void publishObstacleFrame(
        const std::vector<Eigen::VectorXd>& safe_cyls, const std::vector<double>& safe_radii,
        const std::vector<Eigen::VectorXd>& threat_cyls, const std::vector<double>& threat_radii,
        const std::vector<std::tuple<Eigen::Vector2d, double, double, double>>& safe_boxes,
        const std::vector<std::tuple<Eigen::Vector2d, double, double, double>>& threat_boxes,
        const std::vector<Eigen::Vector2d>& safe_vel_pos, const std::vector<Eigen::Vector2d>& safe_vel_val,
        const std::vector<Eigen::Vector2d>& threat_vel_pos, const std::vector<Eigen::Vector2d>& threat_vel_val,
        
        // --- NEW PARAMETERS FOR ROBOT ---
        const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& robot_trace_edges,
        const Eigen::Vector3d& robot_pos,
        const Eigen::VectorXd& robot_orientation, // Quaternion
        const std::vector<float>& robot_color,
        double robot_inflation,
        
        const std::string& frame_id);
    
    void triggerPublish();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_2_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_3_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    visualization_msgs::msg::MarkerArray marker_buffer_; // Holds markers until publish time
    int marker_id_counter_;

};