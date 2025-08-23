// Copyright 2025 Soheil E.nia

#pragma once


#include "motion_planning/utils/visualization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry> // Required for Eigen::Quaterniond and AngleAxisd

#include <vector>
#include <string>
#include "motion_planning/utils/obstacle_checker.hpp"
class RVizVisualization : public Visualization {
public:
    RVizVisualization(rclcpp::Node::SharedPtr node, const std::string& marker_topic = "fmtx_markers");

    void visualizeNodes(const std::vector<Eigen::VectorXd>& nodes, const std::string& frame_id = "map") override;
    void visualizeNodes(const std::vector<Eigen::VectorXd>& nodes, const std::string& frame_id, const std::vector<float>& color, const std::string& ns) override;
 
    void visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id = "map") override;
    void visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id, const std::string& color_str) override;
    void visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id, const std::string& color_str,const std::string& ns) override;

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


private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_2_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_3_;
    int marker_id_counter_;

};