// Copyright 2025 Soheil E.nia

#pragma once


#include "motion_planning/utils/visualization.hpp"

class RVizVisualization : public Visualization {
public:
    RVizVisualization(rclcpp::Node::SharedPtr node, const std::string& marker_topic = "fmtx_markers");

    void visualizeNodes(const std::vector<Eigen::VectorXd>& nodes, const std::string& frame_id = "map") override;
    void visualizeNodes(const std::vector<Eigen::VectorXd>& nodes, const std::string& frame_id, const std::vector<float>& color, const std::string& ns) override;
 
    void visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id = "map") override;
    void visualizeEdges(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges, const std::string& frame_id, const std::string& color_str) override;

    void visualizeCylinder(const std::vector<Eigen::VectorXd>& obstacles, const std::vector<double>& radii, const std::string& frame_id , const std::vector<float>& color , const std::string& ns);

    void visualizeRobotArrow( const Eigen::VectorXd& robot_position, const Eigen::VectorXd& robot_orientation, const std::string& frame_id, const std::vector<float>& color,const std::string& ns);
    void visualizeCube(const std::vector<std::tuple<Eigen::Vector2d, double, double, double>>& box_obstacles, const std::string& frame_id, const std::vector<float>& color, const std::string& ns) ;


    void visualizeTrajectories(const std::vector<std::vector<Eigen::Vector2d>>& trajectories, 
                          const std::string& frame_id,
                          const std::vector<float>& color,
                          const std::string& ns);
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_2_;
    int marker_id_counter_;

};