#pragma once

#include "rclcpp/rclcpp.hpp"
#include "motion_planning/utils/rviz_visualization.hpp" // For visualization
#include "motion_planning/ds/edge_info.hpp"
#include "motion_planning/utils/params.hpp" // Assuming this defines your Params class

#include <Eigen/Dense>
#include <Eigen/Geometry> // For Eigen::Quaternion, Eigen::AngleAxisd
#include <mutex>          // For std::mutex, std::lock_guard
#include <chrono>         // For std::chrono::milliseconds
#include <functional>     // For std::bind
#include <algorithm>      // For std::clamp
#include <sstream>        // For printing vectors



// Helper function definitions
inline Eigen::VectorXd getSpatialPosition(const Eigen::VectorXd& full_state) {
    int D_spatial_dim = (full_state.size() - 1) / 2;
    return full_state.head(D_spatial_dim);
}

inline Eigen::VectorXd getSpatialVelocity(const Eigen::VectorXd& full_state) {
    int D_spatial_dim = (full_state.size() - 1) / 2;
    return full_state.segment(D_spatial_dim, D_spatial_dim);
}


class ROS2Manager : public rclcpp::Node {
public:
    // Constructor
    ROS2Manager(
        std::shared_ptr<RVizVisualization> visualizer,
        const Params& params)
        : Node("ros2_manager",
               rclcpp::NodeOptions().parameter_overrides(
                   std::vector<rclcpp::Parameter>{
                       rclcpp::Parameter("use_sim_time", params.getParam<bool>("use_sim_time", false))
                   }
               )),
          visualizer_(visualizer),
          current_kinodynamic_state_(Eigen::VectorXd::Zero(params.getParam<int>("thruster_state_dimension", 7))),
          current_exec_traj_idx_(0),
          simulation_time_step_(params.getParam<double>("simulation_time_step", 0.01))
    {
        RCLCPP_INFO(this->get_logger(), "Initialized ROS2Manager for Thruster Control (Header-Only).");

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<long int>(simulation_time_step_ * 1000)),
            std::bind(&ROS2Manager::thrusterControlLoop, this));

        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ROS2Manager::updateCallback, this));
    }

    // Public API
    void setInitialState(const Eigen::VectorXd& state) {
        std::lock_guard<std::mutex> lock(planned_traj_mutex_);
        RCLCPP_INFO(this->get_logger(), "Setting initial state to: %s", printEigenVec(state).c_str());
        if (state.size() != current_kinodynamic_state_.size()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot set initial state, dimension mismatch! Manager state size=%ld, provided state size=%ld",
                current_kinodynamic_state_.size(), state.size());
            return;
        }
        current_kinodynamic_state_ = state;
    }

    void setPlannedThrusterTrajectory(const ExecutionTrajectory& traj) {
        std::lock_guard<std::mutex> lock(planned_traj_mutex_);
        current_planned_execution_trajectory_ = traj;
        current_exec_traj_idx_ = 0;
        RCLCPP_INFO(this->get_logger(), "New execution trajectory set with %zu time steps. Total time: %.2f s", traj.Time.size(), traj.total_cost);
    }

private:
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    std::shared_ptr<RVizVisualization> visualizer_;
    Eigen::VectorXd current_kinodynamic_state_;
    std::vector<Eigen::VectorXd> robot_spatial_trace_;
    ExecutionTrajectory current_planned_execution_trajectory_;
    std::mutex planned_traj_mutex_;
    size_t current_exec_traj_idx_;
    double simulation_time_step_;

    std::string printEigenVec(const Eigen::VectorXd& vec) {
        std::stringstream ss;
        ss << "[" << vec.transpose() << "]";
        return ss.str();
    }

    void thrusterControlLoop() {
        std::lock_guard<std::mutex> traj_lock(planned_traj_mutex_);

        const size_t traj_len = current_planned_execution_trajectory_.Time.size();

        if (!current_planned_execution_trajectory_.is_valid || traj_len < 2) {
            return; 
        }

        RCLCPP_INFO(this->get_logger(), "[CTL_LOOP] Current state: %s", printEigenVec(current_kinodynamic_state_).c_str());

        int D_spatial_dim = (current_kinodynamic_state_.size() - 1) / 2;
        double current_sim_time = current_kinodynamic_state_[current_kinodynamic_state_.size() - 1];

        while (current_exec_traj_idx_ < traj_len - 1 &&
               current_sim_time >= current_planned_execution_trajectory_.Time[current_exec_traj_idx_ + 1] - 1e-9) {
            current_exec_traj_idx_++;
        }

        if (current_exec_traj_idx_ >= traj_len - 1) {
            RCLCPP_INFO(this->get_logger(), "[CTL_LOOP] Execution trajectory finished. Robot stopping.");
            current_planned_execution_trajectory_.is_valid = false;
            current_kinodynamic_state_.segment(D_spatial_dim, D_spatial_dim).setZero();
            return;
        }

        Eigen::VectorXd a_k = current_planned_execution_trajectory_.A.row(current_exec_traj_idx_).transpose();
        double t_k_plus_1_planned_end = current_planned_execution_trajectory_.Time[current_exec_traj_idx_ + 1];
        
        double dt_integrate = simulation_time_step_;
        if (current_sim_time + dt_integrate > t_k_plus_1_planned_end + 1e-9) {
            dt_integrate = t_k_plus_1_planned_end - current_sim_time;
            if (dt_integrate < 0) dt_integrate = 0;
        }
        
        Eigen::VectorXd current_spatial_pos = getSpatialPosition(current_kinodynamic_state_);
        Eigen::VectorXd current_spatial_vel = getSpatialVelocity(current_kinodynamic_state_);
        Eigen::VectorXd new_spatial_pos = current_spatial_pos + current_spatial_vel * dt_integrate + (0.5 * a_k.array() * dt_integrate * dt_integrate).matrix();
        Eigen::VectorXd new_spatial_vel = current_spatial_vel + a_k * dt_integrate;
        double new_sim_time = current_sim_time + dt_integrate;

        current_kinodynamic_state_.head(D_spatial_dim) = new_spatial_pos;
        current_kinodynamic_state_.segment(D_spatial_dim, D_spatial_dim) = new_spatial_vel;
        current_kinodynamic_state_[current_kinodynamic_state_.size() - 1] = new_sim_time;

        Eigen::VectorXd robot_orientation_quat = Eigen::VectorXd::Zero(4); 
        if (D_spatial_dim >= 2 && new_spatial_vel.norm() > 1e-6) {
            double yaw = std::atan2(new_spatial_vel[1], new_spatial_vel[0]);
            Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
            robot_orientation_quat << q.x(), q.y(), q.z(), q.w();
        } else {
            Eigen::Quaterniond q_id(1,0,0,0); 
            robot_orientation_quat << q_id.x(), q_id.y(), q_id.z(), q_id.w();
        }
        visualizer_->visualizeRobotArrow(new_spatial_pos, robot_orientation_quat, "map", {0.5f, 0.5f, 0.5f}, "thruster_robot_marker");
        
        robot_spatial_trace_.push_back(new_spatial_pos); 
        visualizer_->visualizeTrajectories({robot_spatial_trace_}, "map", {1.0f, 0.0f, 1.0f}, "robot_trace"); 

        std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> current_segment_edges;
        current_segment_edges.emplace_back(
            current_planned_execution_trajectory_.X.row(current_exec_traj_idx_).transpose(),
            current_planned_execution_trajectory_.X.row(current_exec_traj_idx_ + 1).transpose()
        );
        visualizer_->visualizeEdges(current_segment_edges, "map", "1.0,1.0,0.0", "active_planned_segment");
    }

    void updateCallback() {}
};