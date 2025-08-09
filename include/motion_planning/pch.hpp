// Copyright 2025 Soheil E.nia
/**
 * TODO: At some point I ignored this in some header files. I need to use this everywhere possible to reduce compile time
 */
#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>   // pulls in the EIGEN_MAKE_ALIGNED_OPERATOR_NEW macro
#include <Eigen/StdVector>   

#include <fstream>
#include <iostream>
#include<vector>
#include<set>
#include<map>
#include<unordered_set>
#include<unordered_map>
#include<queue>
#include<functional>
#include<memory>
#include<string>
#include<utility>
#include<numeric>
#include <cmath>
#include <chrono>
#include <mutex>
#include <stack>
#include <array>

#include <tinyxml2.h>


#include <cstdlib>
#include <ctime>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>


#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>


#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker_array.hpp"


#include <boost/container/flat_map.hpp>
#include <random>