cmake_minimum_required(VERSION 3.10)
project(MotionPlanning)
# add_compile_options(-g -O0)  # Force debug symbols and disable optimizations
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE RelWithDebInfo)
 set(CMAKE_BUILD_TYPE Debug)

find_package(Eigen3 REQUIRED)
find_package(nanoflann REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(visualization_msgs REQUIRED) 
find_package(nav_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED) 
find_package(tf2_ros REQUIRED) 
find_package(type_description_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)  # Add this line
find_package(rclcpp_action REQUIRED)  # Add this line

find_package(gz-transport13 REQUIRED)   # Gazebo transport
find_package(gz-msgs10 REQUIRED)        # Gazebo messages
find_package(Protobuf REQUIRED)         # Protocol Buffers
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZeroMQ REQUIRED libzmq)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS} 
  ${rclcpp_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${gz-transport13_INCLUDE_DIRS}
  ${gz-msgs10_INCLUDE_DIRS}
  ${gz-utils2_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${ZeroMQ_INCLUDE_DIRS}
  ${type_description_interfaces_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

# Visualization Library (ROS2 dependent)
add_library(visualization
    src/utils/rviz_visualization.cpp
)

target_precompile_headers(visualization PRIVATE include/motion_planning/pch.hpp)
target_include_directories(visualization PUBLIC include)
target_link_libraries(visualization PUBLIC gz-utils2::gz-utils2)
ament_target_dependencies(visualization PUBLIC rclcpp visualization_msgs nav2_msgs rclcpp_action) 

# Gazebo Obstacle Checker Library
add_library(gz_obs src/utils/gazebo_obstacle_checker.cpp)
target_precompile_headers(gz_obs PRIVATE include/motion_planning/pch.hpp)
target_link_libraries(gz_obs
  gz-transport13
  gz-msgs10
  ${Protobuf_LIBRARIES}
  ${ZeroMQ_LIBRARIES}
)

# Motion Planning Library
add_library(motion_planning
    src/planners/geometric/fmt.cpp
    src/planners/geometric/any_fmt.cpp
    src/planners/geometric/fmta.cpp
    src/planners/geometric/informed_any_fmt.cpp
    src/planners/geometric/fmtx.cpp
    src/planners/geometric/rrtx.cpp
    src/planners/geometric/bit_star.cpp
    src/planners/planner_factory.cpp
    src/state_space/euclidean_state.cpp
    src/state_space/euclidean_statespace.cpp
    src/ds/fmt_node.cpp
    src/ds/rrtx_node.cpp
    src/utils/nano_flann.cpp
)

target_precompile_headers(motion_planning PRIVATE include/motion_planning/pch.hpp)
target_include_directories(motion_planning PUBLIC include) 
target_link_libraries(motion_planning PUBLIC Eigen3::Eigen nanoflann::nanoflann visualization)

# Tests
enable_testing()
add_executable(test_fmtx test/unit/test_fmtx.cpp)
target_link_libraries(test_fmtx motion_planning gz_obs)
ament_target_dependencies(test_fmtx rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_fmtx COMMAND test_fmtx)

add_executable(test_fmta test/unit/test_fmta.cpp)
target_link_libraries(test_fmta motion_planning gz_obs)
ament_target_dependencies(test_fmta rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_fmta COMMAND test_fmta)

add_executable(test_fmt test/unit/test_fmt.cpp)
target_link_libraries(test_fmt motion_planning gz_obs)
ament_target_dependencies(test_fmt rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_fmt COMMAND test_fmt)

add_executable(test_anyfmt test/unit/test_anyfmt.cpp)
target_link_libraries(test_anyfmt motion_planning gz_obs)
ament_target_dependencies(test_anyfmt rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_anyfmt COMMAND test_anyfmt)


add_executable(test_informed_anyfmt test/unit/test_informed_anyfmt.cpp)
target_link_libraries(test_informed_anyfmt motion_planning gz_obs)
ament_target_dependencies(test_informed_anyfmt rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_informed_anyfmt COMMAND test_informed_anyfmt)


add_executable(test_rrtx test/unit/test_rrtx.cpp)
target_link_libraries(test_rrtx motion_planning gz_obs)
ament_target_dependencies(test_rrtx rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_rrtx COMMAND test_rrtx)

add_executable(test_bitstar test/unit/test_bitstar.cpp)
target_link_libraries(test_bitstar motion_planning gz_obs)
ament_target_dependencies(test_bitstar rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_bitstar COMMAND test_bitstar)


add_executable(test_kdtree test/unit/test_kdtree.cpp)
target_link_libraries(test_kdtree motion_planning)
add_test(NAME test_kdtree COMMAND test_kdtree)

# add_executable(test_state test/unit/test_state.cpp)
# target_link_libraries(test_state motion_planning)
# add_test(NAME test_state COMMAND test_state)

# add_executable(test_visual test/unit/test_visual.cpp)
# target_link_libraries(test_visual visualization)
# ament_target_dependencies(test_visual rclcpp visualization_msgs nav2_msgs rclcpp_action)  
# add_test(NAME test_visual COMMAND test_visual)

# add_executable(test_occupancy test/unit/test_occupancy.cpp)
# target_link_libraries(test_occupancy visualization motion_planning gz_obs)
# ament_target_dependencies(test_occupancy rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action) 
# add_test(NAME test_occupancy COMMAND test_occupancy)

add_executable(test_gz test/unit/test_gz.cpp)
target_link_libraries(test_gz gz_obs visualization motion_planning)
ament_target_dependencies(test_gz rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_gz COMMAND test_gz)