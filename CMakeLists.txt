cmake_minimum_required(VERSION 3.10)
project(MotionPlanning)
# add_compile_options(-g -O0)  # Force debug symbols and disable optimizations
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE RelWithDebInfo)
# set(CMAKE_BUILD_TYPE Debug)



# AddressSanitizer for debugging heap corruption
if(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  message(STATUS "Debug build: Enabling AddressSanitizer")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
  set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")
endif()


find_package(Eigen3 REQUIRED)
find_package(nanoflann REQUIRED)

find_package(OpenMP REQUIRED)


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


add_library(kf     
  src/utils/kalman_filter.cpp
  src/utils/kalman_filter_factory.cpp
)
target_include_directories(kf PUBLIC include)
target_link_libraries(kf PUBLIC Eigen3::Eigen)



# Gazebo Obstacle Checker Library
add_library(gz_obs src/utils/gazebo_obstacle_checker.cpp)
target_precompile_headers(gz_obs PRIVATE include/motion_planning/pch.hpp)
target_link_libraries(gz_obs
  gz-transport13
  gz-msgs10
  ${Protobuf_LIBRARIES}
  ${ZeroMQ_LIBRARIES}
  kf
)



# Motion Planning Library
add_library(motion_planning
    src/planners/geometric/fmt.cpp
    src/planners/geometric/any_fmt.cpp
    src/planners/geometric/fmta.cpp
    src/planners/geometric/informed_any_fmt.cpp
    src/planners/geometric/informed_any_fmta.cpp
    src/planners/geometric/fmtx.cpp
    src/planners/kinodynamic/kinodynamic_fmtx.cpp
    src/planners/kinodynamic/kinodynamic_rrtx.cpp
    src/planners/geometric/rrtx.cpp
    src/planners/geometric/bit_star.cpp
    src/planners/planner_factory.cpp
    src/state_space/euclidean_state.cpp
    src/state_space/euclidean_statespace.cpp
    src/state_space/rdt_statespace.cpp
    src/state_space/dubins_statespace.cpp
    src/state_space/dubins_time_statespace.cpp
    src/state_space/thruster_statespace.cpp
    src/ds/fmt_node.cpp
    src/ds/ifmt_node.cpp
    src/ds/bit_node.cpp
    src/ds/rrtx_node.cpp
    src/utils/nano_flann.cpp
    src/utils/weighted_nano_flann.cpp
    src/utils/lie_kd_tree.cpp
)

target_precompile_headers(motion_planning PRIVATE include/motion_planning/pch.hpp)
target_include_directories(motion_planning PUBLIC include) 
target_link_libraries(motion_planning PUBLIC Eigen3::Eigen nanoflann::nanoflann visualization)
if(OpenMP_CXX_FOUND)
  target_compile_options(motion_planning PUBLIC "${OpenMP_CXX_FLAGS}")
  target_link_libraries(motion_planning PUBLIC OpenMP::OpenMP_CXX)
endif()


# Tests
enable_testing()
add_executable(test_fmtx test/unit/test_fmtx.cpp)
target_link_libraries(test_fmtx motion_planning gz_obs)
ament_target_dependencies(test_fmtx rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_fmtx COMMAND test_fmtx)

# add_executable(test_fmta test/unit/test_fmta.cpp)
# target_link_libraries(test_fmta motion_planning gz_obs)
# ament_target_dependencies(test_fmta rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
# add_test(NAME test_fmta COMMAND test_fmta)

add_executable(test_fmt test/unit/test_fmt.cpp)
target_link_libraries(test_fmt motion_planning gz_obs)
ament_target_dependencies(test_fmt rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_fmt COMMAND test_fmt)

add_executable(test_anyfmt test/unit/test_anyfmt.cpp)
target_link_libraries(test_anyfmt motion_planning gz_obs)
ament_target_dependencies(test_anyfmt rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_anyfmt COMMAND test_anyfmt)


# add_executable(test_informed_anyfmt test/unit/test_informed_anyfmt.cpp)
# target_link_libraries(test_informed_anyfmt motion_planning gz_obs)
# ament_target_dependencies(test_informed_anyfmt rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
# add_test(NAME test_informed_anyfmt COMMAND test_informed_anyfmt)

add_executable(test_informed_anyfmta test/unit/test_informed_anyfmta.cpp)
target_link_libraries(test_informed_anyfmta motion_planning gz_obs)
ament_target_dependencies(test_informed_anyfmta rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_informed_anyfmta COMMAND test_informed_anyfmta)

add_executable(test_bitstar test/unit/test_bitstar.cpp)
target_link_libraries(test_bitstar motion_planning gz_obs)
ament_target_dependencies(test_bitstar rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_bitstar COMMAND test_bitstar)

add_executable(test_rrtx test/unit/test_rrtx.cpp)
target_link_libraries(test_rrtx motion_planning gz_obs)
ament_target_dependencies(test_rrtx rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_rrtx COMMAND test_rrtx)


# add_executable(test_kdtree test/unit/test_kdtree.cpp)
# target_link_libraries(test_kdtree motion_planning)
# add_test(NAME test_kdtree COMMAND test_kdtree)

add_executable(test_weighted_kdtree test/unit/test_weighted_kdtree.cpp)
target_link_libraries(test_weighted_kdtree motion_planning)
add_test(NAME test_weighted_kdtree COMMAND test_weighted_kdtree)

add_executable(test_lie_kdtree test/unit/test_lie_kdtree.cpp)
target_link_libraries(test_lie_kdtree motion_planning)
add_test(NAME test_lie_kdtree COMMAND test_lie_kdtree)


add_executable(test_gz test/unit/test_gz.cpp)
target_link_libraries(test_gz gz_obs visualization motion_planning)
ament_target_dependencies(test_gz rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_gz COMMAND test_gz)











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

#################### KINODYNAMIC TESTS #############################
# add_executable(test_kinodynamic_fmtx test/unit/test_kinodynamic_fmtx.cpp)
# target_link_libraries(test_kinodynamic_fmtx motion_planning gz_obs)
# ament_target_dependencies(test_kinodynamic_fmtx rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
# add_test(NAME test_kinodynamic_fmtx COMMAND test_kinodynamic_fmtx)

add_executable(test_kinodynamic_fmtx_R2T test/unit/test_kinodynamic_fmtx_R2T.cpp)
target_link_libraries(test_kinodynamic_fmtx_R2T motion_planning gz_obs)
ament_target_dependencies(test_kinodynamic_fmtx_R2T rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_kinodynamic_fmtx_R2T COMMAND test_kinodynamic_fmtx_R2T)

add_executable(test_kinodynamic_fmtx_dubin_4D test/unit/test_kinodynamic_fmtx_dubin_4D.cpp)
target_link_libraries(test_kinodynamic_fmtx_dubin_4D motion_planning gz_obs)
ament_target_dependencies(test_kinodynamic_fmtx_dubin_4D rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_kinodynamic_fmtx_dubin_4D COMMAND test_kinodynamic_fmtx_dubin_4D)


add_executable(test_kinodynamic_fmtx_thruster_5D test/unit/test_kinodynamic_fmtx_thruster_5D.cpp)
target_link_libraries(test_kinodynamic_fmtx_thruster_5D motion_planning gz_obs)
ament_target_dependencies(test_kinodynamic_fmtx_thruster_5D rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_kinodynamic_fmtx_thruster_5D COMMAND test_kinodynamic_fmtx_thruster_5D)




add_executable(test_kinodynamic_rrtx_R2T test/unit/test_kinodynamic_rrtx_R2T.cpp)
target_link_libraries(test_kinodynamic_rrtx_R2T motion_planning gz_obs)
ament_target_dependencies(test_kinodynamic_rrtx_R2T rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_kinodynamic_rrtx_R2T COMMAND test_kinodynamic_rrtx_R2T)


add_executable(test_kinodynamic_rrtx_dubin_4D test/unit/test_kinodynamic_rrtx_dubin_4D.cpp)
target_link_libraries(test_kinodynamic_rrtx_dubin_4D motion_planning gz_obs)
ament_target_dependencies(test_kinodynamic_rrtx_dubin_4D rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_kinodynamic_rrtx_dubin_4D COMMAND test_kinodynamic_rrtx_dubin_4D)



add_executable(test_kinodynamic_rrtx_thruster_5D test/unit/test_kinodynamic_rrtx_thruster_5D.cpp)
target_link_libraries(test_kinodynamic_rrtx_thruster_5D motion_planning gz_obs)
ament_target_dependencies(test_kinodynamic_rrtx_thruster_5D rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_kinodynamic_rrtx_thruster_5D COMMAND test_kinodynamic_rrtx_thruster_5D)








add_executable(test_dubin_time test/unit/test_dubin_time.cpp)
target_link_libraries(test_dubin_time motion_planning gz_obs)
ament_target_dependencies(test_dubin_time rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_dubin_time COMMAND test_dubin_time)

add_executable(test_thruster test/unit/test_thruster.cpp)
target_link_libraries(test_thruster motion_planning gz_obs)
ament_target_dependencies(test_thruster rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_thruster COMMAND test_thruster)



add_executable(test_collision test/unit/test_collision.cpp)
target_link_libraries(test_collision gz_obs visualization motion_planning)
ament_target_dependencies(test_collision rclcpp nav_msgs visualization_msgs tf2_ros geometry_msgs nav2_msgs rclcpp_action)
add_test(NAME test_collision COMMAND test_collision)

