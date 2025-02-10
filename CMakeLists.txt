cmake_minimum_required(VERSION 3.10)
project(MotionPlanning)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(nanoflann)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS} 
)

add_library(motion_planning
    src/planners/geometric/fmtx.cpp
    src/planners/planner_factory.cpp
    src/state_space/euclidean_state.cpp
    src/state_space/euclidean_statespace.cpp
    src/ds/tree_node.cpp
    src/utils/nano_flann.cpp
)

target_precompile_headers(motion_planning PRIVATE include/motion_planning/pch.hpp)

target_include_directories(motion_planning PUBLIC include) 
target_link_libraries(motion_planning PUBLIC Eigen3::Eigen nanoflann::nanoflann)




enable_testing()
add_executable(test_fmtx test/unit/test_fmtx.cpp)
target_link_libraries(test_fmtx motion_planning)
add_test(NAME test_fmtx COMMAND test_fmtx)


add_executable(test_kdtree test/unit/test_kdtree.cpp)
target_link_libraries(test_kdtree  motion_planning)
add_test(NAME test_kdtree COMMAND test_kdtree)

add_executable(test_state test/unit/test_state.cpp)
target_link_libraries(test_state  motion_planning)
add_test(NAME test_state COMMAND test_state)