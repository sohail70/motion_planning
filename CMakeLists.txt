cmake_minimum_required(VERSION 3.10)
project(MotionPlanning)

set(CMAKE_CXX_STANDARD 17)


add_library(motion_planning
    src/planners/geometric/fmtx.cpp
    src/planners/planner_factory.cpp
    src/state_space/euclidean_state.cpp
    src/state_space/euclidean_statespace.cpp
    src/ds/tree_node.cpp
)

target_precompile_headers(motion_planning PRIVATE include/motion_planning/pch.hpp)

target_include_directories(motion_planning PUBLIC include) 

enable_testing()
add_executable(test_fmtx test/unit/test_fmtx.cpp)
target_link_libraries(test_fmtx motion_planning)
add_test(NAME test_fmtx COMMAND test_fmtx)