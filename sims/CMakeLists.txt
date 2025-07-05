cmake_minimum_required(VERSION 3.10)
project(MovingCylinderPlugin)

# Find the required Gazebo dependencies
find_package(gz-sim8 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-plugin2 REQUIRED)

# Define the source file for your plugin
#add_library(MoverPlugin SHARED gz_move.cc)
add_library(MoverPlugin SHARED gz_move_constant.cc)
add_library(CirclePlugin SHARED gz_circle.cc)
add_library(PathFollowerPlugin SHARED path_follow.cc)
add_library(TeleportPlugin SHARED teleport.cc)
# Link the necessary Gazebo and Ignition libraries
target_link_libraries(MoverPlugin
    gz-sim8::core
    gz-math7::gz-math7
    gz-plugin2::loader
)
target_link_libraries(CirclePlugin
    gz-sim8::core
    gz-math7::gz-math7
    gz-plugin2::loader
)

target_link_libraries(PathFollowerPlugin
    gz-sim8::core
    gz-math7::gz-math7
    gz-plugin2::loader
)

target_link_libraries(TeleportPlugin
    gz-sim8::core
    gz-math7::gz-math7
    gz-plugin2::loader
)
# Install the plugin to the appropriate plugin path
install(TARGETS MoverPlugin
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(TARGETS CirclePlugin
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(TARGETS PathFollowerPlugin
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
# Include the necessary directories for Gazebo and Ignition
include_directories(${gz-sim8_INCLUDE_DIRS} ${gz-math7_INCLUDE_DIRS})

