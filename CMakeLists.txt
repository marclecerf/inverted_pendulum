cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
include(FindPkgConfig)
set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:/usr/local/lib/pkgconfig")

# Find Gazebo
find_package(gazebo REQUIRED)
configure_file(launch.in launch @ONLY)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(cart_plugin SHARED Source/cart_plugin.cc)
target_link_libraries(cart_plugin ${GAZEBO_LIBRARIES})

add_executable(test Source/test.cc)
target_link_libraries(test ${GAZEBO_LIBRARIES})

