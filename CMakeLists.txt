# Top-level build. Mirrors the argos3-examples layout.
# Copy your controller and loop-function CMakeLists.txt into their folders,
# then build with:
#
#   mkdir build && cd build && cmake .. && make
#
cmake_minimum_required(VERSION 3.5)
project(secureflock)

# --- ARGoS ---
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_subdirectory(controllers/footbot_secureflock)
add_subdirectory(loop_functions/secureflock_loop_functions)
