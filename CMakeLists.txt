cmake_minimum_required(VERSION 3.14)
project(TrackingLibrary)

# Set standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# === Find OpenCV ===
find_package(OpenCV REQUIRED)

# === Find Yaml ===
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})

# === Find Eigen ===
find_path(EIGEN3_INCLUDE_DIR Eigen/Dense PATH_SUFFIXES eigen3)
if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Eigen3 not found! Please install libeigen3-dev.")
endif()

# === Include directories ===
include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

# === Source files ===
file(GLOB_RECURSE TRACKING_SOURCES "${PROJECT_SOURCE_DIR}/src/*.cpp")

# === Create static library ===
add_library(tracking STATIC ${TRACKING_SOURCES})

# === Example executable ===
add_executable(example_tracker examples/tracker_example.cpp)
target_link_libraries(example_tracker tracking ${OpenCV_LIBS} yaml-cpp)
