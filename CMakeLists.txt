cmake_minimum_required(VERSION 3.14)
project(tracker-cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(tracker-cpp ${OpenCV_LIBS})

include_directories("C:/msys64/mingw64/include/eigen3")

add_executable(tracker-cpp src/main.cpp)
 