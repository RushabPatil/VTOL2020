cmake_minimum_required( VERSION 3.17)
project(OpenCV_Calibration VERSION 1.0.1)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})
add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBRARIES})



