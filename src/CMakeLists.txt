cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(OpenCV REQUIRED)
find_library(wiringPi_LIB wiringPi)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(car_line car_line.cpp)
target_link_libraries(car_line ${OpenCV_LIBS})
target_link_libraries(car_line ${wiringPi_LIB})

