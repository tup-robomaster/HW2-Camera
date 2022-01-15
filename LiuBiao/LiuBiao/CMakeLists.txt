# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Define project name
project(camera_calibration)

set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV 4.5.0)

message(STATUS "OpenCV library status:")
message(STATUS "	config: ${OpenCV_DIR}")
message(STATUS "	version: ${OpenCV_VERSION}")
message(STATUS "	libraries: ${OpenCV_LIBS}")
message(STATUS "	include path: ${OpenCV_INCLUDE_DIRS}")

# Declare the executable target build from your sources
add_executable(camera main.cpp)

#link your application with OpencvCV libraries
target_link_libraries(camera PRIVATE ${OpenCV_LIBS})
