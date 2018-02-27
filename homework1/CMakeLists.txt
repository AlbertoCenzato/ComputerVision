project(stereo_vision)
cmake_minimum_required(VERSION 2.8)


# find OpenCV 3.4
if(DEFINED ENV{OPENCV_DIR})
    find_package(OpenCV 3.2 REQUIRED PATHS $ENV{OPENCV_DIR})
else()
    message("OPENCV_DIR not set, searching in default location(s)")
    find_package(OpenCV 3.2 REQUIRED)
endif(DEFINED ENV{OPENCV_DIR})

message(STATUS "OpenCV version: ${OpenCV_VERSION}")

find_package(PCL REQUIRED)

file(GLOB EXTRA_MODULE extra-module/*.cpp)
file(GLOB SRC src/*.cpp)

add_definitions(${PCL_DEFINITIONS})
include_directories(include extra_module ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} ${SRC} ${EXTRA_MODULE})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
