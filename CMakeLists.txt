project(stereo_vision)
cmake_minimum_required(VERSION 2.8)
find_package(PCL REQUIRED)

file(GLOB HEADERS src/*.hpp)
file(GLOB SOURCES src/*.cpp)

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
link_directories(${PCL_LIBRARY_DIRS} ${OPENCV_LIBRARY_DIRS})

aux_source_directory(src SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OPENCV_LIBRARIES})
