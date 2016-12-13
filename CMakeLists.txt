project(stereo_vision)
cmake_minimum_required(VERSION 2.8)
find_package(PCL REQUIRED)

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
link_directories(${PCL_LIBRARY_DIRS} ${OPENCV_LIBRARY_DIRS})

#LIBS += -L /usr/local/lib/opencv-3.1/debug/ -llibopencv_core.so.3.1.0

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OPENCV_LIBRARIES})
