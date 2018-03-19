project(lab4)
cmake_minimum_required(VERSION 2.8)

set(EXECUTABLE_1 ${PROJECT_NAME}_people_detection)
set(EXECUTABLE_2 ${PROJECT_NAME}_plane_segmentation)

add_executable(${EXECUTABLE_1} src/people_detection.cpp)
target_link_libraries(${EXECUTABLE_1} ${PCL_LIBRARIES})

add_executable(${EXECUTABLE_2} src/plane_segmentation.cpp)
target_link_libraries(${EXECUTABLE_2} ${PCL_LIBRARIES})