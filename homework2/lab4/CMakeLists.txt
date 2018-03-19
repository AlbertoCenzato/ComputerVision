project(lab4)
cmake_minimum_required(VERSION 2.8)

set(EXECUTABLE_1 ${PROJECT_NAME}_people_detection)

add_executable(${EXECUTABLE_1} src/ground_based_people_detection.cpp)
target_link_libraries(${EXECUTABLE_1} ${PCL_LIBRARIES})