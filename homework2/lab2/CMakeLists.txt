project(lab2)
cmake_minimum_required(VERSION 2.8)

set(EXECUTABLE_1 ${PROJECT_NAME}_compute_normals)
set(EXECUTABLE_2 ${PROJECT_NAME}_compute_FPFH)

add_executable(${EXECUTABLE_1} src/compute_normals.cpp)
target_link_libraries(${EXECUTABLE_1} ${PCL_LIBRARIES})

add_executable(${EXECUTABLE_2} src/compute_FPFH.cpp)
target_link_libraries(${EXECUTABLE_2} ${PCL_LIBRARIES})