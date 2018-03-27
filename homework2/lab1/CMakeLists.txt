project(lab1)
cmake_minimum_required(VERSION 2.8)

set(EXECUTABLE_1 ${PROJECT_NAME}_simple_operations)
set(EXECUTABLE_2 ${PROJECT_NAME}_voxel_grid_filtering)

add_executable(${EXECUTABLE_1} src/simple_operations.cpp)
target_link_libraries(${EXECUTABLE_1} ${PCL_LIBRARIES})

add_executable(${EXECUTABLE_2} src/simple_operations_b.cpp)
target_link_libraries(${EXECUTABLE_2} ${PCL_LIBRARIES})
