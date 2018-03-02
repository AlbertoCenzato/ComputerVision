project(lab3)
cmake_minimum_required(VERSION 2.8)

set(EXECUTABLE_1 ${PROJECT_NAME}_pairwise_registration)
add_executable(${EXECUTABLE_1} src/pairwise_registration.cpp)
target_link_libraries(${EXECUTABLE_1} ${PCL_LIBRARIES})

set(EXECUTABLE_2 ${PROJECT_NAME}_dataset_visualizer)
add_executable(${EXECUTABLE_2} src/dataset_visualizer.cpp)
target_link_libraries(${EXECUTABLE_2} ${PCL_LIBRARIES})