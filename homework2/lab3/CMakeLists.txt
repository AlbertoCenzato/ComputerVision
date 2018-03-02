project(lab3)
cmake_minimum_required(VERSION 2.8)

set(EXECUTABLE_1 ${PROJECT_NAME}_pairwise_registration)
add_executable(${EXECUTABLE_1} src/pairwise_registration.cpp)
target_link_libraries(${EXECUTABLE_1} ${PCL_LIBRARIES})