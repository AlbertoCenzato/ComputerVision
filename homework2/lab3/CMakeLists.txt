project(lab3)
cmake_minimum_required(VERSION 2.8)

include_directories(include)

set(EXECUTABLE_1 ${PROJECT_NAME}_icp_registration)
add_executable(${EXECUTABLE_1} src/icp_registration.cpp)
target_link_libraries(${EXECUTABLE_1} ${PCL_LIBRARIES})

set(EXECUTABLE_2 ${PROJECT_NAME}_complete_registration)
add_executable(${EXECUTABLE_2} src/complete_registration.cpp)
target_link_libraries(${EXECUTABLE_2} ${PCL_LIBRARIES})