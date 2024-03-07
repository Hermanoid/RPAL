cmake_minimum_required(VERSION 3.1)
project(first_project)
set(CMAKE_CXX_STANDARD 17)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# include_directories(include/)

add_executable(eightpoint src/eightpoint.cpp)
add_executable(backline src/backline.cpp)

# add_library(icp src/icp.cpp)

# target_link_libraries(main Eigen3::Eigen)
# target_link_libraries(main icp)