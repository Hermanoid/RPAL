cmake_minimum_required(VERSION 3.1)
project(first_project)
set(CMAKE_CXX_STANDARD 17)

include_directories(include/)

add_library(print_test src/print_test.cpp)

add_executable(main src/program.cpp)

target_link_libraries(main print_test)