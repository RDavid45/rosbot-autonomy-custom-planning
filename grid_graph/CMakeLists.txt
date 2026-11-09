cmake_minimum_required(VERSION 3.16)
project(grid_graph LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(grid_graph src/grid_graph.cpp
        include/grid_graph.hpp)
target_include_directories(grid_graph PUBLIC include)
