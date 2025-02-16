cmake_minimum_required(VERSION 3.7)   # VS_STARTUP_PROPERTY

project(redblob)

add_executable(1_breadth_first            breadth_first.cpp)
add_executable(2_breadth_first_came_from  breadth_first_came_from.cpp)
add_executable(3_breadth_first_early_exit breadth_first_early_exit.cpp)
add_executable(4_dijkstra                 dijkstra.cpp)
add_executable(5_a_star                   a_star.cpp)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT 1_breadth_first)
