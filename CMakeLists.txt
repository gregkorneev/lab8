cmake_minimum_required(VERSION 3.10)

project(lab8_forest_fire_pathfinding CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(app
    main.cpp
    grid.cpp
    pathfinding.cpp
)
