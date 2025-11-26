cmake_minimum_required(VERSION 3.10)

project(lab8_forest_fire_pathfinding CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(lab8
    src/main.cpp
    src/grid.cpp
    src/pathfinding.cpp
)

# говорить компилятору, где лежат заголовки
target_include_directories(lab8 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
