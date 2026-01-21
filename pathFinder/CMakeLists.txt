cmake_minimum_required(VERSION 3.10)

project(Pathfinder)

set(CMAKE_CXX_STANDARD 17)

find_package(GDAL REQUIRED)
include_directories(src ${GDAL_INCLUDE_DIRS})

include_directories(src)
set(SOURCES
    src/main.cpp
    src/Astar.cpp
    src/GeoLoader.cpp
    )

add_executable(pathFinder ${SOURCES})

target_link_libraries(pathFinder ${GDAL_LIBRARIES})