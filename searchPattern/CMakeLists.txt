cmake_minimum_required(VERSION 3.10)

project(searchPattern)

set(CMAKE_CXX_STANDARD 17)

find_package(GDAL REQUIRED)
include_directories(src ${GDAL_INCLUDE_DIRS})

include_directories(src)
set(SOURCES
    src/main.cpp
    src/searchPattern.cpp
    src/GeoLoader.cpp
    )

add_executable(searchPattern ${SOURCES})

target_link_libraries(searchPattern ${GDAL_LIBRARIES})