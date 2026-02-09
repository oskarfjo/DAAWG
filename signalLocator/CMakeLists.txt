cmake_minimum_required(VERSION 3.10)

project(signalLocator)

set(CMAKE_CXX_STANDARD 17)

include_directories(src)
set(SOURCES
    src/main.cpp
    )

add_executable(signalLocator ${SOURCES})