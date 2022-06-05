cmake_minimum_required(VERSION 3.10)

project(optical-flow)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE "Debug")

add_subdirectory(${CMAKE_SOURCE_DIR}/src)