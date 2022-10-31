cmake_minimum_required(VERSION 3.10)

project(use_hello VERSION 1.0)


set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_BUILD_TYPE "Release")

set(HelloWithPCL_DIR ${CMAKE_SOURCE_DIR}/../library/cmake-build-debug)

find_package(HelloWithPCL)

add_executable(${PROJECT_NAME}_prog ${CMAKE_SOURCE_DIR}/main.cpp)

target_link_libraries(
        ${PROJECT_NAME}_prog PUBLIC
        HelloWithPCL::HelloWithPCL
)