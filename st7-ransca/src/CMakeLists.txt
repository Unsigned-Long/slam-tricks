set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

include_directories(./include)

add_executable(${CMAKE_PROJECT_NAME} ./main.cpp)