set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(OpenCV)

include_directories(./include ${OpenCV_INCLUDE_DIRS})

add_executable(${CMAKE_PROJECT_NAME} ./main.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE ${OpenCV_LIBS})

