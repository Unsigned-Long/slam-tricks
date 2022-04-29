set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(OpenCV)
find_package(Eigen3)

include_directories(./include ${OpenCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/../st0-core/include)

add_executable(${CMAKE_PROJECT_NAME} ./main.cpp)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${EIGEN_INCLUDE_DIRS})

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE ${OpenCV_LIBS})