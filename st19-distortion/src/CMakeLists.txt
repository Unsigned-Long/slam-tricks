set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(OpenCV REQUIRED)

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_rolling_shut ${CMAKE_CURRENT_SOURCE_DIR}/src/rolling_shut.cpp)

target_link_libraries(
        ${PROJECT_NAME}_rolling_shut PUBLIC
        ${OpenCV_LIBRARIES}
        pthread
)

add_executable(${PROJECT_NAME}_lidar ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar.cpp)

target_link_libraries(
        ${PROJECT_NAME}_lidar PUBLIC
        ${OpenCV_LIBRARIES}
)