set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(PCL REQUIRED)
find_package(Sophus)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_FILES)

add_executable(${PROJECT_NAME}_prog ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp ${SRC_FILES})

target_include_directories(
        ${PROJECT_NAME}_prog
        PRIVATE
        ${PCL_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
        ${PROJECT_NAME}_prog
        PRIVATE
        ${PCL_LIBRARIES}
        Sophus::Sophus
)