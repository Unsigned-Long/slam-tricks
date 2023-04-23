set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Eigen3 REQUIRED)
find_package(SlamSceneViewer REQUIRED)
find_package(Sophus REQUIRED)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_FILES)

add_executable(${PROJECT_NAME}_prog ${SRC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_include_directories(
        ${PROJECT_NAME}_prog PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${EIGEN_INCLUDE_DIRS}
)

target_link_libraries(
        ${PROJECT_NAME}_prog PRIVATE
        SlamSceneViewer::SlamSceneViewer
        Sophus::Sophus
)