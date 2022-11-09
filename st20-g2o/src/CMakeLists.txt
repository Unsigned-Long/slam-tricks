set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(SlamSceneViewer_DIR /home/csl/CppWorks/artwork/slam-tricks/thirdparty/slam-scene-viewer-build)

find_package(SlamSceneViewer)
find_package(Sophus)
find_package(Ceres)
find_package(OpenCV)

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CERES_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_g2o ${CMAKE_CURRENT_SOURCE_DIR}/src/test_g2o.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/sim_data.cpp)

target_link_libraries(
        ${PROJECT_NAME}_g2o PRIVATE
        SlamSceneViewer::SlamSceneViewer
        Sophus::Sophus
        ${CERES_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_ceres ${CMAKE_CURRENT_SOURCE_DIR}/src/test_ceres.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/sim_data.cpp)

target_link_libraries(
        ${PROJECT_NAME}_ceres PRIVATE
        SlamSceneViewer::SlamSceneViewer
        Sophus::Sophus
        ${CERES_LIBRARIES}
        ${OpenCV_LIBRARIES}
)