set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(SlamSceneViewer_DIR /home/csl/CppWorks/artwork/slam-tricks/thirdparty/slam-scene-viewer-build)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

find_package(SlamSceneViewer)
find_package(Sophus)
find_package(Ceres)
find_package(OpenCV)
find_package(G2O)
find_package(CSparse REQUIRED)

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CERES_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_g2o ${CMAKE_CURRENT_SOURCE_DIR}/src/test_g2o.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/sim_data.cpp)

target_link_libraries(
        ${PROJECT_NAME}_g2o PRIVATE
        SlamSceneViewer::SlamSceneViewer
        Sophus::Sophus
        ${OpenCV_LIBRARIES}
        ${CERES_LIBRARIES}
        ${G2O_LIBRARIES}
)

add_executable(${PROJECT_NAME}_ceres ${CMAKE_CURRENT_SOURCE_DIR}/src/test_ceres.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/sim_data.cpp)

target_link_libraries(
        ${PROJECT_NAME}_ceres PRIVATE
        SlamSceneViewer::SlamSceneViewer
        Sophus::Sophus
        ${OpenCV_LIBRARIES}
        ${CERES_LIBRARIES}
)