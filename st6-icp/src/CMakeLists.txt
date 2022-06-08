set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Sophus)

find_package(Eigen3)

include_directories(include ${SOPHUS_UBCLIDE_DIRS} ${EIGEN_INCLUDE_DIRS})

# add_executable(${CMAKE_PROJECT_NAME}_binding ./binding.cpp)

# target_link_libraries(${CMAKE_PROJECT_NAME}_binding PRIVATE Sophus::Sophus)

add_executable(${CMAKE_PROJECT_NAME}_nobinding ./nobinding.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME}_nobinding PRIVATE Sophus::Sophus)