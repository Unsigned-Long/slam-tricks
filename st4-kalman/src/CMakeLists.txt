set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(CMAKE_DISABLE_FIND_PACKAGE_fmt ON)

find_package(Sophus)

find_package(fmt)

find_package(Eigen3)

find_package(PCL)

include_directories(
    ./include
    ${SOPHUS_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

aux_source_directory(./src SRC_LIST)

add_executable(${CMAKE_PROJECT_NAME} ./main.cpp ${SRC_LIST})

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE ${PCL_LIBRARIES} Sophus::Sophus)