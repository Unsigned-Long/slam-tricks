set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

set(CMAKE_DISABLE_FIND_PACKAGE_fmt ON)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(Sophus REQUIRED)

find_package(PCL REQUIRED)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_FILES)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${SOPHUS_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_library(${CMAKE_PROJECT_NAME} SHARED ${SRC_FILES})

target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC
  ${OpenCV_LIBS}
  Sophus::Sophus
  ${PCL_LIBRARIES}
)

add_executable(${CMAKE_PROJECT_NAME}_prog ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME}_prog PRIVATE ${CMAKE_PROJECT_NAME})