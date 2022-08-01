set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)

add_definitions(${PCL_DEFINITIONS})

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_FILES)

add_library(cbdetector SHARED ${SRC_FILES})

target_include_directories(cbdetector PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(cbdetector PUBLIC
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(${CMAKE_PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

add_executable(${CMAKE_PROJECT_NAME}_clib ${CMAKE_CURRENT_SOURCE_DIR}/clib.cpp)

add_executable(${CMAKE_PROJECT_NAME}_funny ${CMAKE_CURRENT_SOURCE_DIR}/funny.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE cbdetector)

target_link_libraries(${CMAKE_PROJECT_NAME}_clib PRIVATE cbdetector)

target_link_libraries(${CMAKE_PROJECT_NAME}_funny PRIVATE cbdetector)