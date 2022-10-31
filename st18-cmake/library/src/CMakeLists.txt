set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(PCL REQUIRED)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_FILES)

add_library(
        ${LIBRARY_NAME} SHARED
        ${SRC_FILES}
)

target_include_directories(
        ${LIBRARY_NAME} PUBLIC
        ${PCL_INCLUDE_DIRS}
        # for interface
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/include>
)

target_link_libraries(
        ${LIBRARY_NAME} PUBLIC
        ${PCL_LIBRARIES}
        pthread
)