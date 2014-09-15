cmake_minimum_required(VERSION 2.6)

project (DeltaMush) 

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib )

SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

find_package(Maya REQUIRED)

include_directories(${MAYA_INCLUDE_DIR})

set(sources src/plugin.cpp
					src/deformer_node.cpp src/deformer_node.hpp
					src/bind_cmd.cpp src/bind_cmd.hpp)

add_library(DeltaMush SHARED ${sources})

target_link_libraries(DeltaMush ${MAYA_LIBRARIES})
maya_set_plugin_properties(DeltaMush)