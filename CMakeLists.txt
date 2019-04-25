cmake_minimum_required(VERSION 3.1)
cmake_policy(SET CMP0048 OLD)
project(ConstraintGeometry)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

file(GLOB_RECURSE HEADER_FILES
    "src/*.h"
    "src/*.inl"
)

file(GLOB_RECURSE SOURCE_FILES
    "src/*.cpp"
)

file(GLOB_RECURSE SCENES_FILES
    "scenes/*.scn"
    "scenes/*.xml"
)

file(GLOB_RECURSE IGNORED_FILES
    "src/ignored/*.h"
    "src/ignored/*.inl"
    "src/ignored/*.cpp"
)

if(COMMAND setupGitInfo)
    setupGitInfo()
endif()

set_source_files_properties(${IGNORED_FILES} PROPERTIES HEADER_FILE_ONLY TRUE)
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES} ${SCENES_FILES} ${README_FILES} )
set(${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src")

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${${PROJECT_NAME}_INCLUDE_DIRS}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DPLUGIN_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\"")
target_link_libraries(${PROJECT_NAME} SofaCore CollisionAlgorithm)

## Install rules for the library and headers; CMake package configurations files
sofa_generate_package(NAME ${PROJECT_NAME} VERSION 0.1 TARGETS ${PROJECT_NAME} INCLUDE_INSTALL_DIR ${${PROJECT_NAME}_INCLUDE_DIRS})

