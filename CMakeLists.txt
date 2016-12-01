cmake_minimum_required(VERSION 2.8.12)
project(ConstraintGeometryPlugin)

find_package(SofaFramework REQUIRED)
find_package(SofaBase REQUIRED)
find_package(SofaSimulation REQUIRED)
find_package(SofaGeneral REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

set(HEADER_FILES      
     src/initConstraintGeometryPlugin.h
     src/ConstraintGeometry.h
     src/EdgeInterpolation.h
     src/EdgeLinearInterpolation.h
     src/EdgeLinearInterpolation.inl
     src/TriangleInterpolation.h
     src/TriangleLinearInterpolation.h
     src/TriangleLinearInterpolation.inl
     src/FullTriangleLinearInterpolation.h
     src/FullTriangleLinearInterpolation.inl
     src/TetrahedronCutting.h
     src/TetrahedronCutting.inl
     src/PlaneCutting.h
     src/PlaneCutting.inl
)

set(SOURCE_FILES
     src/initConstraintGeometryPlugin.cpp
     src/EdgeLinearInterpolation.cpp
     src/TriangleLinearInterpolation.cpp
     src/FullTriangleLinearInterpolation.cpp
     src/TetrahedronCutting.cpp
     src/PlaneCutting.cpp
)

file(GLOB_RECURSE SCENES_FILES
    "scenes/*.scn"
    "scenes/*.xml"
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES} ${SCENES_FILES} ${README_FILES} )
#set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

set(CONSTRAINTGEOMETRYPLUGIN_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CONSTRAINTGEOMETRYPLUGIN_INCLUDE_DIRS}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DPLUGIN_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\"")
target_link_libraries(${PROJECT_NAME} SofaCore SofaComponentBase SofaSimulationTree SofaUserInteraction SofaConstraint)

## Install rules for the library and headers; CMake package configurations files
sofa_create_package(${PROJECT_NAME} 0.1 ${PROJECT_NAME} ${PROJECT_NAME})

install(DIRECTORY sofa
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h" PATTERN "*hpp" PATTERN "*.inl"
)

