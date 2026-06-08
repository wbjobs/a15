cmake_minimum_required(VERSION 3.20)
project(VoxelGI_ECS VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4 /permissive- /Zc:__cplusplus)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(/O2 /GL)
        add_link_options(/LTCG)
    endif()
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(-O3)
        add_link_options(-O3)
    endif()
endif()

option(VOXELGI_BUILD_EXAMPLES "Build example applications" ON)
option(VOXELGI_USE_OPENGL "Use OpenGL for rendering" ON)

include_directories(${CMAKE_SOURCE_DIR}/include)

file(GLOB_RECURSE VOXELGI_CORE_SOURCES
    ${CMAKE_SOURCE_DIR}/src/core/*.cpp
    ${CMAKE_SOURCE_DIR}/src/ecs/*.cpp
    ${CMAKE_SOURCE_DIR}/src/voxel/*.cpp
    ${CMAKE_SOURCE_DIR}/src/gi/*.cpp
    ${CMAKE_SOURCE_DIR}/src/export/*.cpp
    ${CMAKE_SOURCE_DIR}/src/render/*.cpp
)

add_library(voxel_ecs STATIC ${VOXELGI_CORE_SOURCES})
target_include_directories(voxel_ecs PUBLIC ${CMAKE_SOURCE_DIR}/include)

if(VOXELGI_USE_OPENGL)
    find_package(OpenGL REQUIRED)
    target_compile_definitions(voxel_ecs PUBLIC VOXELGI_USE_OPENGL)
    target_link_libraries(voxel_ecs PUBLIC OpenGL::GL)
endif()

if(VOXELGI_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

enable_testing()
add_subdirectory(tests)
