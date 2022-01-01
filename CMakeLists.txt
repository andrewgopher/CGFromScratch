cmake_minimum_required(VERSION 3.0.0)

project(CGFromScratch)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH  ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "cmake/")

option(FORCE_COLORED_OUTPUT "Always produce ANSI-colored output (GNU/Clang only)." ON)
if (${FORCE_COLORED_OUTPUT})
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
       add_compile_options(-fdiagnostics-color=always)
    elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
       add_compile_options(-fcolor-diagnostics)
    endif()
endif()


find_package(SDL2 REQUIRED)

include_directories(${SDL2_INCLUDE_DIRS})
include_directories(src/)

add_library(geometry SHARED src/geometry.cpp)
add_executable(cg_from_scratch src/main.cpp)
add_executable(test_geometry src/test_geometry.cpp)
target_link_libraries(cg_from_scratch geometry)
target_link_libraries(test_geometry geometry)
target_link_libraries(cg_from_scratch ${SDL2_LIBRARIES})
