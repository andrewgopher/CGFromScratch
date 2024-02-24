cmake_minimum_required(VERSION 3.5.0)

project(CGFromScratch)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

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
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(SDL2 REQUIRED)

include_directories(${SDL2_INCLUDE_DIRS})
include_directories(src/)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
set(CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/cmake/")
find_package(FindPNG++ REQUIRED)
include_directories(${PNG++_INCLUDE_DIR})


add_library(geometry STATIC src/geometry.cpp)
add_library(open_obj STATIC src/open_obj.cpp)
target_link_libraries(open_obj geometry)

add_executable(cg_from_scratch src/main.cpp)
add_executable(test_geometry src/test_geometry.cpp)
add_executable(test_open_obj src/test_open_obj.cpp)
target_link_libraries(cg_from_scratch geometry)
target_link_libraries(cg_from_scratch open_obj)
target_link_libraries(test_geometry geometry)
target_link_libraries(test_open_obj open_obj)
target_link_libraries(cg_from_scratch ${SDL2_LIBRARIES} Threads::Threads)
target_link_libraries(cg_from_scratch png)