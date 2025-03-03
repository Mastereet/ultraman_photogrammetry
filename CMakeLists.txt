cmake_minimum_required(VERSION 3.10)

################################################################################
# Options
################################################################################
option(OPENMP_ENABLED "Whether to enable OpenMP parallelization support" ON)
option(TESTS_ENABLED "Whether to enable tests" ON)

if(TESTS_ENABLED)
    enable_testing()
endif()

project(PHOTOGRAMMETRY LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable most common warnings
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -pedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")

################################################################################
# Include CMake Modules
################################################################################
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Check for compiler flags
include(CheckCXXCompilerFlag)

# Include helper macros and commands, and allow the included file to override
# the CMake policies in this file
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/CMakeHelper.cmake NO_POLICY_SCOPE)

################################################################################
# Dependency configuration
################################################################################

set(PHOTOGRAMMETRY_FIND_QUIETLY FALSE)
include(cmake/FindDependencies.cmake)

################################################################################
# Compiler specific configuration
################################################################################

# Build type
# if not specified, use Release
if(CMAKE_BUILD_TYPE)
    message(STATUS "Build type specified as ${CMAKE_BUILD_TYPE}")
else()
    message(STATUS "Build type not specified, using Release")
    set(CMAKE_BUILD_TYPE Release)
    set(IS_DEBUG OFF)
endif()

# OpenMP support
if(OPENMP_ENABLED)
    check_cxx_compiler_flag("-fopenmp" COMPILER_HAS_OPENMP)
    if(COMPILER_HAS_OPENMP)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
    endif()
endif()

# Hide incorrect warnings for uninitialized Eigen variables under GCC.
if(IS_GNU)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-maybe-uninitialized")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-maybe-uninitialized")
endif()


add_subdirectory(core)
add_subdirectory(image)
add_subdirectory(camera)