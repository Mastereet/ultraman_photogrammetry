# if PHOTOGRAMMETRY_FIND_QUIETLY is true, set PHOTOGRAMMETRY_FIND_TYPE to QUIET
# Only the error message that the library cannot be found will be output, 
# and the search process will not be output.
# if PHOTOGRAMMETRY_FIND_QUIETLY is false, set PHOTOGRAMMETRY_FIND_TYPE to REQUIRED
# If the library cannot be found, the build will fail.
if(PHOTOGRAMMETRY_FIND_QUIETLY)
    set(PHOTOGRAMMETRY_FIND_TYPE QUIET)
else()
    set(PHOTOGRAMMETRY_FIND_TYPE REQUIRED)
endif()

find_package(OpenCV ${PHOTOGRAMMETRY_FIND_TYPE})
find_package(Ceres ${PHOTOGRAMMETRY_FIND_TYPE})
find_package(Eigen3 ${PHOTOGRAMMETRY_FIND_TYPE})

find_package(OpenMP ${PHOTOGRAMMETRY_FIND_TYPE})
if(OPENMP_ENABLED AND OPENMP_FOUND)
    message(STATUS "Enabling OpenMP support")
    add_definitions("-DPHOTOGRAMMETRY_OPENMP_ENABLED")
else()
    message(STATUS "Disabling OpenMP support")
endif()

if(TESTS_ENABLED)
    find_package(GTest ${PHOTOGRAMMETRY_FIND_TYPE})
endif()
