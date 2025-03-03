set(FOLDER_NAME core)

PHOTOGRAMMETRY_ADD_LIBRARY(
    NAME photogrammetry_core
    SOURCES
        
    HEADERS
        eigen_types.hpp
    PUBLIC_LINK_LIBRARIES
        Eigen3::Eigen
)

