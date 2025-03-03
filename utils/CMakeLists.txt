set(FOLDER_NAME utils)

PHOTOGRAMMETRY_ADD_LIBRARY(
    NAME photogrammetry_utils
    SOURCES
        logger.cc 
    HEADERS
        logger.hpp
    PUBLIC_LINK_LIBRARIES
        Eigen3::Eigen
    PRIVATE_LINK_LIBRARIES
        photogrammetry_core
)

PHOTOGRAMMETRY_ADD_TEST(
    NAME know_enum_test
    SOURCES 
        know_enum_test.cc
    HEADERS
        know_enum.hpp
    PUBLIC_LINK_LIBRARIES
        Eigen3::Eigen
    PRIVATE_LINK_LIBRARIES
        photogrammetry_core
)
