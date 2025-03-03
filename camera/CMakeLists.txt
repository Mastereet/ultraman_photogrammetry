set(FOLDER_NAME camera)

PHOTOGRAMMETRY_ADD_LIBRARY(
    NAME photogrammetry_camera
    SOURCES

    HEADERS
        camera_model.hpp
        pinhole_model.hpp
        camera_parametres.hpp
        distortion_model.hpp
    PUBLIC_LINK_LIBRARIES
        Eigen3::Eigen
    PRIVATE_LINK_LIBRARIES
)

PHOTOGRAMMETRY_ADD_TEST(
    NAME pinhole_model_test
    SOURCES
        pinhole_model_test.cc
    HEADERS
        pinhole_model.hpp
    PUBLIC_LINK_LIBRARIES
        Eigen3::Eigen
    PRIVATE_LINK_LIBRARIES
        photogrammetry_camera
        photogrammetry_core
)
